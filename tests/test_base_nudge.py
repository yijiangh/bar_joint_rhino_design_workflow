"""ResolveCollision's base nudge (`core.base_nudge`): the frame, the search that raises the smallest distance, the
branch tests, and the whole chain on the planning fixture (ssik through its sidecar; skipped without it)."""

from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest

from core import base_nudge
from core.base_nudge import Trial, branch_change, frame_at, raise_clearance, turn_centre

# ---------------------------------------------------------------------------
# frame_at / turn_centre
# ---------------------------------------------------------------------------

SEED = np.array([[0.0, -1.0, 0.0, 1000.0],
                 [1.0, 0.0, 0.0, 2000.0],
                 [0.0, 0.0, 1.0, 50.0],
                 [0.0, 0.0, 0.0, 1.0]])  # facing world +Y, standing at (1000, 2000, 50) mm


def test_no_nudge_is_the_seed_and_a_shift_follows_the_seeds_own_axes():
    assert np.allclose(frame_at(SEED, (0, 0, 0), (300.0, 0.0), 0.5), SEED)
    shifted = frame_at(SEED, (0.010, 0.0, 0.0), (300.0, 0.0), 0.5)  # 10 mm along the seed's x = world +Y
    assert np.allclose(shifted[:3, 3], [1000.0, 2010.0, 50.0])
    assert np.allclose(shifted[:3, :3], SEED[:3, :3])


def test_a_turn_keeps_the_centre_where_it_was_and_the_ground_plane():
    centre = (300.0, 100.0)
    turned = frame_at(SEED, (0.0, 0.0, 0.5 * np.radians(10.0)), centre, 0.5)  # 10 degrees
    point = np.array([*centre, 0.0, 1.0])
    assert np.allclose(turned @ np.linalg.solve(turned, SEED @ point), SEED @ point)
    assert np.allclose((np.linalg.inv(SEED) @ turned @ point)[:3], (*centre, 0.0))
    assert np.allclose(turned[:3, 2], SEED[:3, 2])
    assert np.degrees(np.arccos(np.clip(turned[:3, 0] @ SEED[:3, 0], -1, 1))) == pytest.approx(10.0)


def test_turn_centre_is_the_targets_centroid_in_the_seeds_plane():
    local = np.array([[400.0, 300.0, 900.0], [400.0, -300.0, 900.0], [600.0, 300.0, 700.0], [600.0, -300.0, 700.0]])
    world = (SEED @ np.c_[local, np.ones(4)].T).T[:, :3]
    centre, rho = turn_centre(SEED, world)
    assert np.allclose(centre, (500.0, 0.0))
    assert rho == pytest.approx(np.hypot(0.1, 0.3))


# ---------------------------------------------------------------------------
# raise_clearance on a point robot between walls
# ---------------------------------------------------------------------------

def _walls(left_count=3, a=0.005, b=0.016, refuse_beyond=None, why="home"):
    """A robot at x = u[0] between a left wall at -a (counted `left_count` times) and a right wall at +b."""

    def evaluate(u):
        x = float(u[0])
        if refuse_beyond is not None and x > refuse_beyond:
            return Trial(False, why)
        found = {(f"M{i}", "robot", "left"): x + a for i in range(1, left_count + 1)}
        found[("M1", "robot", "right")] = b - x
        return Trial(True, distances=found)

    return evaluate


def test_the_minimum_is_raised_where_least_squares_would_stop_short():
    """A corridor 21 mm wide for a 10 mm clearance: x in [5, 6] mm clears every pair. Least squares on the shortfalls
    from an aim of 20 mm (the first plan: max(0, 20 mm - d)), three pairs pushing from the left and one from the
    right, settles at x = 10.25 mm, where the right pair has only 5.75 mm; raising the smallest distance ends inside
    the strip."""
    outcome = raise_clearance(_walls(), need=0.010, aim=0.020, band=0.1)
    assert outcome.status == "clear"
    assert 0.005 - 1e-9 <= outcome.u[0] <= 0.006 + 1e-9
    assert outcome.trial.lowest(0.1) >= 0.010
    assert abs(outcome.u[1]) < 1e-9 and abs(outcome.u[2]) < 1e-9  # the directions no pair cares about stay put


def test_already_clear_moves_nothing_and_a_refused_seed_does_not_start():
    clear = raise_clearance(_walls(a=0.02, b=0.02), need=0.010, aim=0.020, band=0.1)
    assert clear.status == "clear" and clear.steps == 0 and not clear.u.any()
    start = raise_clearance(lambda u: Trial(False, "home"), need=0.01, aim=0.02, band=0.1)
    assert (start.status, start.why) == ("start", "home")


def test_a_refused_region_ends_with_its_reason_at_the_best_allowed_base():
    """The way out is past x = 3 mm, where the base is refused: it stops there and says why."""
    outcome = raise_clearance(_walls(refuse_beyond=0.003), need=0.010, aim=0.020, band=0.1)
    assert (outcome.status, outcome.why) == ("refused", "home")
    assert 0.0 < outcome.u[0] <= 0.003
    assert outcome.trial.lowest(0.1) == pytest.approx(outcome.u[0] + 0.005)


def test_an_impossible_corridor_is_a_local_optimum_and_esc_stops():
    """8 mm wide for a 10 mm clearance: the best is the middle, which still lacks 6 mm."""
    narrow = raise_clearance(_walls(a=0.002, b=0.006), need=0.010, aim=0.020, band=0.1)
    assert narrow.status == "optimum"
    assert narrow.u[0] == pytest.approx(0.002, abs=1e-4)
    stopped = raise_clearance(_walls(a=0.002, b=0.006), need=0.010, aim=0.020, band=0.1,
                              stop=lambda: True)
    assert stopped.status == "stopped" and stopped.steps == 0


def test_a_bound_is_a_refusal_like_any_other():
    """The evaluator's bound (ChainProbe's 150 mm / 10 deg) ends the search with its reason, not as an optimum."""
    outcome = raise_clearance(_walls(refuse_beyond=0.003, why="bound"), need=0.010, aim=0.020, band=0.1)
    assert (outcome.status, outcome.why) == ("refused", "bound")
    assert 0.0 < outcome.u[0] <= 0.003


def test_a_model_that_overpromises_is_stalled_not_an_optimum():
    """A pair whose distance its finite difference misjudges (a kink just beyond the probe): every step falls short
    of the prediction, the radius shrinks to nothing, and the stop is "stalled", since the same LP at the largest
    radius still promises a gain."""

    def evaluate(u):
        x = float(u[0])
        # ? Rises with x over the 2 mm probe step, then falls back: a kink between the probe and any step.
        value = -0.005 + (x if x <= 0.002 else 0.004 - x)
        return Trial(True, distances={("M1", "robot", "post"): value})

    outcome = raise_clearance(evaluate, need=0.001, aim=0.002, band=0.1)
    assert outcome.status == "stalled", outcome


# ---------------------------------------------------------------------------
# The room pass: the body's minimum, every pair above a floor
# ---------------------------------------------------------------------------

BODY_PAIR = ("M2", "robots/cindy/base_link", "bars/B9")
ARM_PAIR = ("M2", "robots/cindy/left_ur_arm_forearm_link", "bars/B12")


def _room(body=0.010, arm=0.003):
    """Moving the base x along gives the body x more room from B9 and takes x / 2 from the forearm's gap to B12."""

    def evaluate(u):
        x = float(u[0])
        return Trial(True, distances={BODY_PAIR: body + x, ARM_PAIR: arm - 0.5 * x})

    return evaluate


def _body(key):
    return key[1].endswith("base_link")


def test_the_room_pass_stops_where_an_arm_pair_reaches_the_floor():
    """The body wants 50 mm, but at x = 4 mm the forearm is down to the 1 mm floor: the steps stop
    FLOOR_MARGIN_M above it, at x = 3 mm, and it is an optimum. Without the floor the body gets its room and the
    forearm goes into the bar."""
    outcome = raise_clearance(_room(), need=0.050, aim=0.051, band=0.2, counts=_body, floor=0.001)
    assert outcome.status == "optimum", outcome
    assert outcome.u[0] == pytest.approx(0.003, abs=1e-4)
    assert 0.001 <= outcome.trial.distances[ARM_PAIR] <= 0.001 + 2 * base_nudge.FLOOR_MARGIN_M
    assert outcome.trial.distances[BODY_PAIR] == pytest.approx(0.013, abs=1e-4)
    assert abs(outcome.u[1]) < 1e-9 and abs(outcome.u[2]) < 1e-9
    free = raise_clearance(_room(), need=0.050, aim=0.051, band=0.2, counts=_body)
    assert free.status == "clear" and free.trial.distances[ARM_PAIR] < 0.0


def test_the_room_pass_takes_the_shortest_move_to_the_room_from_where_it_starts():
    """With the forearm far away the body gets its 50 mm (aim 51), moving no further than that, from `start`;
    Esc keeps the start."""
    start = np.array([0.002, 0.0, 0.0])
    outcome = raise_clearance(_room(arm=0.1), need=0.050, aim=0.051, band=0.2, counts=_body, floor=0.001, start=start)
    assert outcome.status == "clear"
    assert 0.040 - 1e-9 <= outcome.u[0] <= 0.041 + 1e-6
    stopped = raise_clearance(_room(), need=0.050, aim=0.051, band=0.2, counts=_body, floor=0.001, start=start,
                              stop=lambda: True)
    assert stopped.status == "stopped" and np.allclose(stopped.u, start)
    under = raise_clearance(_room(arm=0.0005), need=0.050, aim=0.051, band=0.2, counts=_body, floor=0.001)
    assert (under.status, under.why) == ("start", "floor")


# ---------------------------------------------------------------------------
# branch_change
# ---------------------------------------------------------------------------

LOWER, UPPER = [-2 * np.pi] * 6, [2 * np.pi] * 6
SEEDQ = [0.3, -1.2, 1.5, -1.0, 1.2, 0.4]


def test_a_nearby_pose_is_on_the_branch():
    assert branch_change(np.add(SEEDQ, 0.05), SEEDQ, LOWER, UPPER, np.radians(45), 1.0, 1.2) == ""


def test_the_elbow_and_wrist_signs_and_their_singular_bands():
    flipped = list(SEEDQ)
    flipped[2] = -0.2
    assert branch_change(flipped, SEEDQ, LOWER, UPPER, np.radians(45)).startswith("elbow (it would flip")
    near = list(SEEDQ)
    near[4] = np.radians(1.0)
    assert branch_change(near, SEEDQ, LOWER, UPPER, np.radians(45)).startswith("wrist (near a singular")
    # * q5 near pi is singular too: sin q5 goes through 0 there.
    at_pi = list(SEEDQ)
    at_pi[4] = np.pi - np.radians(1.0)
    assert branch_change(at_pi, SEEDQ, LOWER, UPPER, np.radians(180)).startswith("wrist (near a singular")


def test_the_shoulder_by_the_jacobian_the_spread_and_the_limits():
    assert branch_change(SEEDQ, SEEDQ, LOWER, UPPER, np.radians(45), -1.0, 1.0).startswith("shoulder (it would flip")
    assert branch_change(SEEDQ, SEEDQ, LOWER, UPPER, np.radians(45), 0.05, 1.0).startswith("shoulder (near")
    far = list(SEEDQ)
    far[0] += np.radians(60)
    assert branch_change(far, SEEDQ, LOWER, UPPER, np.radians(45)).startswith("configuration")
    assert branch_change(SEEDQ, SEEDQ, [0.5] * 6, UPPER, np.radians(45)).startswith("joints")


def test_every_stop_has_a_message():
    for key in ("optimum", "stalled", "steps", "stopped", "evaluations", "bound", "ground", "home", "reach", "branch",
                "start_home", "start_ground", "start_branch", "fixed", "unreachable", "chain", "floor", "room_optimum",
                "room_steps"):
        assert key in base_nudge.MESSAGES


# ---------------------------------------------------------------------------
# The whole chain on the planning fixture
# ---------------------------------------------------------------------------

from test_process_planning import setup  # noqa: E402,F401  (the fixture)


def _ssik_or_skip():
    from husky_assembly_tamp.keyframe import ssik_client
    try:
        ssik_client.solve("left", np.eye(4) @ np.diag([1, 1, 1, 1]) + np.array([[0, 0, 0, 0.4], [0, 0, 0, 0.1],
                                                                                 [0, 0, 0, 0.3], [0, 0, 0, 0]]))
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"ssik sidecar not available: {exc}")


CRATE = "obstacles/crate"
BOX = "obstacles/box"
FOREARM = "robots/cindy/left_ur_arm_forearm_link"


def _placed(setup, name, at, box_m, target, gap):
    """The fixture's process with a cube `box_m` wide at `at(s)`, s found by secant steps from 0.3 until the
    smallest signed distance (as PyBullet measures it, at assembled) between the cube and `target` is `gap`.

    Args:
        at: s -> the cube's centre (3,), metres.
        target: part -> True for the robot parts that count.
    """
    from scaffolding_assembly_core.geometry import BoxShape, Geometry, Pose
    from scaffolding_assembly_core.process import BodySpec
    from core.process_planning import movement, pose_distances

    _process, store, reach, _, rebuild = setup
    keyframes = dict(approach=reach, assembled=reach, retreat=reach)
    box = BoxShape((box_m, box_m, box_m))

    def build(s):
        return rebuild(bodies={name: BodySpec(name, Pose(tuple(at(s))), Geometry((box,), (box,)))}, **keyframes)

    def distance(process):
        scene = process.scene_at(movement(process, "bars/B1", "J", "J_insert"))
        found = pose_distances(store, scene, "robots/cindy", 1.0, log=lambda *_: None)
        return min(d for (a, b), d in found.items() if name in (a, b) and target(b if a == name else a))

    s0, d0 = 0.3, distance(build(0.3))
    s, d = s0 - (d0 - gap), None
    for _ in range(8):
        d = distance(build(s))
        if abs(d - gap) < 1e-4:
            break
        s, s0, d0 = s - (d - gap) * (s - s0) / (d - d0), s, d
    assert d == pytest.approx(gap, abs=2e-4)
    return build(s)


def _with_crate(setup, overlap: float = 0.003, along: float = 0.5):
    """The fixture's process with a 40 mm crate `overlap` metres into Cindy's left forearm at the reach pose.

    Placed out from the forearm, `along` of the way from the elbow to wrist 1 (the forearm, because the fixture's
    reach pose is its home pose with the elbows bent: anything on the upper arm hits it at home too), square to the
    arm and level, then moved in until the signed distance (as PyBullet measures it) is -overlap.
    """
    from scaffolding_assembly_core.geometry import Pose
    from scaffolding_assembly_core.kinematics import link_pose

    _process, _store, reach, _, rebuild = setup
    urdf = rebuild(approach=reach, assembled=reach, retreat=reach).robots["robots/cindy"].urdf
    start, end = (np.array(link_pose(urdf, Pose(), reach, f"left_ur_arm_{link}").position)
                 for link in ("forearm_link", "wrist_1_link"))
    middle = start + along * (end - start)
    out = np.cross(end - start, [0.0, 0.0, 1.0])
    out /= np.linalg.norm(out)
    if out[:2] @ middle[:2] < 0.0:
        out = -out
    return _placed(setup, CRATE, lambda s: middle + s * out, 0.04, lambda part: part == FOREARM, -overlap)


def _with_box_behind(setup, gap: float = 0.020):
    """The fixture's process with a 100 mm box behind Cindy's chassis, low (centre 0.2 m up), `gap` metres from the
    nearest of her parts: what the room pass pushes the base away from."""
    return _placed(setup, BOX, lambda s: np.array([-0.4 - s, 0.0, 0.2]), 0.1,
                   lambda part: part.startswith("robots/cindy/"), gap)


@contextmanager
def _reach_chain(setup, process):
    """At the seed (the world origin), with the planner lent: the chain through the candidate holding the reach pose.

    Yields:
        SimpleNamespace: planner, movements, scenes, groups (left, right), moving (`BaseGroups`), band (first pass),
        table (`ChainTable`), chain.
    """
    from husky_assembly_tamp.keyframe import dual_arm_ik
    from husky_assembly_tamp.keyframe.ik_keyframe import frame_to_mm4
    from core.process_planning import KEYFRAME_MOVEMENTS, keyframe_movements, movement

    _process, store, reach, _, _rebuild = setup
    mirror, movements = keyframe_movements(process, store, "bars/B1")
    scenes = {role: process.scene_at(movement(process, "bars/B1", *KEYFRAME_MOVEMENTS[role]))
              for role in base_nudge.ROLES}
    seed = np.eye(4)
    with mirror.lend() as planner:
        rcell = planner.client.robot_cell
        groups = dual_arm_ik.resolve_arm_groups(rcell)
        arms = [list(rcell.get_configurable_joint_names(group)) for group in groups]
        targets = movements["M2"].target_ee_frames
        enumerated = {"M2": dual_arm_ik.enumerate_ssik_candidate_pairs(
            planner, movements["M2"].start_state, seed, frame_to_mm4(targets["left"]), frame_to_mm4(targets["right"]),
            groups=groups)}
        moving = base_nudge.base_groups("robots/cindy", rcell.robot_model, arms, movements["M2"].start_state)
        band = base_nudge.band_for(0.002)
        table = base_nudge.chain_table(planner, store, "robots/cindy", movements, scenes, seed, "M2", enumerated,
                                       moving.depends, band, groups)
        # * The candidate holding the reach pose (the bar was built from it).
        wanted = np.array([reach[name] for joints in arms for name in joints])
        gaps = [np.max(np.abs(base_nudge.wrapped(
            np.array([dict(zip(c["state"].robot_configuration.joint_names,
                               c["state"].robot_configuration.joint_values))[n] for j in arms for n in j]) - wanted)))
                for c in enumerated["M2"]["candidates"]]
        index = int(np.argmin(gaps))
        assert gaps[index] < 1e-3
        chain = table.chains[index]
        assert chain is not None
        yield SimpleNamespace(planner=planner, movements=movements, scenes=scenes, groups=groups, moving=moving,
                              band=band, table=table, chain=chain, store=store)


def _probe(at, band):
    from core.base_search import HomeCheck

    return base_nudge.ChainProbe(
        at.planner, at.store, "robots/cindy", at.movements, at.scenes, at.chain, np.eye(4),
        snap=lambda point: (np.asarray(point, dtype=float), np.array([0.0, 0.0, 1.0])),
        home_check=HomeCheck(at.planner, at.movements), depends=at.moving.depends, groups=at.groups, band=band,
        max_mm=150.0, max_deg=10.0, branch_deg=45.0, log=lambda *_: None)


def _solves_on_the_branch(at, outcome, probe, monkeypatch):
    """tamp's chain solver (ssik) at the nudged base solves, every pose on the followed branch."""
    from husky_assembly_tamp.keyframe import config as tamp_config

    monkeypatch.setattr(tamp_config, "IK_BACKEND", "ssik")
    ordered = [(role, at.movements[role]) for role in base_nudge.ROLES]
    solved, failed, elsewhere = base_nudge.final_chain(at.planner, ordered, probe.frame(outcome.u),
                                                       outcome.trial.data.states, check_collision=True,
                                                       verbose_pairs=False, groups=at.groups)
    assert solved is not None, failed
    assert elsewhere == []


def test_resolve_moves_the_base_off_a_crate_and_the_solver_takes_the_branch_there(setup, monkeypatch):
    """A crate 3 mm into the left forearm: the nudge moves the base until every pair the base can change keeps
    1 mm, on the same branch, with the real home check; then tamp's chain solver (ssik) solves there on it."""
    _ssik_or_skip()
    with _reach_chain(setup, _with_crate(setup)) as at:
        chain, band = at.chain, at.band
        assert not at.table.fixed
        # * At the seed the crate is the only thing overlapping along the chain (the forearm, maybe the upper arm): 3 mm at
        #   assembled, deeper at the retreat, where the arms pull back towards it.
        overlaps = {key: round(d, 4) for key, d in chain.distances.items() if d < 0}
        assert all(CRATE in key for key in overlaps), overlaps
        assert overlaps[("M2", FOREARM, CRATE)] == pytest.approx(-0.003, abs=3e-4)
        probe = _probe(at, band)
        here = probe(np.zeros(3))
        assert here.allowed
        assert here.lowest(band) == pytest.approx(chain.lowest(band), abs=2e-4)  # the same poses, two ways
        steps = []
        outcome = raise_clearance(probe, need=0.001, aim=0.002, band=band, max_steps=15,
                                  on_step=lambda n, u, trial: steps.append(trial.lowest(band)))
        assert outcome.status == "clear", outcome
        assert outcome.trial.lowest(band) >= 0.001
        assert steps == sorted(steps)  # every accepted step raised the minimum
        moved_mm, turned_deg = probe.moved(outcome.u)
        assert 0.0 < moved_mm <= 150.0 and abs(turned_deg) <= 10.0
        _solves_on_the_branch(at, outcome, probe, monkeypatch)


def test_the_room_pass_moves_the_body_off_a_box_and_keeps_every_pair(setup, monkeypatch):
    """A box 20 mm behind the chassis, nothing overlapping: the room pass (as ResolveCollision runs it after a
    clear) moves the base until the body keeps 50 mm, every step adding room and every pair keeping 1 mm; then tamp's
    chain solver solves there on the branch."""
    _ssik_or_skip()
    with _reach_chain(setup, _with_box_behind(setup)) as at:
        assert at.chain.lowest(at.band) >= 0.001  # clear at the seed: only the room is missing
        band = base_nudge.band_for(0.051)
        probe = _probe(at, band)

        def body(trial):
            return min((d for key, d in trial.distances.items() if at.moving.body_world(key[1], key[2])),
                       default=band)

        here = probe(np.zeros(3))
        assert body(here) == pytest.approx(0.020, abs=2e-4)
        nearest = min((key for key in here.distances if at.moving.body_world(key[1], key[2])),
                      key=here.distances.get)
        assert BOX in nearest
        steps = []
        outcome = raise_clearance(probe, need=0.050, aim=0.051, band=band, max_steps=15,
                                  counts=lambda key: at.moving.body_world(key[1], key[2]), floor=0.001,
                                  on_step=lambda n, u, trial: steps.append(body(trial)))
        assert outcome.status == "clear", outcome
        assert body(outcome.trial) >= 0.050
        assert outcome.trial.lowest(band) >= 0.001  # the floor
        assert steps == sorted(steps)  # every accepted step added room
        moved_mm, turned_deg = probe.moved(outcome.u)
        assert 25.0 < moved_mm <= 150.0 and abs(turned_deg) <= 10.0
        _solves_on_the_branch(at, outcome, probe, monkeypatch)
