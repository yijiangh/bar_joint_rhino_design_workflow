"""Headless collision-contract tests for the static assembly cell.

Pins the collision behaviour of the refactored cell by placing bodies at
KNOWN colliding / clear poses and asserting the planner's verdict on the
SPECIFIC pair (parsed from the full-report ``CollisionCheckError`` message,
which carries the canonical body / link / tool names).

No Rhino: the cell is assembled directly from procedural geometry. The module
skips itself if PyBullet / the URDF + tool assets are unavailable.

    pytest tests/test_bar_action_collisions.py
"""

from __future__ import annotations

import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (SCRIPTS_DIR, TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _bar_rb(deps, length_m=1.0, radius_m=0.02):
    from core import env_collision

    mesh = env_collision._build_bar_cylinder_mesh(length_m, radius_m)
    return deps["RigidBody"](visual_meshes=[mesh], collision_meshes=[mesh], native_scale=1.0)


def _box_rb(deps, size_m=0.12):
    from compas.geometry import Box

    mesh = deps["Mesh"].from_shape(Box(size_m, size_m, size_m))
    return deps["RigidBody"](visual_meshes=[mesh], collision_meshes=[mesh], native_scale=1.0)


# Canonical names in the synthetic scene:
#   bars  B1 (seq1, built), B2 (seq2, active), B3 (seq3, future)
#   joints for connection J1-2: female on B1 (built), male on B2 (active)
BAR_BUILT = "bar_B1"
BAR_ACTIVE = "bar_B2"
BAR_FUTURE = "bar_B3"
JOINT_BUILT = "joint_J1-2_female"   # female sits on the earlier (built) bar
JOINT_ACTIVE = "joint_J1-2_male"    # male sits on the active bar
OBSTACLE = "obstacle_box"
ALL_RB = [BAR_BUILT, BAR_ACTIVE, BAR_FUTURE, JOINT_BUILT, JOINT_ACTIVE, OBSTACLE]

LEFT_TOOL0 = "left_ur_arm_tool0"
RIGHT_TOOL0 = "right_ur_arm_tool0"


def _build_cell():
    """Boot PyBullet + register the synthetic assembly. Returns (planner, rcell, deps)."""
    from core import robot_cell as rc

    deps = rc.import_compas_stack()
    if not rc.is_pb_running():
        rc.start_pb_client(use_gui=False, verbose=True)
    rcell = rc.get_or_load_robot_cell()
    _client, planner = rc.get_planner()

    rcell.rigid_body_models[BAR_BUILT] = _bar_rb(deps)
    rcell.rigid_body_models[BAR_ACTIVE] = _bar_rb(deps)
    rcell.rigid_body_models[BAR_FUTURE] = _bar_rb(deps)
    rcell.rigid_body_models[JOINT_BUILT] = _box_rb(deps)
    rcell.rigid_body_models[JOINT_ACTIVE] = _box_rb(deps)
    rcell.rigid_body_models[OBSTACLE] = _box_rb(deps, size_m=0.2)

    for tid, tm in rc.build_arm_tool_models().items():
        rcell.tool_models[tid] = tm

    planner.set_robot_cell(rcell)
    return planner, rcell, deps


@pytest.fixture(scope="module")
def cell():
    """(planner, rcell, deps) with a synthetic assembly + arm ToolModels."""
    from core import robot_cell as rc

    try:
        bundle = _build_cell()
    except Exception as exc:  # pragma: no cover
        pytest.skip(f"PyBullet / robot cell / tool assets unavailable: {exc}")
    yield bundle
    try:
        rc.stop_pb_client()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# State / collision helpers
# ---------------------------------------------------------------------------


def _fresh_state(planner, rcell):
    """Base state with tools attached, all rigid bodies hidden, robot posed."""
    from core import robot_cell as rc

    state = rc.base_assembly_cell_state()
    state.robot_configuration = rcell.zero_full_configuration()
    for rbs in state.rigid_body_states.values():
        rbs.is_hidden = True
    planner.set_robot_cell_state(state)  # pose robot so link FK is available
    return state


def _tool_ids(rcell):
    from core import robot_cell as rc

    ids = rc.arm_tool_ids()
    return ids["left"], ids["right"]


def _link_frame(planner, link_name):
    client = planner.client
    return client._get_link_frame(client.robot_link_puids[link_name], client.robot_puid)


def _find_link(planner, substr):
    for name in planner.client.robot_link_puids:
        if substr in name:
            return name
    raise AssertionError(f"no robot link contains {substr!r}")


def _compose(link_frame, attach_frame):
    from compas.geometry import Frame, Transformation

    t = Transformation.from_frame(link_frame) * Transformation.from_frame(attach_frame)
    return Frame.from_transformation(t)


def _far_frame():
    from compas.geometry import Frame

    return Frame([0.0, 0.0, 1000.0], [1, 0, 0], [0, 1, 0])


def _place(state, name, frame):
    rbs = state.rigid_body_states[name]
    rbs.is_hidden = False
    rbs.attached_to_link = None
    rbs.attached_to_tool = None
    rbs.attachment_frame = None
    rbs.frame = frame


def _attach(state, name, link, attach_frame=None):
    from compas.geometry import Frame

    rbs = state.rigid_body_states[name]
    rbs.is_hidden = False
    rbs.attached_to_link = link
    rbs.attached_to_tool = None
    rbs.attachment_frame = attach_frame or Frame.worldXY()
    rbs.frame = None


def _report(planner, state):
    """Full collision report string ('' when collision-free)."""
    from compas_fab.backends import CollisionCheckError

    planner.set_robot_cell_state(state)
    try:
        planner.check_collision(state, options={"full_report": True, "verbose": False})
        return ""
    except CollisionCheckError as exc:
        return str(exc)


def _has_pair(report, a, b):
    return any(a in ln and b in ln and "COLLISION" in ln for ln in report.splitlines())


# ---------------------------------------------------------------------------
# Detection tests -- robot link / tool vs built bars + joints (CC.3 / CC.5)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("body", [BAR_BUILT, JOINT_BUILT])
def test_robot_link_vs_built(cell, body):
    """CC.3: a built bar/joint overlapping a robot link is flagged; clear when away."""
    planner, rcell, _ = cell
    link = _find_link(planner, "forearm")

    state = _fresh_state(planner, rcell)
    lf = _link_frame(planner, link)
    _place(state, body, lf)
    assert _has_pair(_report(planner, state), link, body)

    _place(state, body, _far_frame())
    assert not _has_pair(_report(planner, state), link, body)


@pytest.mark.parametrize("body", [BAR_BUILT, JOINT_BUILT])
def test_tool_vs_built(cell, body):
    """CC.5: a built bar/joint overlapping the left tool is flagged; clear when away."""
    planner, rcell, _ = cell
    left_tool, _ = _tool_ids(rcell)

    state = _fresh_state(planner, rcell)
    tf = _link_frame(planner, LEFT_TOOL0)
    _place(state, body, tf)
    assert _has_pair(_report(planner, state), left_tool, body)

    _place(state, body, _far_frame())
    assert not _has_pair(_report(planner, state), left_tool, body)


# ---------------------------------------------------------------------------
# Detection tests -- grasped bar tube / grasped attached joint vs built (CC.4)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("grasped", [BAR_ACTIVE, JOINT_ACTIVE])
@pytest.mark.parametrize("built", [BAR_BUILT, JOINT_BUILT])
def test_grasped_vs_built(cell, grasped, built):
    """CC.4: an attached grasped tube/joint overlapping a built body is flagged."""
    planner, rcell, _ = cell
    from compas.geometry import Frame

    af = Frame([0.0, 0.0, 0.5], [1, 0, 0], [0, 1, 0])  # 0.5 m out along the flange
    state = _fresh_state(planner, rcell)
    _attach(state, grasped, LEFT_TOOL0, af)
    grasped_world = _compose(_link_frame(planner, LEFT_TOOL0), af)

    _place(state, built, grasped_world)
    assert _has_pair(_report(planner, state), grasped, built)

    _place(state, built, _far_frame())
    assert not _has_pair(_report(planner, state), grasped, built)


def test_grasped_bar_vs_robot_link(cell):
    """CC.3: a grasped bar attached at the flange overlaps the wrist link; clear when offset away."""
    planner, rcell, _ = cell
    from compas.geometry import Frame

    wrist = _find_link(planner, "wrist_3")
    state = _fresh_state(planner, rcell)

    _attach(state, BAR_ACTIVE, LEFT_TOOL0, Frame.worldXY())  # bar base at the flange
    assert _has_pair(_report(planner, state), wrist, BAR_ACTIVE)

    _attach(state, BAR_ACTIVE, LEFT_TOOL0, Frame([0, 0, 1000.0], [1, 0, 0], [0, 1, 0]))
    assert not _has_pair(_report(planner, state), wrist, BAR_ACTIVE)


# ---------------------------------------------------------------------------
# Detection tests -- environment obstacle (CC.3 / CC.5 / CC.4)
# ---------------------------------------------------------------------------


def test_robot_link_vs_environment(cell):
    planner, rcell, _ = cell
    link = _find_link(planner, "forearm")
    state = _fresh_state(planner, rcell)

    _place(state, OBSTACLE, _link_frame(planner, link))
    assert _has_pair(_report(planner, state), link, OBSTACLE)

    _place(state, OBSTACLE, _far_frame())
    assert not _has_pair(_report(planner, state), link, OBSTACLE)


def test_tool_vs_environment(cell):
    planner, rcell, _ = cell
    left_tool, _ = _tool_ids(rcell)
    state = _fresh_state(planner, rcell)

    _place(state, OBSTACLE, _link_frame(planner, LEFT_TOOL0))
    assert _has_pair(_report(planner, state), left_tool, OBSTACLE)

    _place(state, OBSTACLE, _far_frame())
    assert not _has_pair(_report(planner, state), left_tool, OBSTACLE)


@pytest.mark.parametrize("grasped", [BAR_ACTIVE, JOINT_ACTIVE])
def test_grasped_vs_environment(cell, grasped):
    planner, rcell, _ = cell
    from compas.geometry import Frame

    af = Frame([0.0, 0.0, 0.5], [1, 0, 0], [0, 1, 0])
    state = _fresh_state(planner, rcell)
    _attach(state, grasped, LEFT_TOOL0, af)
    grasped_world = _compose(_link_frame(planner, LEFT_TOOL0), af)

    _place(state, OBSTACLE, grasped_world)
    assert _has_pair(_report(planner, state), grasped, OBSTACLE)

    _place(state, OBSTACLE, _far_frame())
    assert not _has_pair(_report(planner, state), grasped, OBSTACLE)


# ---------------------------------------------------------------------------
# build_assembly_movements building blocks: M2 attaches the active bodies to
# tool0 (bar gripped), M3 detaches them to their static world pose (bar released).
# ---------------------------------------------------------------------------


def test_m2_attaches_active_m3_detaches(cell):
    """M2 grips the active bar + male onto the left tool0; M3 releases them static.

    This is the core per-movement distinction the IK keyframe tool relies on:
    the approach pose (M2 start) carries the bar with the arm, the assembled pose
    (M3 start) leaves it placed in the world.
    """
    planner, rcell, _ = cell
    import numpy as np
    from core import bar_action

    env_geom = {
        BAR_ACTIVE: {"frame_world_mm": np.eye(4), "parent_bar_id": "B2"},
        JOINT_ACTIVE: {"frame_world_mm": np.eye(4), "parent_bar_id": "B2"},
    }
    active_keys = {BAR_ACTIVE, JOINT_ACTIVE}
    arm_to_male = {"J1-2": "left"}
    tool0 = np.eye(4)

    # M2: both active bodies attach to the left arm's tool0.
    state = _fresh_state(planner, rcell)
    bar_action._set_active_attachments(
        state, active_keys, env_geom, arm_to_male, tool0, tool0, bar_arm_side="left",
    )
    assert state.rigid_body_states[BAR_ACTIVE].attached_to_link == LEFT_TOOL0
    assert state.rigid_body_states[JOINT_ACTIVE].attached_to_link == LEFT_TOOL0

    # M3: the same bodies detach to a static world pose (no link).
    state = _fresh_state(planner, rcell)
    bar_action._detach_active_to_assembled_world(state, active_keys, env_geom)
    assert state.rigid_body_states[BAR_ACTIVE].attached_to_link is None
    assert state.rigid_body_states[JOINT_ACTIVE].attached_to_link is None


# ---------------------------------------------------------------------------
# build_full_assembly_state: future bars hidden, built/active visible static
# ---------------------------------------------------------------------------


def test_build_full_assembly_state_hides_future(cell):
    """future (seq>active) -> is_hidden; built/active -> visible static; env -> visible."""
    planner, rcell, _ = cell
    from core import robot_cell as rc
    from core import ik_collision_setup as ics

    base = rc.base_assembly_cell_state()
    base.robot_configuration = rcell.zero_full_configuration()
    import numpy as np

    eye = np.eye(4)
    collision_bodies = {
        BAR_BUILT: {"frame_world_mm": eye, "kind": "bar", "parent_bar_id": "B1"},
        BAR_ACTIVE: {"frame_world_mm": eye, "kind": "bar", "parent_bar_id": "B2"},
        BAR_FUTURE: {"frame_world_mm": eye, "kind": "bar", "parent_bar_id": "B3"},
        JOINT_BUILT: {"frame_world_mm": eye, "kind": "joint", "parent_bar_id": "B1"},
        JOINT_ACTIVE: {"frame_world_mm": eye, "kind": "joint", "parent_bar_id": "B2"},
        OBSTACLE: {"frame_world_mm": eye, "kind": "environment"},
    }
    seq_map = {"B1": (None, 1), "B2": (None, 2), "B3": (None, 3)}

    state = ics.build_full_assembly_state(base, collision_bodies, seq_map, "B2")
    rbs = state.rigid_body_states
    assert rbs[BAR_FUTURE].is_hidden is True
    assert rbs[JOINT_BUILT].is_hidden is False
    assert rbs[BAR_BUILT].is_hidden is False
    assert rbs[BAR_ACTIVE].is_hidden is False
    assert rbs[OBSTACLE].is_hidden is False
    # key-set equals the cell's collision bodies (assert_cell_state_match contract)
    assert set(rbs.keys()) == set(collision_bodies.keys())


# ---------------------------------------------------------------------------
# Per-movement allowed-touch policy (bar_action._apply_movement_touch_policy)
# ---------------------------------------------------------------------------


def _touch_inputs(rcell):
    """(env_geom, arm_to_male, bar_key, tool_ids) for the J1-2 connection."""
    from core import robot_cell as rc

    env_geom = {
        BAR_ACTIVE: {"parent_bar_id": "B2"},
        JOINT_ACTIVE: {"parent_bar_id": "B2"},      # grasped male on the active bar
        JOINT_BUILT: {"parent_bar_id": "B1"},       # mate female on the built bar
    }
    return env_geom, {"J1-2": "left"}, BAR_ACTIVE, rc.arm_tool_ids()


def test_m1_male_tool_allowed(cell):
    """M1: the grasped male whitelists its own arm tool (gripper holds it)."""
    planner, rcell, _ = cell
    from compas.geometry import Frame
    from core import bar_action

    left_tool, _ = _tool_ids(rcell)
    env_geom, arm_to_male, bar_key, tool_ids = _touch_inputs(rcell)

    state = _fresh_state(planner, rcell)
    _attach(state, JOINT_ACTIVE, LEFT_TOOL0, Frame.worldXY())  # male at flange -> overlaps the tool
    assert _has_pair(_report(planner, state), left_tool, JOINT_ACTIVE)  # baseline: flagged

    bar_action._apply_movement_touch_policy(
        state, "M1", {BAR_ACTIVE, JOINT_ACTIVE}, env_geom, arm_to_male, bar_key, tool_ids,
    )
    assert not _has_pair(_report(planner, state), left_tool, JOINT_ACTIVE)


def test_m1_mate_not_yet_allowed_but_m2_is(cell):
    """M1 keeps male<->built-female flagged (halves apart); M2 whitelists the mate."""
    planner, rcell, _ = cell
    from compas.geometry import Frame
    from core import bar_action

    env_geom, arm_to_male, bar_key, tool_ids = _touch_inputs(rcell)
    af = Frame([0.0, 0.0, 0.5], [1, 0, 0], [0, 1, 0])

    state = _fresh_state(planner, rcell)
    _attach(state, JOINT_ACTIVE, LEFT_TOOL0, af)
    male_world = _compose(_link_frame(planner, LEFT_TOOL0), af)
    _place(state, JOINT_BUILT, male_world)  # built female overlapping the grasped male

    bar_action._apply_movement_touch_policy(
        state, "M1", {BAR_ACTIVE, JOINT_ACTIVE}, env_geom, arm_to_male, bar_key, tool_ids,
    )
    assert _has_pair(_report(planner, state), JOINT_ACTIVE, JOINT_BUILT)  # M1: not yet allowed

    bar_action._apply_movement_touch_policy(
        state, "M2", {BAR_ACTIVE, JOINT_ACTIVE}, env_geom, arm_to_male, bar_key, tool_ids,
    )
    assert not _has_pair(_report(planner, state), JOINT_ACTIVE, JOINT_BUILT)  # M2: mate allowed


def test_m2_bar_vs_built_female_flagged(cell):
    """The bar tube is NEVER whitelisted against a built female, even in M2."""
    planner, rcell, _ = cell
    from compas.geometry import Frame
    from core import bar_action

    env_geom, arm_to_male, bar_key, tool_ids = _touch_inputs(rcell)
    af = Frame([0.0, 0.0, 0.5], [1, 0, 0], [0, 1, 0])

    state = _fresh_state(planner, rcell)
    _attach(state, BAR_ACTIVE, LEFT_TOOL0, af)
    bar_world = _compose(_link_frame(planner, LEFT_TOOL0), af)
    _place(state, JOINT_BUILT, bar_world)

    bar_action._apply_movement_touch_policy(
        state, "M2", {BAR_ACTIVE, JOINT_ACTIVE}, env_geom, arm_to_male, bar_key, tool_ids,
    )
    assert _has_pair(_report(planner, state), BAR_ACTIVE, JOINT_BUILT)


def test_m3_detached_tool_allowed_and_static_skip(cell):
    """M3: tool<->released-male allowed; two detached (static) bodies auto-skip."""
    planner, rcell, _ = cell
    from core import bar_action

    left_tool, _ = _tool_ids(rcell)
    env_geom, arm_to_male, bar_key, tool_ids = _touch_inputs(rcell)

    state = _fresh_state(planner, rcell)
    tf = _link_frame(planner, LEFT_TOOL0)
    _place(state, JOINT_ACTIVE, tf)    # released (static) male sitting at the tool
    _place(state, JOINT_BUILT, tf)     # another released (static) body overlapping it

    bar_action._apply_movement_touch_policy(
        state, "M3", {BAR_ACTIVE, JOINT_ACTIVE}, env_geom, arm_to_male, bar_key, tool_ids,
    )
    report = _report(planner, state)
    assert not _has_pair(report, left_tool, JOINT_ACTIVE)       # tool<->male allowed
    assert not _has_pair(report, JOINT_ACTIVE, JOINT_BUILT)     # static<->static auto-skipped


def test_bar_tool_whitelisted_m1_m3_not_m4(cell):
    """The grasped tube whitelists BOTH gripper tools in M1-M3, none in M4.

    Covers the tool<->tube coarse-mesh overlap (a pipeline artefact): allowed
    while gripped, cleared once released.
    """
    planner, rcell, _ = cell
    from core import bar_action

    left_tool, right_tool = _tool_ids(rcell)
    env_geom, arm_to_male, bar_key, tool_ids = _touch_inputs(rcell)

    for movement, expect_tools in (("M1", True), ("M2", True), ("M3", True), ("M4", False)):
        state = _fresh_state(planner, rcell)
        bar_action._apply_movement_touch_policy(
            state, movement, {BAR_ACTIVE, JOINT_ACTIVE}, env_geom, arm_to_male, bar_key, tool_ids,
        )
        touch = set(state.rigid_body_states[BAR_ACTIVE].touch_bodies or [])
        assert touch == ({left_tool, right_tool} if expect_tools else set()), (movement, touch)


# ---------------------------------------------------------------------------
# Standalone runner (avoids the pytest/cryptography-PyO3 conflict in the Rhino
# site-env; mirrors tests/test_robot_cell_support_smoke.py). Run:
#     python tests/test_bar_action_collisions.py
# ---------------------------------------------------------------------------


def _main() -> int:
    import traceback

    from core import robot_cell as rc

    cases = []
    for body in (BAR_BUILT, JOINT_BUILT):
        cases.append((f"robot_link_vs_built[{body}]", lambda c, b=body: test_robot_link_vs_built(c, b)))
        cases.append((f"tool_vs_built[{body}]", lambda c, b=body: test_tool_vs_built(c, b)))
    for grasped in (BAR_ACTIVE, JOINT_ACTIVE):
        for built in (BAR_BUILT, JOINT_BUILT):
            cases.append(
                (f"grasped_vs_built[{grasped},{built}]",
                 lambda c, g=grasped, b=built: test_grasped_vs_built(c, g, b))
            )
        cases.append((f"grasped_vs_environment[{grasped}]",
                      lambda c, g=grasped: test_grasped_vs_environment(c, g)))
    cases += [
        ("grasped_bar_vs_robot_link", test_grasped_bar_vs_robot_link),
        ("robot_link_vs_environment", test_robot_link_vs_environment),
        ("tool_vs_environment", test_tool_vs_environment),
        ("m2_attaches_active_m3_detaches", test_m2_attaches_active_m3_detaches),
        ("build_full_assembly_state_hides_future", test_build_full_assembly_state_hides_future),
        ("m1_male_tool_allowed", test_m1_male_tool_allowed),
        ("m1_mate_not_yet_allowed_but_m2_is", test_m1_mate_not_yet_allowed_but_m2_is),
        ("m2_bar_vs_built_female_flagged", test_m2_bar_vs_built_female_flagged),
        ("m3_detached_tool_allowed_and_static_skip", test_m3_detached_tool_allowed_and_static_skip),
        ("bar_tool_whitelisted_m1_m3_not_m4", test_bar_tool_whitelisted_m1_m3_not_m4),
    ]

    try:
        bundle = _build_cell()
    except Exception:
        print("[X] could not build cell:")
        traceback.print_exc()
        return 1

    passed = failed = 0
    try:
        for name, fn in cases:
            try:
                fn(bundle)
                print(f"[OK] {name}")
                passed += 1
            except Exception:
                print(f"[FAIL] {name}")
                traceback.print_exc()
                failed += 1
    finally:
        try:
            rc.stop_pb_client()
        except Exception:
            pass

    print(f"\n{passed} passed, {failed} failed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(_main())
