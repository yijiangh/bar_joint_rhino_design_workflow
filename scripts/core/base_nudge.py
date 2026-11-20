"""Nudge Cindy's base until one IK branch of a bar's keyframe chain is clear: ResolveCollision in RSIKKeyframe.

When the chain fails at a base, InspectCandidates shows the IK branch pairs of the failing movement. ResolveCollision
takes one of them and moves the base a little -- x, y and a turn about the ground normal -- until the chain on that
branch (approach, assembled, retreat) keeps every pair the base can change at least `config.BASE_NUDGE_CLEARANCE_M`
apart, as PyBullet measures it. tamp's chain solver then decides at the new base (`final_chain`): nothing here
accepts a pose by itself.

The goal is the smallest distance D(u) over those pairs. `raise_clearance` raises it by a trust-region sequential
linear program: forward-difference gradients of every pair near the minimum, one `scipy.optimize.linprog` per step
for the step that raises the linearised minimum most, and a ratio test on the gain it really made (docs/note.md §37).

- `frame_at`, `turn_centre`, `raise_clearance`, `Trial`, `Outcome`, `branch_change`, `MESSAGES`: numpy and scipy.
- `base_groups`, `chain_table`, `ChainProbe`, `final_chain`: the lent planner (compas_fab, PyBullet) and tamp.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linprog

from core import base_search  # ? read its constants when used: a stale Rhino copy may lack a new name

#: Finite-difference step of the three unknowns, metres.
STEP_M = 0.002
#: Trust radius per unknown, metres: the first, the largest, and the one below which the search gives up.
RADIUS_M, RADIUS_MAX_M, RADIUS_MIN_M = 0.010, 0.020, 0.0001
#: No step is worth taking when the predicted gain is below this slope times the trust radius (metres per metre)...
SLOPE_TOL = 0.01
#: ... or below what PyBullet's distances can tell apart, metres: EPA (overlapping) and GJK (apart), times about 3.
NOISE_OVERLAP_M, NOISE_APART_M = 3.0e-4, 5.0e-5
#: A step is kept if it makes at least this share of the gain it predicted.
ACCEPT_RATIO = 0.1
#: What a metre of move costs in the step's LP, in metres of clearance: a direction that raises the minimum by less
#: than this per metre (below SLOPE_TOL, so never worth a step) stays put instead of moving at random.
MOVE_COST = 1.0e-3
#: How far a point of the robot moves per metre of `u`, at most (shift plus the turn at the chassis' far end):
#: the watch band (`band_for`) must hold every pair that can become the nearest within one step.
LIPSCHITZ = 3.0
#: Where |sin| of the elbow (q3) or of wrist 2 (q5) is below sin 2 degrees, the arm is near a singular pose.
SINGULAR_SIN = float(np.sin(np.radians(2.0)))
#: Where the arm Jacobian's determinant falls below this share of the seed's, it is near a singular pose.
SINGULAR_DET_SHARE = 0.1
#: Following a branch: Newton's method on PyBullet's forward kinematics and Jacobian, from the seed's joints. Its
#: tolerance (metres and radians), its iterations, and the largest joint step per iteration (radians). Measured
#: 2026-11-20: compas_fab's iterative IK needs ~1000 iterations (200-400 ms per pose) for 1e-5 m, and at 1e-4 m its
#: error would swamp a 2 mm finite difference; Newton gets from 15 mm to 1e-7 in 3 iterations, where PyBullet's own
#: link poses stop being more precise. 1e-6 keeps the finite differences' noise near 0.1 %.
NEWTON_TOL, NEWTON_ITERATIONS, NEWTON_MAX_STEP_RAD = 1.0e-6, 25, 0.3
#: Evaluations (trials and probes) before the search gives up, whatever else happens.
MAX_EVALUATIONS = 250
#: Pose names of the keyframe roles, for messages.
POSES = {"M1": "approach (M1)", "M2": "assembled (M2)", "M3": "retreat (M3)"}
ROLES = ("M1", "M2", "M3")

#: The one wording of every way ResolveCollision can end, by `Outcome.status` or, for a refusal, `Outcome.why`.
MESSAGES = {
    "optimum": "Moving the base further no longer reduces the overlap.",
    "steps": "Stopped after {steps} steps (BASE_NUDGE_MAX_STEPS); the overlap was still going down. Raise "
             "BASE_NUDGE_MAX_STEPS in config.py to let it go further.",
    "stopped": "Stopped by Esc after {steps} steps.",
    "evaluations": "Stopped after {evaluations} trial bases (MAX_EVALUATIONS in core/base_nudge.py) without "
                   "clearing it.",
    "bound": "Reached the limit of {max_mm:.0f} mm / {max_deg:.0f} deg from your base (config BASE_NUDGE_MAX_MM / "
             "BASE_NUDGE_MAX_DEG).",
    "ground": "The base would leave the walkable ground.",
    "home": "With the arms at home, the robot would hit the scene at that base.",
    "branch": "Following this branch further would change {detail}, so it would no longer be the candidate you "
              "picked.",
    "reach": "Moving further, {detail} could no longer reach its target on this branch.",
    "stalled": "The distances stopped following their slopes here (two pairs taking turns as the nearest, or "
               "noise), so it stopped.",
    "start_home": "At your base the robot, arms at home, already hits the scene, so ResolveCollision cannot start. "
                  "Use RetryNewBase.",
    "start_ground": "Your base is more than 1 mm off the walkable ground, so ResolveCollision cannot start.",
    "start_branch": "At your base {detail} cannot be followed, so ResolveCollision cannot start.",
    "fixed": "{pairs} overlap with the tools at their targets: no IK branch or base can fix that. Move the tool or "
             "the bar.",
    "unreachable": "No approach branch leads to candidate {index}/{count} (the solver would switch arm "
                   "configuration on the way); try another candidate.",
    "chain": "The branch is clear here, but the IK chain still fails at {pose}; the pairs are on the command line.",
}


# --- --- --- --- --- THE BASE FRAME --- --- --- --- ---

def turn_centre(seed_mm, targets_mm) -> Tuple[np.ndarray, float]:
    """Where the base turns about, and the scale of a turn: the targets' centroid in the seed's plane.

    Turning about the targets instead of the base origin keeps a turn from looking like a sideways shift to the
    optimiser. Scaled by the targets' RMS distance from the centroid, one unit of turn moves them about as far as
    one unit of shift.

    Args:
        seed_mm: The seed base frame (4, 4), mm.
        targets_mm: The flange targets, (N, 3) world points in mm.

    Returns:
        tuple: (centre (2,) in mm in the seed's own x, y; the RMS radius in metres, at least 0.1).
    """
    seed = np.asarray(seed_mm, dtype=float)
    points = np.asarray(targets_mm, dtype=float).reshape(-1, 3)
    local = (np.linalg.inv(seed) @ np.c_[points, np.ones(len(points))].T).T[:, :2]
    centre = local.mean(axis=0)
    rho = float(np.sqrt(np.mean(np.sum((local - centre) ** 2, axis=1)))) / 1000.0
    return centre, max(rho, 0.1)


def frame_at(seed_mm, u, centre_mm, rho_m: float) -> np.ndarray:
    """The base frame nudged by `u` from the seed: the one place the nudge is turned into a frame.

    Args:
        seed_mm: The seed base frame (4, 4), mm; its z axis is the ground normal.
        u: (dx, dy, rho * d_alpha) in metres: the shift along the seed's own x and y, and the turn about the vertical
            through `centre_mm`, as the distance it moves a point `rho_m` away.
        centre_mm: The turn's centre, (2,) mm in the seed's own x, y (`turn_centre`).
        rho_m: The turn's scale, metres.

    Returns:
        np.ndarray: (4, 4) base frame, mm, in the seed's plane.
    """
    dx, dy, arc = (float(value) for value in u)
    angle = arc / float(rho_m)
    cos, sin = np.cos(angle), np.sin(angle)
    centre = np.asarray(centre_mm, dtype=float)[:2]
    turn = np.eye(4)
    turn[:2, :2] = [[cos, -sin], [sin, cos]]
    turn[:2, 3] = centre - turn[:2, :2] @ centre
    shift = np.eye(4)
    shift[:2, 3] = (1000.0 * dx, 1000.0 * dy)
    return np.asarray(seed_mm, dtype=float) @ shift @ turn


# --- --- --- --- --- RAISING THE SMALLEST DISTANCE --- --- --- --- ---

@dataclass
class Trial:
    """What `evaluate(u)` found at one nudge.

    Attributes:
        allowed: The base may be used: inside the bounds, on the ground, home clear, every arm on its branch.
        why: Why not, a key of `MESSAGES` ("bound", "ground", "home", "reach", "branch").
        detail: Words for the message, e.g. "the left arm's elbow at assembled (M2)".
        distances: (role, part, part) -> metres, signed (below 0: overlap), of every pair the base can change that
            is within the band, e.g. {("M1", "robots/cindy/left_ur_arm_forearm_link", "bars/B12"): -0.0031}.
        data: The evaluator's, e.g. the base frame and the solved states.
    """

    allowed: bool
    why: str = ""
    detail: str = ""
    distances: Dict[Tuple[str, str, str], float] = field(default_factory=dict)
    data: object = None

    def lowest(self, band: float) -> float:
        """The smallest distance, metres; `band` when no pair is near."""
        return min(self.distances.values(), default=band)

    def nearest(self) -> Optional[Tuple[str, str, str]]:
        """The (role, part, part) with the smallest distance, or None."""
        return nearest_key(self.distances)


@dataclass
class Outcome:
    """How `raise_clearance` ended.

    Attributes:
        status: "clear" (every pair has the clearance), "optimum" (no move within reach raises the minimum),
            "stalled" (the model promised more than the distances gave), "refused" (refused bases in the way; see
            `why`), "steps" (step limit), "evaluations" (MAX_EVALUATIONS), "stopped" (Esc), "start" (the seed
            itself is not allowed; see `why`).
        u: The nudge reached, (3,) metres: the best allowed one.
        trial: The evaluation at `u`.
        steps: Accepted steps.
        why, detail: The refusal behind a "refused" or "start" stop, as in `Trial`.
    """

    status: str
    u: np.ndarray
    trial: Trial
    steps: int
    why: str = ""
    detail: str = ""


def band_for(aim: float) -> float:
    """The watch band: list every pair up to this distance, metres, so none can become the nearest unseen in a step."""
    return float(aim + LIPSCHITZ * (RADIUS_MAX_M + STEP_M))


def nearest_key(distances: Mapping) -> Optional[tuple]:
    """The key of the smallest distance in a distances dict (`Trial.distances`, `Chain.distances`), or None."""
    return min(distances, key=distances.get) if distances else None


def overlap_parts(distances: Mapping, role: str) -> List[str]:
    """The parts that overlap something (distance below 0) at one pose: what the preview paints red."""
    return [part for key, distance in distances.items() if key[0] == role and distance < 0.0 for part in key[1:]]


def _best_step(d, slopes, free, radius: float, aim: float, low: float) -> Tuple[np.ndarray, float]:
    """The LP of one step: the move within `radius` (per unknown) that raises the linearised minimum most.

    Over x = (delta+, delta-, t), delta = delta+ - delta-: minimise -t + MOVE_COST |delta|_1 subject to the rows
    t - g_i . delta <= d_i and t <= aim. The small cost of moving keeps a direction no pair cares about at 0.

    Returns:
        tuple: (delta (3,), the predicted gain t - low); (0, 0) if the LP fails.
    """
    reach = [(0.0, radius if open_ else 0.0) for open_ in free]
    rows = np.hstack([-slopes, slopes, np.ones((len(d), 1))])
    solved = linprog(c=[MOVE_COST] * 6 + [-1.0], A_ub=rows, b_ub=d, bounds=reach + reach + [(None, aim)],
                     method="highs")
    if not solved.success:
        return np.zeros(3), 0.0
    delta, top = np.asarray(solved.x[:3]) - np.asarray(solved.x[3:6]), float(solved.x[6])
    if top >= aim - 1e-12:
        # ? Capped: any step reaching `aim` is optimal; keep the shortest one along it. Rows that the step brings
        #   closer reach `aim` anyway (t <= d_i + c_i at s = 1); each other row needs s >= its share.
        change = slopes @ delta
        shares = [(aim - di) / ci for di, ci in zip(d, change) if ci > 0.0]
        delta = delta * min(1.0, max([0.0, *shares]))
    return delta, top - low


def raise_clearance(evaluate: Callable[[np.ndarray], Trial], *, need: float, aim: float, band: float,
                    max_steps: int = 25, stop: Callable[[], bool] = lambda: False,
                    on_step: Optional[Callable[[int, np.ndarray, Trial], None]] = None) -> Outcome:
    """Move `u` from 0 until every pair is at least `need` apart, by raising the smallest distance.

    One iteration: the gradient of every pair listed at `u` or at a probe, by forward differences of `STEP_M`
    (backward where the forward probe is refused; a direction refused both ways stays fixed), kept while `u` does
    not change. Then the LP of `_best_step`,

        maximise t - MOVE_COST |delta|_1  subject to  d_i + g_i . delta >= t  (every pair),  t <= aim,
                 |delta_j| <= radius

    and the trial `u + delta`, kept if its smallest distance rose by at least `ACCEPT_RATIO` of the predicted gain.
    The radius (from `RADIUS_M`, at most `RADIUS_MAX_M`) doubles after a good step (ratio above 0.75) and halves after
    a poor or refused one. Any evaluated point that is allowed and clear ends the search at once, a probe included.
    The bounds on the base are `evaluate`'s: a trial beyond them is refused ("bound") like any other.

    It stops without clearing when the predicted gain is below max(`SLOPE_TOL` * radius, the noise floor), or when
    the radius shrinks below `RADIUS_MIN_M`. Then: "refused" if a direction or the last trial was refused (with that
    reason); else the same LP again at `RADIUS_MAX_M` (same gradients, no evaluation): if no move that far raises the
    linearised minimum by more than max(`SLOPE_TOL` * `RADIUS_MAX_M`, noise), "optimum" -- a value bound: the model
    has no better point within reach, as at a first-order stationary point of D. Otherwise the model promised more
    than the distances gave, "stalled" (a kink where the nearest pair switches, or noise).

    Args:
        evaluate: u -> `Trial`. It must depend on u alone: it is called at probes and trials in any order.
        need: The clearance to reach, metres.
        aim: The clearance a step aims for, metres (above `need`, so that a step lands inside, not on the edge).
        band: `evaluate` lists no pair farther than this, metres (`band_for(aim)`); a pair not listed counts as
            `band`.
        max_steps: Accepted steps before "steps".
        stop: Asked before each iteration; True ends with "stopped" (Esc).
        on_step: Called after each accepted step with (steps, u, trial).

    Returns:
        Outcome: How it ended, at the best allowed `u` (each accepted step raises the minimum).
    """
    step, radius = STEP_M, RADIUS_M
    calls = [0]

    def counted(at):
        calls[0] += 1
        return evaluate(at)

    u = np.zeros(3)
    here = counted(u)
    if not here.allowed:
        return Outcome("start", u, here, 0, here.why, here.detail)
    low = here.lowest(band)
    if low >= need:
        return Outcome("clear", u, here, 0)
    steps, model, refusal = 0, None, ("", "")

    def ended():
        """A stop without a clear point (see the docstring)."""
        d, slopes, free, probe_refusal = model
        why, detail = probe_refusal if probe_refusal[0] else refusal
        if why:
            return Outcome("refused", u, here, steps, why, detail)
        _delta, widest = _best_step(d, slopes, free, RADIUS_MAX_M, aim, low)
        noise = NOISE_OVERLAP_M if low < -0.002 else NOISE_APART_M
        return Outcome("optimum" if widest < max(SLOPE_TOL * RADIUS_MAX_M, noise) else "stalled", u, here, steps)

    while True:
        if stop():
            return Outcome("stopped", u, here, steps)
        if calls[0] >= MAX_EVALUATIONS:
            return Outcome("evaluations", u, here, steps)
        if model is None:
            # * Gradients by forward differences; backward where forward is refused; fixed where both are.
            free, probes, probe_refusal = np.ones(3, dtype=bool), [], ("", "")
            for j in range(3):
                for sign in (1.0, -1.0):
                    at = u.copy()
                    at[j] += sign * step
                    probe = counted(at)
                    if probe.allowed:
                        if probe.lowest(band) >= need:
                            return Outcome("clear", at, probe, steps + 1)
                        probes.append((j, sign, probe))
                        break
                    if sign < 0.0:
                        probe_refusal = (probe.why, probe.detail)  # ? refused both ways: this direction is fixed
                else:
                    free[j] = False
            keys = sorted(set(here.distances).union(*(probe.distances for _, _, probe in probes)))
            d = np.array([here.distances.get(key, band) for key in keys])
            slopes = np.zeros((len(keys), 3))
            for j, sign, probe in probes:
                slopes[:, j] = [sign * (probe.distances.get(key, band) - here.distances.get(key, band)) / step
                                for key in keys]
            model = (d, slopes, free, probe_refusal)
        d, slopes, free, _probe_refusal = model
        delta, gain = _best_step(d, slopes, free, radius, aim, low)
        noise = NOISE_OVERLAP_M if low < -0.002 else NOISE_APART_M  # ? EPA only once the hulls' cores overlap
        if gain < max(SLOPE_TOL * radius, noise):
            return ended()
        trial = counted(u + delta)
        if trial.allowed:
            if trial.lowest(band) >= need:
                return Outcome("clear", u + delta, trial, steps + 1)
            ratio = (trial.lowest(band) - low) / gain
        else:
            ratio, refusal = -np.inf, (trial.why, trial.detail)
        if ratio > ACCEPT_RATIO:
            u, here, low = u + delta, trial, trial.lowest(band)
            steps, model, refusal = steps + 1, None, ("", "")
            if on_step is not None:
                on_step(steps, u, here)
            if steps >= max_steps:
                return Outcome("steps", u, here, steps)
            if ratio > 0.75:
                radius = min(2.0 * radius, RADIUS_MAX_M)
        if ratio < 0.25:
            radius *= 0.5
            if radius < RADIUS_MIN_M:
                return ended()


# --- --- --- --- --- THE BRANCH --- --- --- --- ---

def wrapped(angles):
    """Angles wrapped to [-pi, pi)."""
    return (np.asarray(angles, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


def branch_change(values: Sequence[float], seed: Sequence[float], lower: Sequence[float], upper: Sequence[float],
                  max_rad: float, det: Optional[float] = None, det_seed: Optional[float] = None) -> str:
    """Why one arm's joints are no longer on the seed's IK branch, or "" if they are.

    A UR-type arm (not cuspidal) has 8 branches, told apart by three signs: of sin q3 (elbow), of sin q5 (wrist 2)
    and, with those two, of the arm Jacobian's determinant (shoulder). The branches meet continuously where a sign
    goes through 0 (q3 or q5 at 0 or pi, the wrist centre on the shoulder axis), so a warm IK can slide from one onto
    the other without a jump. So each sign is tested, with a band around 0 where the arm is near singular (which
    also keeps the finite differences away from where they blow up). On the calibrated arm the singular set is
    det J = 0, not exactly q3 = 0 or q5 = 0; the bands cover that small offset. A joint farther than `max_rad` from
    the seed is refused too.

    Args:
        values, seed: The arm's six joints in UR order (shoulder pan ... wrist 3), radians.
        lower, upper: The six joint limits.
        max_rad: The farthest any joint may be from the seed, radians, wrapped.
        det, det_seed: The arm Jacobian's determinant here and at the seed (`arm_jacobian_det`); None skips it.

    Returns:
        str: e.g. "elbow (it would flip)", "wrist (near a singular pose)", "shoulder (near a singular pose)",
        "configuration (it would switch)", "joints (one would leave its limits)"; "" when on the branch.
    """
    q, q0 = np.asarray(values, dtype=float), np.asarray(seed, dtype=float)
    for index, name in ((2, "elbow"), (4, "wrist")):
        here, there = float(np.sin(q[index])), float(np.sin(q0[index]))
        if np.sign(here) != np.sign(there):
            return f"{name} (it would flip)"
        if abs(here) < SINGULAR_SIN:
            return f"{name} (near a singular pose)"
    if det is not None and det_seed is not None and det_seed != 0.0:
        if np.sign(det) != np.sign(det_seed):
            return "shoulder (it would flip)"
        if abs(det) < SINGULAR_DET_SHARE * abs(det_seed):
            return "shoulder (near a singular pose)"
    if float(np.max(np.abs(wrapped(q - q0)))) > max_rad:
        return "configuration (it would switch)"
    if np.any(q < np.asarray(lower, dtype=float)) or np.any(q > np.asarray(upper, dtype=float)):
        return "joints (one would leave its limits)"
    return ""


def _arm_pose_and_jacobian(client, joint_names: Sequence[str], end_link: str, values: Sequence[float]):
    """Pose the arm in PyBullet; return the end link's world frame (position, 3x3 rotation) and its 6x6 Jacobian.

    The Jacobian's linear half is taken at the end link's own origin (PyBullet's `localPosition` is relative to the
    link's centre of mass), its angular half in world axes; columns in `joint_names` order.
    """
    import pybullet  # noqa: PLC0415

    body, cid = client.robot_puid, client.client_id
    for name, value in zip(joint_names, values):
        pybullet.resetJointState(body, client.robot_joint_puids[name], float(value), physicsClientId=cid)
    link = client.robot_link_puids[end_link]
    state = pybullet.getLinkState(body, link, computeForwardKinematics=True, physicsClientId=cid)
    inertial = np.array(pybullet.getMatrixFromQuaternion(state[3])).reshape(3, 3)
    origin_in_com = (-inertial.T @ np.asarray(state[2], dtype=float)).tolist()
    movable = [j for j in range(pybullet.getNumJoints(body, physicsClientId=cid))
               if pybullet.getJointInfo(body, j, physicsClientId=cid)[2] != pybullet.JOINT_FIXED]
    positions = [pybullet.getJointState(body, j, physicsClientId=cid)[0] for j in movable]
    zeros = [0.0] * len(movable)
    linear, angular = pybullet.calculateJacobian(body, link, origin_in_com, positions, zeros, zeros,
                                                 physicsClientId=cid)
    # ? compas_fab loads the robot with a free base; PyBullet fixes it only because the root link has no mass. Had
    #   it a mass, the Jacobian would start with 6 base columns: skip whatever comes before the joints.
    offset = len(linear[0]) - len(movable)
    columns = [offset + movable.index(client.robot_joint_puids[name]) for name in joint_names]
    jacobian = np.vstack([np.asarray(linear)[:, columns], np.asarray(angular)[:, columns]])
    rotation = np.array(pybullet.getMatrixFromQuaternion(state[5])).reshape(3, 3)
    return np.asarray(state[4], dtype=float), rotation, jacobian


def arm_jacobian_det(client, joint_names: Sequence[str], end_link: str, values: Sequence[float]) -> float:
    """The determinant of one arm's 6x6 Jacobian (linear over angular) at these joints, from PyBullet.

    It does not depend on where the base stands (a rotation of both halves has determinant 1) nor on the point the
    linear half is taken at (that adds a multiple of the angular rows), so the world may be posed anywhere.

    Args:
        client: The planner's compas_fab PyBullet client.
        joint_names: The arm's six joints, in order.
        end_link: The link the Jacobian is taken at, e.g. "left_ur_arm_tool0".
        values: The six joint values.
    """
    return float(np.linalg.det(_arm_pose_and_jacobian(client, joint_names, end_link, values)[2]))


def follow_arm(client, joint_names: Sequence[str], end_link: str, seed: Sequence[float],
               target) -> Optional[Tuple[list, float]]:
    """One arm's joints that put `end_link` on `target`, by Newton's method from `seed`, with the arm Jacobian's
    determinant there (`arm_jacobian_det`); None if it does not converge.

    The robot's base must already stand where it should in PyBullet (set the planner's state first). Each iteration
    solves J dq = e, e the 6-D error (position, rotation vector), with the step capped at `NEWTON_MAX_STEP_RAD`. From a
    seed on the same branch this converges quadratically to the solution on that branch.

    Args:
        client: The planner's compas_fab PyBullet client.
        joint_names: The arm's six joints, in order.
        end_link: The link to place, e.g. "left_ur_arm_tool0".
        seed: The six joints to start from, radians.
        target: The link's world frame, a compas `Frame` in metres.
    """
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    point = np.asarray(target.point, dtype=float)
    x, y = np.asarray(target.xaxis, dtype=float), np.asarray(target.yaxis, dtype=float)
    wanted = np.column_stack([x, y, np.cross(x, y)])
    q = np.asarray(seed, dtype=float).copy()
    for _ in range(NEWTON_ITERATIONS):
        position, rotation, jacobian = _arm_pose_and_jacobian(client, joint_names, end_link, q)
        error = np.concatenate([point - position, Rotation.from_matrix(wanted @ rotation.T).as_rotvec()])
        if float(np.max(np.abs(error))) < NEWTON_TOL:
            return q.tolist(), float(np.linalg.det(jacobian))
        try:
            dq = np.linalg.solve(jacobian, error)
        except np.linalg.LinAlgError:
            return None
        largest = float(np.max(np.abs(dq)))
        q = q + (dq * (NEWTON_MAX_STEP_RAD / largest) if largest > NEWTON_MAX_STEP_RAD else dq)
    return None


def base_groups(robot_id: str, model, arms: Sequence[Sequence[str]], state) -> Callable[[str, str], bool]:
    """A test of whether a pair's distance changes when the base moves, the arms following their targets.

    The tools sit at their flange targets in the world, and with them the links after each arm's last joint (wrist
    3, flange, tool0) and the carried bar. So a pair whose two parts are both in this world group (with the scene and
    other robots) does not change, up to the IK tolerance; nor does one whose parts both move rigidly with the base
    (the links no arm joint moves, and what hangs on them). Every pair with a moving arm link (shoulder to wrist 2),
    and every pair of a body part with a world part, does.

    Args:
        robot_id: E.g. "robots/cindy"; links are named "<robot_id>/<link>".
        model: The robot's compas_robots model.
        arms: Each arm's six joints, base to flange.
        state: A compas_fab state of the mirror: which bodies hang on which links.

    Returns:
        callable: (part a, part b) -> True when the distance depends on the base.
    """
    from scaffolding_assembly_core.mirrors.compas import subtree  # noqa: PLC0415

    def below(joint: str) -> set:
        return {f"{robot_id}/{name}" for name in subtree(model, model.get_joint_by_name(joint).child.link)}

    arm = set().union(*(below(joints[0]) - below(joints[-1]) for joints in arms))
    at_target = set().union(*(below(joints[-1]) for joints in arms))
    body = {f"{robot_id}/{link.name}" for link in model.links} - arm - at_target
    # ? A body hanging on a link moves with that link: with the arm, or rigidly with the base.
    hung = {name: f"{robot_id}/{item.attached_to_link}" for name, item in state.rigid_body_states.items()
            if item.attached_to_link}
    arm |= {name for name, link in hung.items() if link in arm}
    body |= {name for name, link in hung.items() if link in body}

    def group(part: str) -> str:
        return "arm" if part in arm else "body" if part in body else "world"

    def depends(a: str, b: str) -> bool:
        ga, gb = group(a), group(b)
        return "arm" in (ga, gb) or ga != gb

    return depends


# --- --- --- --- --- THE CHAINS AT THE SEED BASE --- --- --- --- ---

@dataclass
class Chain:
    """One chain of IK branches at the seed base: an approach pair and the pairs the chain solver takes after it.

    Attributes:
        configurations: Role -> the robot's full compas configuration at the seed base (ssik's raw joints).
        distances: As `Trial.distances`, at the seed base.
    """

    configurations: Dict[str, object]
    distances: Dict[Tuple[str, str, str], float]

    def lowest(self, band: float) -> float:
        return min(self.distances.values(), default=band)

    def nearest(self) -> Optional[Tuple[str, str, str]]:
        """The (role, part, part) with the smallest distance, or None."""
        return nearest_key(self.distances)

    def per_pose(self, band: float) -> Dict[str, Tuple[float, Optional[Tuple[str, str]]]]:
        """Role -> (its smallest distance, its pair), for the line under a candidate."""
        out = {}
        for role in ROLES:
            pair = nearest_key({key[1:]: value for key, value in self.distances.items() if key[0] == role})
            out[role] = (self.distances[(role, *pair)], pair) if pair else (band, None)
        return out


def group_values(state, group: str, rcell) -> List[float]:
    """One planning group's joint values in a state, in the group's order (tamp's `extract_group_config`)."""
    from husky_assembly_tamp.keyframe.dual_arm_ik import extract_group_config  # noqa: PLC0415

    return [float(v) for v in extract_group_config(state, group, rcell)["joint_values"]]


def chain_state(movements: Mapping[str, object], chain: Chain, role: str, frame_mm):
    """A chain's pose `role` at a base: the movement's state (its attachments) with the base and the chain's joints."""
    from husky_assembly_tamp.keyframe.dual_arm_ik import apply_base_frame_mm  # noqa: PLC0415

    state = movements[role].start_state.copy()
    apply_base_frame_mm(state, frame_mm)
    state.robot_configuration = chain.configurations[role].copy()
    return state


@dataclass
class ChainTable:
    """The chains at the seed base, for one failing movement's candidates (`chain_table`).

    Attributes:
        chains: Per candidate of the failing movement, in its order, the best chain through it, or None when no
            approach branch leads to it.
        fixed: (role, part, part) -> metres, the overlaps the base cannot change (below 0), e.g. a tool at its
            target in a bar.
    """

    chains: List[Optional[Chain]]
    fixed: Dict[Tuple[str, str, str], float]


def chain_table(planner, store: dict, robot_id: str, movements: Mapping[str, object], scenes: Mapping[str, object],
                seed_mm, failing: str, enumerated: Mapping[str, dict], depends, band: float,
                groups: Tuple[str, str], log=print) -> ChainTable:
    """Every chain at the seed base, one per approach pair, grouped by the failing movement's pair: the best of each.

    A chain goes forward as tamp's chain solver goes: each arm's next branch is the one nearest its last
    (`dual_arm_ik._nearest_cfg`, imported so the rule stays tamp's). The chains through a candidate of the failing
    movement are those that reach its pair there; the best is the one with the largest smallest distance.

    Args:
        planner: The lent planner.
        store: Where the mirrors are kept.
        robot_id: The planned robot.
        movements: Role -> keyframe movement (`process_planning.keyframe_movements`).
        scenes: Role -> the movement's core scene (`process.scene_at`).
        seed_mm: The seed base frame (4, 4), mm.
        failing: The failing movement's role, e.g. "M2".
        enumerated: Role -> `enumerate_ssik_candidate_pairs` result at the seed, for the roles already enumerated;
            the others are enumerated here.
        depends: `base_groups`' test.
        band: Distances are listed up to this, metres.
        groups: The (left, right) planning groups.
    """
    from husky_assembly_tamp.keyframe import dual_arm_ik  # noqa: PLC0415
    from husky_assembly_tamp.keyframe.ik_keyframe import frame_to_mm4  # noqa: PLC0415
    from core import process_planning  # noqa: PLC0415

    rcell = planner.client.robot_cell
    sides = tuple(zip(("left", "right"), groups))
    results = dict(enumerated)
    for role in ROLES:
        if role not in results:
            targets = movements[role].target_ee_frames
            results[role] = dual_arm_ik.enumerate_ssik_candidate_pairs(
                planner, movements[role].start_state, seed_mm, frame_to_mm4(targets["left"]),
                frame_to_mm4(targets["right"]), groups=groups)
    # * Per role and arm, the distinct branches; per role, the candidate state of each (left, right) pair, and
    #   each candidate's pair, in the candidates' order.
    branches, pairs, indices = {}, {}, {}
    for role in ROLES:
        branches[role] = {"left": [], "right": []}
        pairs[role], indices[role] = {}, []
        for candidate in results[role]["candidates"]:
            index = []
            for side, group in sides:
                values = tuple(round(v, 9) for v in group_values(candidate["state"], group, rcell))
                found = branches[role][side]
                if values not in found:
                    found.append(values)
                index.append(found.index(values))
            pairs[role][tuple(index)] = candidate["state"]
            indices[role].append(tuple(index))

    def forward(role_from: str, role_to: str, side: str, i: int) -> int:
        options = [SimpleNamespace(joint_values=list(v)) for v in branches[role_to][side]]
        nearest = dual_arm_ik._nearest_cfg(options, list(branches[role_from][side][i]))
        return options.index(nearest)

    # * One chain per approach pair: its pair at each role.
    chains = {}
    for start in pairs["M1"]:
        at, path = start, {"M1": start}
        for role_from, role_to in (("M1", "M2"), ("M2", "M3")):
            at = tuple(forward(role_from, role_to, side, i) for side, i in zip(("left", "right"), at))
            path[role_to] = at
        if all(path[role] in pairs[role] for role in ROLES):
            chains[start] = path
    # * The distances of each pair a chain uses, one role at a time (a check is valid until the next sync).
    measured, fixed = {}, {}
    for role in ROLES:
        needed = sorted({path[role] for path in chains.values()})
        if not needed:
            continue
        state = pairs[role][needed[0]]
        scene = process_planning.state_scene(scenes[role], robot_id, state)
        check, joint_names, _values = process_planning.arm_check(store, scene, robot_id, band, base_moves=True,
                                                                 log=log)
        for pair in needed:
            configuration = pairs[role][pair].robot_configuration
            values = [float(configuration[name]) for name in joint_names]
            found = {}
            for (a, b), distance in check.distances(values, signed=True).items():
                if depends(a, b):
                    found[(role, a, b)] = float(distance)
                elif distance < 0.0:
                    fixed[(role, a, b)] = float(distance)
            measured[(role, pair)] = found
    # * The failing movement's candidates, each with the best chain through it.
    out = []
    for index in indices[failing]:
        best = None
        for path in chains.values():
            if path[failing] != index:
                continue
            chain = Chain({role: pairs[role][path[role]].robot_configuration.copy() for role in ROLES},
                          {key: value for role in ROLES for key, value in measured[(role, path[role])].items()})
            if best is None or chain.lowest(band) > best.lowest(band):
                best = chain
        out.append(best)
    return ChainTable(out, fixed)


# --- --- --- --- --- FOLLOWING THE CHAIN AS THE BASE MOVES --- --- --- --- ---

class ChainProbe:
    """`evaluate` for `raise_clearance`: one chain's three poses at a nudged base.

    Each pose's joints come from Newton's method (`follow_arm`), always started from the chain's joints at the seed
    base, so a trial depends on `u` alone. Each pose's distances come from its own scene, synced with the robot at
    the trial base (the core's rule: build a check after each sync). The bounds, the ground and the home check come
    first: a trial that fails them is refused before any IK.
    """

    def __init__(self, planner, store: dict, robot_id: str, movements, scenes, chain: Chain, seed_mm, *,
                 snap: Callable, home_check: Callable, depends: Callable, groups: Tuple[str, str], band: float,
                 max_mm: float, max_deg: float, branch_deg: float, log=print):
        """
        Args:
            planner: The lent planner.
            store: Where the mirrors are kept.
            robot_id: The planned robot.
            movements, scenes: Role -> keyframe movement, and its core scene.
            chain: The chain to follow (`chain_table`).
            seed_mm: The seed base frame (4, 4), mm.
            snap: point_mm -> (ground point mm, normal), or (None, None) off the ground.
            home_check: `base_search.HomeCheck`: None where the robot, arms at home, collides.
            depends: `base_groups`' test.
            groups: The (left, right) planning groups.
            band: Distances are listed up to this, metres.
            max_mm, max_deg: How far the base may move and turn from the seed.
            branch_deg: How far any joint may get from the chain's at the seed, degrees.
        """
        from core import process_planning  # noqa: PLC0415

        self._planner, self._store, self._robot_id = planner, store, robot_id
        self._movements, self._scenes, self._chain = movements, scenes, chain
        self.seed_mm = np.asarray(seed_mm, dtype=float)
        self._snap, self._home, self._depends, self._groups = snap, home_check, depends, groups
        self.band, self.max_mm, self.max_deg = float(band), float(max_mm), float(max_deg)
        self._branch = float(np.radians(branch_deg))
        self._log, self._planning = log, process_planning
        targets = [np.asarray(frame.point, dtype=float) * 1000.0 for role in ROLES
                   for frame in movements[role].target_ee_frames.values()]
        self.centre_mm, self.rho_m = turn_centre(self.seed_mm, targets)
        rcell = planner.client.robot_cell
        model = rcell.robot_model
        sides = tuple(zip(("left", "right"), groups))
        self._names = {side: list(rcell.get_configurable_joint_names(group)) for side, group in sides}
        self._limits = {side: ([model.get_joint_by_name(n).limit.lower for n in names],
                               [model.get_joint_by_name(n).limit.upper for n in names])
                        for side, names in self._names.items()}
        self._seeds = {role: {side: group_values(chain_state(movements, chain, role, self.seed_mm), group, rcell)
                              for side, group in sides} for role in ROLES}
        self._ends = {side: rcell.get_end_effector_link_name(group) for side, group in sides}
        self._dets = {role: {side: arm_jacobian_det(planner.client, self._names[side], self._ends[side],
                                                    self._seeds[role][side]) for side in self._names}
                      for role in ROLES}

    def frame(self, u) -> np.ndarray:
        """The base frame of a nudge, (4, 4) mm."""
        return frame_at(self.seed_mm, u, self.centre_mm, self.rho_m)

    def moved(self, u) -> Tuple[float, float]:
        """(mm the base origin moved, degrees it turned) from the seed."""
        frame = self.frame(u)
        return float(np.linalg.norm(frame[:3, 3] - self.seed_mm[:3, 3])), float(np.degrees(u[2] / self.rho_m))

    def __call__(self, u) -> Trial:
        u = np.asarray(u, dtype=float)
        frame = self.frame(u)
        moved_mm, turned_deg = self.moved(u)
        if moved_mm > self.max_mm or abs(turned_deg) > self.max_deg:
            return Trial(False, "bound")
        # * On the ground: the nearest ground point within base_search.OFF_GROUND_MM, in plan as the base search
        #   measures it and in height too (this base keeps the seed's plane instead of following the ground).
        ground, _normal = self._snap(frame[:3, 3])
        off = None if ground is None else float(np.linalg.norm(np.asarray(ground, dtype=float) - frame[:3, 3]))
        if off is None or off > base_search.OFF_GROUND_MM:
            return Trial(False, "ground")
        if self._home(frame) is None:
            return Trial(False, "home")
        rcell = self._planner.client.robot_cell
        states, found = {}, {}
        for role in ROLES:
            movement = self._movements[role]
            state = chain_state(self._movements, self._chain, role, frame)
            self._planner.set_robot_cell_state(state)  # ? the base where Newton's method needs it
            for side, group in zip(("left", "right"), self._groups):
                followed = follow_arm(self._planner.client, self._names[side], self._ends[side],
                                      self._seeds[role][side], movement.target_ee_frames[side])
                if followed is None:
                    return Trial(False, "reach", f"the {side} arm at {POSES[role]}")
                values, det = followed
                arm = rcell.zero_configuration(group)
                by_name = dict(zip(self._names[side], values))
                arm.joint_values = [by_name[name] for name in arm.joint_names]
                state.robot_configuration.merge(arm)
                change = branch_change(values, self._seeds[role][side], *self._limits[side], self._branch,
                                       det, self._dets[role][side])
                if change:
                    return Trial(False, "branch", f"the {side} arm's {change} at {POSES[role]}")
            states[role] = state
            scene = self._planning.state_scene(self._scenes[role], self._robot_id, state)
            for (a, b), distance in self._planning.pose_distances(self._store, scene, self._robot_id, self.band,
                                                                  log=self._log).items():
                if self._depends(a, b):
                    found[(role, a, b)] = float(distance)
        return Trial(True, distances=found, data=SimpleNamespace(frame=frame, states=states))


# --- --- --- --- --- THE FINAL CHECK --- --- --- --- ---

def final_chain(planner, ordered, frame_mm, followed: Mapping[str, object], *, check_collision: bool,
                verbose_pairs: bool, groups: Tuple[str, str]):
    """tamp's chain solver at the resolved base, told to try the followed approach branch first.

    1. ssik's own joints for that branch at this base: the enumerated approach pair nearest the followed joints,
       wrapped (ssik returns raw angles, and joint 1 may run past +-pi), within 1 degree.
    2. `solve_keyframe_chain` with those 12 joints as `home_conf_12`: the cold approach solve tries its
       collision-free pairs nearest-first to it, so this branch first; assembled and retreat chain from it as always.
    3. Every solved pose compared with the followed one (wrapped, 2 degrees): the solver picks assembled and retreat
       by nearness to the pose before, which may pick another branch than the one the nudge cleared.

    Args:
        planner: The lent planner.
        ordered: [(role, movement), ...] for M1, M2, M3.
        frame_mm: The resolved base (4, 4), mm.
        followed: Role -> the followed state at that base (`Trial.data.states`, or the chain's at the seed).
        check_collision, verbose_pairs: As `_solve_chain_with_sampling` passes them.
        groups: The (left, right) planning groups.

    Returns:
        tuple: (solved {role: state} or None, the role that failed or "", the roles the solver put on another
        branch than the followed one). The role is "M1" also when step 1 finds no matching pair.
    """
    from husky_assembly_tamp.keyframe import dual_arm_ik, ik_keyframe  # noqa: PLC0415

    rcell = planner.client.robot_cell

    def both(state):
        """The 12 arm joints, left then right: the order `home_conf_12` takes."""
        return np.asarray([v for group in groups for v in group_values(state, group, rcell)])

    first = both(followed["M1"])
    movement = dict(ordered)["M1"]
    targets = movement.target_ee_frames
    found = dual_arm_ik.enumerate_ssik_candidate_pairs(
        planner, movement.start_state, frame_mm, ik_keyframe.frame_to_mm4(targets["left"]),
        ik_keyframe.frame_to_mm4(targets["right"]), groups=groups)["candidates"]
    raw = [both(c["state"]) for c in found]
    gaps = [float(np.max(np.abs(wrapped(values - first)))) for values in raw]
    if not raw or min(gaps) > np.radians(1.0):
        return None, "M1", []
    prefer = raw[int(np.argmin(gaps))]
    done = []
    solved = ik_keyframe.solve_keyframe_chain(
        planner, ordered, frame_mm, check_collision=check_collision, verbose_pairs=verbose_pairs,
        after_solve=lambda role, _state: done.append(role), home_conf_12=[float(v) for v in prefer])
    if solved is None:
        roles = [role for role, _movement in ordered]
        return None, roles[len(done)] if len(done) < len(roles) else roles[-1], []
    elsewhere = [role for role in solved
                 if float(np.max(np.abs(wrapped(both(solved[role]) - both(followed[role]))))) >= np.radians(2.0)]
    return solved, "", elsewhere
