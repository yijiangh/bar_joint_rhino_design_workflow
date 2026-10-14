"""Propose robot bases for a bar's keyframe chain by optimising a cost, instead of sampling them at random.

The cost of a base (ground point and heading) is `base_region.extension_cost` -- how far the arms are from a partly
extended pose over all flange targets -- plus a penalty where the robot, arms at home, stands near the scene. A base
where it collides there is ruled out: the robot returns to home at this same base after the retreat.

`proposals` yields bases around the base the user picked (the seed), which carries what the cost does not know: the
side of the bar to stand on, and the heading the left / right tools were assigned from. So the search stays within a
radius of the seed and within an angle of its heading:

1. Descend from the seed itself: a short derivative-free descent on (x, y, heading) of the full cost.
2. The arm cost of every ground grid cell in the radius at its best allowed heading: closed form, no planner.
3. Walk those cells from the cheapest and ask `HomeCheck` (one planner call each) until one does not collide,
   descend from there, and yield the result.
4. Asked again (the IK chain failed there), carry on with step 3 away from the bases already given.

Rhino-free (numpy, scipy and the core; `HomeCheck` needs the planner's compas_fab and PyBullet).

- ! A proposal is not a promise: joint limits, the arms' own collisions on the way and the IK branches are not looked
  at. The caller still solves the chain there.
"""

from __future__ import annotations

from typing import Callable, Iterator, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize

from core import base_region

#: What a base that cannot be used costs in the descent (finite: Nelder-Mead compares values).
_RULED_OUT = 1.0e3
#: A descent step is ruled out when the ground is farther than this from where it asked to stand, in mm.
_OFF_GROUND_MM = 1.0


class HomeCheck:
    """The robot at a base with its arms at home: does it collide with the scene, and how near is it?

    Uses the planner's own check on the retreat scene (the new bar in place and let go), so it sees what the IK
    solve will see.

    Attributes:
        available (bool): False when the process has no home configuration: every base then passes, unmeasured.
        calls (int): How many bases were checked.
    """

    def __init__(self, planner, movements, near_m: float = 0.0, log=print):
        """
        Args:
            planner: The lent planner.
            movements (dict): The keyframe movements; "M4" carries the home configuration and the retreat scene.
            near_m: Measure the clearance up to this distance in metres; 0 = only collide or not.
            log: Where the one line about a robot that collides with itself goes.
        """
        self._planner, self._near, self._log = planner, float(near_m), log
        self.calls = 0
        home = movements["M4"].target_configuration
        self.available = home is not None
        self._self_checked = False
        self._always_collides = False
        if self.available:
            self._state = movements["M4"].start_state.copy()
            self._state.robot_configuration = home.copy()
            self._link_names = {index: name for name, index in planner.client.robot_link_puids.items()}

    def __call__(self, frame_mm) -> Optional[float]:
        """None where the robot collides; else its clearance in metres, capped at `near_m` (NaN when not measured).

        Args:
            frame_mm: The base frame (4, 4), translation in mm.
        """
        from compas_fab.backends import CollisionCheckError  # noqa: PLC0415  (heavy, and only needed here)
        from husky_assembly_tamp.keyframe.dual_arm_ik import apply_base_frame_mm  # noqa: PLC0415

        if not self.available:
            return float("nan")
        self.calls += 1
        quiet = {"full_report": False, "verbose": False}
        apply_base_frame_mm(self._state, frame_mm)
        if not self._self_checked:
            # The robot against itself and its own tools (compas_fab's steps CC.1 and CC.2, about 570 link pairs)
            # does not depend on where the base stands: check that once, then only against the scene per base.
            self._self_checked = True
            try:
                self._planner.check_collision(
                    self._state, options=dict(quiet, _skip_cc3=True, _skip_cc4=True, _skip_cc5=True))
            except CollisionCheckError as exc:
                self._always_collides = True
                self._log(f"base_search: the robot collides with itself at home, whatever the base: {exc}")
        if self._always_collides:
            return None
        try:
            self._planner.check_collision(self._state, options=dict(quiet, _skip_cc1=True, _skip_cc2=True))
        except CollisionCheckError:
            return None
        return self._clearance() if self._near > 0.0 else float("nan")

    def _clearance(self) -> float:
        """How near the robot's links are to the scene's standing bodies (the check above left it at the base).

        PyBullet's closest-point query, which the collision check asks with distance 0. Skips what the check skips:
        hidden bodies, bodies the robot carries, and links allowed to touch a body.
        """
        import pybullet  # noqa: PLC0415  (only needed here)

        client, nearest = self._planner.client, self._near
        for name, body_ids in client.rigid_bodies_puids.items():
            body = self._state.rigid_body_states[name]
            if body.is_hidden or body.attached_to_tool or body.attached_to_link:
                continue
            for body_id in body_ids:
                for point in pybullet.getClosestPoints(bodyA=client.robot_puid, bodyB=body_id, distance=self._near,
                                                       physicsClientId=client.client_id):
                    if self._link_names.get(point[3]) not in body.touch_links:  # [3] robot link, [8] distance
                        nearest = min(nearest, point[8])
        return max(nearest, 0.0)


class Trace:
    """What one search did, for `core.base_search_viz` to draw: lists the search appends to as it goes.

    Attributes:
        cells: (base frames (N, 4, 4) in mm, arm costs (N,)) of the ground cells the search ranked in closed form,
            each at its cheapest allowed heading; None before it did.
        costs: (frame, cost) of every base whose full cost was computed.
        checks: (frame, collides) of every base handed to the collision check.
        paths: Per descent, its start and then every base cheaper than all before it.
        proposed: The frames yielded.
    """

    def __init__(self):
        self.cells = None
        self.costs, self.checks, self.paths, self.proposed = [], [], [], []


def crowding(clearance_m, near_m: float, weight: float):
    """The cost of standing near the scene: 0 from `near_m` on, rising with the square to `weight` at contact.

    NaN (not measured) costs nothing. Works on numbers and arrays.
    """
    if near_m <= 0.0:
        return np.zeros_like(np.asarray(clearance_m, dtype=float)) + 0.0
    short = np.clip(1.0 - np.nan_to_num(np.asarray(clearance_m, dtype=float), nan=near_m) / near_m, 0.0, 1.0)
    return weight * short * short


def frame_yaw(frame_mm) -> float:
    """The heading of a base frame, in the convention of `base_region.feasible_yaws`."""
    frame = np.asarray(frame_mm, dtype=float)
    ahead, left = base_region.heading_axes(frame[None, :3, 2])
    return float(np.arctan2(frame[:3, 0] @ left[0], frame[:3, 0] @ ahead[0]))


def _turned(yaws, facing: float):
    """How far headings are turned away from `facing`, in radians (0 to pi)."""
    return np.abs((np.asarray(yaws, dtype=float) - facing + np.pi) % (2.0 * np.pi) - np.pi)


def ranked_cells(soups, limits: Sequence[base_region.Constraint], step_mm: float, yaw_steps: int, prefer: float,
                 seed_mm=None, radius_mm: float = 0.0, facing: Optional[float] = None,
                 facing_range: float = np.pi) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Every ground grid cell the arms reach all targets from, at its cheapest heading, cheapest cell first.

    Args:
        seed_mm, radius_mm: Keep only the cells within this distance of this point in plan; no seed = all.
        facing, facing_range: Keep only the headings within this angle of this one, in radians; no facing = all.

    Returns:
        tuple: (points (N, 3) in mm, unit normals (N, 3), headings (N,) in radians, arm costs (N,)).
    """
    lower, upper = base_region.window(limits)
    points, normals = base_region.ground_grid(soups, lower, upper, step_mm)
    yaws = np.linspace(-np.pi, np.pi, int(yaw_steps), endpoint=False)
    if seed_mm is not None and len(points):
        inside = np.hypot(points[:, 0] - seed_mm[0], points[:, 1] - seed_mm[1]) <= radius_mm
        points, normals = points[inside], normals[inside]
    if not len(points):
        return points, normals, np.zeros(0), np.zeros(0)
    allowed = base_region.feasible_yaws(points, normals, limits, yaws)
    if facing is not None:
        allowed &= (_turned(yaws, facing) <= facing_range)[None, :]
    costs = np.where(allowed, base_region.extension_cost(points, normals, limits, yaws, prefer), np.inf)
    cheapest = costs.argmin(axis=1)
    cost = costs[np.arange(len(points)), cheapest]
    order = np.argsort(cost, kind="stable")
    order = order[np.isfinite(cost[order])]
    return points[order], normals[order], yaws[cheapest[order]], cost[order]


def _descend(point_mm, yaw: float, start_cost: float, start_frame, limits, check: Callable, snap: Callable, *,
             step_mm: float, yaw_step: float, prefer: float, near_m: float, weight: float, evals: int,
             seed_mm=None, radius_mm: float = 0.0, facing: Optional[float] = None, facing_range: float = np.pi,
             avoid=(), avoid_mm: float = 0.0, trace: Optional[Trace] = None):
    """Nelder-Mead on (x, y, heading) from one usable base; returns the cheapest (cost, frame) it saw.

    ? Derivative-free: the clearance has kinks wherever another pair of bodies becomes the nearest, and every value
    costs a planner call, so a few dozen evaluations of a 3-variable simplex are the whole budget.
    """
    best = [start_cost, start_frame]
    path = [start_frame] if start_cost < _RULED_OUT else []
    if trace is not None:
        trace.paths.append(path)
        if path:
            trace.costs.append((start_frame, start_cost))

    def cost_at(u) -> float:
        asked = np.array([point_mm[0] + u[0] * step_mm, point_mm[1] + u[1] * step_mm, point_mm[2]])
        if seed_mm is not None and float(np.hypot(asked[0] - seed_mm[0], asked[1] - seed_mm[1])) > radius_mm:
            return _RULED_OUT
        # Descents from different cells run into the same low point: keep off the bases already tried.
        if any(float(np.hypot(asked[0] - x, asked[1] - y)) < avoid_mm for x, y in avoid):
            return _RULED_OUT
        if facing is not None and float(_turned(yaw + u[2] * yaw_step, facing)) > facing_range:
            return _RULED_OUT
        on_ground, normal = snap(asked)
        if on_ground is None or float(np.hypot(*(np.asarray(on_ground)[:2] - asked[:2]))) > _OFF_GROUND_MM:
            return _RULED_OUT
        at, up, heading = np.asarray(on_ground, dtype=float)[None, :], np.asarray(normal, dtype=float)[None, :], \
            [yaw + u[2] * yaw_step]
        if not base_region.feasible_yaws(at, up, limits, heading)[0, 0]:
            return _RULED_OUT
        frame = base_region.base_frames(at, up, heading)[0]
        clearance = check(frame)
        if clearance is None:
            return _RULED_OUT
        value = float(base_region.extension_cost(at, up, limits, heading, prefer)[0, 0]
                      + crowding(clearance, near_m, weight))
        if trace is not None:
            trace.costs.append((frame, value))
        if value < best[0]:
            best[:] = [value, frame]
            path.append(frame)
        return value

    if evals > 0:
        simplex = np.vstack([np.zeros(3), np.eye(3)])
        minimize(cost_at, np.zeros(3), method="Nelder-Mead",
                 options={"initial_simplex": simplex, "maxfev": int(evals), "xatol": 0.05, "fatol": 1e-5})
    return best[0], best[1]


def proposals(soups, limits: Sequence[base_region.Constraint], check: Callable, snap: Callable, *,
              step_mm: float, yaw_steps: int, prefer: float, near_m: float, weight: float, spacing_mm: float,
              max_checks: int, refine_evals: int, seed_frame_mm=None, radius_mm: float = 0.0,
              facing_range: float = np.pi, log=print, trace: Optional[Trace] = None) -> Iterator[np.ndarray]:
    """Yield base frames (4, 4, translation in mm) for the caller to solve the IK chain at.

    With a seed: first the descent from the seed itself, then the cells around it from the cheapest. The seed itself
    is never yielded (the caller tried it).

    Args:
        soups: The ground as triangle soups, each (vertices (V, 3) in mm, triangles (T, 3) of vertex rows).
        limits: The flange targets as `base_region.constraints`.
        check: `HomeCheck`, or any ``frame_mm -> None | clearance in metres``.
        snap: ``point_mm -> (closest ground point in mm, unit normal)``, or (None, None) off the ground.
        step_mm, yaw_steps: The grid of ground cells and headings the search starts from.
        prefer: The arm extension that costs nothing (see `base_region.extension_cost`).
        near_m, weight: The clearance penalty (see `crowding`).
        spacing_mm: A later proposal starts at least this far from every earlier one.
        max_checks: How many grid cells may be collision-checked in all (the descents' checks come on top).
        refine_evals: Cost evaluations per descent; 0 = yield the grid cell as it is.
        seed_frame_mm: The base the user picked (4, 4), or None to search the whole ground at any heading.
        radius_mm: How far from the seed, in plan, a base may stand.
        facing_range: How far from the seed's heading a base may be turned, in radians.
        log: Where the progress lines go.
        trace: Filled with what the search does, as it goes.
    """
    if trace is not None:
        unrecorded = check

        def check(frame):
            clearance = unrecorded(frame)
            trace.checks.append((np.array(frame, dtype=float), clearance is None))
            return clearance

    seed_mm = facing = None
    if seed_frame_mm is not None:
        seed_frame_mm = np.asarray(seed_frame_mm, dtype=float)
        seed_mm, facing = seed_frame_mm[:3, 3], frame_yaw(seed_frame_mm)
    yaw_step = 2.0 * np.pi / int(yaw_steps)
    bounds = dict(seed_mm=seed_mm, radius_mm=radius_mm, facing=facing, facing_range=facing_range)
    descent = dict(step_mm=step_mm, yaw_step=yaw_step, prefer=prefer, near_m=near_m, weight=weight,
                   evals=refine_evals, trace=trace, **bounds)
    # Keep-away discs (x, y, radius in mm): around a colliding cell just its neighbours, which mostly collide too;
    # around a base already tried the full spacing.
    discs = np.zeros((0, 3))
    tried = [] if seed_mm is None else [(seed_mm[0], seed_mm[1])]
    checks = given = 0

    if seed_mm is not None:
        discs = np.vstack([discs, [seed_mm[0], seed_mm[1], spacing_mm]])
        at, up = seed_mm[None, :], seed_frame_mm[None, :3, 2]
        start = _RULED_OUT
        if base_region.feasible_yaws(at, up, limits, [facing])[0, 0]:
            clearance = check(seed_frame_mm)
            if clearance is not None:
                start = float(base_region.extension_cost(at, up, limits, [facing], prefer)[0, 0]
                              + crowding(clearance, near_m, weight))
        cost, frame = _descend(seed_mm, facing, start, seed_frame_mm, limits, check, snap, **descent)
        moved = float(np.hypot(*(frame[:2, 3] - seed_mm[:2])))
        turned = float(np.degrees(_turned(frame_yaw(frame), facing)))
        if cost >= _RULED_OUT:
            log("base_search: the picked base collides at home or is out of reach, and so is everything a step "
                "away; looking further around it.")
        elif moved < 1.0 and turned < 0.5:
            log(f"base_search: the picked base is already the cheapest around (cost {cost:.3f}); looking further.")
        else:
            given += 1
            log(f"base_search: proposal {given} at ({frame[0, 3]:.0f}, {frame[1, 3]:.0f}, {frame[2, 3]:.0f}) mm, "
                f"cost {cost:.3f} (picked base "
                + ("unusable" if start >= _RULED_OUT else f"{start:.3f}")
                + f"; moved {moved:.0f} mm, turned {turned:.0f} deg from it).")
            discs = np.vstack([discs, [frame[0, 3], frame[1, 3], spacing_mm]])
            tried.append((frame[0, 3], frame[1, 3]))
            if trace is not None:
                trace.proposed.append(frame)
            yield frame

    points, normals, yaws, costs = ranked_cells(soups, limits, step_mm, yaw_steps, prefer, **bounds)
    if trace is not None:
        trace.cells = (base_region.base_frames(points, normals, yaws) if len(points) else np.zeros((0, 4, 4)), costs)
    if not len(points):
        log("base_search: no ground cell "
            + ("within the search radius of the picked base, facing its way, " if seed_mm is not None else "")
            + "has every flange target within the arms' reach.")
        return
    for row in range(len(points)):
        at = points[row]
        if len(discs) and (np.hypot(discs[:, 0] - at[0], discs[:, 1] - at[1]) < discs[:, 2]).any():
            continue
        if checks >= max_checks:
            log(f"base_search: stopped after {checks} collision-checked cell(s), {given} base(s) proposed.")
            return
        frame = base_region.base_frames(at[None, :], normals[row][None, :], [yaws[row]])[0]
        checks += 1
        clearance = check(frame)
        if clearance is None:
            discs = np.vstack([discs, [at[0], at[1], 1.5 * step_mm]])
            continue
        start = float(costs[row] + crowding(clearance, near_m, weight))
        cost, frame = _descend(at, float(yaws[row]), start, frame, limits, check, snap, avoid=tried,
                               avoid_mm=spacing_mm, **descent)
        given += 1
        moved = float(np.hypot(*(frame[:2, 3] - at[:2])))
        log(f"base_search: proposal {given} at ({frame[0, 3]:.0f}, {frame[1, 3]:.0f}, {frame[2, 3]:.0f}) mm, cost "
            f"{cost:.3f} (grid cell {start:.3f}, moved {moved:.0f} mm; cell {row + 1} of {len(points)} by arm cost, "
            f"{checks} checked so far).")
        discs = np.vstack([discs, [at[0], at[1], spacing_mm], [frame[0, 3], frame[1, 3], spacing_mm]])
        tried.append((frame[0, 3], frame[1, 3]))
        if trace is not None:
            trace.proposed.append(frame)
        yield frame
    log(f"base_search: every reachable ground cell is used up ({checks} checked, {given} base(s) proposed).")
