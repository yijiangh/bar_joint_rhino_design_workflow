"""Paint on the walkable ground where the robot base can stand at all (RSIKKeyframe, DEBUG AID).

Rhino half of :mod:`core.base_region` (which owns the math). Per ground grid cell from which every flange target of
the bar's approach / assembled / retreat keyframes is within reach for at least one base heading:

- a dot, pale to dark purple with the width of the longest unbroken range of headings that works there;
- an ORANGE dot instead where the working headings fall into two or more separate ranges;
- a tiny arrow in the dot's color along the middle of that longest range (none where every heading works);
- a CROSS in the same color instead of dot and arrow where the robot collides standing there, arms at home, facing along the arrow
  (toward the targets where every heading works). Checked for every cell, up to a time budget.

With ``config.DEBUG_BASE_REGION_SHADE = "cost"`` the shade and the arrows show what the base search
(:mod:`core.base_search`, ``config.IK_BASE_SEARCH_*``) sees instead: dark = the arms are near a partly extended pose (plus, optionally, the robot is clear of the scene), every
arrow points along the cheapest heading there, and one big dot marks the cheapest cell. No orange dots then.

No mark = no IK solution from that spot, whatever the heading. A dot is NOT a promise: joint limits and the arms'
collisions on the way are not looked at, and a cross speaks for one heading only.

Drawn by a display conduit, so nothing is baked into the document. Switched by ``config.DEBUG_BASE_REGION``.

To remove this view: delete this file; the ``DEBUG_BASE_REGION*`` constants in ``core/config.py``; and the lines
naming ``base_region_viz`` in ``rs_ik_keyframe.py`` (the import, the reload, the one ``stack.enter_context`` call).
KEEP ``core/base_region.py`` and ``core/base_search.py``: the base search of RSIKKeyframe runs on them.
"""

from __future__ import annotations

import contextlib
import importlib
import os
import time

import numpy as np
import Rhino
import scriptcontext as sc
import session_log
import System
from System.Drawing import Color

from core import base_region as _base_region_module
from core import base_search
from core import config
from core import rhino_walkable_ground
from core.rhino_helpers import doc_unit_scale_to_mm
from husky_assembly_tamp.keyframe.dual_arm_ik import frame_to_m_matrix

# Follows this module's own reload, so an edit to the math takes effect without restarting Rhino.
base_region = importlib.reload(_base_region_module)

# The reach search takes about a second and the reload above drops its cache, so the results live in sc.sticky.
_STICKY_REACH = "bar_joint:base_region_reach"
# Last conduit switched on, so one stranded by a hard script abort is switched off on the next run.
_STICKY_CONDUIT = "bar_joint:base_region_conduit"

_ROLES = ("M1", "M2", "M3")
# Purple is free in the preview-color registry (see core.base_guide_viz). Pale = narrow heading range, dark = wide.
_COLOR_NARROW = (225, 215, 245)
_COLOR_WIDE = (90, 30, 170)
_COLOR_SPLIT = (255, 140, 0)
# How many steps the pale-to-dark scale has.
_SHADES = 8
# The marks float this far above the ground so the ground surface does not swallow them.
_LIFT_MM = 5.0
# Arrow length and cross half-size, as shares of the grid spacing.
_ARROW_SHARE = 0.7
_CROSS_SHARE = 0.3


class _RegionConduit(Rhino.Display.DisplayConduit):
    """Draws the dots (a colored point cloud), the heading arrows and the collision crosses."""

    def __init__(self, cloud, arrows, crosses, best=None):
        super().__init__()
        self._cloud = cloud
        self._best = best
        # DrawArrows takes one color per call: one batch of arrows per color.
        self._arrows = [(Color.FromArgb(*rgb), System.Array[Rhino.Geometry.Line](lines))
                        for rgb, lines in arrows.items()]
        self._crosses = [(Color.FromArgb(*rgb), System.Array[Rhino.Geometry.Line](lines))
                         for rgb, lines in crosses.items()]
        self._box = cloud.GetBoundingBox(False)
        for lines in crosses.values():
            for line in lines:
                self._box.Union(line.BoundingBox)

    def CalculateBoundingBox(self, e):
        e.IncludeBoundingBox(self._box)

    def PostDrawObjects(self, e):
        try:
            if self._cloud.Count:
                e.Display.DrawPointCloud(self._cloud, int(config.DEBUG_BASE_REGION_DOT_PX))
            for color, lines in self._arrows:
                e.Display.DrawArrows(lines, color)
            for color, lines in self._crosses:
                e.Display.DrawLines(lines, color, 2)
            if self._best is not None:
                e.Display.DrawPoint(self._best, Rhino.Display.PointStyle.RoundControlPoint, 12,
                                    Color.FromArgb(*_COLOR_WIDE))
        except Exception as exc:  # noqa: BLE001 -- runs on every redraw: fail once, then stay out of the way
            self.Enabled = False
            session_log.caught(exc)
            print(f"RSIKKeyframe (debug): base region drawing failed and was switched off "
                  f"({type(exc).__name__}: {exc}).")


def _reach(urdf) -> dict:
    """``{side: base_region.wrist_reach}`` for both arms, kept across command runs."""
    path = os.path.abspath(str(urdf))
    cache = sc.sticky.setdefault(_STICKY_REACH, {})
    key = (path, os.stat(path).st_mtime_ns)
    if key not in cache:
        cache.clear()
        cache[key] = {side: base_region.wrist_reach(path, f"{side}_ur_arm_tool0") for side in config.ARM_SIDES}
    return cache[key]


def _collisions(planner, movements, frames_mm, near_m=0.0):
    """Which of the base frames put the robot, arms at home, in collision (`base_search.HomeCheck`).

    Returns:
        tuple: (hits (N,) of bool, clearance (N,) in metres -- capped at `near_m`, NaN where not measured --,
        checked, asked) -- how many checks ran within the budget, of how many asked for.
    """
    hits = np.zeros(len(frames_mm), dtype=bool)
    clearance = np.full(len(frames_mm), np.nan)
    check = base_search.HomeCheck(planner, movements, near_m, log=lambda line: print(f"RSIKKeyframe (debug): {line}"))
    if not check.available:
        return hits, clearance, 0, len(frames_mm)
    deadline = time.time() + float(config.DEBUG_BASE_REGION_COLLISION_BUDGET_S)
    checked = 0
    for row, frame in enumerate(frames_mm):
        if time.time() > deadline:
            break
        found = check(frame)
        hits[row] = found is None
        if found is not None:
            clearance[row] = found
        checked += 1
    return hits, clearance, checked, len(frames_mm)


def _point(xyz):
    return Rhino.Geometry.Point3d(float(xyz[0]), float(xyz[1]), float(xyz[2]))


def _build(bar_oid, movements, urdf, planner):
    """The conduit for one bar, or None when there is nothing to paint (the reason is printed)."""
    started = time.time()
    grounds = rhino_walkable_ground.get_all_walkable_grounds()
    soups = rhino_walkable_ground._bar_ground_soups(bar_oid, grounds)
    if not soups:
        print("RSIKKeyframe (debug): base region skipped, the bar has no assigned WalkableGround.")
        return None
    targets = {role: {side: frame_to_m_matrix(frame)
                      for side, frame in (movements[role].target_ee_frames or {}).items()}
               for role in _ROLES}
    limits = base_region.constraints(_reach(urdf), targets)
    if not limits:
        print("RSIKKeyframe (debug): base region skipped, the movements carry no flange targets.")
        return None

    step = float(config.DEBUG_BASE_REGION_STEP_MM)
    lower, upper = base_region.window(limits)
    cells, cell_normals = base_region.ground_grid(soups, lower, upper, step)
    count = int(config.DEBUG_BASE_REGION_YAW_STEPS)
    yaws = np.linspace(-np.pi, np.pi, count, endpoint=False)
    ok = base_region.feasible_yaws(cells, cell_normals, limits, yaws)
    length, middle, runs = base_region.widest_run(ok)
    possible = length > 0
    if not possible.any():
        print(f"RSIKKeyframe (debug): base region is EMPTY: none of {len(cells)} ground cell(s) has all "
              f"{len(limits)} flange targets within reach. No base on this ground can work.")
        return None
    points, normals, ok, length, middle, runs = (
        a[possible] for a in (cells, cell_normals, ok, length, middle, runs))

    by_cost = getattr(config, "DEBUG_BASE_REGION_SHADE", "heading") == "cost"
    anywhere = length == count
    if by_cost:
        # The heading each cell is shown (and collision-checked) with: the cheapest one that works there.
        costs = np.where(ok, base_region.extension_cost(
            points, normals, limits, yaws, float(config.IK_BASE_SEARCH_PREFERRED_EXTENSION)), np.inf)
        cheapest = costs.argmin(axis=1)
        cost, shown_yaw = costs[np.arange(len(points)), cheapest], yaws[cheapest]
    else:
        # ... the middle of its longest range; where every heading works there is no such middle: face the targets.
        ahead, left = base_region.heading_axes(normals)
        to_targets = np.mean([wrist for _, wrist, _ in limits], axis=0) * 1000.0 - points
        facing = np.arctan2((to_targets * left).sum(axis=1), (to_targets * ahead).sum(axis=1))
        shown_yaw = np.where(anywhere, facing, yaws[0] + middle * 2.0 * np.pi / count)
    frames = base_region.base_frames(points, normals, shown_yaw)

    reach_seconds = time.time() - started
    started = time.time()
    near_m = float(config.IK_BASE_SEARCH_CLEARANCE_MM) / 1000.0 if by_cost else 0.0
    clearance = np.full(len(points), np.nan)
    if config.DEBUG_BASE_REGION_COLLISIONS:
        collides, clearance, checked, asked = _collisions(planner, movements, frames, near_m)
    else:
        collides, checked, asked = np.zeros(len(points), dtype=bool), 0, 0
    collision_seconds = time.time() - started

    # How dark each dot is, 0..1: the heading range's width, or the cost turned around (cheapest = darkest).
    best = None
    if by_cost:
        measured = np.isfinite(clearance)
        if measured.any():
            # A colliding cell has no clearance at all: the full penalty, so its shade continues its neighbours'.
            crowded = np.where(measured, 1.0 - clearance / near_m, np.where(collides, 1.0, 0.0))
            cost = cost + float(config.IK_BASE_SEARCH_CLEARANCE_WEIGHT) * crowded * crowded
        low, high = float(cost.min()), float(cost.max())
        dark = 1.0 - (cost - low) / max(high - low, 1e-12)
        if not collides.all():
            best = int(np.where(collides, np.inf, cost).argmin())
    else:
        dark = length / count

    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    lifted = (points + _LIFT_MM * normals) * scale_from_mm
    heading, sideways = frames[:, :3, 0], frames[:, :3, 1]
    narrow, wide = np.array(_COLOR_NARROW, dtype=float), np.array(_COLOR_WIDE, dtype=float)
    cloud = Rhino.Geometry.PointCloud()
    arrows, crosses = {}, {}
    for i, at in enumerate(lifted):
        # Shown in a few shades, so the arrows (and the crosses) of one shade can be drawn together.
        share = round(dark[i] * _SHADES) / _SHADES
        rgb = _COLOR_SPLIT if runs[i] > 1 and not by_cost else narrow + (wide - narrow) * share
        rgb = (int(rgb[0]), int(rgb[1]), int(rgb[2]))
        if collides[i]:
            for diagonal in (heading[i] + sideways[i], heading[i] - sideways[i]):
                half = diagonal * (_CROSS_SHARE * step * scale_from_mm)
                crosses.setdefault(rgb, []).append(Rhino.Geometry.Line(_point(at - half), _point(at + half)))
            continue
        cloud.Add(_point(at), Color.FromArgb(*rgb))
        if by_cost or not anywhere[i]:
            arrows.setdefault(rgb, []).append(
                Rhino.Geometry.Line(_point(at), _point(at + heading[i] * (_ARROW_SHARE * step * scale_from_mm))))

    print(f"RSIKKeyframe (debug): base region painted, {len(points)} of {len(cells)} ground cell(s) possible, "
          f"longest heading range {length.max() * 360.0 / count:.0f} deg, {int((runs > 1).sum())} cell(s) with "
          f"separate heading ranges ({reach_seconds:.2f} s). Outer bound only: joint limits are not checked.")
    if best is not None:
        at, ahead = points[best], frames[best, :3, 0]
        near = f", {clearance[best] * 1000.0:.0f} mm clear" if np.isfinite(clearance[best]) else ""
        print(f"RSIKKeyframe (debug): base region shaded by COST (dark = cheap), {low:.3f} to {high:.3f}. Cheapest "
              f"base at ({at[0]:.0f}, {at[1]:.0f}, {at[2]:.0f}) mm, heading "
              f"{np.degrees(np.arctan2(ahead[1], ahead[0])):.0f} deg from world X{near} (the big dot).")
    if by_cost and near_m > 0.0 and np.isfinite(clearance).any():
        seen = clearance[np.isfinite(clearance)] * 1000.0
        print(f"RSIKKeyframe (debug): base region clearance of the robot at home, {seen.min():.0f} to "
              f"{seen.max():.0f} mm (capped at {near_m * 1000.0:.0f}); {int((seen < near_m * 1000.0 - 1e-6).sum())} "
              f"of {len(seen)} free cell(s) are penalised.")
    if movements["M4"].target_configuration is None:
        print("RSIKKeyframe (debug): base region has no collision crosses, the design has no home configuration.")
    elif asked:
        print(f"RSIKKeyframe (debug): base region collisions, {int(collides.sum())} of {checked} checked cell(s) "
              f"collide with the arms at home ({collision_seconds:.1f} s)"
              + (f"; STOPPED at the {float(config.DEBUG_BASE_REGION_COLLISION_BUDGET_S):.0f} s budget, "
                 f"{asked - checked} cell(s) left unchecked." if checked < asked else "."))
    return _RegionConduit(cloud, arrows, crosses, None if best is None else _point(lifted[best]))


@contextlib.contextmanager
def painted(bar_oid, movements, urdf, planner):
    """Show the base region of one bar for the duration of the with-block; never raises, never blocks the command.

    Args:
        bar_oid: The bar's centerline curve id (its assigned WalkableGround is painted).
        movements (dict): ``{"M1": .., "M2": .., "M3": .., "M4": ..}``; M1..M3 carry ``target_ee_frames``
            {"left", "right"}, M4 the home configuration.
        urdf: The acting robot's URDF.
        planner: The lent planner, for the collision crosses.
    """
    conduit = None
    try:
        stale = sc.sticky.pop(_STICKY_CONDUIT, None)
        if stale is not None:
            stale.Enabled = False
        if getattr(config, "DEBUG_BASE_REGION", False):
            conduit = _build(bar_oid, movements, urdf, planner)
            if conduit is not None:
                conduit.Enabled = True
                sc.sticky[_STICKY_CONDUIT] = conduit
                sc.doc.Views.Redraw()
    except Exception as exc:  # noqa: BLE001 -- a debug view must never break the command
        session_log.caught(exc)
        print(f"RSIKKeyframe (debug): base region unavailable ({type(exc).__name__}: {exc}).")
    try:
        yield conduit
    finally:
        if conduit is not None:
            conduit.Enabled = False
            sc.sticky.pop(_STICKY_CONDUIT, None)
            sc.doc.Views.Redraw()
