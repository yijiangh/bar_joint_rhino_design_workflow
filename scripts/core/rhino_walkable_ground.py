"""Rhino-side WalkableGround helpers: stable ids, meshing, bar association.

This is the Rhino half of the WalkableGround feature (the numpy half lives in
``husky_assembly_tamp.keyframe.walkable_ground``). It handles everything that
needs ``rhinoscriptsyntax`` / ``Rhino``:

- give every WalkableGround brep a stable id (``WG0``, ``WG1``, ...) stored as
  user-text so it survives copy/paste and matches the exported id;
- mesh a WalkableGround brep and convert it to a COMPAS ``Mesh`` (world mm) for
  ``WalkableGround.json``;
- the distance heuristic that auto-associates a bar with its nearest ground(s);
- read/write the bar's associated ground-id list (comma-separated user-text).

Ids and the association list are read back by ``rs_export_all_bar_actions`` at
export time and by ``rs_assign_and_show_walkable_ground`` (the interactive editor
+ base-placement preview).
"""

from __future__ import annotations

import json
from typing import Dict, List, Optional

import numpy as np
import Rhino
import rhinoscriptsyntax as rs

from compas.datastructures import Mesh

from core import config
# The numpy half moved into the tamp submodule with the solvers (core.config
# put it on sys.path); imported under the same private name as before.
from husky_assembly_tamp.keyframe import walkable_ground as _walkable_np
from core.rhino_frame_io import doc_unit_scale_to_mm


# Object types accepted on the WalkableGround layer (same set rs_ik_keyframe uses).
_WALKABLE_TYPES = {rs.filter.surface, rs.filter.polysurface, rs.filter.extrusion}


# ---------------------------------------------------------------------------
# Brep coercion + layer scan
# ---------------------------------------------------------------------------


def as_brep(object_id):
    """Return the ``Rhino.Geometry.Brep`` for *object_id*.

    Handles the Extrusion special case (a closed box primitive), where
    ``rs.coercebrep`` returns ``None`` and we convert the extrusion to a brep.

    Args:
        object_id: a Rhino object id on the WalkableGround layer.

    Returns:
        Rhino.Geometry.Brep: the brep geometry.

    Raises:
        RuntimeError: if the object is not a Brep / Surface / Extrusion.
    """
    brep = rs.coercebrep(object_id)
    if brep is not None:
        return brep
    rhobj = rs.coercerhinoobject(object_id, True, True)
    geom = getattr(rhobj, "Geometry", None)
    if isinstance(geom, Rhino.Geometry.Extrusion):
        return geom.ToBrep(False)
    raise RuntimeError(f"Object {object_id} is not a Brep, Surface or Extrusion.")


def _walkable_oids() -> List:
    """Return the object ids of every surface/polysurface/extrusion on the layer."""
    layer = config.WALKABLE_GROUND_LAYER
    if not rs.IsLayer(layer):
        rs.AddLayer(layer)
        return []
    return [
        oid for oid in rs.ObjectsByLayer(layer) or []
        if rs.ObjectType(oid) in _WALKABLE_TYPES
    ]


# ---------------------------------------------------------------------------
# Stable WalkableGround ids
# ---------------------------------------------------------------------------


def _parse_ground_num(ground_id) -> int:
    """Parse the integer part of a ``WGn`` id, or -1 if it doesn't match."""
    prefix = config.WALKABLE_GROUND_ID_PREFIX
    if not ground_id or not ground_id.startswith(prefix):
        return -1
    try:
        return int(ground_id[len(prefix):])
    except ValueError:
        return -1


def _next_ground_id() -> str:
    """Return the next free ``WGn`` id (max existing number + 1)."""
    max_num = -1
    for oid in _walkable_oids():
        num = _parse_ground_num(rs.GetUserText(oid, config.KEY_WALKABLE_GROUND_ID))
        if num > max_num:
            max_num = num
    return f"{config.WALKABLE_GROUND_ID_PREFIX}{max_num + 1}"


def ensure_ground_id(brep_oid) -> str:
    """Ensure *brep_oid* carries a stable WalkableGround id; assign one if missing.

    Args:
        brep_oid: a Rhino object id on the WalkableGround layer.

    Returns:
        str: the ground id (e.g. ``"WG0"``).
    """
    existing = rs.GetUserText(brep_oid, config.KEY_WALKABLE_GROUND_ID)
    if existing:
        return existing
    new_id = _next_ground_id()
    rs.SetUserText(brep_oid, config.KEY_WALKABLE_GROUND_ID, new_id)
    return new_id


def get_all_walkable_grounds() -> Dict[str, object]:
    """Return ``{ground_id: oid}`` for every WalkableGround brep, assigning ids.

    Every brep on the layer is given an id if it lacks one, so the returned map
    is complete and stable.

    Returns:
        dict[str, object]: ground id -> Rhino object id.
    """
    out = {}
    for oid in _walkable_oids():
        out[ensure_ground_id(oid)] = oid
    return out


# ---------------------------------------------------------------------------
# Brep -> COMPAS mesh (world mm), for WalkableGround.json
# ---------------------------------------------------------------------------


def brep_to_compas_mesh(brep_oid) -> Mesh:
    """Mesh a WalkableGround brep and return it as a COMPAS ``Mesh`` in world mm.

    The brep is meshed with coarse settings (a flat ground needs no fine mesh),
    every face-mesh joined into one, and vertices scaled from doc units to mm so
    the headless base sampler (which works in mm) can snap to it directly.

    Args:
        brep_oid: a Rhino object id on the WalkableGround layer.

    Returns:
        compas.datastructures.Mesh: the ground mesh, vertices in world mm.

    Raises:
        RuntimeError: if the brep cannot be meshed.
    """
    brep = as_brep(brep_oid)
    meshing = Rhino.Geometry.MeshingParameters.Coarse
    face_meshes = Rhino.Geometry.Mesh.CreateFromBrep(brep, meshing)
    if not face_meshes:
        raise RuntimeError(f"Could not mesh WalkableGround brep {brep_oid}.")
    joined = Rhino.Geometry.Mesh()
    for m in face_meshes:
        if m is not None:
            joined.Append(m)

    scale_to_mm = doc_unit_scale_to_mm()
    vertices = [
        (float(v.X) * scale_to_mm, float(v.Y) * scale_to_mm, float(v.Z) * scale_to_mm)
        for v in joined.Vertices
    ]
    faces = []
    for f in joined.Faces:
        # Rhino stores every face as a quad (A,B,C,D); a triangle repeats D == C.
        if f.C == f.D:
            faces.append([f.A, f.B, f.C])
        else:
            faces.append([f.A, f.B, f.C, f.D])
    return Mesh.from_vertices_and_faces(vertices, faces)


# ---------------------------------------------------------------------------
# Distance heuristic: bar centerline -> ground surface
# ---------------------------------------------------------------------------


def closest_dist_bar_to_ground_mm(bar_oid, brep) -> float:
    """Return the min distance (mm) from the bar centerline to *brep*.

    The bar curve is sampled at a handful of points; each is projected onto the
    brep with ``Brep.ClosestPoint`` and the smallest distance is kept. Distances
    are returned in mm so they can be compared against
    ``config.WALKABLE_ASSOC_MAX_DIST_MM``.

    Args:
        bar_oid: the bar centerline curve id.
        brep (Rhino.Geometry.Brep): a WalkableGround brep.

    Returns:
        float: the minimum bar-to-ground distance in mm (``inf`` if no sample
        could be projected).
    """
    # ~11 points along the curve is plenty to catch the nearest span of a bar.
    sample_pts = rs.DivideCurve(bar_oid, 10) or []
    scale_to_mm = doc_unit_scale_to_mm()
    best = float("inf")
    for pt in sample_pts:
        close_pt = brep.ClosestPoint(Rhino.Geometry.Point3d(pt))
        if close_pt is None:
            continue
        dist_mm = close_pt.DistanceTo(Rhino.Geometry.Point3d(pt)) * scale_to_mm
        if dist_mm < best:
            best = dist_mm
    return best


def associate_bar_by_distance(bar_oid, grounds: Dict[str, object]) -> List[str]:
    """Auto-pick which ground ids a bar should sample its base on, by distance.

    Ranks every ground by ``closest_dist_bar_to_ground_mm`` and keeps those within
    ``config.WALKABLE_ASSOC_MAX_DIST_MM``, capped at
    ``config.WALKABLE_ASSOC_MAX_COUNT``. The single nearest ground is always kept
    even if it sits beyond the cutoff, so every bar gets at least one ground.

    Args:
        bar_oid: the bar centerline curve id.
        grounds (dict[str, object]): ``{ground_id: oid}`` (from
            :func:`get_all_walkable_grounds`).

    Returns:
        list[str]: the chosen ground ids, nearest first (empty only if there are
        no grounds at all).
    """
    if not grounds:
        return []
    ranked = []
    for gid, oid in grounds.items():
        try:
            brep = as_brep(oid)
        except RuntimeError:
            continue
        ranked.append((closest_dist_bar_to_ground_mm(bar_oid, brep), gid))
    if not ranked:
        return []
    ranked.sort(key=lambda pair: pair[0])

    max_dist = float(config.WALKABLE_ASSOC_MAX_DIST_MM)
    max_count = int(config.WALKABLE_ASSOC_MAX_COUNT)
    chosen = [gid for dist, gid in ranked if dist <= max_dist][:max_count]
    if not chosen:
        # Nothing within the cutoff -> keep just the nearest so the bar still
        # has somewhere to stand.
        chosen = [ranked[0][1]]
    return chosen


# ---------------------------------------------------------------------------
# Bar <-> ground-id list (user-text on the bar curve)
# ---------------------------------------------------------------------------


def get_bar_ground_ids(bar_oid) -> List[str]:
    """Return the bar's associated ground-id list from user-text (may be empty).

    Args:
        bar_oid: the bar centerline curve id.

    Returns:
        list[str]: the stored ground ids (e.g. ``["WG0", "WG1"]``).
    """
    raw = rs.GetUserText(bar_oid, config.KEY_BAR_WALKABLE_GROUND_IDS)
    if not raw:
        return []
    return [tok.strip() for tok in raw.split(",") if tok.strip()]


def set_bar_ground_ids(bar_oid, ground_ids: List[str]) -> None:
    """Write the bar's associated ground-id list to user-text (comma-separated).

    Args:
        bar_oid: the bar centerline curve id.
        ground_ids (list[str]): the ground ids to store (order preserved).

    Returns:
        None.
    """
    rs.SetUserText(bar_oid, config.KEY_BAR_WALKABLE_GROUND_IDS, ",".join(ground_ids))


# ---------------------------------------------------------------------------
# Batch auto-assign (shared by RSRebuildRobotCell + RSExportAllBarActions)
# ---------------------------------------------------------------------------


def auto_assign_walkable_ground_ids_all_bars(grounds: Dict[str, object] = None):
    """Auto-link every un-associated bar to its nearest WalkableGround(s).

    Non-destructive: a bar that already carries an association (from a previous
    auto-assign or from RSAssignAndShowWalkableGround) is left untouched, so manual
    picks always survive. Only bars with an empty association get the nearest
    ground(s) by distance. This is called both by RSRebuildRobotCell (so a rebuild
    fills them in) and by RSExportAllBarActions (so the export is self-sufficient
    even if a rebuild was skipped).

    Args:
        grounds (dict[str, object] | None): ``{ground_id: oid}`` to assign from;
            when None it is scanned via :func:`get_all_walkable_grounds` (which
            also stamps ids on every brep).

    Returns:
        tuple: ``(n_grounds, n_assigned, n_kept, n_noground)`` -- ground surfaces
        available, bars newly assigned, bars whose existing pick was kept, and
        bars left with none (only when there is no ground at all).
    """
    # Lazy import to avoid any import-order coupling with the bar registry.
    from core.rhino_bar_registry import get_bar_seq_map

    if grounds is None:
        grounds = get_all_walkable_grounds()
    n_assigned = n_kept = n_noground = 0
    for _bar_id, (oid, _seq) in get_bar_seq_map().items():
        if get_bar_ground_ids(oid):
            n_kept += 1
            continue
        ids = associate_bar_by_distance(oid, grounds)
        if ids:
            set_bar_ground_ids(oid, ids)
            n_assigned += 1
        else:
            n_noground += 1
    return len(grounds), n_assigned, n_kept, n_noground


# ---------------------------------------------------------------------------
# Default mobile-base placement (shared heuristic seed)
# ---------------------------------------------------------------------------
#
# The base-placement heuristic itself is Rhino-free (the tamp
# ``keyframe.walkable_ground`` module, imported above as ``_walkable_np``).
# These wrappers gather its Rhino-side inputs for one bar -- the assigned ground
# soups, the bar center, and the average male-joint insertion direction -- so
# BOTH RSAssignAndShowWalkableGround (the interactive preview) and
# RSRebuildRobotCell (the batch populate) produce the exact same base frame.


def _bar_center_mm(bar_oid):
    """Return the bar centerline midpoint (mm), or ``None`` if unreadable."""
    start = rs.CurveStartPoint(bar_oid)
    end = rs.CurveEndPoint(bar_oid)
    if start is None or end is None:
        return None
    scale = doc_unit_scale_to_mm()
    start_mm = np.array([start.X, start.Y, start.Z], dtype=float) * scale
    end_mm = np.array([end.X, end.Y, end.Z], dtype=float) * scale
    return 0.5 * (start_mm + end_mm)


# ---------------------------------------------------------------------------
# Base heading: which SIDE of the bar the mobile base stands on
#
# The heading is the direction the base +X faces; the base stands the standoff
# distance AGAINST it, so the heading alone decides the side -- and, through
# ``core.rhino_tool_place.assign_tool_sides_from_heading``, which end of the bar
# gets the LEFT arm's tool.
#
# The heading comes from the bar's anchor joints: each joint block's local +Z is
# its insertion axis (retreat is -Z -- see
# ``core.bar_action._retreat_tool0_target_mm``). Naively averaging those axes
# degenerates in three ways, all of which used to put the base on an arbitrary
# (often opposite) side:
#
#   a) GROUND joints were not scanned at all, even though assembly IK accepts
#      them as arm anchors (``rs_ik_keyframe._males_on_bar`` scans both layers).
#   b) A male half's local +Z is -bar_X spun about the bar axis by its `jr`, i.e.
#      HORIZONTAL and PERPENDICULAR to the bar. Two anchors ~180 deg apart in
#      `jr` (one coupler grabbing from each side) therefore cancel, and the sum
#      is floating-point noise -- accepted as long as its norm cleared 1e-9.
#   c) A `jr` that tips an axis near-vertical leaves a horizontal component that
#      is also noise once flattened onto the ground.
#
# ``resolve_bar_heading`` below fixes all three: it scans both joint layers,
# flattens each axis onto the ground BEFORE averaging and discards the ones with
# no usable azimuth, and detects cancellation via the mean resultant length. When
# the axes genuinely disagree it falls back to the bar-perpendicular direction
# and picks whichever of the two sides has more clear ground to stand on.
# ---------------------------------------------------------------------------

# How far a candidate standoff point may miss the walkable ground before that
# side is rejected outright. Slack for a base straddling the ground edge.
_OFF_GROUND_TOL_MM = 50.0


def anchor_insertion_axes_mm(bar_id):
    """Return ``[(joint_oid, unit_axis)]`` for every anchor joint on the bar.

    Scans BOTH ``LAYER_JOINT_MALE_INSTANCES`` and ``LAYER_JOINT_GROUND_INSTANCES``,
    mirroring ``rs_ik_keyframe._males_on_bar``: assembly IK treats any tool-bearing
    joint instance on the bar as an arm anchor, so a bar held by one male + one
    ground (or two grounds) must contribute both axes to the heading.

    Args:
        bar_id (str): the bar whose anchor joints to read.

    Returns:
        list: ``[(oid, np.ndarray unit +Z), ...]``; empty when none are readable.
    """
    # Lazy import: keep this Rhino module import-light and avoid import cycles.
    from core import env_collision

    axes = []
    for layer in (config.LAYER_JOINT_MALE_INSTANCES,
                  config.LAYER_JOINT_GROUND_INSTANCES):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            if rs.GetUserText(oid, "parent_bar_id") != bar_id:
                continue
            try:
                frame_mm = np.asarray(env_collision._block_instance_xform_mm(oid), dtype=float)
            except Exception:
                continue
            z_axis = frame_mm[:3, 2]
            norm = float(np.linalg.norm(z_axis))
            if norm > 1e-9:
                axes.append((oid, z_axis / norm))
    return axes


def _bar_axis_mm(bar_oid):
    """Return the bar centerline's unit direction (mm space), or ``None``."""
    start = rs.CurveStartPoint(bar_oid)
    end = rs.CurveEndPoint(bar_oid)
    if start is None or end is None:
        return None
    scale = doc_unit_scale_to_mm()
    vec = (np.array([end.X, end.Y, end.Z], dtype=float)
           - np.array([start.X, start.Y, start.Z], dtype=float)) * scale
    norm = float(np.linalg.norm(vec))
    return vec / norm if norm > 1e-9 else None


def _other_bar_segments_mm(exclude_bar_oid):
    """Return ``[(start_mm, end_mm)]`` for every registered bar except one."""
    from core.rhino_bar_registry import get_bar_seq_map  # lazy: avoid import cycle

    scale = doc_unit_scale_to_mm()
    segments = []
    for _bar_id, (oid, _seq) in get_bar_seq_map().items():
        if str(oid) == str(exclude_bar_oid):
            continue
        start = rs.CurveStartPoint(oid)
        end = rs.CurveEndPoint(oid)
        if start is None or end is None:
            continue
        segments.append((
            np.array([start.X, start.Y, start.Z], dtype=float) * scale,
            np.array([end.X, end.Y, end.Z], dtype=float) * scale,
        ))
    return segments


def _environment_boxes_mm():
    """Return ``[(min_corner_mm, max_corner_mm)]`` for the environment obstacles."""
    if not rs.IsLayer(config.LAYER_ENVIRONMENT):
        return []
    scale = doc_unit_scale_to_mm()
    boxes = []
    for oid in rs.ObjectsByLayer(config.LAYER_ENVIRONMENT) or []:
        corners = rs.BoundingBox(oid)
        if not corners:
            continue
        pts = np.array([[p.X, p.Y, p.Z] for p in corners], dtype=float) * scale
        boxes.append((pts.min(axis=0), pts.max(axis=0)))
    return boxes


def _clearance_mm(point_mm, segments, boxes):
    """Min distance (mm) from a point to any other bar segment / obstacle box."""
    # Lazy import of the shared point-to-segment math (one implementation, also
    # used by the relink ranker).
    from core.joint_relink import _point_segment_distance

    best = float("inf")
    p = np.asarray(point_mm, dtype=float)
    for start, end in segments:
        best = min(best, _point_segment_distance(p, start, end))
    for lo, hi in boxes:
        # Distance to an axis-aligned box: zero inside, else the norm of the
        # per-axis overshoot.
        delta = np.maximum(np.maximum(lo - p, p - hi), 0.0)
        best = min(best, float(np.linalg.norm(delta)))
    return best


def _pick_open_side(soups, bar_oid, ground_point_mm, ground_normal, standoff_mm):
    """Choose between the two bar-perpendicular headings by open ground.

    Used when the anchor axes cancel, so "behind the bar" is undefined. The base
    stands AGAINST the heading, so each candidate is scored at the point it would
    actually occupy: a candidate whose standoff point misses the walkable ground
    is rejected, and of the survivors the one with the most clearance from other
    bars and environment obstacles wins.

    Args:
        soups (list): the bar's ground triangle soups.
        bar_oid: the bar centerline curve id.
        ground_point_mm (np.ndarray): the reference point's ground projection.
        ground_normal (np.ndarray): the ground normal there.
        standoff_mm (float): how far behind the heading the base will stand.

    Returns:
        tuple: ``(heading | None, detail_str)``.
    """
    from core import base_guide_geom  # noqa: PLC0415  (pure numpy, no Rhino)

    bar_axis = _bar_axis_mm(bar_oid)
    if bar_axis is None:
        return None, "bar axis unreadable"
    candidates = base_guide_geom.perpendicular_candidates(bar_axis, ground_normal)
    if candidates is None:
        return None, "bar is perpendicular to the ground (no side to pick)"

    segments = _other_bar_segments_mm(bar_oid)
    boxes = _environment_boxes_mm()
    reasons = {}

    def _score(heading):
        """How open is the point the base would occupy for this heading?

        ``None`` = the base would be off the walkable ground (reject the side).
        """
        stand_pt = np.asarray(ground_point_mm, dtype=float) - heading * float(standoff_mm)
        snapped, _n = _walkable_np.closest_point_on_meshes(soups, stand_pt)
        key = id(heading)
        if snapped is None:
            reasons[key] = "no ground"
            return None
        off_ground = float(np.linalg.norm(np.asarray(snapped, dtype=float) - stand_pt))
        if off_ground > _OFF_GROUND_TOL_MM:
            reasons[key] = f"off ground by {off_ground:.0f}mm"
            return None
        clearance = _clearance_mm(stand_pt, segments, boxes)
        reasons[key] = f"{clearance:.0f}mm clear"
        return clearance

    heading, _scored = base_guide_geom.pick_best_candidate(candidates, _score)
    detail = "; ".join(
        f"{'+n' if i == 0 else '-n'}: {reasons.get(id(c), 'unscored')}"
        for i, c in enumerate(candidates)
    )
    return heading, detail


def resolve_bar_heading(bar_oid, bar_id, soups, ground_point_mm, ground_normal,
                        standoff_mm, flip: bool = False, verbose: bool = True) -> dict:
    """Resolve the base heading for a bar, diagnosing degenerate cases.

    Args:
        bar_oid: the bar centerline curve id.
        bar_id (str): the bar id (used to find its anchor joints).
        soups (list): the bar's ground triangle soups.
        ground_point_mm (np.ndarray): the reference point's ground projection.
        ground_normal (np.ndarray): the ground normal there.
        standoff_mm (float): the standoff the base will use (scores the fallback).
        flip (bool): return the mirrored heading (the RSIKKeyframeAll Flip option).
        verbose (bool): print the per-bar diagnosis.

    Returns:
        dict: ``{"heading": unit np.ndarray, "source": "averaged" |
        "perpendicular-open" | "world-fallback", "ambiguous": bool,
        "axes": [(oid, unit_axis)], "detail": str, "flipped": bool}``.
    """
    from core import base_guide_geom  # noqa: PLC0415  (pure numpy, no Rhino)

    normal = np.asarray(ground_normal, dtype=float)
    axes = anchor_insertion_axes_mm(bar_id)

    # The averaging + degeneracy test is pure math and lives Rhino-free so it can
    # be unit-tested; this function's job is gathering its inputs from the doc.
    verdict = base_guide_geom.resolve_heading_from_axes(
        [axis for _oid, axis in axes], normal
    )
    heading = verdict["heading"]
    ambiguous = verdict["ambiguous"]
    n_vertical = verdict["n_vertical"]
    detail = verdict["detail"]
    source = None if heading is None else "averaged"

    if heading is None:
        heading, side_detail = _pick_open_side(
            soups, bar_oid, ground_point_mm, normal, standoff_mm
        )
        if heading is not None:
            source = "perpendicular-open"
            detail = f"{detail}; open-side fallback [{side_detail}]"
        else:
            # Last resort: the pre-existing world-horizontal fallback.
            heading = _walkable_np._ground_heading_direction(None, normal)
            source = "world-fallback"
            detail = f"{detail}; open-side unavailable [{side_detail}] -> world horizontal"

    if flip:
        heading = -np.asarray(heading, dtype=float)

    if verbose:
        print(f"core.rhino_walkable_ground: heading for bar '{bar_id}' -> {source}"
              f"{' (FLIPPED)' if flip else ''}")
        for oid, axis in axes:
            print(f"    axis {oid}: ({axis[0]:+.2f}, {axis[1]:+.2f}, {axis[2]:+.2f})")
        if n_vertical:
            print(f"    {n_vertical} axis/axes discarded as near-vertical "
                  f"(< {float(config.INSERTION_DIR_MIN_HORIZONTAL):.2f} horizontal)")
        print(f"    {detail}")
        if ambiguous:
            print(f"    AMBIGUOUS: verify which side of bar '{bar_id}' the base stands on.")

    return {
        "heading": np.asarray(heading, dtype=float),
        "source": source,
        "ambiguous": ambiguous,
        "axes": axes,
        "detail": detail,
        "flipped": bool(flip),
    }


def _bar_ground_soups(bar_oid, grounds_map, soup_cache: dict = None):
    """Numpy form of the WalkableGround surface(s) this bar may stand a base on.

    A "triangle soup" is ``(vertices, triangles)``: two numpy arrays holding the
    corner points and the index-triples of a tessellated ground surface, with the
    mesh adjacency dropped -- an unstructured pile of triangles, hence the name.
    The chain is ``Rhino Brep -> Rhino Mesh -> COMPAS Mesh -> soup``, and it
    exists so the base-placement math stays Rhino-free (headless + pytest); its
    only consumer, ``closest_point_on_meshes``, loops over every triangle and so
    needs no connectivity. ``docs/Su_note.md`` section 14 has the long form.

    One soup per ground, because a bar may be assigned up to
    ``config.WALKABLE_ASSOC_MAX_COUNT`` (2) of them.

    Args:
        bar_oid: the bar centerline curve id.
        grounds_map (dict): ``{ground_id: oid}``.
        soup_cache (dict | None): when given, memoizes ``{ground_id: soup}``
            across calls. Tessellating the brep is the expensive step, so any
            caller looping over many bars (the multi-bar IK command, the
            tool-side repair pass) should pass one shared dict.

    Returns:
        list: the triangle soups, in the bar's ground-id order (empty when the
        bar has no assigned or meshable ground).
    """
    soups = []
    for gid in get_bar_ground_ids(bar_oid):
        oid = grounds_map.get(gid)
        if oid is None:
            continue
        if soup_cache is not None and gid in soup_cache:
            if soup_cache[gid] is not None:
                soups.append(soup_cache[gid])
            continue
        try:
            soup = _walkable_np._mesh_to_soup(brep_to_compas_mesh(oid))
        except Exception:
            soup = None
        if soup_cache is not None:
            soup_cache[gid] = soup
        if soup is not None:
            soups.append(soup)
    return soups


def default_base_frame_for_bar(bar_oid, bar_id, grounds_map: Dict[str, object] = None,
                               standoff_mm: float = None,
                               center_mm=None,
                               flip: bool = False,
                               diag_out: dict = None,
                               soup_cache: dict = None,
                               verbose: bool = False) -> Optional[np.ndarray]:
    """Compute the heuristic seed mobile-base frame (4x4 mm) for a bar, or ``None``.

    Projects the reference point onto the bar's assigned WalkableGround(s), steps
    the standoff distance AGAINST the resolved heading, and re-snaps that point
    onto the ground -- the shared ``keyframe.walkable_ground.derive_seed_base``
    heuristic. The base therefore faces along the heading, so driving forward
    (+X) carries the bar into the assembly. Returns ``None`` when the bar has no
    assigned / meshable ground or its curve is unreadable.

    The heading comes from :func:`resolve_bar_heading`, which averages the bar's
    anchor-joint insertion axes and falls back to the open side of the bar when
    they cancel -- see that function for why the naive average was not enough.

    Args:
        bar_oid: the bar centerline curve id.
        bar_id (str): the bar id (used to find its anchor joints).
        grounds_map (dict | None): ``{ground_id: oid}``; scanned via
            :func:`get_all_walkable_grounds` when ``None``.
        standoff_mm (float | None): how far to stand behind the bar, in mm.
            ``None`` -> ``config.IK_BASE_STANDOFF_MM`` (the default single-bar seed).
            The multi-bar IK command (RSIKKeyframeAll) passes
            ``config.IK_BASE_STANDOFF_MULTIBAR_MM`` (500 mm, the middle base-guide
            line).
        center_mm (np.ndarray | None): the ground-projection reference point (mm)
            the base stands behind of. ``None`` -> the bar centerline midpoint
            (:func:`_bar_center_mm`). The multi-bar IK command passes the grabbed-
            joint center (midpoint of the two tool-bearing joints) instead.
        flip (bool): stand on the opposite side (mirrored heading). Drives the
            Flip option in RSIKKeyframeAll.
        diag_out (dict | None): when given, updated in place with the
            :func:`resolve_bar_heading` diagnosis plus ``"ground_point"`` (the
            reference point's ground projection) and ``"off_guide_mm"`` (how far
            the final origin sits from the ideal extension-line point -- non-zero
            means ``derive_seed_base`` fell back to standing under the bar because
            the offset point missed the ground).
        soup_cache (dict | None): shared ``{ground_id: soup}`` memo, forwarded to
            :func:`_bar_ground_soups`. Pass one dict when looping over many bars.
        verbose (bool): print the per-bar heading diagnosis.

    Returns:
        np.ndarray | None: the 4x4 mm base frame, or ``None``.
    """
    if grounds_map is None:
        grounds_map = get_all_walkable_grounds()
    soups = _bar_ground_soups(bar_oid, grounds_map, soup_cache=soup_cache)
    if not soups:
        return None
    if center_mm is None:
        center_mm = _bar_center_mm(bar_oid)
    if center_mm is None:
        return None

    standoff = float(config.IK_BASE_STANDOFF_MM if standoff_mm is None else standoff_mm)
    ground_point, ground_normal = _walkable_np.closest_point_on_meshes(soups, center_mm)
    if ground_point is None:
        return None

    diag = resolve_bar_heading(
        bar_oid, bar_id, soups, ground_point, ground_normal, standoff,
        flip=flip, verbose=verbose,
    )
    if diag_out is not None:
        diag_out.update(diag)
        diag_out["ground_point"] = np.asarray(ground_point, dtype=float)

    try:
        origin, normal, heading_dir = _walkable_np.derive_seed_base(
            soups, center_mm, heading_dir_mm=diag["heading"], standoff_mm=standoff,
        )
        frame = _walkable_np.frame_from_origin_normal_heading(
            origin, normal, origin + heading_dir * 1000.0
        )
    except Exception:
        return None

    if diag_out is not None:
        # derive_seed_base silently stands UNDER the bar when the offset point
        # misses the ground; surface that instead of letting the base drift off
        # the guide line unannounced.
        ideal = np.asarray(ground_point, dtype=float) - diag["heading"] * standoff
        diag_out["off_guide_mm"] = float(np.linalg.norm(origin - ideal))
    return frame


def auto_populate_base_frames_all_bars(grounds: Dict[str, object] = None,
                                       overwrite: bool = False):
    """Write a heuristic seed base frame on bars that lack one (non-destructive).

    For every bar without a saved ``KEY_ASSEMBLY_BASE_FRAME`` (unless
    ``overwrite``), compute the default base via :func:`default_base_frame_for_bar`
    and store it as JSON user-text, so the exported BarAction + every movement's
    start_state (and a headless ``--base saved`` solve) start from a real base
    instead of a placeholder. Bars that ALREADY carry a base -- a hand-picked one
    (RSIKKeyframe / RSAssignAndShowWalkableGround) or one the IK solve wrote -- are
    KEPT as-is: a human-chosen base always takes priority over the auto seed. This
    mirrors the non-destructive WalkableGround auto-assign. Called by
    RSRebuildRobotCell.

    Args:
        grounds (dict | None): ``{ground_id: oid}``; scanned when ``None``.
        overwrite (bool): when True, recompute even bars that already have a base.

    Returns:
        tuple: ``(n_populated, n_kept, n_failed)`` -- bars newly given a base,
        bars whose existing base was kept, and bars skipped (no meshable ground /
        unreadable curve).
    """
    from core.rhino_bar_registry import get_bar_seq_map

    if grounds is None:
        grounds = get_all_walkable_grounds()
    n_populated = n_kept = n_failed = 0
    for bar_id, (oid, _seq) in get_bar_seq_map().items():
        if not overwrite and rs.GetUserText(oid, config.KEY_ASSEMBLY_BASE_FRAME):
            n_kept += 1
            continue
        frame_mm = default_base_frame_for_bar(oid, bar_id, grounds)
        if frame_mm is None:
            n_failed += 1
            continue
        rs.SetUserText(
            oid, config.KEY_ASSEMBLY_BASE_FRAME,
            json.dumps(np.asarray(frame_mm, dtype=float).tolist()),
        )
        n_populated += 1
    return n_populated, n_kept, n_failed


# ---------------------------------------------------------------------------
# Rebuild summary (shared by RSRebuildRobotCell + RSPBStart)
# ---------------------------------------------------------------------------


def rebuild_summary_after_assign(collision_bodies, tool_names):
    """Run the WalkableGround auto-assign + seed-base passes and format the summary.

    Both RSRebuildRobotCell and RSPBStart call this right after
    ``robot_cell.rebuild_assembly_cell`` so their pop-up (and console echo) read
    exactly the same. It runs the two NON-DESTRUCTIVE passes
    (:func:`auto_assign_walkable_ground_ids_all_bars` +
    :func:`auto_populate_base_frames_all_bars`) and turns their counts, together
    with the registered-body and arm-tool counts, into ready-to-show text.

    Args:
        collision_bodies (dict): ``{name: body_info}`` returned by
            ``robot_cell.rebuild_assembly_cell`` (read only for its ``kind`` tags).
        tool_names (list[str]): the arm-tool ids on the cell,
            i.e. ``sorted(rcell.tool_models.keys())`` -- e.g. ``["AT3L", "AT3R"]``.

    Returns:
        tuple[str, str]: ``(popup_msg, console_line)`` -- the multi-line body for
        ``rs.MessageBox`` and a one-line console echo. Neither is prefixed with a
        command name, so each caller can prepend its own (``RSRebuildRobotCell:`` /
        ``RSPBStart:``).
    """
    # ---- The two NON-DESTRUCTIVE passes the rebuild button also runs: link every
    # ---- un-associated bar to its nearest ground, then seed a base pose on any
    # ---- bar that still lacks one (hand-picked / IK-solved bases are kept).
    n_grounds, n_assigned, n_kept, n_noground = auto_assign_walkable_ground_ids_all_bars()
    n_base_pop, n_base_kept, n_base_fail = auto_populate_base_frames_all_bars()

    # ---- Count the registered collision bodies by kind.
    n_bar = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "bar")
    n_joint = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "joint")
    n_env = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "environment")
    n_tools = len(tool_names)

    # WalkableGround line: only warn about un-grounded bars when grounds exist.
    if n_grounds == 0:
        ground_line = "  WalkableGround: none on layer (skipped auto-assign)"
    else:
        ground_line = (
            f"  WalkableGround: {n_grounds} surface(s); "
            f"{n_assigned} bar(s) auto-assigned, {n_kept} kept, {n_noground} still none"
        )
    base_line = (
        f"  Base pose: {n_base_pop} bar(s) auto-populated, {n_base_kept} kept, "
        f"{n_base_fail} skipped (no ground)"
    )
    popup_msg = (
        f"Rebuilt the robot cell:\n"
        f"  {n_bar} bar(s)\n"
        f"  {n_joint} joint half/halves\n"
        f"  {n_env} environment obstacle(s)\n"
        f"  {n_tools} arm tool(s): {', '.join(sorted(tool_names)) or '-'}\n"
        f"{ground_line}\n"
        f"{base_line}"
    )
    console_line = (
        f"{n_bar} bars, {n_joint} joints, {n_env} obstacles, "
        f"{n_tools} tools; WalkableGround {n_grounds} surf / {n_assigned} assigned / "
        f"{n_kept} kept / {n_noground} none; Base pose {n_base_pop} populated / "
        f"{n_base_kept} kept / {n_base_fail} skipped."
    )
    return popup_msg, console_line
