"""Rhino-side WalkableGround helpers: stable ids, meshing, bar association.

This is the Rhino half of the WalkableGround feature (the numpy half lives in
``core.walkable_ground``). It handles everything that needs ``rhinoscriptsyntax`` /
``Rhino``:

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
from core import walkable_ground as _walkable_np
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
# The base-placement heuristic itself is Rhino-free (``core.walkable_ground``).
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


def _avg_male_insertion_dir_mm(bar_id):
    """Average world insertion direction (+Z) over the bar's male joint blocks.

    Each male joint block's local +Z is its insertion axis (retreat is -Z, see
    ``core.bar_action._retreat_tool0_target_mm``). Returns the (unnormalized)
    average, or ``None`` if the bar has no readable male joints.

    Args:
        bar_id (str): the bar whose male joints to average.

    Returns:
        np.ndarray | None: the average insertion direction, or ``None``.
    """
    # Lazy import: keep this Rhino module import-light and avoid import cycles.
    from core import env_collision

    if not rs.IsLayer(config.LAYER_JOINT_MALE_INSTANCES):
        return None
    directions = []
    for moid in rs.ObjectsByLayer(config.LAYER_JOINT_MALE_INSTANCES) or []:
        if rs.GetUserText(moid, "parent_bar_id") != bar_id:
            continue
        try:
            frame_mm = np.asarray(env_collision._block_instance_xform_mm(moid), dtype=float)
        except Exception:
            continue
        z_axis = frame_mm[:3, 2]
        norm = float(np.linalg.norm(z_axis))
        if norm > 1e-9:
            directions.append(z_axis / norm)
    if not directions:
        return None
    avg = np.sum(directions, axis=0)
    return avg if float(np.linalg.norm(avg)) > 1e-9 else None


def _bar_ground_soups(bar_oid, grounds_map):
    """Triangle soups for the bar's assigned WalkableGround(s) (may be empty)."""
    soups = []
    for gid in get_bar_ground_ids(bar_oid):
        oid = grounds_map.get(gid)
        if oid is None:
            continue
        try:
            soups.append(_walkable_np._mesh_to_soup(brep_to_compas_mesh(oid)))
        except Exception:
            continue
    return soups


def default_base_frame_for_bar(bar_oid, bar_id, grounds_map: Dict[str, object] = None) -> Optional[np.ndarray]:
    """Compute the heuristic seed mobile-base frame (4x4 mm) for a bar, or ``None``.

    Stands a standoff behind the bar and faces along the average male-joint
    insertion direction, on the bar's assigned WalkableGround(s) -- the shared
    ``core.walkable_ground.derive_seed_base`` heuristic. Returns ``None`` when the
    bar has no assigned / meshable ground or its curve is unreadable.

    Args:
        bar_oid: the bar centerline curve id.
        bar_id (str): the bar id (used to find its male joints).
        grounds_map (dict | None): ``{ground_id: oid}``; scanned via
            :func:`get_all_walkable_grounds` when ``None``.

    Returns:
        np.ndarray | None: the 4x4 mm base frame, or ``None``.
    """
    if grounds_map is None:
        grounds_map = get_all_walkable_grounds()
    soups = _bar_ground_soups(bar_oid, grounds_map)
    if not soups:
        return None
    center_mm = _bar_center_mm(bar_oid)
    if center_mm is None:
        return None
    insertion_dir = _avg_male_insertion_dir_mm(bar_id)
    try:
        origin, normal, heading_dir = _walkable_np.derive_seed_base(
            soups, center_mm, heading_dir_mm=insertion_dir
        )
        return _walkable_np.frame_from_origin_normal_heading(
            origin, normal, origin + heading_dir * 1000.0
        )
    except Exception:
        return None


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
