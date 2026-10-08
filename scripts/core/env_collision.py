"""Env-collision wiring for assembly IK.

Built bars and their joints (sequence < active step) are registered as
``RobotCell.rigid_body_models`` so compas_fab CC.3/CC.5 evaluate them.

One naming scheme for every cell (Cindy's dual-arm cell AND the Alice/Belle
support cells), so a consumer can key on a body name without knowing which
cell it came from:

- ``bar_<bar_id>``                   bar tubes
- ``joint_<joint_id>_<subtype>``     joint halves (male / female / ground)
- ``obstacle_<name>``                static obstacles (``LAYER_ENVIRONMENT``)
- ``ground_<ground_id>``             floor slabs (the walkable grounds the bars use)

The static bodies (obstacles + floors) are collected once and shared by every
cell (:func:`collect_static_scene_geometry`). A floor body carries its own
always-allowed contacts (the robots' wheels, the frozen robots) in its
``body_info``; the state builders copy them onto every state.

Fast path (cached lightweight RigidBodies)
------------------------------------------
- Joints: each joint half declares ``collision_filename`` (e.g.
  ``T20_Female.obj``) in ``core/joint_pairs.json``. The OBJ is loaded
  once per ``block_name`` into a ``RigidBody`` (``native_scale=0.001``
  since the OBJ is in mm) and shared across all placements of that
  block. Sticky cache: ``bar_joint:env_joint_rb_cache``.
- Bars: a 12-faceted cylinder mesh is built procedurally in METERS in
  the bar's local frame (Z = bar axis, origin at bar start), wrapped in
  a ``RigidBody(native_scale=1.0)``. ``frame_world_mm`` carries the
  bar's local-to-world transform. Sticky cache: ``bar_joint:env_bar_rb_cache``
  keyed by bar oid + (length_mm, radius_mm) signature; rebuilt on
  signature mismatch (bar moved/resized).

Reusing the same ``RigidBody`` instance under multiple
``robot_cell.rigid_body_models`` names is safe -- compas_fab's PyBullet
backend creates a separate PB body per name (see
``pybullet_set_robot_cell.py`` + ``client._add_rigid_body``).

Rhino-only helpers (``collect_built_geometry``) import ``rhinoscriptsyntax``
lazily inside the function body so this module remains importable headless.
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

from core import config
from core import joint_name_conventions as jnc


# * ---- Rigid-body names: ONE scheme for every cell ----
# State-independent ("canonical") names. Cindy's cell and the support cells
# use exactly the same ones (they used to differ: `env_bar_*` in the support
# cells, a leftover of an older design). The prefixes are spelled in
# core.joint_name_conventions, the home of every naming rule.
CANONICAL_BAR_PREFIX = jnc.BAR_KEY_PREFIX
CANONICAL_JOINT_PREFIX = jnc.JOINT_KEY_PREFIX
# Static environment obstacle meshes (LAYER_ENVIRONMENT). Distinct namespace
# so it never collides with bar_/joint_ names.
OBSTACLE_PREFIX = jnc.OBSTACLE_KEY_PREFIX
# Floor slabs, one per walkable ground some bar uses (LAYER_WALKABLE_GROUND).
# Not "obstacle_": an object named "ground" on the environment layer already
# becomes `obstacle_ground`.
FLOOR_PREFIX = "ground_"
# Every body name this module manages (used to drop stale bodies from a cell).
MANAGED_BODY_PREFIXES = (CANONICAL_BAR_PREFIX, CANONICAL_JOINT_PREFIX, OBSTACLE_PREFIX, FLOOR_PREFIX)
# The old support-cell names. Nothing creates them any more; a support cell
# cached earlier in the same Rhino session may still hold them, so the stale
# scans drop them too.
_LEGACY_BODY_PREFIXES = (jnc.ENV_BAR_KEY_PREFIX, jnc.ENV_JOINT_KEY_PREFIX)

# * ---- Body kinds (``body_info["kind"]``) ----
KIND_BAR = "bar"
KIND_JOINT = "joint"
KIND_ENVIRONMENT = "environment"
KIND_FLOOR = "floor"
# Bodies that exist in every scene, whatever the assembly step: always shown.
STATIC_KINDS = (KIND_ENVIRONMENT, KIND_FLOOR)

# Sticky cache keys for the lightweight RigidBody pipeline.
_STICKY_JOINT_RB_CACHE = "bar_joint:env_joint_rb_cache"  # block_name -> RigidBody
_STICKY_BAR_RB_CACHE = "bar_joint:env_bar_rb_cache"      # bar_oid_str -> (signature, RigidBody)
_STICKY_JOINT_OBJ_PATH_MAP = "bar_joint:env_joint_obj_path_map"  # block_name -> abs OBJ path
# The static bodies (obstacles + floors), shared by every cell (see
# collect_static_scene_geometry). Refreshed by RSRebuildRobotCell.
STICKY_STATIC_BODIES = "bar_joint:static_scene_bodies"

# Bar cylinder discretization (12-sided regular polygon, no length subdivision).
BAR_CYLINDER_SIDES = 12


# ---------------------------------------------------------------------------
# * Rigid-body names
# ---------------------------------------------------------------------------


def bar_body_name(bar_id: str) -> str:
    """The rigid-body name of a bar tube, e.g. ``"bar_B3"``."""
    return f"{CANONICAL_BAR_PREFIX}{bar_id}"


def joint_body_name(joint_id: str, subtype: str) -> str:
    """The rigid-body name of a joint half, e.g. ``"joint_J1-3_male"``.

    Same result as ``jnc.joint_key``, but also takes the lower-case role
    (``"female"``) that the touch-policy code passes around.

    Args:
        joint_id (str): the joint id (``"J1-3"``, ``"G1-T20-0"``).
        subtype (str): ``"Male"``, ``"Female"``, ``"Ground"`` or ``"MoCap"``
            (any case).
    """
    return f"{CANONICAL_JOINT_PREFIX}{joint_id}_{subtype.lower()}"


def floor_body_name(ground_id: str) -> str:
    """The rigid-body name of a floor slab, e.g. ``"ground_WG0"``."""
    return f"{FLOOR_PREFIX}{ground_id}"


# ---------------------------------------------------------------------------
# Rhino-side geometry collection
# ---------------------------------------------------------------------------


def _sticky_dict():
    try:
        import scriptcontext as sc
        return sc.sticky
    except ImportError:
        # Fallback for headless tests; persists for module lifetime.
        global _FALLBACK_STICKY
        try:
            return _FALLBACK_STICKY
        except NameError:
            _FALLBACK_STICKY = {}
            return _FALLBACK_STICKY


def _joint_rb_cache():
    sticky = _sticky_dict()
    cache = sticky.get(_STICKY_JOINT_RB_CACHE)
    if cache is None:
        cache = {}
        sticky[_STICKY_JOINT_RB_CACHE] = cache
    return cache


def _bar_rb_cache():
    sticky = _sticky_dict()
    cache = sticky.get(_STICKY_BAR_RB_CACHE)
    if cache is None:
        cache = {}
        sticky[_STICKY_BAR_RB_CACHE] = cache
    return cache


def _joint_obj_path_map():
    """Block_name -> abs OBJ path, derived from joint_pairs.json. Cached in sticky.

    Multiple joint halves can share the same block_name (e.g. T20_Male appears
    in three pair definitions); the map deduplicates on block_name.
    """
    sticky = _sticky_dict()
    cached = sticky.get(_STICKY_JOINT_OBJ_PATH_MAP)
    if cached is not None:
        return cached
    from core.joint_pair import DEFAULT_ASSET_DIR, load_joint_registry
    out: dict = {}
    registry = load_joint_registry()
    for half in registry.halves.values():
        if half.collision_filename and half.block_name not in out:
            out[half.block_name] = half.collision_path(DEFAULT_ASSET_DIR)
    for ground in registry.ground_joints.values():
        if ground.collision_filename and ground.block_name not in out:
            out[ground.block_name] = ground.collision_path(DEFAULT_ASSET_DIR)
    sticky[_STICKY_JOINT_OBJ_PATH_MAP] = out
    return out


def clear_joint_obj_path_cache() -> None:
    """Forget the ``block_name -> OBJ`` map and every joint ``RigidBody`` built
    from it, so the next collision build re-reads ``joint_pairs.json``.

    Both live in ``sc.sticky`` for the whole Rhino session, and nothing else
    ever drops them.  Call after anything that adds or changes a joint's
    collision OBJ (RSDefineJointHalf) or swaps which block a joint uses.

    Clearing the map alone is not enough: a block looked up BEFORE its OBJ was
    registered is cached in the RigidBody cache as ``None`` ("missing -- skip
    this joint"), and would keep being skipped from collision until Rhino
    restarted.  Every joint OBJ reloads on the next build (a few ms each).
    """
    sticky = _sticky_dict()
    sticky.pop(_STICKY_JOINT_OBJ_PATH_MAP, None)
    sticky.pop(_STICKY_JOINT_RB_CACHE, None)


def _build_bar_cylinder_mesh(length_m: float, radius_m: float, sides: int = BAR_CYLINDER_SIDES):
    """Build a low-poly compas Mesh of a cylinder along +Z, base at origin.

    Returns a fresh ``compas.datastructures.Mesh`` (METERS). ``sides`` controls
    the polygon resolution; the side wall is a single segment along Z (no
    subdivision needed -- PyBullet handles long thin triangles fine).
    Caps are fans from a center vertex.
    """
    from compas.datastructures import Mesh as CMesh

    n = int(sides)
    vertices = []
    # Bottom ring (0..n-1), top ring (n..2n-1), bottom center (2n), top center (2n+1).
    for k in range(n):
        theta = (2.0 * math.pi * k) / n
        x = radius_m * math.cos(theta)
        y = radius_m * math.sin(theta)
        vertices.append((x, y, 0.0))
    for k in range(n):
        theta = (2.0 * math.pi * k) / n
        x = radius_m * math.cos(theta)
        y = radius_m * math.sin(theta)
        vertices.append((x, y, length_m))
    bot_center = len(vertices); vertices.append((0.0, 0.0, 0.0))
    top_center = len(vertices); vertices.append((0.0, 0.0, length_m))

    faces = []
    for k in range(n):
        k1 = (k + 1) % n
        # Side as a quad (CCW seen from outside).
        faces.append([k, k1, n + k1, n + k])
        # Bottom cap fan (winding so normal points -Z).
        faces.append([bot_center, k1, k])
        # Top cap fan (normal +Z).
        faces.append([top_center, n + k, n + k1])
    return CMesh.from_vertices_and_faces(vertices, faces)


def _bar_world_frame_mm(bar_oid):
    """Return (length_mm, frame_world_mm) for a bar curve.

    Frame: origin = bar_start (mm), Z = unit(end-start), X = orthogonal_to(Z),
    Y = Z x X. Same convention as ``core.joint_pair.canonical_bar_frame_from_line``
    so the in-Rhino tube preview and the local-frame cylinder mesh align.
    """
    import rhinoscriptsyntax as rs
    from core.rhino_helpers import doc_unit_scale_to_mm
    from core.transforms import frame_from_axes, orthogonal_to, unit

    s = doc_unit_scale_to_mm()
    start = rs.CurveStartPoint(bar_oid)
    end = rs.CurveEndPoint(bar_oid)
    p0 = np.array([float(start.X) * s, float(start.Y) * s, float(start.Z) * s], dtype=float)
    p1 = np.array([float(end.X) * s, float(end.Y) * s, float(end.Z) * s], dtype=float)
    axis = p1 - p0
    length_mm = float(np.linalg.norm(axis))
    if length_mm < 1e-6:
        return 0.0, np.eye(4, dtype=float)
    z_axis = axis / length_mm
    x_axis = orthogonal_to(z_axis)
    y_axis = unit(np.cross(z_axis, x_axis))
    return length_mm, frame_from_axes(p0, x_axis, y_axis, z_axis)


def _get_or_load_joint_rigid_body(block_name, deps):
    """Return a cached ``RigidBody`` for ``block_name`` (loaded once per OBJ).

    The same ``RigidBody`` instance is shared across all joint placements of
    the same block definition (and across all robot_cell.rigid_body_models
    keys that point to that joint type) -- compas_fab's PyBullet backend
    creates a separate PB body per name regardless.
    """
    cache = _joint_rb_cache()
    cached = cache.get(block_name)
    if cached is not None:
        return cached, True  # (rb, hit)
    path_map = _joint_obj_path_map()
    obj_path = path_map.get(block_name, "")
    if not obj_path or not os.path.isfile(obj_path):
        print(
            f"core.env_collision: joint OBJ for block '{block_name}' missing "
            f"(expected {obj_path!r}); env collision will skip this joint."
        )
        cache[block_name] = None
        return None, False
    Mesh = deps["Mesh"]
    RigidBody = deps["RigidBody"]
    t0 = time.perf_counter()
    mesh = Mesh.from_obj(obj_path)
    # OBJ exported in mm (matches the joint .3dm assets); native_scale 0.001 -> meters.
    rb = RigidBody(visual_meshes=[mesh], collision_meshes=[mesh], native_scale=0.001)
    print(
        f"core.env_collision: cold-load joint RB '{block_name}' from {os.path.basename(obj_path)} "
        f"({mesh.number_of_vertices()}v/{mesh.number_of_faces()}f) in "
        f"{(time.perf_counter()-t0)*1000:.1f} ms"
    )
    cache[block_name] = rb
    return rb, False


def _get_or_build_bar_rigid_body(bar_oid, length_mm, radius_mm, deps):
    """Return a cached ``RigidBody`` for a bar tube; rebuild on signature mismatch.

    Cache key = ``str(bar_oid)``; signature = ``(round(length_mm,3), round(radius_mm,3))``.
    A different bar with the same signature still gets its own cache entry --
    cheap, and lets us notice geometry changes per-bar.
    """
    import rhinoscriptsyntax as rs
    cache = _bar_rb_cache()
    key = str(rs.coerceguid(bar_oid))
    sig = (round(float(length_mm), 3), round(float(radius_mm), 3))
    entry = cache.get(key)
    if entry is not None and entry[0] == sig:
        return entry[1], True
    RigidBody = deps["RigidBody"]
    t0 = time.perf_counter()
    mesh = _build_bar_cylinder_mesh(length_m=length_mm / 1000.0, radius_m=radius_mm / 1000.0)
    rb = RigidBody(visual_meshes=[mesh], collision_meshes=[mesh], native_scale=1.0)
    print(
        f"core.env_collision: built bar RB oid={key[:8]} L={length_mm:.1f}mm R={radius_mm:.1f}mm "
        f"({mesh.number_of_vertices()}v/{mesh.number_of_faces()}f) in "
        f"{(time.perf_counter()-t0)*1000:.1f} ms"
    )
    cache[key] = (sig, rb)
    return rb, False


def _raise_on_duplicate_joint_key(out: dict, key: str, joint_oid, collector: str) -> None:
    """Refuse to build a collision scene when two joint blocks claim one name.

    Canonical body names come from the ``joint_id`` USER TEXT + the layer. Two
    blocks carrying the same id (the classic Rhino copy-paste, which clones
    user text) therefore compute the SAME key, and a plain dict assignment
    would silently drop one of them -- its geometry then exists in no collision
    scene at all, so IK and the release checks happily approve poses that pass
    straight through it. That is a wrong-answer failure mode, so stop instead.

    Args:
        out (dict): the collector's output so far.
        key (str): the canonical body name just computed.
        joint_oid: the Rhino object id of the block being added.
        collector (str): calling collector name, for the message.

    Raises:
        RuntimeError: when ``key`` is already taken by a different block.
    """
    import rhinoscriptsyntax as rs

    if key not in out:
        return
    first_oid = out[key].get("source_oid")
    if str(first_oid) == str(joint_oid):
        return  # same object seen twice (layer listed twice) -- harmless
    first_name = rs.ObjectName(first_oid) or str(first_oid)
    second_name = rs.ObjectName(joint_oid) or str(joint_oid)
    raise RuntimeError(
        f"core.env_collision.{collector}: TWO joint blocks map to the same "
        f"collision body '{key}' -- '{first_name}' and '{second_name}' share the "
        "same joint_id user text (typically a copy-pasted block that cloned it). "
        "One of them would be dropped from EVERY collision scene, so IK and the "
        "release checks would approve poses that pass straight through it. "
        "Repair the ids first: run RSUpdatePreview to list every duplicate, then "
        "RSReorderBarID -> Relink to re-derive ids from geometry (review its "
        "plan before applying), then RSRebuildRobotCell."
    )


def collect_built_geometry(active_bar_id, bar_seq_map, include_active=False, exclude_bar_ids=None,
                           all_geom=None):
    """The bars + joints built before a given step (a filter over the full set).

    Same names and payloads as :func:`collect_assembly_geometry` (canonical
    ``bar_<id>`` / ``joint_<jid>_<sub>`` keys, each with ``parent_bar_id``), so
    the support cells and Cindy's cell share one naming scheme. Keeps a bar or
    joint when its parent bar's step is before ``active_bar_id``'s (or equal,
    with ``include_active``). Static bodies (obstacles, floors) are never in the
    result: they are not "built" by any step and are added separately.

    Fake bars and their joint halves are already left out by
    :func:`collect_assembly_geometry`.

    Args:
        active_bar_id (str): the step whose scene is being built.
        bar_seq_map (dict): a ``get_bar_seq_map`` result.
        include_active (bool): also include the active bar + its joints --
            used for AFTER-the-step scenes (e.g. the support robot's
            release-time check runs after the last stabilizing bar is built).
        exclude_bar_ids (list): bar ids to leave out regardless.
        all_geom (dict): a :func:`collect_assembly_geometry` (or
            ``get_env_union``) result to filter; collected when omitted.

    Returns:
        dict: ``{name: body_info}`` for the built bars + joints.
    """
    if active_bar_id not in bar_seq_map:
        return {}
    if all_geom is None:
        all_geom = collect_assembly_geometry(bar_seq_map)
    active_seq = bar_seq_map[active_bar_id][1]
    excluded = set(exclude_bar_ids or [])
    out = {}
    for name, body_info in all_geom.items():
        parent = body_info.get("parent_bar_id")
        # Static bodies have no parent bar; bars of an unknown / excluded
        # parent are not part of this scene.
        if parent is None or parent not in bar_seq_map or parent in excluded:
            continue
        seq = bar_seq_map[parent][1]
        if seq < active_seq or (include_active and seq == active_seq):
            out[name] = body_info
    return out


def collect_assembly_geometry(bar_seq_map):
    """Collect canonical-keyed collision bodies for ALL bars + joints.

    Canonical-keyed collector used by the static-cell pipeline (via
    ``robot_cell.rebuild_assembly_cell``). Keys are canonical (``bar_<bid>`` /
    ``joint_<jid>_<subtype>``) -- no active_/env_ prefixes -- and each
    ``body_info`` carries ``parent_bar_id`` so the state builder can classify
    built / active / future by assembly sequence.

    FAKE bars and the joint halves mounted on them are left out entirely: a
    fake bar is a modeling artifact that only poses a real bar's male half, so
    nothing physical stands there to collide with. The real bar's male half is
    parented to the REAL bar and is unaffected. The fake marks are read from
    the whole document, so a ``bar_seq_map`` with the fake bars already
    filtered out works too (their joint halves are skipped quietly instead of
    being reported as orphans).

    Args:
        bar_seq_map (dict): ``{bar_id: (oid, seq)}`` for the registered bars
            (with or without the fake ones).

    Returns:
        dict: ``{name: body_info}`` where ``body_info`` is
        ``{rigid_body, frame_world_mm, kind, source_oid, parent_bar_id, ...}``.
    """
    import rhinoscriptsyntax as rs
    from core.rhino_bar_registry import get_fake_bar_ids
    from core.rhino_helpers import block_instance_xform_mm

    deps = _import_deps_for_rb()
    t_total = time.perf_counter()
    out = {}
    # * Fake bars are modeling artifacts that only pose a real bar's male half;
    # nothing physical stands there, so neither the tube nor the joint halves
    # mounted on it belong in any collision scene. Read from the whole document
    # (not just `bar_seq_map`), so a caller passing a fake-free map gets no
    # false "orphan joint" notes for the halves mounted on fake bars.
    fake_bar_ids = get_fake_bar_ids()
    bar_hits = bar_misses = 0
    for bid, (oid, _seq) in bar_seq_map.items():
        if bid in fake_bar_ids:
            continue
        length_mm, frame_mm = _bar_world_frame_mm(oid)
        if length_mm <= 0.0:
            continue
        rb, hit = _get_or_build_bar_rigid_body(oid, length_mm, float(config.BAR_RADIUS), deps)
        if rb is None:
            continue
        bar_hits += int(hit); bar_misses += int(not hit)
        out[jnc.bar_key(bid)] = {
            "rigid_body": rb,
            "frame_world_mm": frame_mm,
            "kind": KIND_BAR,
            "source_oid": oid,
            "parent_bar_id": bid,
        }

    # Every joint role belongs in a collision scene -- the full set, not a subset.
    joint_layers = jnc.JOINT_LAYERS
    j_hits = j_misses = 0
    # Blocks whose parent bar is unreadable / not a live bar: invisible to every
    # collision scene, so report them rather than dropping them silently.
    orphan_parents = []
    for layer in joint_layers:
        if not rs.IsLayer(layer):
            continue
        for joint_oid in rs.ObjectsByLayer(layer) or []:
            parent_bar = rs.GetUserText(joint_oid, jnc.UT_PARENT_BAR)
            # A half mounted on a fake bar (its female) goes out with the bar;
            # the real bar's male is parented to the REAL bar and stays.
            if parent_bar in fake_bar_ids:
                continue
            if parent_bar not in bar_seq_map:
                orphan_parents.append(
                    f"{rs.ObjectName(joint_oid) or joint_oid}"
                    f"(parent={parent_bar or '<none>'})"
                )
                continue
            joint_id = rs.GetUserText(joint_oid, jnc.UT_JOINT_ID)
            subtype = jnc.subtype_of_layer(layer)  # the layer is the authority
            block_name = rs.BlockInstanceName(joint_oid)
            if not block_name:
                continue
            rb, hit = _get_or_load_joint_rigid_body(block_name, deps)
            if rb is None:
                continue
            j_hits += int(hit); j_misses += int(not hit)
            xform_mm = block_instance_xform_mm(joint_oid)
            key = jnc.joint_key(joint_id or str(joint_oid), subtype)
            _raise_on_duplicate_joint_key(out, key, joint_oid, "collect_assembly_geometry")
            out[key] = {
                "rigid_body": rb,
                "frame_world_mm": xform_mm,
                "kind": KIND_JOINT,
                "source_oid": joint_oid,
                "block_name": block_name,
                "subtype": subtype,
                "parent_bar_id": parent_bar,
            }
    if fake_bar_ids:
        print(
            f"core.env_collision.collect_assembly_geometry: excluded "
            f"{len(fake_bar_ids)} fake bar(s) + their joint halves from the "
            f"collision scene: {', '.join(sorted(fake_bar_ids))}"
        )
    if orphan_parents:
        print(
            f"core.env_collision.collect_assembly_geometry: NOTE - "
            f"{len(orphan_parents)} joint block(s) skipped because parent_bar_id "
            f"is not a live bar: {', '.join(orphan_parents[:8])}"
            + (" ..." if len(orphan_parents) > 8 else "")
            + " -- they are absent from every collision scene; repair with "
            "RSUpdatePreview / RSReorderBarID -> Relink."
        )
    print(
        f"core.env_collision.collect_assembly_geometry: {len(out)} bodies "
        f"(bars hit/miss={bar_hits}/{bar_misses}, joints hit/miss={j_hits}/{j_misses}) "
        f"in {(time.perf_counter()-t_total)*1000:.1f} ms"
    )
    return out


def _sanitize_obstacle_name(name) -> str:
    """Make a Rhino object name safe to use as a rigid-body key suffix.

    Args:
        name: the raw object name (any type; coerced to str).

    Returns:
        str: ``name`` with non-alphanumeric chars (except ``-``/``_``) replaced
        by ``_``; ``"env"`` if the result is empty.
    """
    cleaned = "".join(
        ch if (ch.isalnum() or ch in "-_") else "_" for ch in str(name).strip()
    )
    return cleaned or "env"


def _coerce_env_brep(oid):
    """Return a ``Rhino.Geometry.Brep`` for a brep/surface/polysurface/extrusion
    object, or ``None`` if *oid* is not brep-like.

    Mirrors ``core.rhino_walkable_ground.as_brep``: ``rs.coercebrep`` handles
    breps/surfaces directly, and closed Extrusion primitives (Rhino's native box)
    are converted via ``Extrusion.ToBrep`` (``rs.coercebrep`` returns ``None`` for
    those).
    """
    import Rhino
    import rhinoscriptsyntax as rs

    brep = rs.coercebrep(oid)
    if brep is not None:
        return brep
    rhobj = rs.coercerhinoobject(oid, True, True)
    geom = getattr(rhobj, "Geometry", None)
    if isinstance(geom, Rhino.Geometry.Extrusion):
        return geom.ToBrep(False)
    return None


def _env_object_to_compas_mesh(oid, scale_to_m, Mesh):
    """Return a COMPAS ``Mesh`` (vertices in METERS, world coords) for an env
    object, or ``None`` if it is neither a mesh nor a meshable brep/extrusion.

    Native meshes are read directly; brep / surface / polysurface / (closed)
    extrusion objects are meshed with coarse settings (same as WalkableGround)
    and the per-face meshes joined into one. Both paths yield Rhino's quad face
    convention (triangles repeat the last index), collapsed to tris/quads for
    ``Mesh.from_vertices_and_faces``.
    """
    import Rhino
    import rhinoscriptsyntax as rs

    if rs.IsMesh(oid):
        verts = rs.MeshVertices(oid)
        faces = rs.MeshFaceVertices(oid)
    else:
        brep = _coerce_env_brep(oid)
        if brep is None:
            return None
        face_meshes = Rhino.Geometry.Mesh.CreateFromBrep(
            brep, Rhino.Geometry.MeshingParameters.Coarse
        )
        if not face_meshes:
            return None
        joined = Rhino.Geometry.Mesh()
        for m in face_meshes:
            if m is not None:
                joined.Append(m)
        verts = [(v.X, v.Y, v.Z) for v in joined.Vertices]
        faces = [(f.A, f.B, f.C, f.D) for f in joined.Faces]
    if not verts or not faces:
        return None
    cverts = [
        (float(p[0]) * scale_to_m, float(p[1]) * scale_to_m, float(p[2]) * scale_to_m)
        for p in verts
    ]
    cfaces = []
    for f in faces:
        a, b, c, d = f
        cfaces.append([a, b, c] if c == d else [a, b, c, d])
    return Mesh.from_vertices_and_faces(cverts, cfaces)


def collect_environment_geometry():
    """Collect static obstacle bodies from ``config.LAYER_ENVIRONMENT``.

    Every mesh, brep, surface, polysurface or (closed) extrusion on that layer
    becomes a static ``obstacle_<name>`` rigid body -- breps/extrusions are
    meshed on the fly (coarse settings). Geometry is already in world
    coordinates (scaled doc-units -> m), so ``frame_world_mm`` is identity.
    Objects that are neither a mesh nor a meshable brep are skipped with a
    warning.

    Returns:
        dict: ``{name: body_info}`` with ``kind:"environment"`` -- the same
        shape as :func:`collect_assembly_geometry`, so the two dicts merge
        directly.
    """
    import rhinoscriptsyntax as rs
    from core.rhino_helpers import doc_unit_scale_to_mm

    deps = _import_deps_for_rb()
    Mesh = deps["Mesh"]
    RigidBody = deps["RigidBody"]

    if not rs.IsLayer(config.LAYER_ENVIRONMENT):
        return {}
    scale_to_m = doc_unit_scale_to_mm() / 1000.0

    out = {}
    used = set()
    n_skipped = 0
    for i, oid in enumerate(rs.ObjectsByLayer(config.LAYER_ENVIRONMENT) or []):
        mesh = _env_object_to_compas_mesh(oid, scale_to_m, Mesh)
        if mesh is None:
            n_skipped += 1
            print(
                f"core.env_collision.collect_environment_geometry: object {oid} on "
                f"{config.LAYER_ENVIRONMENT!r} is not a mesh/brep/extrusion (or could "
                f"not be meshed); skipping."
            )
            continue
        rb = RigidBody(visual_meshes=[mesh], collision_meshes=[mesh], native_scale=1.0)
        name = _sanitize_obstacle_name(rs.ObjectName(oid) or f"env{i}")
        base, k = name, 1
        while name in used:
            name = f"{base}_{k}"
            k += 1
        used.add(name)
        out[jnc.obstacle_key(name)] = {
            "rigid_body": rb,
            "frame_world_mm": np.eye(4, dtype=float),
            "kind": KIND_ENVIRONMENT,
            "source_oid": oid,
        }
    print(
        f"core.env_collision.collect_environment_geometry: {len(out)} obstacle(s) "
        f"from {config.LAYER_ENVIRONMENT!r}"
        + (f" ({n_skipped} skipped)" if n_skipped else "")
    )
    return out


# ---------------------------------------------------------------------------
# * Floor slabs (the walkable grounds the bars use)
# ---------------------------------------------------------------------------


def _polygon_normal(points: np.ndarray) -> np.ndarray:
    """Newell's normal of one polygon (not unit length; zero when degenerate).

    Args:
        points (np.ndarray): ``(n, 3)`` polygon corners in order.

    Returns:
        np.ndarray: the summed cross products (length = twice the area).
    """
    normal = np.zeros(3, dtype=float)
    for i in range(len(points)):
        normal += np.cross(points[i], points[(i + 1) % len(points)])
    return normal


def floor_slab_mesh_data(vertices, faces, thickness_m: float,
                         flat_tol_deg: float = 2.0, max_tilt_deg: float = 45.0) -> tuple:
    """Turn a flat walkable-ground surface mesh into a closed slab under it.

    A single surface has no thickness, and PyBullet loads every collision mesh
    as its convex hull, so a bare face would be a degenerate (flat) solid. The
    slab copies the surface downward by ``thickness_m`` and closes the sides,
    so the top face stays exactly where the robots stand. Rhino-free.

    Args:
        vertices (list): surface vertices, metres, world coordinates.
        faces (list): surface faces (lists of vertex indices).
        thickness_m (float): slab thickness, metres.
        flat_tol_deg (float): largest allowed angle between face normals.
        max_tilt_deg (float): largest allowed angle between the surface normal
            and world up (a wall is not a floor).

    Returns:
        tuple: ``(slab_vertices, slab_faces, up_normal)`` -- the slab mesh
        (metres) and the surface's unit normal, pointing up.

    Raises:
        ValueError: when the surface has no area, is not flat (a stepped
            ground must be split into flat pieces), or is too steep.
    """
    points = np.asarray(vertices, dtype=float)
    world_up = np.array([0.0, 0.0, 1.0])
    normals, areas = [], []
    for face in faces:
        raw = _polygon_normal(points[list(face)])
        size = float(np.linalg.norm(raw))
        if size < 1e-12:
            continue  # zero-area face: no direction to compare
        unit = raw / size
        # Rhino face winding can differ face by face: compare upward-facing normals.
        normals.append(unit if unit @ world_up >= 0.0 else -unit)
        areas.append(size)
    if not normals:
        raise ValueError("the surface has no area")
    mean = np.sum([a * n for a, n in zip(areas, normals)], axis=0)
    mean /= np.linalg.norm(mean)
    worst_deg = max(
        float(np.degrees(np.arccos(np.clip(n @ mean, -1.0, 1.0)))) for n in normals
    )
    if worst_deg > flat_tol_deg:
        raise ValueError(
            f"the surface is not flat (its faces differ by {worst_deg:.1f} deg); "
            "split it into flat pieces"
        )
    tilt_deg = float(np.degrees(np.arccos(np.clip(mean @ world_up, -1.0, 1.0))))
    if tilt_deg > max_tilt_deg:
        raise ValueError(
            f"the surface is tilted {tilt_deg:.0f} deg from horizontal (more than "
            f"{max_tilt_deg:.0f}); a wall is not a floor"
        )

    # * Top = the surface itself; bottom = the same points moved down.
    n_top = len(points)
    bottom = points - float(thickness_m) * mean
    slab_vertices = [list(map(float, p)) for p in points] + [list(map(float, p)) for p in bottom]
    slab_faces = [list(face) for face in faces]
    slab_faces += [[i + n_top for i in reversed(face)] for face in faces]
    # * Side walls along the surface's outline (edges used by exactly one face).
    edge_uses = {}
    for face in faces:
        for i in range(len(face)):
            a, b = face[i], face[(i + 1) % len(face)]
            edge_uses.setdefault((min(a, b), max(a, b)), []).append((a, b))
    for uses in edge_uses.values():
        if len(uses) == 1:
            a, b = uses[0]
            slab_faces.append([b, a, a + n_top, b + n_top])
    return slab_vertices, slab_faces, mean


def used_walkable_ground_ids() -> list:
    """The walkable grounds some real (not fake) bar is assigned to.

    Runs the non-destructive auto-assign first (bars without a ground get the
    nearest one; existing picks are kept), so a freshly built model gets its
    floor on the first rebuild.

    Returns:
        list: sorted ground ids, e.g. ``["WG0"]``.
    """
    from core.rhino_bar_registry import get_bar_seq_map, get_fake_bar_ids
    from core.rhino_walkable_ground import (
        auto_assign_walkable_ground_ids_all_bars,
        get_all_walkable_grounds,
        get_bar_ground_ids,
    )

    grounds = get_all_walkable_grounds()
    if grounds:
        auto_assign_walkable_ground_ids_all_bars(grounds)
    bar_map = get_bar_seq_map()
    fake_bar_ids = get_fake_bar_ids(bar_map)
    return sorted({
        ground_id
        for bar_id, (oid, _seq) in bar_map.items()
        if bar_id not in fake_bar_ids
        for ground_id in get_bar_ground_ids(oid)
    })


def collect_floor_geometry() -> dict:
    """One floor slab body per walkable ground the bars actually use.

    Only the grounds named by some real bar's ``walkable_ground_ids`` become
    floors (a vertical "ground" used as a reference, or an unused one, stays
    out of the collision scene). Each floor carries its always-allowed
    contacts in its ``body_info``: the four wheel links of whichever robot owns
    the cell (``touch_links``) and every frozen-robot obstacle
    (``touch_bodies``). The state builders copy them onto every state.

    Returns:
        dict: ``{ground_<id>: body_info}`` with ``kind == KIND_FLOOR``.

    Raises:
        RuntimeError: when a bar names a ground that does not exist, or a used
            ground is not a flat, roughly horizontal surface.
    """
    import rhinoscriptsyntax as rs
    from core.rhino_helpers import doc_unit_scale_to_mm
    from core.rhino_walkable_ground import get_all_walkable_grounds

    deps = _import_deps_for_rb()
    Mesh = deps["Mesh"]
    RigidBody = deps["RigidBody"]
    scale_to_m = doc_unit_scale_to_mm() / 1000.0

    grounds = get_all_walkable_grounds()
    used = used_walkable_ground_ids()
    unused = sorted(set(grounds) - set(used))
    if not used:
        print(
            "core.env_collision.collect_floor_geometry: NOTE - no bar is assigned to "
            "a walkable ground, so the collision scene has NO floor. Assign grounds "
            "(RSAssignAndShowWalkableGround) and rebuild the cell."
        )
        return {}

    out = {}
    for ground_id in used:
        oid = grounds.get(ground_id)
        if oid is None:
            raise RuntimeError(
                f"Some bar is assigned to walkable ground '{ground_id}', but no surface "
                f"with that id is on {config.LAYER_WALKABLE_GROUND!r}. Re-assign the "
                "bars' grounds (RSAssignAndShowWalkableGround)."
            )
        surface = _env_object_to_compas_mesh(oid, scale_to_m, Mesh)
        if surface is None:
            raise RuntimeError(f"Walkable ground '{ground_id}' could not be meshed.")
        vertices, faces = surface.to_vertices_and_faces()
        try:
            slab_vertices, slab_faces, _up = floor_slab_mesh_data(
                vertices, faces, float(config.FLOOR_SLAB_THICKNESS_MM) / 1000.0,
            )
        except ValueError as exc:
            raise RuntimeError(
                f"Walkable ground '{ground_id}' ({rs.ObjectName(oid) or oid}) cannot be "
                f"a floor body: {exc}."
            ) from exc
        slab = Mesh.from_vertices_and_faces(slab_vertices, slab_faces)
        out[floor_body_name(ground_id)] = {
            "rigid_body": RigidBody(visual_meshes=[slab], collision_meshes=[slab], native_scale=1.0),
            # The slab is built in world coordinates (metres).
            "frame_world_mm": np.eye(4, dtype=float),
            "kind": KIND_FLOOR,
            "source_oid": oid,
            "ground_id": ground_id,
            # Always allowed: the owning robot's wheels stand on it, and the
            # frozen robots (unattached tools) stand on it too.
            "touch_links": list(config.FLOOR_TOUCH_LINKS),
            "touch_bodies": sorted(config.OBSTACLE_TOOL_NAMES.values()),
        }
    print(
        f"core.env_collision.collect_floor_geometry: floor(s) {sorted(out)}"
        + (f"; not used by any bar, left out: {unused}" if unused else "")
    )
    return out


def collect_static_scene_geometry(force: bool = False) -> dict:
    """The static bodies shared by every cell: obstacles + floors.

    Collected once per session and cached, so Cindy's cell and the support
    cells register the very same ``RigidBody`` objects (no re-sending of a
    cell when nothing changed). ``robot_cell.rebuild_assembly_cell``
    (RSRebuildRobotCell) refreshes it with ``force=True``.

    Args:
        force (bool): re-read the document even when a cached copy exists.

    Returns:
        dict: ``{name: body_info}`` for every ``obstacle_*`` and ``ground_*`` body.
    """
    sticky = _sticky_dict()
    cached = sticky.get(STICKY_STATIC_BODIES)
    if cached is not None and not force:
        return cached
    out = dict(collect_environment_geometry())
    out.update(collect_floor_geometry())
    sticky[STICKY_STATIC_BODIES] = out
    return out


def _import_deps_for_rb():
    """Lazy-import only what the cached RB pipeline needs (Mesh + RigidBody)."""
    from compas.datastructures import Mesh as _Mesh
    from compas_fab.robots import RigidBody as _RB
    return {"Mesh": _Mesh, "RigidBody": _RB}


# ---------------------------------------------------------------------------
# RobotCell / state wiring
# ---------------------------------------------------------------------------


def register_env_in_robot_cell(robot_cell, env_geom, *, deps):
    """Mirror cached env ``RigidBody`` instances into ``robot_cell.rigid_body_models``.

    Safe to call repeatedly on object identity: if the cell already holds the
    exact same ``RigidBody`` instance under ``name``, we skip. New names get
    added; managed names (``MANAGED_BODY_PREFIXES``, plus the old ``env_*``
    names a cell cached earlier in the Rhino session may still carry) that are
    no longer wanted get removed. (Support-cell path.)

    Args:
        robot_cell (RobotCell): cell whose ``rigid_body_models`` are updated
            in place.
        env_geom (dict): ``{name: payload}`` where ``payload["rigid_body"]`` is
            the cached ``RigidBody`` (e.g. a ``get_env_union`` result).
        deps (dict): the lazily-imported compas stack (unused here; kept for
            call-site symmetry).

    Returns:
        bool: ``True`` if anything changed (caller may need to re-push the cell).
    """
    t0 = time.perf_counter()
    changed = False

    desired_names = set(env_geom.keys())
    existing_env_names = {
        name for name in robot_cell.rigid_body_models.keys()
        if name.startswith(MANAGED_BODY_PREFIXES + _LEGACY_BODY_PREFIXES)
    }
    n_removed = n_added = n_kept = 0
    for stale in existing_env_names - desired_names:
        robot_cell.rigid_body_models.pop(stale, None)
        changed = True
        n_removed += 1

    for name, payload in env_geom.items():
        rb = payload["rigid_body"]
        existing = robot_cell.rigid_body_models.get(name)
        if existing is rb:
            n_kept += 1
            continue
        robot_cell.rigid_body_models[name] = rb
        changed = True
        n_added += 1
    print(
        f"core.env_collision.register_env_in_robot_cell: "
        f"added={n_added} removed={n_removed} kept={n_kept} "
        f"in {(time.perf_counter()-t0)*1000:.1f} ms"
    )
    return changed


def build_env_state(template_state, env_geom):
    """Return a copy of ``template_state`` with ``rigid_body_states`` populated.

    Each env body is a static body at its world frame: ``frame`` (METERS) set
    from the mm-based ``frame_world_mm`` payload, unattached, shown. Static
    bodies that carry always-allowed contacts in their payload (the floors:
    ``touch_links`` / ``touch_bodies``) get them copied onto the state.
    (Support-cell path.)

    Args:
        template_state (RobotCellState): base state to copy.
        env_geom (dict): ``{name: payload}`` with ``payload["frame_world_mm"]``
            world poses (e.g. a ``get_env_union`` result).

    Returns:
        RobotCellState: a copy of ``template_state`` with the managed bodies
        re-populated.
    """
    from compas_fab.robots import RigidBodyState
    from compas.geometry import Frame

    state = template_state.copy()
    # Keep state workpieces aligned with robot_cell.rigid_body_models: drop
    # every prior managed entry before writing the current env payload.
    stale_env_names = [
        name for name in state.rigid_body_states
        if name.startswith(MANAGED_BODY_PREFIXES + _LEGACY_BODY_PREFIXES)
    ]
    for name in stale_env_names:
        state.rigid_body_states.pop(name, None)

    if not env_geom:
        return state

    for name, payload in env_geom.items():
        m_mm = np.asarray(payload["frame_world_mm"], dtype=float)
        origin_m = m_mm[:3, 3] / 1000.0
        frame = Frame(
            list(map(float, origin_m)),
            list(map(float, m_mm[:3, 0])),
            list(map(float, m_mm[:3, 1])),
        )
        state.rigid_body_states[name] = RigidBodyState(
            frame=frame,
            attached_to_link=None,
            attached_to_tool=None,
            touch_links=list(payload.get("touch_links") or []),
            touch_bodies=list(payload.get("touch_bodies") or []),
            attachment_frame=None,
            is_hidden=False,
        )
    return state


def list_env_summary(env_geom) -> str:
    """One line describing a scene's bodies (bars, joints by block, static bodies).

    Args:
        env_geom (dict): ``{name: body_info}``.

    Returns:
        str: e.g. ``"6 built bars, 14 joints (T20_Male: 6, ...), 1 static body"``.
    """
    if not env_geom:
        return "0 built bars, 0 joints"
    bars = [v for v in env_geom.values() if v.get("kind") == KIND_BAR]
    joints = [v for v in env_geom.values() if v.get("kind") == KIND_JOINT]
    n_static = sum(1 for v in env_geom.values() if v.get("kind") in STATIC_KINDS)
    by_block: dict = {}
    for j in joints:
        bn = j.get("block_name", "?")
        by_block[bn] = by_block.get(bn, 0) + 1
    detail = ""
    if by_block:
        parts = ", ".join(f"{k}: {v}" for k, v in sorted(by_block.items()))
        detail = f" ({parts})"
    return f"{len(bars)} built bars, {len(joints)} joints{detail}, {n_static} static body(ies)"


# NOTE: the verbose pair-count summary (`summarize_check_collision`) moved to
# `husky_assembly_tamp.keyframe.dual_arm_ik` with the solvers -- import it from
# there.
