"""Env-collision wiring for assembly IK.

Built bars and their joints (sequence < active step) are registered as
``RobotCell.rigid_body_models`` so compas_fab CC.3/CC.5 evaluate them.

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


# env_* prefixes for the single-arm SUPPORT cell path (collect_built_geometry /
# register_env_in_robot_cell / build_env_state). The dual-arm assembly cell uses
# the canonical names below instead.
ENV_RB_BAR_PREFIX = "env_bar_"
ENV_RB_JOINT_PREFIX = "env_joint_"

# State-independent ("canonical") names used by the static-cell pipeline.
# `collect_assembly_geometry` emits these directly (no active_/env_ prefixes).
CANONICAL_BAR_PREFIX = "bar_"
CANONICAL_JOINT_PREFIX = "joint_"
# Static environment obstacle meshes (LAYER_ENVIRONMENT). Distinct namespace
# so it never collides with bar_/joint_ names.
OBSTACLE_PREFIX = "obstacle_"

# Sticky cache keys for the lightweight RigidBody pipeline.
_STICKY_JOINT_RB_CACHE = "bar_joint:env_joint_rb_cache"  # block_name -> RigidBody
_STICKY_BAR_RB_CACHE = "bar_joint:env_bar_rb_cache"      # bar_oid_str -> (signature, RigidBody)
_STICKY_JOINT_OBJ_PATH_MAP = "bar_joint:env_joint_obj_path_map"  # block_name -> abs OBJ path

# Bar cylinder discretization (12-sided regular polygon, no length subdivision).
BAR_CYLINDER_SIDES = 12


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
    from core.rhino_frame_io import doc_unit_scale_to_mm
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


def _block_instance_xform_mm(oid):
    """Read a Rhino block instance's world transform as a 4x4 matrix in mm."""
    import Rhino
    import rhinoscriptsyntax as rs
    from core.rhino_frame_io import doc_unit_scale_to_mm

    rh = rs.coercerhinoobject(oid, True, True)
    if not isinstance(rh, Rhino.DocObjects.InstanceObject):
        raise RuntimeError(f"Object {oid} is not a block instance.")
    scale = doc_unit_scale_to_mm()
    xf = rh.InstanceXform
    matrix = np.array([[float(xf[i, j]) for j in range(4)] for i in range(4)], dtype=float)
    matrix[:3, 3] *= scale
    return matrix


def collect_built_geometry(active_bar_id, bar_seq_map):
    """Walk ``bar_seq_map`` and build env-collision payloads for every bar with seq < active_seq.

    Returns ``{rb_name: {rigid_body, frame_world_mm, kind, source_oid, ...}}``:
      - bar entries: cached low-poly cylinder ``RigidBody`` (built in local
        frame), ``frame_world_mm = bar local-to-world transform``.
      - joint entries: cached ``RigidBody`` loaded from the per-block OBJ
        (``collision_filename`` in joint_pairs.json), ``frame_world_mm = block
        instance world xform``.

    The same ``RigidBody`` instance is shared across multiple env names
    when the underlying geometry is identical (joints of the same block,
    bars of the same length+radius). compas_fab's PyBullet backend creates
    a separate PB body per name, so sharing is safe.

    Skips the active bar AND its joints.
    """
    import rhinoscriptsyntax as rs

    deps = _import_deps_for_rb()

    if active_bar_id not in bar_seq_map:
        return {}
    _active_oid, active_seq = bar_seq_map[active_bar_id]

    built_bar_ids = {
        bid: oid for bid, (oid, seq) in bar_seq_map.items() if seq < active_seq
    }
    if not built_bar_ids:
        return {}

    t_total = time.perf_counter()
    out = {}
    bar_hits = bar_misses = 0
    for bid, oid in built_bar_ids.items():
        length_mm, frame_mm = _bar_world_frame_mm(oid)
        if length_mm <= 0.0:
            continue
        rb, hit = _get_or_build_bar_rigid_body(oid, length_mm, float(config.BAR_RADIUS), deps)
        if rb is None:
            continue
        bar_hits += int(hit); bar_misses += int(not hit)
        out[f"{ENV_RB_BAR_PREFIX}{bid}"] = {
            "rigid_body": rb,
            "frame_world_mm": frame_mm,
            "kind": "bar",
            "source_oid": oid,
        }

    # Joints whose parent_bar_id is a built bar.
    joint_layers = (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    )
    j_hits = j_misses = 0
    for layer in joint_layers:
        if not rs.IsLayer(layer):
            continue
        for joint_oid in rs.ObjectsByLayer(layer) or []:
            parent_bar = rs.GetUserText(joint_oid, "parent_bar_id")
            if parent_bar not in built_bar_ids:
                continue
            joint_id = rs.GetUserText(joint_oid, "joint_id")
            # Ground joints store joint_type="ground" but no joint_subtype;
            # fall back so the env tag suffix is meaningful (`_ground` not `_joint`).
            subtype = (
                rs.GetUserText(joint_oid, "joint_subtype")
                or rs.GetUserText(joint_oid, "joint_type")
                or "Joint"
            )
            block_name = rs.BlockInstanceName(joint_oid)
            if not block_name:
                continue
            rb, hit = _get_or_load_joint_rigid_body(block_name, deps)
            if rb is None:
                continue
            j_hits += int(hit); j_misses += int(not hit)
            xform_mm = _block_instance_xform_mm(joint_oid)
            tag = f"{joint_id or str(joint_oid)}_{subtype.lower()}"
            out[f"{ENV_RB_JOINT_PREFIX}{tag}"] = {
                "rigid_body": rb,
                "frame_world_mm": xform_mm,
                "kind": "joint",
                "source_oid": joint_oid,
                "block_name": block_name,
                "subtype": subtype,
            }
    print(
        f"core.env_collision.collect_built_geometry: {len(out)} bodies "
        f"(bars hit/miss={bar_hits}/{bar_misses}, joints hit/miss={j_hits}/{j_misses}) "
        f"in {(time.perf_counter()-t_total)*1000:.1f} ms"
    )
    return out



def collect_assembly_geometry(bar_seq_map):
    """Collect canonical-keyed collision bodies for ALL bars + joints.

    Canonical-keyed collector used by the static-cell pipeline (via
    ``robot_cell.rebuild_assembly_cell``). Keys are canonical (``bar_<bid>`` /
    ``joint_<jid>_<subtype>``) -- no active_/env_ prefixes -- and each
    ``body_info`` carries ``parent_bar_id`` so the state builder can classify
    built / active / future by assembly sequence.

    Args:
        bar_seq_map (dict): ``{bar_id: (oid, seq)}`` for every registered bar.

    Returns:
        dict: ``{name: body_info}`` where ``body_info`` is
        ``{rigid_body, frame_world_mm, kind, source_oid, parent_bar_id, ...}``.
    """
    import rhinoscriptsyntax as rs

    deps = _import_deps_for_rb()
    t_total = time.perf_counter()
    out = {}
    bar_hits = bar_misses = 0
    for bid, (oid, _seq) in bar_seq_map.items():
        length_mm, frame_mm = _bar_world_frame_mm(oid)
        if length_mm <= 0.0:
            continue
        rb, hit = _get_or_build_bar_rigid_body(oid, length_mm, float(config.BAR_RADIUS), deps)
        if rb is None:
            continue
        bar_hits += int(hit); bar_misses += int(not hit)
        out[f"{CANONICAL_BAR_PREFIX}{bid}"] = {
            "rigid_body": rb,
            "frame_world_mm": frame_mm,
            "kind": "bar",
            "source_oid": oid,
            "parent_bar_id": bid,
        }

    joint_layers = (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    )
    j_hits = j_misses = 0
    for layer in joint_layers:
        if not rs.IsLayer(layer):
            continue
        for joint_oid in rs.ObjectsByLayer(layer) or []:
            parent_bar = rs.GetUserText(joint_oid, "parent_bar_id")
            if parent_bar not in bar_seq_map:
                continue
            joint_id = rs.GetUserText(joint_oid, "joint_id")
            subtype = (
                rs.GetUserText(joint_oid, "joint_subtype")
                or rs.GetUserText(joint_oid, "joint_type")
                or "Joint"
            )
            block_name = rs.BlockInstanceName(joint_oid)
            if not block_name:
                continue
            rb, hit = _get_or_load_joint_rigid_body(block_name, deps)
            if rb is None:
                continue
            j_hits += int(hit); j_misses += int(not hit)
            xform_mm = _block_instance_xform_mm(joint_oid)
            tag = f"{joint_id or str(joint_oid)}_{subtype.lower()}"
            out[f"{CANONICAL_JOINT_PREFIX}{tag}"] = {
                "rigid_body": rb,
                "frame_world_mm": xform_mm,
                "kind": "joint",
                "source_oid": joint_oid,
                "block_name": block_name,
                "subtype": subtype,
                "parent_bar_id": parent_bar,
            }
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
    from core.rhino_frame_io import doc_unit_scale_to_mm

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
        out[f"{OBSTACLE_PREFIX}{name}"] = {
            "rigid_body": rb,
            "frame_world_mm": np.eye(4, dtype=float),
            "kind": "environment",
            "source_oid": oid,
        }
    print(
        f"core.env_collision.collect_environment_geometry: {len(out)} obstacle(s) "
        f"from {config.LAYER_ENVIRONMENT!r}"
        + (f" ({n_skipped} skipped)" if n_skipped else "")
    )
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
    added; ``env_*`` names no longer present get removed. (Support-cell path.)

    Args:
        robot_cell (RobotCell): cell whose ``rigid_body_models`` are updated
            in place.
        env_geom (dict): ``{name: payload}`` where ``payload["rigid_body"]`` is
            the cached ``RigidBody`` (``env_bar_*`` / ``env_joint_*`` names).
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
        if name.startswith(ENV_RB_BAR_PREFIX) or name.startswith(ENV_RB_JOINT_PREFIX)
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

    Each env body is a static obstacle: ``frame`` (METERS) set from the
    mm-based ``frame_world_mm`` payload, ``attached_to_tool=None``,
    ``attached_to_link=None``, ``is_hidden=False``. (Support-cell path.)

    Args:
        template_state (RobotCellState): base state to copy.
        env_geom (dict): ``{name: payload}`` with ``payload["frame_world_mm"]``
            world poses (``env_bar_*`` / ``env_joint_*`` names).

    Returns:
        RobotCellState: a copy of ``template_state`` with the ``env_*`` bodies
        re-populated as static obstacles.
    """
    from compas_fab.robots import RigidBodyState
    from compas.geometry import Frame

    state = template_state.copy()
    # Keep state workpieces aligned with robot_cell.rigid_body_models: drop
    # all prior env_* entries before writing the current env payload.
    stale_env_names = [
        name for name in state.rigid_body_states
        if name.startswith(ENV_RB_BAR_PREFIX) or name.startswith(ENV_RB_JOINT_PREFIX)
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
            touch_links=[],
            touch_bodies=[],
            attachment_frame=None,
            is_hidden=False,
        )
    return state


def list_env_summary(env_geom) -> str:
    if not env_geom:
        return "0 built bars, 0 joints"
    bars = [v for v in env_geom.values() if v.get("kind") == "bar"]
    joints = [v for v in env_geom.values() if v.get("kind") == "joint"]
    by_block: dict = {}
    for j in joints:
        bn = j.get("block_name", "?")
        by_block[bn] = by_block.get(bn, 0) + 1
    detail = ""
    if by_block:
        parts = ", ".join(f"{k}: {v}" for k, v in sorted(by_block.items()))
        detail = f" ({parts})"
    return f"{len(bars)} built bars, {len(joints)} joints{detail}"


# NOTE: the verbose pair-count summary (`summarize_check_collision`) moved to
# `husky_assembly_tamp.keyframe.dual_arm_ik` with the solvers -- import it from
# there.
