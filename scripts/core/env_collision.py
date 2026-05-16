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


ENV_RB_BAR_PREFIX = "env_bar_"
ENV_RB_JOINT_PREFIX = "env_joint_"
# Active = the bar currently being assembled + its joints. Treated as static
# obstacles in world coords like env_*, but kept under separate prefixes so
# (a) downstream code can show/hide / re-attach them independently,
# (b) ShowIK / IK keyframe can decide to attach them to a tool link later.
ACTIVE_RB_BAR_PREFIX = "active_bar_"
ACTIVE_RB_JOINT_PREFIX = "active_joint_"
# Tool side the active bar will be attached to (left tool0). Currently the
# active bar is registered as a static world-frame RB; tool-following
# attachment is a follow-up (needs FK on the IK-solved state).
ACTIVE_BAR_ATTACH_LINK = "left_ur_arm_tool0"

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


def collect_active_geometry(active_bar_id, bar_seq_map):
    """Build active-bar + active-joint payloads for the bar currently being assembled.

    Same payload format as :func:`collect_built_geometry` but with
    ``active_bar_`` / ``active_joint_`` name prefixes so downstream visualization
    and collision-state code can treat them differently from already-built env.
    Returns ``{}`` if ``active_bar_id`` is unknown or the bar curve is degenerate.
    """
    import rhinoscriptsyntax as rs

    deps = _import_deps_for_rb()
    if active_bar_id not in bar_seq_map:
        return {}
    active_oid, _active_seq = bar_seq_map[active_bar_id]

    out = {}
    length_mm, frame_mm = _bar_world_frame_mm(active_oid)
    if length_mm <= 0.0:
        return {}
    rb, _hit = _get_or_build_bar_rigid_body(
        active_oid, length_mm, float(config.BAR_RADIUS), deps
    )
    if rb is not None:
        out[f"{ACTIVE_RB_BAR_PREFIX}{active_bar_id}"] = {
            "rigid_body": rb,
            "frame_world_mm": frame_mm,
            "kind": "bar",
            "source_oid": active_oid,
        }

    joint_layers = (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    )
    for layer in joint_layers:
        if not rs.IsLayer(layer):
            continue
        for joint_oid in rs.ObjectsByLayer(layer) or []:
            if rs.GetUserText(joint_oid, "parent_bar_id") != active_bar_id:
                continue
            joint_id = rs.GetUserText(joint_oid, "joint_id")
            # See collect_built_geometry: ground joints have no joint_subtype.
            subtype = (
                rs.GetUserText(joint_oid, "joint_subtype")
                or rs.GetUserText(joint_oid, "joint_type")
                or "Joint"
            )
            block_name = rs.BlockInstanceName(joint_oid)
            if not block_name:
                continue
            rb, _hit = _get_or_load_joint_rigid_body(block_name, deps)
            if rb is None:
                continue
            xform_mm = _block_instance_xform_mm(joint_oid)
            tag = f"{joint_id or str(joint_oid)}_{subtype.lower()}"
            out[f"{ACTIVE_RB_JOINT_PREFIX}{tag}"] = {
                "rigid_body": rb,
                "frame_world_mm": xform_mm,
                "kind": "joint",
                "source_oid": joint_oid,
                "block_name": block_name,
                "subtype": subtype,
            }
    print(
        f"core.env_collision.collect_active_geometry: {len(out)} bodies for bar {active_bar_id}"
    )
    return out


def collect_all_geometry(bar_seq_map):
    """Collect env-collision payloads for ALL bars and joints in the assembly.

    Same payload format as :func:`collect_built_geometry` but covers every bar
    regardless of assembly sequence. All keys use the ``env_bar_`` /
    ``env_joint_`` prefix so downstream canonicalization strips them uniformly.

    Use this when you need the full scene (e.g. export) rather than a
    per-step IK collision context.
    """
    import rhinoscriptsyntax as rs
    import time

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
        out[f"{ENV_RB_BAR_PREFIX}{bid}"] = {
            "rigid_body": rb,
            "frame_world_mm": frame_mm,
            "kind": "bar",
            "source_oid": oid,
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
            out[f"{ENV_RB_JOINT_PREFIX}{tag}"] = {
                "rigid_body": rb,
                "frame_world_mm": xform_mm,
                "kind": "joint",
                "source_oid": joint_oid,
                "block_name": block_name,
                "subtype": subtype,
            }
    print(
        f"core.env_collision.collect_all_geometry: {len(out)} bodies "
        f"(bars hit/miss={bar_hits}/{bar_misses}, joints hit/miss={j_hits}/{j_misses}) "
        f"in {(time.perf_counter()-t_total)*1000:.1f} ms"
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

    Idempotent on object identity: if the cell already holds the exact same
    ``RigidBody`` instance under ``name``, we skip. New names get added; env
    names no longer present get removed. Returns ``True`` if anything changed
    (caller may need to re-push the cell).
    """
    t0 = time.perf_counter()
    changed = False

    desired_names = set(env_geom.keys())
    existing_env_names = {
        name for name in robot_cell.rigid_body_models.keys()
        if name.startswith(ENV_RB_BAR_PREFIX) or name.startswith(ENV_RB_JOINT_PREFIX)
        or name.startswith(ACTIVE_RB_BAR_PREFIX) or name.startswith(ACTIVE_RB_JOINT_PREFIX)
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
    ``attached_to_link=None``, ``is_hidden=False``.
    """
    from compas_fab.robots import RigidBodyState
    from compas.geometry import Frame

    state = template_state.copy()
    # Keep state workpieces aligned with robot_cell.rigid_body_models: drop
    # all prior env_* entries before writing the current env payload.
    stale_env_names = [
        name for name in state.rigid_body_states
        if name.startswith(ENV_RB_BAR_PREFIX) or name.startswith(ENV_RB_JOINT_PREFIX)
        or name.startswith(ACTIVE_RB_BAR_PREFIX) or name.startswith(ACTIVE_RB_JOINT_PREFIX)
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


# ---------------------------------------------------------------------------
# Verbose pair-count summary
# ---------------------------------------------------------------------------


def summarize_check_collision(planner, state) -> str:
    """Run a verbose ``check_collision`` and return a formatted CC.1-CC.5 summary.

    Captures compas_fab's verbose stdout, parses each ``CC.<n> ... - <verdict>``
    line into pass/skipped(by-reason)/collision counters, and renders the
    table from the spec. Swallows the raw verbose lines (too noisy for the
    Rhino console).
    """
    import contextlib
    import io
    import re

    buf = io.StringIO()
    error_msg = None
    with contextlib.redirect_stdout(buf):
        try:
            planner.check_collision(state, options={"verbose": True, "full_report": True})
        except Exception as exc:
            error_msg = str(exc)
    raw = buf.getvalue()

    line_re = re.compile(
        r"^CC\.(\d).*?-\s*(PASS|COLLISION|SKIPPED\s*\(([^)]+)\))",
        re.MULTILINE,
    )
    counters = {n: {"pass": 0, "collision": 0, "skipped": {}} for n in range(1, 6)}
    for m in line_re.finditer(raw):
        cc_n = int(m.group(1))
        verdict = m.group(2)
        if verdict == "PASS":
            counters[cc_n]["pass"] += 1
        elif verdict == "COLLISION":
            counters[cc_n]["collision"] += 1
        else:
            reason = (m.group(3) or "?").strip().lower()
            counters[cc_n]["skipped"][reason] = counters[cc_n]["skipped"].get(reason, 0) + 1

    labels = {
        1: "CC.1 link<->link",
        2: "CC.2 link<->tool",
        3: "CC.3 link<->env ",
        4: "CC.4 rb<->rb    ",
        5: "CC.5 tool<->env ",
    }
    out_lines = []
    for n in range(1, 6):
        c = counters[n]
        total = c["pass"] + c["collision"] + sum(c["skipped"].values())
        skipped_str = (
            ", ".join(f"{cnt}({reason})" for reason, cnt in c["skipped"].items())
            if c["skipped"] else "0"
        )
        out_lines.append(
            f"  {labels[n]} pairs={total:<4} "
            f"passed={c['pass']:<4} skipped={skipped_str:<24} flagged={c['collision']}"
        )
    if error_msg:
        out_lines.append(f"  (check_collision raised: {error_msg.splitlines()[0]})")
    return "\n".join(out_lines)


# ---------------------------------------------------------------------------
# ACM seam (stub for future active-joint wiring)
# ---------------------------------------------------------------------------


def allow_active_joint_touch(state, joint_rb_name: str, arm_side: str):
    """Whitelist active male joint <-> matched arm tool ONLY.

    Wrist-vs-joint is intentionally NOT whitelisted: that's a real
    collision check (a property of the kinematics, not of the proxy mesh).
    """
    # Whitelist tool only — joint↔tool is by design (gripper engages joint at the kinematic mode-change moment). Wrist↔joint stays a real check.
    rb = state.rigid_body_states[joint_rb_name]
    tool = config.LEFT_TOOL_NAME if arm_side == "left" else config.RIGHT_TOOL_NAME
    rb.touch_bodies = sorted(set(rb.touch_bodies) | {tool})


def _add_touch_bodies(rb_state, names):
    """Union ``names`` into ``rb_state.touch_bodies`` (sorted, deduped)."""
    if not names:
        return
    rb_state.touch_bodies = sorted(set(rb_state.touch_bodies) | set(names))


def configure_active_assembly_acm(state, arm_tool_rb_names=None):
    """Whitelist intentional contacts between active-bar bodies, the arm tools,
    and the env joint mates.

    Design-intent ACM (see cc_lessons.md "ACM whitelisting"):
      * ``active_joint_<jid>_male`` ↔ ``env_joint_<jid>_female`` (and the
        female-active / male-env mirror) — these halves mate on snap; skipping
        the pair is the entire point of the assembly motion.
      * ``active_joint_*`` ↔ both arm tool RBs (gripper engages joint heads).
      * ``active_bar_<bid>`` ↔ all ``active_joint_*`` RBs sharing the bar
        (rigid structural bond).
      * ``active_bar_<bid>`` ↔ both arm tool RBs (bar is held via gripped
        joints; over-permissive but bar body rarely touches tool envelope).

    ``arm_tool_rb_names``: ``{"left": rb_name | None, "right": rb_name | None}``
    as returned by ``robot_cell.attach_arm_tool_rigid_bodies``. ``None``
    entries are skipped.

    Mutates ``state`` in place. Safe to call multiple times.
    """
    rb_states = state.rigid_body_states
    tool_names = []
    if arm_tool_rb_names:
        for side in ("left", "right"):
            n = arm_tool_rb_names.get(side)
            if n:
                tool_names.append(n)

    active_joint_keys = [k for k in rb_states if k.startswith(ACTIVE_RB_JOINT_PREFIX)]
    active_bar_keys = [k for k in rb_states if k.startswith(ACTIVE_RB_BAR_PREFIX)]
    env_joint_keys = {k for k in rb_states if k.startswith(ENV_RB_JOINT_PREFIX)}

    n_mate = 0
    for ajk in active_joint_keys:
        # Tag = "<joint_id>_<subtype>" (subtype lowercased on creation).
        tag = ajk[len(ACTIVE_RB_JOINT_PREFIX):]
        if "_" not in tag:
            continue
        jid, sub = tag.rsplit("_", 1)
        other_sub = "female" if sub == "male" else ("male" if sub == "female" else None)
        if other_sub is None:
            continue
        mate_env = f"{ENV_RB_JOINT_PREFIX}{jid}_{other_sub}"
        if mate_env in env_joint_keys:
            _add_touch_bodies(rb_states[ajk], [mate_env])
            _add_touch_bodies(rb_states[mate_env], [ajk])
            n_mate += 1
        # Tool whitelist (both arms; over-permissive but matches design intent).
        if tool_names:
            _add_touch_bodies(rb_states[ajk], tool_names)
            for tn in tool_names:
                if tn in rb_states:
                    _add_touch_bodies(rb_states[tn], [ajk])

    for abk in active_bar_keys:
        _add_touch_bodies(rb_states[abk], active_joint_keys)
        for ajk in active_joint_keys:
            _add_touch_bodies(rb_states[ajk], [abk])
        if tool_names:
            _add_touch_bodies(rb_states[abk], tool_names)
            for tn in tool_names:
                if tn in rb_states:
                    _add_touch_bodies(rb_states[tn], [abk])

    print(
        f"core.env_collision.configure_active_assembly_acm: "
        f"active_joints={len(active_joint_keys)} active_bars={len(active_bar_keys)} "
        f"mating_pairs_whitelisted={n_mate} tools={tool_names}"
    )
    return state
