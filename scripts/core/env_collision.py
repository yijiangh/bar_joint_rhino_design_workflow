"""Env-collision wiring for assembly IK.

Built bars and their joints (sequence < active step) are registered as
``RobotCell.rigid_body_models`` so compas_fab CC.3/CC.5 evaluate them.

Mesh + frame conventions
------------------------
- Meshes are stored in METERS on the ``RigidBody`` (compas convention).
- Frames pushed onto ``RigidBodyState.frame`` are also METERS (compas Frame).
- The ``env_geom`` dict produced by ``collect_built_geometry`` carries the
  raw mm-based 4x4 frames; conversion happens at ``build_env_state``.

Rhino-only helpers (``collect_built_geometry``) import ``rhinoscriptsyntax``
lazily inside the function body so this module remains importable headless.
"""

from __future__ import annotations

import os
import sys

import numpy as np

from core import config


ENV_RB_BAR_PREFIX = "env_bar_"
ENV_RB_JOINT_PREFIX = "env_joint_"


# ---------------------------------------------------------------------------
# Rhino-side geometry collection
# ---------------------------------------------------------------------------


def _bar_tube_compas_mesh(bar_oid, bar_radius_mm: float):
    """Build a compas Mesh of the bar tube (cylinder along the centerline).

    Returns the mesh in METERS (compas convention).
    Uses Rhino API; only call from inside Rhino.
    """
    import Rhino
    import rhinoscriptsyntax as rs

    start = rs.CurveStartPoint(bar_oid)
    end = rs.CurveEndPoint(bar_oid)
    axis = Rhino.Geometry.Vector3d(end.X - start.X, end.Y - start.Y, end.Z - start.Z)
    length = float(axis.Length)
    if length < 1e-9:
        return None
    axis.Unitize()
    base_plane = Rhino.Geometry.Plane(start, axis)
    cyl = Rhino.Geometry.Cylinder(
        Rhino.Geometry.Circle(base_plane, float(bar_radius_mm)), length,
    )
    brep = cyl.ToBrep(True, True)
    if brep is None:
        return None
    mp = Rhino.Geometry.MeshingParameters.Default
    pieces = Rhino.Geometry.Mesh.CreateFromBrep(brep, mp) or []
    if not pieces:
        return None
    combined = Rhino.Geometry.Mesh()
    for piece in pieces:
        combined.Append(piece)
    return _rhino_mesh_to_compas_meters(combined)


def _block_def_render_mesh(block_def):
    """Combine all renderable geometry inside a block definition into one Rhino mesh.

    Lifted from rs_export_pineapple_obj._block_def_render_mesh; intentionally
    duplicated here so this module doesn't depend on a Rhino entry-point script.
    """
    import Rhino

    combined = Rhino.Geometry.Mesh()
    mp = Rhino.Geometry.MeshingParameters.Default
    for rh_obj in block_def.GetObjects():
        geom = rh_obj.Geometry
        if isinstance(geom, Rhino.Geometry.Mesh):
            combined.Append(geom)
        elif isinstance(geom, Rhino.Geometry.Brep):
            for m in (Rhino.Geometry.Mesh.CreateFromBrep(geom, mp) or []):
                combined.Append(m)
        elif isinstance(geom, Rhino.Geometry.Extrusion):
            brep = geom.ToBrep(False)
            for m in (Rhino.Geometry.Mesh.CreateFromBrep(brep, mp) or []):
                combined.Append(m)
        elif isinstance(geom, Rhino.Geometry.SubD):
            sub_mesh = Rhino.Geometry.Mesh.CreateFromSubD(geom, 1)
            if sub_mesh:
                combined.Append(sub_mesh)
    if combined.Vertices.Count == 0:
        return None
    return combined


def _rhino_mesh_to_compas_meters(rmesh):
    """Convert a Rhino mesh whose verts are in mm to a compas Mesh in meters."""
    from compas.datastructures import Mesh as CMesh

    s = 1.0 / 1000.0
    vertices = [(float(v.X) * s, float(v.Y) * s, float(v.Z) * s) for v in rmesh.Vertices]
    faces = []
    for i in range(rmesh.Faces.Count):
        f = rmesh.Faces[i]
        if f.IsTriangle:
            faces.append([f.A, f.B, f.C])
        else:
            faces.append([f.A, f.B, f.C, f.D])
    return CMesh.from_vertices_and_faces(vertices, faces)


def _find_block_def(name):
    import scriptcontext as sc
    for idef in sc.doc.InstanceDefinitions:
        if idef is not None and not idef.IsDeleted and idef.Name == name:
            return idef
    return None


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


def _joint_def_mesh_cache():
    """sticky-backed cache of compas meshes per block definition name (in meters)."""
    try:
        import scriptcontext as sc
        cache = sc.sticky.get("bar_joint:env_joint_def_mesh_cache")
        if cache is None:
            cache = {}
            sc.sticky["bar_joint:env_joint_def_mesh_cache"] = cache
        return cache
    except ImportError:
        return {}


def _bar_tube_mesh_cache():
    try:
        import scriptcontext as sc
        cache = sc.sticky.get("bar_joint:env_bar_tube_mesh_cache")
        if cache is None:
            cache = {}
            sc.sticky["bar_joint:env_bar_tube_mesh_cache"] = cache
        return cache
    except ImportError:
        return {}


def _joint_def_compas_mesh(block_name):
    cache = _joint_def_mesh_cache()
    if block_name in cache:
        return cache[block_name]
    bdef = _find_block_def(block_name)
    if bdef is None:
        return None
    rmesh = _block_def_render_mesh(bdef)
    if rmesh is None:
        return None
    cmesh = _rhino_mesh_to_compas_meters(rmesh)
    cache[block_name] = cmesh
    return cmesh


def _cached_bar_tube_mesh(bar_oid, bar_radius_mm):
    """Bar tube compas mesh, cached by (bar_oid, radius). Bar curves rarely
    change mid-session; if they do, the cache invalidates on RSPBStop.
    """
    import rhinoscriptsyntax as rs
    cache = _bar_tube_mesh_cache()
    key = (str(rs.coerceguid(bar_oid)), float(bar_radius_mm))
    if key in cache:
        return cache[key]
    cmesh = _bar_tube_compas_mesh(bar_oid, bar_radius_mm)
    if cmesh is not None:
        cache[key] = cmesh
    return cmesh


def collect_built_geometry(active_bar_id, bar_seq_map):
    """Walk ``bar_seq_map`` and build env-collision RBs for every bar with seq < active_seq.

    Returns ``{rb_name: {mesh, frame_world_mm, kind, source_oid}}``:
      - bar entries: tube cylinder mesh along the bar curve, ``frame_world_mm = identity``
        (mesh vertices already encode world position).
      - joint entries: block-definition mesh, ``frame_world_mm = block instance xform``
        (the block def origin is the joint's OCF).

    Skips the active bar AND its joints. Headless replays do NOT call this —
    they hydrate from the v2 capture's ``robot_cell_ref`` instead.
    """
    import rhinoscriptsyntax as rs

    if active_bar_id not in bar_seq_map:
        return {}
    _active_oid, active_seq = bar_seq_map[active_bar_id]

    built_bar_ids = {
        bid: oid for bid, (oid, seq) in bar_seq_map.items() if seq < active_seq
    }
    if not built_bar_ids:
        return {}

    out = {}
    for bid, oid in built_bar_ids.items():
        cmesh = _cached_bar_tube_mesh(oid, float(config.BAR_RADIUS))
        if cmesh is None:
            continue
        out[f"{ENV_RB_BAR_PREFIX}{bid}"] = {
            "mesh": cmesh,
            "frame_world_mm": np.eye(4, dtype=float),
            "kind": "bar",
            "source_oid": oid,
        }

    # Joints whose parent_bar_id is a built bar.
    joint_layers = (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
    )
    for layer in joint_layers:
        if not rs.IsLayer(layer):
            continue
        for joint_oid in rs.ObjectsByLayer(layer) or []:
            parent_bar = rs.GetUserText(joint_oid, "parent_bar_id")
            if parent_bar not in built_bar_ids:
                continue
            joint_id = rs.GetUserText(joint_oid, "joint_id")
            subtype = rs.GetUserText(joint_oid, "joint_subtype") or "Joint"
            block_name = rs.BlockInstanceName(joint_oid)
            if not block_name:
                continue
            cmesh = _joint_def_compas_mesh(block_name)
            if cmesh is None:
                continue
            xform_mm = _block_instance_xform_mm(joint_oid)
            tag = f"{joint_id or str(joint_oid)}_{subtype.lower()}"
            out[f"{ENV_RB_JOINT_PREFIX}{tag}"] = {
                "mesh": cmesh,
                "frame_world_mm": xform_mm,
                "kind": "joint",
                "source_oid": joint_oid,
                "block_name": block_name,
                "subtype": subtype,
            }
    return out


# ---------------------------------------------------------------------------
# RobotCell / state wiring
# ---------------------------------------------------------------------------


def register_env_in_robot_cell(robot_cell, env_geom, *, deps):
    """Add env meshes to ``robot_cell.rigid_body_models`` as static obstacles.

    Idempotent on (name, vertex/face count): re-runs that yield the same
    geometry are no-ops; new bars/joints are appended; previously-registered
    env entries that are no longer in ``env_geom`` are removed so a later
    step doesn't pollute the cell with stale obstacles. Returns True if any
    rigid body was added/removed (caller may need to re-push the cell).
    """
    RigidBody = deps.get("RigidBody")
    if RigidBody is None:
        from compas_fab.robots import RigidBody as _RB
        RigidBody = _RB

    changed = False

    desired_names = set(env_geom.keys())
    existing_env_names = {
        name for name in robot_cell.rigid_body_models.keys()
        if name.startswith(ENV_RB_BAR_PREFIX) or name.startswith(ENV_RB_JOINT_PREFIX)
    }
    for stale in existing_env_names - desired_names:
        robot_cell.rigid_body_models.pop(stale, None)
        changed = True

    for name, payload in env_geom.items():
        mesh = payload["mesh"]
        existing = robot_cell.rigid_body_models.get(name)
        if existing is not None:
            existing_mesh = existing.collision_meshes[0] if existing.collision_meshes else None
            if (
                existing_mesh is not None
                and existing_mesh.number_of_vertices() == mesh.number_of_vertices()
                and existing_mesh.number_of_faces() == mesh.number_of_faces()
            ):
                continue
        rb = RigidBody.from_mesh(mesh)
        robot_cell.rigid_body_models[name] = rb
        changed = True
    return changed


def build_env_state(template_state, env_geom):
    """Return a copy of ``template_state`` with ``rigid_body_states`` populated.

    Each env body is a static obstacle: ``frame`` (METERS) set from the
    mm-based ``frame_world_mm`` payload, ``attached_to_tool=None``,
    ``attached_to_link=None``, ``is_hidden=False``.
    """
    if not env_geom:
        return template_state

    from compas_fab.robots import RigidBodyState
    from compas.geometry import Frame

    state = template_state.copy()
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
