#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSIKKeyframe - Dual-arm IK keyframe workflow.

Pick a single bar that already carries exactly two male joints, each with
a robotic tool block placed by ``rs_joint_edit`` (one tool with a name
ending in 'L' for the left arm, one ending in 'R' for the right arm).
The placed tool block instances ARE the wrist+tool proxies; their world
origins are tool0 (the robot flange frame) for IK. The script then:

1. Resolves left/right tool block instances on the picked bar.
2. Prompts for a base point on a Brep in the ``Walkable Ground`` layer
   (under MANAGED Scaffolding) and a heading point defining the base X-axis.
3. Solves dual-arm IK (left then right group). If unreachable, samples
   base frames in a circle around the pick and re-snaps each to the same
   Brep face.
4. Previews the robot via ``core.ik_viz``.
5. Repeats 3-4 for the approach pose, offset along
   ``-unit(avg(tool_z_L, tool_z_R)) * LM_DISTANCE``.
6. On accept, writes ``ik_assembly`` user-text (JSON payload) on the bar
   curve; robot meshes are cleared.
"""

from __future__ import annotations

import datetime
import importlib
import json
import math
import os
import random
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import capture_io as _capture_io_module
from core import config as _config_module
from core import dynamic_preview as _dynamic_preview_module
from core import env_collision as _env_collision_module
from core import highlight_env as _highlight_env_module
from core import ik_viz as _ik_viz_module
from core import robot_cell as _robot_cell_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    get_bar_seq_map,
    repair_on_entry,
    reset_sequence_colors,
    show_sequence_colors,
)
from core.rhino_frame_io import doc_unit_scale_to_mm
from core.rhino_helpers import suspend_redraw  # noqa: F401  (kept for parity)
from core.rhino_tool_place import find_tool_for_joint
from core.robotic_tool import get_robotic_tool


# ---------------------------------------------------------------------------
# Constants / user-text keys
# ---------------------------------------------------------------------------

# Legacy single-blob key (kept for back-compat readers; new writes go to the
# split keys below from `core.config`).
IK_ASSEMBLY_KEY = "ik_assembly"


# ---------------------------------------------------------------------------
# Module reload (matches rs_joint_place.py pattern)
# ---------------------------------------------------------------------------


def _reload_runtime_modules():
    global capture_io, config, dynamic_preview, env_collision, highlight_env, ik_viz, robot_cell
    config = importlib.reload(_config_module)
    dynamic_preview = importlib.reload(_dynamic_preview_module)
    env_collision = importlib.reload(_env_collision_module)
    highlight_env = importlib.reload(_highlight_env_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)
    capture_io = importlib.reload(_capture_io_module)


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Rhino <-> numpy helpers (doc units -> mm)
# ---------------------------------------------------------------------------


def _rhino_xform_to_np_mm(xform):
    scale = doc_unit_scale_to_mm()
    matrix = np.array([[float(xform[i, j]) for j in range(4)] for i in range(4)], dtype=float)
    matrix[:3, 3] *= scale  # translation -> mm
    return matrix


def _np_mm_to_rhino_xform(matrix: np.ndarray):
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    doc_matrix = np.array(matrix, dtype=float, copy=True)
    doc_matrix[:3, 3] *= scale_from_mm
    xform = Rhino.Geometry.Transform(1.0)
    for i in range(4):
        for j in range(4):
            xform[i, j] = float(doc_matrix[i, j])
    return xform


def _block_instance_xform_mm(object_id) -> np.ndarray:
    """Return the block instance's world transform as a 4x4 numpy matrix in mm."""
    obj = rs.coercerhinoobject(object_id, True, True)
    if not isinstance(obj, Rhino.DocObjects.InstanceObject):
        raise RuntimeError(f"Object {object_id} is not a block instance.")
    return _rhino_xform_to_np_mm(obj.InstanceXform)


def _point_to_mm(point) -> np.ndarray:
    scale = doc_unit_scale_to_mm()
    if hasattr(point, "X"):
        return np.array([point.X, point.Y, point.Z], dtype=float) * scale
    return np.asarray(point, dtype=float) * scale


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("Cannot unitize a zero-length vector.")
    return np.asarray(vector, dtype=float) / norm


# ---------------------------------------------------------------------------
# Block / layer helpers
# ---------------------------------------------------------------------------


def _has_block_definition(name) -> bool:
    for idef in sc.doc.InstanceDefinitions:
        if idef is not None and not idef.IsDeleted and idef.Name == name:
            return True
    return False


# ---------------------------------------------------------------------------
# Bar -> (left tool, right tool) resolution
# ---------------------------------------------------------------------------


def _arm_side_from_tool_name(tool_name):
    """Classify a tool by ``tool_name`` user-text suffix: 'L' -> left, 'R' -> right."""
    if not tool_name:
        return None
    last = tool_name.strip()[-1].upper()
    if last == "L":
        return "left"
    if last == "R":
        return "right"
    return None


def _males_on_bar(bar_id):
    """Return list of joint block instance oids whose ``parent_bar_id`` matches.

    Scans BOTH the male-joint layer and the ground-joint layer: assembly IK
    treats any tool-bearing joint instance on the bar as an arm anchor, so a
    bar with one male + one ground (or two grounds) is a valid 2-anchor
    configuration.  The variable name is kept as ``males`` for back-compat
    with downstream code that just consumes opaque block-instance oids.
    """
    out = []
    for layer in (
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        out.extend(
            oid
            for oid in rs.ObjectsByLayer(layer) or []
            if rs.GetUserText(oid, "parent_bar_id") == bar_id
        )
    return out


def _resolve_arm_tools_on_bar(bar_oid):
    """Return ``(bar_id, left_tuple, right_tuple)`` where each tuple is
    ``(male_joint_oid, tool_oid)``, or ``(None, error_message)`` on failure.

    Failures are reported as a 2-tuple so the caller can re-prompt without
    aborting the whole command.
    """
    bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY)
    if not bar_id:
        return None, f"Picked curve has no '{BAR_ID_KEY}' user-text; not a registered bar."

    males = _males_on_bar(bar_id)
    if len(males) != 2:
        return None, (
            f"Bar '{bar_id}' has {len(males)} tool-bearing joint(s) (male+ground); "
            "need exactly 2 (single-joint flow not yet supported)."
        )

    left = right = None
    for moid in males:
        jid = rs.GetUserText(moid, "joint_id")
        if not jid:
            return None, f"Male block on bar '{bar_id}' is missing 'joint_id' user-text."
        toid = find_tool_for_joint(jid)
        if toid is None:
            return None, (
                f"Joint '{jid}' on bar '{bar_id}' has no robotic tool placed. "
                "Run RSJointEdit / tool-cycle first."
            )
        tname = rs.GetUserText(toid, "tool_name") or ""
        side = _arm_side_from_tool_name(tname)
        if side is None:
            return None, (
                f"Tool '{tname}' on joint '{jid}' has no L/R suffix in its name; "
                "cannot decide arm side."
            )
        if side == "left":
            if left is not None:
                return None, f"Bar '{bar_id}' has two LEFT-suffix tools; need one L + one R."
            left = (moid, toid)
        else:
            if right is not None:
                return None, f"Bar '{bar_id}' has two RIGHT-suffix tools; need one L + one R."
            right = (moid, toid)

    if left is None or right is None:
        missing = "left" if left is None else "right"
        return None, f"Bar '{bar_id}' is missing the {missing}-arm tool (need one L + one R)."

    return (bar_id, left, right), None


def _pick_bar_with_arm_tools():
    """Loop until the user picks a bar that satisfies the L/R tool layout,
    or cancels. Returns ``(bar_id, bar_oid, left_tuple, right_tuple)`` or None.
    """
    seq_map = get_bar_seq_map()
    while True:
        bar_oid = pick_bar(
            "Pick the Ln bar to assemble (must have 2 tool-bearing joints with L/R tools placed)"
        )
        if bar_oid is None:
            return None
        result, err = _resolve_arm_tools_on_bar(bar_oid)
        if err is not None:
            print(f"RSIKKeyframe: {err} Pick another bar or press Esc to cancel.")
            continue
        bar_id, left, right = result
        if bar_id not in seq_map:
            print(
                f"RSIKKeyframe: bar '{bar_id}' is not in the bar registry. "
                "Pick another bar or press Esc to cancel."
            )
            continue
        return bar_id, bar_oid, left, right


# ---------------------------------------------------------------------------
# WalkableGround picking
# ---------------------------------------------------------------------------


_WALKABLE_FILTER = rs.filter.surface | rs.filter.polysurface | rs.filter.extrusion


def _as_brep(object_id):
    """Return the Brep geometry for `object_id`, converting from Extrusion if needed.

    `rs.coercebrep` returns None for Rhino Extrusion objects (e.g. a closed
    box primitive); we call `.ToBrep()` on the extrusion geometry so the
    downstream closest-point + face-normal code works uniformly.
    """
    brep = rs.coercebrep(object_id)
    if brep is not None:
        return brep
    rhobj = rs.coercerhinoobject(object_id, True, True)
    geom = getattr(rhobj, "Geometry", None)
    if isinstance(geom, Rhino.Geometry.Extrusion):
        return geom.ToBrep(False)
    raise RuntimeError(f"Object {object_id} is not a Brep, Surface or Extrusion.")


def _breps_in_layer(layer_name):
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
        return []
    accepted = {rs.filter.surface, rs.filter.polysurface, rs.filter.extrusion}
    return [
        oid for oid in rs.ObjectsByLayer(layer_name) or []
        if rs.ObjectType(oid) in accepted
    ]


def _pick_walkable_brep():
    candidates = _breps_in_layer(config.WALKABLE_GROUND_LAYER)
    if not candidates:
        rs.MessageBox(
            f"Layer '{config.WALKABLE_GROUND_LAYER}' has no surface/polysurface/extrusion. "
            "Add one (e.g. a surface patch or a closed box) and try again.",
            0,
            "RSIKKeyframe",
        )
        return None
    if len(candidates) == 1:
        return candidates[0]
    oid = rs.GetObject(
        f"Pick a Brep / Extrusion on the '{config.WALKABLE_GROUND_LAYER}' layer",
        filter=_WALKABLE_FILTER,
    )
    if oid is None or rs.ObjectLayer(oid) != config.WALKABLE_GROUND_LAYER:
        rs.MessageBox(
            f"Please pick an object that belongs to layer '{config.WALKABLE_GROUND_LAYER}'.",
            0,
            "RSIKKeyframe",
        )
        return None
    return oid


def _resolve_sampling_brep_for_base(seed_base_frame_mm, brep_id):
    if brep_id is not None:
        return brep_id

    candidates = _breps_in_layer(config.WALKABLE_GROUND_LAYER)
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    seed_origin_mm = seed_base_frame_mm[:3, 3]
    best_oid = None
    best_dist = None
    for candidate_oid in candidates:
        try:
            brep = _as_brep(candidate_oid)
        except RuntimeError:
            continue
        snapped_origin_mm, _normal = _snap_to_brep(brep, seed_origin_mm)
        if snapped_origin_mm is None:
            continue
        dist = float(np.linalg.norm(snapped_origin_mm - seed_origin_mm))
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_oid = candidate_oid
    return best_oid


def _closest_point_on_brep(brep, point_doc):
    """Return (point_doc, normal_world) of the closest surface point to `point_doc`."""
    best = None
    for face in brep.Faces:
        ok, u, v = face.ClosestPoint(point_doc)
        if not ok:
            continue
        close_pt = face.PointAt(u, v)
        dist = close_pt.DistanceTo(point_doc)
        if best is None or dist < best[0]:
            normal = face.NormalAt(u, v)
            best = (dist, close_pt, normal)
    if best is None:
        return None, None
    _, pt, normal = best
    return pt, normal


def _world_from_base_doc_xform(origin_doc, normal_doc, heading_doc_vec):
    """Doc-unit Rhino.Geometry.Transform with origin=origin_doc, Z=normal,
    X = heading_doc_vec projected onto the plane perp to Z.
    """
    z = Rhino.Geometry.Vector3d(normal_doc)
    z.Unitize()
    v = Rhino.Geometry.Vector3d(heading_doc_vec)
    x_raw = v - (v * z) * z
    if x_raw.Length < 1e-9:
        return None
    x_raw.Unitize()
    y = Rhino.Geometry.Vector3d.CrossProduct(z, x_raw)
    xform = Rhino.Geometry.Transform(1.0)
    xform[0, 0], xform[0, 1], xform[0, 2], xform[0, 3] = x_raw.X, y.X, z.X, origin_doc.X
    xform[1, 0], xform[1, 1], xform[1, 2], xform[1, 3] = x_raw.Y, y.Y, z.Y, origin_doc.Y
    xform[2, 0], xform[2, 1], xform[2, 2], xform[2, 3] = x_raw.Z, y.Z, z.Z, origin_doc.Z
    xform[3, 0], xform[3, 1], xform[3, 2], xform[3, 3] = 0.0, 0.0, 0.0, 1.0
    return xform


def _bake_robot_meshes_at_zero():
    """Return Rhino meshes for every robot link at zero config (for the ghost preview).

    Reads from the cached :class:`RobotModelObject` via
    :func:`ik_viz.get_robot_link_meshes_at_zero` -- baking only happens on the
    first call and is shared with the rest of the IK preview pipeline (so the
    bake cost is amortized rather than thrown away).
    """
    return ik_viz.get_robot_link_meshes_at_zero(layer_key=ik_viz.LAYER_KEY_ASSEMBLY)


def _pick_base_frame_on_walkable(brep_id):
    """Pick base origin + heading on the walkable brep, with the dual-arm
    robot's mesh tracking the cursor. Returns
    (base_origin_mm, base_normal, heading_mm, seed_base_frame_mm) or all None.
    """
    brep = _as_brep(brep_id)
    robot_meshes = _bake_robot_meshes_at_zero()

    with dynamic_preview.mesh_preview(robot_meshes, alpha=0.4) as conduit:
        # Phase A: base origin on brep.
        def _xform_phase_a(cursor_doc):
            close_pt, normal = _closest_point_on_brep(brep, cursor_doc)
            if close_pt is None:
                return None
            world_x = Rhino.Geometry.Vector3d(1.0, 0.0, 0.0)
            return _world_from_base_doc_xform(close_pt, normal, world_x)

        picker_a = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_a)
        picker_a.SetCommandPrompt("Pick base origin on WalkableGround brep")
        picker_a.Constrain(brep, -1, -1, False)
        if picker_a.Get() != Rhino.Input.GetResult.Point:
            return None, None, None, None
        picked_doc = picker_a.Point()
        close_pt, normal = _closest_point_on_brep(brep, picked_doc)
        if close_pt is None:
            return None, None, None, None

        # Phase B: heading point.
        def _xform_phase_b(cursor_doc):
            heading_vec = Rhino.Geometry.Vector3d(
                cursor_doc.X - close_pt.X,
                cursor_doc.Y - close_pt.Y,
                cursor_doc.Z - close_pt.Z,
            )
            return _world_from_base_doc_xform(close_pt, normal, heading_vec)

        picker_b = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_b)
        picker_b.SetCommandPrompt("Pick heading point (defines base +X)")
        picker_b.SetBasePoint(close_pt, True)
        if picker_b.Get() != Rhino.Input.GetResult.Point:
            return None, None, None, None
        heading_doc = picker_b.Point()

    base_origin_mm = _point_to_mm(close_pt)
    normal_v = np.array([normal.X, normal.Y, normal.Z], dtype=float)
    heading_mm = _point_to_mm(heading_doc)
    try:
        seed_base = _frame_from_origin_normal_heading(base_origin_mm, normal_v, heading_mm)
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSIKKeyframe")
        return None, None, None, None
    return base_origin_mm, normal_v, heading_mm, seed_base


def _frame_from_origin_normal_heading(origin_mm, normal, heading_mm) -> np.ndarray:
    z = _unit(normal)
    heading_vec = heading_mm - origin_mm
    x_raw = heading_vec - np.dot(heading_vec, z) * z
    if float(np.linalg.norm(x_raw)) < 1e-6:
        raise RuntimeError("Heading point is collinear with base normal; pick a different point.")
    x = _unit(x_raw)
    y = np.cross(z, x)
    frame = np.eye(4, dtype=float)
    frame[:3, 0] = x
    frame[:3, 1] = y
    frame[:3, 2] = z
    frame[:3, 3] = origin_mm
    return frame


# ---------------------------------------------------------------------------
# Interactive prompts
# ---------------------------------------------------------------------------


def _ask_collision_options(env_count: int = 0):
    """Prompt for IK collision flags + the robot mesh display mode.

    Returns ``(include_self, include_env, mesh_mode)`` or ``None`` on cancel.
    The `mesh_mode` toggle picks between ``ik_viz.MESH_MODE_VISUAL`` and
    ``MESH_MODE_COLLISION`` for `ik_viz.show_state`. The previous choice is
    cached in `sc.sticky` (via ik_viz.set_mesh_mode) so subsequent runs in
    the same session start with the user's last selection pre-toggled.
    """
    current_mesh_mode = ik_viz.get_mesh_mode()
    is_collision_default = current_mesh_mode == ik_viz.MESH_MODE_COLLISION

    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(f"Collision options (env={env_count} bodies)")
    opt_self = Rhino.Input.Custom.OptionToggle(True, "No", "Yes")
    opt_env = Rhino.Input.Custom.OptionToggle(True, "No", "Yes")
    opt_mesh = Rhino.Input.Custom.OptionToggle(is_collision_default, "Visual", "Collision")
    go.AddOptionToggle("IncludeSelf", opt_self)
    go.AddOptionToggle("IncludeEnv", opt_env)
    go.AddOptionToggle("MeshMode", opt_mesh)
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Option:
            continue
        if result == Rhino.Input.GetResult.Nothing:
            mesh_mode = ik_viz.MESH_MODE_COLLISION if bool(opt_mesh.CurrentValue) else ik_viz.MESH_MODE_VISUAL
            ik_viz.set_mesh_mode(mesh_mode)
            return bool(opt_self.CurrentValue), bool(opt_env.CurrentValue), mesh_mode
        return None


def _ask_accept(prompt="Accept this IK keyframe and save it on the bar", allow_accept=True):
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(prompt)
    accept_idx = go.AddOption("Accept") if allow_accept else None
    retry_same_idx = go.AddOption("RetrySameBase")
    retry_new_idx = go.AddOption("RetryNewBase")
    give_up_idx = go.AddOption("GiveUp")
    go.SetCommandPromptDefault("RetrySameBase")
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return "retry_same_base"
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if allow_accept and chosen == accept_idx:
                return "accept"
            if chosen == retry_same_idx:
                return "retry_same_base"
            if chosen == retry_new_idx:
                return "retry_new_base"
            if chosen == give_up_idx:
                return "give_up"
            continue
        return "give_up"


def _ask_reuse_saved_base():
    """Prompt 'Reuse saved base frame?' with [Reuse|NewPick]; default = Reuse.

    Returns True to reuse, False to pick a new base, or None on Esc.
    """
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt("Saved base frame found on this bar; press Enter to reuse")
    reuse_idx = go.AddOption("Reuse")
    new_idx = go.AddOption("NewPick")
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return True
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == reuse_idx:
                return True
            if chosen == new_idx:
                return False
            continue
        return None


def _preview_robot_at_base(planner, template_state, base_frame_mm, mesh_mode):
    """Render the dual-arm robot at `base_frame_mm` with default arm config.

    Cheap visualization for the reuse-base prompt: no IK, just the URDF
    meshes posed at the saved base. Caller is responsible for clearing
    via `ik_viz.end_session()`.
    """
    state = template_state.copy()
    robot_cell._apply_base_frame_mm(state, base_frame_mm)
    robot_cell.set_cell_state(planner, state)
    rcell = robot_cell.get_or_load_robot_cell()
    # Same layer_key as ShowIK so the cache is shared across commands.
    ik_viz.update_state(state, robot_cell=rcell, layer_key=ik_viz.LAYER_KEY_ASSEMBLY)
    ik_viz.set_active_mesh_mode(ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode)


def _resolve_tool_collision_paths(left_tool_oid, right_tool_oid):
    """Return (left_path, right_path) absolute filesystem paths to the per-arm
    tool collision OBJs declared in `core/robotic_tools.json`.

    Empty string for an arm whose tool entry has no `collision_filename`.
    """
    out = {"left": "", "right": ""}
    for side, oid in (("left", left_tool_oid), ("right", right_tool_oid)):
        tname = rs.GetUserText(oid, "tool_name") or ""
        if not tname:
            print(f"RSIKKeyframe: {side} tool block has no 'tool_name' user-text; cannot look up collision OBJ.")
            continue
        try:
            tooldef = get_robotic_tool(tname)
        except KeyError as exc:
            print(f"RSIKKeyframe: {exc}; skipping collision attach for {side} arm.")
            continue
        path = tooldef.collision_path()
        if not path:
            print(
                f"RSIKKeyframe: tool '{tname}' has no 'collision_filename' in robotic_tools.json; "
                f"skipping collision attach for {side} arm."
            )
        out[side] = path
    return out["left"], out["right"]


def _hide_inactive_tool_blocks(active_bar_id):
    """Hide every tool-instance whose joint isn't on `active_bar_id`. Returns
    a list of oids that were actually hidden (so caller can restore).
    """
    if not rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        return []
    active_joint_ids = set()
    for layer in (
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            if (
                rs.GetUserText(oid, "parent_bar_id") == active_bar_id
                and rs.GetUserText(oid, "joint_id")
            ):
                active_joint_ids.add(rs.GetUserText(oid, "joint_id"))
    hidden = []
    for oid in rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or []:
        jid = rs.GetUserText(oid, "joint_id")
        if jid in active_joint_ids:
            continue
        if rs.IsObjectHidden(oid):
            continue
        if rs.HideObject(oid):
            hidden.append(oid)
    return hidden


def _show_objects(oids):
    for oid in oids or []:
        if rs.IsObject(oid):
            rs.ShowObject(oid)


# ---------------------------------------------------------------------------
# IK solving with base sampling
# ---------------------------------------------------------------------------


def _sample_base_offsets(count, radius_mm):
    rng = random.Random()
    for _ in range(count):
        angle = rng.uniform(0.0, 2.0 * math.pi)
        r = rng.uniform(0.0, radius_mm)
        yield np.array([r * math.cos(angle), r * math.sin(angle), 0.0], dtype=float)


def _snap_to_brep(brep, origin_mm):
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    pt_doc = Rhino.Geometry.Point3d(*(origin_mm * scale_from_mm))
    close_pt, normal = _closest_point_on_brep(brep, pt_doc)
    if close_pt is None:
        return None, None
    return _point_to_mm(close_pt), np.array([normal.X, normal.Y, normal.Z], dtype=float)


def _solve_with_sampling(planner, template_state, seed_base_frame_mm,
                         tool0_left_mm, tool0_right_mm,
                         brep_id, heading_mm, include_self, include_env):
    """Try seed first; on failure, sample base frames around seed until success.

    If `brep_id` is None (reuse-saved-base path), the sampling fallback is
    disabled and only the seed base frame is attempted. The caller is
    expected to handle the no-solution case by re-prompting for a fresh
    base frame on the next run.
    """
    check_collision = bool(include_self or include_env)
    # Environment collision is only meaningful when env geometry is registered in
    # the robot cell; first-pass workflow has none, so include_env alone is a hint.
    attempts = [seed_base_frame_mm]
    if brep_id is not None:
        brep = _as_brep(brep_id)
        for offset in _sample_base_offsets(config.IK_BASE_SAMPLE_MAX_ITER, config.IK_BASE_SAMPLE_RADIUS):
            sample_origin_mm = seed_base_frame_mm[:3, 3] + offset
            snapped_origin, normal = _snap_to_brep(brep, sample_origin_mm)
            if snapped_origin is None:
                continue
            try:
                sample_frame = _frame_from_origin_normal_heading(snapped_origin, normal, heading_mm)
            except RuntimeError:
                continue
            attempts.append(sample_frame)
    else:
        print(
            "RSIKKeyframe: brep_id=None (reuse path) - sampling fallback disabled; "
            "trying seed base frame only."
        )

    total = len(attempts)
    for idx, base_frame in enumerate(attempts):
        label = "seed" if idx == 0 else f"sample {idx}/{total - 1}"
        origin = base_frame[:3, 3]
        print(
            f"RSIKKeyframe: trying base frame ({label}) "
            f"at ({origin[0]:.1f}, {origin[1]:.1f}, {origin[2]:.1f}) mm ..."
        )
        state = robot_cell.solve_dual_arm_ik(
            planner,
            template_state,
            base_frame,
            tool0_left_mm,
            tool0_right_mm,
            check_collision=check_collision,
            verbose_pairs=check_collision,
        )
        if state is not None:
            print(
                f"RSIKKeyframe: [OK] IK solution FOUND on attempt {idx + 1}/{total} ({label})."
            )
            return state, base_frame
        print(f"RSIKKeyframe: [x] IK failed on attempt {idx + 1}/{total} ({label}).")
    print(
        f"RSIKKeyframe: [X] IK failed for all {total} attempt(s). "
        f"Consider increasing IK_BASE_SAMPLE_RADIUS / IK_BASE_SAMPLE_MAX_ITER in config.py."
    )
    return None, None


# ---------------------------------------------------------------------------
# Payload
# ---------------------------------------------------------------------------


def _translate_frame(frame_mm, offset_mm):
    out = np.array(frame_mm, dtype=float, copy=True)
    out[:3, 3] = out[:3, 3] + np.asarray(offset_mm, dtype=float)
    return out


def _build_assembly_payload(base_frame_mm, final_state, approach_state, rcell):
    return {
        "robot_id": config.ROBOT_ID,
        "base_frame_world_mm": np.asarray(base_frame_mm, dtype=float).tolist(),
        "final": {
            "left": robot_cell.extract_group_config(final_state, config.LEFT_GROUP, rcell),
            "right": robot_cell.extract_group_config(final_state, config.RIGHT_GROUP, rcell),
        },
        "approach": {
            "left": robot_cell.extract_group_config(approach_state, config.LEFT_GROUP, rcell),
            "right": robot_cell.extract_group_config(approach_state, config.RIGHT_GROUP, rcell),
        },
    }


# ---------------------------------------------------------------------------
# Split user-text writers (one logical concept per key)
# ---------------------------------------------------------------------------


def _build_group_pair(state, rcell):
    return {
        "left": robot_cell.extract_group_config(state, config.LEFT_GROUP, rcell),
        "right": robot_cell.extract_group_config(state, config.RIGHT_GROUP, rcell),
    }


def _write_assembly_base_frame(bar_oid, base_frame_mm):
    payload = np.asarray(base_frame_mm, dtype=float).tolist()
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME, json.dumps(payload))
    print(f"RSIKKeyframe: saved '{config.KEY_ASSEMBLY_BASE_FRAME}' on bar.")


def _write_assembly_keyframes(bar_oid, final_state, approach_state, rcell):
    final_payload = _build_group_pair(final_state, rcell)
    approach_payload = _build_group_pair(approach_state, rcell)
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED, json.dumps(final_payload))
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH, json.dumps(approach_payload))
    print(
        f"RSIKKeyframe: saved '{config.KEY_ASSEMBLY_IK_ASSEMBLED}' + "
        f"'{config.KEY_ASSEMBLY_IK_APPROACH}' on bar."
    )


def _write_legacy_assembly_blob(bar_oid, base_frame_mm, final_state, approach_state, rcell):
    """Back-compat: also write the legacy bundled `ik_assembly` blob.

    `rs_ik_support_keyframe.py` and `rs_show_ik.py` still read this single
    key. Drop this dual-write once both have been migrated to the
    `KEY_ASSEMBLY_*` split keys.
    """
    payload = _build_assembly_payload(base_frame_mm, final_state, approach_state, rcell)
    rs.SetUserText(bar_oid, IK_ASSEMBLY_KEY, json.dumps(payload))
    print(f"RSIKKeyframe: also wrote legacy '{IK_ASSEMBLY_KEY}' blob (back-compat).")


def _read_saved_assembly_base_frame(bar_oid):
    """Return previously-saved base frame as a 4x4 np.ndarray (mm), or None."""
    raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    if not raw:
        return None
    try:
        data = json.loads(raw)
        m = np.asarray(data, dtype=float)
        if m.shape != (4, 4):
            print(
                f"RSIKKeyframe: saved base frame on bar has unexpected shape {m.shape}; ignoring."
            )
            return None
        return m
    except Exception as exc:
        print(f"RSIKKeyframe: could not parse saved base frame ({exc}); ignoring.")
        return None


def _heading_mm_from_base_frame(base_frame_mm):
    """Reconstruct a 'heading point' (origin + 100mm * X-axis) from a saved base frame.

    Used so the sampling code in `_solve_with_sampling` (which derives the
    base X from `heading - origin`) keeps the same X direction across reuse.
    """
    origin = np.asarray(base_frame_mm[:3, 3], dtype=float)
    x_axis = np.asarray(base_frame_mm[:3, 0], dtype=float)
    return origin + 100.0 * x_axis


# ---------------------------------------------------------------------------
# Capture-and-replay
# ---------------------------------------------------------------------------


_CAPTURES_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "tests", "captures"))


def _ask_save_failure_capture(phase_label: str) -> bool:
    """Prompt 'IK failed at <phase>; save capture for headless debugging?'

    Default is No because most users will iterate inside Rhino without
    needing the headless replay. Returns True iff the user typed "Yes".
    """
    answer = rs.GetString(
        f"{phase_label} IK failed. Save capture for headless debugging?",
        "No",
        ["Yes", "No"],
    )
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("y")


def _ask_save_debug_capture() -> bool:
    """Prompt 'Save debug capture for headless replay?' on accept.

    Production state already lives on the bar user-text; the capture is a
    debug artefact (RobotCell geometry + IK targets + options) needed only
    when a bug is being investigated headless. Default No.
    """
    answer = rs.GetString(
        "Save debug capture for headless replay?",
        "No",
        ["Yes", "No"],
    )
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("y")


def _save_capture(
    *,
    target_bar_id,
    left_oid,
    right_oid,
    ocf_left,
    ocf_right,
    tool0_left_final,
    tool0_right_final,
    tool0_left_approach,
    tool0_right_approach,
    initial_state,
    include_self,
    include_env,
    final_state=None,
    approach_state=None,
    rcell=None,
    suffix: str = "",
) -> str | None:
    """Write a v2 capture JSON into ``tests/captures/``.

    The serialized ``RobotCell`` (which already has env rigid bodies
    registered by this point) goes to ``robot_cells/<sha8>.json`` and is
    referenced from the capture. ``initial_state`` carries the seed base
    frame + env rigid_body_states. ``expected`` holds the accepted final /
    approach states (or None on failure captures).
    """
    try:
        os.makedirs(_CAPTURES_DIR, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = f"{timestamp}_{target_bar_id}{suffix}"
        path = os.path.join(_CAPTURES_DIR, f"{stem}.json")

        cell_ref = capture_io.save_robot_cell_if_changed(rcell, _CAPTURES_DIR)

        ik_targets = {
            "final": {
                "left_tool0_world_mm": np.asarray(tool0_left_final, dtype=float).tolist(),
                "right_tool0_world_mm": np.asarray(tool0_right_final, dtype=float).tolist(),
            },
            "approach": {
                "left_tool0_world_mm": np.asarray(tool0_left_approach, dtype=float).tolist(),
                "right_tool0_world_mm": np.asarray(tool0_right_approach, dtype=float).tolist(),
            },
        }
        ik_options = {
            "check_collision": bool(include_self or include_env),
            "include_self": bool(include_self),
            "include_env": bool(include_env),
        }
        source = {
            "target_bar_id": target_bar_id,
            "left_block_name": rs.BlockInstanceName(left_oid),
            "right_block_name": rs.BlockInstanceName(right_oid),
            "left_arm_side": "left",
            "right_arm_side": "right",
            "left_ocf_world_mm": np.asarray(ocf_left, dtype=float).tolist(),
            "right_ocf_world_mm": np.asarray(ocf_right, dtype=float).tolist(),
            "lm_distance_mm": float(config.LM_DISTANCE),
            "doc_unit_scale_to_mm": float(doc_unit_scale_to_mm()),
        }
        capture_io.save_capture_v2(
            path,
            captured_at=datetime.datetime.now().isoformat(timespec="seconds"),
            robot_cell_ref=cell_ref,
            initial_state=initial_state,
            ik_targets=ik_targets,
            ik_options=ik_options,
            expected={"final": final_state, "approach": approach_state},
            source=source,
        )
        rel = os.path.relpath(path, os.path.dirname(SCRIPT_DIR))
        print(f"RSIKKeyframe: capture saved -> {rel}")
        return path
    except Exception as exc:
        print(f"RSIKKeyframe: failed to write capture ({exc}); continuing.")
        return None


def _collect_target_context(
    target_bar_id,
    left_male_oid,
    left_tool_oid,
    right_male_oid,
    right_tool_oid,
):
    # ---- UX: focus the canvas on this bar (hide unbuilt + inactive tools).
    show_sequence_colors(target_bar_id, show_unbuilt=False)
    extra_hidden_tools = _hide_inactive_tool_blocks(target_bar_id)

    # tool0 (flange frame) IS the tool block instance world transform.
    tool0_left_final = _block_instance_xform_mm(left_tool_oid)
    tool0_right_final = _block_instance_xform_mm(right_tool_oid)
    # OCF (object-coordinate-frame) of the male blocks - still saved in the
    # capture for headless replay parity.
    ocf_left = _block_instance_xform_mm(left_male_oid)
    ocf_right = _block_instance_xform_mm(right_male_oid)
    print(
        f"RSIKKeyframe: target Ln bar = {target_bar_id} "
        f"(left tool = {rs.BlockInstanceName(left_tool_oid)}, "
        f"right tool = {rs.BlockInstanceName(right_tool_oid)})."
    )
    return extra_hidden_tools, tool0_left_final, tool0_right_final, ocf_left, ocf_right


def _prepare_collision_template_state(
    rcell,
    planner,
    template_state,
    target_bar_id,
    left_tool_oid,
    right_tool_oid,
):
    # ---- Per-arm tool collision rigid bodies (one-shot per cell).
    left_collision_path, right_collision_path = _resolve_tool_collision_paths(
        left_tool_oid, right_tool_oid
    )
    arm_tool_rb_names = robot_cell.attach_arm_tool_rigid_bodies(
        rcell,
        planner,
        left_collision_path=left_collision_path,
        right_collision_path=right_collision_path,
        native_scale=0.001,
    )
    robot_cell.configure_arm_tool_rigid_body_states(template_state, arm_tool_rb_names)

    env_geom = env_collision.collect_built_geometry(target_bar_id, get_bar_seq_map())
    # Include the active bar + its joints so they are visible in the cell
    # preview AND included in collision checks (matches ShowIK behavior).
    active_geom = env_collision.collect_active_geometry(target_bar_id, get_bar_seq_map())
    env_geom.update(active_geom)
    robot_cell.ensure_env_registered(rcell, env_geom, planner)
    template_state = env_collision.build_env_state(template_state, env_geom)
    # `build_env_state` returns a fresh copy; re-apply tool-RB attachments.
    robot_cell.configure_arm_tool_rigid_body_states(template_state, arm_tool_rb_names)
    # Whitelist intentional design contacts: active joint <-> mating env joint,
    # active bodies <-> arm tool RBs.
    env_collision.configure_active_assembly_acm(template_state, arm_tool_rb_names)
    print(f"RSIKKeyframe: env collision -- {env_collision.list_env_summary(env_geom)}")
    return template_state, env_geom


def _compute_approach_targets(tool0_left_final, tool0_right_final):
    # Approach target: translate tool0 frames along -avg(tool z) * LM_DISTANCE.
    # Tool block local +Z points OUT of the flange toward the joint, so -Z is
    # the retreat direction. (Sign convention: validate visually on first run.)
    z_avg_pre = (tool0_left_final[:3, 2] + tool0_right_final[:3, 2]) / 2.0
    try:
        approach_dir_pre = -_unit(z_avg_pre)
    except ValueError as exc:
        raise RuntimeError("Tool z-axes sum to zero; cannot derive approach direction.") from exc
    offset_pre = approach_dir_pre * float(config.LM_DISTANCE)
    tool0_left_approach = _translate_frame(tool0_left_final, offset_pre)
    tool0_right_approach = _translate_frame(tool0_right_final, offset_pre)
    return tool0_left_approach, tool0_right_approach


def _resolve_seed_base_frame(
    planner,
    template_state,
    saved_base,
    seed_base_frame,
    brep_id,
    heading_mm,
    allow_saved_base_prompt,
):
    if seed_base_frame is not None:
        return seed_base_frame, brep_id, heading_mm, allow_saved_base_prompt

    if allow_saved_base_prompt and saved_base is not None:
        # Default mesh-mode for the preview matches the user's last choice.
        preview_mode = ik_viz.get_mesh_mode()
        _preview_robot_at_base(planner, template_state, saved_base, preview_mode)
        answer = _ask_reuse_saved_base()
        if answer is None:
            print("RSIKKeyframe: cancelled at base-frame reuse prompt.")
            ik_viz.end_session()
            return None
        if answer:
            print("RSIKKeyframe: reusing saved base frame (skipping walkable-ground pick).")
            seed_base_frame = saved_base
            heading_mm = _heading_mm_from_base_frame(saved_base)
            # brep_id stays None -> sampling fallback is disabled in this run.
        ik_viz.end_session()

    if seed_base_frame is None:
        brep_id = _pick_walkable_brep()
        if brep_id is None:
            return None
        _base_origin_mm, _base_normal, heading_mm, seed_base_frame = (
            _pick_base_frame_on_walkable(brep_id)
        )
        if seed_base_frame is None:
            return None

    return seed_base_frame, brep_id, heading_mm, allow_saved_base_prompt


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSIKKeyframe")

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first.", 0, "RSIKKeyframe"
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()
    # Use the module-level wrapper so tool_states[AssemblyLeftTool/AssemblyRightTool] get attached_to_group
    # + touch_links wired up. `rcell.default_cell_state()` alone returns a
    # state with tools un-attached, defeating the IK collision check on tools.
    template_state = robot_cell.default_cell_state()

    rs.UnselectAllObjects()
    picked = _pick_bar_with_arm_tools()
    if picked is None:
        return
    target_bar_id, target_bar_oid, (left_male_oid, left_tool_oid), (right_male_oid, right_tool_oid) = picked

    extra_hidden_tools, tool0_left_final, tool0_right_final, ocf_left, ocf_right = _collect_target_context(
        target_bar_id,
        left_male_oid,
        left_tool_oid,
        right_male_oid,
        right_tool_oid,
    )
    template_state, env_geom = _prepare_collision_template_state(
        rcell,
        planner,
        template_state,
        target_bar_id,
        left_tool_oid,
        right_tool_oid,
    )

    env_token = None
    keep_highlight = False
    try:
        sc.doc.Views.Redraw()

        env_token = highlight_env.highlight_env_for_ik(target_bar_id)

        collision_opts = _ask_collision_options(env_count=len(env_geom))
        if collision_opts is None:
            return
        include_self, include_env, mesh_mode = collision_opts
        try:
            tool0_left_approach, tool0_right_approach = _compute_approach_targets(
                tool0_left_final,
                tool0_right_final,
            )
        except RuntimeError as exc:
            rs.MessageBox(str(exc), 0, "RSIKKeyframe")
            return

        seed_base_frame = None
        brep_id = None
        heading_mm = None
        saved_base = _read_saved_assembly_base_frame(target_bar_oid)
        allow_saved_base_prompt = True

        while True:
            base_resolution = _resolve_seed_base_frame(
                planner,
                template_state,
                saved_base,
                seed_base_frame,
                brep_id,
                heading_mm,
                allow_saved_base_prompt,
            )
            if base_resolution is None:
                return
            seed_base_frame, brep_id, heading_mm, allow_saved_base_prompt = base_resolution

            # Persist the base frame ASAP so a Ctrl+C mid-IK still leaves it on
            # the bar for the next run's reuse path.
            _write_assembly_base_frame(target_bar_oid, seed_base_frame)

            # ---- IK + viewport-redraw lock around the solve+preview block.
            rs.EnableRedraw(False)
            try:
                print("RSIKKeyframe: solving final-target IK...")
                final_state, final_base = _solve_with_sampling(
                    planner, template_state, seed_base_frame,
                    tool0_left_final, tool0_right_final,
                    brep_id, heading_mm, include_self, include_env,
                )
                if final_state is None:
                    rs.EnableRedraw(True)
                    if _ask_save_failure_capture("Final-target"):
                        seed_state = template_state.copy()
                        robot_cell._apply_base_frame_mm(seed_state, seed_base_frame)
                        _save_capture(
                            target_bar_id=target_bar_id,
                            left_oid=left_male_oid,
                            right_oid=right_male_oid,
                            ocf_left=ocf_left,
                            ocf_right=ocf_right,
                            tool0_left_final=tool0_left_final,
                            tool0_right_final=tool0_right_final,
                            tool0_left_approach=tool0_left_approach,
                            tool0_right_approach=tool0_right_approach,
                            initial_state=seed_state,
                            include_self=include_self,
                            include_env=include_env,
                            final_state=None,
                            approach_state=None,
                            rcell=rcell,
                            suffix="_ik_fail_final",
                        )
                    rs.MessageBox("IK failed for the final target (all samples exhausted).", 0, "RSIKKeyframe")
                    action = _ask_accept(
                        "Final-target IK failed. Retry the same base, retry a new base, or give up",
                        allow_accept=False,
                    )
                    if action == "retry_same_base":
                        print("RSIKKeyframe: retrying final-target IK with the same robot base frame.")
                        saved_base = seed_base_frame
                        brep_id = _resolve_sampling_brep_for_base(seed_base_frame, brep_id)
                        allow_saved_base_prompt = False
                        ik_viz.end_session()
                        continue
                    if action == "retry_new_base":
                        print("RSIKKeyframe: retrying final-target IK with a different robot base frame.")
                        saved_base = seed_base_frame
                        seed_base_frame = None
                        heading_mm = None
                        brep_id = None
                        allow_saved_base_prompt = False
                        ik_viz.end_session()
                        continue
                    print("RSIKKeyframe: gave up after final-target IK failure.")
                    return
                # Sync the PyBullet world to the new state, then update the Rhino viz.
                robot_cell.set_cell_state(planner, final_state)
                rs.EnableRedraw(True)
                ik_viz.update_state(final_state, layer_key=ik_viz.LAYER_KEY_ASSEMBLY)
                ik_viz.set_active_mesh_mode(ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode)
                print("RSIKKeyframe: final target reachable. Previewing...")

                rs.EnableRedraw(False)
                print("RSIKKeyframe: solving approach-target IK...")
                approach_state, approach_base = _solve_with_sampling(
                    planner, template_state, final_base,
                    tool0_left_approach, tool0_right_approach,
                    brep_id, heading_mm, include_self, include_env,
                )
                if approach_state is None:
                    rs.EnableRedraw(True)
                    if _ask_save_failure_capture("Approach-target"):
                        seed_state = template_state.copy()
                        robot_cell._apply_base_frame_mm(seed_state, final_base)
                        _save_capture(
                            target_bar_id=target_bar_id,
                            left_oid=left_male_oid,
                            right_oid=right_male_oid,
                            ocf_left=ocf_left,
                            ocf_right=ocf_right,
                            tool0_left_final=tool0_left_final,
                            tool0_right_final=tool0_right_final,
                            tool0_left_approach=tool0_left_approach,
                            tool0_right_approach=tool0_right_approach,
                            initial_state=seed_state,
                            include_self=include_self,
                            include_env=include_env,
                            final_state=final_state,
                            approach_state=None,
                            rcell=rcell,
                            suffix="_ik_fail_approach",
                        )
                    rs.MessageBox("IK failed for the approach target (all samples exhausted).", 0, "RSIKKeyframe")
                    action = _ask_accept(
                        "Approach-target IK failed. Retry the same base, retry a new base, or give up",
                        allow_accept=False,
                    )
                    if action == "retry_same_base":
                        print("RSIKKeyframe: retrying approach-target IK with the same robot base frame.")
                        _write_assembly_base_frame(target_bar_oid, final_base)
                        saved_base = final_base
                        seed_base_frame = final_base
                        heading_mm = _heading_mm_from_base_frame(final_base)
                        brep_id = _resolve_sampling_brep_for_base(final_base, brep_id)
                        allow_saved_base_prompt = False
                        ik_viz.end_session()
                        continue
                    if action == "retry_new_base":
                        print("RSIKKeyframe: retrying approach-target IK with a different robot base frame.")
                        _write_assembly_base_frame(target_bar_oid, final_base)
                        saved_base = final_base
                        seed_base_frame = None
                        heading_mm = None
                        brep_id = None
                        allow_saved_base_prompt = False
                        ik_viz.end_session()
                        continue
                    print("RSIKKeyframe: gave up after approach-target IK failure.")
                    return
                robot_cell.set_cell_state(planner, approach_state)
                rs.EnableRedraw(True)
                ik_viz.update_state(approach_state, layer_key=ik_viz.LAYER_KEY_ASSEMBLY)
                ik_viz.set_active_mesh_mode(ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode)
                print("RSIKKeyframe: approach target reachable. Previewing...")
            finally:
                rs.EnableRedraw(True)

            # Use the same base frame for the stored record (approach_base ~ final_base since we reused).
            # If sampling drifted, prefer final_base (anchors the assembled pose).
            stored_base = final_base
            action = _ask_accept(
                "IK preview ready. Accept, retry the same base, retry a new base, or give up"
            )

            if action == "accept":
                # Re-write base frame (final_base may differ from seed_base_frame
                # if sampling fallback found a better location).
                _write_assembly_base_frame(target_bar_oid, stored_base)
                _write_assembly_keyframes(target_bar_oid, final_state, approach_state, rcell)
                _write_legacy_assembly_blob(target_bar_oid, stored_base, final_state, approach_state, rcell)
                # Capture file is a debug artefact (not production state - production
                # data lives in the bar user-text above). Opt-in, symmetric with the
                # failure path. Default No.
                if _ask_save_debug_capture():
                    seed_state = template_state.copy()
                    robot_cell._apply_base_frame_mm(seed_state, stored_base)
                    _save_capture(
                        target_bar_id=target_bar_id,
                        left_oid=left_male_oid,
                        right_oid=right_male_oid,
                        ocf_left=ocf_left,
                        ocf_right=ocf_right,
                        tool0_left_final=tool0_left_final,
                        tool0_right_final=tool0_right_final,
                        tool0_left_approach=tool0_left_approach,
                        tool0_right_approach=tool0_right_approach,
                        initial_state=seed_state,
                        include_self=include_self,
                        include_env=include_env,
                        final_state=final_state,
                        approach_state=approach_state,
                        rcell=rcell,
                    )
                keep_highlight = True
                break

            if action == "retry_same_base":
                print("RSIKKeyframe: retrying IK with the same robot base frame.")
                _write_assembly_base_frame(target_bar_oid, stored_base)
                saved_base = stored_base
                seed_base_frame = stored_base
                heading_mm = _heading_mm_from_base_frame(stored_base)
                brep_id = _resolve_sampling_brep_for_base(stored_base, brep_id)
                allow_saved_base_prompt = False
                ik_viz.end_session()
                continue

            if action == "retry_new_base":
                print("RSIKKeyframe: retrying IK with a different robot base frame.")
                saved_base = stored_base
                seed_base_frame = None
                heading_mm = None
                brep_id = None
                allow_saved_base_prompt = False
                ik_viz.end_session()
                continue

            print("RSIKKeyframe: gave up; bar keyframes unchanged (base frame still saved for next run).")
            return

    finally:
        rs.EnableRedraw(True)
        ik_viz.end_session()
        if env_token is not None and not keep_highlight:
            highlight_env.revert_env_highlight(env_token)
        # Restore canvas exactly like RSSequenceEdit exit path.
        try:
            _show_objects(extra_hidden_tools)
            reset_sequence_colors()
        except Exception as exc:  # noqa: BLE001 -- never let cleanup mask the real outcome
            print(f"RSIKKeyframe: failed to restore sequence colors ({exc}); continuing.")


if __name__ == "__main__":
    main()
