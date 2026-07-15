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

Right after the base point + heading are chosen (step 2), the command offers an
off-ramp: continue the in-Rhino solve as normal, or just save the base frame on
the bar and exit. The exit path is for the "indicate the base pose in Rhino,
solve the keyframes headlessly" workflow -- the saved base is then picked up by
``headless_bar_action_planner.py --solve-keyframes --base saved`` (export the
bar first). Pressing Enter continues the in-Rhino solve, so the default flow is
unchanged.
"""

from __future__ import annotations

import importlib
import json
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import bar_action as _bar_action_module
from core import config as _config_module
from core import dynamic_preview as _dynamic_preview_module
from core import highlight_env as _highlight_env_module
from core import ik_keyframe as _ik_keyframe_module
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
from core.rhino_helpers import suspend_redraw
from core.rhino_tool_place import find_tool_for_joint
from core.robotic_tool import get_robotic_tool
# Base-frame math shared with the headless sampler. These are pure numpy (no
# Rhino), so they live in `core.walkable_ground` and are imported under the
# private names this script already uses at its call sites.
from core.walkable_ground import (
    frame_from_origin_normal_heading as _frame_from_origin_normal_heading,
    sample_base_offsets as _sample_base_offsets,
)


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
    """Re-import the core runtime modules so edits take effect without restarting Rhino.

    Rebinds the module-level globals (``config``, ``robot_cell``, ``ik_keyframe``,
    etc.) to freshly reloaded copies. Matches the reload pattern used by
    ``rs_joint_place.py`` so a saved change to any shared solve-path module is
    picked up on the next command run.

    Returns:
        None.
    """
    global bar_action, config, dynamic_preview, highlight_env, ik_keyframe, ik_viz, robot_cell
    config = importlib.reload(_config_module)
    dynamic_preview = importlib.reload(_dynamic_preview_module)
    highlight_env = importlib.reload(_highlight_env_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)
    # bar_action builds the M1-M4 movements (collision context + EE targets);
    # ik_keyframe solves the chained IK against them. Reload both so edits to the
    # shared solve path take effect without restarting Rhino.
    bar_action = importlib.reload(_bar_action_module)
    ik_keyframe = importlib.reload(_ik_keyframe_module)


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Rhino <-> numpy helpers (doc units -> mm)
# ---------------------------------------------------------------------------


def _rhino_xform_to_np_mm(xform):
    """Convert a Rhino transform to a 4x4 numpy matrix with translation in mm.

    Args:
        xform (Rhino.Geometry.Transform): a document-unit transform.

    Returns:
        np.ndarray: the same transform as a 4x4 float matrix, with the
        translation column scaled from document units into millimeters.
    """
    scale = doc_unit_scale_to_mm()
    matrix = np.array([[float(xform[i, j]) for j in range(4)] for i in range(4)], dtype=float)
    matrix[:3, 3] *= scale  # translation -> mm
    return matrix


def _np_mm_to_rhino_xform(matrix: np.ndarray):
    """Convert a 4x4 mm numpy matrix back into a document-unit Rhino transform.

    Inverse of :func:`_rhino_xform_to_np_mm`: the translation column is scaled
    from millimeters back into the document's units.

    Args:
        matrix (np.ndarray): a 4x4 transform with translation in mm.

    Returns:
        Rhino.Geometry.Transform: the equivalent transform in document units.
    """
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
    """Convert a point to a length-3 numpy array in millimeters.

    Accepts either a Rhino point (with ``.X`` / ``.Y`` / ``.Z``) or any
    3-element sequence; the coordinates are scaled from document units to mm.

    Args:
        point: a ``Rhino.Geometry.Point3d`` or a 3-element (x, y, z) sequence.

    Returns:
        np.ndarray: the point as ``[x, y, z]`` floats in millimeters.
    """
    scale = doc_unit_scale_to_mm()
    if hasattr(point, "X"):
        return np.array([point.X, point.Y, point.Z], dtype=float) * scale
    return np.asarray(point, dtype=float) * scale


def _unit(vector: np.ndarray) -> np.ndarray:
    """Return the unit-length version of ``vector``.

    Args:
        vector (np.ndarray): any non-zero vector.

    Returns:
        np.ndarray: ``vector`` scaled to length 1.

    Raises:
        ValueError: if ``vector`` is (near) zero length and cannot be unitized.
    """
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("Cannot unitize a zero-length vector.")
    return np.asarray(vector, dtype=float) / norm


# ---------------------------------------------------------------------------
# Block / layer helpers
# ---------------------------------------------------------------------------


def _has_block_definition(name) -> bool:
    """Return True if a live block definition named ``name`` exists in the document.

    Args:
        name (str): the block definition name to look for.

    Returns:
        bool: True if a non-deleted instance definition with that name exists.
    """
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
    """Return the surface / polysurface / extrusion object ids on a layer.

    Creates the layer if it does not exist yet (and then returns an empty list).

    Args:
        layer_name (str): the layer to scan.

    Returns:
        list: Rhino object ids on that layer whose type is surface, polysurface
        or extrusion; other object types are skipped.
    """
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
        return []
    accepted = {rs.filter.surface, rs.filter.polysurface, rs.filter.extrusion}
    return [
        oid for oid in rs.ObjectsByLayer(layer_name) or []
        if rs.ObjectType(oid) in accepted
    ]


def _pick_walkable_brep():
    """Resolve the WalkableGround brep that base frames will be sampled on.

    If the WalkableGround layer holds exactly one brep it is returned directly;
    otherwise the user is prompted to pick one. A message box is shown (and None
    returned) when the layer is empty or the picked object is on the wrong layer.

    Returns:
        Guid or None: the chosen brep / extrusion object id, or None on cancel
        or an invalid pick.
    """
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
    """Pick the WalkableGround brep the seed base frame sits closest to.

    Used on the retry-same-base path, where ``brep_id`` may have been dropped
    (reuse path). If a brep is already known it is returned unchanged; with a
    single candidate that one is used; otherwise every candidate is tested by
    snapping the seed origin onto it and the nearest one wins.

    Args:
        seed_base_frame_mm (np.ndarray): 4x4 mm base frame whose origin is matched.
        brep_id: an already-resolved brep object id, or None to search.

    Returns:
        Guid or None: the best-matching brep object id, or None if the layer
        holds no usable breps.
    """
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
            """Ghost-robot transform while picking the base origin.

            Snaps the cursor onto the brep and builds a base frame there with a
            temporary world +X heading (the real heading is chosen in phase B).

            Args:
                cursor_doc (Rhino.Geometry.Point3d): the live cursor point.

            Returns:
                Rhino.Geometry.Transform or None: the base frame at the snapped
                point, or None when the cursor is off the brep.
            """
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
            """Ghost-robot transform while picking the heading point.

            Keeps the origin fixed at the phase-A pick and aims the base +X from
            that origin toward the current cursor.

            Args:
                cursor_doc (Rhino.Geometry.Point3d): the live cursor point.

            Returns:
                Rhino.Geometry.Transform or None: the base frame with +X toward
                the cursor, or None if the heading is degenerate.
            """
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
    """Prompt the user to accept the current keyframe or retry / give up.

    Enter (accept-nothing) defaults to retrying the same base. When
    ``allow_accept`` is False the ``Accept`` option is hidden -- used on failure
    prompts where there is nothing to accept.

    Args:
        prompt (str): the command prompt text.
        allow_accept (bool): whether to offer the ``Accept`` option.

    Returns:
        str: one of ``"accept"`` / ``"retry_same_base"`` / ``"retry_new_base"``
        / ``"give_up"``.
    """
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


def _render_preview_pose(planner, state, mesh_mode):
    """Render one solved keyframe pose on the cached assembly-layer meshes.

    Shared by the first assembled-pose preview and the ``TogglePose`` cycle in
    :func:`_ask_accept_with_pose_preview`. Poses the planner's PyBullet cell to
    ``state``, delta-updates the cached Rhino robot meshes to match, and forces
    the chosen mesh mode (visual / collision) visible.

    Args:
        planner: The dual-arm planner whose cell is posed to ``state``.
        state (RobotCellState): The keyframe pose to show.
        mesh_mode (str): ``ik_viz.MESH_MODE_VISUAL`` or ``ik_viz.MESH_MODE_COLLISION``.

    Returns:
        None.
    """
    robot_cell.set_cell_state(planner, state)
    rs.EnableRedraw(False)
    try:
        ik_viz.update_state(state, layer_key=ik_viz.LAYER_KEY_ASSEMBLY)
        ik_viz.set_active_mesh_mode(ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode)
    finally:
        rs.EnableRedraw(True)
        sc.doc.Views.Redraw()


def _ask_accept_with_pose_preview(
    display_frames,
    render_fn,
    initial_index: int = 0,
    prompt: str = "Accept this IK keyframe and save it on the bar",
):
    """Accept/retry prompt that also cycles through the solved keyframes.

    Behaves like :func:`_ask_accept` (Accept / RetrySameBase / RetryNewBase /
    GiveUp) but adds a ``TogglePose`` option -- also bound to Enter -- that steps
    through ``display_frames`` and re-renders each pose via ``render_fn`` without
    leaving the prompt. This mirrors RSShowIK's Enter-to-cycle behaviour so the
    user can inspect every keyframe (approach / assembled / retreat / home)
    before deciding, instead of only seeing the assembled pose.

    Args:
        display_frames (list): ``[(label, state), ...]`` keyframes to cycle.
        render_fn (callable): ``render_fn(state)`` -- pushes a state to the
            planner + Rhino preview (see :func:`_render_preview_pose`).
        initial_index (int): index of the pose the caller already rendered; the
            cycle continues from here so the first Enter advances to the next one.
        prompt (str): base command prompt; the current pose label is appended.

    Returns:
        str: one of ``"accept"`` / ``"retry_same_base"`` / ``"retry_new_base"`` /
        ``"give_up"``.
    """
    idx = initial_index % len(display_frames)

    def _advance():
        """Step to the next keyframe, render it, and echo which one is showing."""
        nonlocal idx
        idx = (idx + 1) % len(display_frames)
        label, state = display_frames[idx]
        render_fn(state)
        print(f"RSIKKeyframe: previewing '{label}' keyframe.")

    while True:
        label = display_frames[idx][0]
        go = Rhino.Input.Custom.GetOption()
        go.SetCommandPrompt(f"{prompt} [now: {label}; Enter/TogglePose to cycle keyframes]")
        accept_idx = go.AddOption("Accept")
        toggle_idx = go.AddOption("TogglePose")
        retry_same_idx = go.AddOption("RetrySameBase")
        retry_new_idx = go.AddOption("RetryNewBase")
        give_up_idx = go.AddOption("GiveUp")
        go.AcceptNothing(True)

        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            # Enter cycles to the next keyframe (RSShowIK parity).
            _advance()
            continue
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == accept_idx:
                return "accept"
            if chosen == toggle_idx:
                _advance()
                continue
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


def _ask_save_base_or_continue():
    """After the base pose is set, ask whether to solve IK now or just save + exit.

    This is the off-ramp for the "indicate the base pose in Rhino, solve the
    keyframes headlessly" workflow: the robot base frame has just been written on
    the bar, so choosing to stop here leaves that base on the bar and skips the
    slow in-Rhino IK chain solve. The keyframe IK can then be solved outside Rhino
    with ``headless_bar_action_planner.py --solve-keyframes --base saved`` (it
    reads exactly this saved base). Enter / the default is ``Continue`` so the
    normal solve-in-Rhino path is unchanged for anyone who just presses Enter.

    Returns:
        str: ``"continue"`` to solve the IK chain in Rhino now, ``"save_and_exit"``
        to keep the already-saved base and stop, or ``"cancel"`` on Esc.
    """
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(
        "Base frame set -- Continue to the in-Rhino IK solve, or save the base "
        "and exit (to solve the keyframes headlessly)"
    )
    continue_idx = go.AddOption("Continue")
    save_exit_idx = go.AddOption("SaveBaseAndExit")
    go.SetCommandPromptDefault("Continue")
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return "continue"
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == continue_idx:
                return "continue"
            if chosen == save_exit_idx:
                return "save_and_exit"
            continue
        return "cancel"


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
    """Un-hide every object id in ``oids`` (skipping ids that no longer exist).

    Args:
        oids (list): Rhino object ids to show, e.g. those returned by
            :func:`_hide_inactive_tool_blocks`. None is treated as empty.

    Returns:
        None.
    """
    for oid in oids or []:
        if rs.IsObject(oid):
            rs.ShowObject(oid)


# ---------------------------------------------------------------------------
# IK solving with base sampling
# ---------------------------------------------------------------------------


def _frame_mm_to_doc_marker(frame_mm):
    """Return ``(origin_doc, x_axis_doc, z_axis_doc)`` from a 4x4 mm frame.

    Origin is scaled into doc units; axes are unit vectors so they pass through
    untouched. Used to feed the IKSampleVizConduit which draws in doc units.
    """
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    origin_doc = Rhino.Geometry.Point3d(
        float(frame_mm[0, 3]) * scale_from_mm,
        float(frame_mm[1, 3]) * scale_from_mm,
        float(frame_mm[2, 3]) * scale_from_mm,
    )
    x_axis_doc = Rhino.Geometry.Vector3d(
        float(frame_mm[0, 0]), float(frame_mm[1, 0]), float(frame_mm[2, 0])
    )
    z_axis_doc = Rhino.Geometry.Vector3d(
        float(frame_mm[0, 2]), float(frame_mm[1, 2]), float(frame_mm[2, 2])
    )
    return origin_doc, x_axis_doc, z_axis_doc


def _open_ik_sample_viz(seed_base_frame_mm, sample_radius_mm):
    """Create + enable an IK-sampling viz conduit and draw the seed marker + circle.

    Draws seed origin + X-axis arrow + sampling circle in the seed's tangent plane
    immediately; the solver loop then pushes ghost-robot xforms and per-attempt
    markers through the returned conduit.

    Unlike a ``with`` context, the conduit is NOT auto-disabled -- the caller owns
    its lifetime and must call :func:`_close_ik_sample_viz`. This lets the circle,
    the per-attempt sample markers, and the seed/sample X-axis arrows stay visible
    AFTER the solve finishes (through the accept/retry prompt) so the user can see
    which bases were tried, instead of them vanishing the instant the solve returns.

    Redraw is forced on so the conduit renders even when the surrounding command has
    suspended redraws; the caller restores the redraw state.
    """
    rs.EnableRedraw(True)
    robot_meshes = ik_viz.get_robot_link_meshes_at_zero(layer_key=ik_viz.LAYER_KEY_ASSEMBLY)
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    arrow_len_mm = max(50.0, 0.4 * float(sample_radius_mm))
    origin_doc, x_axis_doc, z_axis_doc = _frame_mm_to_doc_marker(seed_base_frame_mm)
    conduit = dynamic_preview.IKSampleVizConduit(robot_meshes, alpha=0.35)
    conduit.Enabled = True
    conduit.set_seed_and_circle(
        origin_doc,
        x_axis_doc,
        z_axis_doc,
        float(sample_radius_mm) * scale_from_mm,
        arrow_len_mm * scale_from_mm,
    )
    return conduit


def _close_ik_sample_viz(conduit):
    """Disable a conduit opened by :func:`_open_ik_sample_viz` (no-op if ``None``)."""
    if conduit is not None:
        conduit.Enabled = False
        sc.doc.Views.Redraw()


def _snap_to_brep(brep, origin_mm):
    """Project a millimeter point onto the closest point of a brep.

    Args:
        brep (Rhino.Geometry.Brep): the surface to snap onto.
        origin_mm (np.ndarray): the point (mm) to project.

    Returns:
        tuple: ``(snapped_origin_mm, normal)`` where ``snapped_origin_mm`` is a
        length-3 mm array and ``normal`` is the surface normal there as a
        length-3 array; ``(None, None)`` if no closest point was found.
    """
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    pt_doc = Rhino.Geometry.Point3d(*(origin_mm * scale_from_mm))
    close_pt, normal = _closest_point_on_brep(brep, pt_doc)
    if close_pt is None:
        return None, None
    return _point_to_mm(close_pt), np.array([normal.X, normal.Y, normal.Z], dtype=float)


def _solve_chain_with_sampling(planner, movements, seed_base_frame_mm,
                               brep_id, heading_mm, include_self, include_env,
                               viz=None):
    """Solve the M1->M2->M3 IK chain; sample base frames around the seed on failure.

    A base frame is accepted only when the WHOLE chain solves (approach ->
    assembled -> retreat). The chain itself is solved by
    ``core.ik_keyframe.solve_keyframe_chain`` -- one place, shared with the
    headless test. This wrapper only adds the Rhino-specific base search: try the
    seed base, then sample offsets on the WalkableGround brep and re-snap each.

    If ``brep_id`` is None (reuse-saved-base path) the sampling fallback is
    disabled and only the seed base is tried; the caller re-prompts for a fresh
    base on the next run.

    When ``viz`` is an ``IKSampleVizConduit`` (see ``core.dynamic_preview``), the
    ghost robot is positioned at each attempt's base frame before the IK call,
    and a failed/succeeded sample marker is appended after each attempt (idx 0,
    the seed, is omitted because the seed marker already covers it).

    Args:
        planner: the dual-arm planner.
        movements (dict): ``{"M1": .., "M2": .., "M3": ..}`` from
            ``bar_action.build_assembly_movements``.
        seed_base_frame_mm (np.ndarray): 4x4 mm base frame to try first.
        brep_id: WalkableGround brep oid the samples snap to, or None to disable
            sampling.
        heading_mm (np.ndarray): heading point (mm) the sampled frames keep their
            +X toward.
        include_self, include_env (bool): collision toggles; their OR drives
            ``check_collision``.
        viz: optional ``IKSampleVizConduit`` for live base-sampling visualization
            (ghost robot + per-attempt markers), or None to disable it.

    Returns:
        tuple: ``(solved_states, used_base_frame_mm)`` on success where
        ``solved_states`` is ``{"M1": state, "M2": state, "M3": state}``, or
        ``(None, None)`` if no sampled base solves the whole chain.
    """
    check_collision = bool(include_self or include_env)
    ordered = [("M1", movements["M1"]), ("M2", movements["M2"]), ("M3", movements["M3"])]

    # Candidate base frames: the seed first, then samples snapped to the brep.
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

    # Force redraws on while the viz is driving the ghost preview; many callers
    # wrap this function in `rs.EnableRedraw(False)` which otherwise would
    # short-circuit `sc.doc.Views.Redraw()` from the conduit's setters.
    prev_redraw = rs.EnableRedraw(True) if viz is not None else None

    total = len(attempts)
    try:
        for idx, base_frame in enumerate(attempts):
            label = "seed" if idx == 0 else f"sample {idx}/{total - 1}"
            origin = base_frame[:3, 3]
            print(
                f"RSIKKeyframe: trying base frame ({label}) "
                f"at ({origin[0]:.1f}, {origin[1]:.1f}, {origin[2]:.1f}) mm ..."
            )
            if viz is not None:
                viz.set_ghost_xform(_np_mm_to_rhino_xform(base_frame))
            solved = ik_keyframe.solve_keyframe_chain(
                planner,
                ordered,
                base_frame,
                check_collision=check_collision,
                verbose_pairs=check_collision,
            )
            if viz is not None and idx > 0:
                sample_origin_doc, sample_x_doc, _z = _frame_mm_to_doc_marker(base_frame)
                viz.add_tried(sample_origin_doc, sample_x_doc, success=solved is not None)
            if solved is not None:
                print(
                    f"RSIKKeyframe: [OK] M1->M2->M3 IK chain solved on attempt {idx + 1}/{total} ({label})."
                )
                return solved, base_frame
            print(f"RSIKKeyframe: [x] chain IK failed on attempt {idx + 1}/{total} ({label}).")
    finally:
        if prev_redraw is not None:
            rs.EnableRedraw(prev_redraw)
    print(
        f"RSIKKeyframe: [X] chain IK failed for all {total} attempt(s). "
        f"Consider increasing IK_BASE_SAMPLE_RADIUS / IK_BASE_SAMPLE_MAX_ITER in config.py."
    )
    return None, None


# ---------------------------------------------------------------------------
# Payload
# ---------------------------------------------------------------------------


def _build_assembly_payload(base_frame_mm, final_state, approach_state, rcell):
    """Assemble the legacy bundled ``ik_assembly`` payload dict.

    Bundles the robot base frame plus the per-arm joint configs for the final
    (assembled) and approach poses, matching the schema older readers expect.

    Args:
        base_frame_mm (np.ndarray): 4x4 mm robot base frame.
        final_state (RobotCellState): the assembled-pose cell state.
        approach_state (RobotCellState): the approach-pose cell state.
        rcell (RobotCell): used to name the per-group joints.

    Returns:
        dict: the JSON-serializable ``ik_assembly`` payload.
    """
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
    """Extract the left + right arm joint configs from a cell state.

    Args:
        state (RobotCellState): the solved cell state to read joint values from.
        rcell (RobotCell): used to name the per-group joints.

    Returns:
        dict: ``{"left": <left group config>, "right": <right group config>}``.
    """
    return {
        "left": robot_cell.extract_group_config(state, config.LEFT_GROUP, rcell),
        "right": robot_cell.extract_group_config(state, config.RIGHT_GROUP, rcell),
    }


def _write_assembly_base_frame(bar_oid, base_frame_mm):
    """Save the robot base frame on the bar as JSON user-text.

    Args:
        bar_oid: Rhino object id of the bar curve.
        base_frame_mm (np.ndarray): 4x4 mm base frame stored under
            ``config.KEY_ASSEMBLY_BASE_FRAME``.

    Returns:
        None.
    """
    payload = np.asarray(base_frame_mm, dtype=float).tolist()
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME, json.dumps(payload))
    print(f"RSIKKeyframe: saved '{config.KEY_ASSEMBLY_BASE_FRAME}' on bar.")


def _write_assembly_keyframes(bar_oid, approach_state, assembled_state, retreat_state, rcell):
    """Write the three solved keyframes (approach / assembled / retreat) on the bar.

    Each is the per-arm joint config extracted from the corresponding solved
    movement state: ``approach`` = M1's solution, ``assembled`` = M2's, ``retreat``
    = M3's. They feed ``core.bar_action`` when the bar action is exported.

    Args:
        bar_oid: Rhino object id of the bar curve.
        approach_state, assembled_state, retreat_state (RobotCellState): the
            solved M1 / M2 / M3 states from ``ik_keyframe.solve_keyframe_chain``.
        rcell (RobotCell): used to name the per-group joints.

    Returns:
        None.
    """
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH,
                   json.dumps(_build_group_pair(approach_state, rcell)))
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED,
                   json.dumps(_build_group_pair(assembled_state, rcell)))
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_RETREAT,
                   json.dumps(_build_group_pair(retreat_state, rcell)))
    print(
        f"RSIKKeyframe: saved '{config.KEY_ASSEMBLY_IK_APPROACH}' + "
        f"'{config.KEY_ASSEMBLY_IK_ASSEMBLED}' + '{config.KEY_ASSEMBLY_IK_RETREAT}' on bar."
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

    Used so the sampling code in `_solve_chain_with_sampling` (which derives the
    base X from `heading - origin`) keeps the same X direction across reuse.
    """
    origin = np.asarray(base_frame_mm[:3, 3], dtype=float)
    x_axis = np.asarray(base_frame_mm[:3, 0], dtype=float)
    return origin + 100.0 * x_axis


def _collect_target_context(
    target_bar_id,
    left_tool_oid,
    right_tool_oid,
):
    """Focus the canvas on the target bar and read its two tool0 frames.

    Colors the sequence up to this bar (hiding unbuilt bars), hides tool blocks
    that belong to other bars, and reads each placed tool block instance's world
    transform -- which is tool0 (the flange frame) at the assembled pose.

    Args:
        target_bar_id (str): the bar being assembled.
        left_tool_oid: Rhino object id of the left-arm tool block instance.
        right_tool_oid: Rhino object id of the right-arm tool block instance.

    Returns:
        tuple: ``(extra_hidden_tools, tool0_left_final, tool0_right_final)`` where
        ``extra_hidden_tools`` is the list of tool oids hidden (for restore) and
        the two tool0 values are 4x4 mm matrices.
    """
    # ---- UX: focus the canvas on this bar (hide unbuilt + inactive tools).
    show_sequence_colors(target_bar_id, show_unbuilt=False)
    extra_hidden_tools = _hide_inactive_tool_blocks(target_bar_id)

    # tool0 (flange frame) IS the tool block instance world transform.
    tool0_left_final = _block_instance_xform_mm(left_tool_oid)
    tool0_right_final = _block_instance_xform_mm(right_tool_oid)
    print(
        f"RSIKKeyframe: target Ln bar = {target_bar_id} "
        f"(left tool = {rs.BlockInstanceName(left_tool_oid)}, "
        f"right tool = {rs.BlockInstanceName(right_tool_oid)})."
    )
    return extra_hidden_tools, tool0_left_final, tool0_right_final


def _resolve_seed_base_frame(
    planner,
    template_state,
    saved_base,
    seed_base_frame,
    brep_id,
    heading_mm,
    allow_saved_base_prompt,
):
    """Decide which robot base frame to seed the IK solve with.

    Called once per loop turn in ``main``. If a base frame is already in
    hand it is passed straight back. Otherwise this offers to reuse a base
    frame saved on the bar (with a cheap robot preview at that frame), and
    failing that, walks the user through picking a fresh base origin and
    heading on the WalkableGround brep.

    Args:
        planner: The dual-arm planner, used only to pose the preview robot.
        template_state: Neutral cell state copied for the reuse-base preview.
        saved_base: Base frame previously stored on the bar as a 4x4 numpy
            matrix (mm), or None if the bar has no saved frame.
        seed_base_frame: Base frame already chosen this run as a 4x4 numpy
            matrix (mm), or None to resolve a new one.
        brep_id: WalkableGround brep object id that base sampling snaps to,
            or None when sampling should stay disabled (reuse path).
        heading_mm: Heading point (mm) defining the base +X direction, or
            None to derive it during this call.
        allow_saved_base_prompt: When True, offer to reuse ``saved_base``;
            set False on retries so the prompt does not reappear.

    Returns:
        The tuple ``(seed_base_frame, brep_id, heading_mm,
        allow_saved_base_prompt)`` once a base frame is resolved, or None if
        the user cancelled at the reuse prompt or the walkable-ground pick.
    """
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
# ssik candidate inspection (debug aid on chain failure)
# ---------------------------------------------------------------------------


def _ask_chain_failure(allow_inspect: bool):
    """Prompt after a failed IK chain: inspect candidates / retry / give up.

    Like ``_ask_accept(allow_accept=False)`` but adds an ``InspectCandidates``
    option (only when the ssik backend can enumerate candidate pairs). Enter/Esc
    give up.

    Args:
        allow_inspect (bool): show the ``InspectCandidates`` option (ssik only).

    Returns:
        str: ``"inspect"`` / ``"retry_same_base"`` / ``"retry_new_base"`` /
        ``"give_up"``.
    """
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt("IK chain failed. Inspect candidates, retry the same base, retry a new base, or give up")
    inspect_idx = go.AddOption("InspectCandidates") if allow_inspect else None
    retry_same_idx = go.AddOption("RetrySameBase")
    retry_new_idx = go.AddOption("RetryNewBase")
    give_up_idx = go.AddOption("GiveUp")
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return "give_up"
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if allow_inspect and chosen == inspect_idx:
                return "inspect"
            if chosen == retry_same_idx:
                return "retry_same_base"
            if chosen == retry_new_idx:
                return "retry_new_base"
            if chosen == give_up_idx:
                return "give_up"
            continue
        return "give_up"


def _oids_for_offenders(offenders, env_geom, mesh_mode):
    """Map resolved ``(kind, name)`` offenders to the Rhino oids to red-highlight.

    ``offenders`` come already named + classified from
    ``robot_cell.enumerate_ssik_candidate_pairs`` (RigidBody names are reverse-mapped
    there, since the objects carry no ``.name``). Each is resolved to its baked
    preview geometry: robot links + tools via the cached ``ik_viz`` bundle on the
    assembly layer (for the currently-visible ``mesh_mode``), env rigid bodies
    (bars / joints) via ``env_geom[name]["source_oid"]``.

    Args:
        offenders (list): ``[(kind, name), ...]`` with ``kind`` in ``{"link",
            "tool", "rigid_body"}``.
        env_geom (dict): ``{canonical_name: body_info}`` for the active bar's bodies.
        mesh_mode (str): which mesh mode's link/tool GUIDs to color.

    Returns:
        list: Rhino object ids to highlight (may repeat; caller dedups on apply).
    """
    rcell = robot_cell.get_or_load_robot_cell()
    link_geom = ik_viz.get_link_native_geometry(rcell, ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode)
    tool_geom = ik_viz.get_tool_native_geometry(rcell, ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode)

    oids = []
    for kind, name in offenders:
        if kind == "link":
            oids.extend(link_geom.get(name, []))
        elif kind == "tool":
            oids.extend(tool_geom.get(name, []))
        else:  # rigid_body -> the real Rhino bar / joint object
            info = env_geom.get(name)
            if info and info.get("source_oid"):
                oids.append(info["source_oid"])
    return oids


def _apply_red_highlight(oids):
    """Paint `oids` red; return the oids that were actually recolored (for revert)."""
    applied = []
    with suspend_redraw():
        for oid in oids or []:
            try:
                rs.ObjectColor(oid, (255, 40, 40))
            except Exception:
                continue
            applied.append(oid)
    return applied


def _revert_red_highlight(oids):
    """Restore each oid's color source to ByLayer (undo `_apply_red_highlight`)."""
    with suspend_redraw():
        for oid in oids or []:
            try:
                rs.ObjectColorSource(oid, 0)  # 0 = ByLayer
            except Exception:
                continue


def _cycle_ssik_candidates(planner, role, candidates, env_geom):
    """Step through `candidates` in the viewport, red-highlighting each pose's offenders.

    Renders one candidate pair at a time (collision meshes, so overlaps are
    visible), highlights the colliding links / tools / bars red, and prints the
    collision summary. Enter / ``Next`` advance, ``Prev`` goes back, ``Done`` /
    Esc exit. Highlights are always cleared on exit.

    Args:
        planner: the dual-arm planner (posed to each candidate).
        role (str): the failing movement label (e.g. ``"M1"``) for prompts/logs.
        candidates (list): the ``enumerate_ssik_candidate_pairs`` candidate dicts.
        env_geom (dict): ``{canonical_name: body_info}`` for oid resolution.
    """
    mesh_mode = ik_viz.MESH_MODE_COLLISION
    idx = 0
    highlighted = []

    def _show(i):
        """Render candidate ``i``: clear the old highlight, pose it, highlight offenders.

        Args:
            i (int): index into ``candidates`` to display.

        Returns:
            list: the newly-applied highlight oids (so the next call can clear them).
        """
        _revert_red_highlight(highlighted)
        cand = candidates[i]
        _render_preview_pose(planner, cand["state"], mesh_mode)
        new_oids = _apply_red_highlight(
            _oids_for_offenders(cand["offenders"], env_geom, mesh_mode)
        )
        sc.doc.Views.Redraw()
        status = "CLEAR (no collision)" if not cand["in_collision"] else cand["summary"]
        print(f"RSIKKeyframe: [{role}] candidate {i + 1}/{len(candidates)} -- {status}")
        return new_oids

    highlighted = _show(idx)
    try:
        while True:
            cand = candidates[idx]
            tag = "clear" if not cand["in_collision"] else f"{cand['num_offending_pairs']} hit(s)"
            go = Rhino.Input.Custom.GetOption()
            go.SetCommandPrompt(
                f"Inspect {role} ssik candidates [{idx + 1}/{len(candidates)}: {tag}] "
                "-- Enter/Next to cycle, Prev to go back, Done to exit"
            )
            next_idx = go.AddOption("Next")
            prev_idx = go.AddOption("Prev")
            done_idx = go.AddOption("Done")
            go.AcceptNothing(True)
            result = go.Get()
            if result == Rhino.Input.GetResult.Nothing:
                idx = (idx + 1) % len(candidates)
                highlighted = _show(idx)
                continue
            if result == Rhino.Input.GetResult.Option:
                chosen = go.OptionIndex()
                if chosen == next_idx:
                    idx = (idx + 1) % len(candidates)
                    highlighted = _show(idx)
                    continue
                if chosen == prev_idx:
                    idx = (idx - 1) % len(candidates)
                    highlighted = _show(idx)
                    continue
                if chosen == done_idx:
                    break
                continue
            break
    finally:
        _revert_red_highlight(highlighted)
        sc.doc.Views.Redraw()


def _inspect_ssik_candidates(planner, movements, base_frame_mm, include_self, include_env, mesh_mode, env_geom):
    """Diagnose an ssik chain failure by cycling the first failing movement's IK pairs.

    Finds the first movement (M1->M2->M3) that can't solve at ``base_frame_mm``,
    enumerates ALL of its ssik branch pairs (colliding ones included), and lets the
    user step through them with the offenders highlighted -- so they can see why no
    pair is collision-free (often a modeling problem). The mesh display mode is
    restored to ``mesh_mode`` on exit.

    Args:
        planner: the dual-arm planner.
        movements (dict): the ``{"M1".."M4": Movement}`` built for this bar.
        base_frame_mm (np.ndarray): the seed base the user is on.
        include_self, include_env (bool): collision toggles (their OR drives checks).
        mesh_mode (str): the user's normal mesh mode, restored when inspection ends.
        env_geom (dict): ``{canonical_name: body_info}`` for oid resolution.
    """
    check_collision = bool(include_self or include_env)
    ordered = [("M1", movements["M1"]), ("M2", movements["M2"]), ("M3", movements["M3"])]

    print("RSIKKeyframe: locating the first movement that fails to solve at this base ...")
    failing = ik_keyframe.find_first_unsolvable_movement(
        planner, ordered, base_frame_mm, check_collision=check_collision
    )
    if failing is None:
        rs.MessageBox(
            "The chain solved on this re-run, so there are no failing candidates to "
            "inspect. (IK can be non-deterministic -- try the solve again.)",
            0, "RSIKKeyframe",
        )
        return
    role, movement = failing

    # M1/M2/M3 always carry EE targets; guard anyway so a target-less movement
    # (should never be first-failing here) reports cleanly instead of crashing.
    targets = movement.target_ee_frames or {}
    if targets.get("left") is None or targets.get("right") is None:
        rs.MessageBox(
            f"Movement '{role}' has no end-effector target to enumerate ssik "
            "candidates against; nothing to inspect.",
            0, "RSIKKeyframe",
        )
        return

    left_mm = ik_keyframe.frame_to_mm4(movement.target_ee_frames["left"])
    right_mm = ik_keyframe.frame_to_mm4(movement.target_ee_frames["right"])
    print(f"RSIKKeyframe: enumerating ssik candidate pairs for first failing movement '{role}' ...")
    result = robot_cell.enumerate_ssik_candidate_pairs(
        planner, movement.start_state, base_frame_mm, left_mm, right_mm,
    )
    branch_counts = result["branch_counts"]
    candidates = result["candidates"]
    print(
        f"RSIKKeyframe: '{role}' ssik branches -> left={branch_counts['left']}, "
        f"right={branch_counts['right']}; {len(candidates)} candidate pair(s)."
    )

    if not candidates:
        rs.MessageBox(
            f"Movement '{role}': ssik returned no branch pairs to inspect "
            f"(left={branch_counts['left']}, right={branch_counts['right']} branch(es)).\n\n"
            "An arm with 0 branches means its tool0 target is unreachable for that arm "
            "-- almost always a modeling problem (placed tool block pose or base frame).",
            0, "RSIKKeyframe",
        )
        return

    # Show collision meshes so overlaps are visible; restore the user's mode after.
    ik_viz.set_active_mesh_mode(ik_viz.LAYER_KEY_ASSEMBLY, ik_viz.MESH_MODE_COLLISION)
    try:
        _cycle_ssik_candidates(planner, role, candidates, env_geom)
    finally:
        ik_viz.set_active_mesh_mode(ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode)
        sc.doc.Views.Redraw()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    """Run the dual-arm IK keyframe command end to end.

    Reloads the runtime modules, verifies PyBullet is up, and picks a bar that
    carries the required L/R tools. Builds the M1-M4 assembly movements once,
    then loops: resolve a robot base frame (reuse saved / pick fresh), solve the
    approach -> assembled -> retreat IK chain (sampling base offsets on failure),
    preview the solved keyframes, and on Accept write the base frame + keyframes
    back onto the bar as user-text. Restores canvas colors / hidden tools on exit.

    Returns:
        None.
    """
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSIKKeyframe")

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first.", 0, "RSIKKeyframe"
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()

    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSIKKeyframe: aborted (stale collision cell).")
        return

    # Neutral state, used only to pose the robot for the reuse-saved-base preview
    # (that preview needs no IK and no per-movement collision context).
    neutral_seed_state = robot_cell.default_cell_state()

    rs.UnselectAllObjects()
    picked = _pick_bar_with_arm_tools()
    if picked is None:
        return
    # Male-joint oids are not needed past the pick (the tool blocks carry tool0).
    target_bar_id, target_bar_oid, (_, left_tool_oid), (_, right_tool_oid) = picked

    extra_hidden_tools, tool0_left_final, tool0_right_final = _collect_target_context(
        target_bar_id,
        left_tool_oid,
        right_tool_oid,
    )

    # * Build the M1-M4 movements ONCE from the cached cell + the two placed tool
    # blocks (tool0 at the assembled pose). Configs stay unsolved -- the IK chain
    # fills them in. The movement EE targets (approach / assembled / retreat) are
    # world-fixed, so the identity base frame here does not matter: every solve
    # overrides the base with the sampled frame.
    try:
        movements, env_geom = bar_action.build_assembly_movements(
            rcell, planner, target_bar_id,
            np.eye(4, dtype=float),
            tool0_left_final, tool0_right_final,
        )
    except (RuntimeError, ValueError) as exc:
        rs.MessageBox(str(exc), 0, "RSIKKeyframe")
        return

    env_token = None
    keep_highlight = False
    try:
        sc.doc.Views.Redraw()

        env_token = highlight_env.highlight_env_for_ik(target_bar_id)

        collision_opts = _ask_collision_options(env_count=len(env_geom))
        if collision_opts is None:
            return
        include_self, include_env, mesh_mode = collision_opts

        seed_base_frame = None
        brep_id = None
        heading_mm = None
        saved_base = _read_saved_assembly_base_frame(target_bar_oid)
        allow_saved_base_prompt = True

        while True:
            # A base is "freshly decided" this turn only when we don't already
            # hold one: the first turn, or after RetryNewBase cleared it. On
            # RetrySameBase seed_base_frame is still set, so the save/continue
            # off-ramp below is skipped and we go straight back into the solve.
            base_freshly_decided = seed_base_frame is None
            base_resolution = _resolve_seed_base_frame(
                planner,
                neutral_seed_state,
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

            # ! Off-ramp: with the base pose now set + saved, let the user stop
            # here and run the (slow) keyframe IK solve headlessly instead of in
            # Rhino. "Save and exit" just returns -- the base is already on the bar
            # and the headless solver reads it via `--base saved`. Only offered on a
            # freshly-decided base (not RetrySameBase); Enter = Continue keeps the
            # normal in-Rhino solve for the default path.
            if base_freshly_decided:
                # Keep a translucent ghost of the robot at the chosen base visible
                # while the user decides Continue vs SaveBaseAndExit -- it used to
                # vanish the moment the base pick finished (the pick's own
                # mesh_preview conduit closed on return).
                base_ghost = dynamic_preview.MeshPreviewConduit(
                    _bake_robot_meshes_at_zero(), alpha=0.4
                )
                base_ghost.Enabled = True
                base_ghost.update_xform(_np_mm_to_rhino_xform(seed_base_frame))
                try:
                    decision = _ask_save_base_or_continue()
                finally:
                    base_ghost.Enabled = False
                    sc.doc.Views.Redraw()
                if decision == "cancel":
                    print("RSIKKeyframe: cancelled at the base save/continue "
                          "prompt (base frame still saved on the bar).")
                    return
                if decision == "save_and_exit":
                    print(
                        f"RSIKKeyframe: base frame saved on bar '{target_bar_id}'; "
                        "skipped the in-Rhino IK solve.\n"
                        "  Next: export this bar (RSExportBarAction), then solve the "
                        "keyframes headlessly with\n"
                        "  headless_bar_action_planner.py --solve-keyframes "
                        f"--base saved --bar-action {target_bar_id}.json"
                    )
                    return

            # ---- IK + viewport-redraw lock around the solve+preview block.
            rs.EnableRedraw(False)
            # Open the base-sampling viz and keep it alive PAST the solve so the
            # drawn circle + tried-sample markers + seed arrow stay visible through
            # the accept/retry prompt below (the user can see which bases were
            # tried). It is torn down in the `finally` once they decide.
            viz = _open_ik_sample_viz(seed_base_frame, config.IK_BASE_SAMPLE_RADIUS)
            try:
                # * S
                print("RSIKKeyframe: solving M1->M2->M3 IK chain ...")
                solved, used_base = _solve_chain_with_sampling(
                    planner, movements, seed_base_frame,
                    brep_id, heading_mm, include_self, include_env,
                    viz=viz,
                )
                # Drop the moving ghost robot but keep the circle + tried markers +
                # seed arrow, so the "what was tried" history stays on screen without
                # overlapping the assembled-pose preview shown below.
                viz.set_ghost_xform(None)
                if solved is None:
                    rs.EnableRedraw(True)
                    rs.MessageBox("IK failed for the assembly chain (all samples exhausted).", 0, "RSIKKeyframe")
                    # ssik only: let the user cycle through the first failing
                    # movement's candidate IK pairs (colliding ones included), with
                    # the offenders red-highlighted, before deciding what to do. The
                    # gradient backend has no branch pairs to enumerate.
                    allow_inspect = config.IK_BACKEND == "ssik"
                    while True:
                        action = _ask_chain_failure(allow_inspect)
                        if action == "inspect":
                            _inspect_ssik_candidates(
                                planner, movements, seed_base_frame,
                                include_self, include_env, mesh_mode, env_geom,
                            )
                            continue
                        break
                    if action == "retry_same_base":
                        print("RSIKKeyframe: retrying the IK chain with the same robot base frame.")
                        saved_base = seed_base_frame
                        brep_id = _resolve_sampling_brep_for_base(seed_base_frame, brep_id)
                        allow_saved_base_prompt = False
                        ik_viz.end_session()
                        continue
                    if action == "retry_new_base":
                        print("RSIKKeyframe: retrying the IK chain with a different robot base frame.")
                        saved_base = seed_base_frame
                        seed_base_frame = None
                        heading_mm = None
                        brep_id = None
                        allow_saved_base_prompt = False
                        ik_viz.end_session()
                        continue
                    print("RSIKKeyframe: gave up after IK chain failure.")
                    return

                # solved: M1 -> approach, M2 -> assembled, M3 -> retreat.
                approach_state = solved["M1"]
                assembled_state = solved["M2"]
                retreat_state = solved["M3"]

                # Build the toggle-through preview frames: the three solved
                # keyframes plus the fixed home pose the arms return to after
                # retreat (M4's target config). This lets the user step through
                # every keyframe here -- same as RSShowIK -- instead of only
                # seeing the assembled pose.
                display_frames = [
                    ("approach", approach_state),
                    ("assembled", assembled_state),
                    ("retreat", retreat_state),
                ]
                home_cfg = movements["M4"].target_configuration
                if home_cfg is not None:
                    # Reuse the retreat state (right base frame + released
                    # collision context) and swap in the home joint values.
                    home_state = retreat_state.copy()
                    home_state.robot_configuration = home_cfg.copy()
                    display_frames.append(("home", home_state))
                assembled_index = 1  # start the cycle on the assembled pose

                # Preview the assembled (mated) pose first -- the representative
                # keyframe. Enter / TogglePose then steps through the rest.
                _render_preview_pose(planner, assembled_state, mesh_mode)
                print(
                    "RSIKKeyframe: full M1->M2->M3 chain reachable. Previewing "
                    "assembled pose (Enter / TogglePose to step through keyframes)..."
                )

                # used_base may differ from seed_base_frame if the sampling fallback
                # found a better location; it is the base all three keyframes share.
                # Ask INSIDE the try so the sampling viz (circle + tried markers +
                # seed arrow) is still on screen while the user decides.
                stored_base = used_base
                action = _ask_accept_with_pose_preview(
                    display_frames,
                    lambda st: _render_preview_pose(planner, st, mesh_mode),
                    initial_index=assembled_index,
                    prompt="IK preview ready. Accept, retry the same base, retry a new base, or give up",
                )
            finally:
                rs.EnableRedraw(True)
                # Tear down this attempt's sampling viz now that the user has seen it
                # and decided; a fresh viz is opened on the next loop turn (retry).
                _close_ik_sample_viz(viz)
                viz = None

            if action == "accept":
                _write_assembly_base_frame(target_bar_oid, stored_base)
                _write_assembly_keyframes(
                    target_bar_oid, approach_state, assembled_state, retreat_state, rcell,
                )
                _write_legacy_assembly_blob(
                    target_bar_oid, stored_base, assembled_state, approach_state, rcell,
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
