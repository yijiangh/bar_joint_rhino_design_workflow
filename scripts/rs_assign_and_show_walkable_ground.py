#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSAssignAndShowWalkableGround - View + edit which WalkableGround(s) a bar's
mobile base stands on, with a ghost-robot preview.

A scene can have many WalkableGround breps, but a given bar should only sample
its robot base on the one or two nearest it. This ONE left-click command both
shows the current assignment (with the resulting mobile-base placement drawn as a
half-transparent ghost robot) and lets you overwrite it, in a loop:

  1. VIEW. Pick a bar (first run). Its assigned WalkableGround(s) are highlighted,
     the default mobile-base placement is computed (the same heuristic the
     headless solver uses in ``--base sample`` mode) and drawn as a base-frame
     marker + a HALF-TRANSPARENT ghost robot standing on that base. The ground
     ids + base position are printed. You are then asked: change the assignment?
  2. RE-ASSIGN (only if you answered Yes). Click the WalkableGround brep(s) you
     want (multiple allowed), Enter to confirm. That set OVERWRITES the bar's
     assignment (saved to user-text), and the command loops back to step 1 to
     re-visualize. Answer No (or Esc) at step 1 to finish.

If a bar has no assignment yet, the nearest ground(s) are auto-assigned + saved
first so step 1 always has something to show.

The base-placement heuristic is Rhino-free and shared with the headless solver
(``core.walkable_ground.derive_seed_base`` + ``frame_from_origin_normal_heading``):
the base stands a standoff BEHIND the bar and faces along the average male-joint
insertion direction (the way the bar is pushed to mate). Here we feed it the bar
centerline midpoint (its ground projection is where the base stands behind of) and
the average of the bar's male-joint blocks' +Z (their insertion axis).

The ghost robot is FK-only (no PyBullet): its link meshes are harvested once at
the home pose via ``core.ik_viz.get_robot_link_meshes_at_state`` (from a BARE
robot cell so only the robot is drawn, not the assembly bodies) and drawn
translucent through ``core.dynamic_preview.mesh_preview`` -- the same style as the
IK base-sampling ghost -- rigidly transformed onto the computed base frame.
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import base_frame_viz as _base_frame_viz_module
from core import config as _config_module
from core import dynamic_preview as _dynamic_preview_module
from core import env_collision as _env_collision_module
from core import ik_viz as _ik_viz_module
from core import rhino_walkable_ground as _walkable_rhino_module
from core import robot_cell as _robot_cell_module
from core import walkable_ground as _walkable_np_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import BAR_ID_KEY, repair_on_entry
from core.rhino_frame_io import doc_unit_scale_to_mm


# Command name used in every command-line message + dialog title.
CMD = "RSAssignAndShowWalkableGround"

# Object types the re-assign pick accepts (same set as the WalkableGround layer).
_WALKABLE_FILTER = rs.filter.surface | rs.filter.polysurface | rs.filter.extrusion

# ik_viz sub-layer key for the base-placement ghost's harvested meshes. Kept
# distinct from the viewer's "Assembly"/"Support" keys so it never clashes with
# an RSShowBarActionPlan cache bundle.
GHOST_LAYER_KEY = "BasePlacement"

# Opacity of the translucent ghost robot (1.0 = opaque, 0.0 = invisible).
GHOST_ALPHA = 0.4

# Temporary color painted on the assigned WalkableGround brep(s) during the
# preview, reverted to ByLayer on exit (green, matching RSShowBarActionPlan).
GROUND_HIGHLIGHT_COLOR = (60, 200, 90)


def _reload():
    """Re-import the edited core modules so ScriptEditor picks up code changes.

    Assigns the reloaded modules to module-level globals used by the rest of the
    file. Mirrors the reload pattern in the other RS* commands.
    """
    global base_frame_viz, config, dynamic_preview, env_collision, ik_viz
    global walkable_rhino, walkable_np, robot_cell
    config = importlib.reload(_config_module)
    walkable_np = importlib.reload(_walkable_np_module)
    walkable_rhino = importlib.reload(_walkable_rhino_module)
    robot_cell = importlib.reload(_robot_cell_module)
    ik_viz = importlib.reload(_ik_viz_module)
    base_frame_viz = importlib.reload(_base_frame_viz_module)
    dynamic_preview = importlib.reload(_dynamic_preview_module)
    env_collision = importlib.reload(_env_collision_module)


_reload()


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _bar_center_mm(bar_oid):
    """Return the bar centerline midpoint (mm) -- the bar center.

    The base stands behind, and faces, this point's ground projection.

    Args:
        bar_oid: the bar centerline curve id.

    Returns:
        np.ndarray | None: the midpoint in mm, or ``None`` if the curve endpoints
        can't be read.
    """
    start = rs.CurveStartPoint(bar_oid)
    end = rs.CurveEndPoint(bar_oid)
    if start is None or end is None:
        return None
    scale = doc_unit_scale_to_mm()
    start_mm = np.array([start.X, start.Y, start.Z], dtype=float) * scale
    end_mm = np.array([end.X, end.Y, end.Z], dtype=float) * scale
    return 0.5 * (start_mm + end_mm)


def _avg_male_insertion_dir_mm(bar_id):
    """Average world insertion direction over the bar's male joint blocks.

    Each male joint block's local +Z is its insertion axis (the way the male is
    pushed into its mate female -- retreat is -Z, see
    ``core.bar_action._retreat_tool0_target_mm``). We average that +Z over every
    male joint placed on this bar; the base faces along it. Rhino-only.

    Args:
        bar_id (str): the bar whose male joints to average.

    Returns:
        np.ndarray | None: the (unnormalized) average insertion direction, or
        ``None`` if the bar has no readable male joints.
    """
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
    if float(np.linalg.norm(avg)) < 1e-9:
        return None
    return avg


def _grounds_to_soups(ground_ids, grounds_map):
    """Convert the bar's assigned WalkableGround breps into numpy triangle soups.

    Meshes each assigned brep (world mm) and reduces it to the ``(vertices,
    triangles)`` soup the heuristic's closest-point code consumes.

    Args:
        ground_ids (list[str]): the bar's assigned ground ids.
        grounds_map (dict): ``{ground_id: oid}`` from
            ``rhino_walkable_ground.get_all_walkable_grounds``.

    Returns:
        list: a ``core.walkable_ground.TriangleSoup`` per meshable assigned ground
        (empty if none could be meshed).
    """
    soups = []
    for gid in ground_ids:
        oid = grounds_map.get(gid)
        if oid is None:
            continue
        try:
            mesh = walkable_rhino.brep_to_compas_mesh(oid)
            soups.append(walkable_np._mesh_to_soup(mesh))
        except Exception as exc:
            print(f"{CMD}: WalkableGround {gid} could not be meshed ({exc}); skipping.")
    return soups


def _compute_default_base_frame_mm(bar_id, bar_oid, ground_ids, grounds_map):
    """Run the default base-placement heuristic for one bar on its ground(s).

    This is the exact seed the headless ``--base sample`` solver starts from:
    ``derive_seed_base`` stands the base a standoff BEHIND the bar (opposite the
    average male-joint insertion direction) and faces it along that insertion
    direction, and ``frame_from_origin_normal_heading`` turns that into a base
    frame (Z = ground up, X = heading = insertion direction).

    Args:
        bar_id (str): the bar id (also used to find its male joints).
        bar_oid: the bar centerline curve id.
        ground_ids (list[str]): the bar's assigned ground ids.
        grounds_map (dict): ``{ground_id: oid}``.

    Returns:
        np.ndarray | None: the 4x4 mm base frame, or ``None`` if it can't be built
        (no meshable ground, unreadable bar, or a degenerate heading).
    """
    soups = _grounds_to_soups(ground_ids, grounds_map)
    if not soups:
        print(f"{CMD}: bar '{bar_id}' has no meshable WalkableGround; cannot place a base.")
        return None
    center_mm = _bar_center_mm(bar_oid)
    if center_mm is None:
        print(f"{CMD}: bar '{bar_id}' curve endpoints unreadable; cannot place a base.")
        return None
    insertion_dir = _avg_male_insertion_dir_mm(bar_id)
    if insertion_dir is None:
        print(f"{CMD}: bar '{bar_id}' has no readable male joints; base heading falls "
              "back to a world-horizontal direction.")
    try:
        origin, normal, heading_dir = walkable_np.derive_seed_base(
            soups, center_mm, heading_dir_mm=insertion_dir
        )
        # Build the frame facing along the heading direction (base +X).
        return walkable_np.frame_from_origin_normal_heading(
            origin, normal, origin + heading_dir * 1000.0
        )
    except Exception as exc:
        print(f"{CMD}: base-placement heuristic failed for bar '{bar_id}' ({exc}).")
        return None


def _np_mm_to_rhino_xform(matrix_mm):
    """Convert a 4x4 mm transform to a ``Rhino.Geometry.Transform`` in doc units.

    Only the translation column is scaled from mm to the document unit; the
    rotation columns are unitless. Used to rigidly place the ghost robot (whose
    link meshes were harvested at a worldXY base) onto the computed base frame.

    Args:
        matrix_mm (np.ndarray): a 4x4 transform with mm translation.

    Returns:
        Rhino.Geometry.Transform: the same transform in document units.
    """
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    m = np.array(matrix_mm, dtype=float, copy=True)
    m[:3, 3] *= scale_from_mm
    xform = Rhino.Geometry.Transform(1.0)
    for i in range(4):
        for j in range(4):
            xform[i, j] = float(m[i, j])
    return xform


def _prepare_ghost_meshes():
    """Harvest the robot's visual link meshes at the home pose (for the conduit).

    Builds a BARE robot cell (robot model + semantics only, no tools / rigid
    bodies) so only the robot is drawn, poses it at the home arm configuration
    with a worldXY base, and returns duplicated link meshes. The conduit later
    rigidly transforms this whole mesh set onto each computed base frame.

    Returns:
        list | None: the ghost link meshes, or ``None`` if the robot can't be
        loaded (the command then shows the base-frame marker without a ghost).
    """
    try:
        deps = robot_cell.import_compas_stack()
        Frame = deps["Frame"]
        RobotCell = deps["RobotCell"]

        full_cell = robot_cell.get_or_load_robot_cell()
        ghost_cell = RobotCell(full_cell.robot_model, full_cell.robot_semantics)
        state = ghost_cell.default_cell_state()
        state.robot_base_frame = Frame.worldXY()

        # Home arm pose (left 6 then right 6) so the ghost looks folded, not a
        # scarecrow at the zero configuration.
        arm_joint_names = (
            list(ghost_cell.get_configurable_joint_names(config.LEFT_GROUP))
            + list(ghost_cell.get_configurable_joint_names(config.RIGHT_GROUP))
        )
        for name, value in zip(arm_joint_names, config.HUSKY_DUAL_ARM_HOME_CONF_12):
            try:
                state.robot_configuration[name] = float(value)
            except Exception:
                pass  # a name mismatch on one joint shouldn't kill the whole ghost

        meshes = ik_viz.get_robot_link_meshes_at_state(
            state, robot_cell=ghost_cell, layer_key=GHOST_LAYER_KEY
        )
        return meshes or None
    except Exception as exc:
        print(f"{CMD}: ghost robot unavailable ({type(exc).__name__}: {exc}); showing base frame only.")
        return None


# ---------------------------------------------------------------------------
# WalkableGround highlight
# ---------------------------------------------------------------------------


def _highlight_grounds(ground_oids) -> list:
    """Paint the assigned WalkableGround brep(s) green; return the colored oids."""
    colored = []
    for oid in ground_oids:
        try:
            rs.ObjectColor(oid, GROUND_HIGHLIGHT_COLOR)
            colored.append(oid)
        except Exception:
            continue
    return colored


def _revert_grounds(ground_oids) -> None:
    """Restore the highlighted WalkableGround brep(s) to their ByLayer color."""
    for oid in ground_oids:
        try:
            rs.ObjectColorSource(oid, 0)  # 0 = ByLayer
        except Exception:
            continue


# ---------------------------------------------------------------------------
# Step 1 (view + ask) and Step 2 (re-assign)
# ---------------------------------------------------------------------------


def _ask_change_assignment(bar_id) -> bool:
    """Command-line Yes/No prompt: change this bar's WalkableGround assignment?

    Args:
        bar_id (str): the active bar id (shown in the prompt).

    Returns:
        bool: ``True`` only if the user explicitly chose ``Yes``; Enter (default
        ``No``) or Esc return ``False``.
    """
    answer = rs.GetString(
        f"Bar '{bar_id}': change its WalkableGround assignment?",
        "No",
        ["Yes", "No"],
    )
    return bool(answer) and answer.strip().lower().startswith("y")


def _show_and_ask_change(bar_id, bar_oid, ground_ids, grounds_map, robot_meshes) -> bool:
    """Step 1: visualize the assignment + ghost robot, then ask to change it.

    Highlights the assigned ground(s), prints their ids + the base position,
    draws the base-frame marker, and shows a half-transparent ghost robot on the
    default base while a Yes/No prompt is up. Everything but the saved assignment
    is cleaned up before returning.

    Args:
        bar_id (str): the active bar id.
        bar_oid: the bar centerline curve id.
        ground_ids (list[str]): the bar's assigned ground ids.
        grounds_map (dict): ``{ground_id: oid}``.
        robot_meshes (list | None): harvested ghost link meshes, or ``None``.

    Returns:
        bool: ``True`` if the user wants to re-assign, else ``False``.
    """
    ground_oids = []
    for gid in ground_ids:
        oid = grounds_map.get(gid)
        print(f"{CMD}: assigned WalkableGround {gid} -> object {oid}")
        if oid is not None:
            ground_oids.append(oid)

    base_mm = _compute_default_base_frame_mm(bar_id, bar_oid, ground_ids, grounds_map)
    highlighted = _highlight_grounds(ground_oids)
    base_frame_viz.clear_base_frames()
    baked_base = False
    try:
        base_xform = None
        if base_mm is not None:
            origin = base_mm[:3, 3]
            print(
                f"{CMD}: default mobile base for bar '{bar_id}' at "
                f"({origin[0]:.0f}, {origin[1]:.0f}, {origin[2]:.0f}) mm."
            )
            base_frame_viz.draw_base_frame(bar_id, base_mm)
            baked_base = True
            base_xform = _np_mm_to_rhino_xform(base_mm)

        # Show the translucent ghost (if available) for the duration of the prompt.
        if robot_meshes and base_xform is not None:
            with dynamic_preview.mesh_preview(robot_meshes, alpha=GHOST_ALPHA) as conduit:
                conduit.update_xform(base_xform)
                rs.Redraw()
                return _ask_change_assignment(bar_id)
        rs.Redraw()
        return _ask_change_assignment(bar_id)
    finally:
        if baked_base:
            base_frame_viz.clear_base_frames()
        _revert_grounds(highlighted)
        rs.Redraw()


def _pick_new_assignment(bar_id):
    """Step 2: multi-select the WalkableGround brep(s) to assign (overwrite).

    Args:
        bar_id (str): the active bar id (shown in the prompt).

    Returns:
        list[str] | None: the chosen ground ids (order preserved, de-duplicated),
        or ``None`` if the user cancelled / picked nothing valid (keep current).
    """
    rs.UnselectAllObjects()
    picked = rs.GetObjects(
        f"Bar '{bar_id}': select WalkableGround brep(s) to assign (overwrites the "
        "current set), Enter to confirm, Esc to cancel",
        filter=_WALKABLE_FILTER,
        group=False,
        preselect=False,
        select=True,
        minimum_count=1,
    )
    rs.UnselectAllObjects()
    if not picked:
        return None

    new_ids = []
    for oid in picked:
        if rs.ObjectLayer(oid) != config.WALKABLE_GROUND_LAYER:
            print(f"{CMD}: ignoring object not on '{config.WALKABLE_GROUND_LAYER}'.")
            continue
        gid = walkable_rhino.ensure_ground_id(oid)
        if gid not in new_ids:
            new_ids.append(gid)
    if not new_ids:
        print(f"{CMD}: no WalkableGround brep selected; assignment unchanged.")
        return None
    return new_ids


# ---------------------------------------------------------------------------
# Main (single left-click: view <-> re-assign loop)
# ---------------------------------------------------------------------------


def main() -> None:
    _reload()

    repair_on_entry(float(config.BAR_RADIUS), CMD)

    # Pick the bar.
    rs.UnselectAllObjects()
    bar_oid = pick_bar("Pick a bar to view / edit its WalkableGround assignment (Esc to cancel)")
    if bar_oid is None:
        return
    bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY)
    if not bar_id:
        rs.MessageBox(
            "Picked curve is not a registered bar (no 'bar_id' user-text).", 0, CMD
        )
        return

    # Every WalkableGround brep gets a stable id.
    grounds = walkable_rhino.get_all_walkable_grounds()  # {ground_id: oid}
    if not grounds:
        rs.MessageBox(
            f"Layer '{config.WALKABLE_GROUND_LAYER}' has no surface/polysurface/"
            "extrusion. Add at least one WalkableGround brep and try again.",
            0,
            CMD,
        )
        return

    # Ensure there is an assignment to visualize (auto-assign nearest if empty).
    ground_ids = walkable_rhino.get_bar_ground_ids(bar_oid)
    if not ground_ids:
        ground_ids = walkable_rhino.associate_bar_by_distance(bar_oid, grounds)
        walkable_rhino.set_bar_ground_ids(bar_oid, ground_ids)
        print(f"{CMD}: bar '{bar_id}' had no assignment; auto-assigned nearest + saved -> {ground_ids}")
    else:
        print(f"{CMD}: bar '{bar_id}' assigned -> {ground_ids}")

    # Harvest the ghost robot's link meshes once (home pose); reused every loop.
    print(f"{CMD}: preparing ghost robot (loading robot model if needed)...")
    robot_meshes = _prepare_ghost_meshes()

    try:
        # View <-> re-assign loop: show, ask, (maybe) re-pick + overwrite, repeat.
        while True:
            wants_change = _show_and_ask_change(
                bar_id, bar_oid, ground_ids, grounds, robot_meshes
            )
            if not wants_change:
                break

            new_ids = _pick_new_assignment(bar_id)
            if new_ids is None:
                print(f"{CMD}: re-assign cancelled; keeping {ground_ids or '[]'}.")
                continue

            ground_ids = new_ids
            walkable_rhino.set_bar_ground_ids(bar_oid, ground_ids)
            print(f"{CMD}: saved bar '{bar_id}' -> {ground_ids}")
            # A brand-new brep may have just been given an id -> refresh the map.
            grounds = walkable_rhino.get_all_walkable_grounds()
    finally:
        # Remove the hidden baked robot the harvest left on the IK cache layer.
        try:
            ik_viz.discard_cache()
        except Exception:
            pass
        rs.Redraw()


if __name__ == "__main__":
    main()
