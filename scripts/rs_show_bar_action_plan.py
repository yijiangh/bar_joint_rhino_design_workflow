#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.core_lib`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
# r: trimesh==4.12.2
"""RSShowBarActionPlan - View a bar's plan: its keyframe poses (left-click) or its planned motion (right-click).

Left-click (``main``): the KEYFRAME viewer. The document is read into a schema 2
design (``core.design_source``); pick a bar, then Enter (or ``TogglePose``)
cycles its steps (``core.plan_steps``): approach -> assembled -> hold (held bars:
the support robot grips while the arms still hold) -> retreat -> home, then the
release of every hold that lets go after this bar. Each step is the design's
scene at that moment, so the support robots stand exactly where the schedule has
them. The active bar's base frame is drawn (axis triad + footprint). When
``RSLoadSolvedBarAction`` has just loaded solved bars, it starts on them and
offers NextBar/PrevBar.

Right-click (``main_motion``): the MOTION viewer. Reads a design folder written
by RSExportDesign and the planner's ``solutions/`` there; pick a bar and step
through its planned trajectories (transfer -> insert -> retreat -> home) from the
COMMAND LINE (Enter/Next/Prev/Jump), so the viewport stays free to zoom/orbit.
Held bars follow the arm.

Everything draws the core's drawing cell of each scene (``core.design_display``:
visual and collision meshes, no PyBullet); CheckCollision and the jog dialog
check on the robots' planning mirrors (``core.design_planning``). The preview is
non-baked: everything is cleaned up on exit and the sequence display is restored.
"""

from __future__ import annotations

import importlib
import math
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import base_frame_viz as _base_frame_viz_module
from core import config as _config_module
from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path; never reloaded)
from core import design_builder as _design_builder_module
from core import design_display as _design_display_module
from core import design_planning as _design_planning_module
from core import design_source as _design_source_module
from core import ik_viz as _ik_viz_module
from core import plan_steps as _plan_steps_module
from core import rhino_walkable_ground as _walkable_ground_module
from core.rhino_bar_pick import (
    bar_or_tube_filter as _bar_or_tube_filter,
    pick_bar,
    resolve_picked_to_bar_curve as _resolve_picked_to_bar_curve,
)
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    clear_build_stage,
    get_bar_seq_map,
    reset_sequence_colors,
    set_build_stage,
    show_sequence_colors,
)
from core.rhino_helpers import suspend_redraw
from rs_check_design import DESIGN_FOLDER_STICKY_KEY  # the last design folder, shared with RSExportDesign


# Temporary color painted on the active bar's assigned WalkableGround brep(s) so
# the user can see which ground surface the robot base is allowed to stand on.
# Reverted to ByLayer on bar switch / session cleanup (green, distinct from the
# blue bar-selection color and the red collision highlight).
WALKABLE_GROUND_HIGHLIGHT_COLOR = (60, 200, 90)
COLLISION_COLOR = (255, 40, 40)
#: ik_viz sub-layer of the viewers' drawing cell (the assembly robot's; the support robots are drawn in it).
IK_LAYER_KEY_ASSEMBLY = "Assembly"
#: `sc.sticky` key of the bars RSLoadSolvedBarAction just loaded: {"bars": [bar ids], "folder": design folder}.
LOADED_STICKY_KEY = "bar_joint:loaded_solved_bars"


def _reload():
    global base_frame_viz, config, design_display, design_planning, design_source, ik_viz, plan_steps
    global walkable_ground
    config = importlib.reload(_config_module)
    ik_viz = importlib.reload(_ik_viz_module)
    base_frame_viz = importlib.reload(_base_frame_viz_module)
    walkable_ground = importlib.reload(_walkable_ground_module)
    importlib.reload(_design_builder_module)
    design_source = importlib.reload(_design_source_module)
    design_display = importlib.reload(_design_display_module)
    plan_steps = importlib.reload(_plan_steps_module)
    design_planning = importlib.reload(_design_planning_module)


_reload()


# ---------------------------------------------------------------------------
# * Designs
# ---------------------------------------------------------------------------


def design_from_document(command):
    """The document as a schema 2 design, with the session's geometry cache; prints what was left out."""
    result = design_source.RhinoDesignSource(command, design_source.geometry_cache(sc.sticky)).build()
    for problem in result.problems:
        print(f"{command}: {problem}")
    return result.design


def design_folder(command, folder=None):
    """A design folder: `folder`, the last one used, or one the user browses to; None on cancel."""
    folder = folder or sc.sticky.get(DESIGN_FOLDER_STICKY_KEY)
    if not folder or not os.path.isfile(os.path.join(folder, "design.json")):
        folder = rs.BrowseForFolder(folder=folder if folder and os.path.isdir(folder) else None,
                                    message="Select a design folder (written by RSExportDesign)", title=command)
        if not folder:
            return None
    if not os.path.isfile(os.path.join(folder, "design.json")):
        rs.MessageBox(f"No design.json in\n{folder}", 0, command)
        return None
    sc.sticky[DESIGN_FOLDER_STICKY_KEY] = folder
    return folder


def assembly_robot_id():
    """The assembly robot's id ("robots/cindy"): the viewers' drawing cell is its; the others are drawn in it."""
    return design_source.robot_id(config.ASSEMBLY_ROBOT_NAME)


def base_frame_mm(pose):
    """A core pose (metres) as a 4x4 matrix with its translation in mm."""
    matrix = np.array(pose.matrix(), dtype=float)
    matrix[:3, 3] *= 1000.0
    return matrix


def draw_base_frames(design, bar_ids):
    """Bake a base-frame marker (axis triad + footprint) for every bar with a base in the design."""
    frames = {}
    for bar_id in bar_ids:
        base = plan_steps.base_of(design, f"bars/{bar_id}")
        if base is not None:
            frames[bar_id] = base_frame_mm(base)
    if frames:
        base_frame_viz.draw_base_frames(frames)


# ---------------------------------------------------------------------------
# * Session
# ---------------------------------------------------------------------------


class View:
    """One thing a viewer shows: a bar's moment, made into a scene on demand.

    Attributes:
        bar_id (str): The host bar ("B3").
        label (str): What the prompt says.
        make_scene (callable): `make_scene()` -> the scene to draw (a new one each call).
        state: The design state the scene comes from (for the allowed part contacts in collision checks).
    """

    def __init__(self, bar_id, label, make_scene, state):
        self.bar_id, self.label, self.make_scene, self.state = bar_id, label, make_scene, state


def keyframe_views(design, steps):
    """Views of `plan_steps` steps."""
    return [View(plan_steps.short(s.bar), s.label, lambda s=s: plan_steps.step_scene(design, s), s.movement.start)
            for s in steps]


def motion_views(design, frames):
    """Views of trajectory frames; the scene of each movement is made once and copied per row."""
    scenes = {}

    def make(frame):
        scene = scenes.get(frame.movement.id)
        if scene is None:
            scene = scenes[frame.movement.id] = design.scene_at(frame.movement)
        scene = scene.copy()
        design_display.place(scene, frame.robot, frame.base, frame.joints)
        return scene

    bar_of = {m.id: a.bar for a, m in design.movements()}
    return [View(plan_steps.short(bar_of[f.movement.id]), f.label, lambda f=f: make(f), f.movement.start)
            for f in frames]


class PlanSession:
    """Mutable UI state of one viewer run: the design, the views, which one shows, and what is highlighted."""

    def __init__(self, design, command="RSShowBarActionPlan"):
        self.design = design
        self.command = command
        self.robot_id = assembly_robot_id()
        self.views = []
        self.index = 0
        #: Bars to step through with NextBar/PrevBar (the loaded bars); empty in the single-pick flow.
        self.bar_ids = []
        self.active_bar_id = None
        self.active_bar_oid = None
        self.show_unbuilt = False
        self.mesh_mode = ik_viz.get_mesh_mode()
        self.scene = None
        self.cell = None
        self._session_started = False
        self._seq_colored_for = None
        self._highlight_oids = []
        self._wg_highlight_oids = []

    @property
    def view(self):
        return self.views[self.index] if self.views else None

    # ---- mutations -----------------------------------------------------

    def set_views(self, views, index=0):
        """Show these views, starting at `index`."""
        self.views = list(views)
        self.index = max(0, min(index, len(self.views) - 1)) if self.views else 0
        self.refresh()

    def set_active_bar(self, bar_id, bar_oid, index=0):
        """Show one bar's keyframe steps (its oid is used for the sequence colors and its walkable grounds)."""
        self.active_bar_id, self.active_bar_oid = bar_id, bar_oid
        steps = plan_steps.bar_steps(self.design, f"bars/{bar_id}")
        if not steps:
            print(f"{self.command}: bar {bar_id} has no step in the design (not assembled, or no solved keyframe "
                  "and no release after it).")
        self.set_views(keyframe_views(self.design, steps), index)
        self._show_walkable_grounds()

    def step(self, delta):
        """Show the next (or previous) view; wraps around."""
        if self.views:
            self.index = (self.index + delta) % len(self.views)
            self.refresh()

    def toggle_unbuilt(self):
        """Toggle unbuilt visibility AND the persistent build-stage latch.

        Only this explicit user toggle writes the latch: opening the viewer never
        hides the model permanently.
        """
        self.show_unbuilt = not self.show_unbuilt
        if self.show_unbuilt:
            clear_build_stage()
        elif self.active_bar_id is not None:
            entry = get_bar_seq_map().get(self.active_bar_id)
            if entry is not None:
                set_build_stage(self.active_bar_id, entry[1])
        if self.active_bar_id is not None:
            show_sequence_colors(self.active_bar_id, self.show_unbuilt)
            self._seq_colored_for = (self.active_bar_id, self.show_unbuilt)

    def cycle_mesh_mode(self):
        """Flip visual <-> collision (a visibility flip, no rebake)."""
        self.mesh_mode = (ik_viz.MESH_MODE_COLLISION if self.mesh_mode == ik_viz.MESH_MODE_VISUAL
                          else ik_viz.MESH_MODE_VISUAL)
        self._revert_highlight()
        if self._session_started:
            ik_viz.set_active_mesh_mode(IK_LAYER_KEY_ASSEMBLY, self.mesh_mode)
        print(f"{self.command}: mesh_mode={self.mesh_mode}")

    # ---- rendering -----------------------------------------------------

    def refresh(self):
        """Draw the current view."""
        self._revert_highlight()
        view = self.view
        if view is None:
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            return
        # Recoloring walks every bar/joint/tool in the document: only when the
        # host bar or the unbuilt toggle changed.
        host = view.bar_id
        wanted = (host, self.show_unbuilt)
        if self._seq_colored_for != wanted:
            show_sequence_colors(host, self.show_unbuilt)
            self._seq_colored_for = wanted
        self.draw(view.make_scene())
        print(f"{self.command}: {view.label} ({self.index + 1}/{len(self.views)})")

    def draw(self, scene):
        """Draw a scene on the drawing cell; it becomes the scene collision checks look at."""
        cell, state = design_display.cell_and_state(sc.sticky, scene, self.robot_id)
        rs.EnableRedraw(False)
        try:
            if not self._session_started or cell is not self.cell:
                ik_viz.begin_session(robot_cell=cell, active_mesh_mode=self.mesh_mode,
                                     layer_key=IK_LAYER_KEY_ASSEMBLY)
                self._session_started = True
            ik_viz.update_state(state, robot_cell=cell, layer_key=IK_LAYER_KEY_ASSEMBLY, mesh_mode=self.mesh_mode)
        finally:
            rs.EnableRedraw(True)
        self.scene, self.cell = scene, cell

    # ---- collision diagnostic ------------------------------------------

    def check_collision(self, verbose=True):
        """Check the shown scene on every placed robot's mirror; red-highlight the offenders. Returns the count."""
        if self.scene is None or self.view is None:
            print(f"{self.command}: nothing shown to check.")
            return 0
        self._revert_highlight()
        try:
            pairs = design_planning.scene_collisions(self.design, sc.sticky, self.scene, self.view.state)
        except Exception as exc:  # noqa: BLE001 -- a diagnostic must never end the viewer
            print(f"{self.command}: collision check failed ({type(exc).__name__}: {exc}).")
            return 0
        self._highlight_pairs(pairs, verbose)
        return len(pairs)

    def _highlight_pairs(self, pairs, verbose):
        """Paint the drawn meshes of every id in `pairs` red; print the pairs."""
        if verbose:
            if not pairs:
                print(f"{self.command}: CheckCollision -- no collisions detected.")
            for a, b in pairs:
                print(f"{self.command}: COLLISION -- {a} x {b}")
        ids = {i for pair in pairs for i in pair}
        owners = {tool: robot for robot, spec in self.design.robots.items() for tool in spec.tools.values()}
        self._apply_highlight(_guids_for_ids(self.cell, self.robot_id, ids, self.mesh_mode, owners))
        if verbose and pairs:
            print(f"{self.command}: {len(pairs)} colliding pair(s); highlighted "
                  f"{len(self._highlight_oids)} object(s).")

    def interactive_collision_check(self):
        """Open a modal Eto dialog with one slider per arm joint of the assembly robot.

        Every slider tick moves the robot in the shown scene, redraws it, and
        re-checks collisions on its mirror (synced once, then checked at the
        jogged joints) so the user can jog into known bad poses and confirm that
        offenders light up red. On close the highlights are cleared; Enter /
        TogglePose snaps back to a saved pose.
        """
        robot = self.scene.robots.get(self.robot_id) if self.scene is not None else None
        if robot is None or not robot.enabled or not robot.base_tracked:
            print(f"{self.command}: the assembly robot is not placed in this step; nothing to jog.")
            return
        self._revert_highlight()
        joint_specs = []  # (group, name, lower, upper)
        for group in (config.LEFT_GROUP, config.RIGHT_GROUP):
            try:
                names = list(self.cell.get_configurable_joint_names(group))
            except Exception as exc:  # noqa: BLE001
                print(f"{self.command}: no joints for group {group!r} ({exc}); skipping it.")
                continue
            for name in names:
                limit = getattr(self.cell.robot_model.get_joint_by_name(name), "limit", None)
                lo = float(getattr(limit, "lower", -math.pi)) if limit is not None else -math.pi
                hi = float(getattr(limit, "upper", math.pi)) if limit is not None else math.pi
                if hi <= lo:
                    lo, hi = -math.pi, math.pi
                joint_specs.append((group, name, lo, hi))
        if not joint_specs:
            print(f"{self.command}: no configurable joints found; InteractiveCollisionCheck aborted.")
            return
        mirror = None
        try:
            checked = self.scene.copy()
            design_planning.allow_part_contacts(self.design, checked, self.view.state)
            design_planning.plannable(checked, self.robot_id)
            mirror = design_planning.mirror_for(sc.sticky, self.robot_id)
            mirror.sync(checked)
        except Exception as exc:  # noqa: BLE001 -- jog still draws without the check
            print(f"{self.command}: collision checks unavailable while jogging ({exc}).")
            mirror = None
        _run_collision_jog_dialog(self, mirror, joint_specs)
        self._revert_highlight()
        print(f"{self.command}: InteractiveCollisionCheck closed.")

    def _apply_highlight(self, oids):
        with suspend_redraw():
            for oid in oids or []:
                try:
                    rs.ObjectColor(oid, COLLISION_COLOR)
                except Exception:  # noqa: BLE001
                    continue
                self._highlight_oids.append(oid)

    def _revert_highlight(self):
        if not self._highlight_oids:
            return
        with suspend_redraw():
            for oid in self._highlight_oids:
                try:
                    rs.ObjectColorSource(oid, 0)  # ByLayer
                except Exception:  # noqa: BLE001
                    continue
        self._highlight_oids = []

    # ---- walkable-ground highlight -------------------------------------

    def _show_walkable_grounds(self):
        """Temporarily color the active bar's assigned WalkableGround brep(s) green and name them."""
        self._revert_walkable_grounds()
        if self.active_bar_oid is None:
            return
        ground_ids = walkable_ground.get_bar_ground_ids(self.active_bar_oid)
        if not ground_ids:
            print(f"{self.command}: bar {self.active_bar_id} has no assigned WalkableGround "
                  "(run RSAssignAndShowWalkableGround or RSRebuildRobotCell).")
            return
        all_grounds = walkable_ground.get_all_walkable_grounds()
        with suspend_redraw():
            for gid in ground_ids:
                oid = all_grounds.get(gid)
                if oid is None:
                    print(f"{self.command}: bar {self.active_bar_id} references WalkableGround '{gid}' but no "
                          "such brep is in the document.")
                    continue
                try:
                    rs.ObjectColor(oid, WALKABLE_GROUND_HIGHLIGHT_COLOR)
                except Exception:  # noqa: BLE001
                    continue
                self._wg_highlight_oids.append(oid)
        print(f"{self.command}: bar {self.active_bar_id} stands on {ground_ids} (highlighted green).")

    def _revert_walkable_grounds(self):
        if not self._wg_highlight_oids:
            return
        with suspend_redraw():
            for oid in self._wg_highlight_oids:
                try:
                    rs.ObjectColorSource(oid, 0)
                except Exception:  # noqa: BLE001
                    continue
        self._wg_highlight_oids = []

    def cleanup(self):
        self._revert_highlight()
        self._revert_walkable_grounds()
        if self._session_started:
            ik_viz.end_session()
            self._session_started = False
        try:
            reset_sequence_colors()
        except Exception as exc:  # noqa: BLE001 -- never let cleanup mask the real outcome
            print(f"{self.command}: failed to restore sequence colors ({exc}); continuing.")


def _guids_for_ids(cell, robot_id, ids, mesh_mode, owners=None):
    """The drawn meshes of design ids: bodies, tools, the drawing robot's links, other robots (as tools).

    Args:
        owners: Tool id -> the robot it is mounted on; a tool of another robot is drawn as part of that robot.
    """
    links = ik_viz.get_link_native_geometry(cell, IK_LAYER_KEY_ASSEMBLY, mesh_mode)
    tools = ik_viz.get_tool_native_geometry(cell, IK_LAYER_KEY_ASSEMBLY, mesh_mode)
    bodies = ik_viz.get_rb_native_geometry(cell, IK_LAYER_KEY_ASSEMBLY, mesh_mode)
    out = []
    for object_id in ids:
        parts = object_id.split("/")
        if object_id in bodies:
            out += bodies[object_id]
        elif object_id in tools:
            out += tools[object_id]
        elif (owners or {}).get(object_id) in tools:
            out += tools[owners[object_id]]
        elif parts[0] == "robots" and "/".join(parts[:2]) == robot_id:
            out += links.get(parts[2], []) if len(parts) > 2 else [g for gs in links.values() for g in gs]
        elif parts[0] == "robots":
            out += tools.get("/".join(parts[:2]), [])
    return out


# ---------------------------------------------------------------------------
# * Jog dialog
# ---------------------------------------------------------------------------


def _run_collision_jog_dialog(session, mirror, joint_specs):
    """A modal Eto dialog of per-joint sliders over the assembly robot's arms in the shown scene.

    Each slider tick writes the joint value into the scene, redraws it, and (with
    a synced `mirror`) re-checks collisions at the jogged joints and re-highlights
    the offenders. ``joint_specs`` is ``[(group, joint_name, lower_rad, upper_rad)]``;
    sliders are 0..1000 mapped onto each joint's range; values show in degrees.
    """
    import Eto.Forms as forms
    import Eto.Drawing as drawing

    # Eto control constructors don't accept keyword args under Rhino's CPython
    # PythonNet host: construct, then assign.
    def _label(text):
        lbl = forms.Label()
        lbl.Text = text
        return lbl

    def _button(text):
        btn = forms.Button()
        btn.Text = text
        return btn

    steps = 1000
    scene = session.scene
    robot = scene.robots[session.robot_id]

    def to_rad(value, lo, hi):
        return lo + (hi - lo) * (float(value) / steps)

    def to_slider(rad, lo, hi):
        return int(round(max(0.0, min(1.0, (float(rad) - lo) / (hi - lo))) * steps)) if hi > lo else 0

    def check(verbose=False):
        if mirror is None:
            return None
        session._revert_highlight()
        pairs = sorted({tuple(sorted(p)) for p in mirror.collisions(dict(robot.joints), full_report=True)})
        session._highlight_pairs(pairs, verbose)
        return len(pairs)

    def count_text(n):
        if n is None:
            return "Collisions: (no check)"
        return "Collisions: 0 (clear)" if n == 0 else f"Collisions: {n} colliding pair(s)"

    dlg = forms.Dialog[bool]()
    dlg.Title = f"Interactive Collision Check - {session.view.label}"
    dlg.Padding = drawing.Padding(8)
    dlg.Resizable = True
    dlg.MinimumSize = drawing.Size(520, 360)
    status_label = _label("Move a slider to jog the robot.")
    pair_count_label = _label("Collisions: -")
    layout = forms.DynamicLayout()
    layout.Spacing = drawing.Size(6, 4)
    layout.BeginVertical()
    layout.AddRow(_label("Joint"), _label("Jog"), _label("Value (deg)"))

    initial = {}
    sliders = {}

    def make_handler(name, lo, hi, val_label):
        def handler(_sender, _args):
            rad = to_rad(sliders[name].Value, lo, hi)
            val_label.Text = f"{math.degrees(rad):+7.1f}"
            rs.EnableRedraw(False)
            try:
                design_display.place(scene, session.robot_id, joints={name: rad})
                session.draw(scene)
                n = check()
            except Exception as exc:  # noqa: BLE001
                status_label.Text = f"jog failed: {exc}"
                return
            finally:
                rs.EnableRedraw(True)
                sc.doc.Views.Redraw()
            pair_count_label.Text = count_text(n)
            status_label.Text = f"{name} = {math.degrees(rad):+.1f} deg ({rad:+.3f} rad)"
        return handler

    current_group = None
    for group, name, lo, hi in joint_specs:
        if group != current_group:
            layout.AddRow(_label(f"-- {group} --"))
            current_group = group
        rad = max(lo, min(hi, float(robot.joints.get(name, 0.0))))
        initial[name] = rad
        slider = forms.Slider()
        slider.MinValue, slider.MaxValue = 0, steps
        slider.Value = to_slider(rad, lo, hi)
        slider.Width = 260
        sliders[name] = slider
        val_label = _label(f"{math.degrees(rad):+7.1f}")
        val_label.Width = 70
        slider.ValueChanged += make_handler(name, lo, hi, val_label)
        short_name = name.split("_", 2)[-1] if name.startswith(("left_", "right_")) else name
        side = "L" if name.startswith("left_") else ("R" if name.startswith("right_") else "?")
        joint_label = _label(f"[{side}] {short_name}")
        joint_label.Width = 170
        layout.AddRow(joint_label, slider, val_label)

    layout.AddRow(None)
    layout.AddRow(status_label)
    layout.AddRow(pair_count_label)
    reset_btn = _button("Reset to current pose")
    verbose_btn = _button("Verbose check (print pairs)")
    close_btn = _button("Close")

    def on_reset(_sender, _args):
        for _group, name, lo, hi in joint_specs:
            sliders[name].Value = to_slider(initial[name], lo, hi)  # ValueChanged moves the robot

    def on_verbose(_sender, _args):
        try:
            pair_count_label.Text = count_text(check(verbose=True))
            status_label.Text = "verbose check complete -- see the Rhino command history"
        except Exception as exc:  # noqa: BLE001
            status_label.Text = f"verbose check failed: {exc}"

    def on_close(_sender, _args):
        dlg.Close(True)

    reset_btn.Click += on_reset
    verbose_btn.Click += on_verbose
    close_btn.Click += on_close
    layout.AddRow(reset_btn, verbose_btn, close_btn)
    layout.EndVertical()
    dlg.Content = layout
    dlg.AbortButton = close_btn
    try:
        pair_count_label.Text = count_text(check())
    except Exception as exc:  # noqa: BLE001
        pair_count_label.Text = f"initial check failed: {exc}"
    try:
        parent = Rhino.UI.RhinoEtoApp.MainWindow
    except Exception:  # noqa: BLE001
        parent = None
    if parent is None:
        dlg.ShowModal()
    else:
        dlg.ShowModal(parent)


# ---------------------------------------------------------------------------
# * Main loops
# ---------------------------------------------------------------------------


def _resolve_pick_to_bar(picked_obj_id):
    """Resolve a clicked oid (centerline curve or tube preview) to ``(bar_id, bar_curve_oid)`` or ``(None, None)``."""
    bar_curve_id = _resolve_picked_to_bar_curve(picked_obj_id)
    if bar_curve_id is None:
        return None, None
    bar_id = rs.GetUserText(bar_curve_id, BAR_ID_KEY)
    if not bar_id:
        return None, None
    return bar_id, bar_curve_id


def _build_get_option(session):
    go = Rhino.Input.Custom.GetObject()
    now_label = session.view.label if session.view is not None else "-"
    go.SetCommandPrompt(
        f"Pick a bar, Enter=cycle pose [now: {now_label}, mesh: {session.mesh_mode}], "
        "Esc to exit (right-click the button for the motion scrub)"
    )
    go.EnablePreSelect(False, False)
    go.AcceptNothing(True)
    go.SetCustomGeometryFilter(_bar_or_tube_filter)
    go.AddOption("TogglePose")
    go.AddOption("MeshMode")
    go.AddOption("CheckCollision")
    go.AddOption("InteractiveCollisionCheck")
    go.AddOption("HideUnbuilt" if session.show_unbuilt else "ShowUnbuilt")
    if len(session.bar_ids) > 1:
        go.AddOption("NextBar")
        go.AddOption("PrevBar")
    return go


def _step_bar(session, seq_map, delta):
    """Activate the next/previous bar in ``session.bar_ids``."""
    if len(session.bar_ids) < 2 or session.active_bar_id not in session.bar_ids:
        return
    idx = session.bar_ids.index(session.active_bar_id)
    nxt = session.bar_ids[(idx + delta) % len(session.bar_ids)]
    oid_seq = seq_map.get(nxt)
    if oid_seq is not None:
        session.set_active_bar(nxt, oid_seq[0])
        base_frame_viz.clear_base_frames()
        draw_base_frames(session.design, [nxt])


def main() -> None:
    _reload()
    seq_map = get_bar_seq_map()  # {bar_id: (oid, seq)}

    # If RSLoadSolvedBarAction just wrote solved keyframes, start on those bars (no pick) and draw every
    # loaded bar's base frame. Otherwise, classic flow: pick a bar.
    loaded = sc.sticky.pop(LOADED_STICKY_KEY, None) or {}
    loaded_here = [b for b in loaded.get("bars", []) if b in seq_map]
    design = design_from_document("RSShowBarActionPlan")
    if loaded_here:
        bar_ids = sorted(loaded_here, key=lambda b: seq_map[b][1])
        initial_bar_id, initial_oid = bar_ids[0], seq_map[bar_ids[0]][0]
        draw_base_frames(design, bar_ids)
    else:
        rs.UnselectAllObjects()
        initial_oid = pick_bar("Pick a bar to view its plan (Esc to cancel)")
        if initial_oid is None:
            return
        initial_bar_id = rs.GetUserText(initial_oid, BAR_ID_KEY)
        if not initial_bar_id:
            rs.MessageBox("Picked curve is not a registered bar (no 'bar_id' user-text).", 0, "RSShowBarActionPlan")
            return
        bar_ids = []
        draw_base_frames(design, [initial_bar_id])

    session = PlanSession(design)
    session.bar_ids = bar_ids
    try:
        session.set_active_bar(initial_bar_id, initial_oid)
        while True:
            go = _build_get_option(session)
            result = go.Get()
            if result == Rhino.Input.GetResult.Cancel:
                break
            if result == Rhino.Input.GetResult.Object:
                bar_id, bar_oid = _resolve_pick_to_bar(go.Object(0).ObjectId)
                if bar_id is None:
                    print("RSShowBarActionPlan: picked object is not a registered bar; ignoring.")
                elif bar_id == session.active_bar_id:
                    session.refresh()
                else:
                    session.set_active_bar(bar_id, bar_oid)
                    base_frame_viz.clear_base_frames()
                    draw_base_frames(design, [bar_id])
                continue
            if result == Rhino.Input.GetResult.Nothing:
                session.step(+1)
                continue
            if result == Rhino.Input.GetResult.Option:
                name = go.Option().EnglishName
                if name == "TogglePose":
                    session.step(+1)
                elif name == "MeshMode":
                    session.cycle_mesh_mode()
                elif name == "CheckCollision":
                    session.check_collision()
                elif name == "InteractiveCollisionCheck":
                    session.interactive_collision_check()
                elif name in ("ShowUnbuilt", "HideUnbuilt"):
                    session.toggle_unbuilt()
                elif name == "NextBar":
                    _step_bar(session, seq_map, +1)
                elif name == "PrevBar":
                    _step_bar(session, seq_map, -1)
    finally:
        session.cleanup()
        base_frame_viz.clear_base_frames()


def _run_motion_command_loop(session):
    """Step through the views from the Rhino COMMAND LINE: Enter/Next, Prev, Jump, Close/Esc (all wrap around)."""
    n = len(session.views)
    while True:
        go = Rhino.Input.Custom.GetOption()
        go.SetCommandPrompt(f"Motion scrub [{session.view.label}, {session.index + 1}/{n}] "
                            "-- Enter/Next=forward, Prev=back, Jump=index, Esc to exit")
        go.AcceptNothing(True)
        go.AddOption("Next")
        go.AddOption("Prev")
        go.AddOption("Jump")
        go.AddOption("Close")
        res = go.Get()
        if res == Rhino.Input.GetResult.Cancel:
            break
        if res == Rhino.Input.GetResult.Nothing:
            session.step(+1)
            continue
        if res == Rhino.Input.GetResult.Option:
            name = go.Option().EnglishName
            if name == "Next":
                session.step(+1)
            elif name == "Prev":
                session.step(-1)
            elif name == "Jump":
                gi = Rhino.Input.Custom.GetInteger()
                gi.SetCommandPrompt(f"Waypoint index (1..{n})")
                gi.SetLowerLimit(1, False)
                gi.SetUpperLimit(n, False)
                gi.SetDefaultInteger(session.index + 1)
                if gi.Get() == Rhino.Input.GetResult.Number:
                    session.step(gi.Number() - 1 - session.index)
            elif name == "Close":
                break


def main_motion(folder=None, bar=None) -> None:
    """Right-click entry: pick a bar and scrub its planned trajectories from a design folder's solutions.

    Args:
        folder (str | None): The design folder; None asks (default: the last one used).
        bar (tuple | None): ``(bar_id, bar_oid)`` to scrub without a pick.
    """
    from bar_assembly_core.design import read
    from bar_assembly_core.design.solutions import is_stale, read_solutions

    _reload()
    folder = design_folder("RSShowBarActionPlan", folder)
    if folder is None:
        return
    try:
        design = read(folder)
        solutions = read_solutions(design)
    except Exception as exc:  # noqa: BLE001 -- a broken folder is reported, not raised
        rs.MessageBox(f"Cannot read the design folder:\n{exc}", 0, "RSShowBarActionPlan")
        return
    stale = sorted(a for a, s in solutions.items() if is_stale(design, s))
    if stale:
        print(f"RSShowBarActionPlan: stale solutions (the design changed since they were planned): {', '.join(stale)}")

    if bar is not None:
        bar_id, oid = bar
    else:
        rs.UnselectAllObjects()
        oid = pick_bar("Pick a bar to scrub its planned motion (Esc to cancel)")
        if oid is None:
            return
        bar_id = rs.GetUserText(oid, BAR_ID_KEY)
        if not bar_id:
            rs.MessageBox("Picked curve is not a registered bar.", 0, "RSShowBarActionPlan")
            return
    frames = plan_steps.trajectory_frames(design, solutions, f"bars/{bar_id}")
    if not frames:
        rs.MessageBox(f"Bar {bar_id} has no planned trajectory in\n{os.path.join(folder, 'solutions')}\n\n"
                      "Plan it first: python scripts/plan_design.py <design folder>.", 0, "RSShowBarActionPlan")
        return

    session = PlanSession(design)
    session.active_bar_id, session.active_bar_oid = bar_id, oid
    try:
        draw_base_frames(design, [bar_id])
        session.set_views(motion_views(design, frames))
        session._show_walkable_grounds()
        _run_motion_command_loop(session)
    finally:
        session.cleanup()
        base_frame_viz.clear_base_frames()


if __name__ == "__main__":
    main()
