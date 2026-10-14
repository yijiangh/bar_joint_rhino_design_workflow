#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSShowBarActionPlan - View a bar's plan: its keyframe poses (left-click) or its planned motion (right-click).

Left-click (``main``): the KEYFRAME viewer. The document is read into a schema 2
process (``core.process_source``); pick a bar, then Enter (or ``TogglePose``)
cycles its steps (``core.plan_steps``): approach -> assembled -> hold (held bars:
the support robot grips while the arms still hold) -> retreat -> home, then the
release of every hold that lets go after this bar. Each step is the process's
scene at that moment, so the support robots stand exactly where the schedule has
them. The active bar's base frame is drawn (axis triad + footprint). When
``RSLoadSolvedBarAction`` has just loaded solved bars, it starts on them and
offers NextBar/PrevBar.

Right-click (``main_motion``): the MOTION viewer. Reads a process folder written
by RSExportProcess and the planner's ``solutions/`` there; pick a bar and step
through its planned trajectories (transfer -> insert -> retreat -> home) from the
COMMAND LINE (Enter/Next/Prev/Jump), so the viewport stays free to zoom/orbit.
Held bars follow the arm.

Every view is a core scene drawn part by part (``core.ik_viz``: visual and
collision meshes, no compas, no PyBullet); CheckCollision and the jog dialog
check on the robots' planning mirrors (``core.process_planning``). The preview is
non-baked: everything is cleaned up on exit and the sequence display is restored.
"""

from __future__ import annotations

import importlib
import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


from core import base_frame_viz as _base_frame_viz_module
from core import config as _config_module
from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path; never reloaded)
from core import process_builder as _process_builder_module
from core import process_display as _process_display_module
from core import process_planning as _process_planning_module
from core import process_source as _process_source_module
from core import ik_status
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
    COLOR_STALE_IK,
    clear_build_stage,
    get_bar_seq_map,
    reset_sequence_colors,
    sequence_colors,
    set_build_stage,
    show_sequence_colors,
)
from core.rhino_helpers import suspend_redraw
from rs_check_process import PROCESS_FOLDER_STICKY_KEY  # the last process folder, shared with RSExportProcess
from bar_assembly_core.urdf import UR_JOINT_NAMES, load_urdf  # after core_lib: it puts the core on sys.path


# The drawn walkable grounds: the one(s) the active bar stands on light, every other one dark. Greys on purpose:
# the bars carry the sequence colours, and a green floor read as part of the built (green) structure.
WALKABLE_GROUND_ACTIVE_COLOR = (215, 215, 215)
WALKABLE_GROUND_OTHER_COLOR = (95, 95, 95)
COLLISION_COLOR = (255, 40, 40)
#: ik_viz sub-layer the viewers draw their scenes on (every robot, every body).
IK_LAYER_KEY_ASSEMBLY = "Assembly"
#: `sc.sticky` key of the bars RSLoadSolvedBarAction just loaded: {"bars": [bar ids], "folder": process folder}.
LOADED_STICKY_KEY = "bar_joint:loaded_solved_bars"


def _reload():
    global base_frame_viz, config, process_display, process_planning, process_source, ik_viz, plan_steps
    global walkable_ground
    config = importlib.reload(_config_module)
    ik_viz = importlib.reload(_ik_viz_module)
    base_frame_viz = importlib.reload(_base_frame_viz_module)
    walkable_ground = importlib.reload(_walkable_ground_module)
    importlib.reload(_process_builder_module)
    process_source = importlib.reload(_process_source_module)
    process_display = importlib.reload(_process_display_module)
    plan_steps = importlib.reload(_plan_steps_module)
    process_planning = importlib.reload(_process_planning_module)


_reload()


# ---------------------------------------------------------------------------
# * Processes
# ---------------------------------------------------------------------------


def process_from_document(command):
    """The document as a schema 2 process, with the session's geometry cache; prints what was left out."""
    result = process_source.RhinoProcessSource(command, process_source.geometry_cache(sc.sticky)).build()
    for problem in result.problems:
        print(f"{command}: {problem}")
    return result.process


def process_folder(command, folder=None):
    """A process folder: `folder`, the last one used, or one the user browses to; None on cancel."""
    folder = folder or sc.sticky.get(PROCESS_FOLDER_STICKY_KEY)
    if not folder or not os.path.isfile(os.path.join(folder, "process.json")):
        folder = rs.BrowseForFolder(folder=folder if folder and os.path.isdir(folder) else None,
                                    message="Select a process folder (written by RSExportProcess)", title=command)
        if not folder:
            return None
    if not os.path.isfile(os.path.join(folder, "process.json")):
        rs.MessageBox(f"No process.json in\n{folder}", 0, command)
        return None
    sc.sticky[PROCESS_FOLDER_STICKY_KEY] = folder
    return folder


def assembly_robot_id():
    """The assembly robot's id ("robots/cindy"): drawn even while some of its joints are unknown."""
    return process_source.robot_id(config.ASSEMBLY_ROBOT_NAME)


def base_frame_mm(pose):
    """A core pose (metres) as a 4x4 matrix with its translation in mm."""
    matrix = np.array(pose.matrix(), dtype=float)
    matrix[:3, 3] *= 1000.0
    return matrix


@session_log.timed("draw base frame marker")
def draw_base_frames(process, bar_ids):
    """Bake a base-frame marker (axis triad + footprint) for every bar with a base in the process."""
    frames = {}
    for bar_id in bar_ids:
        base = plan_steps.base_of(process, f"bars/{bar_id}")
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
        state: The process state the scene comes from.
    """

    def __init__(self, bar_id, label, make_scene, state):
        self.bar_id, self.label, self.make_scene, self.state = bar_id, label, make_scene, state


def keyframe_views(process, steps):
    """Views of `plan_steps` steps."""
    return [View(plan_steps.short(s.bar), s.label, lambda s=s: plan_steps.step_scene(process, s), s.movement.start)
            for s in steps]


def motion_views(process, frames):
    """Views of trajectory frames; the scene of each movement is made once and copied per row."""
    scenes = {}

    def make(frame):
        scene = scenes.get(frame.movement.id)
        if scene is None:
            scene = scenes[frame.movement.id] = process.scene_at(frame.movement)
        scene = scene.copy()
        process_display.place(scene, frame.robot, frame.base, frame.joints)
        return scene

    bar_of = {m.id: a.bar for a, m in process.movements()}
    return [View(plan_steps.short(bar_of[f.movement.id]), f.label, lambda f=f: make(f), f.movement.start)
            for f in frames]


class PlanSession:
    """Mutable UI state of one viewer run: the process, the views, which one shows, and what is highlighted."""

    def __init__(self, process, command="RSShowBarActionPlan"):
        self.process = process
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
        self._session_started = False
        self._seq_colored_for = None
        self._highlight_oids = []
        #: Drawn ground id -> colour: the active bar's walkable ground (`_show_walkable_grounds`).
        self._ground_colors = {}
        #: Part or robot id -> colour of the drawn meshes at the shown step (`_part_colors`).
        self._colors = {}
        #: Bar id -> why its saved IK no longer fits the document (the core's plan check B15): these bars' robot
        #: poses are drawn where they were solved, not at the bar.
        self.stale = ik_status.stale_ik_bars(process)
        for line in ik_status.stale_ik_lines(self.stale):
            print(f"{command}: {line}")
        if self.stale:
            print(f"{command}: their robot is drawn yellow, at its saved (wrong) pose.")

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
        with session_log.step("plan steps of the bar", bar_id):
            steps = plan_steps.bar_steps(self.process, f"bars/{bar_id}")
        if not steps:
            print(f"{self.command}: bar {bar_id} has no step in the process (not assembled, or no solved keyframe "
                  "and no release after it).")
        self.set_views(keyframe_views(self.process, steps), index)
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
            with session_log.step("color bars by sequence"):
                show_sequence_colors(host, self.show_unbuilt)
            self._seq_colored_for = wanted
        with session_log.step("make scene"):
            scene = view.make_scene()
        self._colors = self._part_colors(host)
        self.draw(scene)
        stale = "  ** STALE IK: the robot's saved pose is not at this bar **" if host in self.stale else ""
        print(f"{self.command}: {view.label} ({self.index + 1}/{len(self.views)}){stale}")

    def _part_colors(self, host):
        """Colours of the drawn meshes at this step: the bars and their joints by sequence, a stale pose's robot.

        The viewer hides the document's tube previews and joint blocks and draws the process's meshes instead, so
        the sequence colours `show_sequence_colors` paints on the document never show: the same colours go onto
        the drawn meshes here.
        """
        by_bar = {f"bars/{bar}": color for bar, color in sequence_colors(host).items()}
        colors = dict(by_bar)
        for body_id, body in self.process.bodies.items():
            if body.mount in by_bar:
                colors[body_id] = by_bar[body.mount]
        if host in self.stale:
            colors[self.robot_id] = COLOR_STALE_IK
        colors.update(self._ground_colors)
        return colors

    def draw(self, scene):
        """Draw a scene in the current step's colours; it becomes the scene collision checks look at."""
        rs.EnableRedraw(False)
        try:
            if not self._session_started:
                # * The viewers show the core's state: every body of the process is drawn from it (the planner's
                #   ground and obstacle meshes too; CheckCollision paints the drawn meshes), and every document
                #   layer it stands in for is hidden for the session.
                ik_viz.begin_session(active_mesh_mode=self.mesh_mode, layer_key=IK_LAYER_KEY_ASSEMBLY,
                                     hide_doc_layers=(*ik_viz.replaced_doc_layers(), config.LAYER_ENVIRONMENT,
                                                      config.LAYER_WALKABLE_GROUND))
                self._session_started = True
            ik_viz.update_state(scene, robot_id=self.robot_id, layer_key=IK_LAYER_KEY_ASSEMBLY,
                                mesh_mode=self.mesh_mode, colors=self._colors)
        finally:
            with session_log.step("viewport redraw"):
                rs.EnableRedraw(True)
        self.scene = scene

    # ---- collision diagnostic ------------------------------------------

    @session_log.timed("check collision")
    def check_collision(self, verbose=True):
        """Check the shown scene on every placed robot's mirror; red-highlight the offenders. Returns the count."""
        if self.scene is None or self.view is None:
            print(f"{self.command}: nothing shown to check.")
            return 0
        self._revert_highlight()
        try:
            pairs = process_planning.scene_collisions(self.process, sc.sticky, self.scene)
        except Exception as exc:  # noqa: BLE001 -- a diagnostic must never end the viewer
            session_log.caught(exc)
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
        self._apply_highlight(ik_viz.objects_of(ids, IK_LAYER_KEY_ASSEMBLY, self.mesh_mode))
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
        urdf_joints = load_urdf(robot.model.urdf).joint_map
        for group, side in ((config.LEFT_GROUP, "left"), (config.RIGHT_GROUP, "right")):
            for name in (f"{side}_ur_arm_{joint}" for joint in UR_JOINT_NAMES):
                if name not in urdf_joints:
                    print(f"{self.command}: no joint {name!r} in the robot; skipping it.")
                    continue
                limit = urdf_joints[name].limit
                lo = float(limit.lower) if limit is not None and limit.lower is not None else -math.pi
                hi = float(limit.upper) if limit is not None and limit.upper is not None else math.pi
                if hi <= lo:
                    lo, hi = -math.pi, math.pi
                joint_specs.append((group, name, lo, hi))
        if not joint_specs:
            print(f"{self.command}: no configurable joints found; InteractiveCollisionCheck aborted.")
            return
        mirror = None
        try:
            mirror = process_planning.sync_seeded(sc.sticky, self.scene.copy(), self.robot_id)
        except Exception as exc:  # noqa: BLE001 -- jog still draws without the check
            session_log.caught(exc)
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

    @session_log.timed("revert collision highlight")
    def _revert_highlight(self):
        if not self._highlight_oids:
            return
        with suspend_redraw():
            for oid in self._highlight_oids:
                try:
                    rs.ObjectColorSource(oid, 0)  # ByLayer
                except Exception:  # noqa: BLE001
                    continue
            # The red went onto meshes that may carry a sequence colour: give those back.
            ik_viz.restore_colors(IK_LAYER_KEY_ASSEMBLY)
        self._highlight_oids = []

    # ---- walkable-ground highlight -------------------------------------

    @session_log.timed("highlight walkable ground")
    def _show_walkable_grounds(self):
        """Colour the drawn grounds: the active bar's assigned WalkableGround light grey, the others dark; name it.

        The ground on screen is the process's (`ground/<id>`, the mesh the planner uses): the document's own
        ground layer is hidden for the session, like every layer the preview replaces.
        """
        self._ground_colors = {body_id: WALKABLE_GROUND_OTHER_COLOR for body_id in self.process.bodies
                               if body_id.startswith("ground/")}
        if self.active_bar_oid is None:
            self._recolor()
            return
        ground_ids = walkable_ground.get_bar_ground_ids(self.active_bar_oid)
        if not ground_ids:
            print(f"{self.command}: bar {self.active_bar_id} has no assigned WalkableGround "
                  "(run RSAssignAndShowWalkableGround, or RSUpdatePreview to assign the nearest).")
        for gid in ground_ids:
            if f"ground/{gid}" not in self.process.bodies:
                print(f"{self.command}: bar {self.active_bar_id} references WalkableGround '{gid}' but the process "
                      "has no such ground (no such brep in the document).")
                continue
            self._ground_colors[f"ground/{gid}"] = WALKABLE_GROUND_ACTIVE_COLOR
        if ground_ids:
            print(f"{self.command}: bar {self.active_bar_id} stands on {ground_ids} (light grey; any other ground "
                  "is dark grey).")
        self._recolor()

    def _revert_walkable_grounds(self):
        self._ground_colors = {}

    def _recolor(self):
        """Apply the current step's colours to what is already drawn (nothing moves)."""
        if self.scene is None or not self._session_started or self.view is None:
            return
        self._colors = self._part_colors(self.view.bar_id)
        ik_viz.update_state(self.scene, robot_id=self.robot_id, layer_key=IK_LAYER_KEY_ASSEMBLY,
                            mesh_mode=self.mesh_mode, colors=self._colors)

    @session_log.timed("viewer cleanup")
    def cleanup(self):
        self._revert_highlight()
        self._revert_walkable_grounds()
        if self._session_started:
            ik_viz.end_session()
            self._session_started = False
        try:
            with session_log.step("reset sequence colors"):
                reset_sequence_colors()
        except Exception as exc:  # noqa: BLE001 -- never let cleanup mask the real outcome
            session_log.caught(exc)
            print(f"{self.command}: failed to restore sequence colors ({exc}); continuing.")


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
                process_display.place(scene, session.robot_id, joints={name: rad})
                session.draw(scene)
                n = check()
            except Exception as exc:  # noqa: BLE001
                session_log.caught(exc)
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
            session_log.caught(exc)
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
        session_log.caught(exc)
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
        draw_base_frames(session.process, [nxt])


def main() -> None:
    _reload()
    seq_map = get_bar_seq_map()  # {bar_id: (oid, seq)}

    # If RSLoadSolvedBarAction just wrote solved keyframes, start on those bars (no pick) and draw every
    # loaded bar's base frame. Otherwise, classic flow: pick a bar.
    loaded = sc.sticky.pop(LOADED_STICKY_KEY, None) or {}
    loaded_here = [b for b in loaded.get("bars", []) if b in seq_map]
    process = process_from_document("RSShowBarActionPlan")
    if loaded_here:
        bar_ids = sorted(loaded_here, key=lambda b: seq_map[b][1])
        initial_bar_id, initial_oid = bar_ids[0], seq_map[bar_ids[0]][0]
        draw_base_frames(process, bar_ids)
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
        draw_base_frames(process, [initial_bar_id])

    session = PlanSession(process)
    session.bar_ids = bar_ids
    try:
        session.set_active_bar(initial_bar_id, initial_oid)
        while True:
            go = _build_get_option(session)
            result = session_log.ask(go)
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
                    draw_base_frames(process, [bar_id])
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
        with session_log.step("clear base frame markers"):
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
        res = session_log.ask(go)
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
                if session_log.ask(gi) == Rhino.Input.GetResult.Number:
                    session.step(gi.Number() - 1 - session.index)
            elif name == "Close":
                break


def main_motion(folder=None, bar=None) -> None:
    """Right-click entry: pick a bar and scrub its planned trajectories from a process folder's solutions.

    Args:
        folder (str | None): The process folder; None asks (default: the last one used).
        bar (tuple | None): ``(bar_id, bar_oid)`` to scrub without a pick.
    """
    from bar_assembly_core.process import read
    from bar_assembly_core.process.solutions import is_stale, read_solutions

    _reload()
    folder = process_folder("RSShowBarActionPlan", folder)
    if folder is None:
        return
    try:
        process = read(folder)
        solutions = read_solutions(process)
    except Exception as exc:  # noqa: BLE001 -- a broken folder is reported, not raised
        session_log.caught(exc)
        rs.MessageBox(f"Cannot read the process folder:\n{exc}", 0, "RSShowBarActionPlan")
        return
    stale = sorted(a for a, s in solutions.items() if is_stale(process, s))
    if stale:
        print(f"RSShowBarActionPlan: stale solutions (the process changed since they were planned): {', '.join(stale)}")

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
    frames = plan_steps.trajectory_frames(process, solutions, f"bars/{bar_id}")
    if not frames:
        rs.MessageBox(f"Bar {bar_id} has no planned trajectory in\n{os.path.join(folder, 'solutions')}\n\n"
                      "Plan it first: python scripts/plan_process.py <process folder>.", 0, "RSShowBarActionPlan")
        return

    session = PlanSession(process)
    session.active_bar_id, session.active_bar_oid = bar_id, oid
    try:
        draw_base_frames(process, [bar_id])
        session.set_views(motion_views(process, frames))
        session._show_walkable_grounds()
        _run_motion_command_loop(session)
    finally:
        session.cleanup()
        base_frame_viz.clear_base_frames()


if __name__ == "__main__":
    session_log.run(__file__, main)
