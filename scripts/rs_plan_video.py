#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSPlanVideo - look at the assembly plan through a camera, and write it out as frames for a video. Prototype.

Pick a curve as the camera path (Enter keeps the current view) and the point the camera looks at (Enter keeps the
view's target). A small window then previews live in the active perspective view:

    Step     the assembly step, as RSShowAssemblyPlan walks them
    Camera   where the camera stands on the curve, 0 to 100 % of its length
    Lens     35 mm equivalent focal length

Export frames writes one PNG per step, frame_0000.png ..., the camera fixed or moving along the curve as the steps
play. Join them with any video editor, or ffmpeg:  ffmpeg -framerate 4 -i frame_%04d.png -pix_fmt yuv420p plan.mp4

The drawing is the Grasshopper sequence preview's (`core.gh_seq_preview`), so both show the same. Closing the window
restores the document and the camera.
"""

from __future__ import annotations

import os
import sys

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

from core import gh_bridge, gh_seq_preview  # noqa: E402

#: Key of the preview's state in `sc.sticky` (through `gh_bridge.state`).
STATE_ID = "plan_video"
#: Settings kept for the next run: lens, frame size, folder, camera moving during export.
SETTINGS_KEY = "bar_joint:plan_video_settings"
#: Resolution of the camera slider.
CAMERA_STEPS = 1000


class PlanVideo:
    """The plan drawn at one step, seen from one camera; writes frames.

    Args:
        view: The Rhino view to drive and capture (perspective).
        path: Camera path, a `Rhino.Geometry.Curve`; None keeps the camera where it is.
        target: What the camera looks at, a `Rhino.Geometry.Point3d`; None keeps the view's target.
        lens: 35 mm equivalent focal length; None keeps the view's.
    """

    def __init__(self, view, path=None, target=None, lens=None):
        self.view = view
        self.path = path
        self.target = target
        viewport = view.ActiveViewport
        self._saved = (viewport.CameraLocation, viewport.CameraTarget, viewport.Camera35mmLensLength,
                       viewport.CameraUp)
        self.lens = float(lens or viewport.Camera35mmLensLength)
        self.step = 0
        self.t = 0.0
        self.frame_count = 0
        self.label = ""

    def start(self) -> dict:
        """Build the timeline from the document and show its first step."""
        result = gh_seq_preview.run(STATE_ID, enable=True, reload=True, step=0)
        self.frame_count = result["frame_count"]
        return self.show(step=0)

    def show(self, step=None, t=None, lens=None) -> dict:
        """Move to a step, a camera position and a lens (each only if given), and redraw."""
        if step is not None:
            self.step = max(0, min(int(step), max(self.frame_count - 1, 0)))
        if t is not None:
            self.t = max(0.0, min(float(t), 1.0))
        if lens is not None:
            self.lens = float(lens)
        self._place_camera()
        result = gh_seq_preview.run(STATE_ID, enable=True, step=self.step)
        self.label = result["step_label"] or result["info"]
        self.view.Redraw()
        return result

    def _place_camera(self) -> None:
        viewport = self.view.ActiveViewport
        location = self.path.PointAtNormalizedLength(self.t) if self.path is not None else viewport.CameraLocation
        target = self.target if self.target is not None else viewport.CameraTarget
        if location.DistanceTo(target) > 1e-6:
            viewport.SetCameraLocations(target, location)
            viewport.CameraUp = Rhino.Geometry.Vector3d.ZAxis
        viewport.Camera35mmLensLength = self.lens

    def capture(self, file_path: str, width: int, height: int) -> None:
        """Save the view as it is drawn now, at `width` x `height` pixels."""
        capture = Rhino.Display.ViewCapture()
        capture.Width, capture.Height = int(width), int(height)
        capture.ScaleScreenItems = False
        capture.DrawAxes = False
        capture.DrawGrid = False
        bitmap = capture.CaptureToBitmap(self.view)
        if bitmap is None:
            raise RuntimeError(f"Rhino could not capture the view for {file_path}")
        bitmap.Save(file_path)
        bitmap.Dispose()

    def export(self, folder: str, width: int, height: int, move_camera: bool, progress=None) -> int:
        """Write one frame per step into `folder`; the camera sweeps the whole path when `move_camera`.

        Returns:
            The number of frames written.
        """
        os.makedirs(folder, exist_ok=True)
        fixed_t = self.t
        n = self.frame_count
        for index in range(n):
            t = index / (n - 1) if move_camera and self.path is not None and n > 1 else fixed_t
            self.show(step=index, t=t)
            self.capture(os.path.join(folder, f"frame_{index:04d}.png"), width, height)
            if progress is not None:
                progress(index + 1, n)
        return n

    def close(self) -> None:
        """Restore the document and the camera."""
        gh_seq_preview.run(STATE_ID, enable=False)
        gh_bridge.forget(STATE_ID)
        location, target, lens, up = self._saved
        viewport = self.view.ActiveViewport
        viewport.SetCameraLocations(target, location)
        viewport.CameraUp = up
        viewport.Camera35mmLensLength = lens
        self.view.Redraw()


def _settings() -> dict:
    defaults = {"width": 1920, "height": 1080, "folder": None, "move_camera": True}
    return {**defaults, **(sc.sticky.get(SETTINGS_KEY) or {})}


def run_window(video: PlanVideo) -> None:
    """The live preview window: step and camera sliders, lens, frame size, Export frames."""
    import Eto.Drawing as drawing
    import Eto.Forms as forms

    # Eto constructors take no keyword arguments under Rhino's CPython: construct, then assign.
    def label(text):
        control = forms.Label()
        control.Text = text
        return control

    def button(text):
        control = forms.Button()
        control.Text = text
        return control

    def stepper(value, low, high, places=0):
        control = forms.NumericStepper()
        control.MinValue, control.MaxValue, control.DecimalPlaces = low, high, places
        control.Value = value
        return control

    settings = _settings()
    dialog = forms.Dialog[bool]()
    dialog.Title = "RSPlanVideo"
    dialog.Padding = drawing.Padding(8)
    dialog.Resizable = True
    dialog.MinimumSize = drawing.Size(560, 0)

    status = label(video.label)
    step_slider = forms.Slider()
    step_slider.MinValue, step_slider.MaxValue = 0, max(video.frame_count - 1, 0)
    step_slider.Value = video.step
    camera_slider = forms.Slider()
    camera_slider.MinValue, camera_slider.MaxValue = 0, CAMERA_STEPS
    camera_slider.Value = int(round(video.t * CAMERA_STEPS))
    camera_slider.Enabled = video.path is not None
    lens = stepper(video.lens, 5, 600, 1)
    width = stepper(settings["width"], 64, 8000)
    height = stepper(settings["height"], 64, 8000)
    move_camera = forms.CheckBox()
    move_camera.Text = "Camera moves along the curve during export"
    move_camera.Checked = bool(settings["move_camera"]) and video.path is not None
    move_camera.Enabled = video.path is not None
    export_button = button("Export frames...")
    close_button = button("Close")

    def redraw(**change):
        try:
            video.show(**change)
            status.Text = video.label
        except Exception as exc:  # noqa: BLE001  (a failed redraw must not close the window)
            session_log.caught(exc)
            status.Text = f"redraw failed: {exc}"

    def on_step(_sender, _args):
        if step_slider.Value != video.step:
            redraw(step=step_slider.Value)

    def on_camera(_sender, _args):
        redraw(t=camera_slider.Value / CAMERA_STEPS)

    def on_lens(_sender, _args):
        redraw(lens=lens.Value)

    def on_export(_sender, _args):
        folder = rs.BrowseForFolder(folder=settings["folder"] if settings["folder"] and os.path.isdir(settings["folder"])
                                    else None, message="Folder for the frames (frame_0000.png ...)",
                                    title="RSPlanVideo")
        if not folder:
            return
        settings.update(folder=folder, width=int(width.Value), height=int(height.Value),
                        move_camera=bool(move_camera.Checked))
        sc.sticky[SETTINGS_KEY] = dict(settings)

        def progress(done, total):
            status.Text = f"writing frame {done}/{total} ..."
            Rhino.RhinoApp.Wait()

        try:
            with session_log.step("export frames", folder):
                n = video.export(folder, settings["width"], settings["height"], settings["move_camera"], progress)
        except Exception as exc:  # noqa: BLE001
            session_log.caught(exc)
            status.Text = f"export failed: {exc}"
            return
        step_slider.Value = video.step
        camera_slider.Value = int(round(video.t * CAMERA_STEPS))
        status.Text = f"wrote {n} frames to {folder}"
        print(f"RSPlanVideo: wrote {n} frames to {folder}\n"
              f"  as a video: ffmpeg -framerate 4 -i frame_%04d.png -pix_fmt yuv420p plan.mp4")

    step_slider.ValueChanged += on_step
    camera_slider.ValueChanged += on_camera
    lens.ValueChanged += on_lens
    export_button.Click += on_export
    close_button.Click += lambda _sender, _args: dialog.Close(True)

    layout = forms.DynamicLayout()
    layout.Spacing = drawing.Size(6, 6)
    layout.AddRow(status)
    layout.AddRow(label("Step"), step_slider)
    layout.AddRow(label("Camera"), camera_slider)
    layout.AddRow(label("Lens (mm)"), lens)
    layout.AddRow(label("Frame size (px)"), forms.TableLayout.HorizontalScaled(width, height))
    layout.AddRow(None, move_camera)
    layout.AddRow(None, forms.TableLayout.HorizontalScaled(export_button, close_button))
    dialog.Content = layout
    dialog.AbortButton = close_button
    try:
        parent = Rhino.UI.RhinoEtoApp.MainWindow
    except Exception:  # noqa: BLE001
        parent = None
    dialog.ShowModal(parent) if parent is not None else dialog.ShowModal()


def main() -> None:
    view = sc.doc.Views.ActiveView
    if not view.ActiveViewport.IsPerspectiveProjection:
        print("RSPlanVideo: make a perspective view active first.")
        return
    curve_id = rs.GetObject("Camera path curve (Enter: keep the current view)", rs.filter.curve)
    path = rs.coercecurve(curve_id) if curve_id else None
    point = rs.GetPoint("Point the camera looks at (Enter: keep the view's target)")
    video = PlanVideo(view, path=path, target=point)
    try:
        with session_log.step("build the timeline"):
            video.start()
        if not video.frame_count:
            print(f"RSPlanVideo: {video.label}")
            return
        run_window(video)
    finally:
        video.close()


if __name__ == "__main__":
    session_log.run(__file__, main)
