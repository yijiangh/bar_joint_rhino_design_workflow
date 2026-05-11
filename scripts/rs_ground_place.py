#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4

"""RSGroundPlace - Anchor a registered ground joint to a bar at a picked point.

Workflow:

1. Pick a registered bar (centerline or tube preview).
2. Pick a point on the bar -- ``jp`` is set to the projection of the
   picked point onto the bar's Z axis (signed distance from ``bar_start``,
   in mm).
3. If more than one ``GroundJointDef`` is registered, pick one by name;
   otherwise the single registered ground joint is used.
4. ``jr`` is auto-computed via the heuristic
   :func:`core.ground_placement.auto_jr_y_down` so the block's local +Y
   axis points as close to world -Z (ground) as possible.
5. A preview block is inserted with role=ground.  Click the block to flip
   ``jr`` by 180 deg; press **Enter** / click **Accept** to bake the
   final instance, **Escape** to cancel.

The baked instance lives on ``LAYER_JOINT_GROUND_INSTANCES`` and carries
``joint_id`` / ``ground_joint_name`` / ``parent_bar_id`` / ``position_mm``
/ ``rotation_deg`` UserText so :mod:`rs_joint_edit` can re-edit it.
"""

import importlib
import math
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import joint_pair as _joint_pair_module
from core import joint_placement as _joint_placement_module
from core import ground_placement as _ground_placement_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import ensure_bar_id, repair_on_entry
from core.rhino_helpers import curve_endpoints, delete_objects, suspend_redraw


def _reload_runtime_modules():
    global config, joint_pair_module, joint_placement, ground_placement
    global JOINT_ROLE_GROUND, GROUND_INSTANCES_LAYER
    config = importlib.reload(_config_module)
    joint_pair_module = importlib.reload(_joint_pair_module)
    joint_placement = importlib.reload(_joint_placement_module)
    ground_placement = importlib.reload(_ground_placement_module)
    JOINT_ROLE_GROUND = ground_placement.JOINT_ROLE_GROUND
    GROUND_INSTANCES_LAYER = ground_placement.GROUND_INSTANCES_LAYER


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Pickers
# ---------------------------------------------------------------------------


def _pick_point_on_bar(bar_id, prompt: str):
    """Prompt the user for a point on the bar curve.

    Uses ``Rhino.Input.Custom.GetPoint`` constrained to the bar curve so
    the cursor slides along it.  Returns a numpy 3-vector or ``None``.
    """
    curve = rs.coercecurve(bar_id)
    if curve is None:
        return None
    gp = Rhino.Input.Custom.GetPoint()
    gp.SetCommandPrompt(prompt)
    gp.Constrain(curve, False)
    gp.PermitObjectSnap(False)
    if gp.Get() != Rhino.Input.GetResult.Point:
        return None
    pt = gp.Point()
    return np.array([pt.X, pt.Y, pt.Z], dtype=float)


def _pick_ground_def(prompt: str = "Select ground joint definition"):
    """Return a ``GroundJointDef`` selected by the user, or ``None``."""
    names = joint_pair_module.list_ground_joint_names()
    if not names:
        rs.MessageBox(
            "No ground joints are registered.\n"
            "Use RSDefineJointHalf with kind=Ground first.",
            0,
            "RSGroundPlace",
        )
        return None
    if len(names) == 1:
        chosen = names[0]
    else:
        chosen = rs.ListBox(names, "Pick a ground joint", "RSGroundPlace")
        if not chosen:
            return None
    reg = joint_pair_module.load_joint_registry()
    return reg.ground_joints[chosen]


# ---------------------------------------------------------------------------
# Interactive preview session
# ---------------------------------------------------------------------------


class _GroundSession:
    """Holds the live preview state for one ground placement."""

    def __init__(self, *, ground, bar_id, bar_start, bar_end, jp, jr):
        self.ground = ground
        self.bar_id = bar_id
        self.bar_start = bar_start
        self.bar_end = bar_end
        self.jp = float(jp)
        self.jr = float(jr)
        # ``flipped`` post-multiplies M_block_from_bar by R_y(pi):
        # block-local +X reverses (block faces the other way along the bar)
        # while block-local +Y is preserved (so it stays Y-down aligned).
        self.flipped = False
        self.preview_id = None

    def _frame(self):
        return ground_placement.fk_ground_block_frame(
            self.bar_start, self.bar_end, self.jp, self.jr, self.ground,
            flipped=self.flipped,
        )

    def show(self):
        """(Re-)insert the preview block at the current jr / flipped state."""
        with suspend_redraw():
            if self.preview_id is not None:
                delete_objects([self.preview_id])
                self.preview_id = None
            self.preview_id = ground_placement.insert_ground_block_preview(
                self.ground.block_name, self._frame()
            )
        print(
            f"RSGroundPlace: jp={self.jp:.2f} mm, "
            f"jr={math.degrees(self.jr):.1f} deg, flipped={self.flipped}"
        )

    def flip(self):
        """Reverse the block's X axis along the bar (Y stays world-down)."""
        self.flipped = not self.flipped
        self.show()

    def cleanup(self):
        if self.preview_id is not None:
            delete_objects([self.preview_id])
            self.preview_id = None


def _ground_role_filter(rhino_object, geometry, component_index):
    """Geometry filter -- accept only the live ground preview block."""
    return rs.GetUserText(rhino_object.Id, "_joint_role") == JOINT_ROLE_GROUND


def _interactive_loop(session: _GroundSession) -> bool:
    """Run the click-to-flip + Accept/Cancel loop.  Returns True on accept."""
    session.show()
    try:
        while True:
            go = Rhino.Input.Custom.GetObject()
            go.SetCommandPrompt(
                "Click ground block to flip its X axis along the bar "
                "(Enter = Accept, Esc = Cancel)"
            )
            go.EnablePreSelect(False, False)
            go.AcceptNothing(True)
            go.SetCustomGeometryFilter(_ground_role_filter)
            go.AddOption("Accept")
            go.AddOption("Flip")

            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                return False
            if result == Rhino.Input.GetResult.Nothing:
                return True
            if result == Rhino.Input.GetResult.Option:
                opt = go.Option()
                if opt is None:
                    continue
                if opt.EnglishName == "Accept":
                    return True
                if opt.EnglishName == "Flip":
                    session.flip()
                continue
            if result == Rhino.Input.GetResult.Object:
                # Any click on the preview block means flip.
                session.flip()
    finally:
        session.cleanup()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSGroundPlace")

    rs.UnselectAllObjects()

    bar_id = pick_bar("Select the bar to anchor a ground joint to")
    if bar_id is None:
        return
    bar_bid = ensure_bar_id(bar_id)
    bar_start, bar_end = curve_endpoints(bar_id)
    bar_start = np.asarray(bar_start, dtype=float)
    bar_end = np.asarray(bar_end, dtype=float)

    point = _pick_point_on_bar(bar_id, "Pick a point on the bar")
    if point is None:
        return
    bar_dir = bar_end - bar_start
    bar_len = float(np.linalg.norm(bar_dir))
    if bar_len <= 0.0:
        print("RSGroundPlace: bar has zero length; aborting.")
        return
    bar_z = bar_dir / bar_len
    jp = float(np.dot(point - bar_start, bar_z))
    print(f"RSGroundPlace: anchored to {bar_bid} at jp={jp:.2f} mm.")

    ground = _pick_ground_def()
    if ground is None:
        return

    jr = ground_placement.auto_jr_y_down(bar_start, bar_end, ground)
    print(
        f"RSGroundPlace: using ground joint '{ground.name}' "
        f"(block='{ground.block_name}'); auto jr={math.degrees(jr):.1f} deg."
    )

    # Make sure the block definition is loaded so the preview can insert it.
    from core.rhino_block_import import require_block_definition  # noqa: PLC0415
    try:
        require_block_definition(ground.block_name, asset_path=ground.asset_path())
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSGroundPlace")
        return

    session = _GroundSession(
        ground=ground,
        bar_id=bar_bid,
        bar_start=bar_start,
        bar_end=bar_end,
        jp=jp,
        jr=jr,
    )
    accepted = _interactive_loop(session)
    if not accepted:
        print("RSGroundPlace: Cancelled.")
        return

    # NOTE: a fresh joint_id (with the next free index suffix) is allocated
    # inside place_ground_block, so multiple ground joints with the same
    # ground definition coexist on the same bar.  No pre-bake delete here.

    ground_oid, joint_id = ground_placement.place_ground_block(
        ground=ground,
        bar_id=bar_bid,
        bar_start=bar_start,
        bar_end=bar_end,
        jp=session.jp,
        jr=session.jr,
        flipped=session.flipped,
    )

    # Auto-place the default robotic tool behind the ground joint, mirroring
    # the male-joint workflow.  Idempotent: re-runs cleanly via
    # `remove_tool_for_joint` inside `place_tool_at_block_instance`.
    from core.rhino_tool_place import auto_place_tool_at_ground_block  # noqa: PLC0415
    auto_place_tool_at_ground_block(ground_oid, joint_id)


if __name__ == "__main__":
    main()
