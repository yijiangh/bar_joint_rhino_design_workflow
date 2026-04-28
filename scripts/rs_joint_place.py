#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSJointPlace - Place connector blocks on one bar pair.

The user is first prompted to confirm the active joint pair (Enter accepts
the last-used pair, or pick a different one from the registered list).
Then select any two registered bars.  The bar with the lower assembly
sequence number is automatically assigned as the female-joint bar (Le);
the bar assembled later receives the male joint (Ln).

The optimizer solves all 4 endpoint-reversal variants.  After an initial
placement you can refine interactively:

  - **Click the female block** to toggle the female joint orientation
    (switches between variant pairs 0↔1 and 2↔3).
  - **Click the male block** to toggle the male joint orientation
    (switches between variant groups 0↔2 and 1↔3).
  - Press **Enter** or click **Accept** to confirm and write the final blocks.
  - Press **Escape** to cancel and remove the preview.
"""

import contextlib
import importlib
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import joint_pair as _joint_pair_module
from core import joint_pair_solver as _joint_pair_solver_module
from core.rhino_helpers import (
    curve_endpoints,
    delete_objects,
    ensure_layer,
    point_to_array,
    set_object_color,
    set_objects_layer,
    suspend_redraw,
)
from core.rhino_bar_registry import (
    ensure_bar_id,
    get_bar_seq_map,
    pick_bar,
    repair_on_entry,
)
from core.rhino_block_import import (
    has_block_definition as _has_block_definition,
    import_block_definition_from_3dm as _import_block_definition_from_3dm,
    require_block_definition as _require_block_definition,
)
from core.rhino_pair_selector import pick_bar_with_pair_option


_FEMALE_INSTANCES_LAYER = _config_module.LAYER_JOINT_FEMALE_INSTANCES
_MALE_INSTANCES_LAYER = _config_module.LAYER_JOINT_MALE_INSTANCES
_PREVIEW_COLORS = [
    (230, 80, 80),
    (80, 80, 230),
    (80, 200, 80),
    (200, 160, 50),
]
_JOINT_ROLE_KEY = "_joint_role"  # user-text key set on interactive preview blocks

# A solved variant is considered acceptable when the female/male screw
# frames coincide within these tolerances.  Special joint geometries
# sometimes have only two valid variants (out of the nominal four); the
# other two land on local minima with large interface errors.  The
# auto-recovery logic below detects this and flips one side once.
_VARIANT_OK_ORIGIN_TOL_MM = 0.05
_VARIANT_OK_Z_AXIS_TOL_RAD = 0.01  # ~0.57 deg


# ---------------------------------------------------------------------------
# Module reload
# ---------------------------------------------------------------------------


def _reload_runtime_modules():
    global config, joint_pair_module, joint_pair_solver_module
    global optimize_pair_placement, screw_alignment_diagnostics, get_joint_pair

    config = importlib.reload(_config_module)
    joint_pair_module = importlib.reload(_joint_pair_module)
    joint_pair_solver_module = importlib.reload(_joint_pair_solver_module)
    optimize_pair_placement = joint_pair_solver_module.optimize_pair_placement
    screw_alignment_diagnostics = joint_pair_solver_module.screw_alignment_diagnostics
    get_joint_pair = joint_pair_module.get_joint_pair


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Generic helpers (point_to_array, curve_endpoints, as_object_id_list,
# ensure_layer, set_objects_layer, delete_objects,
# set_object_color, suspend_redraw) are imported from core.rhino_helpers.
# Block-import helpers (_has_block_definition, _import_block_definition_from_3dm,
# _require_block_definition) are imported from core.rhino_block_import.


def _numpy_to_rhino_transform(matrix):
    xform = Rhino.Geometry.Transform(1.0)
    for row in range(4):
        for col in range(4):
            xform[row, col] = float(matrix[row, col])
    return xform


def _insert_block_instance(
    block_name, frame, *, layer_name=None, color=None, role=None
):
    oid = rs.InsertBlock(block_name, [0, 0, 0])
    if oid is None:
        raise RuntimeError(f"Failed to insert Rhino block '{block_name}'.")
    rs.TransformObject(oid, _numpy_to_rhino_transform(frame))
    if layer_name:
        set_objects_layer(oid, layer_name)
    if color is not None:
        set_object_color(oid, color)
    if role is not None:
        rs.SetUserText(oid, _JOINT_ROLE_KEY, role)
    return oid


# ---------------------------------------------------------------------------
# Variant enumeration
# ---------------------------------------------------------------------------
# The 4 assembly variants come from reversing the bar endpoint order.
# Swapping start/end of a bar reverses its frame Z-axis, which reorients
# the connector along the bar.  Each combination is a full 4-DOF re-opt.
# ---------------------------------------------------------------------------


def _compute_variant(le_start, le_end, ln_start, ln_end, le_rev, ln_rev, *, pair):
    """Compute a single joint-placement optimization for the given reversal flags.

    ``le_rev`` reverses the female-bar endpoint order; ``ln_rev`` reverses the
    male-bar endpoint order.  ``pair`` is the chosen :class:`JointPairDef`.
    """
    les = le_end if le_rev else le_start
    lee = le_start if le_rev else le_end
    lns = ln_end if ln_rev else ln_start
    lne = ln_start if ln_rev else ln_end
    idx = (2 if ln_rev else 0) + (1 if le_rev else 0)
    label = (
        f"{idx + 1}: le={'rev' if le_rev else 'fwd'}, ln={'rev' if ln_rev else 'fwd'}"
    )
    res = optimize_pair_placement(les, lee, lns, lne, pair, return_debug=True)
    diag = screw_alignment_diagnostics(
        res["female_screw_frame"],
        res["male_screw_frame"],
    )
    res.update(
        {
            "variant_index": idx,
            "variant_label": label,
            "variant_solver": "full_pair_optimization",
            "female_flip_rad": float(np.pi) if le_rev else 0.0,
            "male_flip_rad": float(np.pi) if ln_rev else 0.0,
            "origin_error_mm": diag["origin_error_mm"],
            "z_axis_error_rad": diag["z_axis_error_rad"],
            "_le_start": les,
            "_le_end": lee,
            "_ln_start": lns,
            "_ln_end": lne,
        }
    )
    return res


# ---------------------------------------------------------------------------
# Interface metrics
# ---------------------------------------------------------------------------


def _interface_metrics_from_result(result):
    if "origin_error_mm" in result and "z_axis_error_rad" in result:
        return float(result["origin_error_mm"]), float(result["z_axis_error_rad"])
    if "female_screw_frame" in result and "male_screw_frame" in result:
        diagnostics = screw_alignment_diagnostics(
            result["female_screw_frame"],
            result["male_screw_frame"],
        )
        return float(diagnostics["origin_error_mm"]), float(
            diagnostics["z_axis_error_rad"]
        )
    raise KeyError("Result is missing interface diagnostics.")


def is_variant_acceptable(variant):
    """True iff the variant's interface error is within tolerance."""
    origin_err, z_err = _interface_metrics_from_result(variant)
    return (
        origin_err <= _VARIANT_OK_ORIGIN_TOL_MM
        and abs(z_err) <= _VARIANT_OK_Z_AXIS_TOL_RAD
    )


def compute_variant_with_recovery(
    le_start,
    le_end,
    ln_start,
    ln_end,
    le_rev,
    ln_rev,
    *,
    pair,
    recover_side="female",
):
    """Compute the requested variant.  If its interface error exceeds
    tolerance (a sign that this orientation lands on a degenerate local
    minimum), flip ``recover_side`` once and use that variant instead.

    Returns ``(variant, recovered_bool, new_le_rev, new_ln_rev)``.  The
    flag is True when recovery was applied; the new flags reflect the
    actually-used orientation.  Recovery never propagates more than once.
    """
    variant = _compute_variant(
        le_start, le_end, ln_start, ln_end, le_rev, ln_rev, pair=pair
    )
    if is_variant_acceptable(variant):
        return variant, False, le_rev, ln_rev

    origin_err, z_err = _interface_metrics_from_result(variant)
    print(
        f"RSJointPlace: variant {variant['variant_index'] + 1} interface error "
        f"too large (origin={origin_err:.4f} mm, z={np.degrees(z_err):.4f}deg); "
        f"auto-flipping {recover_side} side."
    )
    if recover_side == "female":
        new_le, new_ln = (not le_rev), ln_rev
    else:
        new_le, new_ln = le_rev, (not ln_rev)
    recovered = _compute_variant(
        le_start, le_end, ln_start, ln_end, new_le, new_ln, pair=pair
    )
    rec_o, rec_z = _interface_metrics_from_result(recovered)
    if is_variant_acceptable(recovered):
        print(
            f"  recovery succeeded -> variant {recovered['variant_index'] + 1} "
            f"(origin={rec_o:.4f} mm, z={np.degrees(rec_z):.4f}deg)."
        )
    else:
        print(
            f"  WARNING: recovery did not improve interface error "
            f"(origin={rec_o:.4f} mm, z={np.degrees(rec_z):.4f}deg); "
            f"placing the recovered variant as-is."
        )
    return recovered, True, new_le, new_ln


# ---------------------------------------------------------------------------
# Interactive cycling loop
# ---------------------------------------------------------------------------


class _JointSession:
    """Tracks interactive state for one joint-placement run.

    The four solver variants map to a 2-bit state:

    - ``le_rev`` (bit 0): female joint orientation — toggled by clicking the female block.
    - ``ln_rev`` (bit 1): male joint orientation — toggled by clicking the male block.

    variant_index = (2 if ln_rev else 0) + (1 if le_rev else 0)

    Variants are computed on demand (lazy) and cached so that toggling back to
    a previously-seen orientation does not re-run the solver.
    """

    def __init__(
        self,
        le_start,
        le_end,
        ln_start,
        ln_end,
        female_block_name,
        male_block_name,
        *,
        pair,
        le_rev=False,
        ln_rev=False,
    ):
        self.le_start = le_start
        self.le_end = le_end
        self.ln_start = ln_start
        self.ln_end = ln_end
        self.female_block_name = female_block_name
        self.male_block_name = male_block_name
        self.pair = pair
        self.le_rev = le_rev
        self.ln_rev = ln_rev
        self._cache = {}  # (le_rev, ln_rev) -> variant dict
        self.female_id = None
        self.male_id = None

    def _get_variant(self, le_rev, ln_rev):
        """Return the variant for the given flags, computing it on demand."""
        key = (le_rev, ln_rev)
        if key not in self._cache:
            self._cache[key] = _compute_variant(
                self.le_start,
                self.le_end,
                self.ln_start,
                self.ln_end,
                le_rev,
                ln_rev,
                pair=self.pair,
            )
        return self._cache[key]

    @property
    def current_idx(self):
        return (2 if self.ln_rev else 0) + (1 if self.le_rev else 0)

    @property
    def current_variant(self):
        return self._get_variant(self.le_rev, self.ln_rev)

    def _is_female(self, oid):
        return rs.GetUserText(oid, _JOINT_ROLE_KEY) == "female"

    def _is_male(self, oid):
        return rs.GetUserText(oid, _JOINT_ROLE_KEY) == "male"

    def show_current(self):
        """Delete existing preview and show the current variant."""
        to_delete = [i for i in [self.female_id, self.male_id] if i is not None]
        with suspend_redraw():
            delete_objects(to_delete)
            var = self.current_variant
            color = _PREVIEW_COLORS[self.current_idx % len(_PREVIEW_COLORS)]
            self.female_id = _insert_block_instance(
                self.female_block_name, var["female_frame"], color=color, role="female"
            )
            self.male_id = _insert_block_instance(
                self.male_block_name, var["male_frame"], color=color, role="male"
            )
        _print_variant_info(var)

    def click_female(self):
        """Toggle the female joint orientation and refresh the preview.

        If the new orientation lands on a bad local minimum, flip the
        male side once to recover (mirrors the auto-recovery used by
        ``RSJointEdit``).
        """
        self.le_rev = not self.le_rev
        if not is_variant_acceptable(self.current_variant):
            origin_err, z_err = _interface_metrics_from_result(self.current_variant)
            print(
                f"RSJointPlace: female-flip variant interface error too large "
                f"(origin={origin_err:.4f} mm, z={np.degrees(z_err):.4f}deg); "
                f"auto-flipping male side."
            )
            self.ln_rev = not self.ln_rev
        self.show_current()

    def click_male(self):
        """Toggle the male joint orientation and refresh the preview.

        If the new orientation lands on a bad local minimum, flip the
        female side once to recover.
        """
        self.ln_rev = not self.ln_rev
        if not is_variant_acceptable(self.current_variant):
            origin_err, z_err = _interface_metrics_from_result(self.current_variant)
            print(
                f"RSJointPlace: male-flip variant interface error too large "
                f"(origin={origin_err:.4f} mm, z={np.degrees(z_err):.4f}deg); "
                f"auto-flipping female side."
            )
            self.le_rev = not self.le_rev
        self.show_current()

    def cleanup(self):
        """Delete preview block instances."""
        to_delete = [i for i in [self.female_id, self.male_id] if i is not None]
        delete_objects(to_delete)
        self.female_id = None
        self.male_id = None


def _joint_role_filter(rhino_object, geometry, component_index):
    """Geometry filter — accept only the current interactive preview blocks."""
    return rs.GetUserText(rhino_object.Id, _JOINT_ROLE_KEY) in ("female", "male")


def _interactive_click_loop(session):
    """Let the user click female/male preview blocks to toggle joint orientations.

    Returns the chosen variant dict, or ``None`` if cancelled.
    """
    session.show_current()
    try:
        while True:
            go = Rhino.Input.Custom.GetObject()
            go.SetCommandPrompt(
                "Click female block to flip female, click male block to flip male"
                "  (Enter = Accept)"
            )
            go.EnablePreSelect(False, False)
            go.AcceptNothing(True)
            go.SetCustomGeometryFilter(_joint_role_filter)
            go.AddOption("Accept")

            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                return None

            if result == Rhino.Input.GetResult.Nothing:
                return session.current_variant

            if result == Rhino.Input.GetResult.Option:
                if go.Option().EnglishName == "Accept":
                    return session.current_variant

            if result == Rhino.Input.GetResult.Object:
                clicked_id = go.Object(0).ObjectId
                if session._is_female(clicked_id):
                    session.click_female()
                elif session._is_male(clicked_id):
                    session.click_male()
    finally:
        session.cleanup()


def _print_variant_info(variant):
    origin_err, z_err = _interface_metrics_from_result(variant)
    print(
        f"RSJointPlace: Showing variant {variant['variant_index'] + 1}/4 — "
        f"{variant.get('variant_label', '?')} | "
        f"residual={variant['residual']:.6f}, "
        f"origin err={origin_err:.4f} mm, "
        f"z err={np.degrees(z_err):.4f}deg"
    )


# ---------------------------------------------------------------------------
# Place final blocks
# ---------------------------------------------------------------------------


def _place_joint_blocks(result, le_id, ln_id, le_bar_id, ln_bar_id, *, pair):
    female_frame = result["female_frame"]
    male_frame = result["male_frame"]
    origin_err, z_err = _interface_metrics_from_result(result)

    female_block_name = _require_block_definition(
        pair.female.block_name, asset_path=pair.female.asset_path()
    )
    male_block_name = _require_block_definition(
        pair.male.block_name, asset_path=pair.male.asset_path()
    )

    le_num = le_bar_id.lstrip("B")
    ln_num = ln_bar_id.lstrip("B")
    joint_id = f"J{le_num}-{ln_num}"

    female_type, _, female_subtype = female_block_name.partition("_")
    male_type, _, male_subtype = male_block_name.partition("_")

    # Orientation: P if block x-axis points toward bar end, N toward start
    le_start, le_end = curve_endpoints(le_id)
    le_dir = np.array(le_end) - np.array(le_start)
    le_dir /= np.linalg.norm(le_dir)
    ln_start, ln_end = curve_endpoints(ln_id)
    ln_dir = np.array(ln_end) - np.array(ln_start)
    ln_dir /= np.linalg.norm(ln_dir)

    female_ori = "P" if float(np.dot(female_frame[:3, 0], le_dir)) > 0 else "N"
    male_ori = "P" if float(np.dot(male_frame[:3, 0], ln_dir)) > 0 else "N"

    with suspend_redraw():
        female_id = _insert_block_instance(
            female_block_name, female_frame, layer_name=_FEMALE_INSTANCES_LAYER
        )
        male_id = _insert_block_instance(
            male_block_name, male_frame, layer_name=_MALE_INSTANCES_LAYER
        )
        # Set object names so RSJointEdit can retrieve blocks by joint_id.
        rs.ObjectName(female_id, f"{joint_id}_female")
        rs.ObjectName(male_id, f"{joint_id}_male")
        # Derive le_rev / ln_rev from variant_index so the session state can
        # be restored without re-computing when RSJointEdit opens this joint.
        variant_index = result.get("variant_index", 0)
        le_rev_val = bool(variant_index & 1)
        ln_rev_val = bool((variant_index >> 1) & 1)
        for (
            obj_id,
            block_type,
            block_subtype,
            parent_bar,
            conn_bar,
            pos,
            rot_rad,
            ori,
        ) in [
            (
                female_id,
                female_type,
                female_subtype,
                le_bar_id,
                ln_bar_id,
                result["fjp"],
                result["fjr"],
                female_ori,
            ),
            (
                male_id,
                male_type,
                male_subtype,
                ln_bar_id,
                le_bar_id,
                result["mjp"],
                result["mjr"],
                male_ori,
            ),
        ]:
            rs.SetUserText(obj_id, "joint_id", joint_id)
            rs.SetUserText(obj_id, "joint_type", block_type)
            rs.SetUserText(obj_id, "joint_subtype", block_subtype)
            rs.SetUserText(obj_id, "joint_pair_name", pair.name)
            rs.SetUserText(obj_id, "parent_bar_id", parent_bar)
            rs.SetUserText(obj_id, "connected_bar_id", conn_bar)
            rs.SetUserText(obj_id, "female_parent_bar", le_bar_id)
            rs.SetUserText(obj_id, "male_parent_bar", ln_bar_id)
            rs.SetUserText(obj_id, "position_mm", f"{float(pos):.4f}")
            rs.SetUserText(obj_id, "rotation_deg", f"{float(np.degrees(rot_rad)):.4f}")
            rs.SetUserText(obj_id, "ori", ori)
            rs.SetUserText(obj_id, "le_rev", str(le_rev_val))
            rs.SetUserText(obj_id, "ln_rev", str(ln_rev_val))
            rs.SetUserText(obj_id, "variant_index", str(variant_index))

    print(f"RSJointPlace: Joints placed. Residual: {result['residual']:.6f}")
    if result.get("variant_index") is not None:
        print(
            f"  Variant {int(result['variant_index']) + 1}: "
            f"female flip={np.degrees(float(result.get('female_flip_rad', 0.0))):.1f}deg, "
            f"male flip={np.degrees(float(result.get('male_flip_rad', 0.0))):.1f}deg"
        )
    print(
        f"  FJP={result['fjp']:.2f}, FJR={np.degrees(result['fjr']):.1f}deg, "
        f"MJP={result['mjp']:.2f}, MJR={np.degrees(result['mjr']):.1f}deg"
    )
    print(
        f"  Interface origin err={origin_err:.4f} mm, z-axis err={np.degrees(z_err):.4f}deg"
    )
    return female_id, male_id, joint_id


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSJointPlace")

    rs.UnselectAllObjects()

    bar_a_id, pair = pick_bar_with_pair_option(
        "Select first bar of the joint pair", command_name="RSJointPlace"
    )
    if bar_a_id is None or pair is None:
        return
    print(
        f"RSJointPlace: using pair '{pair.name}' "
        f"(female='{pair.female.block_name}', male='{pair.male.block_name}', "
        f"contact={pair.contact_distance_mm:.4f} mm)"
    )

    bar_b_id = pick_bar("Select second bar of the joint pair")
    if bar_b_id is None:
        return

    # Ensure both bars are registered with IDs and sequence numbers.
    bar_a_bid = ensure_bar_id(bar_a_id)
    bar_b_bid = ensure_bar_id(bar_b_id)

    # Auto-assign Le (female) / Ln (male) by assembly sequence.
    seq_map = get_bar_seq_map()
    seq_a = seq_map.get(bar_a_bid, (None, 9999))[1]
    seq_b = seq_map.get(bar_b_bid, (None, 9999))[1]
    if seq_a <= seq_b:
        le_id, le_bar_id = bar_a_id, bar_a_bid
        ln_id, ln_bar_id = bar_b_id, bar_b_bid
    else:
        le_id, le_bar_id = bar_b_id, bar_b_bid
        ln_id, ln_bar_id = bar_a_id, bar_a_bid

    le_seq = seq_map.get(le_bar_id, (None, "?"))[1]
    ln_seq = seq_map.get(ln_bar_id, (None, "?"))[1]
    print(
        f"RSJointPlace: {le_bar_id} (seq {le_seq}) \u2192 female,  "
        f"{ln_bar_id} (seq {ln_seq}) \u2192 male"
    )

    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)

    # Validate block definitions before solving.
    female_block_name = _require_block_definition(
        pair.female.block_name, asset_path=pair.female.asset_path()
    )
    male_block_name = _require_block_definition(
        pair.male.block_name, asset_path=pair.male.asset_path()
    )

    # Interactive click loop — variants are computed on demand as the user toggles.
    session = _JointSession(
        le_start,
        le_end,
        ln_start,
        ln_end,
        female_block_name,
        male_block_name,
        pair=pair,
    )
    # Apply auto-recovery on the very first variant too, so special joints
    # (with only two valid variants out of the nominal four) don't open
    # with a visibly broken interface.
    if not is_variant_acceptable(session.current_variant):
        origin_err, z_err = _interface_metrics_from_result(session.current_variant)
        print(
            f"RSJointPlace: initial variant interface error too large "
            f"(origin={origin_err:.4f} mm, z={np.degrees(z_err):.4f}deg); "
            f"auto-flipping female side."
        )
        session.le_rev = not session.le_rev
    chosen = _interactive_click_loop(session)
    if chosen is None:
        print("RSJointPlace: Cancelled.")
        return

    _, male_id, joint_id = _place_joint_blocks(
        chosen, le_id, ln_id, le_bar_id, ln_bar_id, pair=pair
    )
    from core.rhino_tool_place import auto_place_tool_at_male_joint
    auto_place_tool_at_male_joint(male_id, joint_id, pair)


if __name__ == "__main__":
    main()
