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
    as_object_id_list,
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


def _has_block_definition(name):
    for instance_def in sc.doc.InstanceDefinitions:
        if (
            instance_def is not None
            and not instance_def.IsDeleted
            and instance_def.Name == name
        ):
            return True
    return False


def _import_block_definition_from_3dm(block_name, asset_path):
    """Import a single block definition named *block_name* from *asset_path*
    into the current Rhino document.  Returns True on success, False otherwise.

    Strategy: read the .3dm file via :class:`Rhino.FileIO.File3dm`, collect
    every geometry/attribute pair found in it (the asset was exported as
    one block's worth of geometry by ``rs_define_joint_pair``), and add
    them as a new InstanceDefinition under *block_name*.  Falls back to
    Rhino's ``_-Insert`` command if the File3dm path fails.
    """
    asset_path = os.path.normpath(asset_path)
    if not os.path.isfile(asset_path):
        print(
            f"  [import_block] asset file not found: {asset_path}"
        )
        return False

    # ---- Attempt 1: RhinoCommon File3dm direct read --------------------
    try:
        file3dm = Rhino.FileIO.File3dm.Read(asset_path)
        if file3dm is None:
            print(f"  [import_block] File3dm.Read returned None for {asset_path}")
        else:
            geometries = []
            attributes = []
            for obj in file3dm.Objects:
                if obj is None or obj.Geometry is None:
                    continue
                geometries.append(obj.Geometry.Duplicate())
                attributes.append(obj.Attributes.Duplicate() if obj.Attributes else None)
            if geometries:
                idef_index = sc.doc.InstanceDefinitions.Add(
                    block_name,
                    f"Imported from {os.path.basename(asset_path)}",
                    Rhino.Geometry.Point3d.Origin,
                    geometries,
                    attributes,
                )
                if idef_index >= 0:
                    print(
                        f"  [import_block] imported '{block_name}' "
                        f"({len(geometries)} object(s)) from {asset_path}"
                    )
                    sc.doc.Views.Redraw()
                    return True
                print(f"  [import_block] InstanceDefinitions.Add returned {idef_index}")
            else:
                print(f"  [import_block] no geometry found in {asset_path}")
    except Exception as exc:
        print(f"  [import_block] File3dm path raised: {exc!r}")

    # ---- Attempt 2: _-Insert command ----------------------------------
    cmd_path = asset_path.replace("/", "\\")
    cmd = (
        '_-Insert _File _Yes "{path}" _Block _Enter '
        '0,0,0 _Enter 1 _Enter 0 _Enter'
    ).format(path=cmd_path)
    print(f"  [import_block] running: {cmd}")
    ok_cmd = rs.Command(cmd, echo=False)
    if ok_cmd and _has_block_definition(block_name):
        # The _-Insert command also placed an instance at the origin --
        # remove it; the caller will insert at the proper location.
        rs.UnselectAllObjects()
        return True
    return False


def _require_block_definition(name, *, asset_path=None):
    """Ensure block *name* is loaded.  If missing and *asset_path* is given,
    attempt to import it from that .3dm file."""
    if _has_block_definition(name):
        return name
    if asset_path:
        if _import_block_definition_from_3dm(name, asset_path):
            return name
    raise RuntimeError(
        f"Missing required Rhino block definition '{name}'."
        + (f"  (Tried to import from {asset_path}.)" if asset_path else "")
    )


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
        """Toggle the female joint orientation and refresh the preview."""
        self.le_rev = not self.le_rev
        self.show_current()

    def click_male(self):
        """Toggle the male joint orientation and refresh the preview."""
        self.ln_rev = not self.ln_rev
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
    return female_id, male_id


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
    chosen = _interactive_click_loop(session)
    if chosen is None:
        print("RSJointPlace: Cancelled.")
        return

    _place_joint_blocks(chosen, le_id, ln_id, le_bar_id, ln_bar_id, pair=pair)


if __name__ == "__main__":
    main()
