#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSBarSubfloor - Add a subfloor bar between two existing bars.

Like RSBarBrace, but lets the user choose two DIFFERENT joint pairs:
one for the joint with the first (Left) existing bar and one for the
joint with the second (Right) existing bar.  The first picked bar is
treated as the Left bar; the second as the Right bar.  This left/right
assignment is implicit (not stored in the model).

The pair selection is exposed as command-line options on the first
bar pick (similar to RSBarBrace's ``Pair=`` option):

    LeftJointPair=...  RightJointPair=...  SwapLeftRight  Length=...

Defaults are kept in document user-text
(``scaffolding.last_subfloor_left_pair`` /
``scaffolding.last_subfloor_right_pair``).

Geometry constraint: the underlying S2-T1 solver assumes a single
bar-to-bar contact distance for both ends of the subfloor bar.  Thus
both chosen pairs MUST have the same ``contact_distance_mm``.  If they
differ, the command aborts with a console message before running the
solver.
"""

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

from core import config
from core import geometry
from core.joint_pair import get_joint_pair
from core.rhino_bar_pick import (
    bar_or_tube_filter,
    get_default_brace_length,
    get_default_subfloor_left_pair_name,
    get_default_subfloor_right_pair_name,
    pick_bar,
    require_pair_names,
    resolve_picked_to_bar_curve,
    set_default_brace_length,
    set_default_subfloor_left_pair_name,
    set_default_subfloor_right_pair_name,
    get_default_pair_name,
)
from core.rhino_helpers import (
    add_centered_line,
    apply_object_display,
    as_object_id_list,
    curve_endpoints,
    delete_objects,
    ensure_layer,
    point_to_array,
    suspend_redraw,
)
from core.rhino_bar_registry import (
    ensure_bar_id,
    ensure_bar_preview,
    paint_bar,
    repair_on_entry,
    reset_bar_color,
)
from core.joint_auto_place import auto_place_joint_pair

SELECTED_BAR_COLOR = config.SELECTED_BAR_COLOR
_PREVIEW_COLORS = [
    (230, 80, 80),
    (80, 80, 230),
    (80, 200, 80),
    (200, 160, 50),
]

_BAR_CENTERLINE_LAYER = config.LAYER_BAR_CENTERLINES
_TUBE_LAYER = config.LAYER_BAR_TUBE_PREVIEWS

# Tolerance (mm) for considering two pairs' contact distances equal.
_CONTACT_DISTANCE_TOL_MM = 1e-3


# ---------------------------------------------------------------------------
# Helpers (cloned from rs_bar_brace; kept local so this command can evolve
# independently of the brace command)
# ---------------------------------------------------------------------------


def _place_axis_line(curve_id, *, color=None, label=None):
    if curve_id is None or not rs.IsObject(curve_id):
        return None
    ensure_layer(_BAR_CENTERLINE_LAYER)
    rs.ObjectLayer(curve_id, _BAR_CENTERLINE_LAYER)
    if color is not None:
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(curve_id, 1)
        rs.ObjectColor(curve_id, color)
    if label:
        rs.SetUserText(curve_id, "axis_label", label)
    return curve_id


def _bake_reference_point(point, label, color):
    point_xyz = point_to_array(point)
    pid = rs.AddPoint(point_xyz.tolist())
    if pid is None:
        return None
    apply_object_display(pid, label, color=color, layer_name=_BAR_CENTERLINE_LAYER)
    return pid


def _bake_axis_tube(axis_curve_id, label, color=None, layer_name=_TUBE_LAYER):
    if axis_curve_id is None or not rs.IsObject(axis_curve_id):
        return []
    start_xyz, end_xyz = curve_endpoints(axis_curve_id)
    axis_vector = end_xyz - start_xyz
    axis_length = float(np.linalg.norm(axis_vector))
    if axis_length <= 1e-9:
        return []
    ensure_layer(layer_name)
    axis_direction = axis_vector / axis_length
    base_plane = Rhino.Geometry.Plane(
        Rhino.Geometry.Point3d(*start_xyz.tolist()),
        Rhino.Geometry.Vector3d(*axis_direction.tolist()),
    )
    cylinder = Rhino.Geometry.Cylinder(
        Rhino.Geometry.Circle(base_plane, float(config.BAR_RADIUS)),
        axis_length,
    )
    brep = cylinder.ToBrep(True, True)
    if brep is None:
        return []
    tube_id = sc.doc.Objects.AddBrep(brep)
    if tube_id is None:
        return []
    baked_ids = apply_object_display(tube_id, label, color=color, layer_name=layer_name)
    for oid in baked_ids:
        rs.SetUserText(oid, "tube_axis_id", str(axis_curve_id))
        rs.SetUserText(oid, "tube_radius", f"{config.BAR_RADIUS:.6f}")
    return baked_ids


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)


# ---------------------------------------------------------------------------
# Solver + preview
# ---------------------------------------------------------------------------


def _solve(le1_id, le2_id, ce1, ce2, target_distance):
    le1_start, le1_end = curve_endpoints(le1_id)
    le2_start, le2_end = curve_endpoints(le2_id)
    ce1 = point_to_array(ce1)
    ce2 = point_to_array(ce2)
    n1 = le1_end - le1_start
    n2 = le2_end - le2_start
    target_distance = float(target_distance)
    try:
        report = geometry.solve_s2_t1_report(
            n1,
            ce1,
            n2,
            ce2,
            target_distance,
            nn_init_hint=ce2 - ce1,
        )
    except Exception as exc:
        print(f"RSBarSubfloor solver error: {exc}")
        return [], None
    return report.get("solutions", []), report


def _create_previews(solutions, ce1, ce2, brace_length):
    preview_items = []
    half_length = float(brace_length) / 2.0
    for index, sol in enumerate(solutions):
        midpoint = 0.5 * (sol["p1"] + sol["p2"])
        direction = sol["nn"] / np.linalg.norm(sol["nn"])
        pt_a = midpoint - half_length * direction
        pt_b = midpoint + half_length * direction
        color = _PREVIEW_COLORS[index % len(_PREVIEW_COLORS)]

        line_id = rs.AddLine(pt_a, pt_b)
        _place_axis_line(line_id, color=color, label=f"RSBarSubfloor_preview_{index + 1}")
        tube_ids = _bake_axis_tube(
            line_id, f"RSBarSubfloor_preview_{index + 1}_tube", color=color
        )

        dot_id = rs.AddTextDot(f"{index + 1}", midpoint)
        apply_object_display(
            dot_id,
            f"RSBarSubfloor_preview_{index + 1}_label",
            color=color,
            layer_name=_BAR_CENTERLINE_LAYER,
        )

        all_ids = [line_id, dot_id] + as_object_id_list(tube_ids)
        pick_set = set(as_object_id_list(all_ids))
        preview_items.append(
            {
                "index": index,
                "pick_ids": pick_set,
                "cleanup_ids": all_ids,
            }
        )
    return preview_items


def _cleanup_previews(preview_items):
    with suspend_redraw():
        for item in preview_items:
            delete_objects(item["cleanup_ids"])


def _print_report(report):
    if report is None:
        return
    solutions = report.get("solutions", [])
    print(f"RSBarSubfloor: {len(solutions)} solution(s) found")
    for i, sol in enumerate(solutions, 1):
        theta_1, theta_2 = sol["angles"]
        print(
            f"  {i}: family={sol.get('sign_family', '?')}, "
            f"angles=({theta_1:.3f},{theta_2:.3f}) rad, "
            f"residual={sol['residual']:.2e}"
        )


# ---------------------------------------------------------------------------
# Interactive loop
# ---------------------------------------------------------------------------


def _interactive_loop(le1_id, le2_id, ce1, ce2, target_distance, brace_length):
    ce1 = point_to_array(ce1)
    ce2 = point_to_array(ce2)
    ce1_ref_id = _bake_reference_point(ce1, "RSBarSubfloor_Ce1", (230, 80, 80))
    ce2_ref_id = _bake_reference_point(ce2, "RSBarSubfloor_Ce2", (80, 80, 230))
    ref_ids = as_object_id_list([ce1_ref_id, ce2_ref_id])

    solutions, report = _solve(le1_id, le2_id, ce1, ce2, target_distance)
    _print_report(report)
    if not solutions:
        rs.MessageBox(
            "No valid subfloor solutions found. Try different contact points."
        )
        delete_objects(ref_ids)
        return None, brace_length

    preview_items = _create_previews(solutions, ce1, ce2, brace_length)

    length_value = Rhino.Input.Custom.OptionDouble(
        float(brace_length), 1.0, 100000.0
    )

    try:
        while True:
            go = Rhino.Input.Custom.GetObject()
            go.SetCommandPrompt(
                f"Click one of the {len(solutions)} candidate bars (numbered 1-{len(solutions)})"
            )
            go.GeometryFilter = (
                Rhino.DocObjects.ObjectType.Curve
                | Rhino.DocObjects.ObjectType.Surface
                | Rhino.DocObjects.ObjectType.Brep
                | Rhino.DocObjects.ObjectType.Annotation
                | Rhino.DocObjects.ObjectType.Point
            )
            go.DisablePreSelect()
            go.AcceptNothing(False)

            slide1_idx = go.AddOption("SlidePointOnLeft")
            slide2_idx = go.AddOption("SlidePointOnRight")
            go.AddOptionDouble("Length", length_value)

            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                return None, brace_length

            if result == Rhino.Input.GetResult.Object:
                obj_ref = go.Object(0)
                picked_id = obj_ref.ObjectId
                chosen_index = None
                for item in preview_items:
                    if picked_id in item["pick_ids"]:
                        chosen_index = item["index"]
                        break
                if chosen_index is not None:
                    sol = solutions[chosen_index]
                    print(
                        f"RSBarSubfloor: Selected solution {chosen_index + 1}, "
                        f"family={sol.get('sign_family', '?')}, "
                        f"residual={sol['residual']:.2e}"
                    )
                    return sol, brace_length
                continue

            if result == Rhino.Input.GetResult.Option:
                new_length = float(length_value.CurrentValue)
                if abs(new_length - float(brace_length)) > 1e-9:
                    brace_length = new_length
                    set_default_brace_length(brace_length)
                    print(
                        f"RSBarSubfloor: bar length updated to "
                        f"{brace_length:.2f} mm; regenerating previews."
                    )
                    _cleanup_previews(preview_items)
                    preview_items = _create_previews(
                        solutions, ce1, ce2, brace_length
                    )
                    continue

                opt = go.Option()
                opt_idx = opt.Index if opt is not None else -1

                if opt_idx == slide1_idx or opt_idx == slide2_idx:
                    if opt_idx == slide1_idx:
                        new_pt = rs.GetPointOnCurve(
                            le1_id, "Pick new contact point on Left bar"
                        )
                    else:
                        new_pt = rs.GetPointOnCurve(
                            le2_id, "Pick new contact point on Right bar"
                        )
                    if new_pt is None:
                        continue
                    if opt_idx == slide1_idx:
                        ce1 = point_to_array(new_pt)
                    else:
                        ce2 = point_to_array(new_pt)

                    _cleanup_previews(preview_items)
                    delete_objects(ref_ids)
                    ce1_ref_id = _bake_reference_point(
                        ce1, "RSBarSubfloor_Ce1", (230, 80, 80)
                    )
                    ce2_ref_id = _bake_reference_point(
                        ce2, "RSBarSubfloor_Ce2", (80, 80, 230)
                    )
                    ref_ids = as_object_id_list([ce1_ref_id, ce2_ref_id])

                    solutions, report = _solve(
                        le1_id, le2_id, ce1, ce2, target_distance
                    )
                    _print_report(report)
                    if not solutions:
                        rs.MessageBox(
                            "No valid subfloor solutions found for the updated points."
                        )
                        delete_objects(ref_ids)
                        return None, brace_length
                    preview_items = _create_previews(
                        solutions, ce1, ce2, brace_length
                    )
                    continue

    finally:
        _cleanup_previews(preview_items)
        delete_objects(ref_ids)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _pick_le1_with_dual_pair_and_length(
    bar_prompt: str,
    command_name: str = "RSBarSubfloor",
    *,
    length_option_name: str = "Length",
    length_min: float = 1.0,
    length_max: float = 100000.0,
):
    """Pick the first existing bar (Le1) while exposing two independent
    ``LeftJointPair`` / ``RightJointPair`` list options, a ``SwapLeftRight``
    toggle, and a numeric ``Length`` option.  RSBarSubfloor-specific.

    Returns ``(bar_curve_id, left_pair, right_pair, length_mm)`` or
    ``(None, None, None, None)`` on cancel / empty registry.
    """
    names = require_pair_names(command_name)
    if names is None:
        return None, None, None, None

    fallback = get_default_pair_name() or names[0]
    if fallback not in names:
        fallback = names[0]
    left_default = get_default_subfloor_left_pair_name() or fallback
    if left_default not in names:
        left_default = fallback
    right_default = get_default_subfloor_right_pair_name() or fallback
    if right_default not in names:
        right_default = fallback

    left_index = names.index(left_default)
    right_index = names.index(right_default)

    length_value = Rhino.Input.Custom.OptionDouble(
        get_default_brace_length(), length_min, length_max
    )

    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(bar_prompt)
        go.EnablePreSelect(True, True)
        go.SetCustomGeometryFilter(bar_or_tube_filter)

        if len(names) == 1:
            left_opt_index = right_opt_index = swap_opt_index = -1
        else:
            left_opt_index = go.AddOptionList("LeftJointPair", names, left_index)
            right_opt_index = go.AddOptionList("RightJointPair", names, right_index)
            swap_opt_index = go.AddOption("SwapLeftRight")
        go.AddOptionDouble(length_option_name, length_value)

        result = go.Get()
        if result == Rhino.Input.GetResult.Cancel:
            return None, None, None, None

        if result == Rhino.Input.GetResult.Option:
            opt = go.Option()
            if opt is not None:
                if opt.Index == left_opt_index:
                    left_index = int(opt.CurrentListOptionIndex)
                elif opt.Index == right_opt_index:
                    right_index = int(opt.CurrentListOptionIndex)
                elif opt.Index == swap_opt_index:
                    left_index, right_index = right_index, left_index
                    print(
                        f"  swapped: LeftJointPair='{names[left_index]}', "
                        f"RightJointPair='{names[right_index]}'"
                    )
            continue

        if result == Rhino.Input.GetResult.Object:
            picked_id = go.Object(0).ObjectId
            rs.UnselectObject(picked_id)
            bar_id = resolve_picked_to_bar_curve(picked_id)
            if bar_id is None:
                continue
            left_name = names[left_index]
            right_name = names[right_index]
            set_default_subfloor_left_pair_name(left_name)
            set_default_subfloor_right_pair_name(right_name)
            chosen_length = float(length_value.CurrentValue)
            set_default_brace_length(chosen_length)
            return (
                bar_id,
                get_joint_pair(left_name),
                get_joint_pair(right_name),
                chosen_length,
            )

        return None, None, None, None


def main():
    _refresh_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSBarSubfloor")
    rs.UnselectAllObjects()

    le1_id, left_pair, right_pair, brace_length = (
        _pick_le1_with_dual_pair_and_length(
            "Select first existing bar (becomes LEFT bar)",
            command_name="RSBarSubfloor",
        )
    )
    if le1_id is None or left_pair is None or right_pair is None:
        return

    # Sanity check: the S2-T1 solver assumes a single bar-to-bar contact
    # distance for both ends.  Reject mismatched pairs here so the user
    # gets a clear message instead of a meaningless solver failure.
    left_d = float(left_pair.contact_distance_mm)
    right_d = float(right_pair.contact_distance_mm)
    if abs(left_d - right_d) > _CONTACT_DISTANCE_TOL_MM:
        msg = (
            f"LeftJointPair '{left_pair.name}' and RightJointPair "
            f"'{right_pair.name}' have different bar-to-bar contact "
            f"distances ({left_d:.4f} mm vs {right_d:.4f} mm).  "
            f"The subfloor solver requires both ends to share a single "
            f"distance.  Either pick two pairs with matching "
            f"contact_distance_mm, or redefine one pair so they agree."
        )
        print(f"RSBarSubfloor: {msg}")
        rs.MessageBox(msg, 0, "RSBarSubfloor")
        return

    target_distance = left_d
    print(
        f"RSBarSubfloor: LEFT pair '{left_pair.name}', "
        f"RIGHT pair '{right_pair.name}' "
        f"(contact distance {target_distance:.4f} mm), "
        f"bar length {float(brace_length):.2f} mm"
    )

    le2_id = None
    line_id = None
    try:
        ensure_bar_id(le1_id)
        ensure_bar_preview(le1_id, float(config.BAR_RADIUS))
        paint_bar(le1_id, SELECTED_BAR_COLOR)

        le2_id = pick_bar("Select second existing bar (becomes RIGHT bar)")
        if le2_id is None:
            return
        ensure_bar_id(le2_id)
        ensure_bar_preview(le2_id, float(config.BAR_RADIUS))
        paint_bar(le2_id, SELECTED_BAR_COLOR)

        ce1 = rs.GetPointOnCurve(le1_id, "Pick contact point on LEFT bar")
        if ce1 is None:
            return
        ce2 = rs.GetPointOnCurve(le2_id, "Pick contact point on RIGHT bar")
        if ce2 is None:
            return

        solution, brace_length = _interactive_loop(
            le1_id, le2_id, ce1, ce2, target_distance, brace_length
        )
        if solution is None:
            print("RSBarSubfloor: Cancelled.")
            return
        print(f"RSBarSubfloor: final bar length {float(brace_length):.2f} mm")

        with suspend_redraw():
            midpoint = 0.5 * (solution["p1"] + solution["p2"])
            line_id = add_centered_line(midpoint, solution["nn"], brace_length)
            _place_axis_line(line_id, label="RSBarSubfloor_Ln")
            ensure_bar_id(line_id)
            ensure_bar_preview(line_id, float(config.BAR_RADIUS))

        theta_1, theta_2 = solution["angles"]
        print(
            f"RSBarSubfloor: Solution placed. "
            f"Family={solution.get('sign_family', '?')}, "
            f"Angles=({theta_1:.3f},{theta_2:.3f}) rad, "
            f"Residual: {solution['residual']:.2e}"
        )

        # Place LEFT pair on the (le1, new_bar) side, RIGHT pair on
        # the (le2, new_bar) side.  Each existing bar is the female,
        # the new subfloor bar is the male.
        auto_place_joint_pair(le1_id, line_id, left_pair)
        auto_place_joint_pair(le2_id, line_id, right_pair)

    finally:
        reset_bar_color(le1_id)
        reset_bar_color(le2_id)
        reset_bar_color(line_id)
        sc.doc.Views.Redraw()


if __name__ == "__main__":
    main()
