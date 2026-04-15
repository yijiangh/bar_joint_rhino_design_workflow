"""Rhino entry point for T1 bar-axis generation."""

import contextlib
import importlib
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import geometry


_S2_PREVIEW_COLORS = [
    (230, 80, 80),
    (80, 80, 230),
    (80, 200, 80),
    (200, 160, 50),
]

_REFERENCE_SEGMENT_PRINT_WIDTH = 0.8
_BAR_AXIS_LAYER = "Bar Axis Lines"
_CONTACT_SEGMENT_LAYER = "Contact Segments"
_TUBE_LAYER = "Tube preview"
_S1_EXISTING_BAR_COLOR = (120, 120, 120)
_DEFAULT_TUBE_COLOR = (205, 150, 60)


def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def _bake_reference_point(point, label, color):
    point_xyz = _point_to_array(point)
    point_id = rs.AddPoint(point_xyz.tolist())
    if point_id is None:
        return None
    _apply_object_display(point_id, label, color=color, layer_name=_CONTACT_SEGMENT_LAYER)
    return point_id


def _bake_reference_segment(start_point, end_point, label, color):
    start_xyz = _point_to_array(start_point)
    end_xyz = _point_to_array(end_point)
    line_id = rs.AddLine(start_xyz.tolist(), end_xyz.tolist())
    if line_id is None:
        return None
    _apply_object_display(line_id, label, color=color, layer_name=_CONTACT_SEGMENT_LAYER)
    if hasattr(rs, "ObjectPrintWidthSource"):
        rs.ObjectPrintWidthSource(line_id, 1)
    if hasattr(rs, "ObjectPrintWidth"):
        rs.ObjectPrintWidth(line_id, _REFERENCE_SEGMENT_PRINT_WIDTH)
    return line_id


def _as_object_id_list(object_ids):
    if object_ids is None:
        return []
    if isinstance(object_ids, (str, bytes)):
        return [object_ids]
    try:
        return [object_id for object_id in object_ids if object_id is not None]
    except TypeError:
        return [object_ids]


def _ensure_layer(layer_name):
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
    return layer_name


def _place_axis_line(curve_id, *, color=None, label=None):
    if curve_id is None or not rs.IsObject(curve_id):
        return None
    _ensure_layer(_BAR_AXIS_LAYER)
    rs.ObjectLayer(curve_id, _BAR_AXIS_LAYER)
    if color is not None:
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(curve_id, 1)
        rs.ObjectColor(curve_id, color)
    if label:
        rs.SetUserText(curve_id, "axis_label", label)
    return curve_id


def _apply_object_display(object_ids, label, color=None, layer_name=None):
    baked_ids = _as_object_id_list(object_ids)
    multiple_objects = len(baked_ids) > 1
    for index, object_id in enumerate(baked_ids):
        if layer_name is not None:
            _ensure_layer(layer_name)
            rs.ObjectLayer(object_id, layer_name)
        if color is not None:
            if hasattr(rs, "ObjectColorSource"):
                rs.ObjectColorSource(object_id, 1)
            rs.ObjectColor(object_id, color)
        object_label = f"{label}_{index + 1}" if multiple_objects else label
        rs.ObjectName(object_id, object_label)
        rs.SetUserText(object_id, "reference_label", label)
    return baked_ids


def _delete_objects(object_ids):
    for object_id in _as_object_id_list(object_ids):
        if rs.IsObject(object_id):
            rs.DeleteObject(object_id)


@contextlib.contextmanager
def _suspend_redraw():
    previous_state = None
    redraw_supported = hasattr(rs, "EnableRedraw")
    try:
        if redraw_supported:
            previous_state = rs.EnableRedraw(False)
        yield
    finally:
        redraw_is_enabled = True
        if redraw_supported:
            redraw_is_enabled = True if previous_state is None else bool(previous_state)
            rs.EnableRedraw(redraw_is_enabled)
        if redraw_is_enabled and hasattr(rs, "Redraw"):
            rs.Redraw()


def _bake_axis_tube(axis_curve_id, label, color=None, layer_name=_TUBE_LAYER):
    if axis_curve_id is None or not rs.IsObject(axis_curve_id):
        return []

    start_xyz, end_xyz = _curve_endpoints(axis_curve_id)
    axis_vector = end_xyz - start_xyz
    axis_length = float(np.linalg.norm(axis_vector))
    if axis_length <= 1e-9:
        return []

    _ensure_layer(layer_name)

    baked_ids = []
    try:
        import Rhino
    except ImportError:
        curve_domain = rs.CurveDomain(axis_curve_id)
        if curve_domain is None:
            return []
        tube_ids = rs.AddPipe(
            axis_curve_id,
            [float(curve_domain[0]), float(curve_domain[1])],
            [float(config.BAR_RADIUS), float(config.BAR_RADIUS)],
            0,
            1,
            False,
        )
        baked_ids = _apply_object_display(tube_ids, label, color=color, layer_name=layer_name)
    else:
        use_endpoint_cylinder = True
        if hasattr(rs, "IsLine"):
            use_endpoint_cylinder = bool(rs.IsLine(axis_curve_id))

        if use_endpoint_cylinder:
            axis_direction = axis_vector / axis_length
            base_plane = Rhino.Geometry.Plane(
                Rhino.Geometry.Point3d(*start_xyz.tolist()),
                Rhino.Geometry.Vector3d(*axis_direction.tolist()),
            )
            cylinder = Rhino.Geometry.Cylinder(Rhino.Geometry.Circle(base_plane, float(config.BAR_RADIUS)), axis_length)
            brep = cylinder.ToBrep(True, True)
            if brep is None:
                return []
            tube_id = sc.doc.Objects.AddBrep(brep)
            if tube_id is None:
                return []
            baked_ids = _apply_object_display(tube_id, label, color=color, layer_name=layer_name)
        else:
            curve_domain = rs.CurveDomain(axis_curve_id)
            if curve_domain is None:
                return []
            tube_ids = rs.AddPipe(
                axis_curve_id,
                [float(curve_domain[0]), float(curve_domain[1])],
                [float(config.BAR_RADIUS), float(config.BAR_RADIUS)],
                0,
                1,
                False,
            )
            baked_ids = _apply_object_display(tube_ids, label, color=color, layer_name=layer_name)
    for object_id in baked_ids:
        rs.SetUserText(object_id, "tube_axis_id", str(axis_curve_id))
        rs.SetUserText(object_id, "tube_radius", f"{config.BAR_RADIUS:.6f}")
    return baked_ids


def _candidate_pick_filter():
    filter_namespace = getattr(rs, "filter", None)
    if filter_namespace is None:
        return 0

    pick_filter = 0
    for filter_name in ("curve", "surface", "polysurface", "annotation", "point"):
        pick_filter |= getattr(filter_namespace, filter_name, 0)
    return pick_filter


def _curve_endpoints(curve_id):
    start = _point_to_array(rs.CurveStartPoint(curve_id))
    end = _point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


def _add_centered_line(midpoint: np.ndarray, direction: np.ndarray):
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    half_length = config.DEFAULT_NEW_BAR_LENGTH / 2.0
    return rs.AddLine(midpoint - half_length * direction, midpoint + half_length * direction)


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)


def _format_s2_t1_diagnostics_line(theta_1, theta_2, objective_value, diagnostics):
    return (
        f"angles=({theta_1:.3f},{theta_2:.3f}) rad, "
        f"total_obj={objective_value:.2e} | "
        f"C1 obj={diagnostics.get('contact_1_objective_component', float('nan')):.2e}, "
        f"offset={diagnostics.get('contact_1_axial_offset', float('nan')):.4f} mm, "
        f"angle={diagnostics.get('contact_1_to_bar_angle_deg', float('nan')):.4f} deg, "
        f"orth_err={diagnostics.get('contact_1_orthogonality_error_deg', float('nan')):.4f} deg | "
        f"C2 obj={diagnostics.get('contact_2_objective_component', float('nan')):.2e}, "
        f"offset={diagnostics.get('contact_2_axial_offset', float('nan')):.4f} mm, "
        f"angle={diagnostics.get('contact_2_to_bar_angle_deg', float('nan')):.4f} deg, "
        f"orth_err={diagnostics.get('contact_2_orthogonality_error_deg', float('nan')):.4f} deg"
    )


def _format_optimizer_setting(setting_value):
    if setting_value is None:
        return "disabled"
    return f"{float(setting_value):.1e}"


def _format_optimizer_result(optimizer_result, optimizer_settings):
    if not optimizer_result:
        return "term=unknown"
    return (
        f"term={optimizer_result.get('termination_reason', 'unknown')}, "
        f"status={optimizer_result.get('status', '?')}, "
        f"success={optimizer_result.get('success', False)}, "
        f"nfev={optimizer_result.get('nfev', '?')}/{optimizer_settings.get('max_nfev', '?')}, "
        f"message={optimizer_result.get('message', '')}"
    )


def _print_s2_t1_report(report):
    theta_1, theta_2 = report["initial_angles"]
    found_families = ", ".join(report["found_sign_families"]) if report["found_sign_families"] else "none"
    optimizer_settings = report.get("optimizer_settings", {})
    print(
        f"S2-T1 solve report: {report['solution_count']} solution(s), "
        f"initial angles=({theta_1:.3f},{theta_2:.3f}) rad, "
        f"found sign families: {found_families}"
    )
    print(
        f"S2-T1 optimizer settings: "
        f"xtol={_format_optimizer_setting(optimizer_settings.get('xtol'))}, "
        f"ftol={_format_optimizer_setting(optimizer_settings.get('ftol'))}, "
        f"gtol={_format_optimizer_setting(optimizer_settings.get('gtol'))}, "
        f"max_nfev={optimizer_settings.get('max_nfev', '?')}"
    )
    for index, solution in enumerate(report["solutions"], start=1):
        diagnostics = solution.get("optimization_diagnostics", {})
        theta_1, theta_2 = solution["angles"]
        print(
            f"  Sol {index} family={solution.get('sign_family', '?')} "
            f"{_format_s2_t1_diagnostics_line(theta_1, theta_2, solution['residual'], diagnostics)} | "
            f"{_format_optimizer_result(solution.get('optimizer_result'), optimizer_settings)}"
        )
    if report["missing_sign_families"]:
        print(
            f"S2-T1 solve report: Missing sign families under current constraints: "
            f"{', '.join(report['missing_sign_families'])}"
        )
        for attempt in report.get("missing_family_attempts", []):
            diagnostics = attempt.get("optimization_diagnostics") or {}
            theta_1, theta_2 = attempt["final_angles"]
            converged_family = attempt.get("converged_sign_family") or "degenerate"
            print(
                f"  Missing family {attempt.get('target_sign_family', '?')} "
                f"best attempt converged to {converged_family} with "
                f"{_format_s2_t1_diagnostics_line(theta_1, theta_2, attempt['objective'], diagnostics)} | "
                f"{_format_optimizer_result(attempt, optimizer_settings)}"
            )
        if report["solution_count"] < 4:
            print(
                f"S2-T1 solve report: Only {report['solution_count']} unique real solution(s) "
                f"were found for this selection."
            )


def get_inputs():
    rs.UnselectAllObjects()
    mode = rs.ListBox(
        ["S1: Join to ONE bar", "S2: Join to TWO bars"],
        "Select connection mode:",
        "T1 - Bar Axis Generation",
    )
    if mode is None:
        return None

    if "ONE" in mode:
        le_id = rs.GetObject("Select existing bar (Le)", rs.filter.curve)
        if le_id is None:
            return None
        ln_id = rs.GetObject("Select new bar (Ln) - will be repositioned", rs.filter.curve)
        if ln_id is None:
            return None
        return {"mode": "S1", "le_id": le_id, "ln_id": ln_id}

    le1_id = rs.GetObject("Select first existing bar (Le1)", rs.filter.curve)
    if le1_id is None:
        return None
    le2_id = rs.GetObject("Select second existing bar (Le2)", rs.filter.curve)
    if le2_id is None:
        return None
    ce1 = rs.GetPointOnCurve(le1_id, "Pick contact point on Le1 (Ce1)")
    if ce1 is None:
        return None
    ce2 = rs.GetPointOnCurve(le2_id, "Pick contact point on Le2 (Ce2)")
    if ce2 is None:
        return None
    ce1_ref_id = _bake_reference_point(ce1, "T1_S2_Ce1", (230, 80, 80))
    ce2_ref_id = _bake_reference_point(ce2, "T1_S2_Ce2", (80, 80, 230))
    return {
        "mode": "S2",
        "le1_id": le1_id,
        "le2_id": le2_id,
        "ce1": ce1,
        "ce2": ce2,
        "ce1_ref_id": ce1_ref_id,
        "ce2_ref_id": ce2_ref_id,
    }


def run_s1_t1(inputs):
    le_start, le_end = _curve_endpoints(inputs["le_id"])
    ln_start, ln_end = _curve_endpoints(inputs["ln_id"])
    le_direction = le_end - le_start
    ln_direction = ln_end - ln_start
    target_distance = float(config.BAR_CONTACT_DISTANCE)

    t_e, t_n = geometry.closest_params_infinite_lines(le_start, le_direction, ln_start, ln_direction)
    p_e = le_start + t_e * le_direction
    p_n = ln_start + t_n * ln_direction

    segment = p_e - p_n
    current_distance = np.linalg.norm(segment)
    if current_distance <= 1e-9:
        rs.MessageBox("Error: the selected bars are coincident, so no unique contact normal exists.")
        return None

    translation = (segment / current_distance) * (current_distance - target_distance)
    shortest_segment_start = p_n + translation
    shortest_segment_end = p_e
    # Preserve the selected bar's original length and object attributes by
    # translating it in place instead of recreating a default-length line.
    line_id = rs.MoveObject(inputs["ln_id"], translation.tolist())
    if line_id is None:
        rs.MessageBox("Error: failed to move the selected new bar.")
        return None

    _bake_reference_segment(
        shortest_segment_start,
        shortest_segment_end,
        "T1_S1_shortest_segment",
        (180, 0, 180),
    )
    _place_axis_line(line_id, label="T1_S1_Ln")
    _bake_axis_tube(inputs["le_id"], "T1_S1_Le_tube", color=_S1_EXISTING_BAR_COLOR)
    _bake_axis_tube(line_id, "T1_S1_bar_tube", color=_DEFAULT_TUBE_COLOR)
    inputs["ln_id"] = line_id
    rs.SelectObject(line_id)
    print(f"S1-T1: Bar placed at distance {target_distance:.2f} from Le")
    return line_id


def run_s2_t1(inputs):
    le1_start, le1_end = _curve_endpoints(inputs["le1_id"])
    le2_start, le2_end = _curve_endpoints(inputs["le2_id"])
    ce1 = _point_to_array(inputs["ce1"])
    ce2 = _point_to_array(inputs["ce2"])
    target_distance = float(config.BAR_CONTACT_DISTANCE)

    n1 = le1_end - le1_start
    n2 = le2_end - le2_start
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
        rs.MessageBox(f"Error while solving S2-T1: {exc}")
        return None

    _print_s2_t1_report(report)
    solutions = report["solutions"]
    if not solutions:
        rs.MessageBox("No valid S2-T1 solution found. See the Rhino command line for diagnostics.")
        return None

    if len(solutions) == 1:
        with _suspend_redraw():
            solution = solutions[0]
            midpoint = 0.5 * (solution["p1"] + solution["p2"])
            line_id = _add_centered_line(midpoint, solution["nn"])
            _place_axis_line(line_id, color=_S2_PREVIEW_COLORS[0], label="T1_S2_Ln")
            _bake_reference_segment(solution["p1"], ce1, "T1_S2_shortest_segment_1", (230, 80, 80))
            _bake_reference_segment(solution["p2"], ce2, "T1_S2_shortest_segment_2", (80, 80, 230))
            _bake_axis_tube(inputs["le1_id"], "T1_S2_Le1_tube", color=_S2_PREVIEW_COLORS[0])
            _bake_axis_tube(inputs["le2_id"], "T1_S2_Le2_tube", color=_S2_PREVIEW_COLORS[1])
            _bake_axis_tube(line_id, "T1_S2_bar_tube", color=_S2_PREVIEW_COLORS[0])
        theta_1, theta_2 = solution["angles"]
        rs.SelectObject(line_id)
        print(
            f"S2-T1: Single solution, bar placed. "
            f"Family={solution.get('sign_family', '?')}, "
            f"Angles=({theta_1:.3f},{theta_2:.3f}) rad, "
            f"Residual: {solution['residual']:.2e}"
        )
        return line_id

    half_length = config.DEFAULT_NEW_BAR_LENGTH / 2.0
    preview_items = []
    with _suspend_redraw():
        for index, solution in enumerate(solutions):
            midpoint = 0.5 * (solution["p1"] + solution["p2"])
            direction = solution["nn"] / np.linalg.norm(solution["nn"])
            pt_a = midpoint - half_length * direction
            pt_b = midpoint + half_length * direction

            line_id = rs.AddLine(pt_a, pt_b)
            color = _S2_PREVIEW_COLORS[index % len(_S2_PREVIEW_COLORS)]
            _place_axis_line(line_id, color=color, label=f"T1_S2_preview_{index + 1}_Ln")
            tube_ids = _bake_axis_tube(
                line_id,
                f"T1_S2_preview_{index + 1}_bar_tube",
                color=color,
            )
            segment_1_id = _bake_reference_segment(
                solution["p1"],
                ce1,
                f"T1_S2_preview_{index + 1}_shortest_segment_1",
                color,
            )
            segment_2_id = _bake_reference_segment(
                solution["p2"],
                ce2,
                f"T1_S2_preview_{index + 1}_shortest_segment_2",
                color,
            )

            dot_id = rs.AddTextDot(f"{index + 1}", midpoint)
            _apply_object_display(dot_id, f"T1_S2_preview_{index + 1}_label", color=color, layer_name=_BAR_AXIS_LAYER)
            preview_items.append(
                {
                    "index": index,
                    "pick_ids": set(
                        _as_object_id_list([line_id, dot_id, segment_1_id, segment_2_id]) + _as_object_id_list(tube_ids)
                    ),
                    "cleanup_ids": [line_id, dot_id, segment_1_id, segment_2_id] + _as_object_id_list(tube_ids),
                }
            )

    picked_id = rs.GetObject(
        f"Click one of the {len(solutions)} candidate bars or tubes (numbered 1-{len(solutions)}):",
        _candidate_pick_filter(),
    )

    chosen_index = None
    with _suspend_redraw():
        for preview_item in preview_items:
            if picked_id in preview_item["pick_ids"]:
                chosen_index = preview_item["index"]
            _delete_objects(preview_item["cleanup_ids"])

    if chosen_index is None:
        print("S2-T1: No solution selected.")
        return None

    with _suspend_redraw():
        solution = solutions[chosen_index]
        midpoint = 0.5 * (solution["p1"] + solution["p2"])
        line_id = _add_centered_line(midpoint, solution["nn"])
        chosen_color = _S2_PREVIEW_COLORS[chosen_index % len(_S2_PREVIEW_COLORS)]
        _place_axis_line(line_id, color=chosen_color, label="T1_S2_Ln")
        _bake_axis_tube(inputs["le1_id"], "T1_S2_Le1_tube", color=_S2_PREVIEW_COLORS[0])
        _bake_axis_tube(inputs["le2_id"], "T1_S2_Le2_tube", color=_S2_PREVIEW_COLORS[1])
        _bake_axis_tube(line_id, "T1_S2_bar_tube", color=chosen_color)
        _bake_reference_segment(solution["p1"], ce1, "T1_S2_shortest_segment_1", (230, 80, 80))
        _bake_reference_segment(solution["p2"], ce2, "T1_S2_shortest_segment_2", (80, 80, 230))
    rs.SelectObject(line_id)
    theta_1, theta_2 = solution["angles"]
    print(
        f"S2-T1: Solution {chosen_index + 1} selected. "
        f"Family={solution.get('sign_family', '?')}, "
        f"Angles=({theta_1:.3f},{theta_2:.3f}) rad, "
        f"Residual: {solution['residual']:.2e}"
    )
    return line_id


def main(rerun=False):
    _refresh_runtime_modules()
    if rerun and "t1_inputs" in sc.sticky:
        inputs = sc.sticky["t1_inputs"]
    else:
        inputs = get_inputs()
        if inputs is None:
            return
        sc.sticky["t1_inputs"] = inputs

    if inputs["mode"] == "S1":
        run_s1_t1(inputs)
    elif inputs["mode"] == "S2":
        run_s2_t1(inputs)


if __name__ == "__main__":
    main(rerun=False)
