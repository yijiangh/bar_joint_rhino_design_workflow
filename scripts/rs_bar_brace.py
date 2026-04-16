#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSBarBrace - Add a brace bar between two existing bars.

Pick two existing bars and two contact points. The solver finds up to 4
candidate brace positions. An interactive command-prompt loop lets you:
  - click a colored candidate bar/tube in the viewport to select it
  - click SlidePointOn1 / SlidePointOn2 to re-pick contact points and re-solve
  - Escape to cancel
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

from core import config
from core import geometry
from core.rhino_helpers import (
    apply_object_display,
    as_object_id_list,
    curve_endpoints,
    delete_objects,
    ensure_layer,
    point_to_array,
    suspend_redraw,
)
from core.rhino_bar_registry import ensure_bar_id, ensure_bar_preview


_PREVIEW_COLORS = [
    (230, 80, 80),
    (80, 80, 230),
    (80, 200, 80),
    (200, 160, 50),
]

_REFERENCE_SEGMENT_PRINT_WIDTH = 0.8
_BAR_AXIS_LAYER = "Bar Axis Lines"
_CONTACT_SEGMENT_LAYER = "Contact Segments"
_TUBE_LAYER = "Tube preview"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Generic helpers (point_to_array, curve_endpoints, as_object_id_list,
# ensure_layer, apply_object_display, delete_objects, suspend_redraw) are
# now imported from core.rhino_helpers.


def _place_axis_line(curve_id, *, color=None, label=None):
    if curve_id is None or not rs.IsObject(curve_id):
        return None
    ensure_layer(_BAR_AXIS_LAYER)
    rs.ObjectLayer(curve_id, _BAR_AXIS_LAYER)
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
    apply_object_display(pid, label, color=color, layer_name=_CONTACT_SEGMENT_LAYER)
    return pid


def _bake_reference_segment(start_point, end_point, label, color):
    start_xyz = point_to_array(start_point)
    end_xyz = point_to_array(end_point)
    line_id = rs.AddLine(start_xyz.tolist(), end_xyz.tolist())
    if line_id is None:
        return None
    apply_object_display(line_id, label, color=color, layer_name=_CONTACT_SEGMENT_LAYER)
    if hasattr(rs, "ObjectPrintWidthSource"):
        rs.ObjectPrintWidthSource(line_id, 1)
    if hasattr(rs, "ObjectPrintWidth"):
        rs.ObjectPrintWidth(line_id, _REFERENCE_SEGMENT_PRINT_WIDTH)
    return line_id


def _bake_axis_tube(axis_curve_id, label, color=None, layer_name=_TUBE_LAYER):
    """Create a tube preview — used only for temporary interactive previews."""
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
        Rhino.Geometry.Circle(base_plane, float(config.BAR_RADIUS)), axis_length,
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


def _add_centered_line(midpoint, direction):
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    half_length = config.DEFAULT_NEW_BAR_LENGTH / 2.0
    return rs.AddLine(midpoint - half_length * direction, midpoint + half_length * direction)


def _candidate_pick_filter():
    filter_namespace = getattr(rs, "filter", None)
    if filter_namespace is None:
        return 0
    pick_filter = 0
    for name in ("curve", "surface", "polysurface", "annotation", "point"):
        pick_filter |= getattr(filter_namespace, name, 0)
    return pick_filter


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)


# ---------------------------------------------------------------------------
# Solve + preview helpers
# ---------------------------------------------------------------------------

def _solve(le1_id, le2_id, ce1, ce2):
    """Run the S2-T1 solver. Returns (solutions, report) or ([], None) on error."""
    le1_start, le1_end = curve_endpoints(le1_id)
    le2_start, le2_end = curve_endpoints(le2_id)
    ce1 = point_to_array(ce1)
    ce2 = point_to_array(ce2)
    n1 = le1_end - le1_start
    n2 = le2_end - le2_start
    target_distance = float(config.BAR_CONTACT_DISTANCE)
    try:
        report = geometry.solve_s2_t1_report(
            n1, ce1, n2, ce2, target_distance,
            nn_init_hint=ce2 - ce1,
        )
    except Exception as exc:
        print(f"RSBarBrace solver error: {exc}")
        return [], None
    return report.get("solutions", []), report


def _create_previews(solutions, ce1, ce2):
    """Create preview tubes + dots for all solutions. Returns list of preview dicts."""
    preview_items = []
    half_length = config.DEFAULT_NEW_BAR_LENGTH / 2.0
    for index, sol in enumerate(solutions):
        midpoint = 0.5 * (sol["p1"] + sol["p2"])
        direction = sol["nn"] / np.linalg.norm(sol["nn"])
        pt_a = midpoint - half_length * direction
        pt_b = midpoint + half_length * direction
        color = _PREVIEW_COLORS[index % len(_PREVIEW_COLORS)]

        line_id = rs.AddLine(pt_a, pt_b)
        _place_axis_line(line_id, color=color, label=f"RSBarBrace_preview_{index + 1}")
        tube_ids = _bake_axis_tube(line_id, f"RSBarBrace_preview_{index + 1}_tube", color=color)
        seg1_id = _bake_reference_segment(sol["p1"], ce1, f"RSBarBrace_preview_{index + 1}_seg1", color)
        seg2_id = _bake_reference_segment(sol["p2"], ce2, f"RSBarBrace_preview_{index + 1}_seg2", color)
        dot_id = rs.AddTextDot(f"{index + 1}", midpoint)
        apply_object_display(dot_id, f"RSBarBrace_preview_{index + 1}_label", color=color, layer_name=_BAR_AXIS_LAYER)

        all_ids = [line_id, dot_id, seg1_id, seg2_id] + as_object_id_list(tube_ids)
        pick_set = set(as_object_id_list(all_ids))
        preview_items.append({
            "index": index,
            "pick_ids": pick_set,
            "cleanup_ids": all_ids,
        })
    return preview_items


def _cleanup_previews(preview_items):
    with suspend_redraw():
        for item in preview_items:
            delete_objects(item["cleanup_ids"])


def _print_report(report):
    if report is None:
        return
    solutions = report.get("solutions", [])
    print(f"RSBarBrace: {len(solutions)} solution(s) found")
    for i, sol in enumerate(solutions, 1):
        theta_1, theta_2 = sol["angles"]
        print(
            f"  {i}: family={sol.get('sign_family', '?')}, "
            f"angles=({theta_1:.3f},{theta_2:.3f}) rad, "
            f"residual={sol['residual']:.2e}"
        )


# ---------------------------------------------------------------------------
# Interactive loop using Rhino.Input.Custom.GetObject
# ---------------------------------------------------------------------------

def _interactive_loop(le1_id, le2_id, ce1, ce2):
    """Interactive solve-preview-select loop. Returns chosen solution dict or None."""
    ce1 = point_to_array(ce1)
    ce2 = point_to_array(ce2)
    ce1_ref_id = _bake_reference_point(ce1, "RSBarBrace_Ce1", (230, 80, 80))
    ce2_ref_id = _bake_reference_point(ce2, "RSBarBrace_Ce2", (80, 80, 230))
    ref_ids = as_object_id_list([ce1_ref_id, ce2_ref_id])

    solutions, report = _solve(le1_id, le2_id, ce1, ce2)
    _print_report(report)
    if not solutions:
        rs.MessageBox("No valid brace solutions found. Try different contact points.")
        delete_objects(ref_ids)
        return None

    preview_items = _create_previews(solutions, ce1, ce2)

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

            slide1_idx = go.AddOption("SlidePointOn1")
            slide2_idx = go.AddOption("SlidePointOn2")

            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                return None

            if result == Rhino.Input.GetResult.Object:
                obj_ref = go.Object(0)
                picked_id = obj_ref.ObjectId
                # Match against preview items
                chosen_index = None
                for item in preview_items:
                    if picked_id in item["pick_ids"]:
                        chosen_index = item["index"]
                        break
                if chosen_index is not None:
                    sol = solutions[chosen_index]
                    print(
                        f"RSBarBrace: Selected solution {chosen_index + 1}, "
                        f"family={sol.get('sign_family', '?')}, "
                        f"residual={sol['residual']:.2e}"
                    )
                    return sol
                # Clicked something that isn't a preview — ignore
                continue

            if result == Rhino.Input.GetResult.Option:
                opt = go.Option()
                opt_idx = opt.Index

                if opt_idx == slide1_idx or opt_idx == slide2_idx:
                    # Re-pick a contact point
                    if opt_idx == slide1_idx:
                        new_pt = rs.GetPointOnCurve(le1_id, "Pick new contact point on Le1")
                    else:
                        new_pt = rs.GetPointOnCurve(le2_id, "Pick new contact point on Le2")
                    if new_pt is None:
                        continue

                    if opt_idx == slide1_idx:
                        ce1 = point_to_array(new_pt)
                    else:
                        ce2 = point_to_array(new_pt)

                    # Clean up old previews and reference points, re-solve
                    _cleanup_previews(preview_items)
                    delete_objects(ref_ids)
                    ce1_ref_id = _bake_reference_point(ce1, "RSBarBrace_Ce1", (230, 80, 80))
                    ce2_ref_id = _bake_reference_point(ce2, "RSBarBrace_Ce2", (80, 80, 230))
                    ref_ids = as_object_id_list([ce1_ref_id, ce2_ref_id])

                    solutions, report = _solve(le1_id, le2_id, ce1, ce2)
                    _print_report(report)
                    if not solutions:
                        rs.MessageBox("No valid brace solutions found for the updated points.")
                        delete_objects(ref_ids)
                        return None
                    preview_items = _create_previews(solutions, ce1, ce2)
                    continue

    finally:
        _cleanup_previews(preview_items)
        delete_objects(ref_ids)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    _refresh_runtime_modules()
    rs.UnselectAllObjects()

    le1_id = rs.GetObject("Select first existing bar (Le1)", rs.filter.curve)
    if le1_id is None:
        return
    le2_id = rs.GetObject("Select second existing bar (Le2)", rs.filter.curve)
    if le2_id is None:
        return
    ce1 = rs.GetPointOnCurve(le1_id, "Pick contact point on Le1")
    if ce1 is None:
        return
    ce2 = rs.GetPointOnCurve(le2_id, "Pick contact point on Le2")
    if ce2 is None:
        return

    solution = _interactive_loop(le1_id, le2_id, ce1, ce2)
    if solution is None:
        print("RSBarBrace: Cancelled.")
        return

    # Bake the chosen solution
    with suspend_redraw():
        midpoint = 0.5 * (solution["p1"] + solution["p2"])
        line_id = _add_centered_line(midpoint, solution["nn"])
        chosen_color = _PREVIEW_COLORS[solution.get("_index", 0) % len(_PREVIEW_COLORS)]
        _place_axis_line(line_id, color=chosen_color, label="RSBarBrace_Ln")

        # Register bars and create/update tube previews
        ensure_bar_id(le1_id)
        ensure_bar_id(le2_id)
        ensure_bar_id(line_id)
        ensure_bar_preview(le1_id, float(config.BAR_RADIUS), color=_PREVIEW_COLORS[0])
        ensure_bar_preview(le2_id, float(config.BAR_RADIUS), color=_PREVIEW_COLORS[1])
        ensure_bar_preview(line_id, float(config.BAR_RADIUS), color=chosen_color)

        ce1_arr = point_to_array(solution["p1"]) if "p1" in solution else None
        ce2_arr = point_to_array(solution["p2"]) if "p2" in solution else None
        # Bake final contact segments using the solution's contact-bar closest points
        # p1/p2 are points on the new bar; the contact points on Le are at distance BAR_CONTACT_DISTANCE
        _bake_reference_segment(solution["p1"], solution["p1"], "RSBarBrace_contact_1", (230, 80, 80))
        _bake_reference_segment(solution["p2"], solution["p2"], "RSBarBrace_contact_2", (80, 80, 230))

    rs.SelectObject(line_id)
    theta_1, theta_2 = solution["angles"]
    print(
        f"RSBarBrace: Solution placed. "
        f"Family={solution.get('sign_family', '?')}, "
        f"Angles=({theta_1:.3f},{theta_2:.3f}) rad, "
        f"Residual: {solution['residual']:.2e}"
    )


if __name__ == "__main__":
    main()
