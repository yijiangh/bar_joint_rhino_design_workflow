"""A new bar touching two existing bars: solve, preview, pick, bake, joint.

The shared body of RSBarBrace and RSBarSubfloor.  Both pick two existing bars
(Le1, Le2) and a contact point on each; the S2-T1 solver
(``core.geometry.solve_s2_t1_report``) returns up to four placements of a new
bar that sits ``contact_distance_mm`` from both.  The user clicks one in the
viewport (``SlidePointOn…`` re-picks a contact point, ``Length`` resizes the
candidates), it is baked as a registered bar, and a joint pair is auto-placed
at each end -- the existing bar takes the receiver, the new bar the male.

Each command keeps only what differs: its first-bar picker (one ``Pair`` for a
brace; ``LeftJointPair`` / ``RightJointPair`` for a subfloor) and the words
it prints, given as a :class:`TwoContactCommand`.

Rhino-runtime only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core import config
from core import geometry
from core.joint_auto_place import auto_place_joint_pair
from core.rhino_bar_pick import pick_bar, set_default_brace_length
from core.rhino_bar_registry import (
    ensure_bar_id,
    ensure_bar_preview,
    paint_bar,
    reset_bar_color,
)
from core.rhino_helpers import (
    add_centered_line,
    apply_object_display,
    as_object_id_list,
    curve_endpoints,
    delete_objects,
    ensure_layer,
    point_to_array,
    set_object_color,
    suspend_redraw,
)


@dataclass(frozen=True)
class TwoContactCommand:
    """The words one command uses; everything else is shared."""

    name: str                  # "RSBarBrace"
    noun: str                  # "brace" -- in "No valid brace solutions found"
    length_noun: str           # "brace length"
    second_bar_prompt: str     # "Select second existing bar (Le2)"
    contact_prompts: tuple     # first pick of each contact point (Le1, Le2)
    slide_options: tuple       # option names that re-pick a contact (Le1, Le2)
    repick_prompts: tuple      # prompts of those re-picks (Le1, Le2)


# ---------------------------------------------------------------------------
# Baking helpers
# ---------------------------------------------------------------------------


def place_axis_line(curve_id, *, color=None, label=None):
    """Move *curve_id* to the bar centre-line layer, optionally colored/labelled."""
    if curve_id is None or not rs.IsObject(curve_id):
        return None
    ensure_layer(config.LAYER_BAR_CENTERLINES)
    rs.ObjectLayer(curve_id, config.LAYER_BAR_CENTERLINES)
    if color is not None:
        set_object_color(curve_id, color)
    if label:
        rs.SetUserText(curve_id, "axis_label", label)
    return curve_id


def bake_reference_point(point, label, color):
    pid = rs.AddPoint(point_to_array(point).tolist())
    if pid is None:
        return None
    apply_object_display(pid, label, color=color, layer_name=config.LAYER_BAR_CENTERLINES)
    return pid


def bake_axis_tube(axis_curve_id, label, color=None):
    """A temporary tube around *axis_curve_id*, for the interactive previews."""
    if axis_curve_id is None or not rs.IsObject(axis_curve_id):
        return []
    start_xyz, end_xyz = curve_endpoints(axis_curve_id)
    axis_vector = end_xyz - start_xyz
    axis_length = float(np.linalg.norm(axis_vector))
    if axis_length <= 1e-9:
        return []
    layer_name = config.LAYER_BAR_TUBE_PREVIEWS
    ensure_layer(layer_name)
    base_plane = Rhino.Geometry.Plane(
        Rhino.Geometry.Point3d(*start_xyz.tolist()),
        Rhino.Geometry.Vector3d(*(axis_vector / axis_length).tolist()),
    )
    cylinder = Rhino.Geometry.Cylinder(
        Rhino.Geometry.Circle(base_plane, float(config.BAR_RADIUS)), axis_length
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


# ---------------------------------------------------------------------------
# Solve + preview
# ---------------------------------------------------------------------------


def solve(cmd, le1_id, le2_id, ce1, ce2, target_distance):
    """Run the S2-T1 solver.  Returns ``(solutions, report)``; ``([], None)`` on error."""
    le1_start, le1_end = curve_endpoints(le1_id)
    le2_start, le2_end = curve_endpoints(le2_id)
    ce1 = point_to_array(ce1)
    ce2 = point_to_array(ce2)
    try:
        report = geometry.solve_s2_t1_report(
            le1_end - le1_start, ce1, le2_end - le2_start, ce2, float(target_distance),
            nn_init_hint=ce2 - ce1,
        )
    except Exception as exc:
        print(f"{cmd.name} solver error: {exc}")
        return [], None
    return report.get("solutions", []), report


def print_report(cmd, report):
    if report is None:
        return
    solutions = report.get("solutions", [])
    print(f"{cmd.name}: {len(solutions)} solution(s) found")
    for i, sol in enumerate(solutions, 1):
        theta_1, theta_2 = sol["angles"]
        print(
            f"  {i}: family={sol.get('sign_family', '?')}, "
            f"angles=({theta_1:.3f},{theta_2:.3f}) rad, "
            f"residual={sol['residual']:.2e}"
        )


def create_previews(cmd, solutions, length):
    """Preview line + tube + number dot per solution, colored by variant."""
    palette = config.VARIANT_PREVIEW_COLORS
    preview_items = []
    half_length = float(length) / 2.0
    for index, sol in enumerate(solutions):
        midpoint = 0.5 * (sol["p1"] + sol["p2"])
        direction = sol["nn"] / np.linalg.norm(sol["nn"])
        color = palette[index % len(palette)]
        label = f"{cmd.name}_preview_{index + 1}"

        line_id = rs.AddLine(midpoint - half_length * direction, midpoint + half_length * direction)
        place_axis_line(line_id, color=color, label=label)
        tube_ids = bake_axis_tube(line_id, f"{label}_tube", color=color)
        dot_id = rs.AddTextDot(f"{index + 1}", midpoint)
        apply_object_display(
            dot_id, f"{label}_label", color=color, layer_name=config.LAYER_BAR_CENTERLINES
        )

        all_ids = [line_id, dot_id] + as_object_id_list(tube_ids)
        preview_items.append(
            {"index": index, "pick_ids": set(as_object_id_list(all_ids)), "cleanup_ids": all_ids}
        )
    return preview_items


def cleanup_previews(preview_items):
    with suspend_redraw():
        for item in preview_items:
            delete_objects(item["cleanup_ids"])


# ---------------------------------------------------------------------------
# Interactive loop
# ---------------------------------------------------------------------------


def interactive_loop(cmd, le1_id, le2_id, ce1, ce2, target_distance, length):
    """Solve-preview-select loop.

    Returns ``(solution, length)``; *length* may have changed through the
    ``Length`` option.  ``(None, length)`` on cancel.
    """
    palette = config.VARIANT_PREVIEW_COLORS
    contacts = [point_to_array(ce1), point_to_array(ce2)]
    bars = (le1_id, le2_id)

    def _bake_refs():
        return as_object_id_list([
            bake_reference_point(contacts[0], f"{cmd.name}_Ce1", palette[0]),
            bake_reference_point(contacts[1], f"{cmd.name}_Ce2", palette[1]),
        ])

    ref_ids = _bake_refs()
    solutions, report = solve(cmd, le1_id, le2_id, contacts[0], contacts[1], target_distance)
    print_report(cmd, report)
    if not solutions:
        rs.MessageBox(f"No valid {cmd.noun} solutions found. Try different contact points.")
        delete_objects(ref_ids)
        return None, length

    preview_items = create_previews(cmd, solutions, length)
    # One OptionDouble kept across iterations: GetObject updates CurrentValue in
    # place, and recreating it each loop would lose the value just typed.
    length_value = Rhino.Input.Custom.OptionDouble(float(length), 1.0, 100000.0)

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
            slide_idx = [go.AddOption(name) for name in cmd.slide_options]
            go.AddOptionDouble("Length", length_value)

            result = go.Get()
            if result == Rhino.Input.GetResult.Cancel:
                return None, length

            if result == Rhino.Input.GetResult.Object:
                picked_id = go.Object(0).ObjectId
                chosen = next(
                    (it["index"] for it in preview_items if picked_id in it["pick_ids"]), None
                )
                if chosen is None:
                    continue  # clicked something that is not a candidate
                sol = solutions[chosen]
                print(
                    f"{cmd.name}: Selected solution {chosen + 1}, "
                    f"family={sol.get('sign_family', '?')}, residual={sol['residual']:.2e}"
                )
                return sol, length

            if result != Rhino.Input.GetResult.Option:
                continue

            new_length = float(length_value.CurrentValue)
            if abs(new_length - float(length)) > 1e-9:
                length = new_length
                set_default_brace_length(length)
                print(f"{cmd.name}: {cmd.length_noun} updated to {length:.2f} mm; "
                      "regenerating previews.")
                cleanup_previews(preview_items)
                preview_items = create_previews(cmd, solutions, length)
                continue

            opt = go.Option()
            side = slide_idx.index(opt.Index) if opt is not None and opt.Index in slide_idx else None
            if side is None:
                continue
            new_pt = rs.GetPointOnCurve(bars[side], cmd.repick_prompts[side])
            if new_pt is None:
                continue
            contacts[side] = point_to_array(new_pt)

            cleanup_previews(preview_items)
            delete_objects(ref_ids)
            ref_ids = _bake_refs()
            solutions, report = solve(
                cmd, le1_id, le2_id, contacts[0], contacts[1], target_distance
            )
            print_report(cmd, report)
            if not solutions:
                rs.MessageBox(f"No valid {cmd.noun} solutions found for the updated points.")
                delete_objects(ref_ids)
                preview_items = []
                return None, length
            preview_items = create_previews(cmd, solutions, length)
    finally:
        cleanup_previews(preview_items)
        delete_objects(ref_ids)


# ---------------------------------------------------------------------------
# The whole command after the first bar is picked
# ---------------------------------------------------------------------------


def run(cmd, le1_id, pairs, target_distance, length):
    """Pick Le2 and both contacts, choose a candidate, bake it, place joints.

    *pairs* is ``(pair_on_le1, pair_on_le2)``: each existing bar takes that
    pair's receiver, the new bar its male.
    """
    le2_id = None
    line_id = None
    try:
        ensure_bar_id(le1_id)
        ensure_bar_preview(le1_id, float(config.BAR_RADIUS))
        paint_bar(le1_id, config.SELECTED_BAR_COLOR)

        le2_id = pick_bar(cmd.second_bar_prompt)
        if le2_id is None:
            return
        ensure_bar_id(le2_id)
        ensure_bar_preview(le2_id, float(config.BAR_RADIUS))
        paint_bar(le2_id, config.SELECTED_BAR_COLOR)

        ce1 = rs.GetPointOnCurve(le1_id, cmd.contact_prompts[0])
        if ce1 is None:
            return
        ce2 = rs.GetPointOnCurve(le2_id, cmd.contact_prompts[1])
        if ce2 is None:
            return

        solution, length = interactive_loop(
            cmd, le1_id, le2_id, ce1, ce2, target_distance, length
        )
        if solution is None:
            print(f"{cmd.name}: Cancelled.")
            return
        print(f"{cmd.name}: final {cmd.length_noun} {float(length):.2f} mm")

        with suspend_redraw():
            midpoint = 0.5 * (solution["p1"] + solution["p2"])
            line_id = add_centered_line(midpoint, solution["nn"], length)
            place_axis_line(line_id, label=f"{cmd.name}_Ln")
            ensure_bar_id(line_id)
            ensure_bar_preview(line_id, float(config.BAR_RADIUS))

        theta_1, theta_2 = solution["angles"]
        print(
            f"{cmd.name}: Solution placed. Family={solution.get('sign_family', '?')}, "
            f"Angles=({theta_1:.3f},{theta_2:.3f}) rad, Residual: {solution['residual']:.2e}"
        )
        # Default variant; each joint can be refined later with RSJointEdit.
        auto_place_joint_pair(le1_id, line_id, pairs[0])
        auto_place_joint_pair(le2_id, line_id, pairs[1])
    finally:
        reset_bar_color(le1_id)
        reset_bar_color(le2_id)
        reset_bar_color(line_id)
        sc.doc.Views.Redraw()
