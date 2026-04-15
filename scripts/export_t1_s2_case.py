"""Export a T1-S2 Rhino selection as a JSON debug case.

Run in Rhino 8 with:
    _-ScriptEditor _R "C:\\path\\to\\export_t1_s2_case.py"
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import geometry as _geometry_module
from core.t1_s2_case_export import (
    build_t1_s2_case_payload,
    default_t1_s2_case_filename,
    write_t1_s2_case,
)


def _reload_runtime_modules():
    global config, geometry

    config = importlib.reload(_config_module)
    geometry = importlib.reload(_geometry_module)


def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def _curve_endpoints(curve_id):
    return _point_to_array(rs.CurveStartPoint(curve_id)), _point_to_array(rs.CurveEndPoint(curve_id))


def _object_metadata(object_id):
    return {
        "object_id": str(object_id),
        "name": rs.ObjectName(object_id),
        "layer": rs.ObjectLayer(object_id),
    }


def _doc_unit_scale_to_mm() -> float:
    return float(Rhino.RhinoMath.UnitScale(sc.doc.ModelUnitSystem, Rhino.UnitSystem.Millimeters))


def _project_root() -> str:
    return os.path.dirname(SCRIPT_DIR)


def _default_export_dir() -> str:
    export_dir = os.path.join(_project_root(), "tests", "debug_cases")
    os.makedirs(export_dir, exist_ok=True)
    return export_dir


def _cached_s2_inputs():
    cached_inputs = sc.sticky.get("t1_inputs")
    if isinstance(cached_inputs, dict) and cached_inputs.get("mode") == "S2":
        return cached_inputs
    return None


def _select_s2_inputs():
    rs.UnselectAllObjects()
    le1_id = rs.GetObject("Select first existing bar (Le1) for the T1-S2 debug export", rs.filter.curve)
    if le1_id is None:
        return None
    le2_id = rs.GetObject("Select second existing bar (Le2) for the T1-S2 debug export", rs.filter.curve)
    if le2_id is None:
        return None
    ce1 = rs.GetPointOnCurve(le1_id, "Pick contact point on Le1 (Ce1) for the T1-S2 debug export")
    if ce1 is None:
        return None
    ce2 = rs.GetPointOnCurve(le2_id, "Pick contact point on Le2 (Ce2) for the T1-S2 debug export")
    if ce2 is None:
        return None
    return {
        "mode": "S2",
        "le1_id": le1_id,
        "le2_id": le2_id,
        "ce1": ce1,
        "ce2": ce2,
    }


def _resolve_inputs():
    cached_inputs = _cached_s2_inputs()
    if cached_inputs is not None:
        use_cached = rs.MessageBox(
            "Use the cached T1-S2 selection from the last T1 run?\nChoose No to repick Le1, Le2, Ce1, and Ce2.",
            4,
            "Export T1-S2 Case",
        )
        if use_cached == 6:
            return cached_inputs

    fresh_inputs = _select_s2_inputs()
    if fresh_inputs is not None:
        sc.sticky["t1_inputs"] = fresh_inputs
    return fresh_inputs


def _pick_export_path():
    document_path = sc.doc.Path if sc.doc is not None else None
    default_filename = default_t1_s2_case_filename(document_path)
    export_path = rs.SaveFileName(
        "Save the T1-S2 debug case as JSON",
        "JSON Files (*.json)|*.json||",
        _default_export_dir(),
        default_filename,
        "json",
    )
    if export_path is None:
        return None
    if not export_path.lower().endswith(".json"):
        export_path = f"{export_path}.json"
    return export_path


def main():
    _reload_runtime_modules()
    inputs = _resolve_inputs()
    if inputs is None:
        return

    export_path = _pick_export_path()
    if export_path is None:
        return

    le1_start, le1_end = _curve_endpoints(inputs["le1_id"])
    le2_start, le2_end = _curve_endpoints(inputs["le2_id"])
    ce1 = _point_to_array(inputs["ce1"])
    ce2 = _point_to_array(inputs["ce2"])
    n1 = le1_end - le1_start
    n2 = le2_end - le2_start
    nn_init_hint = ce2 - ce1

    report = None
    solutions = []
    solver_error = None
    try:
        report = geometry.solve_s2_t1_report(
            n1,
            ce1,
            n2,
            ce2,
            float(config.BAR_CONTACT_DISTANCE),
            nn_init_hint=nn_init_hint,
        )
        solutions = report["solutions"]
    except Exception as exc:
        solver_error = str(exc)

    payload = build_t1_s2_case_payload(
        le1_start,
        le1_end,
        le2_start,
        le2_end,
        ce1,
        ce2,
        bar_contact_distance=float(config.BAR_CONTACT_DISTANCE),
        bar_radius=float(config.BAR_RADIUS),
        nn_init_hint=nn_init_hint,
        solutions=solutions,
        solver_report=report,
        solver_error=solver_error,
        document_path=sc.doc.Path if sc.doc is not None else None,
        model_unit_system=str(sc.doc.ModelUnitSystem) if sc.doc is not None else None,
        scale_to_mm=_doc_unit_scale_to_mm() if sc.doc is not None else None,
        le1_metadata=_object_metadata(inputs["le1_id"]),
        le2_metadata=_object_metadata(inputs["le2_id"]),
    )
    write_t1_s2_case(export_path, payload)

    print(f"T1-S2 debug case exported: {export_path}")
    if solver_error is not None:
        print(f"Solver error captured in the case file: {solver_error}")
    else:
        print(
            f"Captured {len(solutions)} solution(s) with "
            f"BAR_CONTACT_DISTANCE={float(config.BAR_CONTACT_DISTANCE):.2f}"
        )


if __name__ == "__main__":
    main()
