"""Helpers for exporting T1-S2 debug cases outside Rhino."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from typing import Any, Iterable, Mapping, Optional, Sequence

import numpy as np


def _as_vector(values: Iterable[float]) -> np.ndarray:
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != 3:
        raise ValueError("Expected a 3D vector.")
    return array


def _to_xyz_list(values: Iterable[float]) -> list[float]:
    return [float(value) for value in _as_vector(values).tolist()]


def _clean_metadata(metadata: Optional[Mapping[str, Any]]) -> dict[str, Any]:
    if metadata is None:
        return {}
    return {str(key): value for key, value in metadata.items() if value is not None}


def _serialize_solution(solution: Mapping[str, Any]) -> dict[str, Any]:
    p1 = _as_vector(solution["p1"])
    p2 = _as_vector(solution["p2"])
    serialized = {
        "nn": _to_xyz_list(solution["nn"]),
        "p1": _to_xyz_list(p1),
        "p2": _to_xyz_list(p2),
        "midpoint": _to_xyz_list(0.5 * (p1 + p2)),
        "residual": float(solution["residual"]),
    }

    if "angles" in solution:
        angles = tuple(float(angle) for angle in solution["angles"])
        serialized["angles"] = [angles[0], angles[1]]
    if "signs" in solution:
        signs = tuple(float(sign) for sign in solution["signs"])
        serialized["signs"] = [signs[0], signs[1]]
    if "sign_family" in solution:
        serialized["sign_family"] = str(solution["sign_family"])
    if "optimization_diagnostics" in solution:
        serialized["optimization_diagnostics"] = {
            str(key): float(value)
            for key, value in solution["optimization_diagnostics"].items()
        }
    if "optimizer_result" in solution:
        serialized["optimizer_result"] = {
            "nfev": int(solution["optimizer_result"]["nfev"]),
            "status": int(solution["optimizer_result"]["status"]),
            "success": bool(solution["optimizer_result"]["success"]),
            "termination_reason": str(solution["optimizer_result"]["termination_reason"]),
            "message": str(solution["optimizer_result"]["message"]),
        }
    return serialized


def _serialize_attempt(attempt: Mapping[str, Any]) -> dict[str, Any]:
    serialized = {
        "target_sign_family": attempt.get("target_sign_family"),
        "converged_sign_family": attempt.get("converged_sign_family"),
        "objective": float(attempt["objective"]),
        "least_squares_cost": float(attempt["least_squares_cost"]),
        "nfev": int(attempt["nfev"]),
        "status": int(attempt["status"]),
        "success": bool(attempt["success"]),
        "termination_reason": str(attempt["termination_reason"]),
        "message": str(attempt["message"]),
        "seed_angles": [float(attempt["seed_angles"][0]), float(attempt["seed_angles"][1])],
        "final_angles": [float(attempt["final_angles"][0]), float(attempt["final_angles"][1])],
        "objective_components": [float(value) for value in attempt["objective_components"]],
        "normalized_residuals": [float(value) for value in attempt["normalized_residuals"]],
    }
    diagnostics = attempt.get("optimization_diagnostics")
    if diagnostics is not None:
        serialized["optimization_diagnostics"] = {
            str(key): float(value)
            for key, value in diagnostics.items()
        }
    return serialized


def _serialize_optimizer_settings(settings: Mapping[str, Any]) -> dict[str, Any]:
    serialized = {}
    for key, value in settings.items():
        if key == "max_nfev":
            serialized[str(key)] = None if value is None else int(value)
        else:
            serialized[str(key)] = None if value is None else float(value)
    return serialized


def _safe_case_stem(document_path: Optional[str]) -> str:
    if document_path:
        raw_stem = os.path.splitext(os.path.basename(document_path))[0]
    else:
        raw_stem = "untitled"
    safe_stem = "".join(character if character.isalnum() or character in ("-", "_") else "_" for character in raw_stem)
    return safe_stem or "untitled"


def default_t1_s2_case_filename(document_path: Optional[str] = None) -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%SZ")
    return f"{_safe_case_stem(document_path)}_t1_s2_case_{timestamp}.json"


def build_t1_s2_case_payload(
    le1_start: Iterable[float],
    le1_end: Iterable[float],
    le2_start: Iterable[float],
    le2_end: Iterable[float],
    ce1: Iterable[float],
    ce2: Iterable[float],
    *,
    bar_contact_distance: float,
    bar_radius: float,
    nn_init_hint: Optional[Sequence[float]] = None,
    solutions: Optional[Sequence[Mapping[str, Any]]] = None,
    solver_report: Optional[Mapping[str, Any]] = None,
    solver_error: Optional[str] = None,
    document_path: Optional[str] = None,
    model_unit_system: Optional[str] = None,
    scale_to_mm: Optional[float] = None,
    le1_metadata: Optional[Mapping[str, Any]] = None,
    le2_metadata: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    le1_start = _as_vector(le1_start)
    le1_end = _as_vector(le1_end)
    le2_start = _as_vector(le2_start)
    le2_end = _as_vector(le2_end)
    ce1 = _as_vector(ce1)
    ce2 = _as_vector(ce2)
    n1 = le1_end - le1_start
    n2 = le2_end - le2_start
    init_hint = None if nn_init_hint is None else _to_xyz_list(nn_init_hint)
    serialized_solutions = [_serialize_solution(solution) for solution in (solutions or [])]
    solver_output = {
        "solution_count": len(serialized_solutions),
        "solutions": serialized_solutions,
        "error": solver_error,
    }
    if solver_report is not None:
        initial_angles = solver_report.get("initial_angles")
        if initial_angles is not None:
            solver_output["initial_angles"] = [float(initial_angles[0]), float(initial_angles[1])]
        solver_output["found_sign_families"] = [str(label) for label in solver_report.get("found_sign_families", [])]
        solver_output["missing_sign_families"] = [
            str(label) for label in solver_report.get("missing_sign_families", [])
        ]
        solver_output["optimizer_settings"] = _serialize_optimizer_settings(
            solver_report.get("optimizer_settings", {})
        )
        solver_output["missing_family_attempts"] = [
            _serialize_attempt(attempt)
            for attempt in solver_report.get("missing_family_attempts", [])
        ]

    return {
        "format_version": 1,
        "case_type": "t1_s2_solver_case",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "document": {
            "path": document_path,
            "model_unit_system": model_unit_system,
            "scale_to_mm": None if scale_to_mm is None else float(scale_to_mm),
        },
        "config": {
            "bar_contact_distance": float(bar_contact_distance),
            "bar_radius": float(bar_radius),
        },
        "selection": {
            "ce1": _to_xyz_list(ce1),
            "ce2": _to_xyz_list(ce2),
            "le1": {
                "start": _to_xyz_list(le1_start),
                "end": _to_xyz_list(le1_end),
                "direction": _to_xyz_list(n1),
                "length": float(np.linalg.norm(n1)),
                **_clean_metadata(le1_metadata),
            },
            "le2": {
                "start": _to_xyz_list(le2_start),
                "end": _to_xyz_list(le2_end),
                "direction": _to_xyz_list(n2),
                "length": float(np.linalg.norm(n2)),
                **_clean_metadata(le2_metadata),
            },
        },
        "solver_input": {
            "n1": _to_xyz_list(n1),
            "ce1": _to_xyz_list(ce1),
            "n2": _to_xyz_list(n2),
            "ce2": _to_xyz_list(ce2),
            "distance": float(bar_contact_distance),
            "nn_init_hint": init_hint,
        },
        "solver_output": {
            **solver_output,
        },
    }


def write_t1_s2_case(path: str, payload: Mapping[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)
        stream.write("\n")
