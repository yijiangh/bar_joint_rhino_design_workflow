import json

import numpy as np

from core.t1_s2_case_export import (
    build_t1_s2_case_payload,
    default_t1_s2_case_filename,
    write_t1_s2_case,
)


def test_build_t1_s2_case_payload_is_json_safe():
    payload = build_t1_s2_case_payload(
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        [0.0, 8.0, 0.0],
        [0.0, 18.0, 0.0],
        [1.0, 2.0, 3.0],
        [4.0, 5.0, 6.0],
        bar_contact_distance=36.0,
        bar_radius=6.0,
        nn_init_hint=[3.0, 3.0, 3.0],
        solutions=[
            {
                "nn": np.array([0.0, 0.0, 1.0]),
                "p1": np.array([1.0, 2.0, 39.0]),
                "p2": np.array([4.0, 5.0, 39.0]),
                "residual": 1e-12,
                "angles": (1.0, 2.0),
                "sign_family": "(+,+)",
                "optimization_diagnostics": {
                    "contact_1_objective_component": 1e-14,
                    "contact_2_objective_component": 2e-14,
                    "contact_1_to_bar_angle_deg": 89.9,
                    "contact_2_to_bar_angle_deg": 90.1,
                    "contact_1_orthogonality_error_deg": 0.1,
                    "contact_2_orthogonality_error_deg": 0.1,
                    "contact_1_axial_offset": 0.01,
                    "contact_2_axial_offset": -0.01,
                    "contact_1_normalized_residual": 0.001,
                    "contact_2_normalized_residual": -0.001,
                },
                "optimizer_result": {
                    "nfev": 9,
                    "status": 1,
                    "success": True,
                    "termination_reason": "gtol",
                    "message": "`gtol` termination condition is satisfied.",
                },
            }
        ],
        solver_report={
            "initial_angles": (0.5, 1.5),
            "optimizer_settings": {"xtol": None, "ftol": None, "gtol": 1e-12, "max_nfev": 10000},
            "found_sign_families": ["(+,+)"],
            "missing_sign_families": ["(-,+)", "(+,-)", "(-,-)"],
            "missing_family_attempts": [
                {
                    "target_sign_family": "(-,+)",
                    "converged_sign_family": "(+,+)",
                    "objective": 1e-8,
                    "least_squares_cost": 5e-9,
                    "nfev": 12,
                    "status": 1,
                    "success": True,
                    "termination_reason": "gtol",
                    "message": "`gtol` termination condition is satisfied.",
                    "seed_angles": (4.0, 1.5),
                    "final_angles": (0.9, 1.8),
                    "objective_components": [6e-9, 4e-9],
                    "normalized_residuals": [0.0002, -0.0001],
                    "optimization_diagnostics": {
                        "contact_1_objective_component": 6e-9,
                        "contact_2_objective_component": 4e-9,
                        "contact_1_to_bar_angle_deg": 89.0,
                        "contact_2_to_bar_angle_deg": 89.5,
                        "contact_1_orthogonality_error_deg": 1.0,
                        "contact_2_orthogonality_error_deg": 0.5,
                        "contact_1_axial_offset": 0.12,
                        "contact_2_axial_offset": -0.08,
                        "contact_1_normalized_residual": 0.0002,
                        "contact_2_normalized_residual": -0.0001,
                    },
                }
            ],
        },
        document_path=r"C:\tmp\demo model.3dm",
        model_unit_system="Millimeters",
        scale_to_mm=1.0,
        le1_metadata={"object_id": "id-1", "name": "Le1", "layer": "Bars"},
        le2_metadata={"object_id": "id-2", "name": "Le2", "layer": "Bars"},
    )

    assert payload["config"]["bar_contact_distance"] == 36.0
    assert payload["selection"]["le1"]["length"] == 10.0
    assert payload["selection"]["le1"]["object_id"] == "id-1"
    assert payload["solver_input"]["nn_init_hint"] == [3.0, 3.0, 3.0]
    assert payload["solver_output"]["solution_count"] == 1
    assert payload["solver_output"]["solutions"][0]["angles"] == [1.0, 2.0]
    assert payload["solver_output"]["solutions"][0]["midpoint"] == [2.5, 3.5, 39.0]
    assert payload["solver_output"]["solutions"][0]["sign_family"] == "(+,+)"
    assert payload["solver_output"]["solutions"][0]["optimization_diagnostics"]["contact_1_axial_offset"] == 0.01
    assert payload["solver_output"]["solutions"][0]["optimizer_result"]["termination_reason"] == "gtol"
    assert payload["solver_output"]["initial_angles"] == [0.5, 1.5]
    assert payload["solver_output"]["optimizer_settings"]["xtol"] is None
    assert payload["solver_output"]["optimizer_settings"]["ftol"] is None
    assert payload["solver_output"]["optimizer_settings"]["max_nfev"] == 10000
    assert payload["solver_output"]["missing_sign_families"] == ["(-,+)", "(+,-)", "(-,-)"]
    assert payload["solver_output"]["missing_family_attempts"][0]["target_sign_family"] == "(-,+)"
    assert payload["solver_output"]["missing_family_attempts"][0]["termination_reason"] == "gtol"
    assert payload["solver_output"]["missing_family_attempts"][0]["optimization_diagnostics"]["contact_1_axial_offset"] == 0.12

    json.dumps(payload)


def test_write_t1_s2_case_and_default_filename(tmp_path):
    payload = build_t1_s2_case_payload(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        bar_contact_distance=36.0,
        bar_radius=6.0,
        solver_error="solver failed",
    )
    filename = default_t1_s2_case_filename(r"C:\tmp\demo model.3dm")
    output_path = tmp_path / filename

    write_t1_s2_case(str(output_path), payload)

    loaded = json.loads(output_path.read_text(encoding="utf-8"))
    assert output_path.suffix == ".json"
    assert output_path.name.startswith("demo_model_t1_s2_case_")
    assert loaded["solver_output"]["error"] == "solver failed"
