"""Capture v2 file format: flattened RobotCell + RobotCellState + IK targets.

A capture lives at ``tests/captures/<stem>.json`` and references a
``robot_cells/<sha8>.json`` sibling that holds the serialized
``RobotCell.__data__`` (content-hashed; many captures dedupe to one cell file).

NO v1 fallback. ``load_capture`` raises on ``schema_version != 2``.
"""

from __future__ import annotations

import hashlib
import json
import os
from typing import Any


SCHEMA_VERSION = 2
ROBOT_CELLS_SUBDIR = "robot_cells"


def _compas_dumps(obj) -> str:
    from compas.data import json_dumps
    return json_dumps(obj, pretty=False)


def _compas_loads(text: str):
    from compas.data import json_loads
    return json_loads(text)


def _sha8(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:8]


def _robot_cells_dir(captures_dir: str) -> str:
    return os.path.join(captures_dir, ROBOT_CELLS_SUBDIR)


def save_robot_cell_if_changed(robot_cell, captures_dir: str) -> str:
    """Hash ``RobotCell.__data__`` and write the cell file iff new.

    Returns the relative ref ``"robot_cells/<sha8>.json"``.
    """
    text = _compas_dumps(robot_cell)
    sha = _sha8(text)
    rel = f"{ROBOT_CELLS_SUBDIR}/{sha}.json"
    cells_dir = _robot_cells_dir(captures_dir)
    os.makedirs(cells_dir, exist_ok=True)
    abs_path = os.path.join(cells_dir, f"{sha}.json")
    if not os.path.isfile(abs_path):
        with open(abs_path, "w", encoding="utf-8") as f:
            f.write(text)
    return rel


def load_robot_cell_ref(captures_dir: str, ref: str):
    """Load ``RobotCell`` from a ``robot_cells/<sha8>.json`` ref."""
    abs_path = os.path.join(captures_dir, ref.replace("/", os.sep))
    with open(abs_path, "r", encoding="utf-8") as f:
        return _compas_loads(f.read())


def save_capture_v2(
    path: str,
    *,
    captured_at: str,
    robot_cell_ref: str,
    initial_state,
    ik_targets: dict,
    ik_options: dict,
    expected: dict,
    source: dict,
) -> None:
    """Write a v2 capture JSON.

    ``initial_state`` must be a ``RobotCellState``; serialized via compas.
    ``expected`` values may be ``RobotCellState`` instances or ``None``.
    """
    payload = {
        "schema_version": SCHEMA_VERSION,
        "captured_at": captured_at,
        "robot_cell_ref": robot_cell_ref,
        "initial_state": json.loads(_compas_dumps(initial_state)),
        "ik_targets": ik_targets,
        "ik_options": ik_options,
        "expected": {
            k: (json.loads(_compas_dumps(v)) if v is not None else None)
            for k, v in expected.items()
        },
        "source": source,
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def load_capture(path: str) -> dict[str, Any]:
    """Load a v2 capture JSON. Raises on schema_version != 2."""
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    schema = raw.get("schema_version")
    if schema != SCHEMA_VERSION:
        raise RuntimeError(
            f"capture_io.load_capture: unsupported schema_version={schema!r} "
            f"(need {SCHEMA_VERSION}). Regenerate the capture from RSIKKeyframe."
        )
    return raw


def deserialize_state(state_data) -> "RobotCellState":  # noqa: F821
    """Round-trip a state dict (from a v2 capture) back into a ``RobotCellState``."""
    return _compas_loads(json.dumps(state_data))
