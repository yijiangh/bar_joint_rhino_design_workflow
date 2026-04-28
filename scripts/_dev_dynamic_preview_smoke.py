#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""Manual smoke test for core.dynamic_preview.

Open in Rhino ScriptEditor, click Run. The script picks a point in the
viewport while a half-transparent 200x200x200 mm cube ghost follows the
cursor (centered on the cursor). Press Esc to cancel.

This script is for interactive verification only; it has no automated
assertion. Use it after editing core/dynamic_preview.py.
"""

from __future__ import annotations

import importlib
import os
import sys

import Rhino
import scriptcontext as sc

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from core import dynamic_preview  # noqa: E402

importlib.reload(dynamic_preview)


def _unit_cube_mesh(size_mm=200.0):
    half = size_mm / 2.0
    bbox = Rhino.Geometry.BoundingBox(
        Rhino.Geometry.Point3d(-half, -half, -half),
        Rhino.Geometry.Point3d(half, half, half),
    )
    box = Rhino.Geometry.Box(bbox)
    return Rhino.Geometry.Mesh.CreateFromBox(box, 1, 1, 1)


def _translation_to_cursor(cursor_pt):
    return Rhino.Geometry.Transform.Translation(cursor_pt.X, cursor_pt.Y, cursor_pt.Z)


def main():
    cube = _unit_cube_mesh(200.0)
    with dynamic_preview.mesh_preview([cube], alpha=0.5) as conduit:
        picker = dynamic_preview.TrackingGetPoint(conduit, _translation_to_cursor)
        picker.SetCommandPrompt("Move cursor to see ghost cube; click to accept")
        result = picker.Get()
        if result != Rhino.Input.GetResult.Point:
            print("smoke test: cancelled.")
            return
        pt = picker.Point()
        print(f"smoke test: picked point ({pt.X:.2f}, {pt.Y:.2f}, {pt.Z:.2f})")


if __name__ == "__main__":
    main()
