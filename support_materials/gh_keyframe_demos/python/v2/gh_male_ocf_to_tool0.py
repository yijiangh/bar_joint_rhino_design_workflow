#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""C3: per-arm OCF -> tool0 transform for the dual-arm grasp.

Inputs:
    pair_name         : str, default "T20"
    male_left_ocf_mm  : Rhino.Geometry.Transform (mm-tagged) from C2
    male_right_ocf_mm : Rhino.Geometry.Transform (mm-tagged) from C2

Outputs:
    tool0_left_mm    : Rhino.Geometry.Transform (mm-tagged) -- pipe to C4
    tool0_right_mm   : Rhino.Geometry.Transform (mm-tagged) -- pipe to C4
    tool0_left_plane : Rhino.Geometry.Plane (doc units)
    tool0_right_plane: Rhino.Geometry.Plane (doc units)

Mirrors `rs_ik_keyframe._tool0_from_male`. The dispatch key is
`f"{pair_name}_Male"` to match the convention exported by
`RSExportGraspTool0TF` (Joint mode).
"""

import os
import sys

import Rhino

REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
SCRIPTS = os.path.join(REPO, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import numpy as np

from core import config  # noqa: E402
from core.rhino_frame_io import doc_unit_scale_to_mm  # noqa: E402


def _resolve_per_side(name):
    key = f"{name}_Male"
    if key not in config.MALE_JOINT_OCF_TO_TOOL0:
        raise RuntimeError(
            f"No MALE_JOINT_OCF_TO_TOOL0 entry for '{key}'. "
            "Run RSExportGraspTool0TF (Joint mode) for this joint type first."
        )
    per_side = config.MALE_JOINT_OCF_TO_TOOL0[key]
    if "left" not in per_side or "right" not in per_side:
        raise RuntimeError(
            f"MALE_JOINT_OCF_TO_TOOL0['{key}'] is missing left/right entries; "
            "re-run RSExportGraspTool0TF for both arm sides."
        )
    return per_side


def _mm_matrix_to_rhino_plane(matrix_mm, scale_from_mm):
    origin_doc = np.asarray(matrix_mm[:3, 3], dtype=float) * scale_from_mm
    return Rhino.Geometry.Plane(
        Rhino.Geometry.Point3d(float(origin_doc[0]), float(origin_doc[1]), float(origin_doc[2])),
        Rhino.Geometry.Vector3d(float(matrix_mm[0, 0]), float(matrix_mm[1, 0]), float(matrix_mm[2, 0])),
        Rhino.Geometry.Vector3d(float(matrix_mm[0, 1]), float(matrix_mm[1, 1]), float(matrix_mm[2, 1])),
    )


def _xform_raw_to_np_mm(xform):
    """Unpack a Rhino.Geometry.Transform (mm-tagged) into a 4x4 numpy array.
    Inverse of `_np_mm_to_xform_raw` in gh_bar_two_males.py.
    """
    return np.array([[float(xform[i, j]) for j in range(4)] for i in range(4)], dtype=float)


def _np_mm_to_xform_raw(matrix_mm):
    xf = Rhino.Geometry.Transform(1.0)
    for i in range(4):
        for j in range(4):
            xf[i, j] = float(matrix_mm[i, j])
    return xf


per_side = _resolve_per_side(str(pair_name) if pair_name else "T20")

ocf_l = _xform_raw_to_np_mm(male_left_ocf_mm)
ocf_r = _xform_raw_to_np_mm(male_right_ocf_mm)

tool0_l = ocf_l @ per_side["left"]
tool0_r = ocf_r @ per_side["right"]

scale_from_mm = 1.0 / doc_unit_scale_to_mm()

tool0_left_mm = _np_mm_to_xform_raw(tool0_l)
tool0_right_mm = _np_mm_to_xform_raw(tool0_r)
tool0_left_plane = _mm_matrix_to_rhino_plane(tool0_l, scale_from_mm)
tool0_right_plane = _mm_matrix_to_rhino_plane(tool0_r, scale_from_mm)
