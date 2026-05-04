#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: compas==2.13.0
"""C2: bar + two male-joint pose computation.

Inputs:
    pair_name        : str, default "T20"
    bar_length_mm    : float, default 2000.0
    bar_anchor_plane : Rhino.Geometry.Plane. **Origin = bar midpoint** (per
                       docs/coordinate_conventions.md §2). +Z = bar axis.
                       Defaults to WorldXY when None.
    jp_left, jr_left : float -- offset of left male from bar MIDPOINT along
                       bar Z (mm) and rotation about bar Z (rad). jp = 0 puts
                       the male AT the anchor; range [-L/2, +L/2] sweeps the
                       full bar.
    jp_right, jr_right : same for right male

Outputs:
    bar_curve         : Rhino.Geometry.LineCurve (doc units) -- bar centerline
    male_left_plane   : Rhino.Geometry.Plane (doc units)     -- left male block frame
    male_right_plane  : Rhino.Geometry.Plane (doc units)     -- right male block frame
    male_left_ocf_mm  : Rhino.Geometry.Transform (mm-tagged) -- pipe to C3
    male_right_ocf_mm : Rhino.Geometry.Transform (mm-tagged) -- pipe to C3

The OCF outputs are wrapped in `Rhino.Geometry.Transform` (not Python
nested lists) because GH decomposes list-of-lists across wires (item-access
receivers see only the first row). `Transform` is a single Rhino primitive
that GH carries atomically. The matrix entries are mm; downstream
components unpack with `_xform_raw_to_np_mm`.
"""

import os
import sys

import Rhino

REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
SCRIPTS = os.path.join(REPO, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import numpy as np

from core import joint_pair  # noqa: E402
from core.rhino_frame_io import doc_unit_scale_to_mm  # noqa: E402


def _plane_origin_xaxis_yaxis_mm(plane, scale_to_mm):
    o = plane.Origin
    x = plane.XAxis
    y = plane.YAxis
    z = plane.ZAxis
    origin_mm = np.array([o.X, o.Y, o.Z], dtype=float) * scale_to_mm
    return origin_mm, np.array([x.X, x.Y, x.Z]), np.array([y.X, y.Y, y.Z]), np.array([z.X, z.Y, z.Z])


def _mm_matrix_to_rhino_plane(matrix_mm, scale_from_mm):
    origin_doc = np.asarray(matrix_mm[:3, 3], dtype=float) * scale_from_mm
    plane = Rhino.Geometry.Plane(
        Rhino.Geometry.Point3d(float(origin_doc[0]), float(origin_doc[1]), float(origin_doc[2])),
        Rhino.Geometry.Vector3d(float(matrix_mm[0, 0]), float(matrix_mm[1, 0]), float(matrix_mm[2, 0])),
        Rhino.Geometry.Vector3d(float(matrix_mm[0, 1]), float(matrix_mm[1, 1]), float(matrix_mm[2, 1])),
    )
    return plane


def _np_mm_to_xform_raw(matrix_mm):
    """Pack a 4x4 numpy (mm) into a Rhino.Geometry.Transform without unit
    conversion. Used as a GH wire transport; downstream unwraps with
    `_xform_raw_to_np_mm`. NOT the same as `rs_ik_keyframe._np_mm_to_rhino_xform`
    -- that helper bakes a doc-unit Transform for actual geometry transforms.
    """
    xf = Rhino.Geometry.Transform(1.0)
    for i in range(4):
        for j in range(4):
            xf[i, j] = float(matrix_mm[i, j])
    return xf


pair = joint_pair.get_joint_pair(str(pair_name) if pair_name else "T20")
length_mm = float(bar_length_mm) if bar_length_mm is not None else 2000.0
anchor = bar_anchor_plane if bar_anchor_plane is not None else Rhino.Geometry.Plane.WorldXY

scale_to_mm = doc_unit_scale_to_mm()
scale_from_mm = 1.0 / scale_to_mm

origin_mm, _xa, _ya, z_unit = _plane_origin_xaxis_yaxis_mm(anchor, scale_to_mm)
half_length_mm = length_mm * 0.5
# Anchor.Origin == bar MIDPOINT (per docs/coordinate_conventions.md §2).
# Build the bar symmetrically about the anchor.
bar_start_mm = origin_mm - z_unit * half_length_mm
bar_end_mm = origin_mm + z_unit * half_length_mm

bar_frame_mm = joint_pair.canonical_bar_frame_from_line(bar_start_mm, bar_end_mm)

# `canonical_bar_frame_from_line` puts bar_frame.origin at bar_start, so the
# internal jp is measured from bar_start. We want user-facing jp measured
# from the bar MIDPOINT (jp_user = 0 -> joint AT anchor); shift by +L/2.
fk_left = joint_pair.fk_half_from_bar_frame(
    bar_frame_mm, float(jp_left) + half_length_mm, float(jr_left), pair.male
)
fk_right = joint_pair.fk_half_from_bar_frame(
    bar_frame_mm, float(jp_right) + half_length_mm, float(jr_right), pair.male
)

male_left_block_mm = fk_left["block_frame"]
male_right_block_mm = fk_right["block_frame"]

# Rhino outputs (doc units)
bar_curve = Rhino.Geometry.LineCurve(
    Rhino.Geometry.Point3d(*(bar_start_mm * scale_from_mm)),
    Rhino.Geometry.Point3d(*(bar_end_mm * scale_from_mm)),
)
male_left_plane = _mm_matrix_to_rhino_plane(male_left_block_mm, scale_from_mm)
male_right_plane = _mm_matrix_to_rhino_plane(male_right_block_mm, scale_from_mm)

# Wrap mm-tagged 4x4s as Rhino.Geometry.Transform so GH carries them
# atomically across wires (Python list-of-lists gets decomposed into
# branches, which breaks item-access receivers).
male_left_ocf_mm = _np_mm_to_xform_raw(male_left_block_mm)
male_right_ocf_mm = _np_mm_to_xform_raw(male_right_block_mm)
