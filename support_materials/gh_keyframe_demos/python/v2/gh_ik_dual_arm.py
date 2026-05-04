#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: compas==2.13.0
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""C4: dual-arm IK with optional bar-as-rigid-body env collision.

Inputs:
    trigger          : bool   -- rising-edge re-solves; downstream caches
                                 the last solved state until next trigger
    tool0_left_mm    : Rhino.Geometry.Transform (mm-tagged) from C3
    tool0_right_mm   : Rhino.Geometry.Transform (mm-tagged) from C3
    base_plane       : Rhino.Geometry.Plane (doc units) -- robot base
    check_self       : bool, default True   -- self collision
    check_bar_coll   : bool, default False  -- bar-robot collision
    bar_curve        : Rhino.Geometry.Curve (doc units) from C2 -- only used
                       when check_bar_coll=True; tube-meshed and registered
                       as a static rigid body.
    bar_radius_mm    : float, default = config.BAR_RADIUS (10 mm)

Outputs:
    ik_state         : RobotCellState (or None if unsolved)
    solved           : bool
    base_frame_mm    : list[list[float]] 4x4 -- echoed for debugging

Joint-tool collision is allowed by NOT registering the male/female joint
blocks as env rigid bodies; the tool is attached to wrist links via
`touch_links` already configured by `core.robot_cell._configure_tool_states`.
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
from core import env_collision  # noqa: E402
from core import robot_cell  # noqa: E402
from core.rhino_frame_io import doc_unit_scale_to_mm  # noqa: E402


def _plane_to_4x4_mm(plane, scale_to_mm):
    o, x, y, z = plane.Origin, plane.XAxis, plane.YAxis, plane.ZAxis
    matrix = np.eye(4, dtype=float)
    matrix[:3, 0] = [x.X, x.Y, x.Z]
    matrix[:3, 1] = [y.X, y.Y, y.Z]
    matrix[:3, 2] = [z.X, z.Y, z.Z]
    matrix[:3, 3] = np.array([o.X, o.Y, o.Z], dtype=float) * scale_to_mm
    return matrix


def _xform_raw_to_np_mm(xform):
    """Unpack a Rhino.Geometry.Transform (mm-tagged) into a 4x4 numpy array.
    Matches `_np_mm_to_xform_raw` in gh_bar_two_males / gh_male_ocf_to_tool0.
    """
    return np.array([[float(xform[i, j]) for j in range(4)] for i in range(4)], dtype=float)


def _doc_curve_to_mm_curve(curve, scale_to_mm):
    """Return a LineCurve in mm built from `curve`'s endpoints in doc units."""
    s = curve.PointAtStart
    e = curve.PointAtEnd
    return Rhino.Geometry.LineCurve(
        Rhino.Geometry.Point3d(s.X * scale_to_mm, s.Y * scale_to_mm, s.Z * scale_to_mm),
        Rhino.Geometry.Point3d(e.X * scale_to_mm, e.Y * scale_to_mm, e.Z * scale_to_mm),
    )


ik_state = None
solved = False
base_frame_mm = None

if not robot_cell.is_pb_running():
    print("gh_ik_dual_arm: PyBullet not running; trigger gh_init_pb first.")
elif trigger:
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()
    template = robot_cell.default_cell_state()

    scale_to_mm = doc_unit_scale_to_mm()

    # Optional bar-as-RB
    if check_bar_coll and bar_curve is not None:
        radius_mm = float(bar_radius_mm) if bar_radius_mm is not None else float(config.BAR_RADIUS)
        mm_curve = _doc_curve_to_mm_curve(bar_curve, scale_to_mm)
        bar_mesh = env_collision.bar_tube_compas_mesh_from_geometry(mm_curve, radius_mm)
        if bar_mesh is None:
            print("gh_ik_dual_arm: bar tube mesh build failed; skipping env collision.")
            env_geom = {}
        else:
            env_geom = {
                "env_bar_active": {
                    "mesh": bar_mesh,
                    "frame_world_mm": np.eye(4, dtype=float),
                    "kind": "bar",
                }
            }
    else:
        env_geom = {}

    robot_cell.ensure_env_registered(rcell, env_geom, planner)
    if env_geom:
        template = env_collision.build_env_state(template, env_geom)

    base_mm_np = _plane_to_4x4_mm(base_plane, scale_to_mm)
    base_frame_mm = base_mm_np.tolist()

    tool0_l = _xform_raw_to_np_mm(tool0_left_mm)
    tool0_r = _xform_raw_to_np_mm(tool0_right_mm)

    check = bool(check_self) or bool(check_bar_coll)
    state = robot_cell.solve_dual_arm_ik(
        planner,
        template,
        base_mm_np,
        tool0_l,
        tool0_r,
        check_collision=check,
        verbose_pairs=check,
    )
    if state is not None:
        ik_state = state
        solved = True
        # Push to PB so the GUI / SceneObject sync.
        robot_cell.set_cell_state(planner, ik_state)
        print("gh_ik_dual_arm: solved.")
    else:
        print("gh_ik_dual_arm: IK failed.")
