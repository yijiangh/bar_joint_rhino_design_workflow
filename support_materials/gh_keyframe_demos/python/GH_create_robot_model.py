#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via the
# sys.path injection below (mirrors `scripts/core/robot_cell.py`). Do NOT list it
# under `# r:` (the pip cache would ignore submodule SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1

from scriptcontext import sticky as st
import os
import sys
import compas


# ---------------------------------------------------------------------------
# Load compas_fab from the in-repo submodule (`external/compas_fab/src`), NOT a
# pip / `# r:` copy -- mirrors the sys.path injection in
# `scripts/core/robot_cell.py`. A GH component has no reliable `__file__`, so we
# anchor on the `pkg_path` input (which lives inside this repo, at
# `<repo>/asset/husky_urdf`) and walk up its parents until we find
# `external/compas_fab/src`, prepend it to sys.path, then import compas_fab.
#
# After importing we verify the resolved module actually lives under that folder:
# a stale copy cached from a sibling repo earlier in the same Rhino session would
# otherwise silently shadow ours. If that happens, reset the Python 3 (CPython)
# engine (ScriptEditor -> Tools -> Reload Python 3 Engine) to clear the state.
# ---------------------------------------------------------------------------
def _find_compas_fab_src(start_dir):
    d = os.path.normpath(start_dir)
    while True:
        candidate = os.path.join(d, "external", "compas_fab", "src")
        if os.path.isdir(candidate):
            return candidate
        parent = os.path.dirname(d)
        if parent == d:  # reached filesystem root
            return None
        d = parent


_COMPAS_FAB_SRC = _find_compas_fab_src(os.path.dirname(os.path.normpath(pkg_path)))
if _COMPAS_FAB_SRC is None:
    raise RuntimeError(
        "compas_fab submodule not found above pkg_path={!r}. Expected "
        "'<repo>/external/compas_fab/src'. Run "
        "`git submodule update --init --recursive`.".format(pkg_path)
    )
if _COMPAS_FAB_SRC not in sys.path:
    sys.path.insert(0, _COMPAS_FAB_SRC)

from compas_robots import RobotModel
from compas_robots.resources import GithubPackageMeshLoader
from compas_robots.resources import LocalPackageMeshLoader
from compas_robots.ghpython.scene import RobotModelObject
# from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
# from compas_ghpython.artists import RobotModelArtist

_compas_fab_from = getattr(sys.modules["compas_fab"], "__file__", "") or ""
if not os.path.normcase(_compas_fab_from).startswith(os.path.normcase(_COMPAS_FAB_SRC)):
    raise RuntimeError(
        "compas_fab loaded from '{}', NOT the in-repo submodule at '{}'. Reset "
        "the Python 3 (CPython) engine (ScriptEditor -> Tools -> Reload Python 3 "
        "Engine) and re-run.".format(_compas_fab_from, _COMPAS_FAB_SRC)
    )
print("compas_fab loaded from: {}".format(_compas_fab_from))

# Robot meshes are usually in meters
# Set a high percision to prevent parsing errors
# and configure Rhino itself in meters
compas.PRECISION = '12f'

# Store robot in component-based key
# ghenv.Component.InstanceGuid
robot_key = 'robot_{}'.format(str('dual-arm_husky_Cindy'))

if robot_key not in st:
    st[robot_key] = None

loader = LocalPackageMeshLoader(os.path.join(pkg_path), pkg_name)

if load:
# if True:
    # Also load geometry
    husky_loader = LocalPackageMeshLoader(os.path.join(pkg_path), "husky_description")
    dualarm_husky_loader = LocalPackageMeshLoader(os.path.join(pkg_path), "husky_ur_description")
    ur_loader = LocalPackageMeshLoader(os.path.join(pkg_path), "ur_description")
    
    # Create robot model from URDF
    model = RobotModel.from_urdf_string(loader.load_urdf(urdf_file).read())
    model.load_geometry(loader, husky_loader, dualarm_husky_loader, ur_loader)
    print(model)

    st[robot_key] = model

robot_model = st[robot_key]
robot_semantics = None
if srdf_file:
    robot_semantics = RobotSemantics.from_srdf_file(loader.build_path('config', srdf_file), robot_model)