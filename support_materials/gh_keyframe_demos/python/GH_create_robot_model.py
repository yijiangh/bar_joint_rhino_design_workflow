from scriptcontext import sticky as st
import os
import compas

from compas_robots import RobotModel
from compas_robots.resources import GithubPackageMeshLoader
from compas_robots.resources import LocalPackageMeshLoader
from compas_robots.ghpython.scene import RobotModelObject
# from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
# from compas_ghpython.artists import RobotModelArtist

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