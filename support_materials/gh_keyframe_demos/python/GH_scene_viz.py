#sss vsenv: compas_fab_dev

import scriptcontext as sc
from compas.scene import SceneObject
from compas.scene import Scene
from compas_rhino.conversions import frame_to_rhino_plane
from compas.geometry import Frame, Transformation
from compas_fab.robots import TargetMode

from compas_ghpython import create_id

# This script will leak memory due to unused and untracked sceneobject in sticky

debug = True
def debug_print(s):
    if debug:
        print(s)

# Create hash (scope limited to this component) for cacheing SceneObject
# The Data.sha256 function includes the guid of the compas object
#object_hash = object.sha256(as_string=True)
# The ID function is much faster than the sha256()
object_hash = str(id(robot_cell))
options_hash = vis_options.sha256(True)
# The ghenv.Component has a guid assigned by GH to each component instance
gh_component = ghenv.Component
sticky_id = create_id(gh_component, object_hash +"_"+ options_hash+str(single_arm))
debug_print("Cacheing ID: '{}'".format(sticky_id))

# If scene object exist in sticky
scene_object = None
if sc.sticky.has_key(sticky_id):
    scene_object = sc.sticky[sticky_id]
    # Double check if the item is the same
    if scene_object.item is robot_cell:
        debug_print("SceneObject '{}' reused for '{}'".format(type(scene_object).__name__, type(object).__name__))

# Create Scene Object
if scene_object is None:
    scene = Scene()
    kwargs = vis_options.__data__
    scene_object = scene.add(robot_cell, **kwargs)
    debug_print("SceneObject {} newly created for {}".format(type(scene_object).__name__, type(object).__name__))
    sc.sticky[sticky_id] = scene_object

# print(scene_object._scale)
draw = scene_object.draw(robot_cell_state)
world_from_base = Transformation.from_frame(robot_cell_state.robot_base_frame)
if not single_arm:
    left_tool0_cp_frame = robot_cell.robot_model.forward_kinematics(robot_cell_state.robot_configuration, link_name='left_ur_arm_tool0')
    base_from_left_tool0_frame = Transformation.from_frame(left_tool0_cp_frame)    
    left_tool0_frame = frame_to_rhino_plane(Frame.from_transformation(world_from_base * base_from_left_tool0_frame))

    right_tool0_cp_frame = robot_cell.robot_model.forward_kinematics(robot_cell_state.robot_configuration, link_name='right_ur_arm_tool0')
    base_from_right_tool0_frame = Transformation.from_frame(right_tool0_cp_frame)
    right_tool0_frame = frame_to_rhino_plane(Frame.from_transformation(world_from_base * base_from_right_tool0_frame))

    print(left_tool0_cp_frame)
    print(right_tool0_cp_frame)
else:
    base_from_tool0_frame = Transformation.from_frame(
        robot_cell.robot_model.forward_kinematics(robot_cell_state.robot_configuration, link_name='ur_arm_tool0'))
    tool0_frame = frame_to_rhino_plane(Frame.from_transformation(world_from_base * base_from_tool0_frame))
