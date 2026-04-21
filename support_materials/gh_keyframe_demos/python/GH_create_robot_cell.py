
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RigidBodyState
from compas.geometry import Box
from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame

from compas_fab.robots import RobotCellLibrary
from compas_robots import ToolModel

# * This is how we create a robot cell in GH *

robot_cell = RobotCell(robot_model, robot_semantics)

robot_cell.tool_models['Alice'] = ToolModel.from_robot_model(support_robot_model.copy(), Frame.worldXY())
robot_cell.tool_models['Belle'] = ToolModel.from_robot_model(support_robot_model.copy(), Frame.worldXY())

robot_cell.tool_models['AL'] = assembly_tool.copy()
robot_cell.tool_models['AR'] = assembly_tool.copy()

#################
robot_cell.tool_models['SGAlice'] = support_tool.copy()
robot_cell.tool_models['SGBelle'] = support_tool.copy()

#################
robot_cell_state = robot_cell.default_cell_state()

robot_cell_state.tool_states['AL'].attached_to_group = 'base_left_arm_manipulator'
robot_cell_state.tool_states['AL'].touch_links = ['left_ur_arm_wrist_3_link']

robot_cell_state.tool_states['AR'].attached_to_group = 'base_right_arm_manipulator'
robot_cell_state.tool_states['AR'].touch_links = ['right_ur_arm_wrist_3_link']

# if (rigid_bodies):
#     # Make sure RBs have unique names
#     assert len(set([rb.name for rb in rigid_bodies])) == len(rigid_bodies)
#     for rb in rigid_bodies:
#         assert rb.name
#         robot_cell.rigid_body_models[rb.name] = rb
#         robot_cell_state.rigid_body_states[rb.name] = RigidBodyState(
#             plane_to_compas_frame(object_frame),
#             is_hidden=is_hidden,
#         )

from compas.scene import Scene
scene = Scene()
scene.add(robot_cell)