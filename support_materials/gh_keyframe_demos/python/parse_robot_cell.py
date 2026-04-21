import re
import os
import random
import time
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.robots import TargetMode
from compas_fab.backends import CollisionCheckError
from compas_fab.backends.exceptions import InverseKinematicsError
import pybullet_planning as pp

#####################################################
# HELPER FUNCTIONS for converting compas.geometry.Frame to pybullet_planning.pose and vice versa
#####################################################

from compas.geometry import Frame, Transformation
def pose_from_frame(frame, scale=1.0):
    return ([v*scale for v in frame.point], frame.quaternion.xyzw)

def frame_from_pose(pose, scale=1.0):
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=[v*scale for v in point])

def pose_from_transformation(tf, scale=1.0):
    frame = Frame.from_transformation(tf)
    return pose_from_frame(frame, scale)

def transformation_from_pose(pose, scale=1.0):
    frame = frame_from_pose(pose, scale)
    return Transformation.from_frame(frame)

#####################################################

HERE = os.path.dirname(os.path.abspath(__file__))
problem_name = '250806_RobotX_box_redo'

robot_cell = json_load(os.path.join(HERE, '..', '..', problem_name, 'RobotCell.json'))
state_file = 'robotx_box_A5-S4_end_RobotCellState.json'
# state_file = 'robotx_box_A6-S4_end_RobotCellState.json'

match = re.search(r'_A(\d+)-', state_file)
active_bar_name = f"b{match.group(1)}_0" if match else None

robot_cell_state = json_load(os.path.join(HERE, '..', '..', problem_name, 'RobotCellStates', state_file))

use_gui = True
client = PyBulletClient(connection_type="gui" if use_gui else "direct", verbose=True)
client.__enter__()

# make pp know the client id created by compas_fab client
pp.set_client(client.client_id)
pp.CLIENTS[client.client_id] = True if use_gui else None

planner = PyBulletPlanner(client)

tic = time.time()
with pp.LockRenderer(False):
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)
toc = time.time()
print(f"set_robot_cell and set_robot_cell_state took {toc - tic:.3f} seconds")
# wo lock: set_robot_cell and set_robot_cell_state took 29.937 seconds
# with lock: set_robot_cell and set_robot_cell_state took 24.557 seconds

# * Get pyb id for the target bar
target_bar = client.rigid_bodies_puids[active_bar_name][0]
print('target bar name', active_bar_name)
print('target bar id', target_bar)
pp.set_color(target_bar, pp.BLUE)

# * Get the attached tool for the left arm group (the two Victor Assembly Tools)
left_group = "base_left_arm_manipulator"
right_group = "base_right_arm_manipulator"

left_tool = robot_cell.get_attached_tool(robot_cell_state, left_group)
right_tool = robot_cell.get_attached_tool(robot_cell_state, right_group)

if left_tool:
    left_tool_name = left_tool.name
    if left_tool_name and left_tool_name in client.tools_puids:
        left_tool_uid = client.tools_puids[left_tool_name]
        color = [random.random() for _ in range(3)] + [1.0]
        pp.set_color(left_tool_uid, color)
        print(f"Colored left tool '{left_tool_name}' with uid {left_tool_uid}")
    else:
        print(f"Left tool for group '{left_group}' not found in client.tools_puids")
else:
    print(f"No tool attached to group '{left_group}'")

if right_tool:
    right_tool_name = right_tool.name
    if right_tool_name and right_tool_name in client.tools_puids:
        right_tool_uid = client.tools_puids[right_tool_name]
        color = [random.random() for _ in range(3)] + [1.0]
        pp.set_color(right_tool_uid, color)
        print(f"Colored right tool '{right_tool_name}' with uid {right_tool_uid}")
    else:
        print(f"Right tool for group '{right_group}' not found in client.tools_puids")
else:
    print(f"No tool attached to group '{right_group}'")

# TODO: some bugs with planner.forward_kinematics on non-zero robot_base_frame, FK to get tool frame is not working properly atm
# # Get the tool frame for both robots and draw their poses
# world_from_point_tool_left = planner.forward_kinematics(robot_cell_state, TargetMode.TOOL, group=left_group)
# print("Left arm tool TCF frame in World coordinate frame:")
# print(world_from_point_tool_left)
# pp.draw_pose(pose_from_frame(world_from_point_tool_left))

# # Get the tool frame for the right arm
# world_from_point_tool_right = planner.forward_kinematics(robot_cell_state, TargetMode.TOOL, group=right_group)
# print("Right arm tool TCF frame in World coordinate frame:")
# print(world_from_point_tool_right)
# pp.draw_pose(pose_from_frame(world_from_point_tool_right))

# * Get the two support robots (Alice and Belle) as ToolModel
print("\n=== Support Robots (Alice and Belle) ===")
support_robots = ['Alice', 'Belle']

for robot_name in support_robots:
    if robot_name in client.tools_puids:
        robot_uid = client.tools_puids[robot_name]
        print(f"\nSupport robot '{robot_name}' with uid {robot_uid}")
        
        # Get all links of the support robot
        all_links = pp.get_all_links(robot_uid)
        print(f"All links for {robot_name}: {all_links}")
        
        # Print link names and color links with "tool0" in their name
        for link_id in all_links:
            link_name = pp.get_link_name(robot_uid, link_id)
            print(f"  Link {link_id}: {link_name}")
            
            # Color links with "tool0" in their name
            if "tool0" in link_name.lower():
                color = [random.random() for _ in range(3)] + [1.0]
                pp.set_color(robot_uid, color, link=link_id)
                print(f"    Colored link '{link_name}'")
    else:
        print(f"Support robot '{robot_name}' not found in client.tools_puids")

# * Get the two support tools (SGAlice and SGBelle)
print("\n=== Support Tools (SGAlice and SGBelle) ===")
support_tools = ['SGAlice', 'SGBelle']

for tool_name in support_tools:
    if tool_name in client.tools_puids:
        tool_uid = client.tools_puids[tool_name]
        color = [random.random() for _ in range(3)] + [1.0]
        pp.set_color(tool_uid, color)
        print(f"Colored support tool '{tool_name}' with uid {tool_uid}")
    else:
        print(f"Support tool '{tool_name}' not found in client.tools_puids")

pp.wait_if_gui()