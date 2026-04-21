from compas.geometry import Frame

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode
from compas_rhino.conversions import plane_to_compas_frame
import pybullet_planning as pp
import pybullet as p

from compas.geometry import Frame, Transformation

def pose_from_frame(frame, scale=1.0):
    return ([v*scale for v in frame.point], frame.quaternion.xyzw)

def frame_from_pose(pose, scale=1.0):
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=[v*scale for v in point])

targets = []
for tp in target_planes:    
    target = FrameTarget(plane_to_compas_frame(tp), 
                         TargetMode.ROBOT,
                         tolerance_position=tolerance_position,
                         tolerance_orientation=tolerance_orientation
                         )
    targets.append(target)
    pp.draw_pose(pose_from_frame(target.target_frame))

new_robot_cell_state = None
        
if trigger:
    print(planner.client._get_pose_joint_names_and_puids())

    options = {
        "max_results": 20, 
        "check_collision": check_collision,
        "max_descend_iterations" : max_descend_iterations,
        "verbose" : False,
        }

    new_robot_cell_state = robot_cell_state.copy()
    for target, group in zip(targets, groups):
        print(f'ik for {group}')
        try:
        # if 1:
            config = planner.inverse_kinematics(
                target, 
                new_robot_cell_state, 
                group=group, 
                options=options
            )
        except Exception as e:
            print(f"IK failed for {group}: {e}")
            config = None

        if config is not None:
                # To verify the IK result, we can compute the FK with the obtained joint positions
            print(config)
            new_robot_cell_state.robot_configuration.merge(config)

            # # Only update the joints in this group
            # group_joint_names = planner.client.robot_cell.get_configurable_joint_names(group)
            # for joint_name, joint_value in zip(config.joint_names, config.joint_values):
            #     if joint_name in group_joint_names:
            #         new_robot_cell_state.robot_configuration[joint_name] = joint_value

if new_robot_cell_state:
    planner.client._set_robot_configuration(new_robot_cell_state.robot_configuration)

print("Found configuration", config)
# result_count += 1
# print("Found %d configurations" % result_count)

