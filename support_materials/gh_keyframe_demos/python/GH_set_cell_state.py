import pybullet_planning as pp
from compas.geometry import Frame, Transformation

def pose_from_frame(frame, scale=1.0):
    return ([v*scale for v in frame.point], frame.quaternion.xyzw)

def frame_from_pose(pose, scale=1.0):
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=[v*scale for v in point])

planner.set_robot_cell_state(robot_cell_state)
robot_base_frame = robot_cell_state.robot_base_frame
robot_base_pose = pose_from_frame(robot_base_frame)
pp.set_pose(planner.client.robot_puid, robot_base_pose)