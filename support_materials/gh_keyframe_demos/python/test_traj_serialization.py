import os
import sys
import numpy as np
from compas_robots import Configuration
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas.data import json_dump, json_load
from compas_fab.robots import Duration
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIRECTORY = os.path.abspath(os.path.join(HERE, "..", ".."))

# ---------------------------
# Load Robot Cell and State
# ---------------------------

# Load robot cell
robot_cell_path = os.path.join(DATA_DIRECTORY, "250808_cindy_calibration_validation", "RobotCell.json")
robot_cell = json_load(robot_cell_path)
print(f"Loaded robot cell from: {robot_cell_path}")

# Load robot cell state
state_path = os.path.join(DATA_DIRECTORY, "250808_cindy_calibration_validation", "RobotCellStates", "validation_20250808_235435_RobotCellState.json")
robot_cell_state = json_load(state_path)
print(f"Loaded robot cell state from: {state_path}")

# Get robot configuration from state
robot_config = robot_cell_state.robot_configuration
print(f"Robot configuration: {robot_config}")

# Get joint names and values from the configuration
joint_names = robot_config.joint_names
start_joint_values = robot_config.joint_values
joint_types = robot_config.joint_types

print(f"Joint names: {joint_names}")
print(f"Start joint values: {start_joint_values}")
print(f"Joint types: {joint_types}")

# ---------------------------
# Sample random delta and interpolate
# ---------------------------

# Randomly sample a delta in joint space
np.random.seed(42)  # For reproducible results
delta = np.random.uniform(-0.5, 0.5, len(start_joint_values))
end_joint_values = [start + delta_val for start, delta_val in zip(start_joint_values, delta)]

print(f"Random delta: {delta}")
print(f"End joint values: {end_joint_values}")

# Linear interpolation to get intermediate points
num_points = 10
points = []

for i in range(num_points):
    t = i / (num_points - 1)  # Parameter from 0 to 1
    interpolated_values = [start + t * (end - start) for start, end in zip(start_joint_values, end_joint_values)]
    
    point = JointTrajectoryPoint(
        joint_values=interpolated_values,
        joint_types=joint_types,
        time_from_start=Duration(secs=i * 0.5, nsecs=0)  # 0.5 seconds between points, this can be random for now, will be overwritten later anyway
    )
    points.append(point)

# ---------------------------
# Create trajectory
# ---------------------------

trajectory = JointTrajectory(
    joint_names=joint_names,
    trajectory_points=points
)

print(f"Created trajectory with {len(points)} points")

# ---------------------------
# Save to JSON
# ---------------------------

json_file = "robot_trajectory.json"
json_dump(trajectory, json_file)
print(f"Trajectory saved to {json_file}")

# ---------------------------
# Load from JSON
# ---------------------------

loaded_trajectory = json_load(json_file)
print(f"Trajectory loaded from {json_file}")

# ---------------------------
# Verify loaded data
# ---------------------------

print(f"Loaded trajectory joint names: {loaded_trajectory.joint_names}")
for i, pt in enumerate(loaded_trajectory.points):
    print(f"Point {i}: values={pt.values}, time_from_start={pt.time_from_start}")

# ---------------------------
# Visualize in PyBullet
# ---------------------------

print("\nStarting PyBullet visualization...")

with PyBulletClient(connection_type="gui", enable_debug_gui=True, verbose=True) as client:
    # Set up the robot cell
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)
    
    print("Robot cell loaded in PyBullet")
    input("Press Enter to start trajectory visualization...")
    
    # Visualize trajectory point by point
    for i, point in enumerate(loaded_trajectory.points):
        print(f"Moving to trajectory point {i+1}/{len(loaded_trajectory.points)}")
        print(f"  Joint values: {point.joint_values}")
        
        # Set robot configuration
        config = Configuration(
            joint_values=point.joint_values,
            joint_types=point.joint_types,
            joint_names=loaded_trajectory.joint_names
        )
        planner.client._set_robot_configuration(config)
        
        # Wait a bit to see the movement
        time.sleep(1)
    
    print("Trajectory visualization complete!")
    print("Press Enter to exit...")
    input()
