import math
import os
from compas.data import json_load
from compas.geometry import Frame, Transformation
from compas.geometry import angle_vectors
from compas_fab.robots import TargetMode

# Joint names for dual arm Husky robot
HUSKY_DUAL_ARM_JOINT_NAMES_LEFT = [
    "left_ur_arm_shoulder_pan_joint", 
    "left_ur_arm_shoulder_lift_joint", 
    "left_ur_arm_elbow_joint", 
    "left_ur_arm_wrist_1_joint", 
    "left_ur_arm_wrist_2_joint", 
    "left_ur_arm_wrist_3_joint"
]

HUSKY_DUAL_ARM_JOINT_NAMES_RIGHT = [
    "right_ur_arm_shoulder_pan_joint", 
    "right_ur_arm_shoulder_lift_joint", 
    "right_ur_arm_elbow_joint", 
    "right_ur_arm_wrist_1_joint", 
    "right_ur_arm_wrist_2_joint", 
    "right_ur_arm_wrist_3_joint"
]

def load_and_validate_trajectory(trajectory_path, planner, robot_cell_state, groups, tolerance=0.001):
    """
    Load a trajectory and validate that all points maintain consistent tool0-to-tool0 transformation.
    
    Args:
        trajectory_path (str): Path to the trajectory JSON file
        planner: Robot planner instance with forward kinematics capabilities
        groups: List containing [left_arm_group, right_arm_group]
        tolerance (float): Tolerance for position and orientation errors
    
    Returns:
        dict: Validation results
    """
    
    # Step 1: Load trajectory from given path
    print(f"Loading trajectory from: {trajectory_path}")
    trajectory = json_load(trajectory_path)
    print(f"Loaded trajectory with {len(trajectory.points)} points")
    
    # # Step 2: Set joint names if not provided (check at trajectory point level)
    # if not hasattr(trajectory.points[0], 'joint_names') or not trajectory.points[0].joint_names:
    #     # Set joint names for all trajectory points
    #     for point in trajectory.points:
    #         point.joint_names = HUSKY_DUAL_ARM_JOINT_NAMES_LEFT + HUSKY_DUAL_ARM_JOINT_NAMES_RIGHT
    #     print("Set joint names using predefined constants for all trajectory points")
    # else:
    #     print(f"Using existing joint names: {trajectory.points[0].joint_names}")
    
    # Get robot cell and joint names for each arm
    robot_cell = planner.client.robot_cell
    left_arm_group = groups[0]
    right_arm_group = groups[1]
    
    left_joint_names = robot_cell.get_configurable_joint_names(left_arm_group)
    right_joint_names = robot_cell.get_configurable_joint_names(right_arm_group)

    # INSERT_YOUR_CODE
    # Check that left_joint_names and right_joint_names match the expected HUSKY joint name lists
    if left_joint_names != HUSKY_DUAL_ARM_JOINT_NAMES_LEFT:
        raise ValueError(f"left_joint_names do not match HUSKY_DUAL_ARM_JOINT_NAMES_LEFT.\n"
                         f"left_joint_names: {left_joint_names}\n"
                         f"Expected: {HUSKY_DUAL_ARM_JOINT_NAMES_LEFT}")
    if right_joint_names != HUSKY_DUAL_ARM_JOINT_NAMES_RIGHT:
        raise ValueError(f"right_joint_names do not match HUSKY_DUAL_ARM_JOINT_NAMES_RIGHT.\n"
                         f"right_joint_names: {right_joint_names}\n"
                         f"Expected: {HUSKY_DUAL_ARM_JOINT_NAMES_RIGHT}")
    
    # Step 3: Compute expected transformation from first trajectory point
    print("\nComputing expected tool0-to-tool0 transformation from first trajectory point...")
    
    # Get first trajectory point
    first_point = trajectory.points[0]
    # Give first_point joint_names if it doesn't have
    if not hasattr(first_point, 'joint_names') or not first_point.joint_names:
        first_point.joint_names = HUSKY_DUAL_ARM_JOINT_NAMES_LEFT + HUSKY_DUAL_ARM_JOINT_NAMES_RIGHT
    
    # Extract joint values for each arm from the first point
    left_joint_values = [first_point[joint_name] for joint_name in left_joint_names]
    right_joint_values = [first_point[joint_name] for joint_name in right_joint_names]
    
    # Create temporary robot cell state for forward kinematics
    temp_state = robot_cell_state.copy()
    
    # Set joint values for each arm
    for j, joint_name in enumerate(left_joint_names):
        temp_state.robot_configuration[joint_name] = left_joint_values[j]
    for j, joint_name in enumerate(right_joint_names):
        temp_state.robot_configuration[joint_name] = right_joint_values[j]
    
    # Set robot state and get forward kinematics for first point
    planner.set_robot_cell_state(temp_state)
    
    # Get forward kinematics for both arms' tool0 frames from first point
    left_tool0_frame = planner.forward_kinematics(temp_state, TargetMode.ROBOT, group=left_arm_group)
    right_tool0_frame = planner.forward_kinematics(temp_state, TargetMode.ROBOT, group=right_arm_group)
    
    # Calculate expected transformation between arms (left_tool0_from_right_tool0)
    world_from_left_tool0 = Transformation.from_frame(left_tool0_frame)
    world_from_right_tool0 = Transformation.from_frame(right_tool0_frame)
    expected_left_tool0_from_right_tool0 = world_from_left_tool0.inverse() * world_from_right_tool0
    
    print(f"Expected left_tool0_from_right_tool0 transformation computed from first point")
    
    # Step 4: Check all trajectory points maintain the same transformation
    print("\n" + "="*60)
    print("TRAJECTORY VALIDATION - TOOL0 TO TOOL0 TRANSFORMATION CONSISTENCY")
    print("="*60)
    
    verification_results = {
        "success": True,
        "max_position_error": 0.0,
        "max_orientation_error": 0.0,
        "failed_points": [],
        "details": [],
        "expected_transformation": expected_left_tool0_from_right_tool0
    }
    
    print("Verifying tool0-to-tool0 transformation consistency across all trajectory points...")
    
    for i, point in enumerate(trajectory.points):
        if not hasattr(point, 'joint_names') or not point.joint_names:
            point.joint_names = HUSKY_DUAL_ARM_JOINT_NAMES_LEFT + HUSKY_DUAL_ARM_JOINT_NAMES_RIGHT

        # Extract joint values for each arm from the current trajectory point
        left_joint_values = [point[joint_name] for joint_name in left_joint_names]
        right_joint_values = [point[joint_name] for joint_name in right_joint_names]
        
        # Create temporary robot cell state for forward kinematics
        temp_state = robot_cell_state.copy()
        
        # Set joint values for each arm
        for j, joint_name in enumerate(left_joint_names):
            temp_state.robot_configuration[joint_name] = left_joint_values[j]
        for j, joint_name in enumerate(right_joint_names):
            temp_state.robot_configuration[joint_name] = right_joint_values[j]
        
        # Set robot state and get forward kinematics
        planner.set_robot_cell_state(temp_state)
        
        # Get forward kinematics for both arms' tool0 frames
        left_tool0_frame = planner.forward_kinematics(temp_state, TargetMode.ROBOT, group=left_arm_group)
        right_tool0_frame = planner.forward_kinematics(temp_state, TargetMode.ROBOT, group=right_arm_group)
        
        # Calculate actual transformation between arms
        world_from_left_tool0 = Transformation.from_frame(left_tool0_frame)
        world_from_right_tool0 = Transformation.from_frame(right_tool0_frame)
        actual_left_tool0_from_right_tool0 = world_from_left_tool0.inverse() * world_from_right_tool0
        
        # Compare with expected transformation
        position_error = (actual_left_tool0_from_right_tool0.translation_vector - 
                         expected_left_tool0_from_right_tool0.translation_vector).length
        
        # Calculate orientation error by comparing axis vectors
        # Convert transformations to frames to get axis vectors directly
        actual_frame = Frame.from_transformation(actual_left_tool0_from_right_tool0)
        expected_frame = Frame.from_transformation(expected_left_tool0_from_right_tool0)
        
        # Get axis vectors directly from frames
        actual_x_axis = actual_frame.xaxis
        actual_y_axis = actual_frame.yaxis
        actual_z_axis = actual_frame.zaxis
        
        expected_x_axis = expected_frame.xaxis
        expected_y_axis = expected_frame.yaxis
        expected_z_axis = expected_frame.zaxis
        
        # Calculate angles between corresponding axis vectors
        x_axis_error = angle_vectors(actual_x_axis, expected_x_axis)
        y_axis_error = angle_vectors(actual_y_axis, expected_y_axis)
        z_axis_error = angle_vectors(actual_z_axis, expected_z_axis)
        
        # Use the maximum axis error as the overall orientation error
        orientation_error = max(x_axis_error, y_axis_error, z_axis_error)
        
        # Update max errors
        verification_results["max_position_error"] = max(verification_results["max_position_error"], position_error)
        verification_results["max_orientation_error"] = max(verification_results["max_orientation_error"], orientation_error)
        
        # Check if this point fails verification
        if position_error > tolerance or orientation_error > tolerance:
            print(f'Point #{i}: position error {position_error:.6f} m, orientation error {orientation_error:.6f} rad')
            verification_results["success"] = False
            verification_results["failed_points"].append(i)
            verification_results["details"].append({
                "point": i,
                "position_error": position_error,
                "orientation_error": orientation_error,
                "left_tool0_frame": left_tool0_frame,
                "right_tool0_frame": right_tool0_frame
            })
    
    # Print verification results
    print("\n" + "="*60)
    if verification_results["success"]:
        print("✓ Trajectory validation PASSED")
        print(f"  All {len(trajectory.points)} points maintain consistent tool0-to-tool0 transformation")
        print(f"  Max position error: {verification_results['max_position_error']:.6f} m")
        print(f"  Max orientation error: {verification_results['max_orientation_error']:.6f} rad")
    else:
        print("✗ Trajectory validation FAILED")
        print(f"  Failed points: {len(verification_results['failed_points'])}/{len(trajectory.points)}")
        print(f"  Max position error: {verification_results['max_position_error']:.6f} m")
        print(f"  Max orientation error: {verification_results['max_orientation_error']:.6f} rad")
        print(f"  Failed at points: {verification_results['failed_points']}")
    print("="*60)
    
    return verification_results

import os
trajectory_path = os.path.join(path, file_name)
result = load_and_validate_trajectory(trajectory_path, planner, robot_cell_state, groups, tolerance=0.001)