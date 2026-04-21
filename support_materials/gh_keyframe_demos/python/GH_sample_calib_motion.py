"""
Grasshopper Python component for sampling calibration motion.

This component creates a linear interpolation trajectory in joint space by:
1. Taking a starting cell state
2. Adding calib_joint_range to a nominated joint (0 or 1)
3. Linearly interpolating between start and goal configurations
4. Optionally checking collisions at each point
5. Returning the starting cell state and a JointTrajectory
"""

from compas_fab.backends import PyBulletPlanner
from compas_fab.backends import CollisionCheckError
from compas_fab.robots import RobotCellState
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import Duration


def sample_calib_motion_gh(
    planner,
    robot_cell_state,
    target_joint_index,
    calib_joint_range,
    steps=20,
    check_collision=True,
    group=None
):
    """
    Sample calibration motion by linear interpolation in joint space.
    
    Parameters
    ----------
    planner : PyBulletPlanner
        The planner instance
    robot_cell_state : RobotCellState
        The starting cell state containing the current joint configuration
    target_joint_index : int
        The joint index to calibrate (0 or 1)
    calib_joint_range : float
        The range to add to the target joint (often 2*pi)
    steps : int, optional
        Number of interpolation steps (default: 20)
    check_collision : bool, optional
        Whether to check collisions at each point (default: True)
    group : str, optional
        Planning group name. If None, uses the first configurable group from robot_cell
        
    Returns
    -------
    tuple
        (start_state, trajectory) where:
        - start_state: RobotCellState (copy of input state)
        - trajectory: JointTrajectory containing the interpolated path
        
    Raises
    ------
    ValueError
        If target_joint_index is not 0 or 1
    CollisionCheckError
        If check_collision is True and a collision is detected
    """
    
    # Validate target joint index
    if target_joint_index not in [0, 1]:
        raise ValueError(f"target_joint_index must be 0 or 1, got {target_joint_index}")

    # Get planning group
    if group is None:
        # Get the first configurable group from the robot cell
        robot_cell = planner.client.robot_cell
        configurable_groups = robot_cell.get_configurable_groups()
        if not configurable_groups:
            raise ValueError("No configurable groups found in robot_cell")
        group = configurable_groups[0]
        print(f"Using default group: {group}")
    else:
        print(f"Using specified group: {group}")
    
    # Get joint information from the planning group
    robot_cell = planner.client.robot_cell
    joint_names = robot_cell.get_configurable_joint_names(group)
    joint_types = robot_cell.get_configurable_joint_types(group)
    
    # Validate that we have at least target_joint_index + 1 joints
    if len(joint_names) <= target_joint_index:
        raise ValueError(
            f"Planning group '{group}' has only {len(joint_names)} joints, "
            f"but target_joint_index is {target_joint_index}"
        )
    
    print(f"Planning group '{group}' has {len(joint_names)} joints")
    print(f"Joint names: {joint_names}")
    print(f"Target joint index: {target_joint_index} ({joint_names[target_joint_index]})")
    
    # Create a copy of the starting state
    start_state = robot_cell_state.copy()
    
    # Check if the start state is in collision (if collision checking is enabled)
    if check_collision:
        try:
            planner.set_robot_cell_state(start_state)
            options = {"_skip_set_robot_cell_state": True}
            planner.check_collision(start_state, options)
            print("Start state is collision-free")
        except CollisionCheckError as e:
            error_msg = (
                f"The start state is in collision.\n"
                f"  Collision pairs: {e.collision_pairs if hasattr(e, 'collision_pairs') else 'N/A'}\n"
                f"  Error message: {e.message if hasattr(e, 'message') else str(e)}"
            )
            print(f"ERROR: {error_msg}")
            raise CollisionCheckError(error_msg) from e
    
    # Get current joint values from the robot cell state
    start_joint_values = [start_state.robot_configuration[name] for name in joint_names]
    print(f"Start joint values: {start_joint_values}")

    if target_joint_index == 1:
        if abs(start_joint_values[0]) > 1e-8:
            raise ValueError(
                f"target_joint_index==1, but starting configuration for joint '{joint_names[0]}' "
                f"is not zero (value: {start_joint_values[0]})"
            )
    
    # Create goal configuration by adding calib_joint_range to the target joint
    goal_joint_values = start_joint_values.copy()
    goal_joint_values[target_joint_index] += calib_joint_range
    print(f"Goal joint values: {goal_joint_values}")
    print(f"Joint {target_joint_index} range: {start_joint_values[target_joint_index]:.4f} -> {goal_joint_values[target_joint_index]:.4f} (delta: {calib_joint_range:.4f})")
    
    # Create the JointTrajectory
    trajectory = JointTrajectory(
        joint_names=joint_names,
        start_configuration=start_state.robot_configuration
    )
    
    # Add the start point to the trajectory
    trajectory.points.append(
        JointTrajectoryPoint(
            joint_values=start_joint_values,
            joint_types=joint_types,
            joint_names=joint_names,
            time_from_start=Duration(secs=0, nsecs=0)
        )
    )
    
    # Linearly interpolate between start and goal
    print(f"\nInterpolating {steps} steps between start and goal configurations...")
    
    for j in range(steps):
        # Calculate interpolation factor (0 to 1)
        alpha = (j + 1) / steps
        
        # Linearly interpolate joint values
        interpolated_joint_values = [
            start_val + alpha * (goal_val - start_val)
            for start_val, goal_val in zip(start_joint_values, goal_joint_values)
        ]
        
        # Create a temporary state for collision checking
        if check_collision:
            temp_state = start_state.copy()
            for joint_name, joint_value in zip(joint_names, interpolated_joint_values):
                temp_state.robot_configuration[joint_name] = joint_value
            
            # Check collision at this configuration
            try:
                # Set the robot cell state first
                planner.set_robot_cell_state(temp_state)
                # Then check collision with skip flag to avoid setting state again
                options = {"_skip_set_robot_cell_state": True}
                planner.check_collision(temp_state, options)
            except CollisionCheckError as e:
                error_msg = (
                    f"Collision detected at interpolation step {j+1}/{steps} "
                    f"(alpha={alpha:.4f})\n"
                    f"  Joint values: {interpolated_joint_values}\n"
                    f"  Collision pairs: {e.collision_pairs if hasattr(e, 'collision_pairs') else 'N/A'}\n"
                    f"  Error message: {e.message if hasattr(e, 'message') else str(e)}"
                )
                print(f"ERROR: {error_msg}")
                raise CollisionCheckError(error_msg) from e
        
        # Add the interpolated point to the trajectory
        trajectory.points.append(
            JointTrajectoryPoint(
                joint_values=interpolated_joint_values,
                joint_types=joint_types,
                joint_names=joint_names,
                time_from_start=Duration(secs=(j + 1) * 0.1, nsecs=0)  # 0.1 seconds per step
            )
        )
        
        if (j + 1) % max(1, steps // 5) == 0:  # Print progress every ~20%
            print(f"  Step {j+1}/{steps} completed (alpha={alpha:.4f})")
    
    print(f"\nSuccessfully created trajectory with {len(trajectory.points)} points")
    print(f"  Start: {start_joint_values}")
    print(f"  End: {interpolated_joint_values}")
    
    return start_state, trajectory


# ============================================================================
# Grasshopper component interface
# ============================================================================
# Expected inputs from Grasshopper:
# - planner: PyBulletPlanner instance
# - robot_cell_state: RobotCellState instance
# - target_joint_index: int (0 or 1)
# - calib_joint_range: float (often 2*pi)
# - steps: int (default: 20)
# - check_collision: bool (default: True)
# - group: str (optional, planning group name)
# - trigger: bool (to execute the component)

# Initialize outputs
start_state = None
trajectory = None
error_message = None

if trigger:
    try:
        # Validate inputs
        if planner is None:
            raise ValueError("planner input is required")
        if robot_cell_state is None:
            raise ValueError("robot_cell_state input is required")
        if target_joint_index is None:
            raise ValueError("target_joint_index input is required")
        if calib_joint_range is None:
            raise ValueError("calib_joint_range input is required")
        
        # Set defaults
        if steps is None:
            steps = 20
        if check_collision is None:
            check_collision = True
        
        # Call the main function
        start_state, trajectory = sample_calib_motion_gh(
            planner=planner,
            robot_cell_state=robot_cell_state,
            target_joint_index=target_joint_index,
            calib_joint_range=calib_joint_range,
            steps=steps,
            check_collision=check_collision,
            group=group
        )
        
        print("\n" + "="*60)
        print("CALIBRATION MOTION SAMPLING COMPLETED SUCCESSFULLY")
        print("="*60)
        print(f"Start state: {start_state}")
        print(f"Trajectory points: {len(trajectory.points)}")
        print(f"Joint names: {trajectory.joint_names}")
        
    except Exception as e:
        error_message = str(e)
        print(f"\nERROR in sample_calib_motion_gh: {error_message}")
        import traceback
        traceback.print_exc()
        start_state = None
        trajectory = None
