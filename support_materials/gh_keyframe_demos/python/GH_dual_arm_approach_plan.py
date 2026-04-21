from copy import deepcopy
from math import ceil
from math import pi

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Quaternion
from compas.geometry import Transformation
from compas.geometry import Vector
from compas.geometry import axis_angle_from_quaternion
from compas.geometry import cross_vectors
from compas.geometry import is_parallel_vector_vector
from compas.geometry import angle_vectors
from compas_robots import Configuration

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.backends import CollisionCheckError
from compas_fab.backends import InverseKinematicsError
from compas_fab.backends import MPInterpolationInCollisionError
from compas_fab.backends import MPMaxJumpError
from compas_fab.backends import MPNoIKSolutionError
from compas_fab.backends import MPNoPlanFoundError
from compas_fab.backends import MPStartStateInCollisionError
from compas_fab.backends import MPTargetInCollisionError
from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.robots import FrameTarget
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode
from compas_fab.robots import Waypoints

from compas_fab.backends.pybullet.backend_features.helpers import check_max_jump
from compas_fab.backends.pybullet.backend_features.pybullet_plan_cartesian_motion import FrameInterpolator

from compas_rhino.conversions import frame_to_rhino_plane

def plan_dual_arm_cartesian_motion(planner, start_state, options, 
                                    start_world_from_bar, goal_world_from_bar, 
                                    bar_from_left_tool0, bar_from_right_tool0,
                                    groups):
    """
    Plan synchronized dual arm Cartesian motion.
    
    Parameters:
    -----------
    planner : PyBulletPlanner
        The planner instance
    start_state : RobotCellState
        The starting state of the robot
    options : dict
        Planning options
    start_world_from_bar : Frame
        Starting pose of the bar in world coordinates
    goal_world_from_bar : Frame
        Goal pose of the bar in world coordinates
    bar_from_left_tool0 : Transformation
        Transformation from bar frame to left arm tool0 frame
    bar_from_right_tool0 : Transformation
        Transformation from bar frame to right arm tool0 frame
    groups : list
        List containing two planning group names: [base_left_arm_manipulator, base_right_arm_manipulator]
        
    Returns:
    --------
    dict : Dictionary containing waypoints and trajectories for both arms
    """
    
    robot_cell: RobotCell = planner.client.robot_cell
    
    # Extract left and right arm group names from the groups list
    left_arm_group = groups[0]  # base_left_arm_manipulator
    right_arm_group = groups[1]  # base_right_arm_manipulator

    # Setting robot cell state
    planner.set_robot_cell_state(start_state)

    # Check start state input for collision if requested, it will throw an exception if the robot is in collision
    if options.get("check_collision"):
        # This allows options such as `full_report` and `verbose` to pass through to the check_collision method
        options.update({"_skip_set_robot_cell_state": True})

        # Check if the start state is in collision
        try:
            # Note: This is using the CheckCollision Backend Feature
            planner.check_collision(start_state, options)
        except CollisionCheckError as e:
            message = (
                "plan_dual_arm_cartesian_motion: The start_state for plan_cartesian_motion is in collision. \n  - "
                + e.message
            )
            raise MPStartStateInCollisionError(message, start_state=start_state, collision_pairs=e.collision_pairs)

    ## Checking the attached tool and workpiece for collision at every target
    # if options.get("check_collision") and not options.get("skip_preplanning_collision_check"):
    #     intermediate_state = deepcopy(start_state)
    #     intermediate_state.robot_configuration = None
    #     # Convert the targets to PCFs for collision checking
    #     pcf_frames = client.robot_cell.target_frames_to_pcf(
    #         start_state, waypoints.target_frames, waypoints.target_mode, group
    #     )

    #     for pcf_frame in pcf_frames:
    #         try:
    #             planner.check_collision_for_attached_objects_in_planning_group(
    #                 intermediate_state,
    #                 group,
    #                 pcf_frame,
    #                 options,
    #             )
    #         except CollisionCheckError as e:
    #             message = (
    #                 "plan_cartesian_motion_frame_waypoints: The target frame for plan_cartesian_motion is in collision. \n  - "
    #                 + e.message
    #             )
    #             raise MPTargetInCollisionError(message, target=pcf_frame, collision_pairs=e.collision_pairs)

    # Options for Inverse Kinematics
    ik_options = deepcopy(options)
    # NOTE: PyBullet IK is gradient based and will snap to the nearest configuration of the start state
    # NOTE: We are running this IK solver without random search (by setting max_results = 1) to ensure determinism
    #       In this mode, planner.inverse_kinematics() will raise an InverseKinematicsError if no IK solution is found,
    #       or CollisionCheckError if the configuration is found but in collision. We will handle these errors below.
    ik_options["max_results"] = 1
    # NOTE: # We only need the joint values for the group to construct the JointTrajectoryPoint
    ik_options["return_full_configuration"] = False
    # NOTE: The PyBullet IK function is responsible for performing collision checking for this planning function
    # The ["check_collision"] in options is passed also to the ik_options

    # Echo target_mode
    if options["verbose"]:
        print("Target Mode is: {}. Interpolation will happen with this reference".format("TOOL"))

    # target = GraspTarget(
    #     "male_joint",
    #     world_from_bar=tf_world_from_bar,
    #     bar_from_male_joint=tf_bar_from_male_joint,
    #     male_joint_from_tool0=male_joint_from_tool0,
    #     world_from_tool0=tf_world_from_male_joint * male_joint_from_tool0
    # )
    # !* I should use start and goal world_from_bar pose and the bar_from_bar_from_male_joint, and male_joint_from_tool0 to create the target waypoints

    # Create synchronized dual arm waypoint constructs
    # The key insight: interpolate the bar motion first, then derive arm targets
    # This ensures both arms maintain fixed relative transformation throughout the trajectory

    # Assuming we have:
    # - start_world_from_bar: Frame representing the starting pose of the bar in world coordinates
    # - goal_world_from_bar: Frame representing the goal pose of the bar in world coordinates  
    # - bar_from_left_tool0: Transformation from bar frame to left arm tool0 frame
    # - bar_from_right_tool0: Transformation from bar frame to right arm tool0 frame

    # Step 1: Convert transformations to frames for interpolation
    # FrameInterpolator requires Frame objects, not Transformation objects
    start_world_from_bar_frame = Frame.from_transformation(start_world_from_bar)
    goal_world_from_bar_frame = Frame.from_transformation(goal_world_from_bar)
    
    # Create bar interpolation object for consistent motion
    bar_interpolator = FrameInterpolator(start_world_from_bar_frame, goal_world_from_bar_frame, options)

    # Step 2: Generate interpolated bar waypoints
    # Calculate number of waypoints based on distance and angle, similar to original implementation
    num_bar_waypoints = max(2, bar_interpolator.regular_interpolation_steps + 1)
    bar_waypoints = []
    for i in range(num_bar_waypoints):
        t = i / (num_bar_waypoints - 1) if num_bar_waypoints > 1 else 0
        interpolated_bar_frame = bar_interpolator.get_interpolated_frame(t)
        bar_waypoints.append(interpolated_bar_frame)

    # Step 3: For each bar waypoint, compute corresponding arm targets
    left_arm_frames = []
    right_arm_frames = []

    for bar_frame in bar_waypoints:
        world_from_bar_frame = Transformation.from_frame(bar_frame)

        # Left arm target: apply bar_from_left_tool0 transformation to bar_frame
        # Use the transformed() method to apply the transformation
        left_arm_tf = world_from_bar_frame * bar_from_left_tool0
        left_arm_frames.append(Frame.from_transformation(left_arm_tf))
        
        # Right arm target: apply bar_from_right_tool0 transformation to bar_frame
        # Use the transformed() method to apply the transformation
        right_arm_tf = world_from_bar_frame * bar_from_right_tool0
        right_arm_frames.append(Frame.from_transformation(right_arm_tf))

    # Step 4: Create synchronized waypoint objects
    left_arm_waypoints = FrameWaypoints(
        target_frames=left_arm_frames,
        target_mode=TargetMode.ROBOT,
        name="Left Arm Synchronized Waypoints",
        tolerance_position=options.get("tolerance_position", 0.001),
        tolerance_orientation=options.get("tolerance_orientation", 0.01)
    )

    right_arm_waypoints = FrameWaypoints(
        target_frames=right_arm_frames,
        target_mode=TargetMode.ROBOT,
        name="Right Arm Synchronized Waypoints",
        tolerance_position=options.get("tolerance_position", 0.001),
        tolerance_orientation=options.get("tolerance_orientation", 0.01)
    )

    # Store both waypoint sets for later use
    dual_arm_waypoints = {
        "left_arm": left_arm_waypoints,
        "right_arm": right_arm_waypoints
    }

    # Store bar waypoints for reference
    bar_waypoints_obj = FrameWaypoints(
        target_frames=bar_waypoints,
        target_mode=TargetMode.WORKPIECE,  # Bar is the workpiece
        name="Bar Waypoints"
    )
    
    # Create visualization objects for debugging - bar frames and left/right arm tool0 frames
    # if options.get("verbose", False):
    if True:
        print("Creating visualization objects for bar frames and left/right arm tool0 frames...")
        
        # Convert bar frames to Rhino planes
        bar_planes = []
        for frame in bar_waypoints:
            plane = frame_to_rhino_plane(frame)
            bar_planes.append(plane)
        
        # Convert left arm frames to Rhino planes
        left_arm_planes = []
        for frame in left_arm_frames:
            plane = frame_to_rhino_plane(frame)
            left_arm_planes.append(plane)
        
        # Convert right arm frames to Rhino planes
        right_arm_planes = []
        for frame in right_arm_frames:
            plane = frame_to_rhino_plane(frame)
            right_arm_planes.append(plane)
        
        visualization_objects = {
            "bar_planes": bar_planes,
            "left_arm_planes": left_arm_planes,
            "right_arm_planes": right_arm_planes
        }
        
        print(f"Created visualization objects:")
        print(f"  Bar planes: {len(bar_planes)}")
        print(f"  Left arm planes: {len(left_arm_planes)}")
        print(f"  Right arm planes: {len(right_arm_planes)}")
    
    # Store visualization objects in the result for output
    dual_arm_waypoints["visualization"] = visualization_objects

    # DUAL ARM SYNCHRONIZED INTERPOLATION
    # Instead of interpolating between arm waypoints, we use the pre-computed synchronized waypoints
    # This ensures both arms maintain fixed relative transformation throughout the trajectory

    # Get the synchronized waypoints for both arms
    left_arm_waypoints = dual_arm_waypoints["left_arm"]
    right_arm_waypoints = dual_arm_waypoints["right_arm"]

    # Verify we have the same number of waypoints for both arms
    assert len(left_arm_waypoints.target_frames) == len(right_arm_waypoints.target_frames), \
        "Left and right arm waypoints must have the same number of frames for synchronization"

    num_synchronized_waypoints = len(left_arm_waypoints.target_frames)

    if options["verbose"]:
        print(f"Using {num_synchronized_waypoints} synchronized waypoints for dual arm motion")
        print(f"Bar start frame: {start_world_from_bar}")
        print(f"Bar goal frame: {goal_world_from_bar}")

    # Initialize trajectories for both arms using input group names
    left_arm_trajectory = JointTrajectory(
        joint_names=robot_cell.get_configurable_joint_names(left_arm_group), 
        start_configuration=start_state.robot_configuration
    )
    right_arm_trajectory = JointTrajectory(
        joint_names=robot_cell.get_configurable_joint_names(right_arm_group), 
        start_configuration=start_state.robot_configuration
    )

    # Add start configurations
    left_joint_names = robot_cell.get_configurable_joint_names(left_arm_group)
    left_joint_types = robot_cell.get_configurable_joint_types(left_arm_group)
    right_joint_names = robot_cell.get_configurable_joint_names(right_arm_group) 
    right_joint_types = robot_cell.get_configurable_joint_types(right_arm_group)

    # Get start joint values for each arm
    left_start_joints = [start_state.robot_configuration[name] for name in left_joint_names]
    right_start_joints = [start_state.robot_configuration[name] for name in right_joint_names]

    left_arm_trajectory.points.append(
        JointTrajectoryPoint(joint_values=left_start_joints, joint_types=left_joint_types, joint_names=left_joint_names)
    )
    right_arm_trajectory.points.append(
        JointTrajectoryPoint(joint_values=right_start_joints, joint_types=right_joint_types, joint_names=right_joint_names)
    )

    # Process each synchronized waypoint
    for i in range(num_synchronized_waypoints):
        if options["verbose"]:
            print(f"Processing synchronized waypoint {i+1}/{num_synchronized_waypoints}")
        
        # Get the current synchronized frames for both arms
        left_target_frame = left_arm_waypoints.target_frames[i]
        right_target_frame = right_arm_waypoints.target_frames[i]
        
        # Create targets for both arms
        left_target = FrameTarget(
            left_target_frame,
            target_mode=TargetMode.ROBOT,
            tolerance_position=left_arm_waypoints.tolerance_position or 0.001,
            tolerance_orientation=left_arm_waypoints.tolerance_orientation or 0.01
        )
        
        right_target = FrameTarget(
            right_target_frame,
            target_mode=TargetMode.ROBOT,
            tolerance_position=right_arm_waypoints.tolerance_position or 0.001,
            tolerance_orientation=right_arm_waypoints.tolerance_orientation or 0.01
        )
    
        # Solve IK for both arms simultaneously with error handling for debugging
        try:
            # Solve IK for left arm
            # TODO this might cause collision if the start right arm state is not good
            # can ignore this collision for this check?
            left_configuration = planner.inverse_kinematics(
                left_target, start_state, left_arm_group, ik_options
            )
            
            # Solve IK for right arm  
            right_configuration = planner.inverse_kinematics(
                right_target, start_state, right_arm_group, ik_options
            )
            
            # Extract joint values
            left_joint_values = [left_configuration[name] for name in left_joint_names]
            right_joint_values = [right_configuration[name] for name in right_joint_names]
            
            if options["verbose"]:
                print(f"Segment {i+1}, Left arm joint_values: {left_joint_values}")
                print(f"Segment {i+1}, Right arm joint_values: {right_joint_values}")
            
        # Catch errors and return None trajectories for debugging instead of raising
        except InverseKinematicsError as e:
            message = f"plan_dual_arm_cartesian_motion(): Segment {i+1}, Inverse Kinematics failed for dual arm motion.\n"
            message = message + e.message
            print(f"WARNING: {message}")
            print("Returning None trajectories for debugging - check frame visualization")
            
            # Return None trajectories with visualization data for debugging
            dual_arm_waypoints["left_arm_trajectory"] = None
            dual_arm_waypoints["right_arm_trajectory"] = None
            dual_arm_waypoints["error_message"] = message
            dual_arm_waypoints["error_segment"] = i + 1
            return dual_arm_waypoints

        except CollisionCheckError as e:
            message = f"plan_dual_arm_cartesian_motion(): Segment {i+1}, Collision detected for dual arm motion.\n"
            message = message + e.message
            print(f"WARNING: {message}")
            print("Returning None trajectories for debugging - check frame visualization")
            
            # Return None trajectories with visualization data for debugging
            dual_arm_waypoints["left_arm_trajectory"] = None
            dual_arm_waypoints["right_arm_trajectory"] = None
            dual_arm_waypoints["error_message"] = message
            dual_arm_waypoints["error_segment"] = i + 1
            return dual_arm_waypoints
        
        # Check joint jumps between the current and previous point's configuration for both arms
        if i > 0:  # Skip for first waypoint
            try:
                # Check left arm joint jump
                check_max_jump(
                    left_joint_names, left_joint_types, 
                    left_arm_trajectory.points[-1].joint_values, left_joint_values, options
                )
                
                # Check right arm joint jump
                check_max_jump(
                    right_joint_names, right_joint_types, 
                    right_arm_trajectory.points[-1].joint_values, right_joint_values, options
                )
                
            except MPMaxJumpError as e:
                # For dual arm, we can't easily subdivide like single arm, so we return None trajectories for debugging
                message = f"plan_dual_arm_cartesian_motion(): Segment {i+1}, Joint jump too large for dual arm motion.\n"
                message += f"Left arm jump: {e.message}\n"
                message += "Cannot subdivide dual arm motion - consider reducing step size or adjusting trajectory."
                print(f"WARNING: {message}")
                print("Returning None trajectories for debugging - check frame visualization")
                
                # Return None trajectories with visualization data for debugging
                dual_arm_waypoints["left_arm_trajectory"] = None
                dual_arm_waypoints["right_arm_trajectory"] = None
                dual_arm_waypoints["error_message"] = message
                dual_arm_waypoints["error_segment"] = i + 1
                return dual_arm_waypoints
        
        # This point is successful, add the JointTrajectoryPoint to both trajectories
        left_arm_trajectory.points.append(
            JointTrajectoryPoint(
                joint_values=left_joint_values, 
                joint_types=left_joint_types, 
                joint_names=left_joint_names
            )
        )
        
        right_arm_trajectory.points.append(
            JointTrajectoryPoint(
                joint_values=right_joint_values, 
                joint_types=right_joint_types, 
                joint_names=right_joint_names
            )
        )
        
        if options["verbose"]:
            print(f"Segment {i+1} of {num_synchronized_waypoints} completed - JointTrajectoryPoint added for both arms")

    # Store both trajectories in the dual_arm_waypoints for return
    dual_arm_waypoints["left_arm_trajectory"] = left_arm_trajectory
    dual_arm_waypoints["right_arm_trajectory"] = right_arm_trajectory

    if options["verbose"]:
        print(f"Dual arm synchronized motion planning completed:")
        print(f"  Left arm: {len(left_arm_trajectory.points)} trajectory points")
        print(f"  Right arm: {len(right_arm_trajectory.points)} trajectory points")

    # Merge both arm trajectories into one combined trajectory
    if left_arm_trajectory is not None and right_arm_trajectory is not None:
        print("Merging left and right arm trajectories into combined trajectory...")
        
        # Start with left arm trajectory as base
        new_trajectory = left_arm_trajectory
        
        # Get joint names for the right arm group
        group_joint_names = planner.client.robot_cell.get_configurable_joint_names(groups[1])
        print(group_joint_names)
        
        new_robot_cell_state = start_state.copy()
        # Merge each point from right arm trajectory
        for i, point in enumerate(right_arm_trajectory.points):
            new_trajectory.points[i] = new_robot_cell_state.robot_configuration.merged(new_trajectory.points[i])
            new_trajectory.points[i].merge(point)
        
        # Store the merged trajectory
        dual_arm_waypoints["combined_trajectory"] = new_trajectory
        
        if options["verbose"]:
            print(f"  Combined trajectory: {len(new_trajectory.points)} trajectory points")
            print(f"  Joint names: {new_trajectory.joint_names}")

    # Return the dual arm waypoints object containing both trajectories
    return dual_arm_waypoints

options = {
    "verbose": False,
    "check_collision": check_collision,
    "tolerance_position": tolerance_position,
    "tolerance_orientation": tolerance_orientation,
    "max_step_distance": 0.01,  # meters
    "max_step_angle": 0.1,  # radians
    "max_jump_prismatic": 0.1,
    "max_jump_revolute": pi / 2
}

# Call the function with the input parameters
# Note: These variables should be provided as inputs to the script from Grasshopper
# The following variables are expected to be defined in the Grasshopper environment:
# - planner: PyBulletPlanner instance
# - start_state: RobotCellState instance
# - start_world_from_bar: Transformation (starting bar pose in world coordinates)
# - goal_world_from_bar: Transformation (goal bar pose in world coordinates)
# - bar_from_left_tool0: Transformation (left arm tool0 offset from bar)
# - bar_from_right_tool0: Transformation (right arm tool0 offset from bar)
# - groups: list containing two planning group names [base_left_arm_manipulator, base_right_arm_manipulator]
if trigger:
    result = plan_dual_arm_cartesian_motion(
        planner, start_state, options,
        start_world_from_bar, goal_world_from_bar,
        bar_from_left_tool0, bar_from_right_tool0,
        groups
    )
    new_trajectory = result["combined_trajectory"]

    # Extract visualization objects for debugging in Rhino
    if "visualization" in result:
        visualization = result["visualization"]
        
        # These variables are now available for visualization in Grasshopper:
        # - bar_planes: List of Rhino Plane objects for bar waypoint frames
        # - left_arm_planes: List of Rhino Plane objects for left arm tool0 frames
        # - right_arm_planes: List of Rhino Plane objects for right arm tool0 frames
        
        bar_planes = visualization["bar_planes"]
        left_arm_planes = visualization["left_arm_planes"]
        right_arm_planes = visualization["right_arm_planes"]
        
        print("Visualization objects created and available for debugging:")
        print(f"  bar_planes: {len(bar_planes)} planes")
        print(f"  left_arm_planes: {len(left_arm_planes)} planes")
        print(f"  right_arm_planes: {len(right_arm_planes)} planes")
        
        # Check if trajectories are None (due to errors)
        if result.get("left_arm_trajectory") is None or result.get("right_arm_trajectory") is None:
            print("\n⚠️  WARNING: Trajectories are None due to planning errors!")
            if "error_message" in result:
                print(f"Error: {result['error_message']}")
            if "error_segment" in result:
                print(f"Failed at segment: {result['error_segment']}")
            print("Use the visualization objects to debug the frame positions and orientations.")
        else:
            print(f"\n✓ Successfully created trajectories:")
            print(f"  Left arm: {len(result['left_arm_trajectory'].points)} trajectory points")
            print(f"  Right arm: {len(result['right_arm_trajectory'].points)} trajectory points")
            
            # Show combined trajectory info if available
            if "combined_trajectory" in result:
                combined_traj = result["combined_trajectory"]
                print(f"  Combined: {len(combined_traj.points)} trajectory points")
                print(f"  Combined joint names: {combined_traj.joint_names}")
                
                # Verify dual arm synchronization
                print("\n" + "="*60)
                print("DUAL ARM SYNCHRONIZATION VERIFICATION")
                print("="*60)
                
                def verify_dual_arm_synchronization(planner, combined_trajectory, groups, bar_from_left_tool0, bar_from_right_tool0, tolerance=0.001):
                    """
                    Verify that the transformation between the two robot arms' tool0 frames is consistent
                    across all trajectory points using forward kinematics.
                    """
                    robot_cell = planner.client.robot_cell
                    left_arm_group = groups[0]
                    right_arm_group = groups[1]
                    
                    # Get joint names for each arm
                    left_joint_names = robot_cell.get_configurable_joint_names(left_arm_group)
                    right_joint_names = robot_cell.get_configurable_joint_names(right_arm_group)
                    
                    # Expected transformation between arms (left_tool0_from_right_tool0)
                    # This should be: bar_from_left_tool0.inverse() * bar_from_right_tool0
                    expected_left_tool0_from_right_tool0 = bar_from_left_tool0.inverse() * bar_from_right_tool0
                    
                    verification_results = {
                        "success": True,
                        "max_position_error": 0.0,
                        "max_orientation_error": 0.0,
                        "failed_points": [],
                        "details": []
                    }
                    
                    print("Verifying dual arm synchronization across trajectory points...")
                    
                    for i, point in enumerate(combined_trajectory.points):
                        # Extract joint values for each arm from the combined trajectory point
                        left_joint_values = [point[joint_name] for joint_name in left_joint_names]
                        right_joint_values = [point[joint_name] for joint_name in right_joint_names]
                        
                        # Create temporary robot cell states for forward kinematics
                        temp_state = start_state.copy()
                        temp_state.robot_configuration = combined_trajectory.start_configuration.copy()
                        
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
                        left_tool0_from_right_tool0 = world_from_left_tool0.inverse() * world_from_right_tool0
                        
                        # Compare with expected transformation
                        position_error = (left_tool0_from_right_tool0.translation_vector - expected_left_tool0_from_right_tool0.translation_vector).length
                        
                        # Calculate orientation error by comparing axis vectors
                        # Convert transformations to frames to get axis vectors directly
                        actual_frame = Frame.from_transformation(left_tool0_from_right_tool0)
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
                            print(f'pt#{i} - pos error {position_error}, ori error {orientation_error}')
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
                    if verification_results["success"]:
                        print(f"✓ Dual arm synchronization verification PASSED")
                        print(f"  Max position error: {verification_results['max_position_error']:.6f} m")
                        print(f"  Max orientation error: {verification_results['max_orientation_error']:.6f} rad")
                    else:
                        print(f"✗ Dual arm synchronization verification FAILED")
                        print(f"  Failed points: {len(verification_results['failed_points'])}/{len(combined_trajectory.points)}")
                        print(f"  Max position error: {verification_results['max_position_error']:.6f} m")
                        print(f"  Max orientation error: {verification_results['max_orientation_error']:.6f} rad")
                        print(f"  Failed at points: {verification_results['failed_points']}")
                    
                    return verification_results
                
                # Run the verification
                verification_result = verify_dual_arm_synchronization(
                    planner, combined_traj, groups, bar_from_left_tool0, bar_from_right_tool0, tolerance=0.001
                )
                
                # Store verification results in the result dictionary
                result["verification"] = verification_result