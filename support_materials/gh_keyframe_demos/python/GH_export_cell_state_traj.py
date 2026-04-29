# export_robotcell_to_json.py

import json
import os
from datetime import datetime
from compas import json_dump, json_load
from compas_fab.robots import RobotCell, RobotCellState


def export_robotcell_and_state(content, filepath, file_prefix=None, approach_trajectory=None, retreat_trajectory=None):
    """
    Export RobotCell or RobotCellState to JSON with optional trajectory data.
    
    Parameters:
    -----------
    content : RobotCell or RobotCellState
        The content to export
    filepath : str
        Directory path where to save the files
    file_prefix : str, optional
        Prefix for the filename
    approach_trajectory : JointTrajectory, optional
        Approach trajectory to save alongside the state
    retreat_trajectory : JointTrajectory, optional
        Retreat trajectory to save alongside the state
    """
    os.makedirs(filepath, exist_ok=True)  # Ensure directory exists

    content_class = type(content).__name__
    if file_prefix:
        # If this is a RobotCellState, append timestamp for uniqueness
        if isinstance(content, RobotCellState):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_prefix = f"{file_prefix}_{timestamp}"
        file_name = f"{file_prefix}_{content_class}.json"
    else:
        file_name = f"{content_class}.json"

    full_path = os.path.join(filepath, file_name)

    with open(full_path, "w") as f:
        json_dump(content, f, pretty=True)

    print(f"Exported {content_class} to {full_path}")
    
    # If trajectories are provided and content is RobotCellState, also save the trajectories
    if isinstance(content, RobotCellState):
        if approach_trajectory is not None:
            trajectory_class = type(approach_trajectory).__name__
            if file_prefix:
                trajectory_file_name = f"{file_prefix}_approach_{trajectory_class}.json"
            else:
                trajectory_file_name = f"approach_{trajectory_class}.json"
            
            trajectory_path = os.path.join(filepath, trajectory_file_name)
            
            with open(trajectory_path, "w") as f:
                json_dump(approach_trajectory, f, pretty=True)
            
            print(f"Exported approach {trajectory_class} to {trajectory_path}")
        
        if retreat_trajectory is not None:
            trajectory_class = type(retreat_trajectory).__name__
            if file_prefix:
                trajectory_file_name = f"{file_prefix}_retreat_{trajectory_class}.json"
            else:
                trajectory_file_name = f"retreat_{trajectory_class}.json"
            
            trajectory_path = os.path.join(filepath, trajectory_file_name)
            
            with open(trajectory_path, "w") as f:
                json_dump(retreat_trajectory, f, pretty=True)
            
            print(f"Exported retreat {trajectory_class} to {trajectory_path}")


# Example usage:
# robot_cell, robot_cell_state, approach_trajectory, retreat_trajectory = ... (your objects)
if save_robot_cell and robot_cell is not None:
    export_robotcell_and_state(robot_cell, file_path)
if save_state and robot_cell_state is not None:
    export_robotcell_and_state(
        robot_cell_state,
        os.path.join(file_path, 'RobotCellStates'),
        state_name,
        approach_trajectory,  # Pass approach trajectory if available
        retreat_trajectory    # Pass retreat trajectory if available
    )