# Todos

## VL

- [ ] Add validation examples for prefab export edge cases (missing bar IDs, missing joint metadata).
- [x] `RSGroundPlace` (`rs_ground_place.py`): pick a bar + a point on the bar, pick a registered ground joint, auto-compute `jr` so block +Y aligns with world +Z, click-to-flip preview, bake on `LAYER_JOINT_GROUND_INSTANCES` with re-edit metadata. `RSJointEdit` recognises ground blocks and flips by 180 deg.

## YH 

### Assembly IK

- [] the ik workflow should allow user to pick male and female joint to grasp. this will require an additional male-joint_from_tool0 transformation.

- [] need to also support the DeckMale

- optional collision checks for tool preview and show diagnosis

- if bar gets moved, IK should be removed
    - don't have an auto hook atm, needs manual management
    - can do a FK to check consistency

### CC modeling

- Tool should be modeled as a ToolModel not a rigid body.

### MP workflow
Decision:
Like in itj, we don't do mp in Rhino, only do mp in python and just load planned traj json back to Rhino for viz.

- insertion use averaged two vector, LM distance default 15 mm so 1-2 mm before screw engaged
- naming: B1_linear_approach, B1_linear_retract, B1_free_approach
- export four separate trajectories with naming convention

### Robotic-tool definition workflow

- [ ] **rs_define_robotic_tool should also export a per-tool collision OBJ.** After the user picks the tool block + TCP frame points + names the tool, prompt them to pick the visual / collision meshes (any number) and write a single combined OBJ to asset/<block_name>.obj whose origin coincides with the block's local frame (= tool0 / robot flange). Update the dataclass to fill collision_filename with the basename. The IK keyframe workflow already wires this through core.robot_cell.attach_arm_tool_rigid_bodies -- once the export step exists, every newly-defined tool gets per-arm collision out of the box. Preserve X / Y axis sign of the picked meshes (no re-orthonormalization beyond the already-recorded TCP frame).


# RobotAction class
