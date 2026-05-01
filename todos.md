# Todos

## VL

- [ ] Add validation examples for prefab export edge cases (missing bar IDs, missing joint metadata).

## YH 

### Assembly IK

- [] the ik workflow should allow user to pick male and female joint. this will require an additional male-joint_from_tool0 transformation.

- [x] robot half transparent overlay to help user visualize when picking the ground position with two points.
- [x] should also color the env collisions in consideration (built bar etc) green in the ik found, waiting for accept phase so users know what to check for. Now I am not sure if the built bar collision is included at all, since at least in debug_ik script this is not shown in pb gui.
    - needs double check on env collision

- [x] showIK should also display tool, and the mesh should not be baked. Similar behavior like the IK, make it disappear after done. 
    - Works not but ideally this should not use the block instance, it should just load the robot cell

- [] refactor the core robot_cell to assembly_robot_cell

- [] review the robot cell state viz stuff to see its logic
    - show ik a bit slow now

- [] automatically export current state into a json file, first time in a Rhino file, ask user for a path (in Rhino cmd), but keep it in default unless user wants to change it.
- [] batch export all bar's cell state into jsons, print bars that do not have cell state computed.

- [] need to also support the DeckMale

- optional collision checks for pineapple and show diagnosis

- if bar gets moved, IK should be removed
    - don't have an auto hook atm, needs manual management
    - can do a FK to check consistency

- collision geo pick up from a layer, a sequence id will be provided by Victor, and we can just look up the bar id and joint ids accordingly
    - need to decide on joint name for easy look up
    - let's ignore this in the first pass

### MP workflow
Decision:
Like in itj, we don't do mp in Rhino, only do mp in python and just load planned traj json back to Rhino for viz.

- insertion use averaged two vector, LM distance default 15 mm so 1-2 mm before screw engaged
- naming: B1_linear_approach, B1_linear_retract, B1_free_approach
- export four separate trajectories with naming convention

### Robotic-tool definition workflow

- [ ] **`rs_define_robotic_tool` should also export a per-tool collision OBJ.** After the user picks the tool block + TCP frame points + names the tool, prompt them to pick the visual / collision meshes (any number) and write a single combined OBJ to `asset/<block_name>.obj` whose origin coincides with the block's local frame (= tool0 / robot flange). Update the dataclass to fill `collision_filename` with the basename. The IK keyframe workflow already wires this through `core.robot_cell.attach_arm_tool_rigid_bodies` — once the export step exists, every newly-defined tool gets per-arm collision out of the box. Preserve X / Y axis sign of the picked meshes (no re-orthonormalization beyond the already-recorded TCP frame).

### Robotic-tool definition workflow

- [ ] **rs_define_robotic_tool should also export a per-tool collision OBJ.** After the user picks the tool block + TCP frame points + names the tool, prompt them to pick the visual / collision meshes (any number) and write a single combined OBJ to asset/<block_name>.obj whose origin coincides with the block's local frame (= tool0 / robot flange). Update the dataclass to fill collision_filename with the basename. The IK keyframe workflow already wires this through core.robot_cell.attach_arm_tool_rigid_bodies -- once the export step exists, every newly-defined tool gets per-arm collision out of the box. Preserve X / Y axis sign of the picked meshes (no re-orthonormalization beyond the already-recorded TCP frame).

