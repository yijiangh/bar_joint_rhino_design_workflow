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


# RobotAction class

/plan Some note on your plan:
- I don;t wanna touch env_collisions.py since the workflow in rhino is already quite mature and working. What we are building is 
a downstream export function, so we build new stuff to achieve our needs without touching upstream.

Let's take a step back and look at the broader workflow, so we plan for that.

instead of needing a single cell state at a time, 
what we really need is an BarAction.json, which contains a sequence of robot movements for which we need to plan for.

For assembling each bar, the dual-arm robot needs to go thru the following movements:
1. End effector-Constrained Dual-arm Free Motion (CDFM) to bring the bar from a home pose to the approach state 
    - here the constraint is to maintain the relative transformation of the two end effectors since they are rigidly grasping the bar
    - the active bar and joints moutned on this bar, and the tools (end effectors) are attached to the robot
    - Allowed collisions: the tool and the joints that the tool mates to. the tool and the mounted link on the robot
    - Collisions needs to be checked (in addiiton to the normal list of collisions like env-robot, robot self collision): between the attached bodies (tool, joint, bar) and robot itself; and the attached bodies and the env
2. Holding the bar, the dual arm needs to follow a linear path to insert the grasped male joints into its mating female joints
    - Same as above. But in addition, we add ACM between the male joint and female joints since they are meant to be touching. Check scripts\rs_ik_keyframe.py and the ACM setting therein as a source of truth.
3. After the joint is fixed, the dual arm will do a treat LM motion to move away from the joints to a safe distance, to prepare for Free Motion back to home pose.
    - here the two robot arm do not maintain the relative EE constraint, they simply retreat separately following the negative z axis of the joint ocf. But they each will follow a linear motion on the end effector.
    - ACM: between the tool and the male joints
4. Free motion to home pose
    - both arms do not have any bar grasped, so each do a unconstrained motion back to a pre-fix home pose.
    - no additional acm other than tool and mounting links

So if my downstream task is to plan for each one of these movements,
For each movement, I will need a RobotMovement class that contains:
1. Start robot_cell_state
2. robot_id (actuated robot)
2. Allowed COllisoon matrix (acm)
3. robot base position
4. targets:
   4.1. For movement 1,2,3: target end effector frames w.r.t world (and sholud be transformed to the robot base frame for planning)
   4.2. For movement 4: a fixed robot config

So I think I need a class called BarAction that contains the following info:
1. Active bar being assembled
2. Assembly seq (from which we can construct the built bars)
3. A list of RobotMovement (can be both assembly or support)

Note:
1. the RobotMovement can both be pointing to an assembly robot or a support robot, 
in the case of a support robot, the assembly robot will be "downgraded" to an acticulated tool and treated as a collision obstcales.
By default, we assume the actuated robot is the assembly robot, 
2. In the list of RobotMovement, the start state of the next robot movement should be consistent with the end of the last movement. We only keep track of the start state, but essentially each state is the diff of an movemnt applied onto the last start state.