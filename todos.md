# Todos

## VL

- [ ] Add validation examples for prefab export edge cases (missing bar IDs, missing joint metadata).
- [x] `RSGroundPlace` (`rs_ground_place.py`): pick a bar + a point on the bar, pick a registered ground joint, auto-compute `jr` so block +Y aligns with world +Z, click-to-flip preview, bake on `LAYER_JOINT_GROUND_INSTANCES` with re-edit metadata. `RSJointEdit` recognises ground blocks and flips by 180 deg.

## YH 

### Assembly IK

- [] disable arm-arm collision when first solving for left (since right is at whatever warm start position at that moment), but when solving for the right arm enable everything.

#### Longer term

- [] the ik workflow should allow user to pick male and female joint to grasp. this will require an additional male-joint_from_tool0 transformation.

- [] need to also support the DeckMale

- optional collision checks for tool preview and show diagnosis

- if bar gets moved, IK should be removed
    - don't have an auto hook atm, needs manual management
    - can do a FK to check consistency

### MP workflow
Decision:
Like in itj, we don't do mp in Rhino, only do mp in python and just load planned traj json back to Rhino for viz.

- insertion use averaged two vector, LM distance default 15 mm so 1-2 mm before screw engaged
- naming: B1_linear_approach, B1_linear_retract, B1_free_approach
- export four separate trajectories with naming convention

### Robotic-tool definition workflow

- [x] **rs_define_robotic_tool should also export a per-tool collision OBJ.** Done -- RSDefineRoboticTool (AssemblyTool mode) prompts you to pick hand-modeled collision MESH object(s) -- and/or a block instance whose definition already is a low-poly mesh (e.g. the tool block itself; only actual Mesh objects are taken, breps are never auto-meshed) -- and writes them merged to `asset/<block_name>.obj` in the block's local frame, filling `collision_filename`. See also RSSwapRoboticTool for switching the active tool candidate pair.


# RobotAction class
- [x] Add M0 (IndependentDualArmFreeMovement; unplanned lead-in, goal backfilled from M1 start).
- [x] rename RoboticLinearMovement into two: EndEffectorConstrainedDualArmLinearMovement (M2)
      and IndependentDualArmLinearMovement (M3). (M1 -> EndEffectorConstrainedDualArmFreeMovement,
      M4 -> IndependentDualArmFreeMovement.)

Debatable
- After Rhino IK, save confs into a BarAction json file already, and when clicking show IK, load from file directly.
- Or we still save things in Rhino, but needs to include all key info for all movements, not just approach and end.


 what if we try a new planning sequence, we plan M2->M1->M3->M4?
  so M2 's start conf will be propagated to be M1's start conf, and then M1's planning
  logic will need to change -> It doesn't need to solve for goal conf, and the start
  conf's

# Support robot IK

## Finetuning

- the unbuilt structure should be hiden in ik keyframe for holding robot
- I want the same sampling radius viz for holding ik
- toggle thru solved keyframes at the end of hold ik

## Initial prompt
now we are trying to add the support robot back into the ik keyframe. I would like to keep using one ik keyframe button, since only a few bars that has this "unstable" 
state in the sequence would be eligible for the support robot ik.
But there are some complications on the scene modeling. Right now, we only have BarAction class that models the process of assembly actions for the dual arm robot.
For planning the single-arm robot for the support action, we need new classes called BarHoldingAction and BarHoldingReleaseAction in external\rs_data_structure\rs_data_structure\, needs a new python file for this, but I think the bar_action.py and this new hold_action.py share the same base Movement and Action class.

And we will need to split BarAssemblyAction into two halves:
BarAssemblyJointingAction:
the existing M0-M2 BarAssemblyAction, but more detailed:
0. IndependentDualArmFreeMovement: move from robot current conf to the start of EndEffectorConstrainedDualArmFreeMovement with the bar held in hand. (same to previous M0)
1. ManualMovement: operator mount the bar in the robot's EEs.
2. ScaffoldingToolMovement: robot both tools activate grasping screws to grasp the bar (it will automatically stall until tightely grasped)
3. EndEffectorConstrainedDualArmFreeMovement: same as M1 before, free bar-held transfer motion to the start of approach.
4. ScaffoldingToolMovement: robot both tools tighten the jointing screws (keep activating the screw until the end of the next LinearMovement)
5. EndEffectorConstrainedDualArmLinearMovement:  linear bar-held constraint for insertion, for controller, we need to use the Cartesian Compliant arm controller here, while all the other robot movements, unless specially noted, will use the standard joint tracking controller. The controller will switch back to the joint tracking controller after the robot reads a stall signoal from the tool's tightening screws. (same to previous M2)

BarAssemblyReleaseAction:
1. ScaffoldingToolMovement: robot both tools activate the untighten screws to ungrasp the grasped joint and bar.
2. IndependentDualArmLinearMovement: linear arm independent retraction. (same to previous M3)
3. IndependentDualArmFreeMovement: free indepednet transit motion to a given home pose. (same to previous M4)

For the assembly robot, for a bar that does not require holding, 
BarAssemblyJointingAction and BarAssemblyReleaseAction are executed in a sequence.

But when a bar requires holding, the assembly robot will first execute BarAssemblyJointingAction, then wait until the holding robot executes BarHoldingAction, and then the assembly robot can execute BarAssemblyReleaseAction.

For the holding robot, it has two acitons:
BarHoldingAction and BarHoldingReleaseAction

The BarHoldingAction should contain the following movements:
0. SingleArmFreeMovement: move from the robot's current live conf to the start of the approach pose
1. GripperToolMovement (new1): Gripper close
2. SingleArmLinearMovement: approach grasping location
The planning will stop here, since only later all the bars required to stabilize the held bar are installed, the holding robot proceed to the bar unholding action.
In between the holding and unholding actions, multiple bar assembly actions can take place, and sometimes even a second holding robot is required to because to assembly all bars requierd to stbaliize current bar, one of the stablizing bar needs another bar to be installed, which requires its own, additional stabilizing bars. 

BarHoldingReleaseAction should contain the following movements:
1. ToolMovement: Gripper open
4. SingleArmLinearMovement: retreat from graspling location

An example case's step-by-step screenshots is included in docs\support_demo, the index of the file name is the step number. You should read all the photos to read them.
Below, I write out the corresponding image, the action associated. 
we have the following robots:

1 assembly robot (Cindy, referred as AR bellow):
available actions: 
- BarAssemblyJointingAction
- BarAssemblyReleaseAction 

2 support robots (Alice and Belle, referred as SR1 and SR2 below, the two robots are both single arm husky robot, but their kinematic paramters are different in the urdf, so cannot be mixed. But Functionally, they are equivalent. So when choosing a suitable robot, when both are available, you can choose alphabetically the first available one.)
available actions for each:
- BarHoldingAction
- BarHoldingReleaseAction

1.jpeg: we want to assemble B2 which is the blue bar here, and the purple bars are all the not yet built, required bars to stabliize it. Note that here B7 cannot be assembled directly, since it can be only assembled after the vertical bar B5 be assembled. So the holding of B2 needs to wait until B5, B7, B8 all assembled.
Actions:
BarAssemblyJointingAction(AR, B2)

2.jpeg: Then, after the assembly robot builds B2, it cannot let go immediately, a holding robot needs to hold B2, that is why B2 is colored in teal. And then, after the holding robot finishes the bar holding action, the assembly robot can execute the bar assembly action for the diagonal B3.

Actions:
BarHoldingAction(SR1, B2)
BarAssemblyReleaseAction(AR, B2)
BarAssemblyJointingAction(AR, B3)
BarAssemblyReleaseAction(AR, B3)

3.jpeg: Then in 3.jpeg, the robot goes on to assembly B4, no support required.

Actions:
BarAssemblyJointingAction(AR, B4)
BarAssemblyReleaseAction(AR, B4)

4.jpeg: Then B5 can be assembled. But AR cannot let go imeedidately bc of the support required. Here the purple bars show all the bars that need to be assembled before the holding robot can release.

Actions:
BarAssemblyJointingAction(AR, B5)

5.jpeg: Here a new support robot needs to be introduced bc SR1 is occupied to hold B2.

Actions:
BarHoldingAction(SR2, B2)
BarAssemblyReleaseAction(AR, B5)
BarAssemblyJointingAction(AR, B6)
BarAssemblyReleaseAction(AR, B6)

6.jpeg:

Actions:
BarAssemblyJointingAction(AR, B7)
BarAssemblyReleaseAction(AR, B7)

7.jpeg: After B8 is assembled, both the required stablizig bars for B2 and B5 can be released!

Actions:
BarAssemblyJointingAction(AR, B8)
BarAssemblyReleaseAction(AR, B8)
BarHoldingReleaseAction(SR1, B2)
BarHoldingReleaseAction(SR2, B5)

7.jpeg: Once the holding is released, the support robot will not appear in the scene for this bar's assembly. They are assumed to be moved to a location far away to avoid colllisions.
Actions:
BarAssemblyJointingAction(AR, B9)
BarAssemblyReleaseAction(AR, B9)

Now importantly, atm the current BarAssemblyAction is designed only for the assembly robot, and to include the two support robot, 
since compas_fab pybullet backend is not designed to handle multiple robot,
we do the following in implementation:
- when an action is on an assembly robot, the support robots are modelled as a Tool object which will inherent tool configuration (support robot arm conf) from previous state, and assumed to be fixed in the planning for the assembly robot aciton. The support robot's collisions must be included in all planning.
- when an action is on an support robot, the assembly robot and the other support robot is modelled as Tool objects, same as above.
Since compas_fab pybullet client only has one active robot id, so we will maintain three parallel compas_fab session, by default all in headless, no gui mode.
If gui mode is used in rs_start_pb, we only show the one for assembvly robot, keeping the other two pb clients for suport robot headless.

We have a old script on ik for holding robot that captures the ui logic: scripts\rs_ik_support_keyframe.py
But very outdated, and no proper action/movement/state management there. But I think the manual gripper pose selection and the ghost end effector visualization are all reusable.

Like BarAssemblyAction, BarHoldingAction will also need         active_bar_id: str = "",
        assembly_seq: Optional[list] = None,
        walkable_ground_ids: Optional[list] = None,
For scene creation, walkable ground for base sampling, etc. So maybe rethink the class abstraction so they can share this in the base class.

This is a big plan, and ask me alll questions that can help you clarify.