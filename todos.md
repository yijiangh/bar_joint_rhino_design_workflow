# Todos

- [ ] Add validation examples for prefab export edge cases (missing bar IDs, missing joint metadata).
- [ ] Review test coverage for `rs_export_case.py` replay workflows.

## IK Keyframing (YH)

### IK keyframe workflow, including the Pineapple proxy check workflow
Goal: 
allow users to click on two joints, preview the pineapple as a quick proxy if the robot wrist and tool will cause collision, and then compute IK based on user selected mobile base position.
I already have a working prototype in Grasshopper by connecting a few scattered python script that uses compas_fab's pybullet backend, now we want to streamline this as rhino buttons. Below I describe
the input/output and provide example code snippets from my GH demo.

- Input: 
1. user clicks on left arm Male joint, and then right arm Male joint
    - ok for now, but ideally this should be just clicking on a bar, since each bar should only have two Male joints
2. show pineapple preview (robot wrist + tool, prebaked as a block definition in ocf, positioned to the derived tool0 position)
3. ask user to pick a base position, and one more point to indicate the x axis for heading. If no solution found, randomly sample in a small circle neighborhood with a limited trail. Sampling radius and max iter can be set in config.py, don't need to be exposed in Rhino. Directly snap to a brep, which is a more Rhino native way to do things. Don't do point to brep projection. 
4. can ask for user to include collision checks or not, default to True
5. Preview the resulting IK config in Rhino (can use the compas_fab scene display logic to use cache)
    - the tool0 pose in world frame is derived from picked joint's OCF to a fixed joint-ocf_from_tool0 tf stored in config.py

- Output
    - after previewing the IK, the robot base position and config should be saved to an object attribute with key "assembly", with data contains the robot base pose in world frame and dual arm configs in a json format.
    - we throw away the IK robot mesh since storing all will make the Rhino file large.
    - automatically export current state into a json file, first time in a Rhino file, ask user for a path (in Rhino cmd), but keep it in default unless user wants to change it.

An example of how to compute IK for the dual-arm robot that I used in my Grasshopper prototype can be found in:
GH_ik_dual_arm_keyframe.py

Here the groups we used are:
base_left_arm_manipulator and base_right_arm_manipulator

Loading the robot model and init the robot cell and cell state can be found in:
GH_create_robot_model.py and GH_create_robot_cell.py

the urdf used are:
husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf
and 
dual_arm_husky.srdf

These robot related info should be set in config.py.

Pybullet should be used as sticky, example in GH_init_pb.py
Example for setting a state can be found in GH_set_cell_state.py

Additional, related buttons to make:
1. A button that allows the user to click on a bar, and if the ik result is already saved into the bar, show the mesh viz of the IK result again.
2. batch export all bar's cell state into jsons, print bars that do not have cell state computed.
3. export male joint to tool0 using a config-auto gen workflow  (click on frames and export, similar to the rs_export_config.py)

Also make the data to be saved into the bar compatible with what Victor already have in the related scripts in `scripts` fore example rs_bar_brace and snap,
below is the new info I need to add on top of what's existing to support the keyframing workflow

### Bar data stored to the brep
- bar ocf in world frame (Victor should have alreayd computed this following certain conventions, so just pick it up, don't recompute)
- joint types and joint ocfs (these should also already be computed by Victor's workflow)
    - ocf are stored in world frame
    - joint type will be used to dispatch to the right grasp tf (ocf to tool0) for keyframe
- assembly info
    - robot id
    - robot base position in wcf
    - dual arm confs
- support info
    - robot id
    - robot base position in wcf
    - single arm conf

Todos:
- [] add husky_urdf as a submodule here
- [] export the wrist + tool coarse mesh in tool0 frame as a block

Later
- [] optional collision checks for pineapple and show diagnosis
- if bar gets moved, IK should be removed
    - don't have an auto hook atm, needs manual management
    - can do a FK to check consistency
- should support Robotiq gripper
- need to also support the DeckMale
- collision geo pick up from a layer, a sequence id will be provided by Victor, and we can just look up the bar id and joint ids accordingly
    - need to decide on joint name for easy look up
    - let's ignore this in the first pass

### MP workflow
- insertion use averaged two vector, LM distance default 15 mm so 1-2 mm before screw engaged
- naming: B1_linear_approach, B1_linear_retract, B1_free_approach
- export four separate trajectories with naming convention
