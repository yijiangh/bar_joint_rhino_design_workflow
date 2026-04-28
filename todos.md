# Todos

- [ ] Add validation examples for prefab export edge cases (missing bar IDs, missing joint metadata).

- [] the ik workflow should allow user to pick male and female joint. this will require an additional male-joint_from_tool0 transformation.
- [] robot half transparent overlay to help user visualize when picking the ground position with two points.
- [] should show both 

- [] showIK should also display tool, and the mesh should not be baked. Similar behavior like the IK, make it disappear after done

## IK Keyframing (YH)

### IK keyframe workflow, including the Pineapple proxy check workflow
Goal: 
allow users to click on two joints, preview the pineapple as a quick proxy if the robot wrist and tool will cause collision, and then compute IK based on user selected mobile base position.
I already have a working prototype in Grasshopper by connecting a few scattered python script that uses compas_fab's pybullet backend, now we want to streamline this as rhino buttons. Below I describe
the input/output and provide example code snippets from my GH demo.

- User workflow: 
1. user clicks on left arm Male joint, and then right arm Male joint
    - ok for now, but ideally this should be just clicking on a bar, since each bar should only have two Male joints
2. show pineapple preview (robot wrist + tool, note that the left arm wrist and tool are different to right arm ones, prebaked as a block definition in ocf, positioned to the derived tool0 position)
    - The block definition is named "AssemblyLeft_Pineapple" and "AssemblyRight_Pineapple", user should have already pasted them into the Rhino file beforehand. If it doens't exist, just throw a runtime error like in @rs_joint_place
3. ask user to pick a base position, and one more point to indicate the x axis for heading. If no solution found, randomly sample in a small circle neighborhood with a limited trail. Sampling radius and max iter can be set in config.py, don't need to be exposed in Rhino. Directly snap to a brep, which is a more Rhino native way to do things. Don't do point to brep projection. 
4. can ask for user to include collision checks or not, default to True
5. Preview the resulting IK config in Rhino (can use the compas_fab scene display logic to use cache)
    - the tool0 pose in world frame is derived from picked joint's OCF to a fixed joint-ocf_from_tool0 tf stored in config.py
6. then, take the opposite average direction of the z axes of the two male joints, unitize it, and move the two tool0 frames before along this direction for a fixed distance. This distance is called "LM_DISTANCE" and is set to a fixed value of 15 mm in config.py.
7. Redo the same procedure on this offsetted target frames for step 2 to step 5.
8. If the user clicks accept,
    - after previewing the IK, the robot base position and config of step 2-5 should be saved to an object attribute with key "assembly", with data contains the robot base pose in world frame and dual arm configs in a json format.
    - we throw away the IK robot mesh since storing all will make the Rhino file large. 

An example of how to compute IK for the dual-arm robot that I used in my Grasshopper prototype can be found in:
GH_ik_dual_arm_keyframe.py

Here the planning groups we used are:
base_left_arm_manipulator and base_right_arm_manipulator

Loading the robot model and init the robot cell and cell state can be found in:
GH_create_robot_model.py and GH_create_robot_cell.py

the urdf used are:
husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf
and 
dual_arm_husky.srdf

These robot related info should be set in config.py.

Pybullet should be used as sticky so it is persistent, example in GH_init_pb.py.
Example for setting a state can be found in GH_set_cell_state.py

Additional, related buttons to make:
- A button to export male joint to tool0 (joint-ocf_from_tool0 tf) using a config-auto gen workflow  (click on frames and export, similar to the rs_export_config.py)
- A button to start the pb client, give users option to use gui or not (see [])
- A button to disconnect the pb client (see [])
- A button that allows the user to click on a bar, and if the ik result is already saved into the bar, show the mesh viz of the IK result again.

Also make the data to be saved into the bar compatible with what Victor already have in the related scripts in `scripts` fore example @rs_bar_brace and @snap,

### New data to be stored to the bar axis lines
First, read the code in rs_create_bar.py and rs_joint_place.py to understand what data attribute is stored in the bar axis line and joint block instance. 

For the IK keyframing, we will store the following two new info into the bar axis line after completed.

- assembly info
    - robot id
    - robot base position in wcf
    - dual arm confs
- support info
    - robot id
    - robot base position in wcf
    - single arm conf

We don't store:
- bar ocf 
    - this is always computed fresh using the convention in @coordinate-conventions
- joint types and joint ocfs 
    - the joint should be looked up by going through all the baked Joint Block INstance's data attribute and look for the male_parent_id and female_parent_id that matches the selected bar id.
    - ocf are stored in the BlockInstance's transformation
    - joint type will be used to dispatch to the right grasp tf (ocf to tool0) for keyframe

Todos:
- [x] add husky_urdf as a submodule here
- [x] export the wrist + tool coarse mesh in tool0 frame as a block

Later
- automatically export current state into a json file, first time in a Rhino file, ask user for a path (in Rhino cmd), but keep it in default unless user wants to change it.
- batch export all bar's cell state into jsons, print bars that do not have cell state computed.
- optional collision checks for pineapple and show diagnosis
- if bar gets moved, IK should be removed
    - don't have an auto hook atm, needs manual management
    - can do a FK to check consistency
- should support Robotiq gripper
- need to also support the DeckMale
- collision geo pick up from a layer, a sequence id will be provided by Victor, and we can just look up the bar id and joint ids accordingly
    - need to decide on joint name for easy look up
    - let's ignore this in the first pass

### MP workflow
Decision:
Like in itj, we don't do mp in Rhino, only do mp in python and just load planned traj json back to Rhino for viz.

- insertion use averaged two vector, LM distance default 15 mm so 1-2 mm before screw engaged
- naming: B1_linear_approach, B1_linear_retract, B1_free_approach
- export four separate trajectories with naming convention
