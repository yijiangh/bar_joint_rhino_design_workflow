# Todos

- [ ] Add validation examples for prefab export edge cases (missing bar IDs, missing joint metadata).
- [ ] Review test coverage for `rs_export_case.py` replay workflows.

## IK Keyframing (YH)

- [] export male joint to tool0 using a config-auto gen workflow  (click on frames and export)

### Bar
- bar ocf in world frame
- joint types and joint ocfs
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

### Pineapple proxy check workflow
- [] export the wrist + tool coarse mesh in tool0 frame as a block
- [] a button to enable the pineapple, eyeball CC baseline
- [] optional collision checks and show diagnosis
- Input: user clicks on two male joint, but ideally this should be just clicking on a bar, since each bar should only have two Male joints

Later:
- pineapple workflow for support Robotiq gripper
- need to also support the DeckMale

### IK keyframe workflow
- collision geo pick up from a layer, a sequence id will be provided by Victor, and we can just look up the bar id and joint ids accordingly
    - need to decide on joint name for easy look up
    - let's ignore this in the first pass
- Input: 
1. user clicks on left arm Male joint, and then right arm Male joint
2. show pineapple preview
3. ask user to pick a base position, and one more point to indicate the x axis for heading. If no solution found, randomly sample in a small circle neighborhood with a limited trail. Sampling radius and max iter can be set in config.py, don't need to be exposed in Rhino. Directly snap to a brep, which is a more Rhino native way to do things. Don't do point to brep projection. 
4. can ask for user to include collision checks or not, default to True
5. 

- export current state json as an option
- also offer batch export state json into a folder

- if bar gets moved, IK should be removed
    - don't have an auto hook atm, needs manual management
    - can do a FK to check consistency

### MP workflow
- insertion use averaged two vector, LM distance default 15 mm so 1-2 mm before screw engaged
- naming: B1_linear_approach, B1_linear_retract, B1_free_approach
- export four separate trajectories with naming convention
