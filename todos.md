
# Todos

- [] split T1 s1 and s2 into two buttons

# initial prompt
We have designed a new type of connectors (we sometimes call it joint interchangeably) to join scaffodling bars, details of it is noted in this paper: C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\support_materials\papers\robarch2026_robotic_scaffolding_v1_highres.pdf
Now our goal is to implement a automatic script to help designs to optimize positions and connector placement from intuitive design input that allows users to have control.
The detailed specs are in C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\support_materials\specs.pdf
And if you find it helpful, the original ppt file used to generate the pdf is C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\support_materials\specs.ppt.
In the doc, the "CADJ paper" is C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\support_materials\papers\Huang et al. - 2025 - Computational design and fabrication of reusable multi-tangent bar structures.pdf

And the implementation you can find in:
C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\support_materials\FrameX\python\multi_tangent\smilp\contact_opt_func.py
C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\support_materials\FrameX\scripts\run_smilp_contactopt.py
and the distance computation for finite-length (not infinite length) is here:
C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\support_materials\FrameX\python\multi_tangent\contact.py:39

I might not have an active Gurobi license for you to run them but just see if you find them useful.

let's discuss, and in the end, I want you to generate a detailed instruction/plan which I can ask Codex to follow to do the impelementation. Read CLAUDE.md if you haven't.