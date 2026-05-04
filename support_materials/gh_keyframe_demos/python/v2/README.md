# GH dual-arm IK keyframe (v2, parametric)

Live-slider front-end for `rs_ik_keyframe.py`. Drop the five GHPython
components into one GH document; wire them as below; drag sliders.

```
[trigger] [use_gui]
        \  /
       gh_init_pb (C1)  ---> pb_running

[pair_name][bar_length_mm][bar_anchor_plane]                  
       [jp_left][jr_left][jp_right][jr_right]                 
                              \                                
                          gh_bar_two_males (C2)                
                          /  |   |   |   |                    
              bar_curve   |   |   |   male_right_ocf_mm        
                male_left_plane male_right_plane male_left_ocf_mm
                                                  \           
                                            gh_male_ocf_to_tool0 (C3)
                                                  / | | \    
                       tool0_left_mm  tool0_right_mm  tool0_*_plane

[trigger][base_plane][check_self][check_bar_coll]
            tool0_left_mm + tool0_right_mm + bar_curve + bar_radius_mm
                              \                                
                          gh_ik_dual_arm (C4)                  
                              |                                
                          ik_state, solved                     

[vis_options][single_arm]                                      
        ik_state -> gh_scene_viz (C5) -> draw, tool0 planes    
```

## Why these components don't just `import compas_fab`

`compas_fab` is pinned as a git submodule under `external/compas_fab`, NOT
installed via pip into the `scaffolding_env` venv. (See the
`tasks/cc_lessons.md` rule: never list `# r: compas_fab`.) Each component
above starts with the same six-line bootstrap that puts `<repo>/scripts/`
on `sys.path` and imports `core.robot_cell` -- importing that module runs
`_ensure_submodule_compas_fab_loaded()` which prepends
`external/compas_fab/src/` to `sys.path` and verifies subsequent
`import compas_fab ...` calls resolve there.

If you see `ModuleNotFoundError: No module named 'compas_fab'` from a GH
component, the bootstrap is missing or the component imports compas_fab
*before* importing `core.robot_cell`.

## Slider ranges

- `bar_length_mm`: 500 .. 4000 (default 2000)
- `jp_left`, `jp_right`: offset along bar Z from the bar **midpoint** (= anchor
  origin). Useful range is `[-bar_length_mm/2, +bar_length_mm/2]`. The
  `MJP_RANGE = (-500, 500)` constant in `core.config` is the *optimizer* search
  bound, not a physical slider limit -- pick the range that fits your bar.
- `jr_left`, `jr_right`: -3.14159 .. 3.14159 rad
- `bar_radius_mm`: leave at default (10 mm) unless changed in `core.config.BAR_RADIUS`

## Pre-requisites

- `RSPBStart` was run at least once in this Rhino session OR C1 trigger is
  toggled. Both share the same sticky cache.
- For C3 to resolve `MALE_JOINT_OCF_TO_TOOL0[f"{pair_name}_Male"]`, run
  `RSExportGraspTool0TF` (Joint mode) once for both arm sides; the dict
  is auto-written to `scripts/core/config_generated_ik.py`.
- `WalkableGround` Brep / `bar_radius` constants are NOT used by this
  flow -- the GH path takes `base_plane` directly.

## Wire types between components

The OCF and tool0 4x4 matrices flow as `Rhino.Geometry.Transform` (NOT
Python list-of-lists). GH decomposes nested Python lists into branches at
the wire boundary, so an item-access receiver of a 4x4 nested list sees
only the first row (length 4); the matmul `(4,) @ (4,4)` then silently
yields a 1-D vector instead of a 4x4. `Transform` is a single Rhino
primitive that GH carries atomically.

Set the input pin type hint for these wires to **No type hint / Object**
(or "Transform" if your GH version exposes it). The components include
small `_np_mm_to_xform_raw` / `_xform_raw_to_np_mm` helpers to pack/unpack.

## What this does NOT do (use the Rhino path for these)

- Approach pose offset (`-avg(male_z) * LM_DISTANCE`)
- Persisting the solved keyframe to bar user-text (use `RSIKKeyframe` to
  accept + save once you've explored a good pose here)
- Pineapple preview block insert (the SceneObject in C5 already shows the
  full robot incl. attached tool meshes)
