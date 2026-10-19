# Windows checklist: the core merge (`yh/new-core-merge`)

**What this checks:** Jakob's new-core plugin, merged with main (Su's UI work, the MoCap joints), with these additions:
- the drive rules: a ground bar gets no tighten plus a fix-foundation step; a one-sided bar tightens one arm; nothing
  loosens at release;
- the range export;
- MoCap markers carried in `process.json`;
- the core's tool-against-tool collision check.

It can't be tested on Ubuntu, because Rhino doesn't run there. Here we ran the pure-Python tests and a headless build of
`261008_2ModulesDemo_purged.3dm`. This list covers what only Rhino can show.

## Before you start (once)

1. **The Dropbox copy.**
   - If this Windows PC uses the Dropbox-synced repo, note that it also holds:
     - two unpushed rs_data_structure commits (57c2cc9, 8244b39);
     - 5 stashes.
   - Push or save those first (see the session notes), or work in a fresh clone.
2. **Pushes.** These must be on GitHub before Windows can fetch them:
   - the core branch `yh/cc6-tool-pairs`;
   - tamp `yh/core-port`;
   - this repo's `yh/new-core-merge`.
3. **Update the checkout:**
   ```
   git fetch origin
   git switch yh/new-core-merge
   git submodule sync
   git submodule update --init --recursive
   ```
   The core submodule clones over SSH (git@github.com:yijiangh/scaffolding_assembly_core.git).
4. **ssik sidecar: reinstall with the pinned version.** The solver modules now live in tamp and are built with ssik 4.1.0:
   ```
   uv venv --python 3.11 external/ssik_env          # only if it does not exist
   uv pip install --python external/ssik_env "ssik==4.1.0"
   ```

## In Rhino 8

1. Reload `scaffolding_toolbar.rui`, then run **RSRegisterAliases**. The first command installs the Python packages;
   pybullet compiles once, about 5 minutes.
2. Open **`261008_2ModulesDemo_YH.3dm`**. Run **RSIKKeyframeAll** until **RSExportProcess** reports no STALE IK for
   B20–B37 and for the B21 and B35 holds.
3. In **RSShowBarActionPlan**, check:
   - [ ] **B20** (ground bar): the insert has no drives, and a "fix foundation" operator step follows it.
   - [ ] **B21**: only AT3L tightens.
   - [ ] **B22**: both tools tighten.
   - [ ] No release step unscrews anything.
   - [ ] B37 looks right.
4. **RSExportProcess**, Range **B20..B37** → `data_design_study/261019_3bar_holding_process`.
   - [ ] The MoCap layout check does not refuse. If it does, the message names the bar and what to change.
   - [ ] The label dots appear for B20 and B21, named like `joints/J20-21_mocap/M1`, with text `B20 #3 M1`.
   - Leave `261006_3bar_holding_test` untouched.
5. **RSExportProcess** once more with **no range** → `261019_2ModulesDemo_YH_process`.
6. A quick look at:
   - [ ] RSBarSelect and RSJointSelect (the renamed commands);
   - [ ] FlipOne in RSIKKeyframeAll;
   - optionally, RSShowBarActionPlanMotion and the Grasshopper preview.

## Send back
- **The two process folders.** Insync syncs them, and `source/` includes the .3dm.
- **The export summary text** from the command history.
- **The session's `logs/` folder.**
- **Screenshots** of anything red, and of any message box.

On our side, `tests/equivalence/compare_processes.py` will diff your export against a headless build of the same
document (use `--expect-headless-gaps --pos-tol 1e-6 --rot-tol 1e-6 --value-tol 1e-6`), and the new monitor will load
it.
