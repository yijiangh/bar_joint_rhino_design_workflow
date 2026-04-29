# YH Lessons

Notes on environment / tooling gotchas we hit and want to remember.

---

## Rhino ScriptEditor `# r:` + git-SHA pinning ≠ live branch tracking

Pinning a compas_fab (or any) dependency by git SHA in a Rhino ScriptEditor `# r:` directive does **not** automatically refetch when you change the SHA.

**Why:**

- `# r: pkg @ git+https://…@<sha>` runs pip install into `scaffolding_env`.
- Pip tracks installed packages by **name + version** from their `pyproject.toml`, not by source commit. Installing commit A of `compas_fab` produces e.g. `compas_fab==1.0.0b1` in the venv's metadata.
- On the next run with a different SHA, pip sees "compas_fab==1.0.0b1 already satisfies the requirement" and skips the fetch → the venv still runs commit A's code, silently.

**How to force a new SHA to be picked up:**

- Uninstall manually inside the venv:
  - Delete `~/.rhinocode/py39-rh8/envs/scaffolding_env/Lib/site-packages/compas_fab*` (or `pip uninstall compas_fab` from that venv), then re-run the script.
- `# r: --force-reinstall pkg @ git+…@<sha>` — if Rhino forwards pip flags verbatim, this re-fetches on every run. Slow, needs internet each run.
- Nuclear: Tools → Advanced → Reset Python 3 (CPython) Runtime (reinstalls everything).
- Bump the upstream version string — not in our control.

**How to apply:**

- For one-time install of a known-good SHA: fine to use `# r: pkg @ git+…@<sha>`.
- For active iteration on an upstream dev branch (e.g. compas_fab `wip_process`): prefer **submodule + `sys.path` injection** over `# r:`. A local `git checkout <sha>` inside the submodule changes which code loads on the next run, no pip-cache state to fight.
- If we ever do ship a `# r: @ git+…@<sha>` pin in a script, add a comment next to it reminding readers that bumping the SHA requires a venv wipe.

---

## `sys.path.insert` does not displace an already-imported module

Inserting a path at the front of `sys.path` only affects modules that have not yet been imported. If `compas_fab` (or any other module) is already in `sys.modules`, subsequent `import compas_fab` hits the cache and returns the old copy — our shiny new path is ignored.

**Why this bit us:** the sibling `husky-assembly-teleop` repo also has a `compas_fab` submodule at `external/compas_fab`. When its scripts run in the same Rhino Python runtime session first, `compas_fab` gets imported from that path. Later, our script does `sys.path.insert(0, <our path>)` + `import compas_fab`, but Python happily hands back the cached sibling-repo copy. Tracebacks correctly show the D:\…\husky-assembly-teleop\… path — a dead giveaway that our submodule is being shadowed.

**How to apply:** before `sys.path.insert`, purge `sys.modules` entries that start with the package name, then import fresh. See `_ensure_submodule_compas_fab_loaded` in `scripts/core/robot_cell.py`. Also print `<module>.__file__` once at `start_pb_client` time as a diagnostic — staring at that single line saves an hour of guessing next time.

---

## Rhino 8 `# venv` is NOT an isolated Python venv

The ScriptEditor `# venv: <name>` directive creates a *site-env*, not a real venv. It lives at `~/.rhinocode/py39-rh8/site-envs/<name>-<hash>/` and is just an additional `site-packages` directory layered on top of the base Rhino CPython at `~/.rhinocode/py39-rh8/lib/site-packages/`. There is no `pyvenv.cfg`, no `include-system-site-packages` switch — the base always leaks in.

**Why this bit us:** user had previously run `pip install -e` against the base Rhino interpreter to register sibling-repo packages editably, so the base `site-packages` now contains:

- `__editable__.compas_fab-1.1.0.pth` → `D:\…\husky-assembly-teleop\external\compas_fab\src`
- `easy-install.pth` → teleop root + `external/pybullet_planning/src` + `external/husky_assembly_tamp`

Those `.pth` files are processed at every interpreter startup, prepending teleop source trees to `sys.path` regardless of which `# venv:` the current script declares. Classes from teleop's compas_fab and ours collide — double-loaded base-class, super() fails.

**How to apply:**

1. Keep each Rhino script's `# venv:` scoped (`scaffolding_env`, `compas_fab_dev`, etc.), but remember it does NOT isolate you from the base interpreter.
2. Avoid `pip install` (especially `pip install -e`) against the base Rhino interpreter. If you must, install into a sibling site-env that only a specific `# venv:` name reaches, never into `~/.rhinocode/py39-rh8/lib/site-packages/`.
3. To clean a contaminated base, uninstall via the base interpreter's pip directly:

   ```powershell
   C:\Users\yijiangh\.rhinocode\py39-rh8\python.exe -m pip uninstall <pkg> <pkg> ...
   ```

   Or surgically delete the offending `*.pth` and `*-dist-info/` files out of `~/.rhinocode/py39-rh8/lib/site-packages/`, then reset the Python runtime (Tools → Advanced → Reset Python 3 Runtime).
4. After cleanup, `ls ~/.rhinocode/py39-rh8/lib/site-packages/*.pth` should only show truly-shared infra (`distutils-precedence.pth`, `zope.interface-…-nspkg.pth`) — no paths pointing at any repo source tree.

---

## Pin every `# r:` requirement to an exact version

From now on, every `# r:` line in a Rhino script pins an exact version (`# r: numpy==1.24.4`, not `# r: numpy`).

**Why:**

- Rhino's site-envs share the base interpreter's site-packages (see lesson above). If two projects declare `# r: compas` unpinned, the first script to install locks a version in for everyone; the second project silently rides whatever was already there. Debugging class mismatches after a surprise upgrade is painful.
- Unpinned requirements also interact badly with the `# r:` pip-cache behaviour (see the git-SHA lesson above). Pinning makes "what did Rhino actually install?" a deterministic question with a single answer you can read off the script header.
- When bumping a version, the `# r:` diff shows up in git alongside the code change that needed it. Blame-able.

**How to apply:**

- Pin exact versions with `==`. Use `~=` only if an upstream compatibility range is strictly required (document the reason inline).
- Base all new pins on what's currently installed in `~/.rhinocode/py39-rh8/lib/site-packages/*.dist-info/`, unless we have a reason to move. Upgrade the pin deliberately.
- Example baseline (confirmed on Rhino 8 2026-04-23):

  ```
  # r: numpy==1.24.4
  # r: scipy==1.13.1
  # r: compas==2.13.0
  # r: compas_robots==0.6.0
  # r: pybullet==3.2.7
  # r: pybullet_planning==0.6.1
  ```
- For packages loaded from an in-repo submodule (e.g. compas_fab `wip_process`), **do not** list them under `# r:` at all — they come in via `sys.path` injection. Pin the submodule SHA via the `.gitmodules` commit instead.

---

## Don't selectively purge a sibling library while its dependents stay loaded

If module A is imported by module B (and B caches references to A's classes — directly or via a plugin registry), purging A from `sys.modules` and re-importing it leaves B holding the OLD class objects, while A's module globals (and any new instance) reference NEW class objects of the same name. `super()` then fails inside A:

    TypeError: super(type, obj): obj must be an instance or subtype of type

We hit this with `compas_fab` after we tried to selectively purge `compas_fab.*` while `compas` and `compas_robots` (which register compas_fab classes via the compas plugin registry) stayed loaded. The PyBulletBase referenced inside the freshly re-executed `client.py` was a different class object from the one in `PyBulletClient.__bases__` that the plugin registry still pointed at.

**How to apply:**

- To pick up changes in an in-repo library (compas_fab submodule, etc.), use ScriptEditor -> Tools -> Reload Python 3 (CPython) Engine. That clears the entire interpreter — every library is re-imported in lockstep, no class-identity drift.
- The `ResetPy` alias should only purge modules under `scripts/` (our own pure-Python code). It must NOT purge `compas_fab`, `compas_robots`, `compas`, etc. — see `scripts/rs_reset_py.py` for the implementation.
- In `core/robot_cell.py::_ensure_submodule_compas_fab_loaded`: only verify the path; never purge. If verification fails, raise with guidance to use Engine Reset.

---

## compas_fab `check_collision` walks 5 disjoint pair categories (CC.1–CC.5)

`PyBulletPlanner.check_collision(state, options={"verbose": True, "full_report": True})` enumerates collision pairs in five buckets. Every pair runs `_check_collision` (PyBullet contact test) unless a skip rule fires first. Verbose prints one line per pair: `PASS` / `SKIPPED (<reason>)` / `COLLISION`. Source: `external/compas_fab/src/compas_fab/backends/pybullet/backend_features/pybullet_check_collision.py`.

| Step | Pair kind | Coverage | Skip rules |
|---|---|---|---|
| **CC.1** | robot link ↔ robot link | self-collision (left wrist hitting right shoulder, etc.) | SRDF `disable_collisions` → `client.unordered_disabled_collisions` |
| **CC.2** | robot link ↔ tool | tool model clipping into a robot link (pineapple into forearm) | `tool_state.is_hidden`; `link_name in tool_state.touch_links` |
| **CC.3** | robot link ↔ rigid body | env objects (built bars, walls, jigs) hitting the robot — only fires once env is registered as `RobotCell.rigid_body_models` | `rigid_body_state.is_hidden`; `link_name in rigid_body_state.touch_links` |
| **CC.4** | rigid body ↔ rigid body | grasped workpiece hitting another rigid body. Static-vs-static is skipped; needs at least one side `attached_to_tool` / `attached_to_link` | hidden; same body; `touch_bodies` allowance |
| **CC.5** | tool ↔ rigid body | tool meshes hitting env rigid bodies (pineapple swiping a built bar) | hidden; `rb.attached_to_tool == tool_name`; tool in `rb.touch_bodies` |

Mental model:

- **CC.1** robot-vs-self
- **CC.2** robot-vs-tools
- **CC.3 + CC.5** robot/tool-vs-world (env)
- **CC.4** grasped workpiece-vs-world

For an env-collision feature (built bars as static obstacles), the load-bearing pair categories are **CC.3** (arm hits a built bar) and **CC.5** (tool hits a built bar). CC.4 only matters if the bar-being-placed is itself modelled as a rigid body attached to the tool.

**How to apply:**

- To debug "did IK actually consider X?", call `planner.check_collision(state, options={"verbose": True, "full_report": True})`. `full_report=True` keeps it from short-circuiting at the first failure so you see every pair. `debug_ik_collisions.py` already does this.
- "include_env" means nothing if `RobotCell.rigid_body_models` is empty — CC.3/CC.4/CC.5 simply have nothing to iterate. Surface that to the user (e.g. "0 env bodies registered") rather than letting the toggle look effective.
- Skip-rule precedence is: hidden → SRDF/semantic → touch_links/touch_bodies → contact test. If a pair fires unexpectedly, look up that order, don't grep blindly.
- `touch_links` (on tool / rb states) is the right knob for "this overlap is a property of the proxy mesh, not a real collision" (e.g. our wrist_1/2/3 entries on the pineapple). Don't reach for SRDF unless it's truly geometric robot self-collision.

---

## Debug capture ≠ production state — keep them on different save paths

The IK JSON capture (`tests/captures/<timestamp>_<bar>.json`) is a strict superset (RobotCell geometry + IK targets + IK options + source breadcrumbs) needed only by `debug_ik_collisions.py`. Auto-saving it on Accept is over-eager: it duplicates the bar user-text plus extras, but those extras only matter when something's wrong.

**Why:** Production state for chained motion planning lives on the bar's `ik_assembly` / `ik_support` user-text inside the 3dm — that's the canonical per-bar answer, addressable by bar OID, lives with the design. The JSON file in `tests/captures/` is a debug artefact, only useful when replaying a scenario headless to investigate a bug. Two different audiences (motion planner vs Claude debugging session), two different lifetimes (forever-with-design vs throwaway).

**How to apply:**
- Production data write on Accept → unconditional. The user is making a design decision; persist it.
- Debug capture write on Accept → opt-in, default No. Symmetric with the failure-path prompt. The user only wants the file when they're about to ask Claude to debug something.
- If a future feature really needs "all designs' production state in one file" (e.g. exporting a design folder for motion planning), that's a separate `RSExportDesign...` button that reads the bar user-texts directly — NOT a side-effect of the IK Accept path. Don't conflate the two.
