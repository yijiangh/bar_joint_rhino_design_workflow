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
