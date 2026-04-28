# Claude Code Lessons

Short notes on patterns we've hit before. Lead with the pattern; add **Why** and **How to apply**.

---

## PyBullet verbose flag inside Rhino 8 ScriptEditor

Always start `PyBulletClient` with `verbose=True` when the script runs inside Rhino's ScriptEditor.

**Why:** `compas_fab.backends.pybullet.utils.redirect_stdout(enabled=not verbose)` calls `os.fdopen(os.dup(fd), "w")` on stdout. Rhino's ScriptEditor stdout is not backed by a real OS file descriptor, so `os.dup(fd)` raises. Setting `verbose=True` disables the redirect path entirely. The GH prototype already used `verbose=True` — the Rhino-side port should match.

**How to apply:** Any new Rhino script that starts a PyBullet client through compas_fab must pass `verbose=True`. In this repo, default is set inside `scripts/core/robot_cell.py::start_pb_client`. Do not expose a "Quiet" toggle that would flip it unless you handle the stdout fd issue upstream.

---

## Joint OCF → tool0 dispatch key must be the full block name

Use the block-definition name (e.g. `"T20_Male"`) as the dispatch key for `MALE_JOINT_OCF_TO_TOOL0`, not the `joint_type` user-text (`"T20"`).

**Why:** `rs_joint_place.py` assigns `joint_type = block_name.split("_", 1)[0]` and `joint_subtype = block_name.split("_", 1)[1]`. The full block name uniquely identifies the geometric variant (Male vs Female, different families). Dispatching by just `joint_type` collides when we later add a female grasp TF.

**How to apply:** In Rhino scripts, read the dispatch key via `rs.BlockInstanceName(oid)`. Fall back to `f"{joint_type}_{joint_subtype}"` when the block name has been renamed. The `RSExportJointTool0TF` prompt defaults to `"T20_Male"` to keep the two ends consistent.
