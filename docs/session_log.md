# Session log

Every Rhino command of this plugin writes what it did to a text file: one file per Rhino session, in `logs/` at the
repository root (`rs_session_<date>_<time>.log`; the folder is not tracked by git). The log is for reading back a
test session: what was run, what the user answered, what went wrong, and where the time went.

- The first command of a session prints `Session log: <path>` in Rhino's command history.
- `RSShowLog` (type it after `RSRegisterAliases`, or run `_-ScriptEditor _R "rs_show_log.py"`) prints the path and
  opens the file.
- Each line is written and flushed at once: the log is complete up to the last line even if Rhino crashes.
- `RS_LOG=0` in the environment switches it off; `RS_LOG_DIR` moves the folder.

## Reading it

The file starts with the session: date, machine, Rhino and Python version, the plugin's commit (and whether it has
uncommitted changes), the submodules' commits, the package folder and the IK backend.

Then one block per command, in the order they were run:

```
12:43:07.827  #1 RSIKKeyframe  (rs_ik_keyframe.py)
              document: C:\...\demo.3dm | 2314 objects | units Millimeters
12:43:07.827    > repair_on_entry
12:43:07.948      . enforce_managed_layers   0.12 s
12:43:07.994      RSIKKeyframe (sanity): 39 bar centerline(s), 39 tube preview(s).
12:43:07.994    < repair_on_entry   0.17 s
12:43:07.994    ask   GetObject
12:43:08.295    got   Object [B4 (3f2a91c0)]   (after 0.30 s)
12:43:08.476    . solve_keyframe_chain seed   0.10 s
12:43:08.831    caught (the command carried on): Traceback (most recent call last): ...
12:43:09.133  #1 RSIKKeyframe ended: ok
              total 1.31 s = 0.81 s working + 0.50 s waiting for the user
              working time by step (steps inside a step are indented and already counted in it):
                   0.17 s  repair_on_entry
                   0.12 s    enforce_managed_layers
                   0.45 s  solve_keyframe_chain   3x, slowest 0.20 s
                   0.01 s  (outside any step)
              errors caught and carried on from: 1
              Rhino's command history during the command (prompts and clicked options; prints left out):
                Select bar ( Pair=T20 )
```

| Line | Meaning |
| --- | --- |
| plain text | Something the command printed (what the user saw in the command history). `stderr` marks warnings. |
| `> step` … `< step  0.17 s` | A timed step that had lines inside it: began, ended, its working time. |
| `. step  0.12 s` | A timed step with nothing logged inside it. Steps under 0.02 s get no line, only the summary. |
| `ask` / `got … (after 3.2 s)` | A prompt or message box, the answer, and how long the user took. |
| `note` | A value or decision the code chose to record. |
| `caught (the command carried on)` | An error the code handled and continued from, with its traceback. The user saw at most a one-line message. Written the first time it happens at a place; repeats are counted in the summary. |
| `ERROR` | An error that ended the command, with its traceback. |
| `ended: ok` / `ended: FAILED: …` | How the command ended. `ok` only means no error escaped: a cancelled command is `ok` too (the last `got` line says `Cancel`). |

**Time.** `total = working + waiting for the user`. Waiting is the time inside prompts and message boxes; it is
left out of every step's time. The table lists each step once with its total, its count and its slowest run;
indented steps ran inside the step above and are already counted in it, so only the top-level rows and
`(outside any step)` add up to the working time.

- A dynamic preview drawn while the mouse moves runs inside a prompt: it counts as waiting. Steps timed in there
  still appear, nested under the `ask`.
- The command history lines at the end have no times: they show the prompt texts and the options clicked, which
  RhinoCommon getters do not tell the log.

## For developers

`scripts/session_log.py` holds everything; it needs the standard library only and does nothing outside a command
(tests, `plan_process.py`, Grasshopper).

| Call | Use |
| --- | --- |
| `session_log.run(__file__, main)` | The entry point of every command script, in its `if __name__ == "__main__":` block. |
| `with session_log.step("name", detail):` | Time a block. `name` is the summary's key: keep it the same every time and put the bar or robot in `detail`. |
| `@session_log.timed()` | The same for a whole function (not for generators). |
| `session_log.ask(getter)` | In place of `getter.Get()` for RhinoCommon getters. `rs.GetObject`, `rs.MessageBox` and the other `rs` prompts are wrapped automatically while a command runs. |
| `session_log.caught(exc)` | First line of an `except Exception as exc:` block that carries on. |
| `session_log.note("text")` | One line worth reading back. |

- ! Never reload `session_log` (`RSResetPy` leaves it alone): it holds the open file and the running command.
- A command that runs another command's `main` is one block, the outer one.
