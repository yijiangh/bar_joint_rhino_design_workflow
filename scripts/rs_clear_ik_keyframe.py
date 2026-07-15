#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSClearIKKeyframe - Remove a bar's computed dual-arm IK solution.

Left-click (this script): delete only the arm IK configs (approach / assembled /
retreat, i.e. the M1-M3 solution) and KEEP the mobile base position, so the bar
can be re-solved at the same base.

Right-click (``rs_clear_ik_keyframe_and_base.py``): delete the arm IK configs AND
the mobile base position, so the bar reverts to fully "no IK".

Either way M4's home config is a constant (not stored per bar) and is untouched,
and the legacy bundled ``ik_assembly`` blob is removed (its configs would
otherwise linger for legacy readers). No PyBullet needed -- this only edits
user-text on the picked bar curve.
"""

from __future__ import annotations

import importlib
import os
import sys

import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import rhino_bar_registry as _registry_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import BAR_ID_KEY, repair_on_entry


def clear_ik(clear_base_frame: bool) -> None:
    """Pick a bar and clear its IK solution; keep or drop the base per the flag.

    Args:
        clear_base_frame (bool): False = keep the mobile base position (left-click);
            True = also delete the base position (right-click).

    Returns:
        None.
    """
    config = importlib.reload(_config_module)
    registry = importlib.reload(_registry_module)

    title = "RSClearIKKeyframe" + (" (+base)" if clear_base_frame else "")
    repair_on_entry(float(config.BAR_RADIUS), title)

    rs.UnselectAllObjects()
    what = "IK config + base" if clear_base_frame else "IK config (keep base)"
    bar_oid = pick_bar(f"Pick a bar to clear its {what} (Esc to cancel)")
    if bar_oid is None:
        return
    bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY) or "<unregistered>"

    removed = registry.clear_assembly_ik_keyframe(bar_oid, clear_base_frame=clear_base_frame)
    base_kept = (
        not clear_base_frame
        and bool(rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME))
    )

    if not removed:
        print(f"{title}: bar '{bar_id}' had no arm IK config to clear.")
        rs.MessageBox(
            f"Bar '{bar_id}' had no arm IK config to clear."
            + ("\n(The mobile base position, if any, was kept.)" if not clear_base_frame else ""),
            0,
            title,
        )
        return

    base_note = (
        "Mobile base position kept (re-solve at the same base)."
        if base_kept
        else ("Mobile base position kept (none was stored)." if not clear_base_frame
              else "Mobile base position also removed.")
    )
    print(f"{title}: cleared {len(removed)} key(s) on bar '{bar_id}': {removed}; {base_note}")
    rs.MessageBox(
        f"Cleared the arm IK config on bar '{bar_id}'.\n\n"
        "Removed user-text keys:\n  " + "\n  ".join(removed) + "\n\n"
        f"{base_note}\n"
        "M4 home config is a constant and was left untouched. Re-run RSIKKeyframe "
        "(or the headless base+IK sampler) to re-solve.",
        0,
        title,
    )


def main() -> None:
    # Left-click: clear the arm IK config, KEEP the mobile base position.
    clear_ik(clear_base_frame=False)


if __name__ == "__main__":
    main()
