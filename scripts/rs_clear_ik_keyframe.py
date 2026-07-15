#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSClearIKKeyframe - Erase a bar's saved dual-arm IK data.

Left-click (this script): pick ONE bar. Right-click
(``rs_clear_ik_keyframe_all.py``): every registered bar.

Either way it then asks WHAT to erase via two toggles, both defaulting to Erase:

  - ``Keyframe``     -- the M1-M3 arm IK configs (approach / assembled / retreat,
                        plus the legacy bundled blob).
  - ``BasePosition`` -- the mobile base frame (so the bar reverts toward "no IK").

Untick a toggle to KEEP that part (e.g. keep the base to re-solve at the same
spot, or keep the keyframe and only drop the base). M4's home config is a
constant, not stored per bar, so it is always left untouched. No PyBullet needed
-- this only edits user-text on the bar curves.
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


def _ask_what_to_erase(scope_label: str):
    """Two-toggle prompt: erase the Keyframe and/or the Base position?

    Both toggles default to Erase (True), matching the common "wipe it all" case.

    Args:
        scope_label (str): describes what the clear applies to, e.g.
            ``"bar 'B6'"`` or ``"all 12 bar(s)"`` (shown in the prompt).

    Returns:
        tuple[bool, bool] | None: ``(erase_keyframe, erase_base)``, or ``None`` if
        the user cancelled (Esc).
    """
    choices = rs.GetBoolean(
        f"Choose what to erase on {scope_label}, then press Enter (Esc to cancel)",
        (("Keyframe", "Keep", "Erase"), ("BasePosition", "Keep", "Erase")),
        (True, True),
    )
    if choices is None:
        return None
    return bool(choices[0]), bool(choices[1])


def clear_ik(all_bars: bool) -> None:
    """Erase IK data on one picked bar (left-click) or every bar (right-click).

    Picks the scope, then prompts a two-toggle choice of WHAT to erase
    (``Keyframe`` / ``BasePosition``, both default Erase) and applies it to every
    target bar.

    Args:
        all_bars (bool): False = pick one bar (left-click); True = every registered
            bar (right-click).

    Returns:
        None.
    """
    config = importlib.reload(_config_module)
    registry = importlib.reload(_registry_module)

    title = "RSClearIKKeyframe" + (" (all bars)" if all_bars else "")
    repair_on_entry(float(config.BAR_RADIUS), title)

    # 1) Scope: one picked bar, or every registered bar.
    if all_bars:
        seq_map = registry.get_bar_seq_map()  # {bar_id: (oid, seq)}
        if not seq_map:
            rs.MessageBox("No registered bars in the document.", 0, title)
            return
        targets = [(bar_id, oid) for bar_id, (oid, _seq) in seq_map.items()]
        scope_label = f"all {len(targets)} bar(s)"
    else:
        rs.UnselectAllObjects()
        bar_oid = pick_bar("Pick a bar to clear its IK data (Esc to cancel)")
        if bar_oid is None:
            return
        bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY) or "<unregistered>"
        targets = [(bar_id, bar_oid)]
        scope_label = f"bar '{bar_id}'"

    # 2) What to erase -- two toggles, both default Erase.
    answer = _ask_what_to_erase(scope_label)
    if answer is None:
        print(f"{title}: cancelled at the erase prompt.")
        return
    clear_keyframe, clear_base = answer
    if not clear_keyframe and not clear_base:
        rs.MessageBox("Nothing selected to erase (both toggles set to Keep).", 0, title)
        return

    # 3) Clear each target and tally what actually changed.
    n_changed = 0
    total_removed = 0
    for bar_id, oid in targets:
        removed = registry.clear_assembly_ik_keyframe(
            oid, clear_keyframe=clear_keyframe, clear_base_frame=clear_base,
        )
        if removed:
            n_changed += 1
            total_removed += len(removed)
            print(f"{title}: bar '{bar_id}' cleared {removed}")
        else:
            print(f"{title}: bar '{bar_id}' had nothing matching to clear.")

    what = " + ".join(
        label for label, on in (("Keyframe", clear_keyframe), ("Base position", clear_base))
        if on
    )
    summary = (
        f"Erased {what} on {n_changed}/{len(targets)} bar(s) "
        f"({total_removed} user-text key(s) removed).\n"
        "M4 home config is a constant and was left untouched."
    )
    print(f"{title}: {summary}")
    rs.MessageBox(summary, 0, title)


def main() -> None:
    # Left-click: pick ONE bar, then choose what to erase.
    clear_ik(all_bars=False)


if __name__ == "__main__":
    main()
