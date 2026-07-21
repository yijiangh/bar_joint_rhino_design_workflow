#! python 3
"""RSRegisterAliases - make every toolbar macro typeable from the command line.

Rhino toolbar macros only run when you click their button. To type a command
name (e.g. ``RSDefineRoboticTool``) in the command line instead, Rhino needs a
*command alias* -- a typed keyword that expands to a macro. Aliases live in
Rhino's per-user settings, NOT in the .rui, so this script bridges the two:

    For every ``<macro_item>`` in ``scaffolding_toolbar.rui`` it registers an
    alias whose NAME is the macro's command name (the ``<text>`` value, e.g.
    ``RSDefineRoboticTool``) and whose EXPANSION is the macro's ``<script>``
    line (the same ``! _-ScriptEditor _R "rs_*.py"`` the button runs).

Run this ONCE per machine (e.g. from the ScriptEditor:
``_-ScriptEditor _R "register_command_aliases.py"``). The aliases persist across
Rhino sessions. Re-run it whenever the toolbar macros change so the aliases
track them. It is idempotent: an existing alias of the same name is replaced.
"""

import os
import xml.etree.ElementTree as ET

import rhinoscriptsyntax as rs


# The toolbar file sits one level up from this scripts/ folder, at the repo root.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
RUI_PATH = os.path.join(REPO_ROOT, "scaffolding_toolbar.rui")

# Rhino stores UI text under an English (US) locale tag; every macro name and
# script line we care about lives inside this child element.
_LOCALE_TAG = "locale_1033"


def _macro_entries(rui_path: str):
    """Yield ``(command_name, macro_script)`` for every macro in the .rui.

    Args:
        rui_path (str): absolute path to the ``scaffolding_toolbar.rui`` file.

    Yields:
        tuple[str, str]: the macro's command name (e.g. ``RSDefineRoboticTool``)
        and its script line (e.g. ``! _-ScriptEditor _R "rs_define_robotic_tool.py"``).
        Macros missing either piece are skipped.
    """
    tree = ET.parse(rui_path)
    root = tree.getroot()
    # Walk every <macro_item> regardless of how deep it sits in the tree.
    for macro in root.iter("macro_item"):
        name_element = macro.find("text/" + _LOCALE_TAG)
        script_element = macro.find("script")
        if name_element is None or script_element is None:
            continue
        name = (name_element.text or "").strip()
        macro_script = (script_element.text or "").strip()
        if name and macro_script:
            yield name, macro_script


def main() -> None:
    """Register one command alias per toolbar macro, then print a summary.

    Returns:
        None.
    """
    if not os.path.isfile(RUI_PATH):
        print(f"RSRegisterAliases: toolbar file not found at {RUI_PATH}.")
        return

    added = 0
    replaced = 0
    failed = 0
    for name, macro_script in _macro_entries(RUI_PATH):
        # Replace an existing alias so re-runs pick up macro edits cleanly.
        already_present = rs.IsAlias(name)
        if already_present:
            rs.DeleteAlias(name)
        if rs.AddAlias(name, macro_script):
            if already_present:
                replaced += 1
            else:
                added += 1
        else:
            failed += 1
            print(f"  [alias] FAILED to register '{name}'.")

    print(
        f"RSRegisterAliases: {added} added, {replaced} replaced, {failed} failed."
    )
    if failed == 0:
        print(
            "  Done -- type any command name (e.g. RSDefineRoboticTool) in the "
            "Rhino command line to run it. Aliases persist across sessions."
        )


if __name__ == "__main__":
    main()
