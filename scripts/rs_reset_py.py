#! python 3
# venv: scaffolding_env
"""RSResetPy - Purge project modules from `sys.modules` so next import reloads from disk.

A lightweight substitute for the ScriptEditor's "Reload Python 3 (CPython)
Engine" menu action. A full engine reload also resets C-extension state and
the interpreter itself; this script only forgets imported pure-Python
modules under this repo's `scripts/` folder.

What gets purged:
  * Any module whose `__file__` lives under `scripts/` (all `core.*`, all `rs_*`).

What is deliberately NOT purged:
  * `compas`, `compas_robots`, `compas_fab`, `pybullet`, etc. Purging only
    `compas_fab` while `compas`/`compas_robots` remain loaded created
    class-identity bugs (e.g. `super(PyBulletBase, self)` failing) because
    sibling libraries hold references to the older class objects via the
    compas plugin registry. To pick up a new compas_fab submodule SHA, use
    ScriptEditor -> Tools -> Reload Python 3 (CPython) Engine instead.
  * PyBullet state. Pair with RSPBStop + RSPBStart if you need to reset
    the live PyBullet client.
"""

from __future__ import annotations

import os
import sys


SCRIPTS_ROOT = os.path.normcase(os.path.dirname(os.path.abspath(__file__)))


def _is_project_module(name: str, module) -> bool:
    file_attr = getattr(module, "__file__", None)
    if not file_attr:
        return False
    try:
        return os.path.normcase(file_attr).startswith(SCRIPTS_ROOT)
    except Exception:
        return False


def main() -> None:
    stale = [name for name, module in list(sys.modules.items()) if _is_project_module(name, module)]
    for name in stale:
        sys.modules.pop(name, None)
    print(f"RSResetPy: purged {len(stale)} module(s)")


if __name__ == "__main__":
    main()
