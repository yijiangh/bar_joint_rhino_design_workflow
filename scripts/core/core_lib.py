"""Put the shared core `bar_assembly_core` (submodule `external/bar_assembly_core`) and the compas_fab it plans
with (submodule `external/compas_fab`, branch `wip_process`) on `sys.path`.

Import this module before any `bar_assembly_core` import; it needs only os and sys.

- ! Never `importlib.reload` a `bar_assembly_core` module, nor this one's import of it: Rhino commands reload their
  own modules on every run, but a reloaded core breaks `isinstance` checks and the core's caches.
- ! compas_fab comes from the submodule, never from `requirements-rhino.txt` (the pip cache would ignore SHA
  changes, `tasks/yh_lesson.md`). After switching its SHA, reload Rhino's Python engine.
"""

from __future__ import annotations

import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
#: The submodule folder that holds the `bar_assembly_core` package.
CORE_ROOT = os.path.join(_REPO_ROOT, "external", "bar_assembly_core")
#: The compas_fab submodule's package folder.
COMPAS_FAB_SRC = os.path.join(_REPO_ROOT, "external", "compas_fab", "src")


def ensure_compas_fab() -> None:
    """Put the compas_fab submodule first on `sys.path`; warn if another copy is already loaded.

    ? No import here: only the planning and drawing modules need compas_fab, the export does not.
    """
    if os.path.isdir(COMPAS_FAB_SRC) and COMPAS_FAB_SRC not in sys.path:
        sys.path.insert(0, COMPAS_FAB_SRC)
    loaded = getattr(sys.modules.get("compas_fab"), "__file__", None)
    if loaded and not os.path.normcase(loaded).startswith(os.path.normcase(COMPAS_FAB_SRC)):
        print(f"core.core_lib: WARNING compas_fab is loaded from {loaded}, not from the submodule {COMPAS_FAB_SRC}. "
              "Reload Rhino's Python engine (ScriptEditor -> Tools -> Reload Python 3 Engine).")


def ensure_core() -> None:
    """Put the core submodule first on `sys.path` and import it once; warn if another copy is already loaded.

    Raises:
        RuntimeError: If the submodule is not checked out.
    """
    if not os.path.isdir(os.path.join(CORE_ROOT, "bar_assembly_core")):
        raise RuntimeError(f"bar_assembly_core submodule missing at {CORE_ROOT}. "
                           "Run `git submodule update --init external/bar_assembly_core`.")
    if CORE_ROOT not in sys.path:
        sys.path.insert(0, CORE_ROOT)
    import bar_assembly_core  # noqa: F401  (first import resolves via sys.path)
    loaded = os.path.dirname(os.path.dirname(os.path.abspath(bar_assembly_core.__file__)))
    if os.path.normcase(loaded) != os.path.normcase(CORE_ROOT):
        print(f"core.core_lib: WARNING bar_assembly_core is loaded from {loaded}, not from the submodule "
              f"{CORE_ROOT}. Reload Rhino's Python engine to use the pinned copy.")


ensure_core()
ensure_compas_fab()
