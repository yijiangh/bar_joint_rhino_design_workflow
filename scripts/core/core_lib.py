"""Put the shared core `bar_assembly_core` (submodule `external/bar_assembly_core`) on `sys.path`.

Import this module before any `bar_assembly_core` import; it needs only os and sys.

- ! Never `importlib.reload` a `bar_assembly_core` module, nor this one's import of it: Rhino commands reload their
  own modules on every run, but a reloaded core breaks `isinstance` checks and the core's caches.
"""

from __future__ import annotations

import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
#: The submodule folder that holds the `bar_assembly_core` package.
CORE_ROOT = os.path.join(_REPO_ROOT, "external", "bar_assembly_core")


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
