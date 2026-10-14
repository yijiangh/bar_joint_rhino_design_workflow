#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSResetPlanning - Close every planning world and forget the cached geometry and previews; stop the ssik sidecar.

No command needs this in normal use: the IK commands and viewers build each robot's planning mirror (its own
PyBullet world, owned by the shared core) on first use, keep it for the session, and rebuild it when the document's
geometry or a mesh file changes. Run it when something looks stuck: every mirror, the geometry cache and the preview
meshes are dropped, and the ssik IK sidecar (a Python 3.11 process) is stopped. The next IK command or viewer
rebuilds what it needs.
"""

from __future__ import annotations

import importlib
import os
import sys

import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

from core import config  # noqa: F401  (puts the tamp submodule on sys.path)
from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path; never reloaded)
from core import process_planning as _process_planning_module
from core import process_source as _process_source_module
from core import ik_viz as _ik_viz_module


def close_all(store) -> None:
    """Close every planning mirror, forget the geometry cache and the preview meshes."""
    process_planning = importlib.reload(_process_planning_module)
    process_source = importlib.reload(_process_source_module)
    ik_viz = importlib.reload(_ik_viz_module)
    process_planning.close_mirrors(store)
    store.pop(process_source.GEOMETRY_CACHE_KEY, None)
    ik_viz.discard_cache()


def main() -> None:
    close_all(sc.sticky)
    try:
        from husky_assembly_tamp.keyframe import ssik_client
        ssik_client.shutdown()
    except Exception as exc:  # noqa: BLE001 -- nothing to stop, or tamp missing
        session_log.caught(exc)
        print(f"RSResetPlanning: ssik sidecar not stopped ({exc}).")
    print("RSResetPlanning: closed the planning worlds and dropped the cached geometry and previews.")


if __name__ == "__main__":
    session_log.run(__file__, main)
