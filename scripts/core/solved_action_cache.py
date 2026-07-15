"""Session hand-off of loaded solved BarActions: loader -> viewer.

``RSLoadSolvedBarAction`` reads the solved sidecar JSON(s) from disk and stores
the loaded ``BarAssemblyAction`` objects here (keyed by bar id) plus which kind
(``"keyframe"`` / ``"motion"``) was loaded. ``RSShowBarActionPlan`` reads them back to know
which bar(s) to show, and -- for ``"motion"`` -- to scrub the trajectories (which
are too large to keep in the on-curve user-text). Lives in ``sc.sticky`` so it
survives across the two commands within one Rhino session.
"""

from __future__ import annotations

import os

import scriptcontext as sc

from compas.data import json_load


_STICKY_ACTIONS = "bar_joint:loaded_solved_actions"   # {bar_id: BarAssemblyAction}
_STICKY_KIND = "bar_joint:loaded_solved_kind"          # "keyframe" | "motion"


def solved_action_filename(bar_id: str, kind: str) -> str:
    """Return the solved-sidecar filename for a bar, e.g. ``B6.solved_motion.json``."""
    return f"{bar_id}.solved_{kind}.json"


def load_solved_action_file(actions_dir: str, bar_id: str, kind: str):
    """Load one bar's solved sidecar from ``actions_dir`` (``None`` if absent).

    Args:
        actions_dir (str): the ``BarActions/`` folder.
        bar_id (str): the bar id.
        kind (str): ``"keyframe"`` or ``"motion"``.

    Returns:
        BarAssemblyAction | None: the loaded action, or None if the file is missing.
    """
    path = os.path.join(actions_dir, solved_action_filename(bar_id, kind))
    if not os.path.isfile(path):
        return None
    return json_load(path)


def set_loaded(actions_by_bar: dict, kind: str) -> None:
    """Store the loaded actions + kind for the viewer to pick up.

    Args:
        actions_by_bar (dict): ``{bar_id: BarAssemblyAction}``.
        kind (str): ``"keyframe"`` or ``"motion"``.
    """
    sc.sticky[_STICKY_ACTIONS] = dict(actions_by_bar)
    sc.sticky[_STICKY_KIND] = kind


def get_loaded():
    """Return ``(actions_by_bar, kind)`` from the cache (``({}, None)`` if empty)."""
    return dict(sc.sticky.get(_STICKY_ACTIONS) or {}), sc.sticky.get(_STICKY_KIND)


def clear_loaded() -> None:
    """Drop the cached loaded actions (e.g. when the viewer session ends)."""
    sc.sticky.pop(_STICKY_ACTIONS, None)
    sc.sticky.pop(_STICKY_KIND, None)
