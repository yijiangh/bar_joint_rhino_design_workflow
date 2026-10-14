"""Which bars' saved IK no longer fits the document: asked of the core, mapped back to bar ids.

The keyframes on the bar curves (`core.keyframe_store`) are world values: a robot base and joints. Nothing moves them
when the bar, a connector or a tool is moved afterwards, or when the bar is copied with its user text. The core
decides: `plan_check.unreached_targets` (plan check B15) compares each target's joints with the flange poses the
target asks for, which `core.process_source` reads from the document as it is now.

- `stale_ik_bars(process)` is Rhino-free; `document_stale_ik` reads the open document first.
- ! The verdict is computed from the document every time, never saved on the bars: a saved flag would go stale too.
- ? It says the saved pose misses its target, not that it collides: a bar added next to a solved one changes the
  scene, not the target. The collision check of the viewers (CheckCollision) answers that.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional

import session_log

from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path)
from bar_assembly_core.process import Process
from bar_assembly_core.process.plan_check import UnreachedTarget, unreached_targets

#: Action id suffix -> what the action is, for messages.
_ROLES = {"J": "assembly", "R": "assembly", "H": "hold", "HR": "hold"}


def stale_from(misses: Iterable[UnreachedTarget]) -> Dict[str, str]:
    """The core's unreached targets as `{bar id: reason}`, e.g. `{"B25": "assembly pose misses its target by 5.9 m"}`.

    Args:
        misses: `unreached_targets(process)` or a `PlanReport.unreached`.
    """
    worst: Dict[str, Dict[str, UnreachedTarget]] = {}
    for miss in misses:
        bar = miss.bar.split("/", 1)[-1]
        role = _ROLES.get(miss.action.rsplit("_", 1)[-1], "saved")
        seen = worst.setdefault(bar, {})
        if role not in seen or (miss.distance, miss.angle) > (seen[role].distance, seen[role].angle):
            seen[role] = miss
    return {bar: "; ".join(f"{role} pose misses its target by {_how_far(miss)}" for role, miss in roles.items())
            for bar, roles in worst.items()}


def _how_far(miss: UnreachedTarget) -> str:
    import math  # noqa: PLC0415

    if miss.distance >= 1.0:
        return f"{miss.distance:.1f} m"
    if miss.distance >= 1e-4:
        return f"{miss.distance * 1000.0:.1f} mm"
    return f"{math.degrees(miss.angle):.2f} deg"


@session_log.timed("check saved IK against the document (core B15)")
def stale_ik_bars(process: Process) -> Dict[str, str]:
    """`{bar id: reason}` of the bars whose saved IK does not reach its target in this process; empty when all do."""
    return stale_from(unreached_targets(process))


def document_stale_ik(command: str = "RSScaffolding") -> Optional[Dict[str, str]]:
    """`stale_ik_bars` of the open document.

    Returns:
        dict | None: None when the document cannot become a process (duplicate ids, a broken block): the reason is
        printed, and nothing can be said about its saved IK.
    """
    import scriptcontext as sc  # noqa: PLC0415

    from core import process_source  # noqa: PLC0415
    from bar_assembly_core.process import ProcessError  # noqa: PLC0415

    try:
        result = process_source.RhinoProcessSource(command, process_source.geometry_cache(sc.sticky)).build()
        return stale_ik_bars(result.process)
    except ProcessError as exc:
        session_log.caught(exc)
        print(f"{command}: saved IK not checked, the document cannot become a process: "
              + "; ".join(getattr(exc, "problems", None) or [str(exc)]))
    except Exception as exc:  # noqa: BLE001 -- a report must never end the command that asks for it
        session_log.caught(exc)
        print(f"{command}: saved IK not checked ({type(exc).__name__}: {exc}).")
    return None


def stale_ik_lines(stale: Optional[Dict[str, str]], limit: int = 8) -> List[str]:
    """The lines that report stale bars; none when there are none."""
    if not stale:
        return []
    lines = [f"STALE IK on {len(stale)} bar(s): the saved robot pose no longer reaches the bar's tools (the bar, a "
             "joint or a tool moved after the solve, or the bar was copied). Solve them again with RSIKKeyframe."]
    lines += [f"  {bar}: {reason}" for bar, reason in list(stale.items())[:limit]]
    if len(stale) > limit:
        lines.append(f"  ... and {len(stale) - limit} more: {', '.join(list(stale)[limit:])}")
    return lines
