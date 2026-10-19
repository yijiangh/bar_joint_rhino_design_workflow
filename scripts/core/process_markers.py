"""
The MoCap markers of a process: each bar's marker points in its own frame, the layout check, and the Rhino labels.

The markers travel in `process.json` on the halves: `BodySpec.markers` holds each sphere centre in the half's own
frame, in metres (`process_source` reads them from the joint registry). The teleop monitor fits the measured markers
onto each bar's points (husky-assembly-teleop `world/marker_fit.py`, `bar_markers`); `bar_markers` here computes the
same points, under the same names and in the same order, so a Rhino label and a monitor marker always agree.

- A marker's name is `<half id>/<label>`, e.g. `joints/J20-21_mocap/M1` (`joint_name_conventions.marker_name`).
- A bar's frame is its process body's `assembly_pose`: at the bar MIDPOINT, z along the bar. The retired
  `MoCapMarkers.json` used the bar START (`p_old = p_new + (0, 0, L/2)`).
- ! RSExportProcess refuses a process whose markers the monitor could not read: a MoCap half without markers, or a bar
  whose layout fits itself in a second pose (`core.mocap_layout_check`, which also counts fewer than 3 markers).

Pure numpy and the core, no Rhino: `core.rhino_mocap_markers` draws the labels.
"""

from __future__ import annotations

from typing import Dict, List, NamedTuple, Tuple

import numpy as np

from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path)
from core.joint_name_conventions import MOCAP, marker_name
from core.mocap_layout_check import check_layout
from scaffolding_assembly_core.geometry import compose, invert
from scaffolding_assembly_core.process import Process

#: The monitor fits a bar from 3 markers at least; it leaves out a bar with fewer.
MIN_MARKERS = 3

#: Shown with a refused layout: how to fix it.
LAYOUT_HINT = ("Rule of thumb: at least three MoCap joints per tracked bar, not all facing the same way, not "
               "mirror-symmetric about the bar's middle. Add a MoCap joint turned about the bar (RSJointPlace > "
               "JointOnly > MoCap, Rotate), or move one, then export again.")


class MarkerLabel(NamedTuple):
    """One designed marker as the Rhino label shows it.

    Attributes:
        name: The monitor's marker name, e.g. "joints/J20-21_mocap/M1": the dot's object name.
        text: The dot's text, e.g. "B20 #3 M1": the bar, the marker's number on it, its label.
        bar: The bar's body id, e.g. "bars/B20".
        number: The marker's number on its bar, from 1, in the monitor's order (its model index + 1).
        world: The marker in the world, metres, shape (3,).
    """

    name: str
    text: str
    bar: str
    number: int
    world: np.ndarray


def is_mocap_half(part: str) -> bool:
    """Whether a body's `part` (e.g. "T20/MoCap") names a MoCap half."""
    return part.rpartition("/")[2] == MOCAP


def bar_markers(process: Process) -> Dict[str, List[Tuple[str, np.ndarray]]]:
    """Each bar's markers, from the halves mounted on it, in the bar's frame, as the monitor's `bar_markers`.

    ! Unlike the monitor's, a bar with fewer than `MIN_MARKERS` markers is kept, not dropped: `check_layouts` names it
    as too few.

    Args:
        process: The process; `BodySpec.markers` of each half, its `mount` bar and both assembly poses.

    Returns:
        dict[str, list[tuple[str, np.ndarray]]]: Bar id -> [(marker name "<half id>/<label>", point in the bar frame,
            metres, shape (3,))], bars sorted, markers by half id then label (the monitor's order).
    """
    out: Dict[str, List[Tuple[str, np.ndarray]]] = {}
    for half_id in sorted(process.bodies):
        half = process.bodies[half_id]
        if not half.markers or half.mount not in process.bodies:
            continue
        # * A mounted half stays where it sits on its bar, so the two assembly poses give its offset on the bar.
        bar_from_half = compose(invert(process.bodies[half.mount].assembly_pose), half.assembly_pose).matrix()
        for label in sorted(half.markers):
            point = bar_from_half[:3, :3] @ np.asarray(half.markers[label], dtype=float) + bar_from_half[:3, 3]
            out.setdefault(half.mount, []).append((marker_name(half_id, label), point))
    return {bar: out[bar] for bar in sorted(out)}


def check_layouts(process: Process, tol_mm: float) -> List[str]:
    """Why the process's markers cannot go to the monitor, one line each; empty when every MoCap half and bar passes.

    Refused, as the retired `MoCapMarkers.json` export did:
    - a MoCap half with no markers (its block's spheres were never picked);
    - a bar with too few markers (under `MIN_MARKERS`): the monitor leaves it out;
    - a bar whose markers another rigid motion maps onto themselves within `tol_mm` (`mocap_layout_check`): all on
      one line, or a symmetric layout. The monitor could fit that bar in a wrong pose.

    Args:
        process: The process about to be written.
        tol_mm: The layout tolerance, mm (`config.MOCAP_LAYOUT_TOL_MM`); the check itself works in metres.
    """
    problems = [f"{half_id}: MoCap half ({body.part}) without markers; run RSDefineJointHalf on its block and pick "
                f"the marker spheres" for half_id, body in sorted(process.bodies.items())
                if is_mocap_half(body.part) and not body.markers]
    for bar, markers in bar_markers(process).items():
        if len(markers) < MIN_MARKERS:
            problems.append(f"{bar}: too few MoCap markers ({len(markers)}), the monitor fits a bar from "
                            f"{MIN_MARKERS} at least")
            continue
        report = check_layout(np.array([point for _name, point in markers]), tol_mm / 1000.0)
        if not report.ok:
            problems.append(f"{bar}: MoCap marker layout is ambiguous -- {report.reason()}")
    return problems


def marker_labels(process: Process) -> List[MarkerLabel]:
    """Every designed marker with its label text and world position, for the dots RSExportProcess draws.

    The world point is the bar's assembly pose applied to the `bar_markers` point, so the dots show the monitor's
    numbers.
    """
    labels = []
    for bar, markers in bar_markers(process).items():
        world_from_bar = process.bodies[bar].assembly_pose.matrix()
        for number, (name, point) in enumerate(markers, 1):
            text = f"{bar.rpartition('/')[2]} #{number} {name.rpartition('/')[2]}"
            labels.append(MarkerLabel(name, text, bar, number, world_from_bar[:3, :3] @ point + world_from_bar[:3, 3]))
    return labels


__all__ = [
    "LAYOUT_HINT",
    "MIN_MARKERS",
    "MarkerLabel",
    "bar_markers",
    "check_layouts",
    "is_mocap_half",
    "marker_labels",
]
