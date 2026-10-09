"""Bar length groups: bars binned by length, colored and tagged on screen.

Shared by RSBarSelect > SelectByLength (choose a group to select) and
RSBarEdit > BarLength (see the groups while resizing), so a length group means
the same set of bars in both commands.

Lengths are in document units and assumed to be mm, like every bar command.
Rhino-runtime only.
"""

from __future__ import annotations

import colorsys
from collections import defaultdict

import numpy as np
import rhinoscriptsyntax as rs

from core.rhino_bar_registry import (
    _bar_curve_and_tube,
    paint_bar,
    restore_object_colors,
    snapshot_object_colors,
)
from core.rhino_helpers import curve_endpoints, delete_objects, suspend_redraw

#: Bars whose lengths round to the same multiple of this are one group.
LENGTH_BIN_MM = 1.0

#: Fallback color when there are no groups to spread hues over.
_NO_GROUP_COLOR = (200, 200, 200)


def bar_length(curve_id) -> float:
    start, end = curve_endpoints(curve_id)
    return float(np.linalg.norm(end - start))


def bin_length(length_mm: float) -> float:
    return round(length_mm / LENGTH_BIN_MM) * LENGTH_BIN_MM


def group_color(index: int, count: int) -> tuple:
    """A distinct RGB color for group *index* of *count* (evenly spaced hues)."""
    if count <= 0:
        return _NO_GROUP_COLOR
    r, g, b = colorsys.hsv_to_rgb((index / float(count)) % 1.0, 0.65, 0.95)
    return int(r * 255), int(g * 255), int(b * 255)


def build_length_groups(bar_map: dict):
    """Group ``{bar_id: curve_id}`` by length.

    Returns ``(groups, color_by_bin, length_per_bar)``:
    ``groups`` is ``[(length_bin, [bar_id, ...]), ...]`` shortest first,
    ``color_by_bin`` is ``{length_bin: (r, g, b)}``,
    ``length_per_bar`` is ``{bar_id: length_mm}``.
    """
    length_per_bar = {bar_id: bar_length(oid) for bar_id, oid in bar_map.items()}
    bin_to_bars = defaultdict(list)
    for bar_id, length in length_per_bar.items():
        bin_to_bars[bin_length(length)].append(bar_id)
    bins = sorted(bin_to_bars)
    groups = [(b, sorted(bin_to_bars[b])) for b in bins]
    color_by_bin = {b: group_color(i, len(bins)) for i, b in enumerate(bins)}
    return groups, color_by_bin, length_per_bar


def find_length_group(groups, typed_mm: float):
    """Index of the group *typed_mm* falls in, or ``None``.

    The typed value is rounded to the same bin as the grouping.  No nearest
    match: selecting the wrong 20 bars is worse than selecting none.
    """
    target = bin_length(float(typed_mm))
    for i, (length_bin, _bar_ids) in enumerate(groups):
        if abs(length_bin - target) < LENGTH_BIN_MM * 0.5:
            return i
    return None


def available_lengths(groups) -> str:
    return ", ".join(f"{length:.0f}" for length, _ in groups)


def print_length_summary(groups) -> None:
    """Print each group: its length, count and bar ids."""
    print("\n--- Bar Length Groups ---")
    for length, bar_ids in groups:
        print(f"  {length:.0f} mm  x{len(bar_ids)}  : {','.join(bar_ids)}")
    print(f"  Total bars: {sum(len(b) for _, b in groups)}")
    print("--- End ---\n")


def paint_length_groups(bar_map, color_by_bin, length_per_bar) -> None:
    """Paint each bar's centre line and tube in its group's color."""
    with suspend_redraw():
        for bar_id, oid in bar_map.items():
            paint_bar(oid, color_by_bin[bin_length(length_per_bar[bar_id])])


def add_length_dots(bar_map, length_per_bar, name_prefix: str) -> list:
    """A text dot ``"<bar_id>\\n<length>mm"`` at each bar's midpoint.

    Dots are named ``<name_prefix>_<bar_id>``.  Returns their ids.
    """
    dot_ids = []
    with suspend_redraw():
        for bar_id, oid in bar_map.items():
            start, end = curve_endpoints(oid)
            mid = (start + end) * 0.5
            dot_id = rs.AddTextDot(
                f"{bar_id}\n{length_per_bar[bar_id]:.0f}mm",
                (float(mid[0]), float(mid[1]), float(mid[2])),
            )
            if dot_id:
                rs.ObjectName(dot_id, f"{name_prefix}_{bar_id}")
                dot_ids.append(dot_id)
    return dot_ids


class LengthPreview:
    """Bars colored by length group and tagged with their length, until closed.

    ``close()`` puts every bar's exact previous color back (an IK, fake-bar or
    sequence color survives) and deletes the dots.  For commands that do not
    change bar geometry while the preview is up.
    """

    def __init__(self, bar_map, color_by_bin, length_per_bar, name_prefix: str):
        objects = [o for oid in bar_map.values() for o in _bar_curve_and_tube(oid)]
        self._colors = snapshot_object_colors(objects)
        paint_length_groups(bar_map, color_by_bin, length_per_bar)
        self._dots = add_length_dots(bar_map, length_per_bar, name_prefix)
        rs.Redraw()

    def close(self) -> None:
        with suspend_redraw():
            delete_objects(self._dots)
            restore_object_colors(self._colors)
        self._dots, self._colors = [], []
