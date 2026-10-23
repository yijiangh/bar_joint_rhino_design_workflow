#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSCheckMoCapLayout - find bars whose MoCap markers Motive could mix up.

Motive tells tracked bars apart only by the SHAPE their marker spheres form.  If
two bars carry their MoCap joints so that the spheres form the same shape, Motive
cannot say which bar is which.  Click this button and every bar carrying MoCap
joints is compared with every other one.

What it flags:

* **Look-alike bars** -- bars whose marker spheres can be laid onto each other by
  moving and turning one bar, within the tolerance asked at the prompt (default
  ``config.MOCAP_SIMILAR_TOL_MM``).  That includes the same placement turned end
  for end, which looks identical to Motive.  A mirror image does not count (a bar
  cannot be turned into its mirror), and bars with different marker counts are
  not compared.  Each group of look-alikes gets its own colour.
* **Bars the export would refuse** -- a bar whose OWN layout fits itself in two
  poses (fewer than three MoCap joints, all facing one way, ...): the check
  RSExportRobotCell runs, at its tolerance (``config.MOCAP_LAYOUT_TOL_MM``),
  shown here before the export stops on it.  Dark red.

Everything flagged is left SELECTED -- each bar's centre-line and tube, and its
MoCap joints -- and the report on the command line says which bars match and how
(``B22 = B20 end for end, within 0.4 mm``).  The rule is
``core.mocap_similarity``.

Marker positions come from the MoCap blocks placed in the model and the spheres
recorded on the block (RSDefineJointHalf), each in its own bar's frame -- the same
numbers RSExportPrefab writes.  No PyBullet, no export; nothing in the model
changes except colours.  The next run puts back the colours this one painted
over (unless something else repainted the object since); RSClearColorPreview
clears them too.
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import joint_name_conventions as jnc
from core import mocap_layout_check as _layout_check_module
from core import mocap_similarity as _similarity_module
from core import rhino_bar_registry as _registry_module
from core.joint_pair import canonical_bar_frame_from_line, load_joint_registry
from core.marker_points import markers_in_frame_mm
from core.mocap_markers import LAYOUT_HINT
from core.rhino_helpers import (
    NotABlockInstanceError,
    block_instance_xform_mm,
    curve_endpoints,
    doc_unit_scale_to_mm,
    objects_on_layers,
    paint_remembered,
    restore_painted,
    suspend_redraw,
)

# Command name used in every command-line message.
CMD = "RSCheckMoCapLayout"

#: ``sc.sticky`` key: what the last run painted over, so the next run can put it back.
_STICKY_PAINTED = "rs_check_mocap_layout:painted"


def _reload():
    """Re-import the modules this command uses so ScriptEditor picks up changes."""
    global similarity, registry
    importlib.reload(config)
    importlib.reload(_layout_check_module)
    similarity = importlib.reload(_similarity_module)
    registry = importlib.reload(_registry_module)


_reload()


# ---------------------------------------------------------------------------
# Reading the model
# ---------------------------------------------------------------------------


def _mocap_joints_by_bar(bars: dict) -> tuple:
    """``({bar_id: [(joint_id, oid), ...]}, problems)`` for every placed MoCap block."""
    by_bar, problems = {}, []
    for oid in objects_on_layers(jnc.joint_layer(jnc.MOCAP)):
        jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
        if not jid:
            continue
        bar_id = rs.GetUserText(oid, jnc.UT_PARENT_BAR) or ""
        if bar_id not in bars:
            problems.append(f"{jid}: its bar {bar_id or '<none>'} is not a registered bar; left out.")
            continue
        by_bar.setdefault(bar_id, []).append((jid, oid))
    return by_bar, problems


def _layouts_m(bars: dict, joints_by_bar: dict) -> tuple:
    """``({bar_id: (n, 3) marker centres}, problems)`` -- metres, each in its own bar frame.

    A bar with a MoCap block whose spheres were never recorded is left out
    (its layout would be incomplete) and named in *problems*.
    """
    halves = load_joint_registry().halves
    to_mm = doc_unit_scale_to_mm()
    layouts, problems = {}, []
    for bar_id, joints in joints_by_bar.items():
        start, end = curve_endpoints(bars[bar_id])
        bar_frame = canonical_bar_frame_from_line(start * to_mm, end * to_mm)
        points, unreadable = [], []
        for jid, oid in joints:
            half = halves.get(rs.BlockInstanceName(oid) or "")
            if half is None or not half.marker_points_mm:
                unreadable.append(jid)
                continue
            try:
                block_world = block_instance_xform_mm(oid)
            except NotABlockInstanceError:
                unreadable.append(jid)
                continue
            points.extend(markers_in_frame_mm(block_world, half.marker_points_mm, bar_frame).values())
        if unreadable:
            problems.append(
                f"{bar_id}: no recorded marker spheres for {', '.join(sorted(unreadable))} "
                "(RSDefineJointHalf on the block, pick the spheres); bar left out."
            )
            continue
        layouts[bar_id] = np.asarray(points, dtype=float) / 1000.0
    return layouts, problems


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _report(n_bars, tol_mm, groups, group_colors, pairs, refused, unchecked, problems) -> None:
    print(f"{CMD}: {n_bars} bar(s) with MoCap markers compared, look-alike tolerance {tol_mm:.1f} mm.")
    for index, (group, (name, _rgb)) in enumerate(zip(groups, group_colors), 1):
        members = set(group)
        n_markers = next(p.n_markers for p in pairs if p.bar_a in members)
        print(f"  Group {index} ({name}): {', '.join(group)} -- {n_markers} markers each; Motive could mix these up.")
        for pair in pairs:
            if pair.bar_a in members:
                print(f"    {pair.describe()}")
    for bar_id, reason in refused.items():
        print(f"  {bar_id} (dark red): the export would refuse it -- {reason}.")
    if refused:
        print(f"  {LAYOUT_HINT}")
    for bar_a, bar_b, reason in unchecked:
        print(f"  {bar_a} vs {bar_b}: not compared -- {reason}")
    for problem in problems:
        print(f"  NOTE: {problem}")
    if not groups and not refused:
        print(f"{CMD}: no look-alikes -- Motive can tell every MoCap bar apart.")
    else:
        print(
            f"{CMD}: flagged bars and their MoCap joints are selected; the colours stay "
            "until the next RSCheckMoCapLayout or RSClearColorPreview."
        )


def main() -> None:
    _reload()

    tol_mm = rs.GetReal(
        "Look-alike if markers match within (mm)",
        float(config.MOCAP_SIMILAR_TOL_MM),
        0.1,
    )
    if tol_mm is None:
        print(f"{CMD}: cancelled.")
        return

    bars = registry.get_all_bars()
    joints_by_bar, problems = _mocap_joints_by_bar(bars)
    layouts, layout_problems = _layouts_m(bars, joints_by_bar)
    problems += layout_problems

    with suspend_redraw():
        restore_painted(_STICKY_PAINTED)
        rs.UnselectAllObjects()
        if not layouts:
            print(f"{CMD}: no bar carries MoCap joints with recorded markers; nothing to compare.")
            for problem in problems:
                print(f"  NOTE: {problem}")
            return

        pairs, unchecked = similarity.compare_bars(layouts, tol_mm / 1000.0)
        groups = similarity.similar_groups(pairs)
        refused = similarity.refused_bars(layouts, float(config.MOCAP_LAYOUT_TOL_MM) / 1000.0)
        palette = config.MOCAP_SIMILAR_GROUP_COLORS
        group_colors = [palette[i % len(palette)] for i in range(len(groups))]

        # One colour per bar: its group's, or dark red when the export would
        # refuse it (that one stops the export, so it wins).
        color_by_bar = {}
        for group, (_name, rgb) in zip(groups, group_colors):
            for bar_id in group:
                color_by_bar[bar_id] = rgb
        for bar_id in refused:
            color_by_bar[bar_id] = config.MOCAP_REFUSED_COLOR

        color_by_oid = {}
        for bar_id, rgb in color_by_bar.items():
            for oid in registry._bar_curve_and_tube(bars[bar_id]):
                color_by_oid[oid] = rgb
            for _jid, oid in joints_by_bar[bar_id]:
                color_by_oid[oid] = rgb
        paint_remembered(color_by_oid, _STICKY_PAINTED)
        if color_by_oid:
            rs.SelectObjects(list(color_by_oid))

    _report(len(layouts), tol_mm, groups, group_colors, pairs, refused, unchecked, problems)


if __name__ == "__main__":
    main()
