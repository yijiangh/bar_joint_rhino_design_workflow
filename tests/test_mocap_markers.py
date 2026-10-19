"""`core.process_markers`: the MoCap markers of a process, as the monitor reads them, and the export's layout check.

The real `T20_MoCap` registry points on synthetic bars, as bodies the way `core.process_source` writes them: a bar at
its midpoint frame (`bar_pose`), a MoCap half at its block transform (`pose_from_mm`) with the registry's markers in
metres, mounted on its bar. Pure numpy and the core.
"""

from __future__ import annotations

import numpy as np
import pytest

from core import core_lib  # noqa: F401
from core import config
from core import process_markers as pm
from core.joint_pair import canonical_bar_frame_from_line, load_joint_registry
from core.marker_points import markers_in_frame_mm
from core.process_source import bar_pose, pose_from_mm
from core.single_sided_placement import fk_single_block_frame
from scaffolding_assembly_core.geometry import BoxShape, CylinderShape, Geometry
from scaffolding_assembly_core.process import BodySpec, Process, Writer

T20_MOCAP = load_joint_registry().halves["T20_MoCap"]
TOL_MM = float(config.MOCAP_LAYOUT_TOL_MM)
BOX = Geometry((BoxShape((0.02, 0.02, 0.02)),), (BoxShape((0.02, 0.02, 0.02)),))

#: Bar id -> (start, end), mm.
BARS = {
    "B20": ((1325.0, -2250.0, 150.0), (-475.0, -2250.0, 150.0)),   # horizontal, 1800 mm
    "B21": ((925.0, -2314.0, 50.0), (925.0, -2314.0, 1850.0)),     # vertical, 1800 mm
    "B22": ((0.0, 0.0, 0.0), (0.0, 1000.0, 0.0)),                   # one MoCap half: too few markers
    "B23": ((0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)),                   # no MoCap half
}
#: MoCap halves: (joint id, bar, mm along the bar from its start, turn about the bar in degrees).
#: B20's are listed out of order along the bar on purpose; three joints, mixed facing.
MOCAP = [("J20-169", "B20", 1145.3, 0), ("J20-22", "B20", 84.9, 90), ("J20-21", "B20", 420.0, 180),
         ("J21-37", "B21", 1471.9, 0), ("M21-T20-0", "B21", 396.0, 45), ("J21-26", "B21", 1080.0, 270),
         ("M22-T20-0", "B22", 500.0, 0)]


def _bar(bar: str) -> BodySpec:
    """A bar body as `process_source` reads it: a cylinder at its midpoint frame."""
    start, end = (np.asarray(p, dtype=float) for p in BARS[bar])
    length_mm = float(np.linalg.norm(end - start))
    cylinder = CylinderShape(0.01, length_mm / 1000.0)
    return BodySpec(f"bars/{bar}", bar_pose(canonical_bar_frame_from_line(start, end), length_mm),
                    Geometry((cylinder,), (cylinder,)))


def _block_mm(bar: str, along_mm: float, turn_deg: float) -> np.ndarray:
    """A MoCap block's world transform (mm), placed on `bar` the way RSJointPlace places it."""
    start, end = (np.asarray(p, dtype=float) for p in BARS[bar])
    return fk_single_block_frame(start, end, along_mm, np.radians(turn_deg), T20_MOCAP)


def _mocap_half(jid: str, bar: str, along_mm: float, turn_deg: float) -> BodySpec:
    """A MoCap half body as `process_source` reads it: block transform, registry markers in metres, its bar."""
    markers = {label: tuple(c / 1000.0 for c in point) for label, point in T20_MOCAP.marker_points_mm.items()}
    return BodySpec(f"joints/{jid}_mocap", pose_from_mm(_block_mm(bar, along_mm, turn_deg)), BOX, part="T20/MoCap",
                    markers=markers, mount=f"bars/{bar}")


def _process(halves) -> Process:
    """Every bar of `BARS`, the given halves, and a male half without markers on B20."""
    bodies = {f"bars/{bar}": _bar(bar) for bar in BARS}
    bodies.update({half.id: half for half in halves})
    male = BodySpec("joints/J20-21_male", pose_from_mm(_block_mm("B20", 420.0, 0)), BOX, part="T20/Male",
                    mount="bars/B20")
    bodies[male.id] = male
    return Process(None, Writer(2.1, "test", "0", False), {}, {}, bodies, (), {})


@pytest.fixture
def process() -> Process:
    return _process([_mocap_half(*half) for half in MOCAP])


def _apply(pose, point) -> np.ndarray:
    matrix = pose.matrix()
    return matrix[:3, :3] @ np.asarray(point, dtype=float) + matrix[:3, 3]


# * ---------------------------------------------------------------------------
# * bar_markers: names, order and frame, as the monitor's
# * ---------------------------------------------------------------------------


def test_names_and_order_follow_the_monitor(process):
    """`<half id>/<label>`, halves by id then labels: the monitor's model order. Halves without markers add none."""
    markers = pm.bar_markers(process)
    assert list(markers) == ["bars/B20", "bars/B21", "bars/B22"]
    assert [name for name, _ in markers["bars/B20"]] == [
        "joints/J20-169_mocap/M1", "joints/J20-169_mocap/M2", "joints/J20-21_mocap/M1", "joints/J20-21_mocap/M2",
        "joints/J20-22_mocap/M1", "joints/J20-22_mocap/M2"]
    assert [name for name, _ in markers["bars/B21"]][::2] == [
        "joints/J21-26_mocap/M1", "joints/J21-37_mocap/M1", "joints/M21-T20-0_mocap/M1"]


def test_points_are_in_the_bar_frame_at_its_midpoint(process):
    """The bar pose, at the bar's middle, puts each point where the half pose puts its marker."""
    for bar, markers in pm.bar_markers(process).items():
        start, end = (np.asarray(p, dtype=float) for p in BARS[bar.rpartition("/")[2]])
        assert process.bodies[bar].assembly_pose.position == pytest.approx((start + end) / 2000.0, abs=1e-9)
        for name, point in markers:
            half_id, _, label = name.rpartition("/")
            half = process.bodies[half_id]
            assert _apply(process.bodies[bar].assembly_pose, point) == pytest.approx(
                _apply(half.assembly_pose, half.markers[label]), abs=1e-9)


def test_old_bar_start_frame_is_half_a_bar_further(process):
    """The retired `MoCapMarkers.json` had the bar START as origin: `p_old = p_new + (0, 0, L/2)`, for B20."""
    start, end = (np.asarray(p, dtype=float) for p in BARS["B20"])
    old_frame = canonical_bar_frame_from_line(start, end)
    half_length = process.bodies["bars/B20"].geometry.collision[0].height / 2.0
    for name, point in pm.bar_markers(process)["bars/B20"]:
        jid = name.split("/")[1][:-len("_mocap")]
        _, bar, along_mm, turn_deg = next(h for h in MOCAP if h[0] == jid)
        old_mm = markers_in_frame_mm(_block_mm(bar, along_mm, turn_deg), T20_MOCAP.marker_points_mm, old_frame)
        assert np.asarray(old_mm[name.rpartition("/")[2]]) / 1000.0 == pytest.approx(
            point + (0.0, 0.0, half_length), abs=1e-5)


def test_a_bar_with_too_few_markers_is_kept_and_refused(process):
    """B22's one half gives 2 markers: listed, unlike the monitor, which drops it; the check names it."""
    assert len(pm.bar_markers(process)["bars/B22"]) == 2
    assert pm.check_layouts(process, TOL_MM) == [
        "bars/B22: too few MoCap markers (2), the monitor fits a bar from 3 at least"]


# * ---------------------------------------------------------------------------
# * check_layouts: what RSExportProcess refuses
# * ---------------------------------------------------------------------------


def test_good_layouts_pass():
    assert pm.check_layouts(_process([_mocap_half(*h) for h in MOCAP if h[1] != "B22"]), TOL_MM) == []


def test_joints_facing_one_way_are_refused_by_bar():
    """B20's three joints turned alike: M1 and M2 swap under a half-turn, the layout fits itself twice."""
    halves = [_mocap_half(jid, bar, along, 0 if bar == "B20" else turn) for jid, bar, along, turn in MOCAP
              if bar != "B22"]
    problems = pm.check_layouts(_process(halves), TOL_MM)
    assert len(problems) == 1 and problems[0].startswith("bars/B20: MoCap marker layout is ambiguous -- ")


def test_two_joints_are_refused():
    """Two joints are never enough, whatever their spacing and turn."""
    halves = [_mocap_half(*h) for h in MOCAP if h[1] == "B21" and h[0] != "J21-26"]
    problems = pm.check_layouts(_process(halves), TOL_MM)
    assert len(problems) == 1 and "bars/B21: MoCap marker layout is ambiguous" in problems[0]


def test_mocap_half_without_markers_is_refused():
    """A MoCap block whose spheres were never picked: named by its half."""
    bare = BodySpec("joints/J20-30_mocap", pose_from_mm(_block_mm("B20", 10.0, 0)), BOX, part="T40/MoCap",
                    mount="bars/B20")
    halves = [_mocap_half(*h) for h in MOCAP if h[1] != "B22"] + [bare]
    assert pm.check_layouts(_process(halves), TOL_MM) == [
        ("joints/J20-30_mocap: MoCap half (T40/MoCap) without markers; run RSDefineJointHalf on its block and pick "
         "the marker spheres")]


# * ---------------------------------------------------------------------------
# * marker_labels: the Rhino dots
# * ---------------------------------------------------------------------------


def test_labels_number_markers_per_bar_in_the_monitor_order(process):
    """`B20 #3 M1`: the third marker of B20 in the monitor's order; the dot sits on the half's marker."""
    labels = pm.marker_labels(process)
    assert len(labels) == 14
    third = labels[2]
    assert (third.name, third.text, third.bar, third.number) == (
        "joints/J20-21_mocap/M1", "B20 #3 M1", "bars/B20", 3)
    assert [label.text for label in labels if label.bar == "bars/B22"] == ["B22 #1 M1", "B22 #2 M2"]
    half = process.bodies["joints/J20-21_mocap"]
    assert third.world == pytest.approx(_apply(half.assembly_pose, half.markers["M1"]), abs=1e-9)
