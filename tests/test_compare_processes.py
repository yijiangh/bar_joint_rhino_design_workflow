"""`tests/equivalence/compare_processes.py`: two process folders compared field by field, on the builder's fixture.

The three-bar process of `test_process_builder.py`, written to folders: against itself it is equal; with one bar moved
by 1 mm and one drive changed it reports exactly those two differences.
"""

from dataclasses import replace

import pytest

from core.process_builder import build_actions
from equivalence.compare_processes import compare, main, without_environment
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process import write
from test_process_builder import CINDY_URDF, PKG, SCHEDULE, _process, _records

pytestmark = pytest.mark.skipif(not CINDY_URDF.is_file(), reason="asset/husky_urdf submodule not checked out")


@pytest.fixture(scope="module")
def process():
    """The builder fixture with its actions, in memory."""
    process = _process()
    bars, holds = _records(process)
    return build_actions(process, SCHEDULE, bars, holds).process


def _moved_and_loosened(process):
    """B1 moved 1 mm up; B2's insert loosens instead of tightening."""
    bar = process.bodies["bars/B1"]
    x, y, z = bar.assembly_pose.position
    moved = replace(bar, assembly_pose=Pose((x, y, z + 0.001), bar.assembly_pose.orientation))
    action = process.actions["B2_J"]
    insert = action.movements[-1]
    assert insert.id == "B2_J_insert" and insert.drives == {"tools/cindy/AT3L": "tighten"}
    loosened = replace(insert, drives={"tools/cindy/AT3L": "loosen"})
    action = replace(action, movements=action.movements[:-1] + (loosened,))
    return replace(process, bodies={**process.bodies, moved.id: moved}, actions={**process.actions, action.id: action})


def test_folders_equal_and_two_changes_found(process, tmp_path, capsys):
    """Exit 0 against itself; against the changed copy exit 1 with exactly the moved body and the changed drive."""
    write(process, tmp_path / "a", package_dirs=[PKG])
    write(_moved_and_loosened(process), tmp_path / "b", package_dirs=[PKG])

    assert main([str(tmp_path / "a"), str(tmp_path / "a")]) == 0
    assert "EQUAL: 0 difference(s)" in capsys.readouterr().out

    assert main([str(tmp_path / "a"), str(tmp_path / "b")]) == 1
    out = capsys.readouterr().out
    assert "NOT EQUAL: 2 difference(s) in 2 kind(s)" in out
    assert "bars/B1: body.assembly_pose: pose differs by 1 mm, 0 deg" in out
    assert "B2_J_insert: movement.drives[tools/cindy/AT3L]: 'tighten' vs 'loosen'" in out


def test_report_counts_by_group_and_kind(process):
    report = compare(process, _moved_and_loosened(process), package_dirs=[PKG])
    assert report.count() == 2
    assert {group: dict(kinds) for group, kinds in report.kinds.items() if kinds} == {
        "bodies": {"body.assembly_pose": ["bars/B1: body.assembly_pose: pose differs by 1 mm, 0 deg"]},
        "movements": {"movement.drives[*]": ["B2_J_insert: movement.drives[tools/cindy/AT3L]: 'tighten' vs 'loosen'"]},
    }
    assert report.compared["bodies"] == len(process.bodies)
    assert report.compared["movements"] == sum(len(a.movements) for a in process.actions.values())


def test_headless_gaps_leave_out_ground_and_producer(process):
    """A build without the walkable ground differs in its body, mates, actions' ground and present; skipped, equal."""
    headless = without_environment(process)
    report = compare(process, headless, package_dirs=[PKG])
    assert report.kinds["bodies"] == {"only in A": ["ground/WG0: only in A"]}
    assert len(report.kinds["mates"]["only in A"]) == 3
    assert set(report.kinds["actions"]) == {"action.ground"}
    assert set(report.kinds["movements"]) == {"movement.start.present"}
    assert compare(process, headless, headless_gaps=True, package_dirs=[PKG]).count() == 0
