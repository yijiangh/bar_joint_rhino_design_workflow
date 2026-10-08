"""``core.single_sided_placement`` -- Ground and standalone MoCap on one bar.

The default angle about the bar is the part worth pinning: a Ground joint's
foot (block +Y) must face the floor, and a MoCap joint's marker plate (block
+Z, the back of the Female it is built from) must face up -- or +X on a bar
too close to vertical for "up" to be reachable.  Each is checked against a
brute-force search over the angle.

Pure Python -- no Rhino.
"""

from __future__ import annotations

import numpy as np
import pytest

from core import joint_name_conventions as jnc
from core import single_sided_placement as ssp
from core.joint_pair import GroundJointDef, JointHalfDef, load_joint_registry

BARS = {
    "horizontal_x": ((0.0, 0.0, 500.0), (1000.0, 0.0, 500.0)),
    "horizontal_diag": ((0.0, 0.0, 0.0), (600.0, 800.0, 0.0)),
    "sloped": ((0.0, 0.0, 0.0), (500.0, 200.0, 700.0)),
}
VERTICAL = ((0.0, 0.0, 0.0), (5.0, 0.0, 1000.0))  # ~0.3 deg off vertical


@pytest.fixture(scope="module")
def registry():
    return load_joint_registry()


@pytest.fixture(scope="module")
def mocap(registry):
    """A MoCap half built from the registered T20_Female's frames."""
    female = registry.halves["T20_Female"]
    return JointHalfDef(
        block_name="T20_MoCap",
        M_block_from_bar=female.M_block_from_bar,
        M_screw_from_block=female.M_screw_from_block,
    )


@pytest.fixture(scope="module")
def ground(registry):
    return next(iter(registry.ground_joints.values()))


def _best_alignment(bar, definition, axis, target, flipped=False):
    """Brute force: the highest dot(block axis, target) over jr."""
    start, end = (np.asarray(p, dtype=float) for p in bar)
    best = -2.0
    for jr in np.linspace(-np.pi, np.pi, 3601):
        frame = ssp.fk_single_block_frame(start, end, 0.0, jr, definition, flipped=flipped)
        best = max(best, float(frame[:3, axis] @ np.asarray(target, dtype=float)))
    return best


def _alignment(bar, definition, jr, axis, target, flipped=False):
    start, end = (np.asarray(p, dtype=float) for p in bar)
    frame = ssp.fk_single_block_frame(start, end, 0.0, jr, definition, flipped=flipped)
    return float(frame[:3, axis] @ np.asarray(target, dtype=float))


@pytest.mark.parametrize("bar", list(BARS.values()), ids=list(BARS))
def test_mocap_plate_faces_up_as_far_as_the_bar_allows(bar, mocap):
    jr = ssp.auto_jr_mocap(*bar, mocap)
    up = (0.0, 0.0, 1.0)
    assert _alignment(bar, mocap, jr, ssp.MOCAP_PLATE_AXIS, up) == pytest.approx(
        _best_alignment(bar, mocap, ssp.MOCAP_PLATE_AXIS, up), abs=1e-5
    )


def test_mocap_plate_faces_straight_up_on_a_horizontal_bar(mocap):
    bar = BARS["horizontal_x"]
    jr = ssp.auto_jr_mocap(*bar, mocap)
    assert _alignment(bar, mocap, jr, ssp.MOCAP_PLATE_AXIS, (0, 0, 1)) == pytest.approx(1.0)


def test_mocap_on_a_vertical_bar_faces_plus_x(mocap):
    assert ssp.mocap_default_facing(*VERTICAL) == (1.0, 0.0, 0.0)
    jr = ssp.auto_jr_mocap(*VERTICAL, mocap)
    assert _alignment(VERTICAL, mocap, jr, ssp.MOCAP_PLATE_AXIS, (1, 0, 0)) == pytest.approx(
        _best_alignment(VERTICAL, mocap, ssp.MOCAP_PLATE_AXIS, (1, 0, 0)), abs=1e-5
    )


def test_mocap_facing_switches_at_15_degrees_from_vertical():
    def bar_at(deg_from_vertical):
        a = np.radians(deg_from_vertical)
        return (0.0, 0.0, 0.0), (1000.0 * np.sin(a), 0.0, 1000.0 * np.cos(a))

    assert ssp.mocap_default_facing(*bar_at(14.0)) == (1.0, 0.0, 0.0)
    assert ssp.mocap_default_facing(*bar_at(16.0)) == (0.0, 0.0, 1.0)


@pytest.mark.parametrize("flipped", [False, True])
@pytest.mark.parametrize("bar", list(BARS.values()), ids=list(BARS))
def test_ground_foot_faces_the_floor(bar, ground, flipped):
    jr = ssp.auto_jr_y_down(*bar, ground, flipped=flipped)
    down = (0.0, 0.0, -1.0)
    assert _alignment(bar, ground, jr, 1, down, flipped) == pytest.approx(
        _best_alignment(bar, ground, 1, down, flipped), abs=1e-5
    )


def test_flip_keeps_the_ground_angle(ground):
    bar = BARS["sloped"]
    assert ssp.auto_jr_y_down(*bar, ground, flipped=True) == pytest.approx(
        ssp.auto_jr_y_down(*bar, ground, flipped=False)
    )


def test_default_jr_dispatches_by_subtype(ground, mocap):
    bar = BARS["sloped"]
    assert ssp.default_jr(*bar, ground) == ssp.auto_jr_y_down(*bar, ground)
    assert ssp.default_jr(*bar, mocap) == ssp.auto_jr_mocap(*bar, mocap)


def test_only_ground_and_mocap_are_single_sided(registry, ground, mocap):
    assert ssp.subtype_of(ground) == jnc.GROUND
    assert ssp.subtype_of(mocap) == jnc.MOCAP
    with pytest.raises(ValueError):
        ssp.subtype_of(registry.halves["T20_Female"])


def test_definitions_are_found_by_block_name(registry, ground):
    assert ssp.find_definition(ground.block_name, registry) is ground
    assert ssp.find_definition("T20_Female", registry) is None
    assert ssp.find_definition("not a block", registry) is None
    names = [d.block_name for d in ssp.single_sided_definitions(registry)]
    assert names[0] == "T20_Ground"
    assert all(jnc.block_subtype(n) in jnc.SINGLE_SIDED_SUBTYPES for n in names)


def test_ground_definition_has_a_type():
    ground = GroundJointDef(name="X_Ground", block_name="X_Ground", M_block_from_bar=np.eye(4))
    assert ground.type == "X"
