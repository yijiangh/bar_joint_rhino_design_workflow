"""Round-trip the writer in `rs_export_grasp_tool0_tf.py` without Rhino.

Exercises:
- both dicts emit-and-import correctly
- partial re-runs preserve the other dict and the unrelated entries within
  the same dict
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
import tempfile
import types

import numpy as np


TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.abspath(os.path.join(TESTS_DIR, "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

# Module imports rhinoscriptsyntax / Rhino / scriptcontext at top-level
# (directly, or transitively via `core.rhino_frame_io`). Stub them so the
# writer helpers are reachable headless.
for _name in ("rhinoscriptsyntax", "Rhino", "scriptcontext"):
    sys.modules.setdefault(_name, types.ModuleType(_name))


def _load_exporter():
    spec = importlib.util.spec_from_file_location(
        "rs_export_grasp_tool0_tf",
        os.path.join(SCRIPTS_DIR, "rs_export_grasp_tool0_tf.py"),
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_LOAD_COUNTER = [0]


def _load_generated_module(path: str):
    """Load a module from a fixed file by exec'ing its source.

    Avoids importlib's bytecode cache, which can misbehave when the same
    file path is rewritten in quick succession during a single test run.
    """
    _LOAD_COUNTER[0] += 1
    with open(path, "r", encoding="utf-8") as fh:
        source = fh.read()
    name = f"_generated_under_test_{_LOAD_COUNTER[0]}"
    module = types.ModuleType(name)
    module.__file__ = path
    exec(compile(source, path, "exec"), module.__dict__)
    return module


def _make_male_tf(translation):
    tf = np.eye(4, dtype=float)
    tf[:3, 3] = translation
    return tf


def _make_gripper_tf(z_translation):
    tf = np.eye(4, dtype=float)
    tf[:3, 3] = (0.0, 0.0, z_translation)
    return tf


def _assert_close(a, b, tol=1e-9):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    assert a.shape == b.shape, f"shape mismatch: {a.shape} vs {b.shape}"
    assert np.allclose(a, b, atol=tol), f"matrix mismatch:\n{a}\nvs\n{b}"


def test_writer_roundtrip_and_partial_update():
    exporter = _load_exporter()

    male_left = _make_male_tf((-70.0, 0.0, -80.0))
    male_right = _make_male_tf((70.0, 0.0, -80.0))
    male_entries = {
        "T20_Male": {"left": male_left, "right": male_right},
    }
    gripper_robotiq = _make_gripper_tf(-150.0)
    gripper_entries = {"Robotiq": gripper_robotiq}

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "config_generated_ik.py")

        # 1. Initial write of both dicts.
        exporter._write_generated_ik(path, male_entries, gripper_entries)
        mod1 = _load_generated_module(path)
        _assert_close(mod1.MALE_JOINT_OCF_TO_TOOL0["T20_Male"]["left"], male_left)
        _assert_close(mod1.MALE_JOINT_OCF_TO_TOOL0["T20_Male"]["right"], male_right)
        _assert_close(mod1.BAR_GRASP_TO_TOOL0["Robotiq"], gripper_robotiq)

        # 2. Partial update: only the gripper dict changes; male dict is
        # re-emitted from the loader so it must round-trip identically.
        new_robotiq = _make_gripper_tf(-200.0)
        existing_male = exporter._load_existing_male(mod1)
        existing_gripper = exporter._load_existing_gripper(mod1)
        existing_gripper["Robotiq"] = new_robotiq

        exporter._write_generated_ik(path, existing_male, existing_gripper)
        mod2 = _load_generated_module(path)
        _assert_close(mod2.MALE_JOINT_OCF_TO_TOOL0["T20_Male"]["left"], male_left)
        _assert_close(mod2.MALE_JOINT_OCF_TO_TOOL0["T20_Male"]["right"], male_right)
        _assert_close(mod2.BAR_GRASP_TO_TOOL0["Robotiq"], new_robotiq)

        # 3. Partial update: add a second joint type while gripper dict stays.
        new_male_left = _make_male_tf((-50.0, 0.0, -90.0))
        existing_male = exporter._load_existing_male(mod2)
        existing_gripper = exporter._load_existing_gripper(mod2)
        existing_male.setdefault("T30_Male", {})["left"] = new_male_left

        exporter._write_generated_ik(path, existing_male, existing_gripper)
        mod3 = _load_generated_module(path)
        _assert_close(mod3.MALE_JOINT_OCF_TO_TOOL0["T20_Male"]["left"], male_left)
        _assert_close(mod3.MALE_JOINT_OCF_TO_TOOL0["T20_Male"]["right"], male_right)
        _assert_close(mod3.MALE_JOINT_OCF_TO_TOOL0["T30_Male"]["left"], new_male_left)
        _assert_close(mod3.BAR_GRASP_TO_TOOL0["Robotiq"], new_robotiq)

        # 4. Add a second gripper kind; preserve everything.
        new_kind_tf = _make_gripper_tf(-50.0)
        existing_male = exporter._load_existing_male(mod3)
        existing_gripper = exporter._load_existing_gripper(mod3)
        existing_gripper["Schunk"] = new_kind_tf

        exporter._write_generated_ik(path, existing_male, existing_gripper)
        mod4 = _load_generated_module(path)
        _assert_close(mod4.BAR_GRASP_TO_TOOL0["Robotiq"], new_robotiq)
        _assert_close(mod4.BAR_GRASP_TO_TOOL0["Schunk"], new_kind_tf)
        _assert_close(mod4.MALE_JOINT_OCF_TO_TOOL0["T30_Male"]["left"], new_male_left)


def test_writer_emits_empty_dicts():
    exporter = _load_exporter()
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "config_generated_ik.py")
        exporter._write_generated_ik(path, {}, {})
        mod = _load_generated_module(path)
        assert mod.MALE_JOINT_OCF_TO_TOOL0 == {}
        assert mod.BAR_GRASP_TO_TOOL0 == {}


if __name__ == "__main__":
    test_writer_roundtrip_and_partial_update()
    test_writer_emits_empty_dicts()
    print("ok")
