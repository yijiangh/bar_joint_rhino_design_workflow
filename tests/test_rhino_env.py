"""`rhino_env`: one pip call per requirements content, a folder that exists only when complete, the IK backend default.

pip is replaced by a fake that writes a package into `--target`; the real install is checked by hand in Rhino.
"""

import os
import sys

import pytest

import rhino_env


@pytest.fixture
def env(tmp_path, monkeypatch):
    """`rhino_env` with its environments under `tmp_path`, a small requirements file and a fake pip."""
    requirements = tmp_path / "requirements-rhino.txt"
    requirements.write_text("# comment\nfakepkg==1.0\n", encoding="utf-8")
    monkeypatch.setattr(rhino_env, "REQUIREMENTS", str(requirements))
    monkeypatch.setattr(rhino_env, "_active", None)
    monkeypatch.setenv(rhino_env.ENV_ROOT_VARIABLE, str(tmp_path / "envs"))
    monkeypatch.setattr(sys, "path", list(sys.path))
    calls = []

    def fake_pip(command, stdout, stderr, creationflags):
        calls.append(command)
        target = command[command.index("--target") + 1]
        os.makedirs(os.path.join(target, "fakepkg"))
        stdout.write("Successfully installed fakepkg-1.0\n")
        return 0

    monkeypatch.setattr(rhino_env.subprocess, "call", fake_pip)
    monkeypatch.setattr(rhino_env, "python_executable", lambda: sys.executable)
    return {"requirements": requirements, "calls": calls, "root": tmp_path / "envs"}


def test_outside_rhino_does_nothing(env, monkeypatch):
    """Outside Rhino's Python (pytest, plan_design.py): no install, no sys.path change."""
    monkeypatch.setattr(rhino_env, "in_rhino", lambda: False)
    before = list(sys.path)
    assert rhino_env.activate() is None
    assert env["calls"] == [] and sys.path == before


def test_installs_once_with_one_pip_call(env):
    """The first activation installs every requirement in one pip call; later ones reuse the folder."""
    target = rhino_env.activate(force=True)
    assert len(env["calls"]) == 1 and env["calls"][0][-2:] == ["-r", str(env["requirements"])]
    assert os.path.isdir(os.path.join(target, "fakepkg")) and not os.path.exists(target + ".partial")
    assert sys.path[0] == target
    rhino_env._active = None
    assert rhino_env.activate(force=True) == target and len(env["calls"]) == 1


def test_changed_requirements_give_a_new_folder(env):
    """Only the requirements, not comments or blank lines, name the folder."""
    first = rhino_env.activate(force=True)
    env["requirements"].write_text("fakepkg==1.0\n\n# another comment\n", encoding="utf-8")
    assert rhino_env.env_name() == os.path.basename(first)
    env["requirements"].write_text("fakepkg==2.0\n", encoding="utf-8")
    second = rhino_env.activate(force=True)
    assert second != first and len(env["calls"]) == 2


def test_failed_install_leaves_no_folder(env, monkeypatch):
    """A failed pip leaves neither the folder nor `.partial`, and says what to install for pybullet."""
    def failing_pip(command, stdout, stderr, creationflags):
        os.makedirs(command[command.index("--target") + 1])
        stdout.write("Building wheel for pybullet ... error: Microsoft Visual C++ 14.0 or greater is required\n")
        return 1

    monkeypatch.setattr(rhino_env.subprocess, "call", failing_pip)
    with pytest.raises(RuntimeError, match="Visual Studio Build Tools"):
        rhino_env.activate(force=True)
    target = os.path.join(str(env["root"]), rhino_env.env_name())
    assert not os.path.exists(target) and not os.path.exists(target + ".partial")


def test_ik_backend_defaults_to_gradient_without_ssik(env, monkeypatch, tmp_path):
    """No ssik venv or solver modules: gradient. A backend set by the user is kept."""
    monkeypatch.delenv("HUSKY_IK_BACKEND", raising=False)
    monkeypatch.setenv("HUSKY_SSIK_VENV_DIR", str(tmp_path / "no_venv"))
    rhino_env.activate(force=True)
    assert os.environ["HUSKY_IK_BACKEND"] == "gradient"
    monkeypatch.setenv("HUSKY_IK_BACKEND", "ssik")
    rhino_env._active = None
    rhino_env.activate(force=True)
    assert os.environ["HUSKY_IK_BACKEND"] == "ssik"


def test_submodule_folders_are_put_back_on_the_path_every_time(env, monkeypatch):
    """`core.core_lib` adds them only at its first import; a command after a lost `sys.path` must still find them."""
    rhino_env.activate(force=True)
    present = [p for p in rhino_env.SUBMODULE_PATHS if os.path.isdir(p)]
    assert present and all(p in sys.path for p in present)
    for p in present:
        sys.path.remove(p)
    target = rhino_env.activate(force=True)
    assert sys.path[0] == target and all(p in sys.path for p in present)
