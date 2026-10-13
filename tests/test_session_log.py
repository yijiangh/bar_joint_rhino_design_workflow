"""The session log outside Rhino: one command's block, its steps, prompts, caught errors and failures."""

import sys
import time

import pytest

import session_log


@pytest.fixture
def log(tmp_path, monkeypatch):
    """A fresh log file in a temp folder; returns a function reading what was written."""
    monkeypatch.setenv("RS_LOG_DIR", str(tmp_path))
    for name, value in (("_file", None), ("_path", None), ("_failed", False), ("_counter", 0), ("_command", None)):
        monkeypatch.setattr(session_log, name, value)
    monkeypatch.setattr(session_log, "ENABLED", True)

    def read():
        session_log._file.flush()
        return open(session_log._path, encoding="utf-8").read()

    yield read
    if session_log._file is not None:
        session_log._file.close()


def test_nothing_is_logged_outside_a_command(log, tmp_path):
    with session_log.step("alone"):
        pass
    session_log.note("alone")
    session_log.caught(ValueError("alone"))
    assert session_log.timed()(lambda: 5)() == 5
    assert list(tmp_path.iterdir()) == []


def test_command_block_has_prints_steps_and_summary(log):
    def main():
        print("RSDemo: hello")
        with session_log.step("slow step", "B4"):
            time.sleep(0.05)
            with session_log.step("inner"):
                time.sleep(0.03)
        for _ in range(3):
            with session_log.step("quick"):
                pass
        return 7

    assert session_log.run("rs_demo.py", main) == 7
    text = log()
    assert "#1 rs_demo.py" in text
    assert "RSDemo: hello" in text
    assert "> slow step B4" in text and "< slow step B4" in text
    assert ". inner" in text
    assert "ended: ok" in text
    assert "3x, slowest" in text  # the quick steps: no line each, counted in the summary
    assert text.count("quick") == 1
    assert "(outside any step)" in text
    assert sys.stdout.__class__.__name__ != "_Tee"


def test_error_is_logged_with_traceback_and_raised(log):
    def main():
        with session_log.step("doomed"):
            raise RuntimeError("boom")

    with pytest.raises(RuntimeError):
        session_log.run("rs_demo.py", main)
    text = log()
    assert "RAISED RuntimeError" in text
    assert "ERROR Traceback" in text and "boom" in text
    assert "ended: FAILED: RuntimeError: boom" in text
    assert session_log._command is None


def test_caught_errors_are_logged_once_and_counted(log):
    def main():
        for _ in range(4):
            try:
                raise KeyError("missing")
            except Exception as exc:
                session_log.caught(exc)

    session_log.run("rs_demo.py", main)
    text = log()
    assert text.count("caught (the command carried on)") == 1
    assert "errors caught and carried on from: 4" in text
    assert "4x KeyError" in text


def test_waiting_for_the_user_is_not_working_time(log):
    class Getter:
        def Get(self):
            time.sleep(0.2)
            return "Nothing"

    def main():
        with session_log.step("pick"):
            assert session_log.ask(Getter()) == "Nothing"

    session_log.run("rs_demo.py", main)
    text = log()
    assert "ask   Getter" in text and "got   Enter (nothing picked)" in text
    assert "waiting for the user" in text
    working = float(text.split("total ")[1].split(" = ")[1].split(" s")[0])
    assert working < 0.15


def test_a_command_run_by_a_command_is_not_a_second_block(log):
    def inner():
        print("inner ran")

    session_log.run("rs_outer.py", lambda: session_log.run("rs_inner.py", inner))
    text = log()
    assert "#1 rs_outer.py" in text and "#2" not in text and "inner ran" in text


def test_a_command_started_inside_a_prompt_gets_its_own_block(log):
    def interrupting():
        print("interrupting ran")
        time.sleep(0.05)

    class Getter:
        def Get(self):  # Rhino runs the other button's script while this prompt is open
            session_log.run("rs_other.py", interrupting)
            return "Cancel"

    session_log.run("rs_first.py", lambda: session_log.ask(Getter()))
    text = log()
    assert "#2 rs_other.py" in text and "started while #1 rs_first.py was waiting at a prompt" in text
    assert "back in #1 rs_first.py" in text
    assert text.index("#2 rs_other.py ended") < text.index("#1 rs_first.py ended")
    first = text.split("#1 rs_first.py ended")[1]
    total, working = (float(first.split(word)[1].split(" s")[0]) for word in ("total ", " = "))
    assert working <= total and working < 0.04
    assert session_log._command is None


def test_prints_are_logged_after_rhino_put_its_own_stream_back(log):
    real = sys.stdout

    class Getter:
        def Get(self):  # a script run inside the prompt ends: Rhino resets sys.stdout
            sys.stdout = real
            return "Nothing"

    def main():
        session_log.ask(Getter())
        print("printed after the reset")

    session_log.run("rs_demo.py", main)
    assert "printed after the reset" in log().split("ended:")[0]
    assert sys.stdout is real
