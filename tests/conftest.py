import os
import sys

import pytest


TESTS_DIR = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.join(TESTS_DIR, "..", "scripts")
for path in (TESTS_DIR, SCRIPTS_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)


def pytest_addoption(parser):
    parser.addoption(
        "--viz",
        action="store_true",
        default=False,
        help="Show visual plots for tests that support it",
    )


def pytest_configure(config):
    """Record the user's intent for visual test runs."""

    if not config.getoption("--viz"):
        return
    if config.getoption("capture") == "no":
        return

    config.option.capture = "no"
    capture_manager = config.pluginmanager.getplugin("capturemanager")
    if capture_manager is not None:
        capture_manager._method = "no"


@pytest.fixture(autouse=True)
def _disable_capture_for_viz(request, monkeypatch):
    """Make ``--viz`` test execution behave like ``-s``."""

    if not request.config.getoption("--viz"):
        yield
        return

    monkeypatch.setattr(sys, "stdin", sys.__stdin__)
    monkeypatch.setattr(sys, "stdout", sys.__stdout__)
    monkeypatch.setattr(sys, "stderr", sys.__stderr__)

    capture_manager = request.config.pluginmanager.getplugin("capturemanager")
    if capture_manager is None:
        yield
        return

    with capture_manager.global_and_fixture_disabled():
        yield


@pytest.fixture
def viz(request):
    """Return a visualization context that renders after the test body completes."""

    from viz_helpers import VizContext

    enabled = request.config.getoption("--viz")
    ctx = VizContext(enabled=enabled)
    yield ctx
    ctx.show()


@pytest.fixture(scope="module")
def viz_enabled(request):
    """Module-level flag for fixtures that need to choose GUI vs headless mode."""

    return request.config.getoption("--viz")
