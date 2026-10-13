"""The Python packages of the Rhino commands: installed once from `requirements-rhino.txt`, then put on `sys.path`.

Every command calls `activate()` before it imports numpy, compas or any other package. The first call installs the
whole requirements file with ONE pip call into a folder this module owns, named after the file's content:

    %LOCALAPPDATA%/bar_joint_rhino/python/py39-<hash of requirements-rhino.txt>

- ! Not `# venv:` / `# r:`: Rhino installs each `# r:` line with its own pip call, so a later package overwrote the
  pins of an earlier one (scipy pulled numpy 2.0.2 over 1.24.4, compas_robots compas 2.15.1 over 2.13.0), and Rhino
  sometimes rebuilt `scaffolding_env` from scratch.
- The install goes to `<folder>.partial` and is renamed when pip succeeds: the folder exists only when complete.
  A changed requirements file gives a new folder, so a package Rhino has loaded is never overwritten.
- Outside Rhino (pytest, `plan_design.py`) `activate()` does nothing: those use the Python they run in.
- Needs only the standard library; Rhino's API only for messages, when it is there.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
import time
from typing import List, Optional

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#: The one list of the commands' packages.
REQUIREMENTS = os.path.join(REPO_ROOT, "requirements-rhino.txt")
#: Overrides the folder that holds the environments.
ENV_ROOT_VARIABLE = "BAR_JOINT_RHINO_ENV_ROOT"
#: Packages whose copy loaded from elsewhere means Rhino's Python engine must be reloaded.
CHECKED_PACKAGES = ("numpy", "scipy", "trimesh", "compas", "compas_robots", "pybullet", "pybullet_planning")

_active: Optional[str] = None


def in_rhino() -> bool:
    """Whether this is Rhino 8's own CPython (`~/.rhinocode/py39-rh8`)."""
    return ".rhinocode" in os.path.normcase(sys.prefix)


def env_root() -> str:
    """The folder that holds one environment per requirements file content."""
    root = os.environ.get(ENV_ROOT_VARIABLE)
    if root:
        return root
    base = os.environ.get("LOCALAPPDATA") or os.path.join(os.path.expanduser("~"), ".local", "share")
    return os.path.join(base, "bar_joint_rhino", "python")


def env_name(requirements: Optional[str] = None) -> str:
    """`py<major><minor>-<hash>`: the environment for this Python and this requirements file content.

    Comments, blank lines and line endings do not change the hash.
    """
    with open(requirements or REQUIREMENTS, encoding="utf-8") as f:
        lines = [line.split("#", 1)[0].strip() for line in f]
    digest = hashlib.sha256("\n".join(line for line in lines if line).encode("utf-8")).hexdigest()[:10]
    return f"py{sys.version_info[0]}{sys.version_info[1]}-{digest}"


def python_executable() -> str:
    """The interpreter that runs pip: in Rhino `sys.executable` is Rhino itself, so look next to `sys.prefix`.

    Raises:
        RuntimeError: If no interpreter is found.
    """
    candidates = [sys.executable, os.path.join(sys.prefix, "python.exe"), os.path.join(sys.prefix, "bin", "python3")]
    for path in candidates:
        if path and os.path.basename(path).lower().startswith("python") and os.path.isfile(path):
            return path
    raise RuntimeError(f"rhino_env: no Python interpreter found in {sys.prefix}")


def _say(message: str) -> None:
    """Print to Rhino's command history (and stdout), and let Rhino redraw while pip runs."""
    print(message)
    try:
        import Rhino  # noqa: PLC0415  (only in Rhino)

        Rhino.RhinoApp.WriteLine(message)
        Rhino.RhinoApp.Wait()
    except Exception:  # noqa: BLE001  (not in Rhino, or no UI)
        pass


def _tail(path: str, lines: int = 25) -> str:
    """The last lines of a text file, for error messages."""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return "".join(f.readlines()[-lines:])
    except OSError:
        return ""


def install(target: str, requirements: Optional[str] = None, python: Optional[str] = None) -> None:
    """Install the requirements file into `target` with one pip call; `target` appears only when pip succeeded.

    Raises:
        RuntimeError: If pip fails, with the end of its log and, for a failed pybullet build, what to install.
    """
    requirements = requirements or REQUIREMENTS
    partial = target + ".partial"
    shutil.rmtree(partial, ignore_errors=True)
    os.makedirs(os.path.dirname(target), exist_ok=True)
    log = target + ".log"
    # -I: ignore the user site-packages and PYTHON* variables, as Rhino does; pip then sees only `--target`.
    command = [python or python_executable(), "-I", "-m", "pip", "install", "--disable-pip-version-check",
               "--no-warn-script-location", "--target", partial, "-r", requirements]
    _say(f"rhino_env: installing the Python packages of {os.path.basename(requirements)} into {target}. "
         "The first time takes several minutes (pybullet is compiled); Rhino does not respond meanwhile. "
         f"Log: {log}")
    started = time.perf_counter()
    with open(log, "w", encoding="utf-8") as out:
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        code = subprocess.call(command, stdout=out, stderr=subprocess.STDOUT, creationflags=flags)
    if code != 0:
        shutil.rmtree(partial, ignore_errors=True)
        tail = _tail(log)
        hint = ""
        if "pybullet" in tail.lower() or "visual c++" in tail.lower():
            hint = ("\npybullet has no ready-made build for Python 3.9 on Windows, so pip compiles it. Install "
                    "'Visual Studio Build Tools' with the workload 'Desktop development with C++', then run the "
                    "command again.")
        raise RuntimeError(f"rhino_env: pip failed (exit code {code}); full log: {log}{hint}\n{tail}")
    os.replace(partial, target)
    _say(f"rhino_env: installed in {time.perf_counter() - started:.0f} s.")


def _warn_foreign_copies(target: str) -> List[str]:
    """Packages already loaded from outside `target` (an older `scaffolding_env`): they stay in use until a reload."""
    foreign = []
    for name in CHECKED_PACKAGES:
        loaded = getattr(sys.modules.get(name), "__file__", None)
        if loaded and not os.path.normcase(loaded).startswith(os.path.normcase(target)):
            foreign.append(f"{name} ({loaded})")
    if foreign:
        _say("rhino_env: WARNING loaded from outside " + target + ": " + ", ".join(foreign) +
             ". Restart Rhino, or ScriptEditor -> Tools -> Reload Python 3 (CPython) Engine.")
    return foreign


#: The submodule folders that hold importable packages (`core.core_lib` puts them on `sys.path` at its first import).
SUBMODULE_PATHS = (os.path.join(REPO_ROOT, "external", "bar_assembly_core"),
                   os.path.join(REPO_ROOT, "external", "compas_fab", "src"))


def _default_ik_backend() -> None:
    """Use the `gradient` IK backend unless ssik is set up (its venv and its solver modules) or one was chosen."""
    if os.environ.get("HUSKY_IK_BACKEND"):
        return
    venv = os.environ.get("HUSKY_SSIK_VENV_DIR") or os.path.join(REPO_ROOT, "external", "ssik_env")
    python = os.path.join(venv, "Scripts", "python.exe") if sys.platform == "win32" else os.path.join(venv, "bin", "python")
    artifacts = os.environ.get("HUSKY_SSIK_ARTIFACT_DIR") or os.path.join(REPO_ROOT, "asset", "ssik")
    if not (os.path.isfile(python) and os.path.isdir(artifacts)):
        os.environ["HUSKY_IK_BACKEND"] = "gradient"
        _say("rhino_env: ssik is not set up (README, 'Analytical IK solver'); IK uses the gradient backend.")


def activate(force: bool = False) -> Optional[str]:
    """Make the packages of `requirements-rhino.txt` importable; install them first if needed.

    Cheap after the first call. Call it before importing numpy, compas or any other package.

    Args:
        force: Act outside Rhino too (tests).

    Returns:
        The environment folder, or None outside Rhino.
    """
    global _active
    if not (force or in_rhino()):
        return None
    target = os.path.join(env_root(), env_name())
    if _active != target:
        if not os.path.isdir(target):
            install(target)
        _default_ik_backend()
        _warn_foreign_copies(target)
        _active = target
    if sys.path[:1] != [target]:
        if target in sys.path:
            sys.path.remove(target)
        sys.path.insert(0, target)
    # ! Again on every command: `core.core_lib` adds these once, at its first import, and stays in `sys.modules`.
    # ! If `sys.path` loses them before compas_fab is first imported, a later command fails with
    # ! "No module named 'compas_fab'" (seen in Rhino on 2026-10-13).
    for folder in SUBMODULE_PATHS:
        if folder not in sys.path and os.path.isdir(folder):
            sys.path.insert(1, folder)
    return target
