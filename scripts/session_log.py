"""The session log: what each Rhino command did, what the user answered, what went wrong, and where the time went.

One text file per Rhino session, in `<repo>/logs/` (or `RS_LOG_DIR`), written line by line and flushed, so it
survives a crash. Every command script runs its `main` through `run`; nothing is logged outside a command (tests,
`plan_process.py`, Grasshopper), where `step`, `timed`, `ask` and `caught` cost one `if`.

What a command's block holds, in order of time:

- everything the command printed (stdout and stderr are copied, not redirected);
- every prompt and message box with the user's answer (`rs.GetObject`, `rs.MessageBox`, ... are wrapped while the
  command runs; RhinoCommon getters go through `ask`), and how long the user took;
- the timed steps (`step`, `timed`), nested; a step shorter than `QUIET_STEP_S` with nothing logged inside it gets
  no line of its own, only its share of the summary;
- errors the code caught and carried on from (`caught`), with their traceback, the first time each occurs;
- an uncaught error with its traceback;
- at the end: total = working + waiting for the user, the working time by step, and the lines Rhino's command
  history gained that the plugin did not print (the prompts and the options clicked).

- ! Never `importlib.reload` this module: the open file, the command counter and the wrapped functions live here.
- ? "Waiting for the user" includes what runs inside a prompt (a dynamic preview drawn while the mouse moves); steps
  timed in there still show up, nested under the prompt.
- Set `RS_LOG=0` to switch the log off.
"""

from __future__ import annotations

import datetime
import functools
import os
import platform
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from contextlib import contextmanager
from typing import Callable, Dict, List, Optional, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENABLED = os.environ.get("RS_LOG", "1") != "0"
#: A step shorter than this with nothing logged inside it gets no line (it still counts in the summary).
QUIET_STEP_S = 0.02
#: Longest text logged for one answer, message or history line.
MAX_TEXT = 400
#: `rhinoscriptsyntax` functions that wait for the user; wrapped while a command runs.
RS_PROMPTS = ("GetObject", "GetObjects", "GetString", "GetReal", "GetInteger", "GetBoolean", "GetPoint",
              "GetPoints", "GetPointOnCurve", "GetPointOnSurface", "MessageBox", "ListBox", "CheckListBox",
              "MultiListBox", "StringBox", "RealBox", "IntegerBox", "EditBox", "PropertyListBox", "OpenFileName",
              "SaveFileName", "BrowseForFolder")

_lock = threading.RLock()
_file = None
_path: Optional[str] = None
_failed = False  # the log file could not be opened: stay silent for the session
_counter = 0
_command: Optional["_Command"] = None


class _Step:
    def __init__(self, name: str, detail: str, path: Tuple[str, ...]) -> None:
        self.name, self.detail, self.path = name, detail, path
        self.started = time.perf_counter()
        self.stamp = _stamp()
        self.waited_before = _command.waited if _command else 0.0
        self.opened = False  # its "began" line is written


class _Command:
    def __init__(self, number: int, name: str, script: str) -> None:
        self.number, self.name, self.script = number, name, script
        self.started = time.perf_counter()
        self.waited = 0.0  # seconds inside prompts
        self.waiting = 0  # prompts open now: a command Rhino starts meanwhile gets its own block
        self.steps: List[_Step] = []
        #: step path -> [count, working seconds, slowest], in order of first use.
        self.totals: Dict[Tuple[str, ...], List[float]] = {}
        #: (exception type, message, file, line) -> times caught.
        self.caught: Dict[tuple, int] = {}
        self.printed: set = set()
        self.history_start = len(_history())
        self.history_parts: List[str] = []  # the history before each command that interrupted this one
        self.tees: List["_Tee"] = []
        self.thread = threading.get_ident()
        self.restore: List[Callable[[], None]] = []


# --- --- --- --- --- THE FILE --- --- --- --- ---

def path() -> Optional[str]:
    """The session's log file, opened on first use; None if logging is off or no folder could be written."""
    global _file, _path, _failed
    if not ENABLED or _failed:
        return None
    if _file is not None:
        return _path
    name = datetime.datetime.now().strftime("rs_session_%Y%m%d_%H%M%S.log")
    for folder in (os.environ.get("RS_LOG_DIR") or os.path.join(REPO_ROOT, "logs"),
                   os.path.join(tempfile.gettempdir(), "bar_joint_rhino_logs")):
        try:
            os.makedirs(folder, exist_ok=True)
            _file = open(os.path.join(folder, name), "a", encoding="utf-8", errors="replace")  # noqa: SIM115
        except OSError:
            continue
        _path = os.path.join(folder, name)
        for line in _session_header():
            _file.write(line + "\n")
        _file.flush()
        return _path
    _failed = True
    return None


def _stamp() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _write(text: str, stamp: Optional[str] = None, depth: Optional[int] = None) -> None:
    """One entry, its continuation lines aligned under it; opens the enclosing steps first."""
    if path() is None:
        return
    with _lock:
        if _command is not None and depth is None:
            _open_steps()
            depth = len(_command.steps)
        pad = "  " * (1 + (depth or 0)) if _command is not None else ""
        first, *rest = str(text).split("\n")
        try:
            _file.write(f"{stamp or _stamp()}  {pad}{first}\n")
            for line in rest:
                _file.write(f"{'':12}  {pad}{line}\n")
            _file.flush()
        except (OSError, ValueError):
            pass


def _open_steps() -> None:
    for depth, entry in enumerate(_command.steps):
        if not entry.opened:
            entry.opened = True
            _write(f"> {entry.name}{entry.detail}", stamp=entry.stamp, depth=depth)


def _short(text, limit: int = MAX_TEXT) -> str:
    text = str(text)
    return text if len(text) <= limit else f"{text[:limit]} ... ({len(text)} characters)"


def _seconds(value: float) -> str:
    return f"{value:.2f} s" if value < 100 else f"{value:.0f} s"


# --- --- --- --- --- SESSION HEADER --- --- --- --- ---

def _git(folder: str, *args: str) -> Optional[str]:
    try:
        result = subprocess.run(["git", *args], cwd=folder, capture_output=True, text=True, timeout=5,
                                stdin=subprocess.DEVNULL, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    except (OSError, subprocess.SubprocessError, ValueError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def _commit(folder: str) -> str:
    commit = _git(folder, "rev-parse", "--short=12", "HEAD")
    if commit is None:
        return "unknown (git not available)"
    branch = _git(folder, "rev-parse", "--abbrev-ref", "HEAD") or "?"
    dirty = _git(folder, "status", "--porcelain", "--untracked-files=no")
    return f"{commit} on {branch}" + (" + uncommitted changes" if dirty else "")


def _session_header() -> List[str]:
    lines = ["bar_joint_rhino_design_workflow session log",
             f"started   {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
             f"machine   {platform.node()}, {platform.platform()}",
             f"python    {sys.version.split()[0]} ({sys.executable})"]
    try:
        import Rhino  # noqa: PLC0415

        lines.append(f"rhino     {Rhino.RhinoApp.Version} {Rhino.RhinoApp.SdkVersion}")
    except Exception:  # noqa: BLE001 -- outside Rhino (tests)
        lines.append("rhino     not running in Rhino")
    lines.append(f"plugin    {_commit(REPO_ROOT)}")
    for sub in ("bar_assembly_core", "compas_fab", "husky_assembly_tamp"):
        folder = os.path.join(REPO_ROOT, "external", sub)
        if os.path.isdir(folder):
            lines.append(f"  {sub:20}{_commit(folder)}")
    try:
        import rhino_env  # noqa: PLC0415

        lines.append(f"packages  {os.path.join(rhino_env.env_root(), rhino_env.env_name())}")
    except Exception:  # noqa: BLE001
        pass
    lines.append(f"ik        HUSKY_IK_BACKEND={os.environ.get('HUSKY_IK_BACKEND', '(not set: ssik)')}")
    lines += ["", "Each command below: its prints, the prompts with the user's answers, the timed steps",
              "(> began, < ended, . short step), errors, then where the time went.", ""]
    return lines


def _document() -> str:
    try:
        import scriptcontext as sc  # noqa: PLC0415

        doc = sc.doc
        try:  # ! Not `doc.Objects.Count`: it still counts deleted objects kept for undo.
            import Rhino  # noqa: PLC0415

            settings = Rhino.DocObjects.ObjectEnumeratorSettings()
            settings.NormalObjects = settings.LockedObjects = settings.HiddenObjects = True
            settings.DeletedObjects = False
            count = doc.Objects.ObjectCount(settings)
        except Exception:  # noqa: BLE001
            count = f"{doc.Objects.Count} (deleted ones included)"
        return f"document: {doc.Path or doc.Name or '(unsaved)'} | {count} objects | units {doc.ModelUnitSystem}"
    except Exception:  # noqa: BLE001
        return "document: (not available)"


def _history() -> str:
    try:
        import Rhino  # noqa: PLC0415

        return Rhino.RhinoApp.CommandHistoryWindowText or ""
    except Exception:  # noqa: BLE001
        return ""


def _command_names() -> Dict[str, str]:
    """Script file -> toolbar command name, from the toolbar file (read once)."""
    global _names
    if _names is None:
        _names = {}
        try:
            import re  # noqa: PLC0415

            with open(os.path.join(REPO_ROOT, "scaffolding_toolbar.rui"), encoding="utf-8", errors="replace") as f:
                text = f.read()
            for item in re.findall(r"<macro_item\b.*?</macro_item>", text, re.S):
                name = re.search(r"<locale_1033>([^<]+)</locale_1033>", item)
                script = re.search(r'_R\s+"([^"]+\.py)"', item)
                if name and script:
                    _names.setdefault(script.group(1).lower(), name.group(1).strip())
        except OSError:
            pass
    return _names


_names: Optional[Dict[str, str]] = None


# --- --- --- --- --- STDOUT / STDERR --- --- --- --- ---

class _Tee:
    """Copies what is written to a stream into the log, a line at a time; everything else is the stream's."""

    def __init__(self, stream, label: str) -> None:
        self._stream, self._label, self._pending = stream, label, ""

    def write(self, text):
        try:
            if isinstance(text, str) and text:
                with _lock:
                    self._pending += text
                    *lines, self._pending = self._pending.split("\n")
                    for line in lines:
                        self._emit(line)
        except Exception:  # noqa: BLE001 -- the log must never break a print
            pass
        return self._stream.write(text) if self._stream is not None else len(text)

    def _emit(self, line: str) -> None:
        line = line.rstrip("\r")
        if not line.strip():
            return
        if _command is not None:
            _command.printed.add(line.strip())
        _write(f"{self._label}{line}")

    def finish(self) -> None:
        with _lock:
            if self._pending:
                self._emit(self._pending)
                self._pending = ""

    def flush(self):
        return self._stream.flush() if self._stream is not None else None

    def __getattr__(self, name):
        return getattr(self._stream, name)


def _ensure_tees() -> None:
    """Copy stdout and stderr into the log (again): Rhino puts its own streams back when a script ends, also one
    run inside a prompt of the command still running, whose later prints would then miss the log."""
    command = _command
    if command is None:
        return
    if not isinstance(sys.stdout, _Tee):
        sys.stdout = _Tee(sys.stdout, "")
        command.tees.append(sys.stdout)
    if not isinstance(sys.stderr, _Tee):
        sys.stderr = _Tee(sys.stderr, "stderr  ")
        command.tees.append(sys.stderr)


# --- --- --- --- --- PROMPTS --- --- --- --- ---

def _describe(value) -> str:
    """An answer, short: object ids with their names, lists by their length."""
    if value is None:
        return "None (cancelled or nothing picked)"
    if isinstance(value, (list, tuple)):
        if len(value) > 6:
            return f"{len(value)} items: " + ", ".join(_describe(v) for v in value[:6]) + ", ..."
        return "[" + ", ".join(_describe(v) for v in value) + "]"
    text = str(value)
    if len(text) == 36 and text.count("-") == 4:  # an object id
        try:
            import scriptcontext as sc  # noqa: PLC0415
            import System  # noqa: PLC0415

            obj = sc.doc.Objects.FindId(System.Guid(text))
            if obj is not None:
                return f"{obj.Attributes.Name or obj.ObjectType} ({text[:8]})"
        except Exception:  # noqa: BLE001
            pass
    return _short(text)


@contextmanager
def _waiting():
    command, started = _command, time.perf_counter()
    if command is not None:
        command.waiting += 1
    try:
        yield
    finally:
        if command is not None:
            command.waiting -= 1
            command.waited += time.perf_counter() - started
            _ensure_tees()  # a script run inside the prompt left Rhino's own streams behind


def _wrap_prompt(name: str, function: Callable) -> Callable:
    @functools.wraps(function)
    def wrapper(*args, **kwargs):
        if _command is None or threading.get_ident() != _command.thread:
            return function(*args, **kwargs)
        prompt = args[0] if args and isinstance(args[0], str) else kwargs.get("message", "")
        _write(f"ask   rs.{name}: {_short(prompt)}" if prompt else f"ask   rs.{name}")
        started = time.perf_counter()
        with _waiting():
            result = function(*args, **kwargs)
        _write(f"got   {_describe(result)}   (after {_seconds(time.perf_counter() - started)})")
        return result

    return wrapper


def _wrap_rs_prompts(command: _Command) -> None:
    try:
        import rhinoscriptsyntax as rs  # noqa: PLC0415
    except Exception:  # noqa: BLE001
        return
    for name in RS_PROMPTS:
        original = getattr(rs, name, None)
        if callable(original):
            setattr(rs, name, _wrap_prompt(name, original))
            command.restore.append(lambda name=name, original=original: setattr(rs, name, original))


def _getter_answer(getter, result) -> str:
    kind = str(result)
    try:
        if kind == "Option":
            option = getter.Option()
            return f"Option {option.EnglishName}" + (f" = {option.StringOptionValue}" if option.StringOptionValue else "")
        if kind == "Object":
            return "Object " + _describe([getter.Object(i).ObjectId for i in range(getter.ObjectCount)])
        if kind == "Point":
            p = getter.Point()
            return f"Point ({p.X:.1f}, {p.Y:.1f}, {p.Z:.1f})"
        if kind == "String":
            return f"String {_short(getter.StringResult())!r}"
        if kind == "Number":
            return f"Number {getter.Number()}"
    except Exception:  # noqa: BLE001
        pass
    return {"Nothing": "Enter (nothing picked)", "Cancel": "Cancel (Esc)"}.get(kind, kind)


def ask(getter):
    """`getter.Get()` for a RhinoCommon getter, logged with the answer and the time the user took.

    Returns:
        Rhino.Input.GetResult: What `Get()` returned.
    """
    if _command is None or threading.get_ident() != _command.thread:
        return getter.Get()
    _write(f"ask   {type(getter).__name__}")
    started = time.perf_counter()
    with _waiting():
        result = getter.Get()
    _write(f"got   {_getter_answer(getter, result)}   (after {_seconds(time.perf_counter() - started)})")
    return result


# --- --- --- --- --- STEPS --- --- --- --- ---

@contextmanager
def step(name: str, detail: str = ""):
    """Time a block as a named step of the running command. Outside a command: nothing.

    Args:
        name: What is done, the same every time (it is the key of the summary).
        detail: What it is done to this time, e.g. a bar id (logged, not part of the key).
    """
    command = _command
    if command is None or threading.get_ident() != command.thread:
        yield
        return
    with _lock:
        parent = command.steps[-1].path if command.steps else ()
        entry = _Step(name, f" {detail}" if detail else "", parent + (name,))
        command.steps.append(entry)
        command.totals.setdefault(entry.path, [0, 0.0, 0.0])
    failed = ""
    try:
        yield
    except BaseException as exc:
        failed = f"   RAISED {type(exc).__name__}"
        raise
    finally:
        with _lock:
            elapsed = time.perf_counter() - entry.started
            waited = command.waited - entry.waited_before
            working = max(elapsed - waited, 0.0)
            total = command.totals[entry.path]
            total[0] += 1
            total[1] += working
            total[2] = max(total[2], working)
            if entry in command.steps:
                del command.steps[command.steps.index(entry):]
            took = _seconds(working) + (f" (+ {_seconds(waited)} waiting for the user)" if waited >= 0.01 else "")
            depth = len(command.steps)
            if entry.opened:
                _write(f"< {name}{entry.detail}   {took}{failed}", depth=depth)
            elif working >= QUIET_STEP_S or failed:
                _open_steps()  # the steps around it, so the line has its context
                _write(f". {name}{entry.detail}   {took}{failed}", depth=depth)


def timed(name: Optional[str] = None) -> Callable:
    """Decorator: the function is a `step`, named after it unless `name` is given."""

    def decorate(function: Callable) -> Callable:
        label = name or function.__name__.lstrip("_")

        @functools.wraps(function)
        def wrapper(*args, **kwargs):
            if _command is None:
                return function(*args, **kwargs)
            with step(label):
                return function(*args, **kwargs)

        return wrapper

    return decorate


def note(text: str) -> None:
    """One line in the running command's block: a decision or a value worth reading back."""
    if _command is not None:
        _write(f"note  {text}")


def caught(exc: BaseException, context: str = "") -> None:
    """Log an error the code caught and carries on from; call it first in the `except` block.

    The traceback is written the first time an error occurs at a place; repeats are counted in the summary.
    """
    command = _command
    if command is None:
        return
    try:
        frames = traceback.extract_tb(exc.__traceback__)
        where = (frames[-1].filename, frames[-1].lineno) if frames else ("?", 0)
        key = (type(exc).__name__, str(exc)[:200], *where)
        with _lock:
            command.caught[key] = command.caught.get(key, 0) + 1
            if command.caught[key] > 1:
                return
        lines = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__)).rstrip()
        _write(f"caught{' in ' + context if context else ''} (the command carried on): {lines}")
    except Exception:  # noqa: BLE001 -- the log must never break error handling
        pass


# --- --- --- --- --- COMMANDS --- --- --- --- ---

def run(script_file: str, main: Callable, *args, **kwargs):
    """Run a command's `main`, logged. Every command script calls its entry point through this.

    Args:
        script_file: The script's `__file__`; the command is named after its toolbar button.

    Returns:
        What `main` returned. Errors are logged with their traceback and raised again.

    - A command whose `main` runs another command's `main` is one block, the outer one.
    - ? A button clicked while a command waits at a prompt: Rhino runs that script inside the prompt. It gets its
      own block, and its whole time counts as waiting in the command it interrupted.
    """
    global _command, _counter
    parent = _command
    if path() is None or (parent is not None and not parent.waiting):
        return main(*args, **kwargs)
    script = os.path.basename(script_file)
    first = _counter == 0
    _counter += 1
    command = _Command(_counter, _command_names().get(script.lower(), script), script)
    out, err = sys.stdout, sys.stderr
    if parent is not None:  # its history up to here; what follows belongs to this command
        parent.history_parts.append(_history()[parent.history_start:])
    with _lock:
        _command = command
        inside = f"\nstarted while #{parent.number} {parent.name} was waiting at a prompt" if parent else ""
        _write("=" * 100, depth=-1)
        _write(f"#{command.number} {command.name}  ({script}){inside}\n{_document()}", depth=-1)
    if first and out is not None:
        try:
            out.write(f"Session log: {_path}\n")
            command.printed.add(f"Session log: {_path}")
        except Exception:  # noqa: BLE001
            pass
    _ensure_tees()
    if parent is None:  # an interrupting command asks through the prompts already wrapped
        _wrap_rs_prompts(command)
    outcome = "ok"
    try:
        return main(*args, **kwargs)
    except SystemExit as exc:
        outcome = f"ok (exit code {exc.code})"
        raise
    except BaseException as exc:
        outcome = f"FAILED: {type(exc).__name__}: {_short(exc)}"
        command.steps.clear()
        _write("ERROR " + "".join(traceback.format_exception(type(exc), exc, exc.__traceback__)).rstrip())
        raise
    finally:
        for undo in command.restore:
            undo()
        for tee in command.tees:
            tee.finish()
        # Only take the streams back if they are still ours (compas_fab replaces sys.stdout while it connects).
        if sys.stdout in command.tees:
            sys.stdout = out
        if sys.stderr in command.tees:
            sys.stderr = err
        command.steps.clear()
        _finish(command, outcome)
        _command = parent
        if parent is not None:
            parent.history_start = len(_history())
            _write("=" * 100, depth=-1)
            _write(f"back in #{parent.number} {parent.name}, still waiting at its prompt", depth=-1)


def _finish(command: _Command, outcome: str) -> None:
    total = time.perf_counter() - command.started
    working = max(total - command.waited, 0.0)
    lines = [f"#{command.number} {command.name} ended: {outcome}",
             f"total {_seconds(total)} = {_seconds(working)} working + {_seconds(command.waited)} waiting for the user"]
    if command.totals:
        lines.append("working time by step (steps inside a step are indented and already counted in it):")
        top = 0.0
        for step_path, (count, seconds, slowest) in command.totals.items():
            if len(step_path) == 1:
                top += seconds
            times = "" if count == 1 else f"   {int(count)}x, slowest {_seconds(slowest)}"
            lines.append(f"  {_seconds(seconds):>9}  {'  ' * (len(step_path) - 1)}{step_path[-1]}{times}")
        lines.append(f"  {_seconds(max(working - top, 0.0)):>9}  (outside any step)")
    if command.caught:
        lines.append(f"errors caught and carried on from: {sum(command.caught.values())}")
        for (kind, message, filename, lineno), count in command.caught.items():
            lines.append(f"  {count}x {kind}: {_short(message, 120)}  [{os.path.basename(filename)}:{lineno}]")
    history = "".join(command.history_parts) + _history()[command.history_start:]
    history = [line.strip() for line in history.splitlines()]
    history = [line for line in history if line and line not in command.printed]
    if history:
        lines.append("Rhino's command history during the command (prompts and clicked options; prints left out):")
        lines += [f"  {_short(line)}" for line in history[-200:]]
    _write("\n".join(lines), depth=-1)
