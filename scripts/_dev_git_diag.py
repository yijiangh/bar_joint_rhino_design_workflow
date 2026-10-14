#! python 3
"""Dev diagnostic: why `git status` on the core submodule hangs inside Rhino (writer_info takes its 5 s timeout).

Run in Rhino: `_-ScriptEditor _R "_dev_git_diag.py"`. Runs the core's git calls in several variants, each with a
30 s limit, and writes the timings plus git's own trace to `logs/git_diag.txt` (also printed). Changes nothing.
"""

import os
import shutil
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(SCRIPT_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import core_lib  # noqa: E402,F401
from bar_assembly_core.process import version  # noqa: E402

LOGS = os.path.join(REPO, "logs")
os.makedirs(LOGS, exist_ok=True)
OUT = os.path.join(LOGS, "git_diag.txt")
CORE = str(version._PACKAGE_DIR)
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
lines = []


def say(text):
    lines.append(text)
    print(text)


def run(label, cwd, args, trace=False, **kwargs):
    env = dict(os.environ)
    trace_file = os.path.join(LOGS, "git_diag_trace_%d.txt" % len(lines))
    if trace:
        env.update(GIT_TRACE=trace_file, GIT_TRACE_PERFORMANCE=trace_file, GIT_TRACE_SETUP=trace_file)
    started = time.perf_counter()
    process = subprocess.Popen(["git"] + args, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                               env=env, **kwargs)
    try:
        out, err = process.communicate(timeout=30)
        result = "rc=%s out=%r err=%r" % (process.returncode, out.strip()[:60], err.strip()[:200])
    except subprocess.TimeoutExpired:
        process.kill()
        try:
            out, err = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            out, err = "", "(pipes still open after kill: a child process holds them)"
        result = "TIMEOUT after 30 s; out=%r err=%r" % (out.strip()[:60], err.strip()[:300])
    say("%6.2f s  %s: %s" % (time.perf_counter() - started, label, result))
    if trace and os.path.isfile(trace_file):
        with open(trace_file, encoding="utf-8", errors="replace") as f:
            for line in f.read().splitlines()[-25:]:
                say("            trace| " + line[:220])


say("git: %s" % shutil.which("git"))
say("core package dir: %s" % CORE)
say("stdin=%r" % (sys.stdin,))
say("GIT_* env: %r" % {k: v for k, v in os.environ.items() if k.upper().startswith("GIT")})
say("HOME=%r USERPROFILE=%r" % (os.environ.get("HOME"), os.environ.get("USERPROFILE")))

run("1 version", CORE, ["--version"])
run("2 core rev-parse (as the core does)", CORE, ["rev-parse", "--short=12", "HEAD"])
run("3 core status (as the core does), traced", CORE, ["status", "--porcelain", "--", CORE], trace=True)
run("4 same, stdin closed", CORE, ["status", "--porcelain", "--", CORE], stdin=subprocess.DEVNULL)
run("5 same, stdin closed + no console window", CORE, ["status", "--porcelain", "--", CORE],
    stdin=subprocess.DEVNULL, creationflags=NO_WINDOW)
run("6 same as 5, pathspec '.'", CORE, ["status", "--porcelain", "--", "."],
    stdin=subprocess.DEVNULL, creationflags=NO_WINDOW)
run("7 same as 5, no optional locks, no fsmonitor", CORE,
    ["--no-optional-locks", "-c", "core.fsmonitor=false", "status", "--porcelain", "--", CORE],
    stdin=subprocess.DEVNULL, creationflags=NO_WINDOW)
run("8 plugin repo status (as the plugin does)", REPO, ["status", "--porcelain", "--", "scripts"])
run("9 plugin repo status, stdin closed + no console window", REPO, ["status", "--porcelain", "--", "scripts"],
    stdin=subprocess.DEVNULL, creationflags=NO_WINDOW)
run("10 core status again (as the core does)", CORE, ["status", "--porcelain", "--", CORE])

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print("Written to %s" % OUT)
