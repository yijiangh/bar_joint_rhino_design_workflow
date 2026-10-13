#! python 3
"""RSShowLog - Open the session log: what every command of this Rhino session did, and where the time went.

Prints the log file's path and opens it in the default text editor; the `logs/` folder next to `scripts/` holds one
file per Rhino session (`scripts/session_log.py`). Run it with `_-ScriptEditor _R "rs_show_log.py"`.
"""

from __future__ import annotations

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import session_log  # noqa: E402


def main() -> None:
    path = session_log.path()
    if path is None:
        print("RSShowLog: the session log is off (RS_LOG=0) or its folder cannot be written.")
        return
    print(f"RSShowLog: {path}")
    print(f"RSShowLog: all sessions: {os.path.dirname(path)}")
    try:
        os.startfile(path)  # noqa: S606 -- the user's own log, in their editor
    except (OSError, AttributeError) as exc:
        print(f"RSShowLog: could not open it ({exc}).")


if __name__ == "__main__":
    session_log.run(__file__, main)
