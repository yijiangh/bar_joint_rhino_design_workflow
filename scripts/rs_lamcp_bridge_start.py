#! python 3
# venv: scaffolding_env
# ! No `# r:` package lines on purpose: the bridge only uses the Python standard
# ! library. The venv line is still important -- it makes this script run in the
# ! same scaffolding_env interpreter as the other RS* buttons, so code sent in by
# ! Claude Code sees the same installed packages and the same `sc.sticky` state
# ! (for example the live PyBullet client started by RSPBStart).
"""RSLamcpBridgeStart - Let Claude Code drive this Rhino session (no Grasshopper needed).

Left-click: start the LAMCP bridge, a small HTTP server on
http://127.0.0.1:8765 that lives INSIDE Rhino. Claude Code's lamcp MCP tools
send Python code to this server, so the agent can test RS* scripts, reload
edited modules, and save viewport screenshots on its own -- instead of asking
the human to click buttons and report back.

Right-click (RSLamcpBridgeStop) shuts the server down again.

This is a Rhino-native port of the `Lamcp_Bridge` Grasshopper component from
https://github.com/gramaziokohler/lamcp (v0.7.1), so Grasshopper does not need
to be open anymore. Do not enable both at once -- they both want port 8765.

On top of the original component this script also pre-defines
`_lamcp_run_on_ui` in the shared execution globals: code that must touch the
Rhino UI thread (view capture, redraws, document mutation) can always call
that helper, even before any MCP capture tool has injected its own copy.

Typical workflow:
    1. Open Rhino, click RSLamcpBridgeStart once.
    2. Ask Claude Code to test / debug something in this Rhino session.
    3. (Optional) right-click RSLamcpBridgeStop when you want Rhino left alone.
The server dies together with Rhino, so one click per Rhino session is enough.
"""

from __future__ import annotations

import contextlib
import io
import json
import threading
import traceback
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer

import Rhino
import scriptcontext as sc
import System


# ---- Session-wide storage keys inside `sc.sticky`. Same names as the official
# ---- Grasshopper bridge component, so if both ever run in the same interpreter
# ---- they see each other's state instead of silently double-starting.
SHARED_GLOBALS_KEY = "_lamcp_bridge_globals"
SERVER_KEY = "_lamcp_bridge_server"

# ! Must stay 8765: the lamcp MCP server on the Claude Code side defaults to
# ! LAMCP_BRIDGE_URL = http://127.0.0.1:8765 and never scans other ports.
PORT = 8765


def run_on_ui(fn, timeout: float = 20.0):
    """Run `fn` on Rhino's UI thread and hand its return value back.

    The bridge executes incoming code on its HTTP server thread. Reading
    RhinoCommon from there is usually fine, but anything that touches the UI
    or mutates the document (view capture, redraws, adding/deleting objects
    in bulk) must happen on the UI thread or Rhino can hard-crash. This
    helper is published into the shared execution globals under the name
    `_lamcp_run_on_ui` -- the same name the lamcp MCP server uses -- so
    incoming code can always call it.

    Args:
        fn: A function taking no arguments. Runs on the UI thread.
        timeout (float): Seconds to wait for the UI thread before giving up.

    Returns:
        Whatever `fn` returned.

    Raises:
        Exception: If `fn` raised (the traceback text is included), or if the
            UI thread did not run the job within `timeout` seconds.
    """
    box = {"result": None, "error": None}
    done = threading.Event()

    def _action():
        try:
            box["result"] = fn()
        except Exception:
            box["error"] = traceback.format_exc()
        finally:
            done.set()

    if Rhino.RhinoApp.InvokeRequired:
        # * We are on a background thread: hand the job to the UI thread and
        # * block here until it signals completion (or times out).
        Rhino.RhinoApp.InvokeOnUiThread(System.Action(_action))
        if not done.wait(timeout):
            raise Exception(f"UI-thread operation timed out after {timeout}s")
    else:
        _action()

    if box["error"] is not None:
        raise Exception("UI-thread operation failed:\n" + box["error"])
    return box["result"]


def _exec_code(code: str) -> dict:
    """Execute one incoming code snippet and package the outcome as a dict.

    Mirrors the official Grasshopper bridge behavior: stdout/stderr are
    captured, the special variable `_` carries the return value (sent back as
    `repr(_)`), and an exception comes back as a formatted traceback text
    instead of crashing the server. The globals dict persists across calls,
    so imports and variables accumulate over a session.

    Args:
        code (str): Python source sent by the MCP server.

    Returns:
        dict: Keys `stdout`, `stderr`, `result` (repr string or None) and
            `error` (traceback string or None).
    """
    out = io.StringIO()
    err = io.StringIO()
    exec_globals = sc.sticky.setdefault(SHARED_GLOBALS_KEY, {"__name__": "__lamcp_bridge__"})
    error_text = None
    result_repr = None
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            exec(code, exec_globals, exec_globals)
            result_repr = repr(exec_globals.get("_"))
    except BaseException:
        error_text = traceback.format_exc()

    return {
        "stdout": out.getvalue(),
        "stderr": err.getvalue(),
        "result": result_repr,
        "error": error_text,
    }


class _ExecHandler(BaseHTTPRequestHandler):
    """Tiny HTTP handler: GET /health for liveness, POST /exec to run code."""

    def log_message(self, fmt, *args):
        # * Silence the default per-request console logging -- it would spam
        # * Rhino's command line on every MCP call.
        pass

    def _send(self, status_code: int, payload: dict):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"status": "ok"})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/exec":
            self._send(404, {"error": "not found"})
            return
        length = int(self.headers.get("Content-Length", "0"))
        try:
            raw = self.rfile.read(length).decode("utf-8")
            body = json.loads(raw)
            code = body.get("code", "")
        except Exception as exc:
            self._send(400, {"error": "bad request: " + repr(exc)})
            return
        # ? The request body also carries a `timeout` field; like the official
        # ? bridge we ignore it here -- the MCP server enforces its own timeout.
        self._send(200, _exec_code(code))


def _start_server(port: int) -> dict:
    """Start the bridge HTTP server on a background (daemon) thread.

    Args:
        port (int): Local port to listen on (only reachable from this machine).

    Returns:
        dict: State handle with keys `server`, `thread`, `port`; keep it around
            to stop the server later.
    """
    server = HTTPServer(("127.0.0.1", port), _ExecHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True, name="lamcp-bridge")
    thread.start()
    return {"server": server, "thread": thread, "port": port}


def _stop_server(state: dict) -> None:
    """Shut down a server previously created by `_start_server`.

    Args:
        state (dict): The state handle returned by `_start_server`.
    """
    try:
        state["server"].shutdown()
        state["server"].server_close()
    except Exception:
        # * Best effort: a server that already died counts as stopped.
        pass


def main() -> None:
    """Start the bridge (idempotent: a second click just reports the status)."""
    # ---- Already running? Then say so and leave everything alone.
    state = sc.sticky.get(SERVER_KEY)
    if state is not None and state["thread"].is_alive():
        print(
            f"RSLamcpBridgeStart: bridge already listening on http://127.0.0.1:{state['port']}. "
            "Nothing to do."
        )
        return
    if state is not None:
        # * Leftover entry from a server that already died: clean it up first.
        del sc.sticky[SERVER_KEY]

    # ---- Publish the UI-thread helper BEFORE the server accepts any code, so
    # ---- incoming snippets can rely on `_lamcp_run_on_ui` from the first call.
    exec_globals = sc.sticky.setdefault(SHARED_GLOBALS_KEY, {"__name__": "__lamcp_bridge__"})
    exec_globals["_lamcp_run_on_ui"] = run_on_ui

    try:
        sc.sticky[SERVER_KEY] = _start_server(PORT)
    except OSError as exc:
        # ! Port taken but not by us. Most likely the Lamcp_Bridge Grasshopper
        # ! component is still enabled somewhere (or a second Rhino instance
        # ! runs a bridge). Deliberately NOT falling back to another port --
        # ! the MCP server only ever looks at 8765 -- so tell the user instead.
        print(
            f"RSLamcpBridgeStart: could not open 127.0.0.1:{PORT} ({exc}). "
            "Another program owns that port -- if the Lamcp_Bridge Grasshopper "
            "component is enabled, set its toggle to False (or close that GH "
            "definition), then click this button again."
        )
        return

    print(
        f"RSLamcpBridgeStart: bridge listening on http://127.0.0.1:{PORT}. "
        "Claude Code can now run Python in this Rhino session. "
        "Right-click this button (RSLamcpBridgeStop) to stop it."
    )


def stop() -> None:
    """Stop the bridge if this Rhino session started one (used by right-click)."""
    state = sc.sticky.pop(SERVER_KEY, None)
    if state is None:
        print(
            "RSLamcpBridgeStop: no bridge was started from Rhino, nothing to stop. "
            "If Claude Code still reaches this session, the Lamcp_Bridge "
            "Grasshopper component owns port 8765 -- disable its toggle instead."
        )
        return
    _stop_server(state)
    print(
        f"RSLamcpBridgeStop: bridge on port {state['port']} stopped. "
        "Claude Code can no longer reach this Rhino session."
    )


if __name__ == "__main__":
    main()
