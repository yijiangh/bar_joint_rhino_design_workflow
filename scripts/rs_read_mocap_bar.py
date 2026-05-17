#! python 3
# venv: scaffolding_env
"""Read one OptiTrack rigid body and bake its labeled markers into Rhino.

Connects to a Motive NatNet stream, waits for one frame containing the
selected rigid body's labeled markers, converts each marker position from
mocap (Y-up, meters) to Rhino doc coordinates (Z-up, doc units), and
bakes them as named points on the ``MoCap_Retrieval`` layer.

Axis remap (mocap -> Rhino):
    rhino.x =  mocap.x
    rhino.y = -mocap.z
    rhino.z =  mocap.y

Points are named ``<rigid_body>_<i>`` (0-based, ordered by marker id) so
re-running the command on the same rigid body replaces the previous bake.

Settings (server/client IP, multicast flag, last-picked rigid body name)
are remembered in ``sc.sticky`` for the Rhino session.
"""

import math
import os
import sys
import threading
import time

try:  # pragma: no cover - only importable inside Rhino ScriptEditor
    import Rhino
    import rhinoscriptsyntax as rs
    import scriptcontext as sc
except ImportError:  # standalone import (e.g. from the smoke test)
    Rhino = None
    rs = None
    sc = None

# Make the optitrack sibling package importable when run via ScriptEditor.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from optitrack.NatNetClient import NatNetClient  # noqa: E402


# ===== DEFAULTS (edit if your LAN changes) =====
_DEFAULT_CLIENT_IP = "192.168.0.24"      # this Windows machine's IP on the mocap LAN
_DEFAULT_SERVER_IP = "192.168.0.117"     # Motive PC IP
_DEFAULT_USE_MULTICAST = False           # match Motive > Streaming > Transmission Type

# name -> mocap streaming-id (from Motive Asset properties).
_RIGID_BODIES = {
    "bar_x": 4631,
    "bar_y": 4632,
    "bar_z": 4633,
}
# ===============================================

_LAYER_NAME = "MoCap_Retrieval"
_LAYER_COLOR = (255, 128, 0)
_FRAME_TIMEOUT_S = 5.0

# Sanity check thresholds (real-world distances in mm).
_EXPECTED_MARKER_COUNT = 4
_MARKER_ERROR_MAX_MM = 2.5
_PAIR_DIST_MIN_MM = 70.0
_PAIR_DIST_MAX_MM = 90.0
_PAIR_DIST_DIFF_MAX_MM = 5.0

_STICKY_CLIENT_IP = "rs_read_mocap_bar.client_ip"
_STICKY_SERVER_IP = "rs_read_mocap_bar.server_ip"
_STICKY_MULTICAST = "rs_read_mocap_bar.use_multicast"
_STICKY_RIGID_BODY = "rs_read_mocap_bar.rigid_body"


# ---------------------------------------------------------------------------
# Layer helpers (kept local; rhino_helpers depends on numpy)
# ---------------------------------------------------------------------------

def _ensure_layer(layer_name, color):
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name, color=color)
    else:
        rs.LayerColor(layer_name, color)
    if hasattr(rs, "LayerVisible") and not rs.LayerVisible(layer_name):
        rs.LayerVisible(layer_name, True)
    return layer_name


def _delete_existing_markers(layer_name, rigid_body_name):
    """Remove any prior point objects on the layer whose name starts with the
    rigid body prefix, so re-running replaces (instead of duplicating)."""
    if not rs.IsLayer(layer_name):
        return 0
    ids = rs.ObjectsByLayer(layer_name) or []
    prefix = "{}_".format(rigid_body_name)
    removed = 0
    for oid in ids:
        name = rs.ObjectName(oid) or ""
        if name == rigid_body_name or name.startswith(prefix):
            rs.DeleteObject(oid)
            removed += 1
    return removed


# ---------------------------------------------------------------------------
# Mocap state + connection
# ---------------------------------------------------------------------------

class _MocapState:
    """Thread-safe holder for the single rigid body we want."""

    def __init__(self, model_id):
        self._lock = threading.Lock()
        self._model_id = model_id
        self._markers = None  # dict[marker_id] = {'pos': (x,y,z), ...}
        self._got = threading.Event()

    def on_labeled_markers(self, labeled_marker_from_model_id):
        markers = labeled_marker_from_model_id.get(self._model_id)
        if not markers:
            return
        with self._lock:
            # Shallow-copy is enough; values are simple dicts of floats.
            self._markers = {mid: dict(m) for mid, m in markers.items()}
            self._got.set()

    def wait(self, timeout_s):
        return self._got.wait(timeout=timeout_s)

    def markers(self):
        with self._lock:
            return self._markers


def _safe_shutdown(client):
    """Close NatNet sockets without joining the receive threads.

    NatNetClient.shutdown() calls thread.join() with no timeout. On Windows
    the blocking ``recvfrom`` in the data/command threads does not always
    wake up cleanly when the socket is closed, so join() blocks forever and
    freezes the Rhino UI. We instead signal the threads, close the sockets,
    and leave the (now-idle) threads to die on their own.
    """
    try:
        client.stop_threads = True
    except AttributeError:
        pass
    for attr in ("command_socket", "data_socket"):
        sock = getattr(client, attr, None)
        if sock is None:
            continue
        try:
            sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Settings prompt
# ---------------------------------------------------------------------------

def _prompt_string(prompt, default):
    result = rs.GetString(prompt, default)
    if result is None:
        return None
    return result.strip() or default


def _prompt_settings():
    """Interactive GetOption loop. Returns dict or None on cancel."""
    names = list(_RIGID_BODIES.keys())

    client_ip = sc.sticky.get(_STICKY_CLIENT_IP, _DEFAULT_CLIENT_IP)
    server_ip = sc.sticky.get(_STICKY_SERVER_IP, _DEFAULT_SERVER_IP)
    use_multicast = bool(sc.sticky.get(_STICKY_MULTICAST, _DEFAULT_USE_MULTICAST))
    last_name = sc.sticky.get(_STICKY_RIGID_BODY, names[0])
    if last_name not in names:
        last_name = names[0]
    rb_index = names.index(last_name)

    while True:
        go = Rhino.Input.Custom.GetOption()
        go.SetCommandPrompt(
            "MoCap settings (server={}, client={}, multicast={}, body={})".format(
                server_ip, client_ip, use_multicast, names[rb_index]
            )
        )
        opt_multicast = Rhino.Input.Custom.OptionToggle(use_multicast, "No", "Yes")
        idx_body = go.AddOptionList("RigidBody", names, rb_index)
        idx_server = go.AddOption("ServerIP")
        idx_client = go.AddOption("ClientIP")
        go.AddOptionToggle("Multicast", opt_multicast)
        go.AcceptNothing(True)

        result = go.Get()
        if result == Rhino.Input.GetResult.Cancel:
            return None
        if result == Rhino.Input.GetResult.Nothing:
            use_multicast = bool(opt_multicast.CurrentValue)
            break
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == idx_body:
                rb_index = go.Option().CurrentListOptionIndex
            elif chosen == idx_server:
                got = _prompt_string("Motive server IP", server_ip)
                if got is None:
                    return None
                server_ip = got
            elif chosen == idx_client:
                got = _prompt_string("This machine's client IP", client_ip)
                if got is None:
                    return None
                client_ip = got
            # multicast toggle updates in-place via opt_multicast
            use_multicast = bool(opt_multicast.CurrentValue)
            continue
        return None

    settings = {
        "client_ip": client_ip,
        "server_ip": server_ip,
        "use_multicast": use_multicast,
        "rigid_body_name": names[rb_index],
        "rigid_body_id": _RIGID_BODIES[names[rb_index]],
    }
    sc.sticky[_STICKY_CLIENT_IP] = client_ip
    sc.sticky[_STICKY_SERVER_IP] = server_ip
    sc.sticky[_STICKY_MULTICAST] = use_multicast
    sc.sticky[_STICKY_RIGID_BODY] = names[rb_index]
    return settings


# ---------------------------------------------------------------------------
# Bake
# ---------------------------------------------------------------------------

def _mocap_to_rhino_point(pos_m, scale):
    """Mocap (Y-up, meters) -> Rhino (Z-up, doc units)."""
    mx, my, mz = float(pos_m[0]), float(pos_m[1]), float(pos_m[2])
    return Rhino.Geometry.Point3d(mx * scale, -mz * scale, my * scale)


def _scalar(value):
    """NatNet struct.unpack sometimes hands back single-element tuples."""
    if isinstance(value, (tuple, list)):
        return value[0] if value else 0.0
    return value


def _dist_m(p, q):
    return math.sqrt(
        (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2
    )


def _validate_and_pair(markers):
    """Check marker count / validity / error and find the best pairing.

    Returns ``(centers_m, info)`` where ``centers_m`` is a 2-tuple of mocap
    (meters) center points and ``info`` is a status string; or
    ``(None, error_message)`` on failure.
    """
    if len(markers) != _EXPECTED_MARKER_COUNT:
        return None, "Expected {} markers but received {}.".format(
            _EXPECTED_MARKER_COUNT, len(markers)
        )

    items = sorted(markers.items())  # [(mid, m), ...]
    for mid, m in items:
        if not m.get("is_valid", True):
            return None, "Marker id={} is occluded/invalid.".format(mid)
        err_m = float(_scalar(m.get("error", 0.0)))
        if err_m * 1000.0 > _MARKER_ERROR_MAX_MM:
            return None, (
                "Marker id={} has residual {:.3f} mm (> {:.1f} mm threshold)."
            ).format(mid, err_m * 1000.0, _MARKER_ERROR_MAX_MM)
        if m.get("pos") is None:
            return None, "Marker id={} has no position.".format(mid)

    ids = [mid for mid, _ in items]
    pos = {mid: tuple(float(c) for c in m["pos"]) for mid, m in items}

    pairings = [
        ((ids[0], ids[1]), (ids[2], ids[3])),
        ((ids[0], ids[2]), (ids[1], ids[3])),
        ((ids[0], ids[3]), (ids[1], ids[2])),
    ]
    best = None
    for pair_a, pair_b in pairings:
        d_a = _dist_m(pos[pair_a[0]], pos[pair_a[1]])
        d_b = _dist_m(pos[pair_b[0]], pos[pair_b[1]])
        score = max(d_a, d_b)
        if best is None or score < best[0]:
            best = (score, pair_a, pair_b, d_a, d_b)

    _, pair_a, pair_b, d_a, d_b = best
    d_a_mm = d_a * 1000.0
    d_b_mm = d_b * 1000.0

    if not (_PAIR_DIST_MIN_MM <= d_a_mm <= _PAIR_DIST_MAX_MM and
            _PAIR_DIST_MIN_MM <= d_b_mm <= _PAIR_DIST_MAX_MM):
        return None, (
            "Pair distances out of range: {:.1f} mm and {:.1f} mm "
            "(expected {:.0f}-{:.0f} mm). Wrong rigid body selected?"
        ).format(d_a_mm, d_b_mm, _PAIR_DIST_MIN_MM, _PAIR_DIST_MAX_MM)

    if abs(d_a_mm - d_b_mm) > _PAIR_DIST_DIFF_MAX_MM:
        return None, (
            "Pair distances differ by {:.1f} mm (> {:.1f} mm threshold): "
            "{:.1f} mm vs {:.1f} mm."
        ).format(abs(d_a_mm - d_b_mm), _PAIR_DIST_DIFF_MAX_MM, d_a_mm, d_b_mm)

    def _mid(pair):
        a = pos[pair[0]]
        b = pos[pair[1]]
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    centers_m = (_mid(pair_a), _mid(pair_b))
    info = "pairs: ({}+{}) d={:.1f}mm, ({}+{}) d={:.1f}mm".format(
        pair_a[0], pair_a[1], d_a_mm, pair_b[0], pair_b[1], d_b_mm,
    )
    return centers_m, info


def _write_marker_user_data(oid, marker_id, marker, rigid_body_name, scale):
    """Attach all available NatNet metadata to the baked point as user text.

    Stored under both the default (None) and the ``mocap`` plugin section
    so it's visible in the Properties > User Text panel and queryable via
    ``rs.GetUserText(oid, key)``.
    """
    pos = marker.get("pos") or (0.0, 0.0, 0.0)
    size = _scalar(marker.get("size", 0.0))
    err_m = float(_scalar(marker.get("error", 0.0)))
    param = marker.get("param", 0)
    is_valid = marker.get("is_valid", True)
    rhino_pt = _mocap_to_rhino_point(pos, scale)
    fields = {
        "rigid_body": str(rigid_body_name),
        "rigid_body_id": str(_RIGID_BODIES.get(rigid_body_name, "")),
        "marker_id": str(marker_id),
        "is_valid": str(bool(is_valid)),
        "param": str(int(param) if param is not None else 0),
        "size_m": "{:.6f}".format(float(size)),
        "residual_mm": "{:.4f}".format(err_m * 1000.0),
        "mocap_pos_m_x": "{:.6f}".format(float(pos[0])),
        "mocap_pos_m_y": "{:.6f}".format(float(pos[1])),
        "mocap_pos_m_z": "{:.6f}".format(float(pos[2])),
        "rhino_pos_x": "{:.6f}".format(rhino_pt.X),
        "rhino_pos_y": "{:.6f}".format(rhino_pt.Y),
        "rhino_pos_z": "{:.6f}".format(rhino_pt.Z),
        "axis_remap": "rhino=(mx, -mz, my)*scale  scale={:.6f}".format(scale),
        "source": "NatNet labeled_marker",
    }
    for key, value in fields.items():
        rs.SetUserText(oid, key, value)


def _bake_markers_and_line(markers, centers_m, rigid_body_name):
    """Delete any prior bake for this rigid body, then re-bake points + line.

    Returns ``(num_points, line_action)`` where ``line_action`` is
    ``"created"`` or ``"replaced"``.
    """
    scale = Rhino.RhinoMath.UnitScale(
        Rhino.UnitSystem.Meters, sc.doc.ModelUnitSystem
    )
    removed = _delete_existing_markers(_LAYER_NAME, rigid_body_name)
    line_action = "replaced" if removed > 0 else "created"

    valid_items = []
    for mid, m in markers.items():
        pos = m.get("pos")
        if pos is None:
            continue
        if not m.get("is_valid", True):
            continue
        valid_items.append((mid, pos))
    valid_items.sort(key=lambda kv: kv[0])

    num_points = 0
    for i, (mid, pos) in enumerate(valid_items):
        pt = _mocap_to_rhino_point(pos, scale)
        oid = rs.AddPoint(pt)
        if oid is None:
            continue
        rs.ObjectName(oid, "{}_{}".format(rigid_body_name, i))
        rs.ObjectLayer(oid, _LAYER_NAME)
        _write_marker_user_data(oid, mid, markers[mid], rigid_body_name, scale)
        num_points += 1

    p0 = _mocap_to_rhino_point(centers_m[0], scale)
    p1 = _mocap_to_rhino_point(centers_m[1], scale)
    line_id = rs.AddLine(p0, p1)
    if line_id is not None:
        rs.ObjectName(line_id, rigid_body_name)
        rs.ObjectLayer(line_id, _LAYER_NAME)

    return num_points, line_action


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _fetch_markers(settings):
    """One-shot NatNet read. Returns (markers_dict, None) or (None, err_msg)."""
    state = _MocapState(settings["rigid_body_id"])
    client = NatNetClient()
    client.set_client_address(settings["client_ip"])
    client.set_server_address(settings["server_ip"])
    client.set_use_multicast(settings["use_multicast"])
    client.labeled_marker_listener = state.on_labeled_markers

    try:
        started = client.run()
    except OSError as exc:
        return None, "Socket setup failed: {}".format(exc)
    if started is False:
        _safe_shutdown(client)
        return None, (
            "Could not start NatNet client (check server/client IPs and "
            "that Motive Streaming is enabled)."
        )

    print("Waiting up to {:.1f}s for rigid body '{}' (id={}) from {} ...".format(
        _FRAME_TIMEOUT_S, settings["rigid_body_name"],
        settings["rigid_body_id"], settings["server_ip"],
    ))
    got = state.wait(_FRAME_TIMEOUT_S)
    # Brief grace so a complete labeled-marker packet lands.
    time.sleep(0.1)
    _safe_shutdown(client)

    markers = state.markers()
    if not got or not markers:
        return None, (
            "No labeled markers received for '{}' (id={}) within {:.1f}s. "
            "Check: Motive Streaming enabled; rigid body is visible; "
            "server/client IPs and multicast flag match Motive."
        ).format(
            settings["rigid_body_name"], settings["rigid_body_id"],
            _FRAME_TIMEOUT_S,
        )
    return markers, None


def main():
    # Create the managed layer up front so the user sees it immediately,
    # even if the connection fails before any bake.
    _ensure_layer(_LAYER_NAME, _LAYER_COLOR)
    sc.doc.Views.Redraw()

    while True:
        settings = _prompt_settings()
        if settings is None:
            print("Cancelled.")
            return

        markers, err = _fetch_markers(settings)
        if err is not None:
            print("[mocap] " + err)
            rs.MessageBox(err + "\n\nClick OK to adjust options and try again.")
            continue

        result = _validate_and_pair(markers)
        centers_m, info = result
        if centers_m is None:
            print("[mocap] validation failed: " + info)
            rs.MessageBox(
                "Marker validation failed:\n\n" + info +
                "\n\nClick OK to adjust options and try again."
            )
            continue

        num_points, line_action = _bake_markers_and_line(
            markers, centers_m, settings["rigid_body_name"]
        )
        rs.Redraw()
        print(
            "Baked {} markers + 1 line ({}) for '{}' on layer '{}'. {}".format(
                num_points, line_action,
                settings["rigid_body_name"], _LAYER_NAME, info,
            )
        )
        return


if __name__ == "__main__":
    main()
