"""Rhino-side drawing cache: robots, tools and bodies of a core `Scene`, for IK previews and plan viewing.

What this module owns
---------------------
For each ``layer_key`` (one preview, e.g. "Assembly" or "SupportSolve") it keeps one entry per part of the scene,
keyed by the part's id (``core.design_display.drawn_parts``, from the core's ``scene_parts``): bodies ("bars/B3"),
robot links with shapes ("robots/cindy/left_ur_arm_wrist_3_link") and mounted tools ("tools/cindy/AT3L"). Each part is
baked into the Rhino document ONCE, a visual and a collision mesh, each named by the part's id, on the sub-layers
``LAYER_IK_CACHE::<layer_key>::Visual`` and ``::Collision``. No compas: the meshes come from the core's `Geometry`.

How the meshes move
-------------------
:func:`update_state` moves each part's objects by the change in its transform since the last call, and hides the
parts the scene does not show (disabled bodies, robots that are absent or parked, see
``design_display.parked``). A part is baked again only when its `Geometry` object changes (the core shares them and
never changes one); parts whose id leaves the scene are deleted, new ids are baked. Switching between the visual
and the collision meshes is a sub-layer visibility flip -- nothing is baked again.

Session lifecycle
-----------------

* :func:`begin_session` shows the cache layer and hides the user-modeled document layers that overlap the preview;
  :func:`end_session` hides the cache layer and restores them.
* :func:`update_state` draws a scene on one layer key; the other layer keys are hidden (one preview at a time).
* :func:`set_active_mesh_mode` flips which mesh set is visible.
* :func:`objects_of` finds the drawn objects of design ids (offender highlighting); :func:`robot_meshes` copies a
  robot's meshes for a see-through ghost without touching the document.
* :func:`end_session` leaves the geometry in place, so the next session only moves it;
  :func:`discard_cache` deletes everything (hard reset).
"""

from __future__ import annotations

import hashlib
import os
from typing import Dict, Iterable, List, Optional

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino  # noqa: F401  (kept so other scripts can import Rhino through this file)
import session_log

from core import config
from core import joint_name_conventions as jnc
from core.rhino_helpers import ensure_layer, numpy_to_xform

# ---------------------------------------------------------------------------
# * Shared layer-key names
# ---------------------------------------------------------------------------
# Both rs_show_bar_action_plan and rs_ik_keyframe use these names, so they land in the same
# cache slot and share one baked preview.

LAYER_KEY_ASSEMBLY = "Assembly"
LAYER_KEY_SUPPORT = "Support"


# ---------------------------------------------------------------------------
# * Sticky-dictionary keys
# ---------------------------------------------------------------------------
# Rhino's ``sc.sticky`` is a dictionary that survives between script runs in the
# same Rhino session, so we stash our cache there under these keys.

# The part cache maps layer_key -> {"layer": Rhino layer path, "parts": {part id: part}, "mode": mesh mode}, a part
# being {"geometry", "owner", "xform" (4x4, document units), "visual": [guids], "collision": [guids], "shown"}.
_STICKY_PART_CACHE = "bar_joint:ik_viz_part_cache"
# (id(geometry), mesh mode, scale) -> (geometry, Rhino mesh at the identity, document units): shared by every layer
# and by the ghosts. The geometry is kept so its id is not reused.
_STICKY_MESHES = "bar_joint:ik_viz_meshes"
_STICKY_CACHE_INITIALIZED = "bar_joint:ik_viz_cache_initialized"
_STICKY_HIDDEN_DOC_LAYERS = "bar_joint:ik_viz_hidden_doc_layers"
_STICKY_MESH_MODE = "bar_joint:ik_viz_mesh_mode"
# Which Rhino document this cache was built for. If the user opens a different
# .3dm file in the same Rhino session, the cached parts point at Rhino
# objects that no longer exist, and leftover meshes from the previous file may
# still sit on LAYER_IK_CACHE -- both need to be cleared.
_STICKY_DOC_SERIAL = "bar_joint:ik_viz_doc_serial"


# ---------------------------------------------------------------------------
# * Which mesh set is shown: display mesh vs. collision mesh
# ---------------------------------------------------------------------------

MESH_MODE_VISUAL = "visual"
MESH_MODE_COLLISION = "collision"
_VALID_MESH_MODES = (MESH_MODE_VISUAL, MESH_MODE_COLLISION)
#: Sub-layer of each mesh mode under a layer key.
_MODE_LAYERS = {MESH_MODE_VISUAL: "Visual", MESH_MODE_COLLISION: "Collision"}


def set_mesh_mode(mode: str) -> None:
    """Remember which mesh set (display or collision) later calls should show.

    Args:
        mode (str): either ``"visual"`` or ``"collision"``.
    """
    if mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh mode must be one of {_VALID_MESH_MODES}, got {mode!r}")
    sc.sticky[_STICKY_MESH_MODE] = mode


def get_mesh_mode() -> str:
    """Return the currently remembered mesh set, defaulting to ``"visual"``."""
    return sc.sticky.get(_STICKY_MESH_MODE, MESH_MODE_VISUAL)


def _check_mode(mesh_mode: str) -> str:
    """The mesh mode, or a ValueError if it is not one."""
    if mesh_mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh mode must be one of {_VALID_MESH_MODES}, got {mesh_mode!r}")
    return mesh_mode


# ---------------------------------------------------------------------------
# * Internals: units, layers, meshes
# ---------------------------------------------------------------------------


def _doc_units_per_metre() -> float:
    """Document units in one metre (1000 in a millimetre document); the core's geometry is in metres."""
    from core.rhino_helpers import doc_unit_scale_to_mm

    return 1000.0 / doc_unit_scale_to_mm()


def _delete_layer_objects(layer_name: str, recursive: bool = True) -> int:
    """Delete every Rhino object on a layer (and its sub-layers) and count them.

    ``rs.ObjectsByLayer`` only matches one exact layer path, so without walking
    the sub-layers we would miss baked meshes that live on
    ``LAYER_IK_CACHE::<layer_key>::<MeshMode>`` sub-layers.

    Args:
        layer_name (str): the layer to clear.
        recursive (bool): also clear every descendant sub-layer. Defaults to True.

    Returns:
        int: how many Rhino objects were deleted.
    """
    if not rs.IsLayer(layer_name):
        return 0
    layers = [layer_name]
    if recursive:
        i = 0
        while i < len(layers):
            kids = rs.LayerChildren(layers[i]) or []
            layers.extend(kids)
            i += 1
    all_oids = []
    for lname in layers:
        oids = rs.ObjectsByLayer(lname) or []
        if oids:
            all_oids.extend(oids)
    return _delete(all_oids)


def _delete(guids) -> int:
    """Delete Rhino objects without redrawing in between, hidden and locked ones too; returns how many went.

    - ! Not `rs.DeleteObjects`: it deletes objects in normal mode only and skips the rest without a word, and the
      parts a scene does not show are hidden (`update_state`). Hidden previews were never deleted: a document
      saved after a few sessions carried tens of thousands of them, each "purged" again on every start.
    """
    if not guids:
        return 0
    deleted = 0
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        for guid in guids:
            obj = sc.doc.Objects.FindId(guid)
            # quiet, and ignoreModes: hidden and locked objects are deleted as well.
            if obj is not None and sc.doc.Objects.Delete(obj, True, True):
                deleted += 1
    finally:
        sc.doc.Views.RedrawEnabled = was
    return deleted


def _cache() -> dict:
    """Return the sticky part cache, creating an empty one on first use."""
    cache = sc.sticky.get(_STICKY_PART_CACHE)
    if cache is None:
        cache = sc.sticky[_STICKY_PART_CACHE] = {}
    return cache


def _flush_cache_layer_once() -> None:
    """Once per opened .3dm file, delete leftover preview meshes from before.

    A previous session (or a previous file saved in this same Rhino session) can
    leave stray meshes on ``LAYER_IK_CACHE``. This clears them the first time the
    cache is touched for a document.

    We notice a new document by watching ``sc.doc.RuntimeSerialNumber``: when it
    changes, the cached parts point at Rhino objects that are gone, so we drop the
    part cache and the "already cleaned" flag and start fresh.
    """
    try:
        cur_serial = int(sc.doc.RuntimeSerialNumber)
    except Exception:
        cur_serial = None
    prev_serial = sc.sticky.get(_STICKY_DOC_SERIAL)
    if cur_serial is not None and prev_serial != cur_serial:
        sc.sticky.pop(_STICKY_PART_CACHE, None)
        sc.sticky.pop(_STICKY_CACHE_INITIALIZED, None)
        sc.sticky[_STICKY_DOC_SERIAL] = cur_serial
    if sc.sticky.get(_STICKY_CACHE_INITIALIZED):
        return
    # Clear every temporary preview layer the IK commands draw onto. LAYER_IK_CACHE
    # holds the part meshes (on per-layer_key, per-mesh-mode sub-layers); the preview
    # layers hold inserted block copies (tool previews, the Robotiq gripper) placed by
    # rs_ik_keyframe. All of these are throwaway previews, so it is safe to delete them
    # on the first IK action in a freshly-opened document. "IKPineapplePreview" is an
    # old layer name we still clear so older documents tidy themselves up on first run.
    ensure_layer(config.LAYER_IK_CACHE)
    total = 0
    with session_log.step("purge old preview objects"):
        for lname in (config.LAYER_IK_CACHE, getattr(config, "SUPPORT_PREVIEW_LAYER", None), "IKPineapplePreview"):
            if lname:
                total += _delete_layer_objects(lname)
    if total:
        print(f"ik_viz: purged {total} orphan preview object(s) before initial bake.")
    sc.sticky[_STICKY_CACHE_INITIALIZED] = True


def _resolve_layer(layer_key: Optional[str]) -> str:
    """Turn a short ``layer_key`` into a full Rhino layer path.

    Returns ``LAYER_IK_CACHE`` on its own when ``layer_key`` is empty, otherwise
    ``LAYER_IK_CACHE::<layer_key>``.
    """
    if layer_key:
        return config.LAYER_PATH_SEP.join((config.LAYER_IK_CACHE, layer_key))
    return config.LAYER_IK_CACHE


def _mode_layer(parent: str, mesh_mode: str) -> str:
    """The sub-layer one mesh mode's objects live on."""
    return config.LAYER_PATH_SEP.join((parent, _MODE_LAYERS[mesh_mode]))


def _entry(layer_key: Optional[str]) -> dict:
    """The cache entry of one layer key, its layers made on first use."""
    _flush_cache_layer_once()
    cache = _cache()
    key = layer_key or ""
    entry = cache.get(key)
    if entry is None:
        layer = _resolve_layer(layer_key)
        entry = cache[key] = {"layer": layer, "parts": {}, "mode": get_mesh_mode()}
    for mode in _VALID_MESH_MODES:
        if not rs.IsLayer(_mode_layer(entry["layer"], mode)):
            ensure_layer(_mode_layer(entry["layer"], mode))
    return entry


def _rhino_mesh(geometry, mesh_mode: str, scale: float):
    """One Rhino mesh of a geometry's visual or collision shapes, in its own frame, in document units; cached.

    Returns None if the geometry has no shapes of that kind (a body without collision shapes never collides).
    """
    from bar_assembly_core.geometry import shape_mesh

    meshes = sc.sticky.setdefault(_STICKY_MESHES, {})
    key = (id(geometry), mesh_mode, scale)
    found = meshes.get(key)
    if found is not None and found[0] is geometry:
        return found[1]
    shapes = geometry.visual if mesh_mode == MESH_MODE_VISUAL else geometry.collision
    if not shapes:
        meshes[key] = (geometry, None)
        return None
    triangle_sets = [shape_mesh(shape) for shape in shapes]
    n_faces = sum(len(triangles.faces) for triangles in triangle_sets)
    path = _disk_cache_path(triangle_sets, scale)
    mesh = _read_cached_mesh(path, n_faces) if path else None
    if mesh is None:
        mesh = Rhino.Geometry.Mesh()
        # ! One Python -> Rhino call per vertex and per face: a robot's visual meshes have 0.7 to 1 million
        # vertices, 7 s for a support robot. Hence the disk cache for the large ones.
        with session_log.step("build Rhino mesh"):
            for triangles in triangle_sets:
                offset = mesh.Vertices.Count
                for x, y, z in (np.asarray(triangles.vertices) * scale).tolist():
                    mesh.Vertices.Add(x, y, z)
                for a, b, c in (np.asarray(triangles.faces) + offset).tolist():
                    mesh.Faces.AddFace(a, b, c)
            mesh.Normals.ComputeNormals()
            mesh.Compact()
        if path:
            _write_cached_mesh(path, mesh)
    # ? Double-precision vertices: parts are moved by many small transforms, single precision would drift.
    mesh.Vertices.UseDoublePrecisionVertices = True
    meshes[key] = (geometry, mesh)
    return mesh


# --- Rhino meshes on disk ---
# Building a large mesh costs one Python -> Rhino call per vertex and face; reading it back from a .3dm file is one
# call. Measured in Rhino on a support robot (17 meshes, 686 000 vertices): built in 7.2 s, read in 0.16 s.

#: Meshes with fewer vertices plus faces than this are built every time (bars, joints: milliseconds each).
_DISK_CACHE_MIN_SIZE = 5000


def _disk_cache_dir() -> str:
    """Where the built meshes are kept between sessions: next to the Python packages of `rhino_env`."""
    import tempfile

    root = os.environ.get("LOCALAPPDATA") or tempfile.gettempdir()
    return os.path.join(root, "bar_joint_rhino", "mesh_cache")


def _disk_cache_path(triangle_sets, scale: float) -> Optional[str]:
    """The cache file of a mesh, named by its content (vertices, faces, scale); None for a small mesh.

    ? Named by content, so a changed robot or tool file gives a new name; the old file is just never read again.
    Set `RS_MESH_CACHE=0` to switch the cache off.
    """
    if os.environ.get("RS_MESH_CACHE", "1") == "0":
        return None
    if sum(len(t.vertices) + len(t.faces) for t in triangle_sets) < _DISK_CACHE_MIN_SIZE:
        return None
    digest = hashlib.sha1(f"v1 {scale!r}".encode())
    for triangles in triangle_sets:
        digest.update(np.ascontiguousarray(triangles.vertices, dtype=np.float64).tobytes())
        digest.update(np.ascontiguousarray(triangles.faces, dtype=np.int64).tobytes())
    return os.path.join(_disk_cache_dir(), digest.hexdigest() + ".3dm")


def _read_cached_mesh(path: str, n_faces: int):
    """The mesh kept in a cache file, or None if there is none or it does not fit (then it is built again)."""
    if not os.path.isfile(path):
        return None
    try:
        with session_log.step("read mesh from disk cache"):
            for obj in Rhino.FileIO.File3dm.Read(path).Objects:
                mesh = obj.Geometry.DuplicateMesh()  # its own copy: the file object is dropped
                if mesh.Faces.Count == n_faces:
                    return mesh
    except Exception as exc:  # noqa: BLE001 -- an unreadable cache file must never stop a preview
        session_log.caught(exc)
    return None


def _write_cached_mesh(path: str, mesh) -> None:
    """Keep a built mesh for later sessions; written beside its name, then moved, so a file is never half there."""
    try:
        with session_log.step("write mesh to disk cache"):
            os.makedirs(os.path.dirname(path), exist_ok=True)
            partial = path[:-4] + ".part.3dm"
            file = Rhino.FileIO.File3dm()
            file.Objects.AddMesh(mesh)
            if file.Write(partial, 8):
                os.replace(partial, path)
    except Exception as exc:  # noqa: BLE001 -- the cache is an optimization only
        session_log.caught(exc)


def _pose_xform(pose, scale: float) -> np.ndarray:
    """A world pose as a 4x4 transform in document units."""
    matrix = pose.matrix()
    matrix[:3, 3] *= scale
    return matrix


def _bake(part_id: str, geometry, layer: str, scale: float) -> dict:
    """Bake a part's visual and collision meshes at the identity, named by its id, on its layer's sub-layers."""
    baked = {}
    for mode in _VALID_MESH_MODES:
        mesh = _rhino_mesh(geometry, mode, scale)
        baked[mode] = []
        if mesh is None:
            continue
        attributes = Rhino.DocObjects.ObjectAttributes()
        attributes.LayerIndex = sc.doc.Layers.FindByFullPath(_mode_layer(layer, mode), -1)
        attributes.Name = part_id
        with session_log.step("add mesh to document"):
            baked[mode].append(sc.doc.Objects.AddMesh(mesh, attributes))
    return baked


def _guids(part: dict) -> list:
    """Every Rhino object of a part, both mesh modes."""
    return part[MESH_MODE_VISUAL] + part[MESH_MODE_COLLISION]


def _show_mode(entry: dict, mesh_mode: str) -> None:
    """Show one mesh mode's sub-layer of an entry and hide the other's."""
    for mode in _VALID_MESH_MODES:
        layer = _mode_layer(entry["layer"], mode)
        if rs.IsLayer(layer) and bool(rs.LayerVisible(layer)) != (mode == mesh_mode):
            rs.LayerVisible(layer, mode == mesh_mode)
    entry["mode"] = mesh_mode


# ---------------------------------------------------------------------------
# * Public session API
# ---------------------------------------------------------------------------


@session_log.timed("ik_viz.begin_session")
def begin_session(
    mesh_mode: Optional[str] = None,
    hide_tool_instances: bool = True,  # back-compat shim; use hide_doc_layers
    layer_key: Optional[str] = None,
    *,
    active_mesh_mode: Optional[str] = None,
    hide_doc_layers: Optional[Iterable[str]] = None,
) -> None:
    """Start an interactive preview session.

    * Shows the cache layer and ``layer_key``'s sub-layer, with only ``active_mesh_mode`` visible.
    * Hides every layer in ``hide_doc_layers`` for the length of the session and
      restores each one's previous visibility in :func:`end_session`. The default
      is the user-modeled document layers that overlap the preview (tube previews,
      female/male joint copies, robotic-tool block copies) so the cached
      robot/tool/environment meshes are the only thing on screen. Pass an empty
      list to keep everything visible.

    The meshes are baked by the first :func:`update_state`.

    Args:
        mesh_mode (str | None): older name of ``active_mesh_mode``.
        hide_tool_instances (bool): older flag; when False the tool-copy layer is
            left visible. Prefer ``hide_doc_layers``.
        layer_key (str | None): which preview's sub-layer to use.
        active_mesh_mode (str | None): which mesh mode to show first; None keeps the remembered one.
        hide_doc_layers (Iterable[str] | None): document layers to hide for the
            session; ``None`` uses the default overlapping layers.
    """
    active_mesh_mode = _check_mode(active_mesh_mode or mesh_mode or get_mesh_mode())
    set_mesh_mode(active_mesh_mode)

    ensure_layer(config.LAYER_IK_CACHE)
    rs.LayerVisible(config.LAYER_IK_CACHE, True)
    entry = _entry(layer_key)
    if entry["layer"] != config.LAYER_IK_CACHE:
        rs.LayerVisible(entry["layer"], True)
    _show_mode(entry, active_mesh_mode)

    if hide_doc_layers is None:
        # Default: the user-modeled document layers that overlap the preview
        # (tubes / joints / tool blocks). Bar centerlines stay visible so the
        # user can still pick them.
        hide_doc_layers = replaced_doc_layers()
        if not hide_tool_instances:
            hide_doc_layers = tuple(
                ln for ln in hide_doc_layers if ln != config.LAYER_TOOL_INSTANCES
            )
    _hide_doc_layers(hide_doc_layers)


def replaced_doc_layers() -> tuple:
    """The document layers a preview hides by default: what it draws itself (tubes, joint and tool blocks)."""
    return (config.LAYER_BAR_TUBE_PREVIEWS, *jnc.JOINT_LAYERS, config.LAYER_TOOL_INSTANCES)


def _hide_doc_layers(layer_names: Iterable[str]) -> None:
    """Hide each existing layer, recording its previous visibility.

    The recorded visibilities are restored by :func:`end_session`.
    """
    prev = dict(sc.sticky.get(_STICKY_HIDDEN_DOC_LAYERS) or {})
    for ln in layer_names:
        if not ln or not rs.IsLayer(ln):
            continue
        if ln in prev:
            continue  # already recorded by an earlier begin_session
        prev[ln] = bool(rs.LayerVisible(ln))
        if prev[ln]:
            rs.LayerVisible(ln, False)
    sc.sticky[_STICKY_HIDDEN_DOC_LAYERS] = prev


def _restore_hidden_doc_layers() -> None:
    """Restore the visibility of layers hidden by :func:`_hide_doc_layers`."""
    prev = sc.sticky.pop(_STICKY_HIDDEN_DOC_LAYERS, None) or {}
    for ln, was_visible in prev.items():
        if was_visible and rs.IsLayer(ln):
            rs.LayerVisible(ln, True)


@session_log.timed("ik_viz.update_state")
def update_state(
    scene,
    *,
    robot_id: Optional[str] = None,
    mesh_mode: Optional[str] = None,
    layer_key: Optional[str] = None,
    hide_bodies: bool = False,
    colors=None,
) -> None:
    """Draw a scene on one preview layer: bake new parts, move the others, hide what the scene does not show.

    Args:
        scene: The core `Scene` to draw; not changed.
        robot_id: The acting robot: drawn even while some of its joints are unknown (at 0). Other robots with
            unknown joints, and robots that are absent or whose base is unknown, are hidden.
        mesh_mode (str | None): which mesh mode to show; None keeps the layer's.
        layer_key (str | None): which preview's sub-layer to draw on; the other previews are hidden.
        hide_bodies (bool): draw the robots and tools only (the document's own
            geometry shows the bars, e.g. in the Grasshopper preview).
        colors (Mapping[str, tuple] | None): id -> (r, g, b). A part takes the colour of its own id
            ("bars/B3", "joints/J1-3_male") or else of the robot it belongs to ("robots/cindy": its links
            and mounted tools); every other part is drawn in its layer's colour. None: no part is coloured,
            and parts an earlier call coloured go back to the layer's.
    """
    from core.design_display import drawn_parts

    entry = _entry(layer_key)
    parts = entry["parts"]
    scale = _doc_units_per_metre()
    wanted = drawn_parts(scene, (robot_id,) if robot_id else (), hide_bodies=hide_bodies)
    # ! Never a second copy of what the document already shows in the same place: two coplanar surfaces
    #   flicker as the view moves (z-fighting). The walkable ground stays the document's (it is picked on and
    #   highlighted); the obstacles too, unless the caller hid their layer for the session.
    for prefix, layer in (("ground/", config.LAYER_WALKABLE_GROUND), ("obstacles/", config.LAYER_ENVIRONMENT)):
        if rs.IsLayer(layer) and rs.LayerVisible(layer):
            for part_id in [p for p in wanted if p.startswith(prefix)]:
                wanted[part_id] = wanted[part_id]._replace(pose=None)

    # A layer hidden by `robot_meshes`' callers, `end_session` or another preview is shown again.
    if rs.IsLayer(config.LAYER_IK_CACHE) and not rs.LayerVisible(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, True)
    if entry["layer"] != config.LAYER_IK_CACHE and not rs.LayerVisible(entry["layer"]):
        rs.LayerVisible(entry["layer"], True)
    # * One preview at a time: the other layer keys' meshes (another command's last pose) are hidden.
    for other in _cache().values():
        if other is not entry and other["layer"] != config.LAYER_IK_CACHE and rs.IsLayer(other["layer"]) \
                and rs.LayerVisible(other["layer"]):
            rs.LayerVisible(other["layer"], False)

    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        gone = [part_id for part_id in parts if part_id not in wanted]
        _delete([g for part_id in gone for g in _guids(parts[part_id])])
        for part_id in gone:
            del parts[part_id]

        hide, show = [], []
        for part_id, drawn in wanted.items():
            part = parts.get(part_id)
            if part is not None and part["geometry"] is not drawn.geometry:
                _delete(_guids(part))
                del parts[part_id]
                part = None
            if drawn.pose is None:
                if part is not None and part["shown"]:
                    hide += _guids(part)
                    part["shown"] = False
                continue  # ? Never shown yet: baked when it first is.
            if part is None:
                part = parts[part_id] = {"geometry": drawn.geometry, "owner": drawn.owner, "xform": np.eye(4),
                                         "shown": True, **_bake(part_id, drawn.geometry, entry["layer"], scale)}
            part["owner"] = drawn.owner
            target = _pose_xform(drawn.pose, scale)
            delta = target @ np.linalg.inv(part["xform"])
            if not np.allclose(delta, np.eye(4), rtol=0.0, atol=1e-9):
                guids = _guids(part)
                if guids:
                    # ? Keep the ids Rhino returns (the same ones, unless it replaced an object).
                    with session_log.step("move part"):
                        moved = list(rs.TransformObjects(guids, numpy_to_xform(delta)) or guids)
                    count = len(part[MESH_MODE_VISUAL])
                    part[MESH_MODE_VISUAL], part[MESH_MODE_COLLISION] = moved[:count], moved[count:]
                part["xform"] = target
            if not part["shown"]:
                show += _guids(part)
                part["shown"] = True
        if hide:
            rs.HideObjects(hide)
        if show:
            rs.ShowObjects(show)
        colors = colors or {}
        for part_id, part in parts.items():
            wanted_color = colors.get(part_id) or colors.get(part.get("owner"))
            part["color"] = tuple(wanted_color) if wanted_color else None
        _apply_colors(parts)
        _show_mode(entry, _check_mode(mesh_mode) if mesh_mode else entry["mode"])
    finally:
        sc.doc.Views.RedrawEnabled = was
    sc.doc.Views.Redraw()


def _apply_colors(parts: dict, force: bool = False) -> None:
    """Give the parts their remembered colour (`part["color"]`; None: the layer's), one Rhino call per colour.

    Args:
        force: Paint every coloured part again, not only those whose colour changed since it was last applied
            (a caller's own highlight, reverted to by-layer, took the colour off without this module knowing).
    """
    by_color: Dict[Optional[tuple], list] = {}
    for part in parts.values():
        color = part.get("color")
        if (color is None and force) or (not force and part.get("applied") == color):
            continue
        by_color.setdefault(color, []).extend(_guids(part))
        part["applied"] = color
    if not by_color:
        return
    with session_log.step("color parts"):
        for color, guids in by_color.items():
            guids = [g for g in guids if rs.IsObject(g)]
            if not guids:
                continue
            if color is None:
                rs.ObjectColorSource(guids, 0)  # by layer
            else:
                rs.ObjectColor(guids, color)  # also sets the colour source to "by object"


def restore_colors(layer_key: Optional[str]) -> None:
    """Paint the coloured parts of a preview again, after a caller's own highlight was reverted to by-layer."""
    _apply_colors(_entry(layer_key)["parts"], force=True)


def set_layer_visible(layer_key: Optional[str], visible: bool) -> None:
    """Show or hide one preview's layer of the cache.

    ``layer_key=None`` toggles the root cache layer, which cascades to all its
    sub-layers. Use this to hide one preview (for example the support arm
    when the active bar carries no support load) while leaving others visible.

    Args:
        layer_key (str | None): the preview to toggle; ``None`` for the root.
        visible (bool): show it when True, hide it when False.
    """
    layer_name = _resolve_layer(layer_key)
    if rs.IsLayer(layer_name):
        rs.LayerVisible(layer_name, bool(visible))


def set_active_mesh_mode(layer_key: Optional[str], mesh_mode: str) -> None:
    """Show one mesh mode's objects of a preview and hide the other's (sub-layer visibility, nothing is baked).

    Args:
        layer_key (str | None): which preview to switch.
        mesh_mode (str): ``"visual"`` or ``"collision"``.
    """
    set_mesh_mode(_check_mode(mesh_mode))
    _show_mode(_entry(layer_key), mesh_mode)


def objects_of(ids: Iterable[str], layer_key: Optional[str], mesh_mode: str) -> List:
    """The Rhino objects drawn for design ids on one preview, in one mesh mode (e.g. collision offenders).

    An id is a part ("bars/B3", "tools/cindy/AT3L", "robots/cindy/left_ur_arm_wrist_3_link"), or a robot
    ("robots/alice": every link and tool it carries). Ids drawn nowhere (a link without shapes) give nothing.

    Args:
        ids: The design ids.
        layer_key (str | None): the preview.
        mesh_mode (str): ``"visual"`` or ``"collision"``.
    """
    entry = _cache().get(layer_key or "")
    if entry is None:
        return []
    parts = entry["parts"]
    out = []
    for object_id in ids:
        if object_id in parts:
            out += parts[object_id][_check_mode(mesh_mode)]
        else:
            out += [g for part in parts.values() if part["owner"] == object_id for g in part[mesh_mode]]
    return out


def robot_meshes(robot, mesh_mode: str = MESH_MODE_VISUAL) -> List:
    """Copies of a robot's meshes (links and mounted tools) where its base and joints put them, document units.

    For see-through previews (``core.dynamic_preview``) that own their meshes; nothing is added to the document.
    Parts without shapes of `mesh_mode` give their other mesh instead.

    Args:
        robot: A core `RobotObject`, e.g. from ``design_display.ghost_scene``.
        mesh_mode (str): which meshes to copy.
    """
    from bar_assembly_core.scene import robot_parts

    other = MESH_MODE_COLLISION if _check_mode(mesh_mode) == MESH_MODE_VISUAL else MESH_MODE_VISUAL
    scale = _doc_units_per_metre()
    meshes = []
    for _part_id, geometry, pose in robot_parts(robot):
        mesh = _rhino_mesh(geometry, mesh_mode, scale) or _rhino_mesh(geometry, other, scale)
        if mesh is None:
            continue
        copy = mesh.DuplicateMesh()
        copy.Transform(numpy_to_xform(_pose_xform(pose, scale)))
        meshes.append(copy)
    return meshes


@session_log.timed("ik_viz.end_session")
def end_session() -> None:
    """End the preview session: hide the cache layer and restore the doc layers.

    The geometry on the cache layer is deliberately NOT deleted; the next session
    picks it up through the cached parts and keeps moving it from its current pose.
    """
    if rs.IsLayer(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, False)

    _restore_hidden_doc_layers()


@session_log.timed("ik_viz.discard_cache")
def discard_cache() -> None:
    """Hard reset: delete every object on the cache layer (and its sub-layers) and drop the cached parts and meshes.

    Use this only for a hard reset (robot or tool files changed, or an explicit user flush). Switching bars or
    environments does NOT need it -- :func:`update_state` keeps the parts in sync as it goes.
    """
    sc.sticky.pop(_STICKY_PART_CACHE, None)
    sc.sticky.pop(_STICKY_MESHES, None)
    sc.sticky.pop(_STICKY_CACHE_INITIALIZED, None)
    _delete_layer_objects(config.LAYER_IK_CACHE)
    sc.doc.Views.Redraw()
