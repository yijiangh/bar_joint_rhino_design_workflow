"""Rhino-side scene cache for dual-arm robot visualization.

Backed by :class:`compas_fab.rhino.scene.RobotCellObject`, which inherits the
caching + delta-transform machinery from :class:`BaseRobotCellObject`. Each
call to :func:`update_state` therefore re-uses the same baked Rhino doc
geometry and only applies the *delta* transform of every robot link / tool;
no Rhino objects are deleted or re-baked between frames.

Session lifecycle
-----------------

Callers wrap their interactive preview between :func:`begin_session` and
:func:`end_session`. Inside the session they call :func:`update_state(state)`
each time the robot cell state changes (e.g. on pose cycle or bar switch).

* On the FIRST :func:`update_state` of a Rhino session (no cached scene
  object in ``sc.sticky``), the layer ``config.LAYER_IK_CACHE`` is purged
  of any orphan geometry from a previous Rhino instance, then the robot +
  tool meshes are baked into it.
* On subsequent calls, the cached :class:`RobotCellObject` updates its
  child SceneObjects, which delta-transform the existing Rhino doc
  objects in place via :func:`Rhino.RhinoDoc.Objects.Transform`.
* :func:`end_session` only HIDES the cache layer; the geometry stays in
  the doc so the next session can keep transforming it from where it
  was last left.
* :func:`discard_cache` is the hard reset (drops the cached scene object
  AND deletes everything on the cache layer); use it when the underlying
  ``RobotCell`` is rebuilt or when the user wants to flush state.

Legacy ``show_state`` / ``clear_scene`` API is preserved so older callers
(``rs_ik_support_keyframe.py``, ``yj_functions/...``) continue to work; both
delegate to the cached path internally.
"""

from __future__ import annotations

from typing import Iterable, Optional

import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino  # noqa: F401  -- kept for downstream callers that re-import

from core import config
from core.rhino_helpers import ensure_layer
from core.robot_cell import default_cell_state, get_or_load_robot_cell, import_compas_stack


# ---------------------------------------------------------------------------
# Sticky keys
# ---------------------------------------------------------------------------

# Multi-cell cache: {(id(robot_cell), layer_name): {"obj": <RobotCellObject>,
# "mesh_mode": str, "layer": str}}.  Each cell is baked onto its own (sub-)
# layer so callers can hide one cell without affecting others (e.g. hide the
# support arm without losing the dual-arm bake).  All sub-layers nest under
# ``config.LAYER_IK_CACHE`` so toggling that root in :func:`begin_session` /
# :func:`end_session` cascades.
_STICKY_CELL_CACHE = "bar_joint:ik_viz_cell_cache"
_STICKY_CACHE_INITIALIZED = "bar_joint:ik_viz_cache_initialized"
_STICKY_HIDDEN_DOC_LAYERS = "bar_joint:ik_viz_hidden_doc_layers"
_STICKY_MESH_MODE = "bar_joint:ik_viz_mesh_mode"

# Back-compat sticky keys still consulted by yj_functions/ scripts. The new
# cached-cell path no longer writes these, but legacy callers that pop them
# will simply find them empty / missing.
_STICKY_SCENE_OBJECT = "bar_joint:ik_viz_scene_object"
_STICKY_DRAWN_IDS = "bar_joint:ik_viz_drawn_ids"


# ---------------------------------------------------------------------------
# Public mesh-mode toggle
# ---------------------------------------------------------------------------

MESH_MODE_VISUAL = "visual"
MESH_MODE_COLLISION = "collision"
_VALID_MESH_MODES = (MESH_MODE_VISUAL, MESH_MODE_COLLISION)


def set_mesh_mode(mode: str) -> None:
    """Pin the mesh-display mode used by subsequent calls in this session."""
    if mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh mode must be one of {_VALID_MESH_MODES}, got {mode!r}")
    sc.sticky[_STICKY_MESH_MODE] = mode


def get_mesh_mode() -> str:
    """Return the current mesh-display mode (defaults to ``"visual"``)."""
    return sc.sticky.get(_STICKY_MESH_MODE, MESH_MODE_VISUAL)


# ---------------------------------------------------------------------------
# Internals: scaling, layer flushing, scene-object lifecycle
# ---------------------------------------------------------------------------


def _native_scale_for_doc() -> float:
    """compas geometry is in meters; the BaseRobotCellObject scales by
    ``1 / native_scale`` to reach the active Rhino doc unit. So ``native_scale``
    must be the ``meters per doc unit`` ratio.

    For a doc in millimeters: 1 doc unit = 0.001 m, so ``native_scale = 0.001``.
    """
    from core.rhino_frame_io import doc_unit_scale_to_mm

    # `doc_unit_scale_to_mm()` returns "mm per doc unit". Divide by 1000 to
    # get "m per doc unit", which is what BaseRobotModelObject expects.
    return doc_unit_scale_to_mm() / 1000.0


def _delete_layer_objects(layer_name: str) -> int:
    """Delete every Rhino doc object on ``layer_name``; return the count."""
    if not rs.IsLayer(layer_name):
        return 0
    oids = rs.ObjectsByLayer(layer_name) or []
    if not oids:
        return 0
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        rs.DeleteObjects(oids)
    finally:
        sc.doc.Views.RedrawEnabled = was
    return len(oids)


def _ensure_compas_fab_rhino_registered() -> None:
    """Trigger the compas_fab Rhino plugin registration so ``Scene().add(robot_cell)``
    yields our :class:`RobotCellObject` rather than falling back to the
    cross-context base (which would raise ``NotImplementedError`` in
    ``_initial_draw``).
    """
    try:
        import compas_fab.rhino.scene  # noqa: F401
    except Exception as exc:  # pragma: no cover -- defensive
        print(f"ik_viz: compas_fab.rhino.scene import failed ({exc}); "
              "RobotCell Rhino SceneObject will not be available.")


def _create_cell_scene_object(robot_cell, mesh_mode: str, layer_name: str):
    """Build a fresh :class:`RobotCellObject` for ``robot_cell``.

    The object's ``_initial_draw`` is deferred until the first ``update()`` /
    ``draw()`` call (handled by :class:`BaseRobotCellObject`).
    """
    _ensure_compas_fab_rhino_registered()
    deps = import_compas_stack()
    scene = deps["Scene"]()
    draw_visual = mesh_mode == MESH_MODE_VISUAL
    draw_collision = mesh_mode == MESH_MODE_COLLISION
    # RobotCellObject now always draws RobotModel + ToolModels + all
    # registered RigidBodies in the cell.
    return scene.add(
        robot_cell,
        draw_visual=draw_visual,
        draw_collision=draw_collision,
        native_scale=_native_scale_for_doc(),
        layer=layer_name,
    )


def _get_cell_cache() -> dict:
    cache = sc.sticky.get(_STICKY_CELL_CACHE)
    if cache is None:
        cache = {}
        sc.sticky[_STICKY_CELL_CACHE] = cache
    return cache


def _flush_cache_layer_once() -> None:
    """On the first cache touch in this Rhino session, delete any orphan
    geometry left over on ``LAYER_IK_CACHE`` from a prior Rhino instance.

    Once the per-session flag is set, subsequent cache rebuilds (e.g. mesh
    mode change) do their own targeted cleanup and skip the bulk purge.
    """
    if sc.sticky.get(_STICKY_CACHE_INITIALIZED):
        return
    ensure_layer(config.LAYER_IK_CACHE)
    n_purged = _delete_layer_objects(config.LAYER_IK_CACHE)
    if n_purged:
        print(f"ik_viz: purged {n_purged} orphan object(s) from "
              f"'{config.LAYER_IK_CACHE}' before initial bake.")
    sc.sticky[_STICKY_CACHE_INITIALIZED] = True


def _resolve_layer(layer_key: Optional[str], mesh_mode: Optional[str] = None) -> str:
    """Map ``(layer_key, mesh_mode)`` to the absolute Rhino layer path.

    ``LAYER_IK_CACHE [/<layer_key>] [/<MeshMode>]``.  ``mesh_mode`` segment
    is title-cased so it reads as a normal Rhino layer name ("Visual",
    "Collision").
    """
    parts = [config.LAYER_IK_CACHE]
    if layer_key:
        parts.append(layer_key)
    if mesh_mode:
        parts.append(mesh_mode.capitalize())
    return config.LAYER_PATH_SEP.join(parts)


def _cache_key(robot_cell, layer_key: Optional[str], mesh_mode: str):
    return (id(robot_cell), layer_key or "", mesh_mode)


def _get_or_create_cached_scene_object(robot_cell, mesh_mode: str, layer_key: Optional[str]):
    """Return the per-(cell, layer_key, mesh_mode) cached :class:`RobotCellObject`.

    Each (cell, layer_key, mesh_mode) gets its OWN sub-sub-layer
    (``LAYER_IK_CACHE/<layer_key>/<MeshMode>``) so visual + collision can be
    baked simultaneously and toggled by layer visibility alone -- no rebake
    on mesh-mode switch.
    """
    _flush_cache_layer_once()
    cache = _get_cell_cache()
    layer_name = _resolve_layer(layer_key, mesh_mode)
    key = _cache_key(robot_cell, layer_key, mesh_mode)
    entry = cache.get(key)
    if entry is not None:
        return entry["obj"]

    ensure_layer(layer_name)
    scene_object = _create_cell_scene_object(robot_cell, mesh_mode, layer_name)
    cache[key] = {
        "obj": scene_object,
        "mesh_mode": mesh_mode,
        "layer": layer_name,
        "layer_key": layer_key or "",
    }
    return scene_object


# ---------------------------------------------------------------------------
# Public session API
# ---------------------------------------------------------------------------


def _normalize_modes(mesh_modes, mesh_mode) -> tuple:
    if mesh_modes is not None:
        modes = tuple(mesh_modes)
    elif mesh_mode is not None:
        modes = (mesh_mode,)
    else:
        modes = (get_mesh_mode(),)
    for m in modes:
        if m not in _VALID_MESH_MODES:
            raise ValueError(f"mesh mode must be one of {_VALID_MESH_MODES}, got {m!r}")
    return modes


def begin_session(
    robot_cell=None,
    mesh_mode: Optional[str] = None,
    hide_tool_instances: bool = True,  # back-compat shim; use hide_doc_layers
    layer_key: Optional[str] = None,
    *,
    mesh_modes: Optional[Iterable[str]] = None,
    active_mesh_mode: Optional[str] = None,
    hide_doc_layers: Optional[Iterable[str]] = None,
) -> None:
    """Enter an interactive IK preview session.

    * Pre-bakes one :class:`RobotCellObject` per ``mesh_modes`` entry on its
      own sub-sub-layer so toggling visual<->collision is just a layer
      visibility flip later.
    * Ensures only ``active_mesh_mode``'s sub-sub-layer is visible.
    * Hides every layer in ``hide_doc_layers`` for the duration of the
      session and restores their previous visibility in :func:`end_session`.
      Defaults to the user-modeled doc layers that overlap the cell viz
      (tube previews, female/male joint instances, robotic-tool block
      instances) so the cached robot/tool/env meshes are the only thing on
      screen. Pass an empty list to keep everything visible.
    """
    modes = _normalize_modes(mesh_modes, mesh_mode)
    if active_mesh_mode is None:
        active_mesh_mode = modes[0]
    if active_mesh_mode not in _VALID_MESH_MODES:
        raise ValueError(f"active_mesh_mode must be one of {_VALID_MESH_MODES}, got {active_mesh_mode!r}")
    set_mesh_mode(active_mesh_mode)

    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()

    ensure_layer(config.LAYER_IK_CACHE)
    rs.LayerVisible(config.LAYER_IK_CACHE, True)
    parent = _resolve_layer(layer_key)
    if parent != config.LAYER_IK_CACHE:
        ensure_layer(parent)
        rs.LayerVisible(parent, True)

    if hide_doc_layers is None:
        # Default = the user-modeled doc layers that visually overlap the
        # cell viz (tubes / joints / tool blocks). Bar centerlines stay
        # visible so picking still works.
        hide_doc_layers = (
            config.LAYER_BAR_TUBE_PREVIEWS,
            config.LAYER_JOINT_FEMALE_INSTANCES,
            config.LAYER_JOINT_MALE_INSTANCES,
            config.LAYER_TOOL_INSTANCES,
        )
        if not hide_tool_instances:
            hide_doc_layers = tuple(
                ln for ln in hide_doc_layers if ln != config.LAYER_TOOL_INSTANCES
            )
    _hide_doc_layers(hide_doc_layers)

    # Pre-build a cached cell-object per mode so the cache is hot.
    for m in modes:
        _get_or_create_cached_scene_object(robot_cell, m, layer_key)
    set_active_mesh_mode(layer_key, active_mesh_mode)


def _hide_doc_layers(layer_names: Iterable[str]) -> None:
    """Hide each existing layer; record prev visibility for ``end_session`` restore."""
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
    prev = sc.sticky.pop(_STICKY_HIDDEN_DOC_LAYERS, None) or {}
    for ln, was_visible in prev.items():
        if was_visible and rs.IsLayer(ln):
            rs.LayerVisible(ln, True)


def update_state(
    state,
    *,
    robot_cell=None,
    mesh_mode: Optional[str] = None,
    layer_key: Optional[str] = None,
    mesh_modes: Optional[Iterable[str]] = None,
) -> None:
    """Apply ``state`` to the cached Rhino preview.

    Updates EVERY ``mesh_modes`` entry on ``layer_key`` (defaults to all
    cached entries for that cell+layer_key, so a viewer that pre-baked
    visual+collision keeps both in sync as the user changes pose).

    First call (per ``(robot_cell, layer_key, mode)``) bakes the meshes onto
    that mode's sub-sub-layer; subsequent calls delta-transform them in place.
    """
    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()

    if mesh_modes is None and mesh_mode is None:
        # Auto: update every mode that's cached for this (cell, layer_key).
        cache = _get_cell_cache()
        modes = tuple(
            entry["mesh_mode"]
            for k, entry in cache.items()
            if k[0] == id(robot_cell) and k[1] == (layer_key or "")
        )
        if not modes:
            modes = (get_mesh_mode(),)
    else:
        modes = _normalize_modes(mesh_modes, mesh_mode)

    parent = _resolve_layer(layer_key)
    if rs.IsLayer(parent) and not rs.LayerVisible(parent):
        rs.LayerVisible(parent, True)

    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        for m in modes:
            scene_object = _get_or_create_cached_scene_object(robot_cell, m, layer_key)
            scene_object.update(state)
    finally:
        sc.doc.Views.RedrawEnabled = was
    sc.doc.Views.Redraw()


def set_layer_visible(layer_key: Optional[str], visible: bool) -> None:
    """Toggle visibility of a single sub-layer of the IK cache.

    ``layer_key=None`` toggles the root cache layer (which cascades to all
    sub-layers).  Use this to hide one cell's preview (e.g. the support arm
    when the active bar has no support payload) while keeping others visible.
    """
    layer_name = _resolve_layer(layer_key)
    if rs.IsLayer(layer_name):
        rs.LayerVisible(layer_name, bool(visible))


def set_active_mesh_mode(layer_key: Optional[str], mesh_mode: str) -> None:
    """Show only ``mesh_mode``'s sub-sub-layer under ``layer_key``; hide the others.

    Cheap (no rebake): each mode's geometry was pre-baked by
    :func:`begin_session` onto its own sub-sub-layer.
    """
    if mesh_mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh mode must be one of {_VALID_MESH_MODES}, got {mesh_mode!r}")
    set_mesh_mode(mesh_mode)
    cache = _get_cell_cache()
    target_lkey = layer_key or ""
    for k, entry in cache.items():
        if k[1] != target_lkey:
            continue
        layer_name = entry["layer"]
        if rs.IsLayer(layer_name):
            rs.LayerVisible(layer_name, entry["mesh_mode"] == mesh_mode)


def get_cached_scene_object(robot_cell, layer_key: Optional[str], mesh_mode: str):
    """Return the cached :class:`RobotCellObject` for ``(cell, layer_key, mesh_mode)`` or ``None``."""
    cache = _get_cell_cache()
    entry = cache.get(_cache_key(robot_cell, layer_key, mesh_mode))
    return entry["obj"] if entry else None


def get_link_native_geometry(robot_cell, layer_key: Optional[str], mesh_mode: str) -> dict:
    """Return ``{link_name: [rhino_guid, ...]}`` for the cached robot model.

    Used by collision-highlight code that needs to recolor specific links.
    Returns ``{}`` if the cell/layer/mode is not cached or the underlying
    ``RobotModelObject`` has not run its initial draw yet.
    """
    so = get_cached_scene_object(robot_cell, layer_key, mesh_mode)
    if so is None or so._robot_model_scene_object is None:
        return {}
    rmo = so._robot_model_scene_object
    if mesh_mode == MESH_MODE_VISUAL:
        return dict(rmo._links_visual_mesh_native_geometry)
    return dict(rmo._links_collision_mesh_native_geometry)


def get_tool_native_geometry(
    robot_cell, layer_key: Optional[str], mesh_mode: str
) -> dict:
    """Return ``{tool_name: [rhino_guid, ...]}`` flattening every tool link."""
    so = get_cached_scene_object(robot_cell, layer_key, mesh_mode)
    if so is None:
        return {}
    out = {}
    for tool_name, tool_so in (so._tool_scene_objects or {}).items():
        if tool_so is None:
            continue
        guids = []
        meshes = (
            tool_so._links_visual_mesh_native_geometry
            if mesh_mode == MESH_MODE_VISUAL
            else tool_so._links_collision_mesh_native_geometry
        )
        for lst in meshes.values():
            guids.extend(lst)
        out[tool_name] = guids
    return out


def end_session() -> None:
    """Exit the IK preview session: hide the cache layer + restore the tool layer.

    Geometry on the cache layer is intentionally NOT deleted; the next session
    will pick it up via the cached scene object and continue transforming it
    from its current pose.
    """
    if rs.IsLayer(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, False)

    _restore_hidden_doc_layers()


def discard_cache() -> None:
    """Hard reset: delete every object on the cache layer (and all sub-layers)
    and drop the cached scene objects. Use this when the underlying robot cell
    has been rebuilt (e.g. on PyBullet restart) or when the user explicitly
    wants a clean slate.
    """
    cache = sc.sticky.pop(_STICKY_CELL_CACHE, None) or {}
    sc.sticky.pop(_STICKY_CACHE_INITIALIZED, None)
    seen_layers = set()
    for entry in cache.values():
        layer_name = entry.get("layer")
        if layer_name and layer_name not in seen_layers:
            _delete_layer_objects(layer_name)
            seen_layers.add(layer_name)
    if rs.IsLayer(config.LAYER_IK_CACHE) and config.LAYER_IK_CACHE not in seen_layers:
        _delete_layer_objects(config.LAYER_IK_CACHE)
    sc.doc.Views.Redraw()


# ---------------------------------------------------------------------------
# Back-compat shims (legacy single-shot API)
# ---------------------------------------------------------------------------


def show_state(state, mesh_mode: Optional[str] = None, *, robot_model=None) -> None:
    """Legacy single-shot draw entry point.

    Now routed through the cached :func:`update_state` path. The ``robot_model``
    argument is honored by re-keying the cache to the cell whose ``robot_model``
    matches; callers that mix dual-arm and support previews already swap the
    underlying cell in PyBullet via ``set_cell_state``, so we look up the
    correct cell from sticky as needed.

    On entry, the cache layer is forced visible (so legacy callers that do not
    use :func:`begin_session` still see the preview); the layer is left visible
    for the caller's own cleanup pass.
    """
    if mesh_mode is None:
        mesh_mode = get_mesh_mode()

    rcell = None
    if robot_model is not None:
        rcell = _resolve_cell_for_robot_model(robot_model)
    if rcell is None:
        rcell = get_or_load_robot_cell()

    ensure_layer(config.LAYER_IK_CACHE)
    rs.LayerVisible(config.LAYER_IK_CACHE, True)
    update_state(state, robot_cell=rcell, mesh_mode=mesh_mode)


def _resolve_cell_for_robot_model(robot_model):
    """Best-effort lookup of the RobotCell whose ``robot_model`` is ``robot_model``.

    Used only by the legacy :func:`show_state` shim so existing callers that
    pass a bare ``robot_model`` still bind the cache to a real cell.
    """
    try:
        rcell = get_or_load_robot_cell()
        if rcell.robot_model is robot_model:
            return rcell
    except Exception:
        pass
    try:
        from core import robot_cell_support
        scell = robot_cell_support.get_or_load_support_cell()
        if scell.robot_model is robot_model:
            return scell
    except Exception:
        pass
    return None


def reset_home() -> None:
    """Show the robot at its default cell state (zero configuration, identity base)."""
    show_state(default_cell_state())


def clear_scene() -> None:
    """Legacy clear: discard cache + delete all baked meshes.

    Modern callers should prefer :func:`end_session` (hides without deleting).
    """
    discard_cache()
