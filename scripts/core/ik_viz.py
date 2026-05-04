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

from typing import Optional

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
_STICKY_TOOL_LAYER_PREV_VISIBLE = "bar_joint:ik_viz_tool_layer_prev_visible"
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
    # Bars/joints already exist in the doc as separately-managed Rhino
    # geometry; the IK preview only needs the robot + the two attached tools.
    return scene.add(
        robot_cell,
        draw_visual=draw_visual,
        draw_collision=draw_collision,
        draw_rigid_bodies=False,
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


def _resolve_layer(layer_key: Optional[str]) -> str:
    """Map an optional ``layer_key`` to the absolute Rhino layer path.

    ``layer_key=None`` -> bake directly on ``config.LAYER_IK_CACHE``.
    Otherwise -> ``config.LAYER_IK_CACHE + '::' + layer_key`` (sub-layer).
    """
    if not layer_key:
        return config.LAYER_IK_CACHE
    return config.LAYER_IK_CACHE + config.LAYER_PATH_SEP + layer_key


def _get_or_create_cached_scene_object(robot_cell, mesh_mode: str, layer_key: Optional[str]):
    """Return the per-(cell, layer) cached :class:`RobotCellObject`,
    rebuilding when needed.

    A rebuild is triggered when:

    * no entry is cached for ``(id(robot_cell), layer_key)``; or
    * the cached entry was built for a different ``mesh_mode``.

    Other cache entries are left untouched.  When a rebuild is needed for
    one entry, only that entry's layer is purged.
    """
    _flush_cache_layer_once()
    cache = _get_cell_cache()
    layer_name = _resolve_layer(layer_key)
    key = (id(robot_cell), layer_name)
    entry = cache.get(key)
    if entry is not None and entry.get("mesh_mode") == mesh_mode:
        return entry["obj"]

    if entry is not None:
        # Mesh-mode swap on this cell: scrub only its sub-layer.
        print(f"ik_viz: mesh_mode changed for cell on '{layer_name}'; rebuilding.")
        _delete_layer_objects(layer_name)
        cache.pop(key, None)

    ensure_layer(layer_name)
    scene_object = _create_cell_scene_object(robot_cell, mesh_mode, layer_name)
    cache[key] = {"obj": scene_object, "mesh_mode": mesh_mode, "layer": layer_name}
    return scene_object


# ---------------------------------------------------------------------------
# Public session API
# ---------------------------------------------------------------------------


def begin_session(
    robot_cell=None,
    mesh_mode: Optional[str] = None,
    hide_tool_instances: bool = True,
    layer_key: Optional[str] = None,
) -> None:
    """Enter an interactive IK preview session.

    * Ensures the IK cache layer (and the optional sub-layer for ``layer_key``)
      exists and is visible.
    * Optionally hides the user's robotic-tool-instance layer for the
      duration of the session (the cell's tool models are drawn by us
      instead).
    * Pre-creates the cached :class:`RobotCellObject` so the very first
      :func:`update_state` call is the only one that pays the bake cost.
    """
    if mesh_mode is None:
        mesh_mode = get_mesh_mode()
    if mesh_mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh_mode must be one of {_VALID_MESH_MODES}, got {mesh_mode!r}")
    set_mesh_mode(mesh_mode)

    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()

    ensure_layer(config.LAYER_IK_CACHE)
    rs.LayerVisible(config.LAYER_IK_CACHE, True)
    sub_layer = _resolve_layer(layer_key)
    if sub_layer != config.LAYER_IK_CACHE:
        ensure_layer(sub_layer)
        rs.LayerVisible(sub_layer, True)

    if hide_tool_instances and rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        prev_visible = rs.LayerVisible(config.LAYER_TOOL_INSTANCES)
        sc.sticky[_STICKY_TOOL_LAYER_PREV_VISIBLE] = bool(prev_visible)
        if prev_visible:
            rs.LayerVisible(config.LAYER_TOOL_INSTANCES, False)
    else:
        sc.sticky[_STICKY_TOOL_LAYER_PREV_VISIBLE] = None

    # Pre-build the cell scene object so the cache is hot before the first update.
    _get_or_create_cached_scene_object(robot_cell, mesh_mode, layer_key)


def update_state(
    state,
    *,
    robot_cell=None,
    mesh_mode: Optional[str] = None,
    layer_key: Optional[str] = None,
) -> None:
    """Apply ``state`` to the cached Rhino preview.

    First call (per ``(robot_cell, layer_key)``) bakes the robot + tool meshes
    into the resolved sub-layer; subsequent calls delta-transform them in
    place.  Use a distinct ``layer_key`` per cell when previewing several
    cells at once (e.g. ``"Assembly"`` and ``"Support"``) so each can be
    hidden independently via :func:`set_layer_visible`.
    """
    if mesh_mode is None:
        mesh_mode = get_mesh_mode()
    if mesh_mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh_mode must be one of {_VALID_MESH_MODES}, got {mesh_mode!r}")

    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()

    scene_object = _get_or_create_cached_scene_object(robot_cell, mesh_mode, layer_key)

    layer_name = _resolve_layer(layer_key)
    if rs.IsLayer(layer_name) and not rs.LayerVisible(layer_name):
        rs.LayerVisible(layer_name, True)

    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
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


def end_session() -> None:
    """Exit the IK preview session: hide the cache layer + restore the tool layer.

    Geometry on the cache layer is intentionally NOT deleted; the next session
    will pick it up via the cached scene object and continue transforming it
    from its current pose.
    """
    if rs.IsLayer(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, False)

    prev_visible = sc.sticky.pop(_STICKY_TOOL_LAYER_PREV_VISIBLE, None)
    if prev_visible is True and rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        rs.LayerVisible(config.LAYER_TOOL_INSTANCES, True)


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
