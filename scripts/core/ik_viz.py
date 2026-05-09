"""Rhino-side scene cache for dual-arm robot visualization.

Direct-ownership model: this module owns one :class:`RobotModelObject` for
the robot, a dict of :class:`RobotModelObject` for tools, and a dict of
:class:`RigidBodyObject` for rigid bodies, all per ``(robot_cell, layer_key)``.
This replaces the previous ``RobotCellObject`` wrapper so each piece can be
invalidated independently (e.g. RB keyset changes on bar switch don't
rebake the robot/tools).

Each scene object is baked ONCE with both ``draw_visual=True`` and
``draw_collision=True``; mesh-mode toggle is a per-GUID hide/show on the
inactive mode's native-geometry list (no rebake).

Session lifecycle
-----------------

* :func:`begin_session` builds the bundle for ``layer_key`` and pre-bakes
  it (so first :func:`update_state` is fast). Hides the user-modeled doc
  layers that overlap the cell viz; restores them in :func:`end_session`.
* :func:`update_state` applies a :class:`RobotCellState` to the bundle.
  RB keyset diffs against the cached set and adds/removes SOs in place.
* :func:`set_active_mesh_mode` flips per-GUID visibility, no rebake.
* :func:`end_session` hides the cache layer; geometry stays so the next
  session resumes by delta-transform.
* :func:`discard_cache` deletes everything; use only on hard reset.
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
# Shared layer-key constants (used by rs_show_ik, rs_ik_keyframe so both
# commands hit the same cache slot).
# ---------------------------------------------------------------------------

LAYER_KEY_ASSEMBLY = "Assembly"
LAYER_KEY_SUPPORT = "Support"


# ---------------------------------------------------------------------------
# Sticky keys
# ---------------------------------------------------------------------------

# Bundle cache: {(id(robot_cell), layer_key): {
#     "robot_so": RobotModelObject,
#     "tool_sos": {tool_id: RobotModelObject},
#     "rb_sos":   {rb_id: RigidBodyObject},
#     "rb_keyset": frozenset,
#     "active_mesh_mode": str,
#     "layer": str,
# }}
_STICKY_BUNDLE_CACHE = "bar_joint:ik_viz_bundle_cache"
_STICKY_CACHE_INITIALIZED = "bar_joint:ik_viz_cache_initialized"
_STICKY_HIDDEN_DOC_LAYERS = "bar_joint:ik_viz_hidden_doc_layers"
_STICKY_MESH_MODE = "bar_joint:ik_viz_mesh_mode"
# Doc serial: detects .3dm reopen within the same Rhino session. When the doc
# changes, the cached bundles point to deleted GUIDs and orphan geometry from
# the previous file's save sits on LAYER_IK_CACHE -- both must be flushed.
_STICKY_DOC_SERIAL = "bar_joint:ik_viz_doc_serial"

# Back-compat sticky keys still consulted by yj_functions/ scripts.
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


def _delete_layer_objects(layer_name: str, recursive: bool = True) -> int:
    """Delete every Rhino doc object on ``layer_name`` (and, by default, on
    every descendant sub-layer); return the total count.

    ``rs.ObjectsByLayer`` only matches the exact layer path, so without the
    recursive walk we'd miss baked geometry living on
    ``LAYER_IK_CACHE::<layer_key>::<MeshMode>`` sub-layers.
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
    if not all_oids:
        return 0
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        rs.DeleteObjects(all_oids)
    finally:
        sc.doc.Views.RedrawEnabled = was
    return len(all_oids)


def _ensure_compas_fab_rhino_registered() -> None:
    """Trigger the compas_fab Rhino plugin registration so :class:`Scene` adds
    yield our Rhino :class:`RobotModelObject` / :class:`RigidBodyObject`
    rather than the cross-context base (which raises ``NotImplementedError``).
    """
    try:
        import compas_fab.rhino.scene  # noqa: F401
    except Exception as exc:  # pragma: no cover -- defensive
        print(f"ik_viz: compas_fab.rhino.scene import failed ({exc}); "
              "Rhino SceneObjects will not be available.")


def _import_rhino_scene_classes():
    """Lazy-import the per-type Rhino scene classes (avoids module load order
    issues when this file is imported in non-Rhino tests)."""
    _ensure_compas_fab_rhino_registered()
    from compas.scene import SceneObject
    from compas_fab.rhino.scene import RobotModelObject, RigidBodyObject
    return SceneObject, RobotModelObject, RigidBodyObject


def _create_robot_so(robot_cell, layer_name: str):
    """Build a :class:`RobotModelObject` for the cell's robot model.

    Both visual and collision are baked at once; mode toggle is per-GUID
    visibility on the SO's internal native-geometry dicts.
    """
    SceneObject, RobotModelObject, _ = _import_rhino_scene_classes()
    return SceneObject(
        item=robot_cell.robot_model,
        sceneobject_type=RobotModelObject,
        draw_visual=True,
        draw_collision=True,
        native_scale=_native_scale_for_doc(),
        layer=layer_name,
    )


def _create_tool_so(tool_model, layer_name: str):
    SceneObject, RobotModelObject, _ = _import_rhino_scene_classes()
    return SceneObject(
        item=tool_model,
        sceneobject_type=RobotModelObject,
        draw_visual=True,
        draw_collision=True,
        native_scale=_native_scale_for_doc(),
        layer=layer_name,
    )


def _create_rb_so(rigid_body, layer_name: str):
    SceneObject, _, RigidBodyObject = _import_rhino_scene_classes()
    return SceneObject(
        item=rigid_body,
        sceneobject_type=RigidBodyObject,
        draw_visual=True,
        draw_collision=True,
        native_scale=_native_scale_for_doc(),
        layer=layer_name,
    )


def _bundle_cache() -> dict:
    cache = sc.sticky.get(_STICKY_BUNDLE_CACHE)
    if cache is None:
        cache = {}
        sc.sticky[_STICKY_BUNDLE_CACHE] = cache
    return cache


def _flush_cache_layer_once() -> None:
    """On the first cache touch per .3dm doc, purge any orphan geometry
    left on ``LAYER_IK_CACHE`` from a prior session / a previous file save.

    Detects .3dm reopen (same Rhino session, different doc) via
    ``sc.doc.RuntimeSerialNumber``: when it changes, drop the bundle cache and
    the init flag so the next bake starts clean.
    """
    try:
        cur_serial = int(sc.doc.RuntimeSerialNumber)
    except Exception:
        cur_serial = None
    prev_serial = sc.sticky.get(_STICKY_DOC_SERIAL)
    if cur_serial is not None and prev_serial != cur_serial:
        # Doc changed (file reopen, new file, etc.). Stale bundles point at
        # GUIDs that no longer exist; orphan meshes from the old file's save
        # may sit on LAYER_IK_CACHE.
        sc.sticky.pop(_STICKY_BUNDLE_CACHE, None)
        sc.sticky.pop(_STICKY_CACHE_INITIALIZED, None)
        sc.sticky[_STICKY_DOC_SERIAL] = cur_serial
    if sc.sticky.get(_STICKY_CACHE_INITIALIZED):
        return
    # Purge every transient preview layer the IK commands bake onto.
    # `LAYER_IK_CACHE` holds robot/tool/RB meshes (recursive sub-layers per
    # layer_key + mesh-mode); the preview layers hold inserted block
    # instances (pineapples, Robotiq gripper) baked by rs_show_ik /
    # rs_ik_keyframe. All are purely transient previews -- safe to drop on
    # first IK touch in a freshly-opened doc.
    ensure_layer(config.LAYER_IK_CACHE)
    purge_targets = [
        config.LAYER_IK_CACHE,
        getattr(config, "SUPPORT_PREVIEW_LAYER", None),
        "IKPineapplePreview",
    ]
    total = 0
    for lname in purge_targets:
        if not lname:
            continue
        total += _delete_layer_objects(lname)
    if total:
        print(f"ik_viz: purged {total} orphan preview object(s) before initial bake.")
    sc.sticky[_STICKY_CACHE_INITIALIZED] = True


def _resolve_layer(layer_key: Optional[str]) -> str:
    """Map ``layer_key`` to absolute Rhino layer path: ``LAYER_IK_CACHE [/<layer_key>]``."""
    if layer_key:
        return config.LAYER_PATH_SEP.join((config.LAYER_IK_CACHE, layer_key))
    return config.LAYER_IK_CACHE


def _bundle_key(robot_cell, layer_key: Optional[str]):
    return (id(robot_cell), layer_key or "")


def _rb_so_guids(rb_so) -> list:
    """Return all native GUIDs (visual + collision) baked by ``rb_so``."""
    return list(rb_so._visual_mesh_native_geometry or []) + list(rb_so._collision_mesh_native_geometry or [])


def _delete_rb_so_geometry(rb_so) -> None:
    guids = _rb_so_guids(rb_so)
    if not guids:
        return
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        rs.DeleteObjects(guids)
    finally:
        sc.doc.Views.RedrawEnabled = was


def _force_initial_draw_rb(rb_so) -> None:
    """Ensure ``_visual_mesh_native_geometry`` / ``_collision_mesh_native_geometry``
    are populated. ``RigidBodyObject.draw()`` triggers ``_initial_draw`` on first call.
    """
    if not rb_so._visual_mesh_native_geometry and not rb_so._collision_mesh_native_geometry:
        rb_so.draw()


def _force_initial_draw_robot(robot_so, base_frame=None) -> None:
    """Trigger ``RobotModelObject._initial_draw`` (link meshes baked onto the layer).

    Pass an explicit ``base_frame`` (compas Frame) so the bake matches the
    initial cell state's base; otherwise the SO bakes at worldXY.
    """
    if not robot_so._links_visual_mesh_native_geometry and not robot_so._links_collision_mesh_native_geometry:
        # Calling .draw() with no args defers configuration; pass a dummy
        # zero config / identity base so initial geometry is created.
        robot_so.draw()


def _sync_rb_keyset(bundle: dict, robot_cell) -> None:
    """Diff cached RB SOs against ``robot_cell.rigid_body_models``.

    * Removes SOs whose RB id is no longer in the cell (deletes their
      baked Rhino guids).
    * Adds new SOs for RB ids that appeared.
    Robot + tools are left alone.
    """
    layer_name = bundle["layer"]
    rb_sos = bundle["rb_sos"]
    desired = set(robot_cell.rigid_body_models.keys())
    cached = set(rb_sos.keys())

    # Removed RBs.
    for stale in cached - desired:
        try:
            _delete_rb_so_geometry(rb_sos[stale])
        except Exception as exc:
            print(f"ik_viz: error deleting RB SO geometry for {stale!r}: {exc}")
        del rb_sos[stale]

    # New RBs.
    for new_id in desired - cached:
        rb = robot_cell.rigid_body_models[new_id]
        rb_sos[new_id] = _create_rb_so(rb, layer_name)
        _force_initial_draw_rb(rb_sos[new_id])

    bundle["rb_keyset"] = frozenset(desired)


def _get_or_create_bundle(robot_cell, layer_key: Optional[str]) -> dict:
    """Return per-(cell, layer_key) bundle of {robot_so, tool_sos, rb_sos, ...}.

    On first call: creates the SOs, ensures the parent layer exists, and
    triggers initial draws so the bundle is ready for ``update_state``.
    Subsequent calls return the cached bundle as-is (callers that need RB
    keyset sync should call :func:`_sync_rb_keyset` explicitly).
    """
    _flush_cache_layer_once()
    cache = _bundle_cache()
    key = _bundle_key(robot_cell, layer_key)
    bundle = cache.get(key)
    if bundle is not None:
        return bundle

    layer_name = _resolve_layer(layer_key)
    ensure_layer(layer_name)

    robot_so = _create_robot_so(robot_cell, layer_name)
    _force_initial_draw_robot(robot_so)

    tool_sos = {}
    for tool_id, tool in robot_cell.tool_models.items():
        tool_sos[tool_id] = _create_tool_so(tool, layer_name)
        _force_initial_draw_robot(tool_sos[tool_id])

    rb_sos = {}
    for rb_id, rb in robot_cell.rigid_body_models.items():
        rb_sos[rb_id] = _create_rb_so(rb, layer_name)
        _force_initial_draw_rb(rb_sos[rb_id])

    bundle = {
        "robot_so": robot_so,
        "tool_sos": tool_sos,
        "rb_sos": rb_sos,
        "rb_keyset": frozenset(rb_sos.keys()),
        "active_mesh_mode": get_mesh_mode(),
        "layer": layer_name,
        "layer_key": layer_key or "",
    }
    cache[key] = bundle
    return bundle


def _iter_mode_guids(bundle: dict, mesh_mode: str):
    """Yield every Rhino GUID the bundle has baked for ``mesh_mode``.

    Robot + tools store per-link dicts (``_links_<mode>_mesh_native_geometry``
    -> ``{link_name: [guid, ...]}``); RBs store flat lists
    (``_<mode>_mesh_native_geometry``).
    """
    is_visual = mesh_mode == MESH_MODE_VISUAL
    robot_so = bundle["robot_so"]
    link_dict = (
        robot_so._links_visual_mesh_native_geometry
        if is_visual
        else robot_so._links_collision_mesh_native_geometry
    )
    for guids in (link_dict or {}).values():
        for g in guids:
            yield g
    for tool_so in (bundle["tool_sos"] or {}).values():
        link_dict = (
            tool_so._links_visual_mesh_native_geometry
            if is_visual
            else tool_so._links_collision_mesh_native_geometry
        )
        for guids in (link_dict or {}).values():
            for g in guids:
                yield g
    for rb_so in (bundle["rb_sos"] or {}).values():
        guids = (
            rb_so._visual_mesh_native_geometry
            if is_visual
            else rb_so._collision_mesh_native_geometry
        )
        for g in (guids or []):
            yield g


def _set_visibility_for_mode(bundle: dict, mesh_mode: str) -> None:
    """Show ``mesh_mode`` GUIDs, hide the OTHER mode's GUIDs.

    Per-GUID visibility flip via ``rs.HideObjects`` / ``rs.ShowObjects``.
    """
    other = MESH_MODE_COLLISION if mesh_mode == MESH_MODE_VISUAL else MESH_MODE_VISUAL
    show = list(_iter_mode_guids(bundle, mesh_mode))
    hide = list(_iter_mode_guids(bundle, other))
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        if hide:
            rs.HideObjects(hide)
        if show:
            rs.ShowObjects(show)
    finally:
        sc.doc.Views.RedrawEnabled = was
    bundle["active_mesh_mode"] = mesh_mode


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
    # mesh_modes is now informational only (we always bake both); kept in the
    # signature so existing callers don't break.
    _ = _normalize_modes(mesh_modes, mesh_mode)
    if active_mesh_mode is None:
        active_mesh_mode = get_mesh_mode()
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

    # Build the bundle so all geometry (robot/tools/RBs, both modes) is baked
    # before the first update_state.
    bundle = _get_or_create_bundle(robot_cell, layer_key)
    _set_visibility_for_mode(bundle, active_mesh_mode)


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

    First call (per ``(robot_cell, layer_key)``) bakes the meshes (both
    modes); subsequent calls delta-transform them in place. RB keyset is
    diff'd against the cell on every call: new RBs are baked, removed RBs
    are deleted -- so bar/env switches don't need an explicit ``discard_cache``.

    ``mesh_mode`` / ``mesh_modes`` are accepted for back-compat; the bundle
    always holds both modes, so this only controls which one is currently
    visible.
    """
    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()

    bundle = _get_or_create_bundle(robot_cell, layer_key)
    _sync_rb_keyset(bundle, robot_cell)

    # Bug B: get_robot_link_meshes_at_zero() hides LAYER_IK_CACHE on exit so
    # the ghost preview is the only thing visible during base picking. After
    # IK solves, update_state must force the root visible -- otherwise only
    # the sub-layer toggle below shows up but the parent stays hidden so
    # nothing renders.
    if rs.IsLayer(config.LAYER_IK_CACHE) and not rs.LayerVisible(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, True)
    parent = _resolve_layer(layer_key)
    if parent != config.LAYER_IK_CACHE and rs.IsLayer(parent) and not rs.LayerVisible(parent):
        rs.LayerVisible(parent, True)

    # Compute attached-RB frames (mirrors BaseRobotCellObject.update).
    state = robot_cell.compute_attach_objects_frames(state)

    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        bundle["robot_so"].update(
            state.robot_configuration, state.robot_base_frame
        )
        for tool_id, tool_so in bundle["tool_sos"].items():
            ts = state.tool_states.get(tool_id)
            if ts is None:
                continue
            tool_so.update(ts.configuration, ts.frame)
        for rb_id, rb_so in bundle["rb_sos"].items():
            rbs = state.rigid_body_states.get(rb_id)
            if rbs is None:
                continue
            rb_so.update(rbs)
        # Re-apply visibility for the active mode (any newly-baked geometry
        # from sync_rb_keyset starts visible by default).
        active = mesh_mode if mesh_mode in _VALID_MESH_MODES else bundle["active_mesh_mode"]
        _set_visibility_for_mode(bundle, active)
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
    """Show ``mesh_mode``'s GUIDs, hide the other mode's GUIDs (no rebake).

    Operates on the bundle for ``layer_key`` only; other bundles are unaffected.
    """
    if mesh_mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh mode must be one of {_VALID_MESH_MODES}, got {mesh_mode!r}")
    set_mesh_mode(mesh_mode)
    cache = _bundle_cache()
    target_lkey = layer_key or ""
    for k, bundle in cache.items():
        if k[1] != target_lkey:
            continue
        _set_visibility_for_mode(bundle, mesh_mode)


def get_cached_bundle(robot_cell, layer_key: Optional[str]):
    """Return the cached bundle dict for ``(cell, layer_key)`` or ``None``.

    Bundle layout: ``{"robot_so", "tool_sos", "rb_sos", "rb_keyset",
    "active_mesh_mode", "layer", "layer_key"}``.
    """
    return _bundle_cache().get(_bundle_key(robot_cell, layer_key))


def get_cached_scene_object(robot_cell, layer_key: Optional[str], mesh_mode: str):
    """Back-compat shim: returns the cached robot SceneObject (mesh_mode ignored).

    Old callers that asked for a per-mode RobotCellObject now get a single
    :class:`RobotModelObject`; tools / RBs are accessed via :func:`get_cached_bundle`.
    """
    bundle = get_cached_bundle(robot_cell, layer_key)
    return bundle["robot_so"] if bundle else None


def get_link_native_geometry(robot_cell, layer_key: Optional[str], mesh_mode: str) -> dict:
    """Return ``{link_name: [rhino_guid, ...]}`` for the cached robot model.

    Used by collision-highlight code to recolor specific links.
    """
    bundle = get_cached_bundle(robot_cell, layer_key)
    if bundle is None:
        return {}
    rmo = bundle["robot_so"]
    if mesh_mode == MESH_MODE_VISUAL:
        return dict(rmo._links_visual_mesh_native_geometry or {})
    return dict(rmo._links_collision_mesh_native_geometry or {})


def get_tool_native_geometry(
    robot_cell, layer_key: Optional[str], mesh_mode: str
) -> dict:
    """Return ``{tool_name: [rhino_guid, ...]}`` flattened across tool links."""
    bundle = get_cached_bundle(robot_cell, layer_key)
    if bundle is None:
        return {}
    out = {}
    for tool_name, tool_so in (bundle["tool_sos"] or {}).items():
        guids = []
        meshes = (
            tool_so._links_visual_mesh_native_geometry
            if mesh_mode == MESH_MODE_VISUAL
            else tool_so._links_collision_mesh_native_geometry
        )
        for lst in (meshes or {}).values():
            guids.extend(lst)
        out[tool_name] = guids
    return out


def get_robot_link_meshes_at_zero(robot_cell=None, layer_key: Optional[str] = None):
    """Return a flat list of Rhino meshes for every robot link at zero config.

    Reads the cached :class:`RobotModelObject`'s baked GUIDs and duplicates them
    as standalone meshes (so the caller can feed them to a DisplayConduit
    without owning the cached doc objects). Triggers initial-bake if needed.

    Leaves the cache layer HIDDEN on exit so the caller's ghost preview is
    not competing with the baked robot at world origin. The next
    :func:`begin_session` / :func:`update_state` call will show it again.

    Replaces the legacy ``_bake_robot_meshes_at_zero`` hack which queried
    ``rs.ObjectsByLayer`` (broken when meshes are nested under sub-layers).
    """
    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()
    bundle = _get_or_create_bundle(robot_cell, layer_key)
    rmo = bundle["robot_so"]
    # Bug C: the cached SO holds meshes at whatever pose the LAST update_state
    # left them. For the ghost-preview-at-zero use case, we must re-pose the
    # robot at zero-config + worldXY base BEFORE harvesting, otherwise the
    # ghost appears at the previous pose and the click<->ghost offset is wrong.
    Frame = import_compas_stack()["Frame"]
    zero_state = default_cell_state()
    zero_state.robot_base_frame = Frame.worldXY()
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        rmo.update(zero_state.robot_configuration, zero_state.robot_base_frame)
    finally:
        sc.doc.Views.RedrawEnabled = was
    meshes = []
    link_dict = rmo._links_visual_mesh_native_geometry or {}
    for guids in link_dict.values():
        for g in guids:
            m = rs.coercemesh(g)
            if m is not None:
                meshes.append(m.DuplicateMesh())
    if rs.IsLayer(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, False)
    return meshes


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
    and drop the cached bundles. Use only on hard reset (PyBullet restart,
    explicit user-flush). Bar/env switches do NOT need this -- :func:`update_state`
    already syncs the RB keyset incrementally.
    """
    cache = sc.sticky.pop(_STICKY_BUNDLE_CACHE, None) or {}
    sc.sticky.pop(_STICKY_CACHE_INITIALIZED, None)
    seen_layers = set()
    for bundle in cache.values():
        layer_name = bundle.get("layer")
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
