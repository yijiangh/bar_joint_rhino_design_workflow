"""Rhino-side scene cache for showing the dual-arm robot during IK preview.

What this module owns
---------------------
For each ``(robot_cell, layer_key)`` pair it keeps one drawing object for the
robot, one per tool, and one per rigid body (bars, joints, environment). In
compas these drawing objects are called "scene objects", so throughout this file
a variable named ``*_so`` (``robot_so``, ``tool_sos``, ``rb_sos``) is one of
them.

Owning each piece on its own (instead of one big combined wrapper) means we can
throw away and rebuild just one piece without touching the others. For example,
when the set of rigid bodies changes as the user switches bars, the robot and
tool meshes are left alone.

How the meshes are drawn
------------------------
Every scene object is baked into the Rhino document ONCE, with both its display
mesh (``draw_visual``) and its collision mesh (``draw_collision``). Switching
between "show display mesh" and "show collision mesh" is then just hiding one set
of Rhino objects and showing the other -- nothing is baked again.

Session lifecycle
-----------------

* :func:`begin_session` builds the drawing objects for ``layer_key`` and bakes
  them up front (so the first :func:`update_state` is fast). It hides the
  user-modeled document layers that overlap the preview and restores them in
  :func:`end_session`.
* :func:`update_state` poses everything to match a :class:`RobotCellState`. It
  compares the current set of rigid bodies against the cached one and adds or
  removes drawing objects in place.
* :func:`set_active_mesh_mode` flips which mesh set is visible; nothing is baked.
* :func:`end_session` hides the cache layer but leaves the geometry in place, so
  the next session resumes by just moving the existing meshes.
* :func:`discard_cache` deletes everything; use it only on a hard reset.
"""

from __future__ import annotations

from typing import Iterable, Optional

import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino  # noqa: F401  (kept so other scripts can import Rhino through this file)

from core import config
from core.rhino_helpers import ensure_layer
from core.robot_cell import default_cell_state, get_or_load_robot_cell, import_compas_stack


# ---------------------------------------------------------------------------
# * Shared layer-key names
# ---------------------------------------------------------------------------
# Both rs_show_ik and rs_ik_keyframe use these names, so they land in the same
# cache slot and share one baked preview.

LAYER_KEY_ASSEMBLY = "Assembly"
LAYER_KEY_SUPPORT = "Support"


# ---------------------------------------------------------------------------
# * Sticky-dictionary keys
# ---------------------------------------------------------------------------
# Rhino's ``sc.sticky`` is a dictionary that survives between script runs in the
# same Rhino session, so we stash our cache there under these keys.

# The bundle cache maps (id(robot_cell), layer_key) -> a dict shaped like:
#     {
#         "robot_so":  drawing object for the robot,
#         "tool_sos":  {tool_id: drawing object},
#         "rb_sos":    {rigid_body_id: drawing object},
#         "rb_keyset": frozenset of the rigid-body ids currently cached,
#         "active_mesh_mode": "visual" or "collision",
#         "layer": Rhino layer path the meshes live on,
#     }
_STICKY_BUNDLE_CACHE = "bar_joint:ik_viz_bundle_cache"
_STICKY_CACHE_INITIALIZED = "bar_joint:ik_viz_cache_initialized"
_STICKY_HIDDEN_DOC_LAYERS = "bar_joint:ik_viz_hidden_doc_layers"
_STICKY_MESH_MODE = "bar_joint:ik_viz_mesh_mode"
# Which Rhino document this cache was built for. If the user opens a different
# .3dm file in the same Rhino session, the cached drawing objects point at Rhino
# objects that no longer exist, and leftover meshes from the previous file may
# still sit on LAYER_IK_CACHE -- both need to be cleared.
_STICKY_DOC_SERIAL = "bar_joint:ik_viz_doc_serial"

# Older keys still read by the yj_functions/ scripts; kept for those callers.
_STICKY_SCENE_OBJECT = "bar_joint:ik_viz_scene_object"
_STICKY_DRAWN_IDS = "bar_joint:ik_viz_drawn_ids"


# ---------------------------------------------------------------------------
# * Which mesh set is shown: display mesh vs. collision mesh
# ---------------------------------------------------------------------------

MESH_MODE_VISUAL = "visual"
MESH_MODE_COLLISION = "collision"
_VALID_MESH_MODES = (MESH_MODE_VISUAL, MESH_MODE_COLLISION)


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


# ---------------------------------------------------------------------------
# * Internals: unit scaling, layer clean-up, drawing-object lifecycle
# ---------------------------------------------------------------------------


def _native_scale_for_doc() -> float:
    """Return how many meters one Rhino document unit represents.

    compas geometry is stored in meters. The base robot-cell drawing object
    scales geometry by ``1 / native_scale`` to reach the document's unit, so
    ``native_scale`` has to be the "meters per document unit" ratio.

    Example: in a millimeter document one unit is 0.001 m, so this returns 0.001.

    Returns:
        float: meters per document unit.
    """
    from core.rhino_frame_io import doc_unit_scale_to_mm

    # `doc_unit_scale_to_mm()` gives "millimeters per document unit". Dividing by
    # 1000 turns that into "meters per document unit", which is what the base
    # robot-model drawing object expects.
    return doc_unit_scale_to_mm() / 1000.0


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
    """Load the compas_fab Rhino plugin so we get Rhino-flavored drawing objects.

    Importing ``compas_fab.rhino.scene`` registers the Rhino versions of
    ``RobotModelObject`` / ``RigidBodyObject``. Without it, compas hands back the
    cross-context base classes, whose draw methods raise ``NotImplementedError``.
    """
    try:
        import compas_fab.rhino.scene  # noqa: F401
    except Exception as exc:  # pragma: no cover -- defensive
        print(f"ik_viz: compas_fab.rhino.scene import failed ({exc}); "
              "Rhino SceneObjects will not be available.")


def _import_rhino_scene_classes():
    """Import the Rhino drawing-object classes, only when actually needed.

    Kept lazy so importing this module in a non-Rhino test does not trip over
    module load-order problems.

    Returns:
        tuple: ``(SceneObject, RobotModelObject, RigidBodyObject)``.
    """
    _ensure_compas_fab_rhino_registered()
    from compas.scene import SceneObject
    from compas_fab.rhino.scene import RobotModelObject, RigidBodyObject
    return SceneObject, RobotModelObject, RigidBodyObject


def _create_robot_so(robot_cell, layer_name: str):
    """Build the drawing object for the cell's robot model.

    Both the display mesh and the collision mesh are baked at once; switching
    between them later is just hiding one set of Rhino objects and showing the
    other.

    Args:
        robot_cell: the cell whose ``robot_model`` is drawn.
        layer_name (str): Rhino layer path to bake onto.

    Returns:
        The robot drawing object.
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
    """Build the drawing object for one tool model (display + collision meshes)."""
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
    """Build the drawing object for one rigid body (display + collision meshes)."""
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
    """Return the sticky bundle cache, creating an empty one on first use."""
    cache = sc.sticky.get(_STICKY_BUNDLE_CACHE)
    if cache is None:
        cache = {}
        sc.sticky[_STICKY_BUNDLE_CACHE] = cache
    return cache


def _flush_cache_layer_once() -> None:
    """Once per opened .3dm file, delete leftover preview meshes from before.

    A previous session (or a previous file saved in this same Rhino session) can
    leave stray meshes on ``LAYER_IK_CACHE``. This clears them the first time the
    cache is touched for a document.

    We notice a new document by watching ``sc.doc.RuntimeSerialNumber``: when it
    changes, the cached drawing objects point at Rhino objects that are gone, so
    we drop the bundle cache and the "already cleaned" flag and start fresh.
    """
    try:
        cur_serial = int(sc.doc.RuntimeSerialNumber)
    except Exception:
        cur_serial = None
    prev_serial = sc.sticky.get(_STICKY_DOC_SERIAL)
    if cur_serial is not None and prev_serial != cur_serial:
        # A different document is open now (file reopened, new file, etc.). The
        # cached drawing objects point at Rhino objects that are gone, and stray
        # meshes from the old file may still sit on LAYER_IK_CACHE.
        sc.sticky.pop(_STICKY_BUNDLE_CACHE, None)
        sc.sticky.pop(_STICKY_CACHE_INITIALIZED, None)
        sc.sticky[_STICKY_DOC_SERIAL] = cur_serial
    if sc.sticky.get(_STICKY_CACHE_INITIALIZED):
        return
    # Clear every temporary preview layer the IK commands draw onto. LAYER_IK_CACHE
    # holds the robot / tool / rigid-body meshes (on per-layer_key, per-mesh-mode
    # sub-layers); the preview layers hold inserted block copies (tool previews,
    # the Robotiq gripper) placed by rs_show_ik / rs_ik_keyframe. All of these are
    # throwaway previews, so it is safe to delete them on the first IK action in a
    # freshly-opened document. "IKPineapplePreview" is an old layer name we still
    # clear so older documents tidy themselves up on first run.
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
    """Turn a short ``layer_key`` into a full Rhino layer path.

    Returns ``LAYER_IK_CACHE`` on its own when ``layer_key`` is empty, otherwise
    ``LAYER_IK_CACHE::<layer_key>``.
    """
    if layer_key:
        return config.LAYER_PATH_SEP.join((config.LAYER_IK_CACHE, layer_key))
    return config.LAYER_IK_CACHE


def _bundle_key(robot_cell, layer_key: Optional[str]):
    """Build the cache key for a ``(robot_cell, layer_key)`` pair."""
    return (id(robot_cell), layer_key or "")


def _rb_so_guids(rb_so) -> list:
    """Return every Rhino object id (display + collision) a rigid body drew."""
    return list(rb_so._visual_mesh_native_geometry or []) + list(rb_so._collision_mesh_native_geometry or [])


def _delete_rb_so_geometry(rb_so) -> None:
    """Delete all Rhino objects a rigid body's drawing object baked."""
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
    """Make sure a rigid body has actually baked its meshes.

    A rigid body's ``draw()`` performs the first bake and fills in its display and
    collision geometry lists; this triggers it only if those lists are still empty.
    """
    if not rb_so._visual_mesh_native_geometry and not rb_so._collision_mesh_native_geometry:
        rb_so.draw()


def _force_initial_draw_robot(robot_so, base_frame=None) -> None:
    """Make sure a robot (or tool) has baked its link meshes onto the layer.

    The first ``draw()`` call is what actually creates the link geometry.

    Args:
        robot_so: the robot or tool drawing object to bake.
        base_frame: optional compas Frame to bake at; when omitted the meshes are
            baked at the world origin.
    """
    if not robot_so._links_visual_mesh_native_geometry and not robot_so._links_collision_mesh_native_geometry:
        # Calling .draw() with no configuration bakes the link geometry at the
        # zero configuration / world-origin base, which is all we need here.
        robot_so.draw()


def _sync_rb_keyset(bundle: dict, robot_cell) -> None:
    """Make the cached rigid-body drawing objects match the cell's rigid bodies.

    * Any cached rigid body the cell no longer has is removed (its baked Rhino
      objects are deleted).
    * Any rigid body the cell has but the cache does not is added and baked.

    The robot and the tools are left untouched.

    Args:
        bundle (dict): the cache bundle for one ``(cell, layer_key)`` pair.
        robot_cell: the cell whose ``rigid_body_models`` is the source of truth.
    """
    layer_name = bundle["layer"]
    rb_sos = bundle["rb_sos"]
    desired = set(robot_cell.rigid_body_models.keys())
    cached = set(rb_sos.keys())

    # Rigid bodies the cell dropped: delete their meshes and forget them.
    for stale in cached - desired:
        try:
            _delete_rb_so_geometry(rb_sos[stale])
        except Exception as exc:
            print(f"ik_viz: error deleting RB SO geometry for {stale!r}: {exc}")
        del rb_sos[stale]

    # Rigid bodies the cell added: build and bake a drawing object for each.
    for new_id in desired - cached:
        rb = robot_cell.rigid_body_models[new_id]
        rb_sos[new_id] = _create_rb_so(rb, layer_name)
        _force_initial_draw_rb(rb_sos[new_id])

    bundle["rb_keyset"] = frozenset(desired)


def _get_or_create_bundle(robot_cell, layer_key: Optional[str]) -> dict:
    """Return the cache bundle for ``(cell, layer_key)``, building it if needed.

    On the first call it creates the drawing objects, makes sure the parent layer
    exists, and bakes the initial geometry so the bundle is ready for
    :func:`update_state`. Later calls return the cached bundle as-is (callers that
    need the rigid-body set refreshed should call :func:`_sync_rb_keyset`).

    Returns:
        dict: the bundle, shaped like ``{robot_so, tool_sos, rb_sos, ...}``.
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
        # Ids of the rigid bodies the current step keeps hidden (bars/joints that
        # are not built yet). Refreshed by update_state and read by
        # _set_visibility_for_mode. Empty until the first state is applied.
        "hidden_rb_ids": set(),
        "layer": layer_name,
        "layer_key": layer_key or "",
    }
    cache[key] = bundle
    return bundle


def _iter_robot_tool_mode_guids(bundle: dict, mesh_mode: str):
    """Yield every robot and tool Rhino object id baked for one mesh mode.

    The robot and tools keep their meshes in per-link dictionaries
    (``_links_<mode>_mesh_native_geometry`` maps ``link_name -> [id, ...]``).
    Rigid bodies are handled separately (see :func:`_rb_mode_guids`) so each
    body's per-step hidden/shown state can be respected -- the robot and tools are
    never hidden step by step.

    Args:
        bundle (dict): the cache bundle.
        mesh_mode (str): ``"visual"`` or ``"collision"``.

    Yields:
        Rhino object ids for the robot and tool links.
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


def _rb_mode_guids(rb_so, mesh_mode: str) -> list:
    """Return one rigid body's baked Rhino object ids for a mesh mode (flat list)."""
    if mesh_mode == MESH_MODE_VISUAL:
        return list(rb_so._visual_mesh_native_geometry or [])
    return list(rb_so._collision_mesh_native_geometry or [])


def _set_visibility_for_mode(bundle: dict, mesh_mode: str) -> None:
    """Show one mesh mode and hide the other, keeping not-yet-built bars hidden.

    The current step marks every rigid body whose parent bar comes later in the
    assembly order than the bar being built with ``is_hidden=True`` (done in
    ``ik_collision_setup.build_full_assembly_state``). :func:`update_state` copies
    those ids onto the bundle as ``hidden_rb_ids``, and here we keep their meshes
    hidden in BOTH mesh modes. Without this, flipping the mesh mode would re-show
    every rigid body, so the preview would draw all bars at their final assembled
    pose instead of only the ones that exist at this step.

    Visibility is flipped per Rhino object with ``rs.HideObjects`` / ``rs.ShowObjects``.

    Args:
        bundle (dict): the cache bundle.
        mesh_mode (str): the mesh mode to show; the other one is hidden.
    """
    other = MESH_MODE_COLLISION if mesh_mode == MESH_MODE_VISUAL else MESH_MODE_VISUAL
    hidden_rb_ids = bundle.get("hidden_rb_ids") or set()

    # Robot + tools: show the active mode, hide the other. They are never hidden
    # step by step.
    show = list(_iter_robot_tool_mode_guids(bundle, mesh_mode))
    hide = list(_iter_robot_tool_mode_guids(bundle, other))

    # Rigid bodies: a not-yet-built bar stays hidden in both modes; a bar that
    # exists at this step follows the mesh-mode toggle like everything else.
    for rb_id, rb_so in (bundle["rb_sos"] or {}).items():
        hide.extend(_rb_mode_guids(rb_so, other))
        active_guids = _rb_mode_guids(rb_so, mesh_mode)
        if rb_id in hidden_rb_ids:
            hide.extend(active_guids)
        else:
            show.extend(active_guids)

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
# * Public session API
# ---------------------------------------------------------------------------


def _normalize_modes(mesh_modes, mesh_mode) -> tuple:
    """Turn the various mesh-mode arguments into a validated tuple of modes."""
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
    """Start an interactive IK preview session.

    * Bakes the robot, tools, and rigid bodies (both mesh modes at once) so that
      toggling display <-> collision later is only a visibility flip.
    * Leaves only ``active_mesh_mode`` visible.
    * Hides every layer in ``hide_doc_layers`` for the length of the session and
      restores each one's previous visibility in :func:`end_session`. The default
      is the user-modeled document layers that overlap the preview (tube previews,
      female/male joint copies, robotic-tool block copies) so the cached
      robot/tool/environment meshes are the only thing on screen. Pass an empty
      list to keep everything visible.

    Args:
        robot_cell: the cell to preview; ``None`` loads the default dual-arm cell.
        mesh_mode (str | None): older single-mode argument; still accepted.
        hide_tool_instances (bool): older flag; when False the tool-copy layer is
            left visible. Prefer ``hide_doc_layers``.
        layer_key (str | None): which sub-layer bundle to use.
        mesh_modes (Iterable[str] | None): older argument; informational only now.
        active_mesh_mode (str | None): which mesh mode to show first.
        hide_doc_layers (Iterable[str] | None): document layers to hide for the
            session; ``None`` uses the default overlapping layers.
    """
    # We always bake both mesh modes now, so mesh_modes is only informational; it
    # stays in the signature so existing callers keep working.
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
        # Default: the user-modeled document layers that overlap the preview
        # (tubes / joints / tool blocks). Bar centerlines stay visible so the
        # user can still pick them.
        hide_doc_layers = (
            config.LAYER_BAR_TUBE_PREVIEWS,
            config.LAYER_JOINT_FEMALE_INSTANCES,
            config.LAYER_JOINT_MALE_INSTANCES,
            config.LAYER_JOINT_GROUND_INSTANCES,
            config.LAYER_TOOL_INSTANCES,
        )
        if not hide_tool_instances:
            hide_doc_layers = tuple(
                ln for ln in hide_doc_layers if ln != config.LAYER_TOOL_INSTANCES
            )
    _hide_doc_layers(hide_doc_layers)

    # Build the bundle so all geometry (robot, tools, rigid bodies, both mesh
    # modes) is baked before the first update_state.
    bundle = _get_or_create_bundle(robot_cell, layer_key)
    _set_visibility_for_mode(bundle, active_mesh_mode)


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


def update_state(
    state,
    *,
    robot_cell=None,
    mesh_mode: Optional[str] = None,
    layer_key: Optional[str] = None,
    mesh_modes: Optional[Iterable[str]] = None,
) -> None:
    """Pose the cached preview to match ``state``.

    The first call (per ``(robot_cell, layer_key)``) bakes the meshes (both mesh
    modes); later calls just move the existing meshes into place. On every call the
    cached set of rigid bodies is compared against the cell: new bodies are baked
    and removed ones are deleted, so switching bars or environments does not need a
    manual :func:`discard_cache`.

    ``mesh_mode`` / ``mesh_modes`` are still accepted but only decide which mesh
    mode is currently visible; the bundle always holds both.

    Args:
        state (RobotCellState): the pose to show.
        robot_cell: the cell to pose; ``None`` loads the default dual-arm cell.
        mesh_mode (str | None): which mesh mode to leave visible.
        layer_key (str | None): which sub-layer bundle to use.
        mesh_modes (Iterable[str] | None): older argument; accepted but unused.
    """
    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()

    bundle = _get_or_create_bundle(robot_cell, layer_key)
    _sync_rb_keyset(bundle, robot_cell)

    # get_robot_link_meshes_at_zero() hides LAYER_IK_CACHE on the way out so that
    # only the ghost preview shows while the user picks a base. After IK solves,
    # update_state has to force the root layer visible again -- otherwise the
    # sub-layer toggle below turns geometry on, but the hidden parent layer keeps
    # everything off screen.
    if rs.IsLayer(config.LAYER_IK_CACHE) and not rs.LayerVisible(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, True)
    parent = _resolve_layer(layer_key)
    if parent != config.LAYER_IK_CACHE and rs.IsLayer(parent) and not rs.LayerVisible(parent):
        rs.LayerVisible(parent, True)

    # Work out where each attached rigid body sits (same as the base cell does).
    state = robot_cell.compute_attach_objects_frames(state)

    # Record which rigid bodies this step keeps hidden: the not-yet-built
    # bars/joints whose parent bar is later in the assembly order than the bar
    # being built (marked is_hidden=True by build_full_assembly_state). The
    # visibility pass below keeps them hidden so the preview shows only the bars
    # that actually exist at this step.
    bundle["hidden_rb_ids"] = {
        rb_id
        for rb_id, rbs in state.rigid_body_states.items()
        if getattr(rbs, "is_hidden", False)
    }

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
        # Re-apply the active mesh mode's visibility (anything newly baked by
        # _sync_rb_keyset starts out visible by default).
        active = mesh_mode if mesh_mode in _VALID_MESH_MODES else bundle["active_mesh_mode"]
        _set_visibility_for_mode(bundle, active)
    finally:
        sc.doc.Views.RedrawEnabled = was
    sc.doc.Views.Redraw()


def set_layer_visible(layer_key: Optional[str], visible: bool) -> None:
    """Show or hide a single sub-layer of the IK cache.

    ``layer_key=None`` toggles the root cache layer, which cascades to all its
    sub-layers. Use this to hide one cell's preview (for example the support arm
    when the active bar carries no support load) while leaving others visible.

    Args:
        layer_key (str | None): the sub-layer to toggle; ``None`` for the root.
        visible (bool): show it when True, hide it when False.
    """
    layer_name = _resolve_layer(layer_key)
    if rs.IsLayer(layer_name):
        rs.LayerVisible(layer_name, bool(visible))


def set_active_mesh_mode(layer_key: Optional[str], mesh_mode: str) -> None:
    """Show one mesh mode's objects and hide the other's, without re-baking.

    Only the bundle for ``layer_key`` is affected; other bundles are left alone.

    Args:
        layer_key (str | None): which bundle to switch.
        mesh_mode (str): ``"visual"`` or ``"collision"``.
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
    """Return the cache bundle for ``(cell, layer_key)``, or ``None`` if absent.

    The bundle is shaped like ``{"robot_so", "tool_sos", "rb_sos", "rb_keyset",
    "active_mesh_mode", "layer", "layer_key"}``.
    """
    return _bundle_cache().get(_bundle_key(robot_cell, layer_key))


def get_cached_scene_object(robot_cell, layer_key: Optional[str], mesh_mode: str):
    """Return the cached robot drawing object (``mesh_mode`` is ignored).

    Older callers asked for a single per-mode combined object; they now get the
    one robot drawing object. Tools and rigid bodies are reached through
    :func:`get_cached_bundle`.
    """
    bundle = get_cached_bundle(robot_cell, layer_key)
    return bundle["robot_so"] if bundle else None


def get_link_native_geometry(robot_cell, layer_key: Optional[str], mesh_mode: str) -> dict:
    """Return ``{link_name: [rhino_object_id, ...]}`` for the cached robot model.

    Used by the collision-highlight code to recolor specific links.
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
    """Return ``{tool_name: [rhino_object_id, ...]}``, flattened across tool links."""
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


def get_robot_link_meshes_at_state(state, robot_cell=None, layer_key: Optional[str] = None):
    """Return a flat list of Rhino meshes for every robot link posed at ``state``.

    Poses the cached robot drawing object at ``state``'s configuration and base
    frame, then makes a standalone copy of each display link mesh (so the caller
    can hand them to a see-through preview -- see ``core.dynamic_preview.mesh_preview``
    -- without owning the cached document objects). Bakes the geometry first if it
    has not been baked yet.

    On exit the cache layer is left HIDDEN, so the caller's ghost preview is the
    only robot on screen; the next :func:`begin_session` / :func:`update_state`
    call shows the baked copy again.

    Args:
        state (RobotCellState): the pose to harvest; its ``robot_configuration``
            and ``robot_base_frame`` are pushed onto the robot before copying.
        robot_cell (RobotCell | None): the cell to bake from; ``None`` uses the
            cached dual-arm cell.
        layer_key (str | None): which sub-layer bundle to use.

    Returns:
        list: copied ``Rhino.Geometry.Mesh`` objects at the posed positions.
    """
    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()
    bundle = _get_or_create_bundle(robot_cell, layer_key)
    rmo = bundle["robot_so"]
    # The cached drawing object still holds meshes at whatever pose the LAST
    # update_state left them, so re-pose to `state` before copying (otherwise the
    # ghost would show a stale pose).
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        rmo.update(state.robot_configuration, state.robot_base_frame)
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


def get_robot_link_meshes_at_zero(robot_cell=None, layer_key: Optional[str] = None):
    """Return a flat list of Rhino meshes for every robot link at the zero pose.

    A thin wrapper over :func:`get_robot_link_meshes_at_state` that poses the robot
    at the zero configuration and a world-origin base -- the fixed pose the IK
    base-frame sampling ghost uses (it then slides the candidate base on top as a
    rigid transform). See that function for the copy and clean-up details.

    Replaces the old ``_bake_robot_meshes_at_zero`` approach, which asked
    ``rs.ObjectsByLayer`` for the meshes and broke once they were nested under
    sub-layers.
    """
    if robot_cell is None:
        robot_cell = get_or_load_robot_cell()
    Frame = import_compas_stack()["Frame"]
    zero_state = default_cell_state()
    zero_state.robot_base_frame = Frame.worldXY()
    return get_robot_link_meshes_at_state(zero_state, robot_cell, layer_key)


def end_session() -> None:
    """End the preview session: hide the cache layer and restore the doc layers.

    The geometry on the cache layer is deliberately NOT deleted; the next session
    picks it up through the cached drawing objects and keeps moving it from its
    current pose.
    """
    if rs.IsLayer(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, False)

    _restore_hidden_doc_layers()


def discard_cache() -> None:
    """Hard reset: delete every object on the cache layer (and its sub-layers) and
    drop the cached bundles.

    Use this only for a hard reset (PyBullet restart, or an explicit user flush).
    Switching bars or environments does NOT need it -- :func:`update_state` already
    keeps the rigid-body set in sync as it goes.
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
# * Older single-shot API kept for existing callers
# ---------------------------------------------------------------------------


def show_state(state, mesh_mode: Optional[str] = None, *, robot_model=None) -> None:
    """Older one-shot draw entry point, now routed through :func:`update_state`.

    The ``robot_model`` argument is honored by re-pointing the cache at the cell
    whose ``robot_model`` matches it; callers that mix the dual-arm and support
    previews already swap the underlying cell in PyBullet via ``set_cell_state``,
    so the right cell is looked up from sticky when needed.

    On entry the cache layer is forced visible (so older callers that skip
    :func:`begin_session` still see the preview); it is left visible for the
    caller's own clean-up.

    Args:
        state: the pose to draw.
        mesh_mode (str | None): which mesh mode to show.
        robot_model: optional robot model used to pick the matching cell.
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
    """Best-effort lookup of the cell whose ``robot_model`` is ``robot_model``.

    Used only by the older :func:`show_state` path so callers that pass a bare
    ``robot_model`` still bind the cache to a real cell.
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
    """Draw the robot at its default cell state (zero configuration, identity base)."""
    show_state(default_cell_state())


def clear_scene() -> None:
    """Older clear: discard the cache and delete every baked mesh.

    Newer callers should prefer :func:`end_session`, which hides without deleting.
    """
    discard_cache()
