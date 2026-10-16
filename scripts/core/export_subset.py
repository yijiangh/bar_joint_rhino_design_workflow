"""Cut an export down to a range of bars (``from .. until``).

A partial export keeps the scene of the bars built up to ``until`` and drops
everything that only exists later:

- **actions** are written for the bars ``from .. until`` only (the exporter
  decides that; see :func:`range_bar_ids`);
- **bodies**: a bar whose step is after ``until`` is dropped, and so is every
  joint half mounted on it. Joint halves follow their OWN bar (``parent_bar_id``),
  so a female half on an early bar stays even when its male is on a dropped bar;
- **floors**: a floor slab stays only when some exported action may stand on
  its walkable ground (the bundle checker's rule D6);
- **sequence**: every action's ``assembly_seq`` is cut to the bars up to
  ``until`` (the planner reads ``assembly_seq[:index(active bar)]`` as the
  built set, so the bars before ``from`` must stay in it).

The actions are built exactly as for a full export (every state carries every
body of the cell, the later ones hidden), then trimmed here. The SAME set of
dropped names is removed from the cells and from every state, because
compas_fab requires a state to name exactly the cell's bodies
(``RobotCell.assert_cell_state_match``).

Rhino-free on purpose (plain dicts and duck-typed cells / states), so it runs
under pytest: ``tests/test_export_subset.py``.
"""

from __future__ import annotations

from core.env_collision import (
    CANONICAL_BAR_PREFIX,
    CANONICAL_JOINT_PREFIX,
    FLOOR_PREFIX,
    KIND_BAR,
    KIND_FLOOR,
    KIND_JOINT,
)


#: Name prefixes of the bodies a range cut may drop: bars, joint halves and
#: floor slabs. Environment obstacles and anything else always stay.
TRIMMABLE_PREFIXES = (CANONICAL_BAR_PREFIX, CANONICAL_JOINT_PREFIX, FLOOR_PREFIX)


# * ---------------------------------------------------------------- sequence


def ordered_bar_ids(bar_map: dict) -> list:
    """Bar ids in assembly order.

    Args:
        bar_map (dict): ``{bar_id: (oid, seq)}`` (a ``get_real_bar_seq_map`` result).

    Returns:
        list: the bar ids sorted by their step number.
    """
    return [bar_id for bar_id, _oid_seq in sorted(bar_map.items(), key=lambda kv: kv[1][1])]


def _bar_step(bar_map: dict, bar_id: str, role: str) -> int:
    """The step number of one bar, with a clear error when it is unknown.

    Args:
        bar_map (dict): ``{bar_id: (oid, seq)}``.
        bar_id (str): the bar to look up.
        role (str): what the bar is used for ("from" / "until"), for the message.

    Returns:
        int: the bar's step number.

    Raises:
        RuntimeError: the bar is not in the map (deleted, or a fake bar).
    """
    if bar_id not in bar_map:
        raise RuntimeError(
            f"The export range's {role} bar {bar_id!r} is not a real registered bar "
            "(deleted, renamed, or marked fake). Pick the range again."
        )
    return int(bar_map[bar_id][1])


def cut_assembly_seq(bar_map: dict, until_bar_id: str) -> list:
    """The assembly sequence up to and including ``until_bar_id``.

    Args:
        bar_map (dict): ``{bar_id: (oid, seq)}`` (real bars only).
        until_bar_id (str): the last bar of the export.

    Returns:
        list: bar ids in assembly order, ending with ``until_bar_id``.
    """
    until_step = _bar_step(bar_map, until_bar_id, "until")
    return [bar_id for bar_id in ordered_bar_ids(bar_map) if int(bar_map[bar_id][1]) <= until_step]


def range_bar_ids(bar_map: dict, from_bar_id: str, until_bar_id: str) -> list:
    """The bars whose actions a range export writes: ``from .. until``, inclusive.

    Args:
        bar_map (dict): ``{bar_id: (oid, seq)}`` (real bars only).
        from_bar_id (str): the first bar to export.
        until_bar_id (str): the last bar to export.

    Returns:
        list: bar ids in assembly order.

    Raises:
        RuntimeError: unknown bar, or ``from`` comes after ``until``.
    """
    from_step = _bar_step(bar_map, from_bar_id, "from")
    until_step = _bar_step(bar_map, until_bar_id, "until")
    if from_step > until_step:
        raise RuntimeError(
            f"The export range starts at {from_bar_id} (step {from_step}) but ends "
            f"at {until_bar_id} (step {until_step}); the start must not come after the end."
        )
    return [
        bar_id for bar_id in ordered_bar_ids(bar_map)
        if from_step <= int(bar_map[bar_id][1]) <= until_step
    ]


# * ---------------------------------------------------------------- which bodies go


def dropped_body_names(
    collision_bodies: dict,
    bar_map: dict,
    until_bar_id: str,
    used_ground_ids: set,
) -> set:
    """The bodies a range export leaves out of its cells and states.

    Args:
        collision_bodies (dict): every body of the assembly cell,
            ``{name: body_info}`` (``robot_cell.ensure_assembly_cell``); bars and
            joint halves carry ``kind`` + ``parent_bar_id``, floors carry
            ``kind`` + ``ground_id``.
        bar_map (dict): ``{bar_id: (oid, seq)}`` (real bars only).
        until_bar_id (str): the last bar of the export.
        used_ground_ids (set): the walkable grounds some exported action may
            stand on (:func:`used_ground_ids`).

    Returns:
        set: names of the bars / joint halves mounted on a bar after
        ``until_bar_id``, plus the floors of unused grounds.

    Raises:
        RuntimeError: a bar or joint half whose bar is not in ``bar_map`` (the
            cell is out of date: run RSRebuildRobotCell).
    """
    until_step = _bar_step(bar_map, until_bar_id, "until")
    dropped = set()
    unknown = []
    for name, body_info in collision_bodies.items():
        kind = body_info.get("kind")
        if kind in (KIND_BAR, KIND_JOINT):
            parent = body_info.get("parent_bar_id")
            if parent not in bar_map:
                unknown.append(f"{name} (bar {parent!r})")
                continue
            # Same ordering test as env_collision.collect_built_geometry: a
            # body exists once its own bar is built.
            if int(bar_map[parent][1]) > until_step:
                dropped.add(name)
        elif kind == KIND_FLOOR:
            if body_info.get("ground_id") not in used_ground_ids:
                dropped.add(name)
    if unknown:
        raise RuntimeError(
            "The robot cell has bodies on bars that are not real registered bars: "
            + ", ".join(sorted(unknown)[:8])
            + (" ..." if len(unknown) > 8 else "")
            + ". The cell is out of date: run RSRebuildRobotCell, then export again."
        )
    return dropped


def used_ground_ids(actions) -> set:
    """The walkable grounds the given actions may stand on.

    Args:
        actions (iterable): exported actions (each has ``walkable_ground_ids``).

    Returns:
        set: the union of their ground ids (the bundle checker's "used" grounds).
    """
    used = set()
    for action in actions:
        used.update(getattr(action, "walkable_ground_ids", None) or [])
    return used


def bodies_outside_cell(actions, cell_body_names) -> set:
    """Bars / joint halves / floors the actions name but a bundle's cell lacks.

    Used when ONE bar is re-exported into an existing (possibly partial)
    bundle: its fresh states carry every body of the live cell, and the ones
    the bundle's ``RobotCell.json`` does not have must be dropped again.
    Only trimmable names are returned (see :data:`TRIMMABLE_PREFIXES`), so a
    body the bundle never cut (an environment obstacle) is never removed here.

    Args:
        actions (iterable): freshly built actions.
        cell_body_names (iterable): the rigid-body names of the bundle's cell.

    Returns:
        set: names to drop from the actions' states.
    """
    in_cell = set(cell_body_names)
    outside = set()
    for action in actions:
        for movement in action.movements:
            state = movement.start_state
            if state is None:
                continue
            for name in state.rigid_body_states:
                if name.startswith(TRIMMABLE_PREFIXES) and name not in in_cell:
                    outside.add(name)
    return outside


# * ---------------------------------------------------------------- trimming


def trim_robot_cell(robot_cell, dropped: set):
    """A copy of a robot cell without the dropped bodies.

    The cached cell in the Rhino session is never changed. The robot model,
    semantics, tool models and kept rigid bodies are shared by reference
    (they are large and the export only reads them); only the two
    dictionaries are new.

    Args:
        robot_cell (RobotCell): the cell to copy.
        dropped (set): rigid-body names to leave out.

    Returns:
        RobotCell: the trimmed copy (same class as ``robot_cell``).
    """
    return type(robot_cell)(
        robot_model=robot_cell.robot_model,
        robot_semantics=robot_cell.robot_semantics,
        tool_models=dict(robot_cell.tool_models),
        rigid_body_models={
            name: body for name, body in robot_cell.rigid_body_models.items()
            if name not in dropped
        },
    )


def trim_action_states(action, dropped: set, assembly_seq: list | None = None) -> int:
    """Remove the dropped bodies from every movement state of one action, in place.

    Also removes the dropped names from every kept body's allowed contacts
    (``touch_bodies``), so no state allows a contact with a body that is not
    in the scene. When ``assembly_seq`` is given, the action's sequence is
    replaced by it.

    Args:
        action: an exported action (``.movements``, each with a ``start_state``).
        dropped (set): rigid-body names to remove.
        assembly_seq (list | None): the cut sequence to write on the action.

    Returns:
        int: how many body states were removed (summed over the movements).
    """
    n_removed = 0
    for movement in action.movements:
        state = movement.start_state
        if state is None:
            continue
        for name in [n for n in state.rigid_body_states if n in dropped]:
            del state.rigid_body_states[name]
            n_removed += 1
        for body_state in state.rigid_body_states.values():
            touch = body_state.touch_bodies or []
            if any(other in dropped for other in touch):
                body_state.touch_bodies = [other for other in touch if other not in dropped]
    if assembly_seq is not None:
        action.assembly_seq = list(assembly_seq)
    return n_removed


def assert_states_match_cell(robot_cell, actions, cell_label: str) -> None:
    """Check that every movement state names exactly the cell's tools and bodies.

    Args:
        robot_cell (RobotCell): the (trimmed) cell the actions will ship with.
        actions (iterable): the actions to check.
        cell_label (str): the cell's file name, for the error message.

    Raises:
        RuntimeError: the first movement whose state does not match, with the
            mismatching names.
    """
    for action in actions:
        for movement in action.movements:
            state = movement.start_state
            if state is None:
                continue
            try:
                robot_cell.assert_cell_state_match(state)
            except ValueError as exc:
                raise RuntimeError(
                    f"{movement.movement_id} ({action.action_id}) does not match "
                    f"{cell_label}: {exc}"
                ) from exc


def assert_states_name_bodies(actions, body_names, cell_label: str) -> None:
    """Check that every movement state names exactly the given rigid bodies.

    The single-bar re-export has no cell object at hand, only the body names
    the bundle's schedule recorded for ``RobotCell.json``; this is the same
    body check as :func:`assert_states_match_cell` on those names.

    Args:
        actions (iterable): the actions to check.
        body_names (iterable): the rigid-body names of the bundle's cell.
        cell_label (str): the cell's file name, for the error message.

    Raises:
        RuntimeError: the first movement whose state names other bodies.
    """
    expected = set(body_names)
    for action in actions:
        for movement in action.movements:
            state = movement.start_state
            if state is None:
                continue
            names = set(state.rigid_body_states)
            if names != expected:
                raise RuntimeError(
                    f"{movement.movement_id} ({action.action_id}) does not match "
                    f"{cell_label}: not in the cell {sorted(names - expected)[:6]}, "
                    f"missing {sorted(expected - names)[:6]}"
                )
