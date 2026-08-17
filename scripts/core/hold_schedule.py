"""Derive which support robot holds which bar, and for how long.

Rhino-free on purpose (plain dicts in, plain dicts out) so it runs under
pytest and in any headless consumer. The Rhino-side reader that feeds it is
``core.rhino_bar_registry.collect_hold_inputs()``.

Semantics (matching ``rhino_bar_registry.get_unstable_bars``):

- A bar with a non-empty ``supported_until`` list must be HELD by a support
  robot from its own assembly step until every listed stabilizing bar that
  comes LATER in the sequence is built. Listed bars that come earlier are
  already built when the bar is placed and contribute nothing; if ALL listed
  bars come earlier, the bar never needs a hold.
- The hold releases right after the LAST stabilizing bar's assembly release
  — that bar is the hold's ``release_after_bar_id``.
- A support robot is busy from its hold's start step through the release
  step INCLUSIVE (the release action runs after that step's assembly
  actions), so it cannot take a new hold at the release step itself.
- When more than one robot is free, the alphabetically-first one is picked
  (the ``support_robot_names`` tuple is priority-ordered).

Occupancy is DERIVED, never stored: this module recomputes the whole plan
from the sequence data on every call, so sequence edits can never leave a
stale "occupied" flag anywhere.
"""

from __future__ import annotations


def derive_hold_plan(
    bar_seq: dict,
    supported_until: dict,
    support_robot_names: tuple = ("Alice", "Belle"),
) -> dict:
    """Assign a support robot + hold interval to every bar that needs holding.

    Args:
        bar_seq (dict): ``{bar_id: assembly step int}`` for every bar.
        supported_until (dict): ``{bar_id: [stabilizing bar ids]}``; bars
            missing here (or with an empty list) never need holding.
        support_robot_names (tuple): priority-ordered robot names; callers in
            Rhino pass ``config.SUPPORT_ROBOT_NAMES``.

    Returns:
        dict: ``{held_bar_id: {"robot_name": str, "hold_start_seq": int,
        "release_after_seq": int, "release_after_bar_id": str}}``.

    Raises:
        RuntimeError: on duplicate steps, on a ``supported_until`` entry that
            names an unknown bar, or when a hold is needed but every support
            robot is busy (more concurrent holds than robots — the error
            names the step and who is holding what, so the user can revisit
            the sequence in RSSequenceEdit).
    """
    # * ---- input sanity: every bar one unique step ----
    seq_to_bar = {}
    for bar_id, seq in bar_seq.items():
        seq = int(seq)
        if seq in seq_to_bar:
            raise RuntimeError(
                f"Duplicate assembly step {seq} (bars {seq_to_bar[seq]!r} and "
                f"{bar_id!r}). Run RSSequenceEdit / the sequence repair first."
            )
        seq_to_bar[seq] = bar_id

    plan = {}
    # Holds that are still active while we walk forward:
    # (release_after_seq, robot_name, held_bar_id)
    active_holds = []

    for bar_id in sorted(bar_seq, key=lambda b: int(bar_seq[b])):
        seq = int(bar_seq[bar_id])
        deps = supported_until.get(bar_id) or []
        if not deps:
            continue

        missing = sorted(d for d in deps if d not in bar_seq)
        if missing:
            raise RuntimeError(
                f"Bar {bar_id!r} lists unknown stabilizing bar(s) {missing} in "
                "supported_until. Run the sequence repair (stale refs) or fix "
                "the list in RSSequenceEdit > EditSupports."
            )

        # Only stabilizers built AFTER this bar keep it unstable.
        later_seqs = [int(bar_seq[d]) for d in deps if int(bar_seq[d]) > seq]
        if not later_seqs:
            continue  # every stabilizer is already built when this bar is placed
        release_after_seq = max(later_seqs)

        # A robot is still busy if its current hold releases at this step or
        # later (the release action runs AFTER this step's assembly actions).
        busy = {robot for (rel, robot, _held) in active_holds if rel >= seq}
        free = [r for r in support_robot_names if r not in busy]
        if not free:
            holding_desc = "; ".join(
                f"{robot} holds {held} until after {seq_to_bar[rel]} (step {rel})"
                for (rel, robot, held) in active_holds
                if rel >= seq
            )
            raise RuntimeError(
                f"No free support robot for bar {bar_id!r} at step {seq}: "
                f"{holding_desc}. More concurrent holds than robots "
                f"({len(support_robot_names)}) — revisit the sequence or the "
                "supported_until lists in RSSequenceEdit."
            )

        robot_name = free[0]
        active_holds.append((release_after_seq, robot_name, bar_id))
        plan[bar_id] = {
            "robot_name": robot_name,
            "hold_start_seq": seq,
            "release_after_seq": release_after_seq,
            "release_after_bar_id": seq_to_bar[release_after_seq],
        }

    return plan


def robots_holding_at_step(hold_plan: dict, seq: int) -> dict:
    """Which support robots stand frozen in the scene while step ``seq`` runs.

    A hold is present during step ``seq`` when it started at an EARLIER step
    and has not released yet (``hold_start_seq < seq <= release_after_seq``).
    The active bar's own hold (``hold_start_seq == seq``) is deliberately NOT
    included: that robot only arrives mid-step, after the assembly robot has
    finished jointing — the support flow itself deals with it.

    Args:
        hold_plan (dict): a ``derive_hold_plan`` result.
        seq (int): the assembly step being planned.

    Returns:
        dict: ``{robot_name: held_bar_id}``.
    """
    seq = int(seq)
    return {
        entry["robot_name"]: held_bar_id
        for held_bar_id, entry in hold_plan.items()
        if entry["hold_start_seq"] < seq <= entry["release_after_seq"]
    }


def build_action_schedule(
    assembly_seq: list,
    hold_plan: dict,
    assembly_robot_name: str = "Cindy",
) -> list:
    """The global interleaved action order across all robots.

    Per bar, in sequence order: Jointing; Holding (when the bar needs it);
    Release; then every HoldingRelease whose last stabilizing bar is this bar.
    This is exactly the ordering in the docs/support_demo walkthrough.

    Args:
        assembly_seq (list): bar ids in ascending step order.
        hold_plan (dict): a ``derive_hold_plan`` result.
        assembly_robot_name (str): the assembly robot's name.

    Returns:
        list: ordered ``{"kind", "bar_id", "robot_name"}`` dicts, where kind
        is one of "jointing" / "holding" / "release" / "holding_release".
        For "holding"/"holding_release" the bar_id is the HELD bar.
    """
    entries = []
    for bar_id in assembly_seq:
        entries.append(
            {"kind": "jointing", "bar_id": bar_id, "robot_name": assembly_robot_name}
        )
        if bar_id in hold_plan:
            entries.append(
                {"kind": "holding", "bar_id": bar_id,
                 "robot_name": hold_plan[bar_id]["robot_name"]}
            )
        entries.append(
            {"kind": "release", "bar_id": bar_id, "robot_name": assembly_robot_name}
        )
        # Every hold whose LAST stabilizing bar is this bar releases now,
        # in hold-start order (first held, first released).
        releasing = sorted(
            (held for held, e in hold_plan.items() if e["release_after_bar_id"] == bar_id),
            key=lambda held: hold_plan[held]["hold_start_seq"],
        )
        for held in releasing:
            entries.append(
                {"kind": "holding_release", "bar_id": held,
                 "robot_name": hold_plan[held]["robot_name"]}
            )
    return entries
