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
- The LAST stabilizing bar (highest step) is the hold's
  ``release_after_bar_id``. When the robot lets go, relative to that bar, is
  the held bar's release timing (``supported_release`` user text, picked in
  RSSequenceEdit > EditSupports):

  - ``before_insert``: right before the last bar's insert. The held bar
    stands unheld while the assembly robot inserts the last bar.
  - ``after_insert``: right after the last bar's insert, while the assembly
    robot still grips it, and before the last bar's own hold. For example,
    Cindy inserts B25, Alice lets go of B21, Alice holds B25, Cindy releases
    B25.
  - ``after_release`` (the default, the only behaviour before 2026-11): after
    the assembly robot has released the last bar.

- A support robot is busy from its hold's start step until it lets go. One
  that lets go ``before_insert`` or ``after_insert`` is free for the last
  bar's own hold at that same step. One that lets go ``after_release`` is
  still busy at that step.
- When more than one robot is free, the alphabetically-first one is picked
  (the ``support_robot_names`` tuple is priority-ordered).

Occupancy is DERIVED, never stored: this module recomputes the whole plan
from the sequence data on every call, so sequence edits can never leave a
stale "occupied" flag anywhere.
"""

from __future__ import annotations

#: The three release timings (values of the ``supported_release`` user text).
BEFORE_INSERT = "before_insert"
AFTER_INSERT = "after_insert"
AFTER_RELEASE = "after_release"
RELEASE_TIMINGS = (BEFORE_INSERT, AFTER_INSERT, AFTER_RELEASE)
#: A bar with a support list but no saved timing keeps the old behaviour.
DEFAULT_RELEASE_TIMING = AFTER_RELEASE


def derive_hold_plan(
    bar_seq: dict,
    supported_until: dict,
    support_robot_names: tuple = ("Alice", "Belle"),
    release_timing: dict = None,
) -> dict:
    """Assign a support robot + hold interval to every bar that needs holding.

    Args:
        bar_seq (dict): ``{bar_id: assembly step int}`` for every bar.
        supported_until (dict): ``{bar_id: [stabilizing bar ids]}``; bars
            missing here (or with an empty list) never need holding.
        support_robot_names (tuple): priority-ordered robot names; callers in
            Rhino pass ``config.SUPPORT_ROBOT_NAMES``.
        release_timing (dict): ``{bar_id: one of RELEASE_TIMINGS}``; bars
            missing here use ``DEFAULT_RELEASE_TIMING``.

    Returns:
        dict: ``{held_bar_id: {"robot_name": str, "hold_start_seq": int,
        "release_after_seq": int, "release_after_bar_id": str,
        "release_timing": str}}``.

    Raises:
        RuntimeError: on duplicate steps, on a ``supported_until`` entry that
            names an unknown bar, on an unknown release timing, or when a hold
            is needed but every support robot is busy (more concurrent holds
            than robots — the error names the step and who is holding what,
            so the user can revisit the sequence in RSSequenceEdit).
    """
    release_timing = release_timing or {}
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
    # (release_after_seq, robot_name, held_bar_id, release timing)
    active_holds = []

    def still_busy(rel, timing, seq):
        """Whether a hold that ends at step `rel` keeps its robot from a new hold at step `seq`."""
        return rel > seq or (rel == seq and timing == AFTER_RELEASE)

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
        timing = release_timing.get(bar_id) or DEFAULT_RELEASE_TIMING
        if timing not in RELEASE_TIMINGS:
            raise RuntimeError(
                f"Bar {bar_id!r} has an unknown release timing {timing!r} "
                f"(expected one of {', '.join(RELEASE_TIMINGS)}). Pick it again in "
                "RSSequenceEdit > EditSupports."
            )

        # Only stabilizers built AFTER this bar keep it unstable.
        later_seqs = [int(bar_seq[d]) for d in deps if int(bar_seq[d]) > seq]
        if not later_seqs:
            continue  # every stabilizer is already built when this bar is placed
        release_after_seq = max(later_seqs)

        # A robot whose hold ends at this very step is free if it lets go before
        # this bar's hold (before or after this bar's insert), busy otherwise.
        busy = {robot for (rel, robot, _held, t) in active_holds if still_busy(rel, t, seq)}
        free = [r for r in support_robot_names if r not in busy]
        if not free:
            holding_desc = "; ".join(
                f"{robot} holds {held} until {seq_to_bar[rel]} (step {rel}, {t})"
                for (rel, robot, held, t) in active_holds
                if still_busy(rel, t, seq)
            )
            raise RuntimeError(
                f"No free support robot for bar {bar_id!r} at step {seq}: "
                f"{holding_desc}. More concurrent holds than robots "
                f"({len(support_robot_names)}) — revisit the sequence or the "
                "supported_until lists in RSSequenceEdit."
            )

        robot_name = free[0]
        active_holds.append((release_after_seq, robot_name, bar_id, timing))
        plan[bar_id] = {
            "robot_name": robot_name,
            "hold_start_seq": seq,
            "release_after_seq": release_after_seq,
            "release_after_bar_id": seq_to_bar[release_after_seq],
            "release_timing": timing,
        }

    return plan


def describe_release(entry: dict) -> str:
    """When a hold lets go, in words: e.g. "after B25's insert (step 11)".

    Args:
        entry (dict): one ``derive_hold_plan`` entry.
    """
    bar, seq = entry["release_after_bar_id"], entry["release_after_seq"]
    when = {BEFORE_INSERT: "before {}'s insert", AFTER_INSERT: "after {}'s insert",
            AFTER_RELEASE: "after {}'s release"}[entry.get("release_timing", DEFAULT_RELEASE_TIMING)]
    return f"{when.format(bar)} (step {seq})"


def robots_holding_at_step(hold_plan: dict, seq: int) -> dict:
    """Which support robots stand frozen in the scene while step ``seq``'s bar is inserted.

    A hold is present during step ``seq`` when it started at an EARLIER step
    and has not released yet: ``hold_start_seq < seq <= release_after_seq``,
    except that a ``before_insert`` hold is already gone during its last
    bar's step. The active bar's own hold (``hold_start_seq == seq``) is
    deliberately NOT included: that robot only arrives mid-step, after the
    assembly robot has finished jointing — the support flow itself deals with
    it.

    Args:
        hold_plan (dict): a ``derive_hold_plan`` result.
        seq (int): the assembly step being planned.

    Returns:
        dict: ``{robot_name: held_bar_id}``.
    """
    seq = int(seq)
    out = {}
    for held_bar_id, entry in hold_plan.items():
        rel = entry["release_after_seq"]
        gone_at_rel = entry.get("release_timing", DEFAULT_RELEASE_TIMING) == BEFORE_INSERT
        if entry["hold_start_seq"] < seq and (seq < rel or (seq == rel and not gone_at_rel)):
            out[entry["robot_name"]] = held_bar_id
    return out


def build_action_schedule(
    assembly_seq: list,
    hold_plan: dict,
    assembly_robot_name: str = "Cindy",
) -> list:
    """The global interleaved action order across all robots.

    Per bar L, in sequence order:

    1. HoldingRelease of every hold whose last stabilizing bar is L, timing
       ``before_insert``;
    2. Jointing of L;
    3. HoldingRelease of those with timing ``after_insert``;
    4. Holding of L (when L needs it);
    5. Release of L;
    6. HoldingRelease of those with timing ``after_release``.

    Within one group, first held is first released. With ``after_insert``
    this gives, e.g., Cindy inserts B25, Alice lets go of B21, Alice holds
    B25, Cindy releases B25.

    Args:
        assembly_seq (list): bar ids in ascending step order.
        hold_plan (dict): a ``derive_hold_plan`` result.
        assembly_robot_name (str): the assembly robot's name.

    Returns:
        list: ordered ``{"kind", "bar_id", "robot_name"}`` dicts, where kind
        is one of "jointing" / "holding" / "release" / "holding_release".
        For "holding"/"holding_release" the bar_id is the HELD bar.
    """
    def releases(bar_id, timing):
        held_bars = sorted(
            (held for held, e in hold_plan.items()
             if e["release_after_bar_id"] == bar_id
             and e.get("release_timing", DEFAULT_RELEASE_TIMING) == timing),
            key=lambda held: hold_plan[held]["hold_start_seq"],
        )
        return [{"kind": "holding_release", "bar_id": held,
                 "robot_name": hold_plan[held]["robot_name"]} for held in held_bars]

    entries = []
    for bar_id in assembly_seq:
        entries += releases(bar_id, BEFORE_INSERT)
        entries.append(
            {"kind": "jointing", "bar_id": bar_id, "robot_name": assembly_robot_name}
        )
        entries += releases(bar_id, AFTER_INSERT)
        if bar_id in hold_plan:
            entries.append(
                {"kind": "holding", "bar_id": bar_id,
                 "robot_name": hold_plan[bar_id]["robot_name"]}
            )
        entries.append(
            {"kind": "release", "bar_id": bar_id, "robot_name": assembly_robot_name}
        )
        entries += releases(bar_id, AFTER_RELEASE)
    return entries
