"""Build-stage arithmetic for the persistent ``HideUnbuilt`` filter.

The filter itself lives in :mod:`core.rhino_bar_registry` -- this module holds
only the part that needs no Rhino: the document-user-text key, the wire format
of its value, and the rule that turns a saved stage into "hide everything past
step N".

Why a separate module.  ``rhino_bar_registry`` imports ``Rhino`` /
``rhinoscriptsyntax`` / ``scriptcontext`` at module level, so nothing in it can
be imported outside the Rhino process -- ``pytest`` cannot reach it at all (see
``tests/conftest.py``).  The resolution rule below has three branches that only
fire after a renumber or a delete, which is exactly what is tedious to verify by
hand, so it lives here where ``tests/test_build_stage.py`` can exercise it.

The saved value
---------------
One document user-text entry, saved inside the .3dm::

    scaffolding.build_stage = "B7|7"      # bar id | step number.  Missing = filter off.

It carries **both** the bar id and the step number on purpose.  Bar ids are not
permanent -- ``RSReorderBarID`` renames them -- so after a renumber the bar you
staged at might be called B4 while a *different* bar is now called B7.  The id is
the primary key (follow the physical bar); the step is the safety net for when
that bar no longer exists.
"""

#: Document user-text key holding the active build stage.  Read/written through
#: ``rhino_bar_registry.get_build_stage`` / ``set_build_stage``.
BUILD_STAGE_KEY = "scaffolding.build_stage"

#: Separator between the bar id and the step number in the saved value.
_SEPARATOR = "|"

#: Resolution outcomes returned by :func:`resolve_build_stage_seq`.
STATUS_OFF = "off"            # not latched -- nothing to do
STATUS_OK = "ok"              # stage bar still exists; use its CURRENT seq
STATUS_FALLBACK = "fallback"  # stage bar gone; nearest earlier step used instead
STATUS_CLEAR = "clear"        # unresolvable; switch the filter off and say why


def format_build_stage(bar_id, seq):
    """Serialise *bar_id* + *seq* to the saved form, e.g. ``"B7|7"``."""
    return f"{bar_id}{_SEPARATOR}{int(seq)}"


def parse_build_stage(raw):
    """Parse a saved value back to ``(bar_id, seq)``, or ``None`` if unreadable.

    Never raises -- a hand-edited or truncated value is just ``None``, which the
    caller treats as "switch the filter off".
    """
    if not raw:
        return None
    parts = str(raw).split(_SEPARATOR)
    if len(parts) != 2:
        return None
    bar_id = parts[0].strip()
    if not bar_id:
        return None
    try:
        seq = int(parts[1].strip())
    except (TypeError, ValueError):
        return None
    return (bar_id, seq)


def resolve_build_stage_seq(raw, bar_map):
    """Decide which assembly step the saved stage *raw* means right now.

    Args:
        raw: the saved value, e.g. ``"B7|7"``.  Empty / ``None`` = not latched.
        bar_map: ``{bar_id: (oid, seq)}`` as returned by
            ``rhino_bar_registry.get_bar_seq_map()``.  Only ``seq`` is read, so
            any 2-sequence works; *oid* is ignored.

    Returns:
        ``(status, seq, bar_id, message)``.

        =============== ======================================= ==============================
        status          when                                    caller does
        =============== ======================================= ==============================
        ``off``         *raw* is empty                          nothing, the filter is off
        ``ok``          the stage bar still exists              hide everything past *seq*
        ``fallback``    the stage bar is gone, but a bar sits   rewrite the saved value to
                        at or before the saved step             *bar_id*, print *message*, hide
        ``clear``       unreadable, or nothing at/before the    wipe the saved value, print
                        saved step                              *message*, show everything
        =============== ======================================= ==============================

        On ``ok`` / ``fallback``, *seq* is the bar's **current** step, which is
        what makes a renumber transparent: ``"B7|7"`` against a document where
        B7 has since moved to step 4 resolves to 4, not 7.
    """
    if not raw:
        return (STATUS_OFF, None, None, None)

    parsed = parse_build_stage(raw)
    if parsed is None:
        return (
            STATUS_CLEAR,
            None,
            None,
            f"stage filter: saved value {raw!r} is unreadable -- filter switched "
            "off, everything is visible again.",
        )

    bar_id, stored_seq = parsed
    entry = bar_map.get(bar_id)
    if entry is not None:
        return (STATUS_OK, int(entry[1]), bar_id, None)

    # The staged bar is gone (deleted, or renamed by a path that did not remap
    # the key).  Fall back to the nearest EARLIER step so the view barely moves,
    # instead of dropping the filter and revealing the whole model at once.
    earlier = [
        (int(seq), bid)
        for bid, (_oid, seq) in bar_map.items()
        if int(seq) <= stored_seq
    ]
    if earlier:
        new_seq, new_bar_id = max(earlier)
        return (
            STATUS_FALLBACK,
            new_seq,
            new_bar_id,
            f"stage filter: {bar_id} (step {stored_seq}) no longer exists -- fell "
            f"back to {new_bar_id} (step {new_seq}), the nearest earlier step.",
        )

    return (
        STATUS_CLEAR,
        None,
        None,
        f"stage filter: {bar_id} (step {stored_seq}) no longer exists and no bar "
        "remains at or before that step -- filter switched off, everything is "
        "visible again.",
    )


def stage_filter_note(caller, bar_id, seq):
    """The one shared reminder printed while the filter is latched.

    Without it, a user who forgot the latch has no idea why half the model is
    missing and why bars refuse to be picked.
    """
    return (
        f"{caller or 'RSScaffolding'} (stage filter): showing up to {bar_id} "
        f"(step {seq}). Later bars/joints/tools are HIDDEN and cannot be picked "
        "or snapped to. RSSequenceEdit > ShowUnbuilt to reveal."
    )
