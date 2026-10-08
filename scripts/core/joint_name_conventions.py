"""Every naming rule for joints, bars, tools and collision bodies -- one module.

Nothing outside this module builds or slices one of these names.  Pure Python,
no Rhino, so every rule here is unit-tested (``tests/test_joint_name_conventions.py``).

The one rule
------------
Every joint block is named ``<Type>_<Subtype>``:

    T20_Female   T20_Male   T20_Ground   T20_MoCap
    T20SubLeft_Female   T20SubRight_Female   T20Deck12_Male

``Type`` is the product family; ``Subtype`` says what the block does and is one
of :data:`SUBTYPES`.  Everything else is DERIVED (example: ``T20_MoCap`` as the
receiver of a pair between bars B40 and B53):

    name                 rule                                         example
    -------------------  -------------------------------------------  --------------------
    role                 Subtype lower-cased                          mocap
    registry ``kind``    the role                                     mocap
    Rhino layer          MANAGED Scaffolding::Joint <Subtype> Instances
    joint id, pair       J<receiver bar>-<male bar>                   J40-53
    joint id, one bar    G<bar>-<Type>-<i>  /  M<bar>-<Type>-<i>      M40-T20-0
    object name          <joint id>_<role>                            J40-53_mocap
    collision key        joint_<joint id>_<role>                      joint_J40-53_mocap
    tool id              T<joint id>                                  (MoCap has no tool)

The LAYER a placed block sits on is the authority for its Subtype.  User text is
copied verbatim onto a duplicated object, so code that needs a placed block's
role asks :func:`subtype_of_layer`.

Code compares Subtypes (``"Female"``).  The lower-case role (``"female"``)
appears only inside strings written to disk or to PyBullet, and only this module
spells it.
"""

from __future__ import annotations

import re


# ---------------------------------------------------------------------------
# Subtypes -- THE declaration
# ---------------------------------------------------------------------------

FEMALE = "Female"
MALE = "Male"
GROUND = "Ground"
#: A Female with an OptiTrack marker plate bolted to its back.  A male screws
#: into it exactly as into a Female, but it is fitted BY HAND, so it never
#: carries a robot tool.
MOCAP = "MoCap"

#: Every Subtype, in layer scan order.
SUBTYPES = (FEMALE, MALE, GROUND, MOCAP)

#: Carries a robotic tool.  ``rs_ik_keyframe._resolve_arm_tools_on_bar``
#: requires EXACTLY TWO of these on the bar being assembled, so adding one here
#: breaks every bar that carries one.
TOOL_BEARING_SUBTYPES = (MALE, GROUND)
#: A male screws INTO these -- the receiving slot of a mate.
RECEIVER_SUBTYPES = (FEMALE, MOCAP)
#: The halves a MATE places: either receiver, plus the male.
PAIRED_SUBTYPES = (*RECEIVER_SUBTYPES, MALE)
#: May be placed on ONE bar with no partner.
SINGLE_SIDED_SUBTYPES = (GROUND, MOCAP)
#: Stored in the registry's ``halves`` table (they have a screw bore).  Ground
#: lives in its own ``ground_joints`` table: no partner, no screw frame.
HALF_SUBTYPES = (FEMALE, MALE, MOCAP)

_SUBTYPE_BY_ROLE = {subtype.lower(): subtype for subtype in SUBTYPES}


def _check(subtype: str) -> str:
    if subtype not in SUBTYPES:
        raise ValueError(
            f"unknown joint subtype {subtype!r}; expected one of {SUBTYPES}"
        )
    return subtype


def role(subtype: str) -> str:
    """``"MoCap"`` -> ``"mocap"``: the spelling used inside names on disk."""
    return _check(subtype).lower()


# ---------------------------------------------------------------------------
# Rhino layers
# ---------------------------------------------------------------------------

LAYER_PATH_SEP = "::"
MANAGED_LAYER_ROOT = "MANAGED Scaffolding"


def managed_layer(name: str) -> str:
    """Full path of a sublayer of :data:`MANAGED_LAYER_ROOT`."""
    return MANAGED_LAYER_ROOT + LAYER_PATH_SEP + name


def joint_layer(subtype: str) -> str:
    """The layer every placed block of *subtype* sits on.

    These strings are recorded inside every saved .3dm, so this rule can never
    change; the tests pin them.
    """
    return managed_layer(f"Joint {_check(subtype)} Instances")


_SUBTYPE_BY_LAYER = {joint_layer(s): s for s in SUBTYPES}


def subtype_of_layer(layer: str) -> str | None:
    """The Subtype a joint layer holds, or ``None`` for any other layer."""
    return _SUBTYPE_BY_LAYER.get(layer)


LAYER_FEMALE = joint_layer(FEMALE)
LAYER_MALE = joint_layer(MALE)
LAYER_GROUND = joint_layer(GROUND)
LAYER_MOCAP = joint_layer(MOCAP)

#: Every layer holding placed joint blocks.
JOINT_LAYERS = tuple(joint_layer(s) for s in SUBTYPES)
TOOL_BEARING_LAYERS = tuple(joint_layer(s) for s in TOOL_BEARING_SUBTYPES)
RECEIVER_LAYERS = tuple(joint_layer(s) for s in RECEIVER_SUBTYPES)
SINGLE_SIDED_LAYERS = tuple(joint_layer(s) for s in SINGLE_SIDED_SUBTYPES)
#: The layers a half placed BY A MATE can sit on: either receiver, plus male.
#: A standalone MoCap half shares the MoCap layer but belongs to no mate.
PAIRED_LAYERS = tuple(joint_layer(s) for s in PAIRED_SUBTYPES)


# ---------------------------------------------------------------------------
# Block names
# ---------------------------------------------------------------------------


def block_name(type_: str, subtype: str) -> str:
    """``("T20", "MoCap")`` -> ``"T20_MoCap"``."""
    if not type_:
        raise ValueError("a joint block needs a non-empty Type")
    return f"{type_}_{_check(subtype)}"


def split_block_name(name: str) -> tuple[str, str]:
    """``"T20_MoCap"`` -> ``("T20", "MoCap")``.

    Splits at the LAST underscore, so a Type may itself contain one.  Raises
    ``ValueError`` for a name that does not follow ``<Type>_<Subtype>``.
    """
    type_, sep, subtype = str(name).rpartition("_")
    if not sep or not type_ or subtype not in SUBTYPES:
        raise ValueError(
            f"joint block {name!r} is not named <Type>_<Subtype> "
            f"with Subtype one of {SUBTYPES} (e.g. 'T20_Female')"
        )
    return type_, subtype


def is_block_name(name: str) -> bool:
    try:
        split_block_name(name)
    except ValueError:
        return False
    return True


def block_type(name: str) -> str:
    """``"T20SubLeft_Female"`` -> ``"T20SubLeft"``."""
    return split_block_name(name)[0]


def block_subtype(name: str) -> str:
    """``"T20SubLeft_Female"`` -> ``"Female"``."""
    return split_block_name(name)[1]


# ---------------------------------------------------------------------------
# Bars
# ---------------------------------------------------------------------------

BAR_PREFIX = "B"


def bar_id(number) -> str:
    """``7`` -> ``"B7"``."""
    return f"{BAR_PREFIX}{int(number)}"


def bar_num(bar_id_: str) -> str:
    """``"B7"`` -> ``"7"``.  ``"?"`` for an empty id; any other text unchanged."""
    text = str(bar_id_ or "")
    if not text:
        return "?"
    return text[len(BAR_PREFIX):] if text.startswith(BAR_PREFIX) else text


def bar_number(bar_id_: str) -> int | None:
    """``"B7"`` -> ``7``; ``None`` when it is not a ``B<int>`` id."""
    text = str(bar_id_ or "").strip().upper()
    if not text.startswith(BAR_PREFIX):
        return None
    try:
        return int(text[len(BAR_PREFIX):])
    except ValueError:
        return None


def bar_sort_key(bar_id_: str) -> float:
    """Sort key for bar ids: ``B2`` before ``B10``; anything else last."""
    number = bar_number(bar_id_)
    return float("inf") if number is None else number


def parse_bar_id(token: str) -> str | None:
    """User input ``"b12"`` / ``"12"`` / ``" B012 "`` -> ``"B12"``; else ``None``."""
    text = str(token or "").strip().upper()
    if text.startswith(BAR_PREFIX):
        text = text[len(BAR_PREFIX):]
    if not text.isdigit():
        return None
    return bar_id(int(text))


# ---------------------------------------------------------------------------
# Joint ids
# ---------------------------------------------------------------------------

PAIR_ID_PREFIX = "J"
#: Single-sided joint ids start with this letter: ``G4-T20-0``, ``M7-T20-0``.
SINGLE_ID_PREFIX = {GROUND: "G", MOCAP: "M"}
_SUBTYPE_BY_SINGLE_PREFIX = {v: k for k, v in SINGLE_ID_PREFIX.items()}

_SINGLE_ID_RE = re.compile(r"^([A-Z])([^-]+)-(.+)-(\d+)$")


def pair_joint_id(receiver_bar_id: str, male_bar_id: str) -> str:
    """``("B40", "B53")`` -> ``"J40-53"``.  Receiver bar first."""
    return f"{PAIR_ID_PREFIX}{bar_num(receiver_bar_id)}-{bar_num(male_bar_id)}"


def single_joint_id_base(subtype: str, bar_id_: str, type_: str) -> str:
    """``(GROUND, "B4", "T20")`` -> ``"G4-T20"`` -- the id minus its index."""
    if subtype not in SINGLE_ID_PREFIX:
        raise ValueError(f"{subtype!r} is not a single-sided subtype")
    return f"{SINGLE_ID_PREFIX[subtype]}{bar_num(bar_id_)}-{type_}"


def single_joint_id(subtype: str, bar_id_: str, type_: str, index: int) -> str:
    """``(GROUND, "B4", "T20", 0)`` -> ``"G4-T20-0"``.

    Several single-sided joints of one Type can sit on one bar; *index* tells
    them apart (the caller picks the next free one from the document).
    """
    return f"{single_joint_id_base(subtype, bar_id_, type_)}-{int(index)}"


def split_single_joint_id(jid: str):
    """``"G4-T20-0"`` -> ``(GROUND, "B4", "T20", 0)``; ``None`` otherwise."""
    match = _SINGLE_ID_RE.match(str(jid or ""))
    if not match or match.group(1) not in _SUBTYPE_BY_SINGLE_PREFIX:
        return None
    return (
        _SUBTYPE_BY_SINGLE_PREFIX[match.group(1)],
        f"{BAR_PREFIX}{match.group(2)}",
        match.group(3),
        int(match.group(4)),
    )


def single_sided_subtype_of_id(jid: str) -> str | None:
    """``GROUND`` for ``"G..."``, ``MOCAP`` for ``"M..."``, ``None`` for a pair id."""
    parts = split_single_joint_id(jid)
    return parts[0] if parts else None


def next_free_index(used) -> int:
    """The smallest index >= 0 not in *used* -- the ``i`` of a new ``G4-T20-<i>``."""
    i = 0
    while i in used:
        i += 1
    return i


def is_single_sided(subtype: str, jid: str) -> bool:
    """True for a joint on ONE bar: every Ground block, and a MoCap block whose
    id is a standalone ``M…`` id.

    *subtype* comes from the block's LAYER (the authority); a MoCap block with a
    ``J…`` id is the receiver of a pair, not single-sided.
    """
    if subtype == GROUND:
        return True
    return subtype == MOCAP and single_sided_subtype_of_id(jid) == MOCAP


def is_paired_half(subtype: str, jid: str) -> bool:
    """True for a half placed by a mate: Female / Male, or a MoCap receiver."""
    return subtype in PAIRED_SUBTYPES and not is_single_sided(subtype, jid)


def rebar_single_joint_id(jid: str, new_bar_id: str) -> str:
    """The same single-sided id, moved to *new_bar_id* (RSReorderBarID)."""
    parts = split_single_joint_id(jid)
    if parts is None:
        raise ValueError(f"{jid!r} is not a single-sided joint id")
    subtype, _old_bar, type_, index = parts
    return single_joint_id(subtype, new_bar_id, type_, index)


# ---------------------------------------------------------------------------
# Object names, collision keys, tool ids
# ---------------------------------------------------------------------------


def object_name(jid: str, subtype: str) -> str:
    """``("J40-53", MOCAP)`` -> ``"J40-53_mocap"``: a placed block's Rhino name."""
    return f"{jid}_{role(subtype)}"


def split_object_name(name: str) -> tuple[str, str | None]:
    """``"J40-53_mocap"`` -> ``("J40-53", MOCAP)``; ``(name, None)`` without a role.

    Splits at the LAST underscore -- safe because no role contains one.
    """
    base, sep, tail = str(name or "").rpartition("_")
    if sep and base and tail.lower() in _SUBTYPE_BY_ROLE:
        return base, _SUBTYPE_BY_ROLE[tail.lower()]
    return str(name or ""), None


#: PyBullet body-key prefixes.  The dual-arm assembly cell uses the plain ones;
#: the single-arm support cell uses the ``env_`` ones; ``obstacle_`` bodies come
#: from the Environment layer.  The namespaces never collide.
JOINT_KEY_PREFIX = "joint_"
BAR_KEY_PREFIX = "bar_"
OBSTACLE_KEY_PREFIX = "obstacle_"
ENV_JOINT_KEY_PREFIX = "env_joint_"
ENV_BAR_KEY_PREFIX = "env_bar_"


def joint_key(jid: str, subtype: str, *, env: bool = False) -> str:
    """``("J40-53", MOCAP)`` -> ``"joint_J40-53_mocap"``."""
    prefix = ENV_JOINT_KEY_PREFIX if env else JOINT_KEY_PREFIX
    return prefix + object_name(jid, subtype)


def split_joint_key(key: str, *, env: bool = False) -> tuple[str, str] | None:
    """``"joint_J40-53_mocap"`` -> ``("J40-53", MOCAP)``; ``None`` for any other key."""
    prefix = ENV_JOINT_KEY_PREFIX if env else JOINT_KEY_PREFIX
    text = str(key)
    if not text.startswith(prefix):
        return None
    jid, subtype = split_object_name(text[len(prefix):])
    if subtype is None:
        return None
    return jid, subtype


def is_joint_key(key: str, subtypes=SUBTYPES, *, env: bool = False) -> bool:
    """True when *key* is the collision key of a joint of one of *subtypes*."""
    parts = split_joint_key(key, env=env)
    return parts is not None and parts[1] in subtypes


def receiver_keys(jid: str) -> tuple[str, ...]:
    """Every key the receiver of pair *jid* can have, Female first."""
    return tuple(joint_key(jid, s) for s in RECEIVER_SUBTYPES)


def bar_key(bar_id_: str, *, env: bool = False) -> str:
    """``"B40"`` -> ``"bar_B40"``."""
    return (ENV_BAR_KEY_PREFIX if env else BAR_KEY_PREFIX) + str(bar_id_)


def obstacle_key(name: str) -> str:
    return OBSTACLE_KEY_PREFIX + str(name)


TOOL_ID_PREFIX = "T"


def tool_id(jid: str) -> str:
    """``"J40-53"`` -> ``"TJ40-53"``: the id of the tool held at that joint."""
    return f"{TOOL_ID_PREFIX}{jid}"


# ---------------------------------------------------------------------------
# User-text keys on placed blocks (the strings are on disk; never respell)
# ---------------------------------------------------------------------------

UT_JOINT_ID = "joint_id"
#: The block's Type ("T20").
UT_JOINT_TYPE = "joint_type"
#: The block's Subtype ("MoCap").  Informational -- read the LAYER for the role.
UT_JOINT_SUBTYPE = "joint_subtype"
#: Name of the mate a pair was placed from ("T20").
UT_PAIR_NAME = "joint_pair_name"
UT_BLOCK_NAME = "block_name"
UT_PARENT_BAR = "parent_bar_id"
UT_CONNECTED_BAR = "connected_bar_id"
#: The receiver's bar.  Spelled "female" on disk, also for a MoCap receiver.
UT_RECEIVER_BAR = "female_parent_bar"
UT_MALE_BAR = "male_parent_bar"
UT_POSITION = "position_mm"
UT_ROTATION = "rotation_deg"
UT_ORI = "ori"
UT_LE_REV = "le_rev"
UT_LN_REV = "ln_rev"
UT_VARIANT = "variant_index"
UT_FLIPPED = "flipped"
UT_TOOL_ID = "tool_id"
UT_TOOL_NAME = "tool_name"
#: Transient tag on interactive PREVIEW blocks only; its value is a Subtype.
UT_PREVIEW_SUBTYPE = "_joint_role"


# ---------------------------------------------------------------------------
# Mate names
# ---------------------------------------------------------------------------


def default_mate_name(receiver_block: str, male_block: str, registered_blocks) -> str:
    """The name RSDefineJointMate proposes for a new mate.

    Same Type on both halves -> that Type (``T20_Female`` + ``T20_Male`` ->
    ``T20``).  Different Types -> the one that is a VARIANT, i.e. has a single
    registered block (``T20SubLeft_Female`` + ``T20_Male`` -> ``T20SubLeft``;
    ``T20_Female`` + ``T20Deck12_Male`` -> ``T20Deck12``).  Otherwise both
    Types joined.
    """
    receiver_type = block_type(receiver_block)
    male_type = block_type(male_block)
    if receiver_type == male_type:
        return receiver_type
    counts: dict = {}
    for name in registered_blocks:
        if is_block_name(name):
            counts[block_type(name)] = counts.get(block_type(name), 0) + 1
    variants = [t for t in (receiver_type, male_type) if counts.get(t, 0) <= 1]
    if len(variants) == 1:
        return variants[0]
    return f"{receiver_type}{male_type}"


# ---------------------------------------------------------------------------
# Legacy names -- the one-off migration of documents saved before this rule
# ---------------------------------------------------------------------------

#: Block definitions renamed to follow ``<Type>_<Subtype>``.
LEGACY_BLOCK_RENAMES = {"T20Ground": "T20_Ground"}
#: Mates renamed to their Type.
LEGACY_MATE_RENAMES = {
    "T20Deck12_Pair": "T20Deck12",
    "T20SFloorLeft": "T20SubLeft",
    "T20SFloorRight": "T20SubRight",
}


def migrate_joint_id(jid: str) -> str:
    """A legacy ground id ``G4-T20Ground-0`` -> ``G4-T20-0``; any other id unchanged.

    Ground ids used to carry the ground DEFINITION name (which was the block
    name); they now carry the Type.
    """
    parts = split_single_joint_id(jid)
    if parts is None:
        return jid
    subtype, bar, middle, index = parts
    new_block = LEGACY_BLOCK_RENAMES.get(middle)
    if new_block is None:
        return jid
    return single_joint_id(subtype, bar, block_type(new_block), index)
