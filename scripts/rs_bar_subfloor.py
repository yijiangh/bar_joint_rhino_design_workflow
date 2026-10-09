#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSBarSubfloor - Add a subfloor bar between two existing bars.

Like RSBarBrace, but lets the user choose two DIFFERENT joint pairs:
one for the joint with the first (Left) existing bar and one for the
joint with the second (Right) existing bar.  The first picked bar is
treated as the Left bar; the second as the Right bar.  This left/right
assignment is implicit (not stored in the model).

The pair selection is exposed as command-line options on the first
bar pick (similar to RSBarBrace's ``Pair=`` option):

    LeftJointPair=...  RightJointPair=...  SwapLeftRight  Length=...

Defaults are kept in document user-text
(``scaffolding.last_subfloor_left_pair`` /
``scaffolding.last_subfloor_right_pair``).

Geometry constraint: the underlying S2-T1 solver assumes a single
bar-to-bar contact distance for both ends of the subfloor bar.  Thus
both chosen pairs MUST have the same ``contact_distance_mm``.  If they
differ, the command aborts with a console message before running the
solver.

The body is shared with RSBarBrace in ``core.two_contact_bar``; this file
keeps the first-bar picker (left / right pairs) and the subfloor wording.
"""

import importlib
import os
import sys

import Rhino
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import geometry
from core import two_contact_bar
from core.joint_pair import get_joint_pair
from core.rhino_bar_pick import (
    bar_or_tube_filter,
    get_default_brace_length,
    get_default_pair_name,
    get_default_subfloor_left_pair_name,
    get_default_subfloor_right_pair_name,
    require_pair_names,
    resolve_picked_to_bar_curve,
    set_default_brace_length,
    set_default_subfloor_left_pair_name,
    set_default_subfloor_right_pair_name,
)
from core.rhino_bar_registry import repair_on_entry


# Tolerance (mm) for considering two pairs' contact distances equal.
_CONTACT_DISTANCE_TOL_MM = 1e-3

COMMAND = two_contact_bar.TwoContactCommand(
    name="RSBarSubfloor",
    noun="subfloor",
    length_noun="bar length",
    second_bar_prompt="Select second existing bar (becomes RIGHT bar)",
    contact_prompts=("Pick contact point on LEFT bar", "Pick contact point on RIGHT bar"),
    slide_options=("SlidePointOnLeft", "SlidePointOnRight"),
    repick_prompts=("Pick new contact point on Left bar", "Pick new contact point on Right bar"),
)


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)
    importlib.reload(two_contact_bar)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _pick_le1_with_dual_pair_and_length(
    bar_prompt: str,
    command_name: str = "RSBarSubfloor",
    *,
    length_option_name: str = "Length",
    length_min: float = 1.0,
    length_max: float = 100000.0,
):
    """Pick the first existing bar (Le1) while exposing two independent
    ``LeftJointPair`` / ``RightJointPair`` list options, a ``SwapLeftRight``
    toggle, and a numeric ``Length`` option.  RSBarSubfloor-specific.

    Returns ``(bar_curve_id, left_pair, right_pair, length_mm)`` or
    ``(None, None, None, None)`` on cancel / empty registry.
    """
    names = require_pair_names(command_name)
    if names is None:
        return None, None, None, None

    fallback = get_default_pair_name() or names[0]
    if fallback not in names:
        fallback = names[0]
    left_default = get_default_subfloor_left_pair_name() or fallback
    if left_default not in names:
        left_default = fallback
    right_default = get_default_subfloor_right_pair_name() or fallback
    if right_default not in names:
        right_default = fallback

    left_index = names.index(left_default)
    right_index = names.index(right_default)

    length_value = Rhino.Input.Custom.OptionDouble(
        get_default_brace_length(), length_min, length_max
    )

    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(bar_prompt)
        go.EnablePreSelect(True, True)
        go.SetCustomGeometryFilter(bar_or_tube_filter)

        if len(names) == 1:
            left_opt_index = right_opt_index = swap_opt_index = -1
        else:
            left_opt_index = go.AddOptionList("LeftJointPair", names, left_index)
            right_opt_index = go.AddOptionList("RightJointPair", names, right_index)
            swap_opt_index = go.AddOption("SwapLeftRight")
        go.AddOptionDouble(length_option_name, length_value)

        result = go.Get()
        if result == Rhino.Input.GetResult.Cancel:
            return None, None, None, None

        if result == Rhino.Input.GetResult.Option:
            opt = go.Option()
            if opt is not None:
                if opt.Index == left_opt_index:
                    left_index = int(opt.CurrentListOptionIndex)
                elif opt.Index == right_opt_index:
                    right_index = int(opt.CurrentListOptionIndex)
                elif opt.Index == swap_opt_index:
                    left_index, right_index = right_index, left_index
                    print(
                        f"  swapped: LeftJointPair='{names[left_index]}', "
                        f"RightJointPair='{names[right_index]}'"
                    )
            continue

        if result == Rhino.Input.GetResult.Object:
            picked_id = go.Object(0).ObjectId
            rs.UnselectObject(picked_id)
            bar_id = resolve_picked_to_bar_curve(picked_id)
            if bar_id is None:
                continue
            left_name = names[left_index]
            right_name = names[right_index]
            set_default_subfloor_left_pair_name(left_name)
            set_default_subfloor_right_pair_name(right_name)
            chosen_length = float(length_value.CurrentValue)
            set_default_brace_length(chosen_length)
            return (
                bar_id,
                get_joint_pair(left_name),
                get_joint_pair(right_name),
                chosen_length,
            )

        return None, None, None, None


def main():
    _refresh_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSBarSubfloor")
    rs.UnselectAllObjects()

    le1_id, left_pair, right_pair, brace_length = (
        _pick_le1_with_dual_pair_and_length(
            "Select first existing bar (becomes LEFT bar)",
            command_name="RSBarSubfloor",
        )
    )
    if le1_id is None or left_pair is None or right_pair is None:
        return

    # Sanity check: the S2-T1 solver assumes a single bar-to-bar contact
    # distance for both ends.  Reject mismatched pairs here so the user
    # gets a clear message instead of a meaningless solver failure.
    left_d = float(left_pair.contact_distance_mm)
    right_d = float(right_pair.contact_distance_mm)
    if abs(left_d - right_d) > _CONTACT_DISTANCE_TOL_MM:
        msg = (
            f"LeftJointPair '{left_pair.name}' and RightJointPair "
            f"'{right_pair.name}' have different bar-to-bar contact "
            f"distances ({left_d:.4f} mm vs {right_d:.4f} mm).  "
            f"The subfloor solver requires both ends to share a single "
            f"distance.  Either pick two pairs with matching "
            f"contact_distance_mm, or redefine one pair so they agree."
        )
        print(f"RSBarSubfloor: {msg}")
        rs.MessageBox(msg, 0, "RSBarSubfloor")
        return

    target_distance = left_d
    print(
        f"RSBarSubfloor: LEFT pair '{left_pair.name}', "
        f"RIGHT pair '{right_pair.name}' "
        f"(contact distance {target_distance:.4f} mm), "
        f"bar length {float(brace_length):.2f} mm"
    )

    two_contact_bar.run(
        COMMAND, le1_id, (left_pair, right_pair), target_distance, brace_length
    )


if __name__ == "__main__":
    main()
