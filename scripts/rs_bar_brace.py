#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSBarBrace - Add a brace bar between two existing bars.

Pick two existing bars and two contact points. The solver finds up to 4
candidate brace positions. An interactive command-prompt loop lets you:
  - click a colored candidate bar/tube in the viewport to select it
  - click SlidePointOn1 / SlidePointOn2 to re-pick contact points and re-solve
  - Escape to cancel

The body is shared with RSBarSubfloor in ``core.two_contact_bar``; this file
keeps the first-bar picker (one ``Pair`` option) and the brace wording.
"""

from __future__ import annotations

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
from core.joint_pair import JointPairDef, get_joint_pair
from core.rhino_bar_pick import (
    bar_or_tube_filter,
    get_default_brace_length,
    require_pair_names,
    resolve_default_pair_index,
    resolve_picked_to_bar_curve,
    set_default_brace_length,
    set_default_pair_name,
)
from core.rhino_bar_registry import repair_on_entry


COMMAND = two_contact_bar.TwoContactCommand(
    name="RSBarBrace",
    noun="brace",
    length_noun="brace length",
    second_bar_prompt="Select second existing bar (Le2)",
    contact_prompts=("Pick contact point on Le1", "Pick contact point on Le2"),
    slide_options=("SlidePointOn1", "SlidePointOn2"),
    repick_prompts=("Pick new contact point on Le1", "Pick new contact point on Le2"),
)


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)
    importlib.reload(two_contact_bar)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _pick_le1_with_pair_and_length(
    bar_prompt: str,
    command_name: str = "RSBarBrace",
    *,
    length_option_name: str = "Length",
    length_min: float = 1.0,
    length_max: float = 100000.0,
) -> tuple[object, JointPairDef, float] | tuple[None, None, None]:
    """Pick the first existing bar (Le1) while exposing inline ``Pair`` and
    ``Length`` options.  RSBarBrace-specific: the chosen length becomes the
    centered brace line's length and is persisted as the new doc default.

    Returns ``(bar_curve_id, JointPairDef, length_mm)`` or
    ``(None, None, None)`` on cancel / empty registry.
    """
    names = require_pair_names(command_name)
    if names is None:
        return None, None, None

    selected_index = resolve_default_pair_index(names, None)
    length_value = Rhino.Input.Custom.OptionDouble(
        get_default_brace_length(), length_min, length_max
    )

    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(bar_prompt)
        go.EnablePreSelect(True, True)
        go.SetCustomGeometryFilter(bar_or_tube_filter)
        list_opt_index = (
            -1 if len(names) == 1
            else go.AddOptionList("Pair", names, selected_index)
        )
        go.AddOptionDouble(length_option_name, length_value)

        result = go.Get()
        if result == Rhino.Input.GetResult.Cancel:
            return None, None, None

        if result == Rhino.Input.GetResult.Option:
            opt = go.Option()
            if opt is not None and opt.Index == list_opt_index:
                selected_index = int(opt.CurrentListOptionIndex)
            # length_value is updated in place by the OptionDouble.
            continue

        if result == Rhino.Input.GetResult.Object:
            picked_id = go.Object(0).ObjectId
            rs.UnselectObject(picked_id)
            bar_id = resolve_picked_to_bar_curve(picked_id)
            if bar_id is None:
                continue
            chosen = names[selected_index]
            set_default_pair_name(chosen)
            chosen_length = float(length_value.CurrentValue)
            set_default_brace_length(chosen_length)
            return bar_id, get_joint_pair(chosen), chosen_length

        return None, None, None


def main():
    _refresh_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSBarBrace")
    rs.UnselectAllObjects()

    le1_id, pair, brace_length = _pick_le1_with_pair_and_length(
        "Select first existing bar (Le1)", command_name="RSBarBrace"
    )
    if le1_id is None or pair is None:
        return
    target_distance = float(pair.contact_distance_mm)
    print(
        f"RSBarBrace: using pair '{pair.name}' "
        f"(contact distance {target_distance:.4f} mm), "
        f"brace length {float(brace_length):.2f} mm"
    )
    two_contact_bar.run(COMMAND, le1_id, (pair, pair), target_distance, brace_length)


if __name__ == "__main__":
    main()
