#! python 3
# venv: scaffolding_env
# r: numpy
"""RSDefineJointMate - Define a mate between two existing joint halves.

Workflow:

    1. Pick FEMALE block instance.
    2. Pick FEMALE bar axis line.
    3. Pick MALE block instance.
    4. Pick MALE bar axis line.
    5. Enter the mate name.

The script:
  - Looks up the female and male halves in the normalized registry by
    block-definition name.  Both halves MUST already exist
    (use RSDefineJointHalf first).
  - Computes `contact_distance_mm` as the common-perpendicular distance
    between the two infinite bar center lines (in millimetres).  The two
    bars must NOT be parallel.
  - Prints the derived value, then prompts Accept / Edit / Cancel.
  - Persists the mate to `scripts/core/joint_pairs.json` (does NOT touch
    half geometry; halves are immutable here).

Stacked picks: each pick auto-hides the previous selection, then everything
is restored at the end.
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import joint_pair as _joint_pair_module
from core import joint_pick_helpers as _picks_module


def _reload():
    global jp_mod, picks
    jp_mod = importlib.reload(_joint_pair_module)
    picks = importlib.reload(_picks_module)


_reload()

_DIALOG = "RSDefineJointMate"


def _ask_accept_edit(default_value_mm: float) -> float | None:
    answer = rs.GetString(
        f"contact_distance_mm = {default_value_mm:.4f} mm   (Accept / Edit / Cancel)",
        "Accept",
        ["Accept", "Edit", "Cancel"],
    )
    if answer is None:
        return None
    a = answer.strip().lower()
    if a.startswith("a"):
        return float(default_value_mm)
    if a.startswith("c"):
        return None
    if a.startswith("e"):
        edited = rs.GetReal("Enter contact_distance_mm", default_value_mm)
        if edited is None:
            return None
        return float(edited)
    return None


def main() -> None:
    _reload()
    rs.UnselectAllObjects()
    scale_to_mm = picks.doc_unit_scale_to_mm()
    print(f"{_DIALOG}: scale_to_mm = {scale_to_mm:g}")

    registry = jp_mod.load_joint_registry()
    print(
        f"{_DIALOG}: registry has {len(registry.halves)} halves, "
        f"{len(registry.mates)} mates."
    )

    selected: list = []

    female_block_id = picks.pick_block_instance(
        "Pick FEMALE block instance", _DIALOG
    )
    if female_block_id is None:
        print(f"{_DIALOG}: cancelled at female block pick.")
        return
    selected.append(female_block_id)

    with picks.temporarily_hidden(selected):
        female_bar_id = picks.pick_line("Pick FEMALE bar axis line", _DIALOG)
    if female_bar_id is None:
        print(f"{_DIALOG}: cancelled at female bar axis pick.")
        return
    selected.append(female_bar_id)

    with picks.temporarily_hidden(selected):
        male_block_id = picks.pick_block_instance("Pick MALE block instance", _DIALOG)
    if male_block_id is None:
        print(f"{_DIALOG}: cancelled at male block pick.")
        return
    selected.append(male_block_id)

    with picks.temporarily_hidden(selected):
        male_bar_id = picks.pick_line("Pick MALE bar axis line", _DIALOG)
    if male_bar_id is None:
        print(f"{_DIALOG}: cancelled at male bar axis pick.")
        return
    selected.append(male_bar_id)

    _, female_block_name = picks.block_instance_frame(female_block_id)
    _, male_block_name = picks.block_instance_frame(male_block_id)
    print(f"{_DIALOG}: female block = '{female_block_name}'")
    print(f"{_DIALOG}: male   block = '{male_block_name}'")

    if not female_block_name or not male_block_name:
        rs.MessageBox(
            "Both block instances must reference named block definitions.", 0, _DIALOG
        )
        return
    if female_block_name == male_block_name:
        rs.MessageBox("Female and male blocks must be different.", 0, _DIALOG)
        return

    if female_block_name not in registry.halves:
        rs.MessageBox(
            f"No registered joint half with block_name='{female_block_name}'. "
            "Run RSDefineJointHalf for this block first.",
            0,
            _DIALOG,
        )
        return
    if male_block_name not in registry.halves:
        rs.MessageBox(
            f"No registered joint half with block_name='{male_block_name}'. "
            "Run RSDefineJointHalf for this block first.",
            0,
            _DIALOG,
        )
        return

    female_half = registry.halves[female_block_name]
    male_half = registry.halves[male_block_name]
    if female_half.kind != "female":
        print(
            f"{_DIALOG}: WARNING: half '{female_block_name}' is registered as "
            f"kind={female_half.kind!r}, not 'female'."
        )
    if male_half.kind != "male":
        print(
            f"{_DIALOG}: WARNING: half '{male_block_name}' is registered as "
            f"kind={male_half.kind!r}, not 'male'."
        )

    # Mate name
    mate_name = rs.GetString("Mate name (required)")
    if mate_name is None:
        print(f"{_DIALOG}: cancelled at mate name input.")
        return
    mate_name = mate_name.strip()
    if not mate_name:
        rs.MessageBox("Mate name is required.", 0, _DIALOG)
        return

    # Compute contact distance from the two bar lines
    fbar_s_doc, fbar_e_doc = picks.line_endpoints(female_bar_id)
    mbar_s_doc, mbar_e_doc = picks.line_endpoints(male_bar_id)
    fbar_s_mm = picks.vec_to_mm(fbar_s_doc, scale_to_mm)
    fbar_d_mm = picks.vec_to_mm(fbar_e_doc - fbar_s_doc, scale_to_mm)
    mbar_s_mm = picks.vec_to_mm(mbar_s_doc, scale_to_mm)
    mbar_d_mm = picks.vec_to_mm(mbar_e_doc - mbar_s_doc, scale_to_mm)

    fbar_unit = fbar_d_mm / float(np.linalg.norm(fbar_d_mm))
    mbar_unit = mbar_d_mm / float(np.linalg.norm(mbar_d_mm))
    parallel_sin = float(np.linalg.norm(np.cross(fbar_unit, mbar_unit)))
    if parallel_sin < 1e-3:
        rs.MessageBox(
            "The two picked bars are (nearly) parallel. Contact distance is "
            "only well-defined for non-parallel bars; pick bars at a clear "
            "angle (orthogonal is ideal) and try again.",
            0,
            _DIALOG,
        )
        return

    from core.geometry import distance_infinite_lines

    auto_distance = abs(
        float(distance_infinite_lines(fbar_s_mm, fbar_d_mm, mbar_s_mm, mbar_d_mm))
    )
    print(f"{_DIALOG}: auto contact_distance_mm = {auto_distance:.4f}")

    contact_distance = _ask_accept_edit(auto_distance)
    if contact_distance is None:
        print(f"{_DIALOG}: cancelled at contact-distance confirmation.")
        return
    print(f"{_DIALOG}: final contact_distance_mm = {contact_distance:.4f}")

    pair = jp_mod.JointPairDef(
        name=mate_name,
        female=female_half,
        male=male_half,
        contact_distance_mm=contact_distance,
    )
    # save WITHOUT overwriting half geometry (halves are owned by the half
    # command, not by mates).
    jp_mod.save_joint_pair(pair, overwrite_halves=False)
    print(f"{_DIALOG}: saved mate '{mate_name}' "
          f"(female='{female_block_name}', male='{male_block_name}').")
    print(f"{_DIALOG}: done.")


if __name__ == "__main__":
    main()
