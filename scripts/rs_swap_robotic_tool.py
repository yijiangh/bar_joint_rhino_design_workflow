#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSSwapRoboticTool - make a tool candidate's L/R pair the ACTIVE pair.

The command lists every tool registered in ``robotic_tools.json`` in a pop-up
selection box; click either the L or the R member and the name suffix resolves
the pair partner, and then the whole document + kinematics config switches to
that pair:

    1. registry `active` entry updated (scripts/core/robotic_tools.json)
    2. EVERY placed tool instance is re-placed with the side-matching new
       tool (each joint keeps the arm side it already had -- male J* and
       ground G* joints alike)
    3. both tool block definitions are imported/force-refreshed from their
       asset/<tool_name>.3dm files, so no candidate source block is needed
    4. bars carrying solved IK keyframes are listed in a WARNING (they were
       solved against the OLD tools; re-run RSIKKeyframe per bar)
    5. the IK preview mesh cache is dropped so the next RSIKKeyframe redraws
       the new tool (its baked tool meshes are not auto-refreshed on a swap)
    6. if PyBullet is running you are offered an immediate collision-cell
       rebuild; otherwise run RSPBStart + RSRebuildRobotCell before any IK

Nothing is changed until the pre-flight checks pass: both sides of the pair
must be registered, and each side needs its .3dm asset AND collision OBJ
(re-run RSDefineRoboticTool in AssemblyTool mode for a side that fails).
"""

from __future__ import annotations

import importlib
import os
import sys

import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import ik_viz as _ik_viz_module
from core import robot_cell as _robot_cell_module
from core import robotic_tool as _robotic_tool_module
from core import rhino_tool_place as _tool_place_module
from core.rhino_bar_registry import apply_build_stage_visibility, get_bar_seq_map


def _preflight_pair_assets(pair: dict) -> None:
    """Fail BEFORE any mutation if either side's files are missing.

    Args:
        pair (dict): ``{"left": RoboticToolDef, "right": RoboticToolDef}``.

    Raises:
        RuntimeError: listing every missing .3dm / collision OBJ.
    """
    problems = []
    for side in ("left", "right"):
        tool = pair[side]
        if not tool.asset_filename or not os.path.isfile(tool.asset_path()):
            problems.append(
                f"  - {tool.name}: block asset missing "
                f"({tool.asset_path() or '<unset>'})"
            )
        if not tool.collision_filename or not os.path.isfile(tool.collision_path()):
            problems.append(
                f"  - {tool.name}: collision OBJ missing "
                f"({tool.collision_path() or '<unset>'})"
            )
    if problems:
        raise RuntimeError(
            "Pre-flight failed; nothing was changed. Re-run RSDefineRoboticTool "
            "(AssemblyTool mode) for the listed tool(s):\n" + "\n".join(problems)
        )


def _warn_stale_keyframes(config) -> None:
    """List bars whose solved IK keyframes were computed with the OLD tools."""
    stale = []
    for bar_id, (oid, seq) in get_bar_seq_map().items():
        has_keyframe = any(
            rs.GetUserText(oid, key)
            for key in (
                config.KEY_ASSEMBLY_BASE_FRAME,
                config.KEY_ASSEMBLY_IK_APPROACH,
                config.KEY_ASSEMBLY_IK_ASSEMBLED,
            )
        )
        if has_keyframe:
            stale.append((seq, bar_id))
    if not stale:
        return
    bar_list = ", ".join(bar_id for _seq, bar_id in sorted(stale))
    print(
        f"  WARNING: {len(stale)} bar(s) carry IK keyframes solved with the "
        f"OLD tools: [{bar_list}] -- re-run RSIKKeyframe per bar "
        "(RSClearIKKeyframe to erase), then re-export."
    )


def _offer_cell_rebuild(robot_cell) -> None:
    """Rebuild the collision cell now (if PyBullet runs) or print instructions."""
    if not robot_cell.is_pb_running():
        print(
            "  PyBullet not running -- click RSPBStart, then RSRebuildRobotCell, "
            "before any IK command (the staleness prompt will remind you)."
        )
        return
    answer = rs.GetString(
        "Rebuild the collision cell with the new tools now?",
        "Rebuild",
        ["Rebuild", "Later"],
    )
    if answer is None or answer.strip().lower().startswith("l"):
        print(
            "  cell NOT rebuilt -- click RSRebuildRobotCell before any IK "
            "command (the staleness prompt will remind you)."
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()
    robot_cell.rebuild_assembly_cell(rcell, planner)
    print(
        f"  collision cell rebuilt with tools: {sorted(rcell.tool_models.keys())}"
    )


def main() -> None:
    robot_cell = importlib.reload(_robot_cell_module)
    robotic_tool = importlib.reload(_robotic_tool_module)
    tool_place = importlib.reload(_tool_place_module)
    config = importlib.reload(_config_module)
    ik_viz = importlib.reload(_ik_viz_module)

    tools = robotic_tool.load_robotic_tools()
    if not tools:
        rs.MessageBox(
            "No robotic tools are registered. Define a left/right pair first "
            "with RSDefineRoboticTool (AssemblyTool mode).",
            0,
            "RSSwapRoboticTool",
        )
        return

    available_names = sorted(tools)
    print(f"RSSwapRoboticTool: available robotic tools ({len(available_names)}):")
    for available_name in available_names:
        print(f"  - {available_name}")

    # Pre-highlight the currently-active tool (if it resolves) so the current
    # pick is obvious in the list; fall back to no default otherwise.
    try:
        default_name = robotic_tool.get_active_pair_names().get("left")
    except (RuntimeError, ValueError):
        default_name = None

    # Pop-up list: click a tool name to activate its pair (no typing). Picking
    # either the L or R member resolves the full pair downstream.
    name = rs.ListBox(
        available_names,
        "Select the robotic tool to activate (either the L or R member of a pair)",
        "RSSwapRoboticTool",
        default_name,
    )
    if name is None:
        print("RSSwapRoboticTool: Cancelled.")
        return
    name = name.strip()

    try:
        if name not in tools:
            raise ValueError(
                f"Tool {name!r} is not registered. Available tools: "
                + ", ".join(available_names)
            )
        # Resolve either typed member to its full L/R pair.
        pair = robotic_tool.resolve_pair_for_tool(name, tools)
        # Everything on disk present for BOTH sides before we mutate anything.
        _preflight_pair_assets(pair)
    except (RuntimeError, ValueError) as exc:
        rs.MessageBox(str(exc), 0, "RSSwapRoboticTool")
        return

    print(
        f"RSSwapRoboticTool: activating pair {pair['left'].name} (left) + "
        f"{pair['right'].name} (right)"
    )

    # 1. Persist the headless truth FIRST: even if a later Rhino step fails,
    #    re-running this button reconciles the document with the registry.
    robotic_tool.set_active_pair(pair["left"].name, pair["right"].name)
    print("  registry 'active' entry updated (robotic_tools.json).")

    # 2. Re-place every placed tool instance (validate-first; aborts before
    #    deleting anything if the document has broken tool instances).
    try:
        summary = tool_place.replace_all_tool_instances(pair)
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSSwapRoboticTool")
        return
    print(
        f"  re-placed {summary['replaced']} tool instance(s): "
        f"{summary['left']} left, {summary['right']} right."
    )

    # 3. Doc-default tool: keep the previous default's SIDE, on the new pair.
    prev_default = tool_place.get_default_tool_name()
    prev_side = robotic_tool.arm_side_from_tool_name(prev_default or "") or "left"
    tool_place.set_default_tool_name(pair[prev_side].name)
    print(f"  default robotic tool for this document set to: {pair[prev_side].name}")

    # 4. Stale solved keyframes: warn only (never auto-cleared).
    _warn_stale_keyframes(config)

    # 5. Drop the IK preview cache. The ik_viz bundle bakes the robot + TOOL
    #    meshes once, keyed by the robot-cell object, and never re-syncs the tools
    #    when the active pair changes (update_state only diffs rigid bodies). So
    #    without this, the next RSIKKeyframe preview (e.g. the toggle-pose phase)
    #    would still draw the OLD tool. Dropping the bundle forces a fresh bake of
    #    the new tool on the next IK run; safe even if no IK preview ran this session.
    ik_viz.discard_cache()
    print("  IK preview cache dropped (next RSIKKeyframe re-bakes the new tool).")

    # 6. Collision cell: rebuild now or tell the user exactly what to click.
    _offer_cell_rebuild(robot_cell)

    # 7. Re-assert the build-stage filter -- MANDATORY here, and the last
    #    visibility-touching step on purpose.  Step 2 above deleted and
    #    re-inserted every tool block, and a new Rhino object is always born
    #    visible, so without this every tool on every unbuilt bar reappears.
    #    This command is also the one entry point that does NOT call
    #    repair_on_entry, so nothing else in the run would put them back.
    #    Outside replace_all_tool_instances, not inside it, so its own
    #    suspend_redraw batching is left intact.
    apply_build_stage_visibility(caller="RSSwapRoboticTool")


if __name__ == "__main__":
    main()
