"""Read-only checker for an exported design problem (BarActions + ActionSchedule).

Checks the exported files against the rules agreed in
``docs/support_export_issues_from_monitor.md`` (issues D1-D9) and prints one
PASS / FAIL line per rule, with the first few offenders for every FAIL. It
reads plain JSON only (no compas, no Rhino), so it runs with any Python 3.

Usage:
    python tests/check_export_bundle.py <export folder> [--cells]

``--cells`` also scans the large ``RobotCell*.json`` files for body names
(slow: each file is a few hundred MB).

The file name does not start with ``test_`` on purpose: pytest does not pick
it up, because it needs an exported folder to look at.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys


# * ---- Names shared with the exporter (kept as plain strings on purpose) ----

# Wheel links of every Husky URDF (dual-arm Cindy, single-arm Alice/Belle).
WHEEL_LINKS = (
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
)
# Frozen-robot obstacle tools, one per robot.
OBSTACLE_TOOLS = ("ObstacleRobotCindy", "ObstacleRobotAlice", "ObstacleRobotBelle")
# Where a robot that is not in the scene stands (metres).
PARKED_POINT_M = (50.0, 50.0, 0.0)
# Prefix of the exported floor bodies (one per used walkable ground).
FLOOR_PREFIX = "ground_"
# The support robot's gripper tool.
SUPPORT_GRIPPER = "SupportGripper"

# Expected movement names (the part of the id after "_<kind>_M<n>_").
JOINTING_NORMAL = (
    "free_to_load", "manual_mount_bar", "tool_grasp_bar",
    "CDFM_transfer_to_approach", "tool_tighten_joint", "LM_insert",
)
JOINTING_GROUND = (
    "free_to_load", "manual_mount_bar", "tool_grasp_bar",
    "CDFM_transfer_to_approach", "LM_insert", "manual_fix_foundation",
)
RELEASE = ("tool_ungrasp_bar", "LM_retreat", "free_home")
# Hold movements in which the support gripper touches the held bar.
GRIPPER_CONTACT = {
    "__H": ("LM_to_grasp", "gripper_close"),
    "__HR": ("gripper_open", "LM_retreat"),
}

# Movement id = "<bar>_<kind>_M<n>_<name>".
_MOVEMENT_ID_RE = re.compile(r"^(?P<bar>.+?)_(?P<kind>J|R|H|HR)_M(?P<num>[0-9]+)_(?P<name>.+)$")


# * ---- Small readers ----


def movement_parts(movement_id: str) -> tuple:
    """Split a movement id into its parts.

    Args:
        movement_id (str): e.g. ``"B1_J_M4_LM_insert"``.

    Returns:
        tuple: ``(bar_id, kind, number, name)``, or ``(None, None, None, movement_id)``
        when the id does not follow the pattern.
    """
    match = _MOVEMENT_ID_RE.match(movement_id)
    if match is None:
        return None, None, None, movement_id
    return match["bar"], match["kind"], int(match["num"]), match["name"]


def movements_of(action: dict) -> list:
    """The movement data dicts of one action, in order.

    Args:
        action (dict): one loaded action file.

    Returns:
        list: each movement's ``data`` dict, with its ``dtype`` copied in.
    """
    out = []
    for movement in action["data"]["movements"]:
        data = dict(movement["data"])
        data["_dtype"] = movement.get("dtype", "")
        out.append(data)
    return out


def start_state(movement: dict) -> dict:
    """The movement's start state data, or an empty dict when there is none."""
    state = movement.get("start_state")
    return state["data"] if state else {}


def body_states(state: dict) -> dict:
    """``{body name: body state data}`` of one state."""
    return {name: value["data"] for name, value in state.get("rigid_body_states", {}).items()}


def tool_point(state: dict, tool_name: str):
    """The base point (metres) of one tool in a state, or None when absent."""
    tool = state.get("tool_states", {}).get(tool_name)
    if not tool or not tool["data"].get("frame"):
        return None
    return tuple(round(v, 4) for v in tool["data"]["frame"]["data"]["point"])


def base_point(state: dict):
    """The acting robot's base point (metres) in a state, or None."""
    frame = state.get("robot_base_frame")
    if not frame:
        return None
    return tuple(round(v, 4) for v in frame["data"]["point"])


def same_point(a, b, tol_m: float = 1e-3) -> bool:
    """True when two points (metres) are within ``tol_m`` of each other.

    Args:
        a: first point (tuple of 3 floats) or None.
        b: second point (tuple of 3 floats) or None.
        tol_m (float): allowed distance per coordinate, metres.

    Returns:
        bool: False when either point is None.
    """
    if a is None or b is None:
        return False
    return all(abs(x - y) <= tol_m for x, y in zip(a, b))


def load_bundle(root: str) -> tuple:
    """Load the schedule and every action file of one export folder.

    Args:
        root (str): the export folder (holds ``ActionSchedule.json`` and ``BarActions/``).

    Returns:
        tuple: ``(schedule dict, {file stem: action dict})``.
    """
    with open(os.path.join(root, "ActionSchedule.json")) as f:
        schedule = json.load(f)
    actions = {}
    actions_dir = os.path.join(root, "BarActions")
    for name in sorted(os.listdir(actions_dir)):
        if name.endswith(".json") and name.count(".") == 1:
            with open(os.path.join(actions_dir, name)) as f:
                actions[name[:-5]] = json.load(f)
    return schedule, actions


def ground_bars(actions: dict) -> set:
    """Bars whose jointing action carries ground joints on the arms.

    A ground joint body name ends with ``_ground``; on a ground bar it rides an
    arm flange during the insert.

    Args:
        actions (dict): ``{file stem: action}``.

    Returns:
        set: bar ids of the ground bars.
    """
    out = set()
    for stem, action in actions.items():
        if not stem.endswith("__J"):
            continue
        for movement in movements_of(action):
            for name, body in body_states(start_state(movement)).items():
                if name.endswith("_ground") and body.get("attached_to_link"):
                    out.add(stem[:-3])
    return out


# * ---- The rules ----


class Report:
    """Collects PASS / FAIL lines."""

    def __init__(self):
        self.failed = 0

    def rule(self, label: str, offenders: list) -> None:
        """Print one rule's result.

        Args:
            label (str): what the rule checks, in plain words.
            offenders (list): one string per violation; empty means PASS.
        """
        if not offenders:
            print(f"PASS  {label}")
            return
        self.failed += 1
        print(f"FAIL  {label}  ({len(offenders)} problem(s))")
        for line in offenders[:6]:
            print(f"        - {line}")
        if len(offenders) > 6:
            print(f"        ... and {len(offenders) - 6} more")


def check_movement_lists(report: Report, actions: dict) -> None:
    """D7 + D9: the release has no untighten; ground bars have no tighten."""
    grounds = ground_bars(actions)
    no_untighten, release_shape, ids_in_order = [], [], []
    ground_shape, normal_shape = [], []
    for stem, action in actions.items():
        movements = movements_of(action)
        names = []
        for index, movement in enumerate(movements):
            _bar, _kind, number, name = movement_parts(movement["movement_id"])
            names.append(name)
            if number != index:
                ids_in_order.append(f"{movement['movement_id']} is movement {index}")
            if movement.get("tool_action") == "untighten":
                no_untighten.append(movement["movement_id"])
        bar_id = stem.split("__")[0]
        if stem.endswith("__R") and tuple(names) != RELEASE:
            release_shape.append(f"{stem}: {names}")
        if stem.endswith("__J"):
            insert = next((m for m in movements if m["movement_id"].endswith("_LM_insert")), None)
            ends_on = (insert or {}).get("notes", {}).get("ends_on")
            if bar_id in grounds:
                if tuple(names) != JOINTING_GROUND or ends_on != "target_reached" \
                        or "ManualMovement" not in movements[-1]["_dtype"]:
                    ground_shape.append(f"{stem}: {names}, insert ends_on={ends_on}")
            elif tuple(names) != JOINTING_NORMAL or ends_on != "tool_stall_signal":
                normal_shape.append(f"{stem}: {names}, insert ends_on={ends_on}")
    report.rule("D7  no movement runs the jointing screws backwards (untighten)", no_untighten)
    report.rule("D7  every release action is ungrasp, retreat, home", release_shape)
    report.rule(f"D9  ground bars {sorted(grounds)}: grasp, transfer, insert (ends at target), operator fix",
                ground_shape)
    report.rule("D9  other bars keep tighten + insert (ends on the screw stall)", normal_shape)
    report.rule("D7/D9 movement numbers follow the movement order", ids_in_order)


def check_names_and_floor(report: Report, schedule: dict, actions: dict) -> None:
    """D2 + D6: one naming scheme; the used walkable grounds are floor bodies."""
    used = sorted({gid for a in actions.values() for gid in a["data"].get("walkable_ground_ids", [])})
    floors = {f"{FLOOR_PREFIX}{gid}" for gid in used}
    grounds = ground_bars(actions)
    env_names, missing_floor, floor_contacts, extra_floor, ground_contacts = [], [], [], [], []
    for stem, action in actions.items():
        for movement in movements_of(action):
            bodies = body_states(start_state(movement))
            if not bodies:
                continue
            mid = movement["movement_id"]
            for name in bodies:
                if name.startswith(("env_bar_", "env_joint_")):
                    env_names.append(f"{mid}: {name}")
                    break
            for name in bodies:
                if name.startswith(FLOOR_PREFIX) and name not in floors:
                    extra_floor.append(f"{mid}: {name}")
            for floor in sorted(floors):
                body = bodies.get(floor)
                if body is None or body.get("is_hidden"):
                    missing_floor.append(f"{mid}: {floor} missing or hidden")
                    continue
                tools_here = set(start_state(movement).get("tool_states", {})) & set(OBSTACLE_TOOLS)
                lacking = (set(WHEEL_LINKS) - set(body.get("touch_links") or [])) | (
                    tools_here - set(body.get("touch_bodies") or []))
                if lacking:
                    floor_contacts.append(f"{mid}: {floor} does not allow {sorted(lacking)}")
            # Ground joints of a ground bar touch the floor only in the insert and the fix step.
            bar_id, kind, _n, name = movement_parts(mid)
            if kind == "J" and bar_id in grounds:
                should = name in ("LM_insert", "manual_fix_foundation")
                for body_name, body in bodies.items():
                    if not (body_name.endswith("_ground") and body.get("attached_to_link")):
                        continue
                    has = bool(floors & set(body.get("touch_bodies") or []))
                    if has != should:
                        ground_contacts.append(f"{mid}: {body_name} floor contact={has}, expected {should}")
    report.rule("D2  no body uses the old env_bar_/env_joint_ names", env_names)
    report.rule(f"D6  every state has the used floor(s) {sorted(floors)}, shown", missing_floor)
    report.rule("D6  every floor lets the wheels and the frozen robots touch it", floor_contacts)
    report.rule("D6  no floor for unused walkable grounds", extra_floor)
    report.rule("D6  ground joints may touch the floor only in the insert and the fix step", ground_contacts)


def check_holds(report: Report, schedule: dict, actions: dict) -> None:
    """D5 + D1: hold scenes show the held bar; the release shows its own holder."""
    holds = {h["bar_id"]: h for h in schedule.get("holds", [])}
    shown, gripper, own_holder, same_step = [], [], [], []
    order = [e for e in schedule["schedule"] if e["type"] == "BarHoldingReleaseAction"]
    for bar_id, hold in holds.items():
        robot_tool = f"ObstacleRobot{hold['robot']}"
        bar_key = f"bar_{bar_id}"
        hold_base = None
        for suffix in ("__H", "__HR"):
            action = actions.get(f"{bar_id}{suffix}")
            if action is None:
                continue
            for movement in movements_of(action):
                state = start_state(movement)
                bodies = body_states(state)
                body = bodies.get(bar_key)
                mid = movement["movement_id"]
                _b, _k, _n, name = movement_parts(mid)
                # The holder's base, read from its own hold file (for the D1 rule below).
                if suffix == "__H" and name == "gripper_close":
                    hold_base = base_point(state)
                if body is None or body.get("is_hidden"):
                    shown.append(f"{mid}: {bar_key} missing or hidden")
                    continue
                should = name in GRIPPER_CONTACT[suffix]
                if (SUPPORT_GRIPPER in (body.get("touch_bodies") or [])) != should:
                    gripper.append(f"{mid}: gripper contact expected {should}")
        # D1: the bar's own holder stands at its hold pose in the release.
        release = actions.get(f"{bar_id}__R")
        if release is not None and hold_base is not None:
            for movement in movements_of(release):
                state = start_state(movement)
                point = tool_point(state, robot_tool)
                allowed = robot_tool in (body_states(state).get(bar_key, {}).get("touch_bodies") or [])
                if not same_point(point, hold_base) or not allowed:
                    own_holder.append(f"{movement['movement_id']}: {robot_tool} at {point}, allowed={allowed}")
    # Same-step releases: a hold released earlier in the same step is gone.
    for index, entry in enumerate(order):
        release_after = holds[entry["bar_id"]]["release_after_bar_id"]
        earlier = [e for e in order[:index] if holds[e["bar_id"]]["release_after_bar_id"] == release_after]
        action = actions.get(f"{entry['bar_id']}__HR")
        if action is None:
            continue
        for gone in earlier:
            tool = f"ObstacleRobot{gone['robot']}"
            for movement in movements_of(action):
                point = tool_point(start_state(movement), tool)
                if point is not None and not same_point(point, PARKED_POINT_M):
                    same_step.append(f"{movement['movement_id']}: {tool} still at {point}")
    report.rule("D5  hold and hold-release scenes show the held bar", shown)
    report.rule("D5  the gripper may touch the held bar only from the linear approach on", gripper)
    report.rule("D1  each held bar's release shows its own holder at the hold pose", own_holder)
    report.rule("D1  a hold released earlier in the same step is gone from later releases", same_step)


def check_sequence(report: Report, root: str, schedule: dict, actions: dict) -> None:
    """D4 + D8: one bar sequence everywhere; every scheduled file exists."""
    expected = schedule["assembly_seq"]
    seq, missing = [], []
    for stem, action in actions.items():
        if action["data"].get("assembly_seq") != expected:
            seq.append(f"{stem}: {len(action['data'].get('assembly_seq', []))} bars vs {len(expected)}")
    for entry in schedule["schedule"]:
        if not os.path.isfile(os.path.join(root, entry["file"])):
            missing.append(entry["file"])
    report.rule("D4  every action's bar sequence equals the schedule's (no fake bars)", seq)
    report.rule("D8  every file the schedule names exists", missing)


def check_cells(report: Report, root: str, floors: set) -> None:
    """D2 + D6 in the cell files: canonical names, floor bodies present."""
    problems = []
    for name in sorted(os.listdir(root)):
        if not (name.startswith("RobotCell") and name.endswith(".json")):
            continue
        with open(os.path.join(root, name), encoding="utf-8") as f:
            text = f.read()
        if '"env_bar_' in text or '"env_joint_' in text:
            problems.append(f"{name}: still has env_bar_/env_joint_ bodies")
        for floor in sorted(floors):
            if f'"{floor}"' not in text:
                problems.append(f"{name}: no {floor} body")
    report.rule("D2/D6 cell files use canonical names and carry the floor", problems)


def main() -> int:
    """Run every rule on one export folder.

    Returns:
        int: 0 when every rule passes, 1 otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("root", help="export folder (ActionSchedule.json + BarActions/)")
    parser.add_argument("--cells", action="store_true", help="also scan the RobotCell*.json files")
    args = parser.parse_args()

    schedule, actions = load_bundle(args.root)
    print(f"{args.root}: {len(actions)} action file(s), {len(schedule['schedule'])} schedule entries")
    report = Report()
    check_movement_lists(report, actions)
    check_names_and_floor(report, schedule, actions)
    check_holds(report, schedule, actions)
    check_sequence(report, args.root, schedule, actions)
    if args.cells:
        used = {gid for a in actions.values() for gid in a["data"].get("walkable_ground_ids", [])}
        check_cells(report, args.root, {f"{FLOOR_PREFIX}{gid}" for gid in used})
    print(f"\n{report.failed} rule(s) failed.")
    return 1 if report.failed else 0


if __name__ == "__main__":
    sys.exit(main())
