"""Check that the old plugin and the new plugin describe the same assembly, from two exports of one .3dm file.

    python tests/equivalence/export_equivalence.py <old export> <new process folder> [--json out.json]
                                                   [--no-worlds] [--no-collisions] [--no-processes]

- `old export`: the old plugin's RSExportAllBarActions folder (RobotCell*.json, BarActions/, ActionSchedule.json,
  WalkableGround.json), written by hs/mocap-experiment @ 069275f.
- `new process folder`: the new plugin's RSExportProcess folder (schema 2.1), from the same file.

Test 1, planner worlds: the old export's cells and movement start states are the old plugin's planning worlds. The
new plugin's are the process's scenes (`Process.scene_at`) as compas_fab objects (`CompasFabCell`), which is what
`CompasFabMirror` plans on. Cells, states, flange poses and colliding pairs are compared with `compas_compare`.

Test 2, processes: the old export converted by the core (`legacy.convert_in_memory`) against the new process as read
(`process.read`): content hashes, then every field by id, then each movement's scene (world poses).

Old movements map onto new ones as the converter folds them (`legacy.export.actions`): a tighten runs into the insert
after it, an untighten into the ungrasp after it. The mapping is checked against the converted process's counts.

Every difference is counted under a kind; `KNOWN` names the explained ones. Each step's time is in the report.
! Loads three cells of ~360 MB: ~4 GB of memory, a few minutes.
"""

from __future__ import annotations

import dataclasses
import json
import re
import sys
import time
from argparse import ArgumentParser
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

REPO = Path(__file__).resolve().parents[2]
for _path in (Path(__file__).resolve().parent, REPO / "external" / "scaffolding_assembly_core",
              REPO / "external" / "compas_fab" / "src"):
    sys.path.insert(0, str(_path))
import compas_compare as cc  # noqa: E402

from scaffolding_assembly_core.process import read  # noqa: E402
from scaffolding_assembly_core.process.write import process_hashes  # noqa: E402
import trimesh  # noqa: E402
from compas.geometry import Transformation  # noqa: E402
from scaffolding_assembly_core.geometry import Geometry, Pose, TriMesh, compose, invert, shape_mesh  # noqa: E402
from scaffolding_assembly_core.legacy.conversion import SERIALS, robot_files  # noqa: E402
from scaffolding_assembly_core.legacy.export import ACTION_CODES, ACTION_KINDS, from_export, load_export  # noqa: E402
from scaffolding_assembly_core.legacy.timing import Stopwatch  # noqa: E402
from scaffolding_assembly_core.mirrors.cell import CompasFabCell  # noqa: E402

OLD, CORE = "old export", "core"
PAIR = (OLD, CORE)
#: Tolerance of the process diff: positions (m), angles (rad), other numbers.
#: ? Angles: the old export stores frames as two axes, the process as a quaternion rounded to 12 decimals.
POSE_TOL, ANGLE_TOL, VALUE_TOL = 1e-6, 1e-5, 1e-9
#: How far apart two shapes placed in the world may be (m), and a body's motion from its process placement (m, rad).
SHAPE_TOL, MOTION_TOL, MOTION_ANGLE_TOL = 1e-4, 1e-4, 1e-5

#: Difference kinds that are explained, beyond `compas_compare.KNOWN`. Any other kind fails the run.
KNOWN: Dict[str, str] = {
    # * Representation: the same world, written differently. "body placement in the world" checks what they stand for.
    "body state: frame differs": "body frames are a convention (a bar's sits at its end in the old export, at its "
                                 "middle in the process); the placement check compares where the shapes are",
    "body state: hidden body frame differs": "a hidden body's frame is never used; frames are a convention anyway",
    "body state: grasp differs": "grasps are relative to the body frame, a convention; see the placement check",
    "body state: attached body has a frame": "the core also writes the world frame of an attached body; compas_fab "
                                             "ignores it",
    "body state: attachment differs": "a held bar's halves ride on the first holder's flange in the core, on their "
                                      "own flange in the old export; both arms hold the bar, see the placement check",
    "bars: prism against cylinder": "bars are 12-sided prisms in the old export and cylinders (finer prisms) in the "
                                    "process, 0.34 mm apart at most",
    "geometry: visual differs": "robot visual meshes: the old export stores reduced copies of the source files the "
                                "core loads; bars as above. Collision checks never use visual shapes",
    "geometry: collision differs": "bars only (checked): prism against cylinder, and the frame convention",
    "other robot: welded tool link named differently": "<flange>_obstacle_tool against <flange>_tool; same geometry",
    "tool state: group name differs, same flange": "the support gripper on `manipulator` (old) or the base-rooted "
                                                   "`base_arm_manipulator` (core); both end at the same flange",
    "tool state: attached tool has a frame": "compas_fab ignores a frame on an attached tool",
    "tool state: attachment_frame None vs identity": "compas_fab reads an empty attachment frame as identity",
    # * Contacts: written by hand in the old plugin, derived in the core. The collision check compares the outcome.
    "tool state: touch_links differ": "the core lets a tool touch the last links of its arm (`urdf.TOOL_TOUCHES_ARM_LINKS`)",
    "body state: touch_bodies differ": "allowed contacts are derived in the core (format 8.4) and listed on both "
                                       "bodies; the old plugin wrote them per movement. See the colliding pairs",
    "body state: touch_bodies mirrored": "the same contact listed on the other body",
    "body state: touch_links differ": "as touch_bodies: derived contacts with robot links",
    "body state: hidden body touches differ": "a hidden body's contacts are never used",
    # * The new plugin describes more, or decides differently on purpose.
    "floor: only in core": "the old cells have no floor; the process holds the walkable ground as a body",
    "bodies differ": "obstacles: the old plugin put them in Cindy's cell only; the process gives every robot's cell "
                     "all of them",
    "body keys differ": "as above: obstacles missing from the old support cells",
    "movement only in core": "HR ends with `leave` (free, away from the structure): the support robot has to move "
                             "away from the bar; the old export has no such movement (confirmed 2026-10-13)",
    "tool frame differs": "the support gripper's TCP: at the flange in the old export, between the fingers on the "
                          "bar axis in the process, which is the proper value (confirmed 2026-10-13)",
    "configuration None in one": "the old export leaves a start configuration open where the planner fills it; the "
                                 "process carries the last known joints forward (every state reads alone)",
    "flanges differ": "as above: no configuration on one side, so no forward kinematics to compare",
    "body state: hidden differs": "the old export shows a bar and its halves before the operator mounts them; the "
                                  "process has them absent until the mount",
}
cc.KNOWN.update(KNOWN)


# --- --- --- --- --- MOVEMENT MAP --- --- --- --- ---

def movement_map(export) -> Tuple[Dict[Tuple[str, str], Tuple[str, int]], Dict[str, List[str]]]:
    """Old (action id, movement id) -> (new action id, index of the new movement it starts), as the converter folds.

    Returns:
        tuple[dict, dict]: The map, and per reason the old movements that start no new movement ("insert run into its
            tighten", "untighten folded into the ungrasp").
    """
    mapping, dropped, seen = {}, defaultdict(list), set()
    for entry, action in export.actions:
        code = ACTION_CODES[ACTION_KINDS[type(action).__name__]]
        new_action = f"{action.active_bar_id}_{code}"
        while new_action in seen:  # as `legacy.export._unique`
            new_action += "_2"
        seen.add(new_action)
        index, position, moves = 0, 0, list(action.movements)
        while position < len(moves):
            move = moves[position]
            tool_action = getattr(move, "tool_action", None)
            if tool_action == "untighten":
                dropped["untighten folded into the ungrasp"].append(move.movement_id)
                position += 1
                continue
            merged = (tool_action == "tighten" and bool(getattr(move, "overlaps_next", False))
                      and position + 1 < len(moves))
            mapping[(entry["action_id"], move.movement_id)] = (new_action, index)
            if merged:
                dropped["insert run into its tighten"].append(moves[position + 1].movement_id)
            position += 2 if merged else 1
            index += 1
    return mapping, dropped


# --- --- --- --- --- TEST 1: PLANNER WORLDS --- --- --- --- ---

def load_old(export, process) -> cc.Loaded:
    """The old plugin's planning objects, keyed by the new process's movement ids."""
    robots = {entry["robot_id"]: f"robots/{name.lower()}" for name, entry in export.schedule["robots"].items()}
    cells = {robots[name]: cell for name, cell in export.cells.items()}
    mapping, _ = movement_map(export)
    states = {}
    for entry, action in export.actions:
        for movement in action.movements:
            target = mapping.get((entry["action_id"], movement.movement_id))
            if target is None or target[0] not in process.actions:
                continue
            new_action, index = target
            if index < len(process.actions[new_action].movements):
                key = (new_action, process.actions[new_action].movements[index].id)
                states[key] = (f"robots/{entry['robot'].lower()}", movement.start_state)
    loaded = cc.Loaded(OLD, cells, states)
    cc.name_maps(loaded, process)
    return loaded


def load_core(process, report: cc.Report, watch: Stopwatch) -> cc.Loaded:
    """The new plugin's planning objects: per acting robot one `CompasFabCell`, per movement the state of its scene.

    ? Not strict: where the process leaves the acting robot's base or joints open, the cell parks it; the old export
      then has no configuration either, or a placeholder.
    """
    cells, states, keys, builders = {}, {}, {}, {}
    rebuilds = defaultdict(int)
    for action, movement in process.movements():
        robot = action.robot
        builder = builders.get(robot) or builders.setdefault(robot, CompasFabCell(robot, visual=True))
        scene = process.scene_at(movement)
        if builder.update(scene) is not None:
            rebuilds[robot] += 1
        states[(action.id, movement.id)] = (robot, builder.state(scene, strict=False))
        cells[robot] = builder.cell
    watch.lap("core: cells and states from scenes")
    for robot, cell in cells.items():
        keys[robot] = {key: cc.GROUND if key.startswith("ground/") else key
                       for key in [*cell.tool_models, *cell.rigid_body_models]}
    report.notes.append(f"core: cells built per robot: {dict(rebuilds)} (1 each: no rebuild between movements)")
    return cc.Loaded(CORE, cells, states, keys)


def compare_worlds(report: cc.Report, old: cc.Loaded, core: cc.Loaded, process, collisions: bool,
                   watch: Stopwatch, log, cells: bool = True) -> None:
    """Cells, states, flange poses and colliding pairs of the two planning worlds."""
    for robot in sorted(set(old.cells) & set(core.cells)) if cells else ():
        log(f"comparing cells: {robot}")
        cc.compare_cells(report, PAIR, old, core, robot)
    if set(old.cells) != set(core.cells):
        report.category(PAIR, "acting robots").add("robots", [("acting robots differ",
                                                             f"{sorted(set(old.cells) ^ set(core.cells))}")])
    watch.lap("compare cells")
    common = sorted(set(old.states) & set(core.states))
    only = report.category(PAIR, "movements matched")
    only.compared, only.equal = len(set(old.states) | set(core.states)), len(common)
    for label, keys in ((OLD, set(old.states) - set(core.states)), (CORE, set(core.states) - set(old.states))):
        for key in sorted(keys):
            only.counts[f"movement only in {label}"] += 1
            only.kinds[f"movement only in {label}"].append(key[1])
    log(f"comparing states: {len(common)} movements")
    for key in common:
        cc.compare_states(report, PAIR, old, core, key)
        cc.compare_kinematics(report, PAIR, old, core, key)
    _only_bars(report)
    watch.lap("compare states")
    compare_placement(report, old, core, common)
    watch.lap("compare body placement")
    if not collisions:
        return
    found = {}
    for robot in sorted(old.cells):
        keys = [key for key in common if old.states[key][0] == robot
                and old.states[key][1].robot_configuration is not None
                and not process.scene_at(_movement(process, key)).robots[robot].unmeasured]
        log(f"collisions: old export {robot} ({len(keys)} states)")
        found.update(cc.collision_pairs(old, robot, keys))
    watch.lap("collisions: old export (compas_fab check_collision)")
    log(f"collisions: core mirror ({len(found)} states)")
    mirror, allowed = cc.mirror_collisions(process, sorted(found))
    watch.lap("collisions: core (CompasFabMirror)")
    cc.compare_mirror(report, found, mirror, allowed, process, PAIR)
    report.notes.append(f"collisions: {len(found)} of {len(common)} movements checked (both sides give the acting "
                        f"robot's joints); old export {sum(map(len, found.values()))} colliding pairs, core "
                        f"{sum(map(len, mirror.values()))}")


def body_world(cell, state, body_state, cache: dict) -> Optional[np.ndarray]:
    """A body's world pose (4x4) in a compas_fab state: its frame, or its link's pose times the attachment frame."""
    if not body_state.attached_to_link:
        return cc._matrix(body_state.frame) if body_state.frame is not None else None
    if state.robot_configuration is None:
        return None
    if id(state) not in cache:
        model = cell.robot_model
        cache[id(state)] = (model.compute_transformations(cc._joint_map(state.robot_configuration)),
                            np.asarray(Transformation.from_frame(state.robot_base_frame).matrix))
    transformations, base = cache[id(state)]
    joint = cell.robot_model.get_link_by_name(body_state.attached_to_link).parent_joint
    link = base @ np.asarray(transformations[joint.name].matrix) @ cc._matrix(joint.current_origin)
    return link @ cc._matrix(body_state.attachment_frame)


def _placed(body, matrix: np.ndarray):
    parts = cc.body_parts(body, "collision")
    if not parts:
        return None
    vertices, faces, offset = [], [], 0
    for part in parts:
        vertices.append(part.vertices @ matrix[:3, :3].T + matrix[:3, 3])
        faces.append(part.faces + offset)
        offset += len(part.vertices)
    return trimesh.Trimesh(np.vstack(vertices), np.vstack(faces), process=False)


def compare_placement(report: cc.Report, old: cc.Loaded, core: cc.Loaded, keys) -> None:
    """Where every visible body is in the world, whatever its frame convention.

    Per body, the transform from the old frame to the core's is a constant if both sides move the body alike: the
    first state gives it (and there the two collision shapes, placed in the world, must coincide), every other state
    is compared with it.
    """
    category = report.category(PAIR, "body placement in the world")
    shapes = report.category(PAIR, "body shape in the world")
    reference: Dict[Tuple[str, str], np.ndarray] = {}
    cache: dict = {}
    for key in keys:
        robot, sa = old.states[key]
        _, sb = core.states[key]
        cell_a, cell_b = old.cells[robot], core.cells[robot]
        names_b = {v: k for k, v in core.keys[robot].items()}
        problems = []
        for name_a, state_a in sa.rigid_body_states.items():
            body = old.keys[robot][name_a]
            state_b = sb.rigid_body_states.get(names_b.get(body, ""))
            if body == cc.GROUND or state_b is None or state_a.is_hidden or state_b.is_hidden:
                continue
            wa, wb = body_world(cell_a, sa, state_a, cache), body_world(cell_b, sb, state_b, cache)
            if wa is None or wb is None:
                continue
            offset = np.linalg.inv(wa) @ wb
            first = reference.get((robot, body))
            if first is None:
                reference[(robot, body)] = offset
                x = _placed(cell_a.rigid_body_models[name_a], wa)
                y = _placed(cell_b.rigid_body_models[names_b[body]], wb)
                gap = surface_gap(x, y) if x is not None and y is not None else 0.0
                shapes.deviation("surface m", gap)
                kind = [] if gap <= SHAPE_TOL else \
                    [("bars: prism against cylinder" if body.startswith("bars/") and gap < 5e-4
                      else "body shape placed differently", f"surfaces up to {gap * 1000:.3g} mm apart ({key[1]})")]
                shapes.add(f"{robot}:{body}", kind)
                continue
            change = np.linalg.inv(first) @ offset
            distance = float(np.linalg.norm(change[:3, 3]))
            angle = float(np.arccos(np.clip((np.trace(change[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
            category.deviation("m", distance)
            category.deviation("rad", angle)
            if distance > MOTION_TOL or angle > MOTION_ANGLE_TOL:
                problems.append(("body placed differently", f"{body}: {distance * 1000:.4g} mm, "
                                                            f"{np.degrees(angle):.4g} deg"))
        category.add(key[1], problems)


def _only_bars(report: cc.Report) -> None:
    """`geometry: collision differs` is explained for bars only: any other body is its own, unexplained kind."""
    category = report.category(PAIR, "body collision")
    others = [line for line in category.kinds.get("geometry: collision differs", []) if ":bars/" not in line]
    for line in others:
        category.counts["collision shape of a body that is not a bar differs"] += 1
        category.kinds["collision shape of a body that is not a bar differs"].append(line)


def _movement(process, key):
    return next(m for m in process.actions[key[0]].movements if m.id == key[1])


# --- --- --- --- --- TEST 2: PROCESSS --- --- --- --- ---

_ID = re.compile(r"(?<![A-Za-z])(?:[BJG]\d+[\w-]*|WG\d+)")


def _kind(path: str) -> str:
    """A path with its ids replaced by `*`, to count differences of one sort together."""
    return _ID.sub("*", path)


def _rotation_angle(a, b) -> float:
    dot = abs(float(np.dot(np.asarray(a, float), np.asarray(b, float))))
    return 2.0 * float(np.arccos(min(1.0, dot)))


def diff(a, b, path: str, out: List[Tuple[str, str]]) -> None:
    """Every difference between two values of a process, as (path, detail); floats within tolerance are equal."""
    if isinstance(a, Pose) and isinstance(b, Pose):
        distance = float(np.linalg.norm(np.subtract(a.position, b.position)))
        angle = _rotation_angle(a.orientation, b.orientation)
        if distance > POSE_TOL or angle > ANGLE_TOL:
            out.append((path, f"pose differs by {distance * 1000:.4g} mm, {np.degrees(angle):.4g} deg"))
    elif isinstance(a, TriMesh) and isinstance(b, TriMesh):
        if a.vertices.shape != b.vertices.shape or a.faces.shape != b.faces.shape:
            out.append((path, f"mesh {a.vertices.shape[0]} vertices / {a.faces.shape[0]} faces vs "
                              f"{b.vertices.shape[0]} / {b.faces.shape[0]}"))
        elif not np.allclose(a.vertices, b.vertices, atol=POSE_TOL) or not np.array_equal(a.faces, b.faces):
            gap = float(np.abs(a.vertices - b.vertices).max())
            out.append((path, f"mesh vertices differ by up to {gap * 1000:.4g} mm"))
    elif isinstance(a, Geometry) and isinstance(b, Geometry):
        diff(a.collision, b.collision, f"{path}.collision", out)
        diff(a.visual, b.visual, f"{path}.visual", out)
    elif dataclasses.is_dataclass(a) and type(a) is type(b):
        for item in dataclasses.fields(a):
            diff(getattr(a, item.name), getattr(b, item.name), f"{path}.{item.name}", out)
    elif isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b), key=str):
            if key not in a or key not in b:
                out.append((f"{path}[{key}]", f"only in {'second' if key not in a else 'first'}"))
            else:
                diff(a[key], b[key], f"{path}[{key}]", out)
    elif isinstance(a, (set, frozenset)) and isinstance(b, (set, frozenset)):
        if a != b:
            out.append((path, f"only first {sorted(a - b, key=str)[:8]} ({len(a - b)}), only second "
                              f"{sorted(b - a, key=str)[:8]} ({len(b - a)})"))
    elif isinstance(a, (tuple, list)) and isinstance(b, (tuple, list)):
        if len(a) != len(b):
            out.append((path, f"{len(a)} item(s) vs {len(b)}: {_short(a)} vs {_short(b)}"))
        else:
            for index, (x, y) in enumerate(zip(a, b)):
                diff(x, y, f"{path}[{index}]", out)
    elif isinstance(a, bool) or isinstance(b, bool) or not isinstance(a, (int, float)) \
            or not isinstance(b, (int, float)):
        if a != b:
            out.append((path, f"{_short(a)} vs {_short(b)}"))
    elif abs(a - b) > VALUE_TOL:
        out.append((path, f"{a:.9g} vs {b:.9g} (differ by {abs(a - b):.3g})"))


def world_mesh(shapes, pose: Pose):
    """Shapes of one body as one triangle mesh in the world, or None without shapes."""
    meshes = [shape_mesh(shape) for shape in shapes]
    if not meshes:
        return None
    vertices, faces, offset = [], [], 0
    for mesh in meshes:
        vertices.append(np.asarray(mesh.vertices, float))
        faces.append(np.asarray(mesh.faces, np.int64) + offset)
        offset += len(mesh.vertices)
    matrix = pose.matrix()
    points = np.vstack(vertices) @ matrix[:3, :3].T + matrix[:3, 3]
    return trimesh.Trimesh(points, np.vstack(faces), process=False)


def surface_gap(a, b, samples: int = 1500) -> float:
    """Largest distance from (up to `samples`) vertices of each mesh to the other's surface, metres."""
    worst = 0.0
    for source, target in ((a, b), (b, a)):
        points = np.asarray(source.vertices)
        if len(points) > samples:
            points = points[np.random.default_rng(0).choice(len(points), samples, replace=False)]
        worst = max(worst, float(trimesh.proximity.closest_point(target, points)[1].max()))
    return worst


def motion(world: Pose, process: Pose) -> Pose:
    """The rigid motion, in the world, that carries a body from its process placement to `world`.

    It does not depend on where the body's own frame sits, so two processes with different body frames compare.
    """
    return compose(world, invert(process))


def _short(value, limit: int = 160) -> str:
    text = repr(value)
    return text if len(text) <= limit else text[:limit] + "…"


def compare_processes(report: cc.Report, a, b, labels: Tuple[str, str], watch: Stopwatch) -> None:
    """Two in-memory processes: hashes, every field by id, then each movement's scene."""
    hashes = report.category(labels, "content hashes")
    process_a, actions_a = process_hashes(a, [cc.DATA / "husky_urdf"])
    process_b, actions_b = process_hashes(b, [cc.DATA / "husky_urdf"])
    hashes.add("process.json", [] if process_a == process_b else [("process.json hash differs", "")])
    for action in sorted(set(actions_a) & set(actions_b)):
        hashes.add(action, [] if actions_a[action] == actions_b[action] else [("action hash differs", "")])
    watch.lap("processes: content hashes")

    for name in ("robots", "tools", "bodies"):
        category = report.category(labels, f"process {name}")
        first, second = getattr(a, name), getattr(b, name)
        for key in sorted(set(first) | set(second)):
            if key not in first or key not in second:
                category.add(key, [(f"{name}: only in {labels[1] if key not in first else labels[0]}", "")])
                continue
            found: List[Tuple[str, str]] = []
            diff(first[key], second[key], name, found)
            if name == "bodies":
                # * A body's frame is a convention (a bar's sits at its end in the old export, at its middle in
                #   the new process): compare the shapes where the assembly poses put them, not the poses.
                found = [(path, detail) for path, detail in found
                         if not path.startswith(("bodies.assembly_pose", "bodies.geometry"))]
                for which in ("collision", "visual"):
                    x = world_mesh(getattr(first[key].geometry, which), first[key].assembly_pose)
                    y = world_mesh(getattr(second[key].geometry, which), second[key].assembly_pose)
                    if x is None or y is None:
                        if (x is None) != (y is None):
                            found.append((f"bodies.{which}: only one side has shapes", ""))
                        continue
                    gap = surface_gap(x, y)
                    category.deviation(f"{which} surface m", gap)
                    if gap > SHAPE_TOL:
                        found.append((f"{key.split('/')[0]} {which} shape placed in the world differs",
                                      f"surfaces up to {gap * 1000:.3g} mm apart"))
            category.add(key, [(_kind(path), detail) for path, detail in found])
    for name in ("mates", "schedule"):
        category = report.category(labels, f"process {name}")
        found = []
        diff(getattr(a, name), getattr(b, name), name, found)
        category.add(name, found)

    actions = report.category(labels, "actions")
    movements = report.category(labels, "movements")
    for key in sorted(set(a.actions) | set(b.actions)):
        if key not in a.actions or key not in b.actions:
            actions.add(key, [(f"action only in {labels[1] if key not in a.actions else labels[0]}", "")])
            continue
        x, y = a.actions[key], b.actions[key]
        found = []
        for item in ("type", "robot", "bar", "ground", "supports_until"):
            diff(getattr(x, item), getattr(y, item), f"action.{item}", found)
        ids_x, ids_y = [m.id for m in x.movements], [m.id for m in y.movements]
        if ids_x != ids_y:
            found.append(("action: movement ids differ", f"{ids_x} vs {ids_y}"))
        actions.add(key, found)
        by_id = {m.id: m for m in y.movements}
        for move in x.movements:
            if move.id not in by_id:
                continue
            found = []
            for item in dataclasses.fields(move):
                if item.name not in ("id", "label", "notes"):
                    diff(getattr(move, item.name), getattr(by_id[move.id], item.name), f"movement.{item.name}", found)
            # ? Grasps are relative to body frames, which differ by convention; the scene check below compares
            #   where held bodies are in the world.
            movements.add(move.id, [(_kind(path), detail) for path, detail in found if ".grasp" not in path])
    watch.lap("processes: fields")

    scenes = report.category(labels, "scene world poses")
    second = {(action.id, movement.id): movement for action, movement in b.movements()}
    for action, movement in a.movements():
        other = second.get((action.id, movement.id))
        if other is None:
            continue
        scene_a, scene_b = a.scene_at(movement), b.scene_at(other)
        found = []
        for body in sorted(set(scene_a.bodies) & set(scene_b.bodies)):
            on_a, on_b = scene_a.bodies[body].enabled, scene_b.bodies[body].enabled
            if on_a != on_b:
                found.append((f"scene: {body.split('/')[0]} present differs", f"{body}: {on_a} vs {on_b}"))
            elif on_a:
                moved_a = motion(scene_a.world_poses[body], a.bodies[body].assembly_pose)
                moved_b = motion(scene_b.world_poses[body], b.bodies[body].assembly_pose)
                distance = float(np.linalg.norm(np.subtract(moved_a.position, moved_b.position)))
                angle = _rotation_angle(moved_a.orientation, moved_b.orientation)
                scenes.deviation("body motion m", distance)
                scenes.deviation("body motion rad", angle)
                if distance > MOTION_TOL or angle > MOTION_ANGLE_TOL:
                    found.append((f"scene: {body.split('/')[0]} placed differently",
                                  f"{body}: {distance * 1000:.4g} mm, {np.degrees(angle):.4g} deg apart"))
        for robot in sorted(set(scene_a.robots) & set(scene_b.robots)):
            x, y = scene_a.robots[robot], scene_b.robots[robot]
            state = []
            diff((x.enabled, x.base_tracked, sorted(x.unmeasured)), (y.enabled, y.base_tracked, sorted(y.unmeasured)),
                 f"{robot} (enabled, base known, open joints)", state)
            if x.enabled and y.enabled and x.base_tracked and y.base_tracked:
                diff(x.base, y.base, f"{robot} base", state)
                diff({k: v for k, v in x.joints.items() if k not in x.unmeasured},
                     {k: v for k, v in y.joints.items() if k not in y.unmeasured}, f"{robot} joints", state)
            found += [(f"scene: robot {_kind(p).split('[')[0].split(' ', 1)[-1]} differs", f"{p}: {d}")
                      for p, d in state]
        scenes.add(movement.id, found)
    watch.lap("processes: scenes")


# --- --- --- --- --- RUN --- --- --- --- ---

def run(old_export: Path, new_process: Path, worlds: bool = True, collisions: bool = True, processes: bool = True,
        log=print, cells: bool = True) -> Tuple[cc.Report, Stopwatch]:
    """Both tests on one pair of exports.

    Returns:
        tuple[Report, Stopwatch]: Every category and difference kind, and the time of each step.
    """
    report, watch = cc.Report(), Stopwatch()
    log("reading the new process")
    process = read(new_process)
    watch.lap("new: read process folder")
    log("loading the old export")
    export = load_export(old_export, cc.quiet, watch)
    mapping, dropped = movement_map(export)
    report.notes.append(f"old export: {len(export.actions)} actions, "
                        f"{sum(len(a.movements) for _, a in export.actions)} movements; new process: "
                        f"{len(process.actions)} actions, {sum(len(a.movements) for a in process.actions.values())} "
                        f"movements")
    report.notes.append("old movements that start no new movement: "
                        + ", ".join(f"{len(ids)} {reason}" for reason, ids in dropped.items()))
    report.notes.append(f"tolerances: joints {cc.JOINT_TOL}, frames {cc.FRAME_TOL} m / rad, mesh points "
                        f"{cc.MESH_TOL} m; process diff: poses {POSE_TOL} m / rad, values {VALUE_TOL}")
    converted = None
    if processes:
        log("converting the old export")
        converted = from_export(export, robot_files(cc.DATA), serials=SERIALS, report=cc.quiet)
        watch.lap("old: convert to a process")
        counts = {key: len(action.movements) for key, action in converted.actions.items()}
        mapped = defaultdict(int)
        for new_action, _ in mapping.values():
            mapped[new_action] += 1
        if dict(mapped) != counts:
            report.notes.append("! the movement map disagrees with the converter for: "
                                + ", ".join(sorted(k for k in set(mapped) | set(counts)
                                                   if mapped.get(k) != counts.get(k))[:10]))
    if worlds:
        old = load_old(export, process)
        core = load_core(process, report, watch)
        compare_worlds(report, old, core, process, collisions, watch, log, cells)
    if processes:
        log("comparing the processes")
        compare_processes(report, converted, process, ("old export converted", "new process"), watch)
    return report, watch


def main() -> None:
    """Command line."""
    parser = ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("old_export", type=Path)
    parser.add_argument("new_process", type=Path)
    parser.add_argument("--json", type=Path, help="write the full report here")
    parser.add_argument("--no-worlds", action="store_true", help="skip test 1")
    parser.add_argument("--no-collisions", action="store_true", help="skip test 1's collision checks")
    parser.add_argument("--no-cells", action="store_true", help="skip test 1's cell comparison (the slowest step)")
    parser.add_argument("--no-processes", action="store_true", help="skip test 2")
    args = parser.parse_args()
    start = time.perf_counter()
    report, watch = run(args.old_export, args.new_process, not args.no_worlds, not args.no_collisions,
                        not args.no_processes, cells=not args.no_cells)
    report.print()
    print(f"\ntiming: {watch.summary()}; whole run {time.perf_counter() - start:.1f} s")
    if args.json:
        result = report.as_dict()
        result["timing"] = {"laps": watch.laps, "total": time.perf_counter() - start}
        args.json.write_text(json.dumps(result, indent=1, default=str), encoding="utf8")
        print(f"report: {args.json}")
    sys.exit(1 if report.unexpected() else 0)


if __name__ == "__main__":
    main()
