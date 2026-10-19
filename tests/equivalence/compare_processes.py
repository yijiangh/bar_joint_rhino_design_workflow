"""Compare two schema 2.1 process folders field by field, by id; print the differences grouped by kind.

    python tests/equivalence/compare_processes.py <first folder> <second folder> [--pos-tol 1e-9] [--rot-tol 1e-9]
                                                  [--value-tol 1e-9] [--examples 3] [--expect-headless-gaps]

The use case: a document built headlessly (`fake_rhino.doc_from_3dm` + `RhinoProcessSource`, written by the core's
`write`) against the folder RSExportProcess wrote in Rhino on Windows from the same .3dm. Both folders are read with
the core's `read`, so both pass the file checks (A) first.

Compared, each by id:
- bodies: every field of a body: `assembly_pose` (within the position and rotation tolerances), the shapes (type and
  size; a mesh by the hash of its vertices and triangles), `part`, `mount`, `markers`, `pose_in_tcp`, `label`;
- tools: shapes, `tcp`, `kind`;
- robots: the hash of the robot files (URDF, SRDF, meshes), `serial`, `tools`, `ground_links`; not the file paths,
  which name each folder;
- mates, and the schedule in order;
- actions: `type`, `robot`, `bar`, `ground`, `supports_until`, the movement ids in order; then per movement id every
  part (`arms`, `base`, `path`, `coupled`, `controller`, `line`, `drives`, `ends_on`), its start state and its target
  (robot bases and joints within the tolerances, `present`, `built`, `attached` with grasps, `poses`, tool grips);
- producer (the repository, commit and command that made the process).

`--expect-headless-gaps`: the headless build reads no breps, so it has no walkable ground and no obstacles. Ground
and obstacle bodies, the mates with them, an action's `ground`, the ground and obstacles in a state's `present`, and
`producer` are left out on both sides.

Exit code 0 when the two are equal (after those skips), 1 otherwise. `export_equivalence.py` reuses `diff`.
"""

from __future__ import annotations

import dataclasses
import hashlib
import re
import sys
from argparse import ArgumentParser
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO / "external" / "scaffolding_assembly_core") not in sys.path:
    sys.path.insert(0, str(REPO / "external" / "scaffolding_assembly_core"))

from scaffolding_assembly_core.geometry import Geometry, Pose, TriMesh  # noqa: E402
from scaffolding_assembly_core.process import Process, read  # noqa: E402
from scaffolding_assembly_core.urdf import robot_files_hash  # noqa: E402

#: Body id prefixes of what a headless build cannot read: walkable ground and obstacles come from breps.
ENVIRONMENT = ("ground/", "obstacles/")
#: The two folders' names in the report.
FIRST, SECOND = "A", "B"


@dataclass(frozen=True)
class Tolerance:
    """How far apart two numbers of the processes may be and still count as equal.

    Attributes:
        position: A pose's position, metres.
        angle: A pose's rotation, radians.
        value: Any other number (joint values, distances), in its own unit.
        mesh: A mesh's vertices, metres; None: meshes are equal only with the same content hash.
    """

    position: float = 1e-9
    angle: float = 1e-9
    value: float = 1e-9
    mesh: Optional[float] = None


# --- --- --- --- --- FIELD BY FIELD (shared with export_equivalence.py) --- --- --- --- ---

def rotation_angle(a, b) -> float:
    """The angle between two rotations given as quaternions, radians; 0 for the same numbers.

    ? From the chord between the two unit quaternions, not from the arccos of their dot product: a quaternion
      rounded to 12 decimals is not of length one exactly, and the arccos turns that into ~1e-6 rad between a
      rotation and itself.
    """
    qa, qb = (np.asarray(q, float) / np.linalg.norm(q) for q in (a, b))
    if np.dot(qa, qb) < 0.0:
        qb = -qb  # * q and -q are the same rotation
    return 4.0 * float(np.arcsin(min(1.0, float(np.linalg.norm(qa - qb)) / 2.0)))


def short(value, limit: int = 160) -> str:
    """A value's repr, cut to `limit` characters."""
    text = repr(value)
    return text if len(text) <= limit else text[:limit] + "…"


def mesh_hash(mesh: TriMesh) -> str:
    """The SHA-1 of a mesh's vertices and triangles: equal for the same mesh, read from any folder."""
    return hashlib.sha1(np.ascontiguousarray(mesh.vertices).tobytes()
                        + np.ascontiguousarray(mesh.faces).tobytes()).hexdigest()


def diff(a, b, path: str, out: List[Tuple[str, str]], tol: Tolerance) -> None:
    """Every difference between two values of a process, as (path, detail); numbers within `tol` are equal.

    Args:
        a: A value of the first process: a dataclass, dict, set, sequence or plain value.
        b: The same value of the second.
        path: Where the value is, e.g. "body.assembly_pose"; the paths of the differences extend it.
        out: The differences found are appended here.
        tol: The tolerances.
    """
    if isinstance(a, Pose) and isinstance(b, Pose):
        distance = float(np.linalg.norm(np.subtract(a.position, b.position)))
        angle = rotation_angle(a.orientation, b.orientation)
        if distance > tol.position or angle > tol.angle:
            out.append((path, f"pose differs by {distance * 1000:.4g} mm, {np.degrees(angle):.4g} deg"))
    elif isinstance(a, TriMesh) and isinstance(b, TriMesh):
        if a.vertices.shape != b.vertices.shape or a.faces.shape != b.faces.shape:
            out.append((path, f"mesh {a.vertices.shape[0]} vertices / {a.faces.shape[0]} faces vs "
                              f"{b.vertices.shape[0]} / {b.faces.shape[0]}"))
        elif tol.mesh is None:
            if mesh_hash(a) != mesh_hash(b):
                gap = float(np.abs(a.vertices - b.vertices).max())
                same_faces = np.array_equal(a.faces, b.faces)
                out.append((path, f"mesh content differs (vertices up to {gap * 1000:.4g} mm apart, "
                                  f"{'same' if same_faces else 'other'} triangles)"))
        elif not np.allclose(a.vertices, b.vertices, atol=tol.mesh) or not np.array_equal(a.faces, b.faces):
            gap = float(np.abs(a.vertices - b.vertices).max())
            out.append((path, f"mesh vertices differ by up to {gap * 1000:.4g} mm"))
    elif isinstance(a, Geometry) and isinstance(b, Geometry):
        diff(a.collision, b.collision, f"{path}.collision", out, tol)
        diff(a.visual, b.visual, f"{path}.visual", out, tol)
    elif dataclasses.is_dataclass(a) and type(a) is type(b):
        for item in dataclasses.fields(a):
            diff(getattr(a, item.name), getattr(b, item.name), f"{path}.{item.name}", out, tol)
    elif isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b), key=str):
            if key not in a or key not in b:
                out.append((f"{path}[{key}]", f"only in {'second' if key not in a else 'first'}"))
            else:
                diff(a[key], b[key], f"{path}[{key}]", out, tol)
    elif isinstance(a, (set, frozenset)) and isinstance(b, (set, frozenset)):
        if a != b:
            out.append((path, f"only first {sorted(a - b, key=str)[:8]} ({len(a - b)}), only second "
                              f"{sorted(b - a, key=str)[:8]} ({len(b - a)})"))
    elif isinstance(a, (tuple, list)) and isinstance(b, (tuple, list)):
        if len(a) != len(b):
            out.append((path, f"{len(a)} item(s) vs {len(b)}: {short(a)} vs {short(b)}"))
        else:
            for index, (x, y) in enumerate(zip(a, b)):
                diff(x, y, f"{path}[{index}]", out, tol)
    elif isinstance(a, bool) or isinstance(b, bool) or not isinstance(a, (int, float)) \
            or not isinstance(b, (int, float)):
        if a != b:
            out.append((path, f"{short(a)} vs {short(b)}"))
    elif abs(a - b) > tol.value:
        out.append((path, f"{a:.9g} vs {b:.9g} (differ by {abs(a - b):.3g})"))


# --- --- --- --- --- THE REPORT --- --- --- --- ---

def kind_of(path: str) -> str:
    """A difference's kind: its path with every key and index replaced by `*`, e.g. "movement.drives[*]"."""
    return re.sub(r"\[[^\]]*\]", "[*]", path)


@dataclass
class Report:
    """The differences between two processes, by group ("bodies", "actions", ...) and kind.

    Attributes:
        compared: Group -> how many items (ids) were compared.
        differing: Group -> the ids with a difference.
        kinds: Group -> kind -> one line per difference, "<id>: <path>: <detail>".
        notes: Lines printed before the groups: what was skipped, the tolerances.
    """

    compared: Dict[str, int] = field(default_factory=dict)
    differing: Dict[str, set] = field(default_factory=dict)
    kinds: Dict[str, Dict[str, List[str]]] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def add(self, group: str, item: str, found: Sequence[Tuple[str, str]]) -> None:
        """Count one compared item of a group, with the differences found on it."""
        self.compared[group] = self.compared.get(group, 0) + 1
        self.differing.setdefault(group, set())
        self.kinds.setdefault(group, {})
        for path, detail in found:
            self.differing[group].add(item)
            self.kinds[group].setdefault(kind_of(path), []).append(f"{item}: {path}: {detail}" if detail
                                                                   else f"{item}: {path}")

    def count(self) -> int:
        """int: Every difference, over all groups."""
        return sum(len(lines) for group in self.kinds.values() for lines in group.values())

    def lines(self, examples: int = 3) -> List[str]:
        """The report: the notes, then per group its counts and per kind its count and first `examples` lines."""
        out = list(self.notes)
        for group, compared in self.compared.items():
            kinds = self.kinds[group]
            out.append(f"{group}: {compared} compared, {len(self.differing[group])} with differences")
            for kind, found in sorted(kinds.items(), key=lambda kv: (-len(kv[1]), kv[0])):
                out.append(f"  {kind} ({len(found)})")
                out += [f"    {line}" for line in found[:examples]]
                if len(found) > examples:
                    out.append(f"    ... and {len(found) - examples} more")
        out.append(f"{'EQUAL' if not self.count() else 'NOT EQUAL'}: {self.count()} difference(s) in "
                   f"{sum(len(k) for k in self.kinds.values())} kind(s)")
        return out


# --- --- --- --- --- COMPARING --- --- --- --- ---

def is_environment(body_id: str) -> bool:
    """Whether a body is walkable ground or an obstacle."""
    return body_id.startswith(ENVIRONMENT)


def without_environment(process: Process) -> Process:
    """The process as a headless build sees it: no walkable ground, no obstacles, no producer."""
    def state(start):
        return replace(start, present=frozenset(body for body in start.present if not is_environment(body)))

    actions = {key: replace(action, ground=(), movements=tuple(replace(m, start=state(m.start))
                                                              for m in action.movements))
               for key, action in process.actions.items()}
    return replace(process, bodies={k: v for k, v in process.bodies.items() if not is_environment(k)},
                   mates=frozenset(m for m in process.mates if not any(map(is_environment, m))), actions=actions,
                   producer=None)


@dataclass(frozen=True)
class RobotEntry:
    """What is compared of a robot: the hash of its files (URDF, SRDF, meshes) and its fields, not the file paths."""

    files_hash: str
    serial: Optional[str]
    tools: Dict[str, str]
    ground_links: Tuple[str, ...]


def robot_entry(robot, package_dirs: Sequence[Path]) -> RobotEntry:
    """A robot of a process as compared."""
    return RobotEntry(robot_files_hash(robot.urdf, robot.srdf, package_dirs), robot.serial, robot.tools,
                      robot.ground_links)


def _by_id(report: Report, group: str, first: dict, second: dict, compare_pair) -> None:
    """Compare two maps by id: an id on one side only is one difference, the rest go through `compare_pair`."""
    for key in sorted(set(first) | set(second)):
        if key not in first or key not in second:
            report.add(group, key, [(f"only in {SECOND if key not in first else FIRST}", "")])
        else:
            report.add(group, key, compare_pair(first[key], second[key]))


def compare(a: Process, b: Process, tol: Tolerance = Tolerance(), headless_gaps: bool = False,
            package_dirs: Sequence[Path] = ()) -> Report:
    """Every difference between two processes.

    Args:
        a: The first process (A).
        b: The second process (B).
        tol: The tolerances.
        headless_gaps: Leave out what a headless build cannot have (`without_environment`).
        package_dirs: Where the robot files' `package://` meshes are, for processes not read from a folder.
    """
    report = Report()
    if headless_gaps:
        gone = [sum(map(is_environment, p.bodies)) for p in (a, b)]
        report.notes.append(f"skipped (--expect-headless-gaps): ground and obstacle bodies ({gone[0]} in {FIRST}, "
                            f"{gone[1]} in {SECOND}), their mates, actions' ground, present ground, producer")
        a, b = without_environment(a), without_environment(b)
    report.notes.append(f"tolerances: position {tol.position:g} m, rotation {tol.angle:g} rad, other numbers "
                        f"{tol.value:g}; meshes " + ("by content hash" if tol.mesh is None else f"{tol.mesh:g} m"))

    def found(x, y, path: str) -> List[Tuple[str, str]]:
        """The differences between two values, under `path`."""
        out: List[Tuple[str, str]] = []
        diff(x, y, path, out, tol)
        return out

    def action_found(x, y) -> List[Tuple[str, str]]:
        """An action's own fields and its movement ids in order; the movements themselves are compared by id."""
        out = [d for item in ("type", "robot", "bar", "ground", "supports_until", "label", "notes")
               for d in found(getattr(x, item), getattr(y, item), f"action.{item}")]
        ids_x, ids_y = [m.id for m in x.movements], [m.id for m in y.movements]
        if ids_x != ids_y:
            out.append(("action.movements", f"movement ids differ: {short(ids_x)} vs {short(ids_y)}"))
        return out

    def movement_found(x, y) -> List[Tuple[str, str]]:
        """Every field of a movement but its id: the parts, the start state, the target, label and notes."""
        return [d for item in dataclasses.fields(x) if item.name != "id"
                for d in found(getattr(x, item.name), getattr(y, item.name), f"movement.{item.name}")]

    if not headless_gaps:
        report.add("producer", "producer", found(a.producer, b.producer, "producer"))
    _by_id(report, "robots", {k: robot_entry(r, package_dirs) for k, r in a.robots.items()},
           {k: robot_entry(r, package_dirs) for k, r in b.robots.items()}, lambda x, y: found(x, y, "robot"))
    _by_id(report, "tools", a.tools, b.tools, lambda x, y: found(x, y, "tool"))
    _by_id(report, "bodies", a.bodies, b.bodies, lambda x, y: found(x, y, "body"))
    # * A mate is its pair of ids: it is on both sides or on one only.
    _by_id(report, "mates", {" + ".join(m): m for m in a.mates}, {" + ".join(m): m for m in b.mates},
           lambda x, y: [])
    report.add("schedule", "schedule", found(a.schedule, b.schedule, "schedule"))
    _by_id(report, "actions", a.actions, b.actions, action_found)
    shared = sorted(set(a.actions) & set(b.actions))
    _by_id(report, "movements", {m.id: m for k in shared for m in a.actions[k].movements},
           {m.id: m for k in shared for m in b.actions[k].movements}, movement_found)
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Command line: read both folders, print the report; returns the exit code (0 equal, 1 not)."""
    parser = ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("first", type=Path, help="process folder A")
    parser.add_argument("second", type=Path, help="process folder B")
    parser.add_argument("--pos-tol", type=float, default=1e-9, help="pose positions, metres (default 1e-9)")
    parser.add_argument("--rot-tol", type=float, default=1e-9, help="pose rotations, radians (default 1e-9)")
    parser.add_argument("--value-tol", type=float, default=1e-9, help="other numbers, e.g. joints (default 1e-9)")
    parser.add_argument("--examples", type=int, default=3, help="examples printed per kind (default 3)")
    parser.add_argument("--expect-headless-gaps", action="store_true",
                        help="leave out ground and obstacle bodies, their mates and producer: a headless build "
                             "reads no breps")
    args = parser.parse_args(argv)
    report = compare(read(args.first), read(args.second), Tolerance(args.pos_tol, args.rot_tol, args.value_tol),
                     args.expect_headless_gaps)
    print(f"{FIRST}: {args.first}\n{SECOND}: {args.second}")
    print("\n".join(report.lines(args.examples)))
    return 1 if report.count() else 0


if __name__ == "__main__":
    sys.exit(main())
