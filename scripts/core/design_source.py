"""
Read the Rhino document into a schema 2 `Design` (bar_assembly_core), fresh on every command.

`RhinoDesignSource(command).build()` reads bars, halves, ground, obstacles, robots, tools, mates and the per-bar
keyframe records, then `core.design_builder` adds the actions. Units are converted once, here: document units and
mm to metres.

- ! Call it after `repair_on_entry` (it repairs sequences and migrates old joint names) and keep no `Design` between
  commands: the document may change in between.
- Fake bars and their halves are left out. Duplicate ids and scaled or mirrored blocks are refused.
- Needs numpy, the core and Rhino; not compas, compas_fab, PyBullet or the tamp submodule.
- ! Pass the same `cache` (a dict in `sc.sticky`, `geometry_cache`) on every command: an unchanged document then
  gives the same `Geometry` and `Tool` objects, so the planning mirrors keep their cells instead of rebuilding.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import tempfile
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from core import config
from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path)
from core import joint_name_conventions as jnc
from core.design_builder import BarRecord, BuildResult, HoldRecord, build_actions, grasp_of
from bar_assembly_core.design import BodySpec, Design, DesignError, Producer, RobotSpec
from bar_assembly_core.design.version import writer_info
from bar_assembly_core.geometry import CylinderShape, Geometry, Pose, TriMesh
from bar_assembly_core.kinematics import link_pose
from bar_assembly_core.robot import Tool
from bar_assembly_core.urdf import UR_JOINT_NAMES, copy_robot, urdf_links

REPO = "bar_joint_rhino_design_workflow"
#: Robot name -> hardware serial. ? Rhino has no source for these; same values as the monitor's config.
SERIALS = {"Cindy": "0806", "Alice": "0804", "Belle": "0805"}
#: Tool kinds (format §7). ? Rhino has no source: Cindy's tools are scaffolding_v3, the support gripper a Robotiq.
ASSEMBLY_TOOL_KIND, SUPPORT_TOOL_KIND = "scaffolding_v3", "robotiq"
#: Flange side -> robot links Cindy's tools may touch (as `core.robot_cell._ARM_TOOL_WRIST_TOUCH_LINKS`).
ARM_TOOL_TOUCH_LINKS = {"left": ("left_ur_arm_wrist_2_link", "left_ur_arm_wrist_3_link"),
                        "right": ("right_ur_arm_wrist_2_link", "right_ur_arm_wrist_3_link")}
#: A block transform whose rotation part is further than this from a rotation is refused (scaled or mirrored).
ROTATION_TOLERANCE = 1e-6


class DesignSourceError(DesignError):
    """The document cannot become a design (duplicate ids, a broken block); `problems` lists every reason."""


@dataclass
class SourceResult:
    """The design read from the document and what was left out or assumed, one line each."""

    design: Design
    problems: List[str] = field(default_factory=list)


# --- --- --- --- --- PURE HELPERS (no Rhino) --- --- --- --- ---

def pose_from_mm(matrix_mm) -> Pose:
    """A pose in metres from a 4x4 transform with its translation in mm."""
    matrix = np.array(matrix_mm, dtype=float)
    matrix[:3, 3] /= 1000.0
    return Pose.from_matrix(matrix)


def rotation_problem(matrix) -> Optional[str]:
    """Why a 4x4 transform is not a rigid motion (scaled, sheared or mirrored), or None."""
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    if np.abs(rotation.T @ rotation - np.eye(3)).max() > ROTATION_TOLERANCE:
        return "it is scaled or sheared"
    if np.linalg.det(rotation) < 0:
        return "it is mirrored"
    return None


def joints_from_groups(groups) -> Optional[Dict[str, float]]:
    """One joint map from Rhino's keyframe record: `{"left": {joint_names, joint_values}, ...}` or one such group."""
    if not groups:
        return None
    parts = [groups] if "joint_names" in groups else [groups[side] for side in sorted(groups)]
    return {str(n): float(v) for part in parts for n, v in zip(part["joint_names"], part["joint_values"])}


def bar_pose(frame_mm, length_mm: float) -> Pose:
    """A bar's design pose: its Rhino frame (origin at the start, z along the bar) moved to the midpoint, metres."""
    matrix = np.array(frame_mm, dtype=float)
    matrix[:3, 3] += matrix[:3, 2] * (length_mm / 2.0)
    return pose_from_mm(matrix)


def unit(vector) -> Tuple[float, float, float]:
    """A vector scaled to length one."""
    vector = np.asarray(vector, dtype=float)
    return tuple(float(v) for v in vector / np.linalg.norm(vector))


def read_obj(path: str, scale: float) -> TriMesh:
    """An OBJ file's `v` and `f` lines as a TriMesh, coordinates times `scale`; polygons become triangle fans.

    ? Parsed here, not with trimesh: trimesh needs PIL for OBJ files with texture coordinates.
    """
    vertices, triangles = [], []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "v":
                vertices.append([float(c) * scale for c in parts[1:4]])
            elif parts[0] == "f":
                ids = [int(token.split("/")[0]) for token in parts[1:]]
                ids = [i - 1 if i > 0 else len(vertices) + i for i in ids]
                triangles += [(ids[0], ids[k], ids[k + 1]) for k in range(1, len(ids) - 1)]
    return TriMesh.from_arrays(vertices, triangles)


def body_id(prefix: str, name: str) -> str:
    """A body id from a Rhino name: characters outside `[A-Za-z0-9_.-]` become `_`."""
    clean = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in str(name).strip())
    return f"{prefix}/{clean or 'unnamed'}"


#: `sc.sticky` key of the geometry cache.
GEOMETRY_CACHE_KEY = "bar_joint:core_geometry_cache"


def geometry_cache(store: dict) -> dict:
    """The geometry cache kept in `store` (Rhino's `sc.sticky`); a new one when the core was reloaded.

    ? A plain dict, not a class: this module is reloaded on every command, the cache must outlive it.
    """
    cache = store.get(GEOMETRY_CACHE_KEY)
    if not isinstance(cache, dict) or cache.get("core") is not Geometry:
        cache = store[GEOMETRY_CACHE_KEY] = {"core": Geometry, "items": {}, "hits": 0, "misses": 0}
    return cache


def cached(cache: dict, key: tuple, make):
    """The object cached under a content key, made by `make()` on a miss.

    ! Body geometry keys include the body id: the mirrors name collision pairs by geometry object, so two bodies
      must never share one `Geometry`.
    """
    items = cache["items"]
    if key in items:
        cache["hits"] += 1
    else:
        cache["misses"] += 1
        items[key] = make()
    return items[key]


def file_digest(path: str) -> str:
    """SHA-1 of a file's bytes: the content key of a mesh file."""
    with open(path, "rb") as f:
        return hashlib.sha1(f.read()).hexdigest()


def robot_files(cache: dict, name: str, urdf: Path, srdf: Path) -> Tuple[Path, Path]:
    """A robot's URDF and SRDF with relative mesh paths (copied once per cache), as the mirrors need them.

    ? The source URDFs name their meshes `package://`; compas_fab cannot load those from a design in memory.
    """
    stamp = tuple(os.path.getmtime(f) for f in (urdf, srdf))

    def make():
        dest = Path(tempfile.gettempdir()) / "bar_joint_core" / "robots" / name.lower()
        return copy_robot(urdf, srdf, dest, package_dirs=[Path(config.HUSKY_PKG_PATH)])
    return cached(cache, ("robot", str(urdf), str(srdf), stamp), make)


def _git(*args: str) -> Optional[str]:
    """Run git in this repository; None if git is missing or fails (Rhino on Windows may have no git on PATH)."""
    try:
        result = subprocess.run(["git", *args], cwd=config.REPO_ROOT, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def producer(command: str) -> Producer:
    """The `producer` block: this repository's commit, uncommitted changes, and the command."""
    commit = _git("rev-parse", "--short=12", "HEAD")
    dirty = bool(_git("status", "--porcelain", "--", "scripts")) if commit else False
    return Producer(REPO, commit or "unknown", dirty, command)


# --- --- --- --- --- THE DOCUMENT --- --- --- --- ---

class RhinoDesignSource:
    """Builds a `Design` from the active Rhino document. Make a new one per command."""

    def __init__(self, command: str, cache: Optional[dict] = None):
        """Remember the command name for `producer` and messages.

        Args:
            command: The Rhino command, for `producer`.
            cache: A `geometry_cache`, kept between commands; a fresh one if None.
        """
        self.command = command
        self.cache = cache if cache is not None else geometry_cache({})
        self.problems: List[str] = []
        self.bodies: Dict[str, BodySpec] = {}
        self.duplicates: List[str] = []
        self.ground_oids: Dict[str, object] = {}
        self.tool_names: Dict[str, str] = {}

    def build(self) -> SourceResult:
        """Read the document and build the design with its actions.

        Raises:
            DesignSourceError: On duplicate ids or broken blocks, listing all of them.
        """
        import rhinoscriptsyntax as rs
        from core.hold_schedule import build_action_schedule, derive_hold_plan
        from core.rhino_bar_registry import collect_hold_inputs, get_bar_seq_map, get_fake_bar_ids

        seq_map = get_bar_seq_map()
        self._check_duplicate_bars(rs)
        fake = get_fake_bar_ids(seq_map)
        bars = {b: v for b, v in seq_map.items() if b not in fake}
        if fake:
            self.problems.append(f"fake bars left out with their halves: {', '.join(sorted(fake))}")
        order = [b for b, _ in sorted(bars.items(), key=lambda kv: kv[1][1])]

        self._read_bars(bars)
        halves = self._read_halves(rs, bars, fake)
        grounds = self._read_ground(rs)
        self.ground_oids = grounds
        self._read_obstacles(rs)
        if self.duplicates:
            raise DesignSourceError(self.duplicates)
        robots, tools = self._robots_and_tools()
        mates = self._mates(halves, grounds)

        bar_seq, supported = collect_hold_inputs(bars)
        try:
            hold_plan = derive_hold_plan(bar_seq, supported, config.SUPPORT_ROBOT_NAMES)
        except RuntimeError as exc:
            self.problems.append(f"hold plan: {exc}; no holds exported")
            hold_plan = {}
        records = {f"bars/{b}": r for b in order if f"bars/{b}" in self.bodies
                   for r in [self._bar_record(rs, b, bars[b][0], halves)] if r}
        holds = {f"bars/{b}": h for b in hold_plan if f"bars/{b}" in self.bodies
                 for h in [self._hold_record(rs, b, bars[b][0], hold_plan[b], robots, supported)] if h}
        kinds = {"jointing": "J", "release": "R", "holding": "H", "holding_release": "HR"}
        schedule = [(kinds[e["kind"]], f"bars/{e['bar_id']}")
                    for e in build_action_schedule(order, hold_plan, config.ASSEMBLY_ROBOT_NAME)]

        design = Design(folder=None, writer=writer_info(), robots=robots, tools=tools, bodies=dict(self.bodies),
                        schedule=(), actions={}, mates=frozenset(mates), producer=producer(self.command))
        result: BuildResult = build_actions(
            design, schedule, records, holds, insert_distance=config.LM_APPROACH_DISTANCE / 1000.0,
            retreat_distance=config.LM_RETREAT_DISTANCE / 1000.0,
            support_distance=config.SUPPORT_LM_DISTANCE_MM / 1000.0)
        return SourceResult(result.design, self.problems + result.problems)

    # * Bodies

    def _add(self, body: BodySpec, source: str) -> None:
        """Add a body; a second body with the same id is a duplicate."""
        if body.id in self.bodies:
            self.duplicates.append(f"duplicate id {body.id} ({source}); run RSUpdatePreview, then RSReorderBarID "
                                   f"-> Relink")
            return
        self.bodies[body.id] = body

    def _check_duplicate_bars(self, rs) -> None:
        """Two bar curves with one `bar_id` (copy/paste) are a duplicate."""
        from core.rhino_bar_registry import BAR_ID_KEY, BAR_TYPE_KEY, BAR_TYPE_VALUE
        seen: Dict[str, int] = {}
        for oid in rs.AllObjects() or []:
            if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE:
                bar = rs.GetUserText(oid, BAR_ID_KEY)
                seen[bar] = seen.get(bar, 0) + 1
        self.duplicates += [f"duplicate bar id {bar} on {n} curves" for bar, n in sorted(seen.items()) if n > 1]

    def _read_bars(self, bars) -> None:
        """Every real bar as a cylinder at its midpoint frame."""
        import rhinoscriptsyntax as rs
        from core.env_collision import _bar_world_frame_mm
        from core.rhino_bar_registry import BAR_RADIUS_KEY

        for bar, (oid, _seq) in bars.items():
            length_mm, frame_mm = _bar_world_frame_mm(oid)
            if length_mm <= 0.0:
                self.problems.append(f"bar {bar} has zero length; left out")
                continue
            try:
                radius_mm = float(rs.GetUserText(oid, BAR_RADIUS_KEY) or config.BAR_RADIUS)
            except ValueError:
                radius_mm = float(config.BAR_RADIUS)
            size = (round(radius_mm / 1000.0, 9), round(length_mm / 1000.0, 9))
            geometry = cached(self.cache, (f"bars/{bar}", "cylinder") + size,
                              lambda: Geometry((CylinderShape(*size),), (CylinderShape(*size),)))
            self._add(BodySpec(f"bars/{bar}", bar_pose(frame_mm, length_mm), geometry), f"bar {bar}")

    def _file_geometry(self, owner: str, path: str, scale: float = 0.001) -> Optional[Geometry]:
        """The geometry of a body or tool from a collision OBJ (mm unless `scale` says otherwise); None if missing.

        The mesh is read once per file content and shared; the `Geometry` is the owner's own.
        """
        if not path or not os.path.isfile(path):
            return None
        digest = file_digest(path)
        mesh = cached(self.cache, ("obj", path, scale, digest), lambda: read_obj(path, scale))
        return cached(self.cache, (owner, "obj", path, scale, digest), lambda: Geometry((mesh,), (mesh,)))

    def _read_halves(self, rs, bars, fake) -> Dict[str, Tuple[str, str, str]]:
        """Every joint block on a real bar, with part, mount and markers.

        Returns:
            dict: Half id -> (joint id, subtype, bar id).
        """
        from core.joint_pair import load_joint_registry
        from core.rhino_helpers import block_instance_xform_mm

        registry = load_joint_registry()
        out = {}
        for layer in jnc.JOINT_LAYERS:
            if not rs.IsLayer(layer):
                continue
            subtype = jnc.subtype_of_layer(layer)
            for oid in rs.ObjectsByLayer(layer) or []:
                parent = rs.GetUserText(oid, jnc.UT_PARENT_BAR)
                jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
                name = rs.ObjectName(oid) or str(oid)
                if parent in fake:
                    continue
                if parent not in bars or not jid:
                    self.problems.append(f"joint block {name} has no live parent bar or no joint_id; left out")
                    continue
                block = rs.BlockInstanceName(oid) or ""
                definition = registry.definition(block) if jnc.is_block_name(block) else None
                if definition is None:
                    self.problems.append(f"joint block {name}: block {block!r} is not in joint_pairs.json; left out")
                    continue
                xform = block_instance_xform_mm(oid)
                why = rotation_problem(xform)
                half = f"joints/{jnc.object_name(jid, subtype)}"
                if why:
                    self.duplicates.append(f"{half}: its block transform is refused: {why}")
                    continue
                geometry = self._file_geometry(half, definition.collision_path())
                if geometry is None:
                    self.problems.append(f"{half}: collision OBJ of {block} missing; the half never collides")
                    geometry = cached(self.cache, (half, "empty"), lambda: Geometry((), ()))
                markers = {str(label): tuple(float(c) / 1000.0 for c in point)
                           for label, point in getattr(definition, "marker_points_mm", {}).items()}
                type_, block_subtype = jnc.split_block_name(block)
                self._add(BodySpec(half, pose_from_mm(xform), geometry, part=f"{type_}/{block_subtype}",
                                   markers=markers, mount=f"bars/{parent}"), f"joint block {name}")
                out[half] = (jid, subtype, parent)
        return out

    def _rhino_geometry(self, owner: str, oid) -> Optional[Geometry]:
        """A mesh or meshable brep in world coordinates, metres, by content; None if neither."""
        import Rhino
        import rhinoscriptsyntax as rs
        from core.env_collision import _coerce_env_brep
        from core.rhino_helpers import doc_unit_scale_to_mm

        if rs.IsMesh(oid):
            mesh = rs.coercemesh(oid)
        else:
            brep = _coerce_env_brep(oid)
            if brep is None:
                return None
            mesh = Rhino.Geometry.Mesh()
            for part in Rhino.Geometry.Mesh.CreateFromBrep(brep, Rhino.Geometry.MeshingParameters.Coarse) or []:
                if part is not None:
                    mesh.Append(part)
        if mesh is None or mesh.Vertices.Count == 0:
            return None
        scale = doc_unit_scale_to_mm() / 1000.0
        vertices = [(v.X * scale, v.Y * scale, v.Z * scale) for v in mesh.Vertices]
        triangles = []
        for f in mesh.Faces:
            triangles.append((f.A, f.B, f.C))
            if f.C != f.D:  # * Rhino stores triangles as quads with D == C
                triangles.append((f.A, f.C, f.D))
        vertices, triangles = np.asarray(vertices, dtype=float), np.asarray(triangles, dtype=np.int64)
        digest = hashlib.sha1(vertices.round(9).tobytes() + triangles.tobytes()).hexdigest()

        def make():
            trimesh = TriMesh.from_arrays(vertices, triangles)
            return Geometry((trimesh,), (trimesh,))
        return cached(self.cache, (owner, "rhino", digest), make)

    def _read_ground(self, rs) -> Dict[str, object]:
        """Every walkable ground brep with an id, as a world-frame mesh. Returns ground id -> Rhino object id."""
        out = {}
        layer = config.LAYER_WALKABLE_GROUND
        for oid in (rs.ObjectsByLayer(layer) or []) if rs.IsLayer(layer) else []:
            gid = rs.GetUserText(oid, config.KEY_WALKABLE_GROUND_ID)
            if not gid:
                self.problems.append(f"walkable ground {oid} has no id; left out (run RSAssignAndShowWalkableGround)")
                continue
            geometry = self._rhino_geometry(f"ground/{gid}", oid)
            if geometry is None:
                self.problems.append(f"walkable ground {gid} could not be meshed; left out")
                continue
            self._add(BodySpec(f"ground/{gid}", Pose(), geometry), f"walkable ground {gid}")
            out[f"ground/{gid}"] = oid
        return out

    def _read_obstacles(self, rs) -> None:
        """Every mesh or brep on the environment layer, as a world-frame mesh."""
        layer = config.LAYER_ENVIRONMENT
        for i, oid in enumerate((rs.ObjectsByLayer(layer) or []) if rs.IsLayer(layer) else []):
            name = body_id("obstacles", rs.ObjectName(oid) or f"env{i}")
            base, k = name, 1
            while name in self.bodies:
                name, k = f"{base}_{k}", k + 1
            geometry = self._rhino_geometry(name, oid)
            if geometry is None:
                self.problems.append(f"environment object {oid} is neither a mesh nor a brep; left out")
                continue
            self._add(BodySpec(name, Pose(), geometry), f"environment object {oid}")

    # * Robots, tools, mates

    def _assembly_tools(self) -> Dict[str, object]:
        """Side -> the tool Cindy carries for the whole design: the one placed most often, else the active pair's.

        ? Schema 2 has one tool per flange; bars placed with another variant are reported in their records.
        """
        import rhinoscriptsyntax as rs
        from collections import Counter
        from core.robotic_tool import arm_side_from_tool_name, get_active_pair, load_robotic_tools

        registry = load_robotic_tools()
        placed = {"left": Counter(), "right": Counter()}
        layer = config.LAYER_TOOL_INSTANCES
        for oid in (rs.ObjectsByLayer(layer) or []) if rs.IsLayer(layer) else []:
            name = rs.GetUserText(oid, jnc.UT_TOOL_NAME) or ""
            side = arm_side_from_tool_name(name)
            if side in placed and name in registry:
                placed[side][name] += 1
        chosen = dict(get_active_pair())
        for side, counts in placed.items():
            if counts:
                chosen[side] = registry[counts.most_common(1)[0][0]]
        return chosen

    def _robots_and_tools(self) -> Tuple[Dict[str, RobotSpec], Dict[str, Tool]]:
        """Cindy with her tool pair (`_assembly_tools`), Alice and Belle with the Robotiq gripper."""
        robots, tools = {}, {}
        pkg = config.HUSKY_PKG_PATH
        cindy = robot_id(config.ASSEMBLY_ROBOT_NAME)
        urdf = Path(pkg, config.HUSKY_URDF_PKG_NAME, "urdf", config.HUSKY_URDF_FILENAME)
        srdf = Path(pkg, config.HUSKY_URDF_PKG_NAME, config.HUSKY_SRDF_REL_PATH)
        flanges = {}
        self.tool_names = {}
        for side, tool in self._assembly_tools().items():
            self.tool_names[side] = tool.name
            tool_id = f"tools/{tool.name}"
            touch = tuple(f"{cindy}/{link}" for link in ARM_TOOL_TOUCH_LINKS[side])
            tools[tool_id] = self._tool(tool_id, self._file_geometry(tool_id, tool.collision_path()),
                                        pose_from_mm(tool.M_tcp_from_block), ASSEMBLY_TOOL_KIND, touch)
            flanges[config.ASSEMBLY_TOOL0_LINKS[side]] = tool_id
        urdf, srdf = robot_files(self.cache, config.ASSEMBLY_ROBOT_NAME, urdf, srdf)
        robots[cindy] = RobotSpec(cindy, urdf, srdf, SERIALS.get(config.ASSEMBLY_ROBOT_NAME), flanges, wheels(urdf))

        gripper_path = config.ROBOTIQ_GRIPPER_TOOL_MESH  # * this OBJ is in metres
        tcp = pose_from_mm(np.linalg.inv(np.asarray(config.BAR_GRASP_TO_TOOL0["Robotiq"], dtype=float)))
        for name in config.SUPPORT_ROBOT_NAMES:
            rid = robot_id(name)
            spec = config.SUPPORT_ROBOTS[name]
            urdf = Path(pkg, config.SUPPORT_URDF_PKG_NAME, "urdf", spec["urdf_filename"])
            srdf = Path(pkg, config.SUPPORT_URDF_PKG_NAME, spec["srdf_rel_path"])
            urdf, srdf = robot_files(self.cache, name, urdf, srdf)
            tool_id = f"tools/{name.lower()}/{config.SUPPORT_TOOL_NAME}"
            tools[tool_id] = self._tool(tool_id, self._file_geometry(tool_id, gripper_path, scale=1.0), tcp,
                                        SUPPORT_TOOL_KIND,
                                        tuple(f"{rid}/{link}" for link in config.SUPPORT_TOOL_TOUCH_LINKS))
            robots[rid] = RobotSpec(rid, urdf, srdf, SERIALS.get(name), {config.SUPPORT_TOOL0_LINK: tool_id},
                                    wheels(urdf))
        return robots, tools

    def _tool(self, tool_id: str, geometry: Optional[Geometry], tcp: Pose, kind: str, touch) -> Tool:
        """A tool, the same object while nothing about it changed; no geometry means it never collides."""
        if geometry is None:
            self.problems.append(f"{tool_id}: collision OBJ missing; the tool never collides")
            geometry = cached(self.cache, (tool_id, "empty"), lambda: Geometry((), ()))
        key = ("tool", tool_id, id(geometry), tuple(np.round(tcp.position, 9)), tuple(np.round(tcp.orientation, 9)),
               kind, tuple(touch))
        return cached(self.cache, key, lambda: Tool(tool_id, geometry, tcp, kind, tuple(touch)))

    def _mates(self, halves, grounds) -> List[Tuple[str, str]]:
        """Male with receiver by joint id; each ground half with the nearest walkable ground."""
        import Rhino
        from core.env_collision import _coerce_env_brep

        by_jid = {}
        for half, (jid, subtype, _bar) in halves.items():
            by_jid.setdefault(jid, {})[subtype] = half
        mates = []
        for jid, sides in sorted(by_jid.items()):
            receivers = [sides[s] for s in jnc.RECEIVER_SUBTYPES if s in sides]
            if jnc.MALE in sides and receivers:
                mates.append(tuple(sorted((sides[jnc.MALE], receivers[0]))))
        breps = {g: _coerce_env_brep(oid) for g, oid in grounds.items()}
        for half, (_jid, subtype, _bar) in sorted(halves.items()):
            if subtype != jnc.GROUND or not breps:
                continue
            p = self.bodies[half].pose.position
            scale = 1000.0 / self._doc_scale_mm()
            point = Rhino.Geometry.Point3d(p[0] * scale, p[1] * scale, p[2] * scale)
            nearest = min(breps, key=lambda g: breps[g].ClosestPoint(point).DistanceTo(point))
            mates.append(tuple(sorted((half, nearest))))
        return mates

    def _doc_scale_mm(self) -> float:
        """Millimetres per document unit."""
        from core.rhino_helpers import doc_unit_scale_to_mm
        return doc_unit_scale_to_mm()

    # * Keyframe records

    def _bar_record(self, rs, bar: str, oid, halves) -> Optional[BarRecord]:
        """Cindy's record of one bar: placed tools, the halves they are on, keyframes. None if a tool is missing."""
        from core.rhino_helpers import block_instance_xform_mm
        from core.rhino_tool_place import find_tool_for_joint
        from core.robotic_tool import arm_side_from_tool_name

        cindy = robot_id(config.ASSEMBLY_ROBOT_NAME)
        active = self.tool_names
        tool0, on, retreat, tool_ids, z_axes, ground_points = {}, {}, {}, {}, [], []
        for half, (jid, subtype, parent) in sorted(halves.items()):
            if parent != bar or subtype not in jnc.TOOL_BEARING_SUBTYPES:
                continue
            toid = find_tool_for_joint(jid)
            if toid is None:
                continue
            name = rs.GetUserText(toid, jnc.UT_TOOL_NAME) or ""
            side = arm_side_from_tool_name(name)
            if side is None:
                self.problems.append(f"{bar}: tool on {half} has no L/R suffix ({name!r})")
                continue
            if name != active.get(side):
                self.problems.append(f"{bar}: tool {name} on {half}, but the design's {side} tool is "
                                     f"{active.get(side)} (B14 will flag the grasp)")
            flange = f"{cindy}/{config.ASSEMBLY_TOOL0_LINKS[side]}"
            if flange in tool0:
                self.problems.append(f"{bar}: two {side} tools; bar left out")
                return None
            xform = block_instance_xform_mm(toid)
            tool0[flange] = pose_from_mm(xform)
            on[flange] = half
            tool_ids[flange] = f"tools/{active[side]}"
            half_z = self.bodies[half].pose.matrix()[:3, 2]
            retreat[flange] = unit(-half_z)
            z_axes.append(xform[:3, 2])
            if subtype == jnc.GROUND:
                ground_points.append(half)
        if len(tool0) != 2:
            self.problems.append(f"{bar}: needs a left and a right tool on its male or ground halves, has "
                                 f"{len(tool0)}; bar left out")
            return None
        if ground_points:
            insert = self._ground_insert_direction(bar, ground_points)
        else:
            insert = unit(np.mean(z_axes, axis=0))  # * along the tools' z, into the joint
        base_raw = rs.GetUserText(oid, config.KEY_ASSEMBLY_BASE_FRAME)
        keyframes = {key: _json(rs.GetUserText(oid, key)) for key in (
            config.KEY_ASSEMBLY_IK_APPROACH, config.KEY_ASSEMBLY_IK_ASSEMBLED, config.KEY_ASSEMBLY_IK_RETREAT)}
        solved = bool(base_raw and keyframes[config.KEY_ASSEMBLY_IK_APPROACH]
                      and keyframes[config.KEY_ASSEMBLY_IK_ASSEMBLED])
        home = dict(zip([f"{side}_ur_arm_{j}" for side in ("left", "right") for j in UR_JOINT_NAMES],
                        (float(v) for v in config.HUSKY_DUAL_ARM_HOME_CONF_12)))
        grounds = tuple(f"ground/{g.strip()}" for g in (rs.GetUserText(oid, config.KEY_BAR_WALKABLE_GROUND_IDS) or
                                                         "").split(",") if f"ground/{g.strip()}" in self.bodies)
        return BarRecord(
            bar=f"bars/{bar}", robot=cindy, tools=tool_ids, tool0=tool0, on=on, insert_direction=insert,
            retreat_direction=retreat, base=pose_from_mm(_json(base_raw)) if _json(base_raw) else None,
            approach=joints_from_groups(keyframes[config.KEY_ASSEMBLY_IK_APPROACH]) if solved else None,
            assembled=joints_from_groups(keyframes[config.KEY_ASSEMBLY_IK_ASSEMBLED]) if solved else None,
            retreat=joints_from_groups(keyframes[config.KEY_ASSEMBLY_IK_RETREAT]) if solved else None,
            home=home, ground=grounds)

    def _ground_insert_direction(self, bar: str, ground_halves) -> Tuple[float, float, float]:
        """A ground bar is inserted straight down onto the ground: against the nearest ground's normal."""
        import Rhino
        from core.env_collision import _coerce_env_brep

        normals = []
        scale = 1000.0 / self._doc_scale_mm()
        for half in ground_halves:
            p = self.bodies[half].pose.position
            point = Rhino.Geometry.Point3d(p[0] * scale, p[1] * scale, p[2] * scale)
            best = None
            for oid in self.ground_oids.values():
                brep = _coerce_env_brep(oid)
                if brep is None:
                    continue
                try:
                    # ? RhinoCommon's out parameters come back as a tuple after the return value.
                    ok, closest, _ci, _s, _t, normal = brep.ClosestPoint(point, 1e9)
                except (TypeError, ValueError) as exc:
                    self.problems.append(f"{bar}: ground normal not read ({exc})")
                    continue
                if ok and (best is None or closest.DistanceTo(point) < best[0]):
                    best = (closest.DistanceTo(point), np.array([normal.X, normal.Y, normal.Z]))
            if best is not None:
                normals.append(best[1] if best[1][2] >= 0 else -best[1])  # ? normals point up, away from the ground
        if not normals:
            self.problems.append(f"{bar}: no walkable ground under its ground halves; inserting along world -z")
            return (0.0, 0.0, -1.0)
        return unit(-np.mean(normals, axis=0))

    def _hold_record(self, rs, bar: str, oid, entry, robots, supported) -> Optional[HoldRecord]:
        """A support robot's record of one hold, from the held bar's support keys. None without a grasp frame."""
        rid = robot_id(entry["robot_name"])
        grasp_raw = _json(rs.GetUserText(oid, config.KEY_SUPPORT_GRASP_FRAME))
        if grasp_raw is None:
            self.problems.append(f"hold of {bar} by {rid}: no support grasp frame picked; hold left out")
            return None
        solved_for = rs.GetUserText(oid, config.KEY_SUPPORT_ROBOT)
        base_raw = _json(rs.GetUserText(oid, config.KEY_SUPPORT_BASE_FRAME))
        approach = joints_from_groups(_json(rs.GetUserText(oid, config.KEY_SUPPORT_IK_APPROACH)))
        held = joints_from_groups(_json(rs.GetUserText(oid, config.KEY_SUPPORT_IK_HELD)))
        if solved_for and solved_for != entry["robot_name"]:
            self.problems.append(f"hold of {bar}: keyframe solved for {solved_for} but the sequence assigns "
                                 f"{entry['robot_name']}; its base and joints are left open")
            base_raw, approach, held = None, None, None
        flange_link = config.SUPPORT_TOOL0_LINK
        tool0 = pose_from_mm(np.asarray(grasp_raw, dtype=float)
                             @ np.asarray(config.BAR_GRASP_TO_TOOL0["Robotiq"], dtype=float))
        base = pose_from_mm(base_raw) if base_raw is not None else None
        bar_pose_ = self.bodies[f"bars/{bar}"].pose
        if base is not None and held:
            # * With a solved grasp, hold the bar where the robot's forward kinematics put the flange.
            grasp = grasp_of(link_pose(robots[rid].urdf, base, held, flange_link), bar_pose_)
        else:
            grasp = grasp_of(tool0, bar_pose_)
        until = tuple(f"bars/{b}" for b in supported.get(bar, ()) if f"bars/{b}" in self.bodies)
        grounds = tuple(f"ground/{g.strip()}" for g in (rs.GetUserText(oid, config.KEY_BAR_WALKABLE_GROUND_IDS) or
                                                         "").split(",") if f"ground/{g.strip()}" in self.bodies)
        return HoldRecord(bar=f"bars/{bar}", robot=rid, flange=f"{rid}/{flange_link}",
                          tool=robots[rid].tools[flange_link], tool0=tool0, grasp=grasp, base=base,
                          approach=approach, held=held, supports_until=until, ground=grounds)


def robot_id(name: str) -> str:
    """"Cindy" -> "robots/cindy"."""
    return f"robots/{name.lower()}"


def wheels(urdf: Path) -> Tuple[str, ...]:
    """The links a robot stands on: its wheel links."""
    return tuple(sorted(link for link in urdf_links(urdf) if "wheel" in link))


def _json(raw):
    """Parsed user text, or None if empty or broken."""
    try:
        return json.loads(raw) if raw else None
    except ValueError:
        return None
