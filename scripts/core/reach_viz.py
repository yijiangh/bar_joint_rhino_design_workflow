"""Ghost spheres showing what each arm can reach from a candidate base.

RSIKKeyframe lets the user pick where the mobile base stands. Until now the only
reach feedback was a flat circle of ``config.IK_BASE_SAMPLE_RADIUS`` -- which is
a base-SAMPLING parameter reused as a display radius, not a reachability model.
This module adds the honest 3D version: one translucent sphere per arm, centered
on that arm's mount link and sized to ``config.ARM_REACH_RADIUS_MM`` (the UR5e's
nominal reach), ghosted at the candidate base so the user can see what the arms
can actually touch from there.

It is still an approximation -- a UR5e's true workspace is a shell with a dead
zone around its own column, not a solid ball -- so the real reachability test
remains the IK solve. The sphere is there to rule out obviously-too-far bases
before paying for one.

The spheres are built ONCE at the identity base and handed to
``dynamic_preview.MeshPreviewConduit(extra_meshes=...)``, which applies the same
model transform as the ghost robot. So they track the cursor for free and nothing
is ever baked into the document.
"""

from __future__ import annotations

import numpy as np

from core import config


# Cached {side: 4x4 base-local frame (mm)}; the mount is bolted on, so this only
# depends on the URDF and is computed once per Rhino session.
_MOUNT_FRAMES_CACHE = None


def clear_mount_cache() -> None:
    """Drop the cached mount frames (call after swapping robot models)."""
    global _MOUNT_FRAMES_CACHE
    _MOUNT_FRAMES_CACHE = None


def _link_name(endpoint) -> str:
    """Return the link name from a compas_robots Parent/ChildLink (or a str)."""
    name = getattr(endpoint, "link", None)
    return str(name if name is not None else endpoint)


def _frame_to_mm4(frame) -> np.ndarray:
    """Convert a compas ``Frame`` (meters) to a 4x4 numpy transform in mm.

    Same conversion as ``core.bar_action._frame_to_mm4``; duplicated here so this
    module stays importable without pulling in the movement builder.
    """
    matrix = np.eye(4, dtype=float)
    matrix[:3, 0] = np.asarray(frame.xaxis, dtype=float)
    matrix[:3, 1] = np.asarray(frame.yaxis, dtype=float)
    matrix[:3, 2] = np.asarray(frame.zaxis, dtype=float)
    matrix[:3, 3] = np.asarray(frame.point, dtype=float) * 1000.0  # m -> mm
    return matrix


def _chain_to_link(robot_model, target_link: str):
    """Return the 4x4 mm transform from the model root to ``target_link``.

    Walks the joint tree from the target link UP to the root, composing each
    joint's ``origin``. Composing origins alone IS forward kinematics at the ZERO
    configuration -- which is exactly the pose the ghost robot meshes are baked at
    (``rs_ik_keyframe._bake_robot_meshes_at_zero``), so the spheres line up with
    the ghost by construction. That also means no joint values and no
    version-sensitive ``forward_kinematics`` call are needed: this uses only plain
    data attributes, which differ across compas_robots versions far less than the
    APIs do.

    (Every joint from the husky base up to an arm mount is fixed in this URDF
    anyway, so zero-config and any-config agree for these two links.)

    Returns:
        np.ndarray | None: the 4x4 mm transform, or ``None`` when the link is not
        in the model at all.
    """
    by_child = {}
    for joint in getattr(robot_model, "joints", []) or []:
        try:
            by_child[_link_name(joint.child)] = joint
        except Exception:  # noqa: BLE001 -- a malformed joint must not kill the walk
            continue

    link = str(target_link)
    known_links = {str(getattr(link_obj, "name", ""))
                   for link_obj in getattr(robot_model, "links", []) or []}
    if known_links and link not in known_links:
        print(f"core.reach_viz: link '{link}' is not in the robot model "
              f"({len(known_links)} links). Check config.ARM_MOUNT_LINKS.")
        return None

    matrix = np.eye(4, dtype=float)
    seen = set()
    while link in by_child:
        if link in seen:  # malformed URDF -> refuse rather than loop forever
            print(f"core.reach_viz: joint loop at link '{link}'; giving up.")
            return None
        seen.add(link)
        joint = by_child[link]
        origin = getattr(joint, "origin", None)
        if origin is not None:  # a joint with no <origin> tag means identity
            matrix = _frame_to_mm4(origin) @ matrix
        link = _link_name(joint.parent)
    return matrix


def arm_mount_frames_mm(robot_model) -> dict:
    """Return ``{side: 4x4 mm frame}`` for each arm's mount link, base-relative.

    Args:
        robot_model: the cell's compas_robots ``RobotModel``
            (``rcell.robot_model``).

    Returns:
        dict: ``{"left": 4x4, "right": 4x4}`` for whichever sides resolved;
        empty when neither did (the caller then skips the sphere preview).
    """
    global _MOUNT_FRAMES_CACHE
    if _MOUNT_FRAMES_CACHE is not None:
        return _MOUNT_FRAMES_CACHE

    frames = {}
    for side, link_name in config.ARM_MOUNT_LINKS.items():
        try:
            matrix = _chain_to_link(robot_model, link_name)
        except Exception as exc:  # noqa: BLE001 -- a preview must never break a pick
            print(f"core.reach_viz: could not locate '{link_name}' ({exc}).")
            matrix = None
        if matrix is None:
            print(f"core.reach_viz: arm mount '{link_name}' not resolved; "
                  "skipping that reach sphere.")
            continue
        origin = matrix[:3, 3]
        print(f"core.reach_viz: {side} arm mount '{link_name}' at "
              f"({origin[0]:.0f}, {origin[1]:.0f}, {origin[2]:.0f}) mm "
              "relative to the mobile base.")
        frames[side] = matrix

    if not frames:
        print("core.reach_viz: no arm mount resolved -- no reach spheres will be "
              "drawn. Check config.ARM_MOUNT_LINKS against the loaded URDF.")
    _MOUNT_FRAMES_CACHE = frames
    return frames


def reach_sphere_meshes(robot_model, radius_mm: float = None):
    """Build the arm reach spheres as Rhino meshes at the IDENTITY base frame.

    The meshes are in DOCUMENT units and positioned relative to a base frame at
    the world origin, which is exactly what ``MeshPreviewConduit`` expects: it
    multiplies them by the candidate base transform on every redraw.

    Args:
        robot_model: the cell's compas_robots ``RobotModel``.
        radius_mm (float | None): sphere radius; ``None`` ->
            ``config.ARM_REACH_RADIUS_MM``.

    Returns:
        list: ``Rhino.Geometry.Mesh`` objects (empty when no mount resolved, so
        the caller can pass the result straight through as ``extra_meshes``).
    """
    import Rhino  # noqa: PLC0415  (Rhino runtime)

    from core.rhino_frame_io import doc_unit_scale_to_mm  # noqa: PLC0415

    frames = arm_mount_frames_mm(robot_model)
    if not frames:
        return []

    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    radius_doc = float(config.ARM_REACH_RADIUS_MM if radius_mm is None
                       else radius_mm) * scale_from_mm

    meshes = []
    for _side, matrix in sorted(frames.items()):
        origin = np.asarray(matrix, dtype=float)[:3, 3] * scale_from_mm
        center = Rhino.Geometry.Point3d(float(origin[0]), float(origin[1]), float(origin[2]))
        sphere = Rhino.Geometry.Sphere(center, radius_doc)
        # 24 x 16 is smooth enough to read as a ball at any sane zoom without
        # costing anything on a per-mouse-move redraw.
        mesh = Rhino.Geometry.Mesh.CreateFromSphere(sphere, 24, 16)
        if mesh is not None:
            meshes.append(mesh)
    print(f"core.reach_viz: {len(meshes)} reach sphere(s) built at radius "
          f"{float(config.ARM_REACH_RADIUS_MM if radius_mm is None else radius_mm):.0f} mm.")
    return meshes