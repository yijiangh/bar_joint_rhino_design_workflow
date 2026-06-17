"""Headless IK-keyframe replay: re-solve the M1->M2->M3 chain on a saved BarAction.

Loads a pre-generated ``RobotCell.json`` + ``BarAction.json`` (exported by the
Rhino workflow) and runs the SAME shared solve as ``rs_ik_keyframe`` --
``core.ik_keyframe.solve_keyframe_chain`` -- against each movement's
``start_state`` + ``target_ee_frames``. The action's movements already carry the
per-keyframe collision context (M2 grips the bar at approach, M3 releases it at
the assembled pose); this test confirms the chained IK lands all three.

With ``--gui`` it steps through the three freshly-solved poses
(approach -> assembled -> retreat) with ``pp.wait_if_gui()`` so you can eyeball
each one. Reuses the planner/replay helpers from
``tests/headless_bar_action_planner.py`` (no duplication).

CLI:
    python tests/headless_ik_keyframe.py [<data_root>]
        [--problem 2026-05-16_double_kissing_jig_demo]
        [--bar-action B6.json]
        [--gui]
        [--cell <RobotCell.json>]
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
CFAB_SRC = os.path.join(REPO_ROOT, "external", "compas_fab", "src")
TAMP_SRC = os.path.join(REPO_ROOT, "external", "husky_assembly_tamp")
RSDS_SRC = os.path.join(REPO_ROOT, "external", "rs_data_structure")

for _p in (SCRIPTS_DIR, TESTS_DIR, CFAB_SRC, TAMP_SRC, RSDS_SRC):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

# Outside Rhino, plain python.exe does not see the `# venv: scaffolding_env`
# site-env where compas / compas_fab / pybullet live; wire it onto sys.path.
from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()

# Reuse the planner boot + replay helpers from the sibling headless harness.
from headless_bar_action_planner import (  # noqa: E402  (path-prepend gate above)
    DEFAULT_DATA_ROOT,
    DEFAULT_PROBLEM,
    DEFAULT_BAR_ACTION,
    check_collision,
    color_rigid_body,
    fill_missing_config,
    match_role,
    start_planner,
)


def _resolve_bar_rb_name(cell, bar_id: str) -> Optional[str]:
    """Return the rigid-body name for ``bar_id`` (``bar_<id>`` or bare), or None."""
    for candidate in (bar_id, f"bar_{bar_id}"):
        if candidate in cell.rigid_body_models:
            return candidate
    return None


def _body_uids(planner, name):
    """Resolve a cell body name to its PyBullet uid(s).

    Tools (``AT3L``/``AT3R``) live in ``client.tools_puids`` (one uid each);
    rigid bodies (``bar_<id>`` / ``joint_*``) live in ``client.rigid_bodies_puids``
    (a list, one per sub-mesh).

    Returns:
        list[int]: matching PyBullet body uids (empty if the name is unknown).
    """
    client = planner.client
    if name in client.tools_puids:
        return [client.tools_puids[name]]
    return list(client.rigid_bodies_puids.get(name, []))


def set_tool_alpha(planner, tool_name, alpha):
    """Set a tool's opacity in the GUI across all its links (alpha < 1 = see-through)."""
    import pybullet

    cid = planner.client.client_id
    uid = planner.client.tools_puids.get(tool_name)
    if uid is None:
        return
    for link in range(-1, pybullet.getNumJoints(uid, physicsClientId=cid)):
        pybullet.changeVisualShape(
            uid, link, rgbaColor=[0.85, 0.85, 0.6, alpha], physicsClientId=cid)


def _draw_cross(cid, point, *, size=0.001, color=(0, 0, 0), width=2):
    """Draw a 3-axis cross marker at ``point``; returns its debug-item ids."""
    import pybullet

    handles = []
    for axis in range(3):
        half = [0.0, 0.0, 0.0]
        half[axis] = size / 2.0
        p1 = [point[i] - half[i] for i in range(3)]
        p2 = [point[i] + half[i] for i in range(3)]
        handles.append(pybullet.addUserDebugLine(
            p1, p2, lineColorRGB=list(color), lineWidth=width, physicsClientId=cid))
    return handles


def draw_closest_points(planner, name_a, name_b, *, max_distance=0.05):
    """Draw PyBullet's closest-point line between two bodies and report the gap.

    Queries ``p.getClosestPoints`` on the **collision** geometry (the same meshes
    the collision check uses), so it tells a real penetration apart from a
    visual-only overlap or a true gap -- the visual meshes shown in the GUI can
    differ from the collision meshes. The signed ``contactDistance`` is negative
    when the collision shapes actually interpenetrate, positive when separated.

    For each closest pair found within ``max_distance`` it draws a line between
    the two surface points (red = penetration, green = gap), labels it with the
    signed distance in mm, and prints the same. Returns the debug-item handles so
    the caller can clear them after the pause.

    Args:
        planner (PyBulletPlanner): active planner.
        name_a, name_b (str): cell body names (tool id or rigid-body name).
        max_distance (float): query radius in meters; pairs farther than this are
            not reported (raise it to see larger gaps).

    Returns:
        list[int]: PyBullet debug-item ids (lines + text) to remove later.
    """
    import pybullet

    cid = planner.client.client_id
    uids_a = _body_uids(planner, name_a)
    uids_b = _body_uids(planner, name_b)
    if not uids_a or not uids_b:
        print(f"[closest] could not resolve {name_a!r} / {name_b!r}; skipping.")
        return []

    handles = []
    found = False
    for ua in uids_a:
        for ub in uids_b:
            for cp in pybullet.getClosestPoints(ua, ub, max_distance, physicsClientId=cid):
                found = True
                pos_a, pos_b, dist = cp[5], cp[6], cp[8]
                penetrating = dist < 0.0
                color = [1, 0, 0] if penetrating else [0, 1, 0]
                tag = "PENETRATION" if penetrating else "gap"
                print(f"[closest] {name_a} <-> {name_b}: {tag} {dist * 1000.0:+.3f} mm")
                mid = [(pos_a[i] + pos_b[i]) / 2.0 for i in range(3)]
                # Line connecting the two PyBullet contact points + a cross at each.
                handles.append(pybullet.addUserDebugLine(
                    pos_a, pos_b, lineColorRGB=color, lineWidth=4, physicsClientId=cid))
                handles += _draw_cross(cid, pos_a, color=[0,0,0])
                handles += _draw_cross(cid, pos_b, color=[0,0,0])
                handles.append(pybullet.addUserDebugText(
                    f"{dist * 1000.0:+.2f}mm", mid, textColorRGB=color, textSize=1.3,
                    physicsClientId=cid))
    if not found:
        print(f"[closest] {name_a} <-> {name_b}: no points within {max_distance * 1000.0:.0f} mm.")
    return handles


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data_root", nargs="?", default=DEFAULT_DATA_ROOT,
                        help="Parent folder containing per-problem subfolders.")
    parser.add_argument("--problem", default=DEFAULT_PROBLEM,
                        help="Subfolder with BarActions/ + RobotCell.json.")
    parser.add_argument("--bar-action", default=DEFAULT_BAR_ACTION,
                        help="BarAction filename inside <root>/<problem>/BarActions/.")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--no-collision", action="store_true",
                        help="Solve IK without collision checking (reachability-only diagnostic).")
    parser.add_argument("--cell", default=None,
                        help="Path to RobotCell.json (default: <root>/<problem>/RobotCell.json).")
    args = parser.parse_args()

    problem_dir = os.path.join(os.path.abspath(args.data_root), args.problem)
    action_path = os.path.join(problem_dir, "BarActions", args.bar_action)
    cell_path = args.cell or os.path.join(problem_dir, "RobotCell.json")
    for label, path in (("BarAction", action_path), ("RobotCell.json", cell_path)):
        if not os.path.isfile(path):
            print(f"[X] missing {label}: {path}")
            return 1

    from compas.data import json_load
    import pybullet_planning as pp

    from core import config as _config
    from core import ik_keyframe
    from core import robot_cell as _robot_cell

    print(f"[load] RobotCell    <- {cell_path}")
    rcell = json_load(cell_path)
    print(f"[load] BarAction    <- {action_path}")
    action = json_load(action_path)
    print(f"  action_id     : {action.action_id}")
    print(f"  movements     : {len(action.movements)}")

    # Pick the three movements we re-solve (M4 is the free home move; not IK here).
    by_role: dict = {}
    for mv in action.movements:
        role = match_role(mv)
        if role in ("M1", "M2", "M3"):
            by_role.setdefault(role, mv)
    missing = [r for r in ("M1", "M2", "M3") if r not in by_role]
    if missing:
        print(f"[X] action is missing movements {missing}")
        return 1

    print(f"\n[pb] starting PyBullet ({'GUI' if args.gui else 'DIRECT'})")
    _client, planner = start_planner(rcell, use_gui=args.gui)

    # Prime robot_cell's sticky so `solve_dual_arm_ik` -> `_ensure_dual_arm_cell_loaded`
    # is a no-op against THIS planner+cell instead of reloading a Rhino-cached one.
    _robot_cell._STICKY[_robot_cell._STICKY_ROBOT_CELL] = rcell
    _robot_cell._STICKY[_robot_cell._STICKY_CURRENT_CELL_KIND] = "dual_arm"

    try:
        # All three movements share one base frame; read it off M1's start_state.
        m1 = by_role["M1"]
        fill_missing_config(
            m1.start_state, rcell, _config.HOME_CONFIG_LEFT, _config.HOME_CONFIG_RIGHT,
        )
        base_frame_mm = ik_keyframe.frame_to_mm4(m1.start_state.robot_base_frame)

        # M1 starts at HOME -- exactly like rs_ik_keyframe (no warm-start shortcut).
        # solve_dual_arm_ik's cold random restarts are what make M1 reachable; M2/M3
        # then warm-start off the chain. This keeps the headless a faithful proxy.
        ordered_movements = [("M1", by_role["M1"]), ("M2", by_role["M2"]), ("M3", by_role["M3"])]
        print("\n[ik] solving M1->M2->M3 chain (same code path as rs_ik_keyframe) ...")
        solved = ik_keyframe.solve_keyframe_chain(
            planner, ordered_movements, base_frame_mm,
            check_collision=not args.no_collision, verbose_pairs=not args.no_collision,
        )
        if solved is None:
            print("[X] IK chain FAILED (one of M1/M2/M3 had no collision-free solution).")
            return 2
        print("[OK] full M1->M2->M3 IK chain solved.")

        if not args.gui:
            print("[step] skipped (run with --gui to step through the poses).")
            return 0

        # Tint the active bar so it is easy to track as the arms move.
        active_bar_id = getattr(action, "active_bar_id", "") or ""
        bar_rb = _resolve_bar_rb_name(rcell, active_bar_id)
        if bar_rb is not None:
            color_rigid_body(planner, bar_rb, rgba=(0.1, 0.4, 1.0, 0.7))

        # --- contact-penetration visualization (disabled; kept for re-enabling) ---
        # Ghosts the joint halves + tools and draws the PyBullet closest-point
        # cross/line probe at each pose. Commented out now that the tool<->bar
        # overlap is understood (coarse collision-mesh artefact, ~1-3 mm).
        # active_joint_rbs = sorted(
        #     k for k, rb in m2.start_state.rigid_body_states.items()
        #     if k.startswith("joint_") and rb.attached_to_link
        # )
        # for jrb in active_joint_rbs:
        #     color_rigid_body(planner, jrb, rgba=(0.7, 0.7, 0.7, 0.3))
        # if active_joint_rbs:
        #     print(f"[viz] half-transparent joints on {active_bar_id}: {active_joint_rbs}")
        # for tool in sorted(planner.client.tools_puids):
        #     set_tool_alpha(planner, tool, 0.7)
        # import pybullet
        # tool_names = sorted(planner.client.tools_puids)

        # Step through the three freshly-solved poses.
        for role, label in (("M1", "approach"), ("M2", "assembled"), ("M3", "retreat")):
            state = solved[role]
            with pp.LockRenderer(False):
                planner.set_robot_cell_state(state)
            check_collision(planner, state, label=f"{role} ({label})")
            # handles = []
            # if bar_rb is not None:
            #     for tool in tool_names:
            #         handles += draw_closest_points(planner, tool, bar_rb, max_distance=0.05)
            pp.wait_if_gui(f"{role}: {label} pose -- Enter to continue")
            # for h in handles:
            #     pybullet.removeUserDebugItem(h, physicsClientId=planner.client.client_id)
        print("[step] done.")
        return 0
    finally:
        try:
            pp.disconnect()
        except Exception as exc:
            print(f"[pb] disconnect raised ({exc}); continuing.")


if __name__ == "__main__":
    raise SystemExit(main())
