"""Headless verifier for BarAssemblyAction JSONs under `<data_root>/BarActions/`.

Loads every `BarAssemblyAction` under `<data_root>/BarActions/*.json`, walks
each movement, pushes its `start_state` through `planner.set_robot_cell_state`
(+ collision check) and reports.

For M4 (`RoboticFreeMovement`), `start_state.robot_configuration is None`
by design (the planner fills it from M3's solved retreat). The verifier
substitutes `HOME_CONFIG_LEFT/RIGHT` from `core.config` for the test only.

CLI:
    python tests/debug_load_bar_action.py [<data_root>] [--action <name> ...] [--cell <path>] [--gui] [--no-collision]

Default `<data_root>` is the design-study folder; default action set is every
`*.json` under `<data_root>/BarActions/`.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (SCRIPTS_DIR, TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()

SUBMODULE_PATH = os.path.join(REPO_ROOT, "external", "compas_fab", "src")
if os.path.isdir(SUBMODULE_PATH) and SUBMODULE_PATH not in sys.path:
    sys.path.insert(0, SUBMODULE_PATH)


DEFAULT_DATA_ROOT = (
    r"D:\GDrive\.shortcut-targets-by-id\1kAas3Pk4TrTZiJ9VmlCxLEf3oRIuSqM5"
    r"\2025-03 Husky Assembly\data_design_study\2026-05-14_foc_demo_reduced"
)
LEFT_GROUP = "base_left_arm_manipulator"
RIGHT_GROUP = "base_right_arm_manipulator"


# ---------------------------------------------------------------------------
# Local helpers (cell load, planner boot, per-state verify)
# ---------------------------------------------------------------------------


def _find_robot_cell_path(action_path: str) -> str | None:
    """Walk up from `action_path` (1 or 2 dirs) looking for `RobotCell.json`."""
    action_dir = os.path.dirname(os.path.abspath(action_path))
    for cand in (action_dir, os.path.dirname(action_dir), os.path.dirname(os.path.dirname(action_dir))):
        rc = os.path.join(cand, "RobotCell.json")
        if os.path.isfile(rc):
            return rc
    rc = os.path.join(DEFAULT_DATA_ROOT, "RobotCell.json")
    if os.path.isfile(rc):
        return rc
    return None


def _start_planner(rcell, *, use_gui: bool = False):
    """Boot a PyBullet client + planner with `rcell` loaded. Returns (client, planner)."""
    from compas_fab.backends import PyBulletClient, PyBulletPlanner
    import pybullet_planning as pp

    client = PyBulletClient(connection_type="gui" if use_gui else "direct", verbose=True)
    client.__enter__()
    pp.set_client(client.client_id)
    pp.CLIENTS[client.client_id] = True if use_gui else None
    planner = PyBulletPlanner(client)
    t0 = time.time()
    with pp.LockRenderer(False):
        planner.set_robot_cell(rcell)
    print(f"[pb] set_robot_cell: {time.time() - t0:.2f}s")
    return client, planner


def _verify_state(state, planner, *, label: str = "", run_collision_check: bool = True) -> bool:
    """Push `state` through `planner.set_robot_cell_state` and run a collision check."""
    from compas_fab.backends import CollisionCheckError
    import pybullet as pb
    import pybullet_planning as pp

    tag = f" [{label}]" if label else ""
    t0 = time.time()
    with pp.LockRenderer(False):
        planner.set_robot_cell_state(state)
    print(f"[pb]{tag} set_robot_cell_state: {time.time() - t0:.2f}s")

    n_bodies = pb.getNumBodies(physicsClientId=planner.client.client_id)
    print(f"[pb]{tag} world has {n_bodies} body(ies)")

    rcell = planner.client.robot_cell
    for grp in (LEFT_GROUP, RIGHT_GROUP):
        try:
            tool = rcell.get_attached_tool(state, grp)
        except Exception:
            tool = None
        print(f"  group {grp!r}: attached tool = {getattr(tool, 'name', None)}")

    if not run_collision_check:
        print(f"[skip]{tag} collision check disabled.")
        return True

    print(f"[check]{tag} planner.check_collision(state, full_report=True) ...")
    try:
        planner.check_collision(state, options={"full_report": True, "verbose": False})
        print(f"[OK]{tag} no collisions reported.")
        return True
    except CollisionCheckError as exc:
        pairs = list(getattr(exc, "collision_pairs", []) or [])
        print(f"[!!]{tag} {len(pairs)} colliding pair(s):")
        for line in str(exc).splitlines():
            print(f"     {line}")
        return False


def _resolve_action_paths(data_root: str, names):
    """Return abs paths to the actions to verify, sorted by filename."""
    actions_dir = os.path.join(data_root, "BarActions")
    if names:
        return [os.path.join(actions_dir, n) for n in names]
    return sorted(glob.glob(os.path.join(actions_dir, "*.json")))


def _fill_missing_config(state, rcell, home_left, home_right, left_group: str, right_group: str) -> None:
    """If `state.robot_configuration is None`, fill it with HOME_CONFIG (test-only)."""
    if state is None or state.robot_configuration is not None:
        return
    cfg = rcell.zero_full_configuration()
    left_names = list(rcell.get_configurable_joint_names(left_group))
    right_names = list(rcell.get_configurable_joint_names(right_group))
    for name, val in zip(left_names, home_left):
        cfg[name] = float(val)
    for name, val in zip(right_names, home_right):
        cfg[name] = float(val)
    state.robot_configuration = cfg


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data_root", nargs="?", default=DEFAULT_DATA_ROOT,
                        help="Folder containing BarActions/<bar_id>.json + RobotCell.json.")
    parser.add_argument("--action", action="append", default=None,
                        help="BarAction filename inside <data_root>/BarActions/. "
                             "Repeatable; if omitted, all *.json in that folder are walked.")
    parser.add_argument(
        "--cell", default=None,
        help="Path to RobotCell.json (default: <data_root>/RobotCell.json).",
    )
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--no-collision", action="store_true")
    args = parser.parse_args()

    data_root = os.path.abspath(args.data_root)
    if not os.path.isdir(data_root):
        print(f"[X] missing data_root: {data_root}")
        return 1

    action_paths = _resolve_action_paths(data_root, args.action)
    if not action_paths:
        print(f"[X] no actions found under {os.path.join(data_root, 'BarActions')}")
        return 1
    for p in action_paths:
        if not os.path.isfile(p):
            print(f"[X] missing: {p}")
            return 1

    cell_path = args.cell or os.path.join(data_root, "RobotCell.json")
    if not os.path.isfile(cell_path):
        cell_path = _find_robot_cell_path(action_paths[0]) or cell_path
    if not os.path.isfile(cell_path):
        print(f"[X] could not locate RobotCell.json (tried {cell_path}; --cell <path> to override)")
        return 1

    from compas.data import json_load
    import pybullet_planning as pp

    print(f"[load] RobotCell    <- {cell_path}")
    rcell = json_load(cell_path)
    print(f"  robot model   : {getattr(rcell.robot_model, 'name', '<?>')}")
    print(f"  tool models   : {sorted(rcell.tool_models.keys())}")
    print(f"  rigid bodies  : {len(rcell.rigid_body_models)}")

    print(f"[load] {len(action_paths)} BarAction file(s):")
    actions = []
    for p in action_paths:
        a = json_load(p)
        actions.append((os.path.basename(p), a))
        seq = getattr(a, "assembly_seq", []) or []
        active_id = getattr(a, "active_bar_id", "<?>")
        try:
            idx = seq.index(active_id)
        except (ValueError, AttributeError):
            idx = -1
        print(f"   - {os.path.basename(p)} "
              f"(active_bar_id={active_id}, "
              f"assembly_index={idx}/{len(seq)}, "
              f"movements={len(getattr(a, 'movements', []))})")

    from core import config as _config

    print(f"\n[pb] starting PyBullet ({'GUI' if args.gui else 'DIRECT'})")
    _client, planner = _start_planner(rcell, use_gui=args.gui)

    rc = 0
    try:
        for a_idx, (a_name, action) in enumerate(actions):
            print(f"\n###### [{a_idx + 1}/{len(actions)}] action: {a_name} ######")
            for idx, mv in enumerate(action.movements):
                mv_tag = f"{idx + 1}/{len(action.movements)} {type(mv).__name__} {mv.movement_id}"
                print(f"\n=== [{mv_tag}] ===")
                state = mv.start_state
                if state is None:
                    print(f"[skip] {mv.movement_id}: start_state is None.")
                    continue
                _fill_missing_config(
                    state, rcell,
                    _config.HOME_CONFIG_LEFT, _config.HOME_CONFIG_RIGHT,
                    _config.LEFT_GROUP, _config.RIGHT_GROUP,
                )
                ok = _verify_state(
                    state, planner,
                    label=mv.movement_id,
                    run_collision_check=not args.no_collision,
                )
                if not ok:
                    rc = 2
                if args.gui:
                    print(f"[gui] inspect {mv.movement_id}; press <enter> in GUI to continue.")
                pp.wait_if_gui()
    finally:
        try:
            pp.disconnect()
        except Exception as exc:
            print(f"[pb] disconnect raised ({exc}); continuing.")

    return rc


if __name__ == "__main__":
    raise SystemExit(main())
