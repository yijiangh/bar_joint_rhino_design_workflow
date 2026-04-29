"""Debug a captured RSIKSupportKeyframe scenario, pure-Python (no Rhino).

Run from the repo root:

    python tests/debug_ik_support_collisions.py tests/captures/<file>_support_*.json

What it does:

    1. Boots a fresh PyBullet GUI session and loads the SUPPORT cell
       (single-arm Husky-Alice + SupportGripper Robotiq + DualArm dual-arm-as-tool obstacle).
    2. Configures the DualArm tool's pose + internal joint configuration from
       the capture's `assembled` block (the just-assembled bar's
       ik_assembly payload).
    3. Sanity-prints the support cell's tool_models vs. template state's
       tool_states + the planner's currently-loaded cell-kind. Mismatches
       in cell tools vs. state tools are the symptom of a missed
       cross-module cell swap (see lessons.md).
    4. Solves single-arm IK twice for the captured tool0 target + base:
         (a) check_collision=True  -> production path. Tells us if IK
             accepts or rejects this config.
         (b) check_collision=False -> always returns a geometric config
             (when one exists). We push it into PyBullet so we can see
             what's clipping even if (a) rejected.
    5. Calls `performCollisionDetection` and enumerates contact pairs;
       highlights colliding links RED via `changeVisualShape`.
    6. Keeps the GUI open until the user closes the window.

Use this when:
    * You see "The tools in the cell state do not match the tools in the
      robot cell" — usually means the planner has a different cell loaded
      than the state expects (cell-swap bug).
    * IK rejects unexpectedly.
    * IK accepts but the support arm visibly clips the dual-arm.

Companion to `tests/debug_ik_collisions.py` (dual-arm assembly side).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (SCRIPTS_DIR, TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


_RED = [1.0, 0.2, 0.2, 0.85]
_ENV_GREEN = [0.235, 0.7, 0.235, 0.85]


def _decode_or_none(b):
    if b is None:
        return None
    if isinstance(b, bytes):
        return b.decode("utf-8", errors="replace")
    return str(b)


def _body_label(p, body_id):
    info = p.getBodyInfo(body_id)
    if not info:
        return f"body[{body_id}]"
    base_link_name = _decode_or_none(info[0]) or ""
    body_name = _decode_or_none(info[1]) or ""
    return body_name or base_link_name or f"body[{body_id}]"


def _link_label(p, body_id, link_index):
    if link_index == -1:
        info = p.getBodyInfo(body_id)
        return _decode_or_none(info[0]) if info else "<base>"
    try:
        ji = p.getJointInfo(body_id, link_index)
        return _decode_or_none(ji[12]) or f"link[{link_index}]"
    except Exception:
        return f"link[{link_index}]"


def _safe_change_color(p, body_id, link_index, color, client_id):
    try:
        p.changeVisualShape(
            objectUniqueId=body_id,
            linkIndex=link_index,
            rgbaColor=color,
            physicsClientId=client_id,
        )
    except Exception as exc:
        print(f"  (changeVisualShape failed body={body_id} link={link_index}: {exc})")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", help="Path to a support-IK capture JSON in tests/captures/.")
    parser.add_argument("--headless", action="store_true",
                        help="Connect PyBullet in DIRECT mode (no GUI). Useful for CI.")
    parser.add_argument("--simulate-prior-dual-arm", action="store_true",
                        help="Reproduce a Rhino session that ran RSIKKeyframe BEFORE "
                        "RSIKSupportKeyframe: load the dual-arm cell into the planner first, "
                        "then run support IK without swapping. Use to verify the cell-swap bug.")
    args = parser.parse_args()

    if not os.path.isfile(args.capture):
        print(f"[X] capture not found: {args.capture}")
        return 1

    from core import capture_io, env_collision  # noqa: F401
    capture = capture_io.load_capture(args.capture)

    from core import robot_cell as rc
    from core import robot_cell_support as rcs
    import pybullet as p

    if rc.is_pb_running():
        rc.stop_pb_client()

    use_gui = not args.headless
    print(f"debug_ik_support_collisions: launching PyBullet ({'GUI' if use_gui else 'DIRECT'}) ...")
    rc.start_pb_client(use_gui=use_gui, verbose=True)
    _client, planner = rc.get_planner()
    client_id = planner.client.client_id

    if args.simulate_prior_dual_arm:
        print("(Repro mode: planner has dual-arm cell loaded; testing support IK without swap.)")

    captures_dir = os.path.dirname(os.path.abspath(args.capture))
    rcell = capture_io.load_robot_cell_ref(captures_dir, capture["robot_cell_ref"])
    rc._STICKY[rcs._STICKY_SUPPORT_CELL] = rcell
    planner.set_robot_cell(rcell)
    rc._STICKY[rc._STICKY_CURRENT_CELL_KIND] = "support"

    template = capture_io.deserialize_state(capture["initial_state"])
    base_frame = template.robot_base_frame
    base_origin = base_frame.point
    tool0 = np.asarray(capture["ik_targets"]["support"]["tool0_world_mm"], dtype=float)
    include_self = capture["ik_options"]["include_self"]
    include_env = capture["ik_options"]["include_env"]

    base_frame_mm = np.eye(4, dtype=float)
    base_frame_mm[:3, 0] = base_frame.xaxis
    base_frame_mm[:3, 1] = base_frame.yaxis
    base_frame_mm[:3, 2] = base_frame.zaxis
    base_frame_mm[:3, 3] = np.asarray(base_frame.point) * 1000.0

    print(
        f"debug_ik_support_collisions: phase=support, "
        f"base_xy=({base_origin[0] * 1000:.1f}, {base_origin[1] * 1000:.1f}) mm, "
        f"include_self={include_self}, include_env={include_env}"
    )

    n_bar = sum(1 for name in rcell.rigid_body_models.keys() if name.startswith(env_collision.ENV_RB_BAR_PREFIX))
    n_joint = sum(1 for name in rcell.rigid_body_models.keys() if name.startswith(env_collision.ENV_RB_JOINT_PREFIX))
    print(f"debug_ik_support_collisions: env: {n_bar} built bars + {n_joint} joints registered as rigid bodies")

    print(f"\n=== Support cell tool_models: {sorted(rcell.tool_models.keys())}")
    print(f"=== Template tool_states:    {sorted(template.tool_states.keys())}")
    print(f"=== Planner cell-kind sticky: {rc._STICKY.get(rc._STICKY_CURRENT_CELL_KIND)!r} "
          "(should be 'support' before solve_support_ik runs)")

    print("\n--- IK with check_collision=True (production path) ---")
    state_with = rcs.solve_support_ik(planner, template, base_frame_mm, tool0, check_collision=True)
    if state_with is not None:
        print("[OK] support IK PASSED with collision check.")
    else:
        print("[X] support IK REJECTED with collision check.")

    print("\n--- IK with check_collision=False (always returns a config when reachable) ---")
    state_open = rcs.solve_support_ik(planner, template, base_frame_mm, tool0, check_collision=False)
    if state_open is None:
        print("[X] IK failed even without collision check — target geometrically unreachable.")
        if use_gui:
            print("Closing PyBullet in 3 s ...")
            time.sleep(3)
        rc.stop_pb_client()
        return 1

    rcs.set_cell_state(planner, state_open)

    p.performCollisionDetection(physicsClientId=client_id)
    cps = p.getContactPoints(physicsClientId=client_id)

    pairs: dict = {}
    for cp in cps:
        body_a, body_b, link_a, link_b = cp[1], cp[2], cp[3], cp[4]
        a = (body_a, link_a)
        b = (body_b, link_b)
        key = tuple(sorted([a, b]))
        pairs[key] = pairs.get(key, 0) + 1

    n = p.getNumBodies(physicsClientId=client_id)
    bodies = [(p.getBodyUniqueId(i, physicsClientId=client_id), None) for i in range(n)]
    bodies = [(uid, _body_label(p, uid)) for uid, _ in bodies]
    print(f"\n=== Bodies in PyBullet world: {len(bodies)} ===")
    for uid, label in bodies:
        print(f"  - body[{uid}] {label}")

    for uid, label in bodies:
        if label.startswith(env_collision.ENV_RB_BAR_PREFIX) or label.startswith(env_collision.ENV_RB_JOINT_PREFIX):
            _safe_change_color(p, uid, -1, _ENV_GREEN, client_id)

    pb_client = planner.client
    print(f"\n=== compas_fab.client.unordered_disabled_collisions: "
          f"{len(pb_client.unordered_disabled_collisions)} pairs ===")
    for pair in list(pb_client.unordered_disabled_collisions)[:10]:
        print(f"  - {sorted(pair)}")
    if len(pb_client.unordered_disabled_collisions) > 10:
        print(f"  ... +{len(pb_client.unordered_disabled_collisions) - 10} more")

    print("\n=== tool_states touch_links (state_open) ===")
    for tool_name, ts in state_open.tool_states.items():
        print(f"  - {tool_name}: attached_to_group={ts.attached_to_group} "
              f"touch_links={list(ts.touch_links)} is_hidden={ts.is_hidden}")

    print("\n=== compas_fab.check_collision(state_open, verbose=True, full_report=True) ===")
    try:
        planner.check_collision(state_open, options={"verbose": True, "full_report": True})
        print("[OK] check_collision passed with no collisions (production IK should accept).")
    except Exception as exc:
        print(f"[X] check_collision raised:\n{exc}")

    print(f"\n=== Active contact pairs at the no-check config: {len(pairs)} ===")
    if not pairs:
        print("  (no contact points)")
    else:
        for (a, b), count in pairs.items():
            ba, la = a
            bb, lb = b
            la_s = f"{_body_label(p, ba)} / {_link_label(p, ba, la)}"
            lb_s = f"{_body_label(p, bb)} / {_link_label(p, bb, lb)}"
            print(f"  - {la_s}   <->   {lb_s}   ({count} pt)")
            _safe_change_color(p, ba, la, _RED, client_id)
            _safe_change_color(p, bb, lb, _RED, client_id)

    print()
    if state_with is None and pairs:
        print("Diagnosis: production IK rejected AND PyBullet sees clipping at the no-check")
        print("           config. The pairs above are likely what's blocking IK.")
    elif state_with is not None and pairs:
        print("Diagnosis: production IK ACCEPTED but PyBullet still sees clipping. The pairs")
        print("           above are NOT being treated as collisions by compas_fab.")
    elif state_with is not None and not pairs:
        print("Diagnosis: IK accepted, no contacts. Healthy result.")
    else:
        print("Diagnosis: IK rejected, no contacts found. Likely seed mismatch or cell-tools")
        print("           mismatch (planner had wrong cell loaded). Check the sticky line above.")

    if use_gui:
        print("\nPyBullet GUI is open. Highlighted (red) links are the colliding ones.")
        print("Rotate / pan to inspect. Close the GUI window to exit.")
        try:
            while p.isConnected(physicsClientId=client_id):
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
    else:
        rc.stop_pb_client()
    return 0


if __name__ == "__main__":
    sys.exit(main())
