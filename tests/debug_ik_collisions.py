"""Debug IK collisions on a captured RSIKKeyframe scenario, pure-Python (no Rhino).

Run from the repo root:

    python tests/debug_ik_collisions.py tests/captures/<file>.json
    python tests/debug_ik_collisions.py tests/captures/<file>.json --phase approach

What it does:

    1. Boots a fresh PyBullet *GUI* session and loads the dual-arm Husky cell
       with tool models attached (same path as RSIKKeyframe).
    2. Solves the dual-arm IK twice for the captured target:
         (a) check_collision = True  -> the production code path. Tells us
             whether the IK we'd use in Rhino accepts or rejects this config.
         (b) check_collision = False -> always returns a config (when one
             geometrically exists). We push this into PyBullet so we can see
             *what was actually clipping* even if (a) rejected it.
    3. Calls `performCollisionDetection` and enumerates every active contact
       pair: body name + link name on both sides, contact-point count.
    4. Colors every colliding link RED via `changeVisualShape` so the GUI
       shows exactly which parts overlap.
    5. Keeps the GUI open until the user closes the window.

Use this when:
    * IK rejects (you expected it to succeed) -> see what compas_fab thinks
      is in the way at the no-check config.
    * IK accepts (but you can see it clip in Rhino) -> the contact pairs
      tell you whether PyBullet sees the clip; if it does but compas_fab
      didn't reject, the bug is in compas_fab's allowed-pair logic.

Caveat: contact points are reported at the *no-check* IK config, which can
differ from the seed compas_fab tried with collision checking on. They are
a strong indicator, not exact reproduction. For deeper diagnosis, lower
`IK_MAX_RESULTS` to 1 in `core/config.py` so the solver only tries one seed.
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

# Make Rhino's `scaffolding_env` site-env (where `# r:` packages live) reachable
# from a plain `python.exe` invocation. No-op inside Rhino.
from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


_RED = [1.0, 0.2, 0.2, 0.85]


def _unit(v):
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("zero vector")
    return v / n


def _tool0_from_ocf(ocf, block_name, side, table):
    if block_name not in table or side not in table[block_name]:
        raise RuntimeError(
            f"MALE_JOINT_OCF_TO_TOOL0 missing entry [{block_name}][{side}]. "
            "Re-run RSExportJointTool0TF in Rhino."
        )
    return np.asarray(ocf, dtype=float) @ np.asarray(table[block_name][side], dtype=float)


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


def _enumerate_bodies(p, client_id):
    """List every body in the simulation by uid + label."""
    n = p.getNumBodies(physicsClientId=client_id)
    out = []
    for i in range(n):
        uid = p.getBodyUniqueId(i, physicsClientId=client_id)
        label = _body_label(p, uid)
        out.append((uid, label))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", help="Path to a JSON capture file in tests/captures/.")
    parser.add_argument("--phase", choices=["final", "approach"], default="final",
                        help="Which target to inspect (default: final).")
    parser.add_argument("--headless", action="store_true",
                        help="Connect PyBullet in DIRECT mode (no GUI, no blocking on close); "
                        "useful for automated/CI runs.")
    args = parser.parse_args()

    if not os.path.isfile(args.capture):
        print(f"[X] capture not found: {args.capture}")
        return 1

    with open(args.capture, "r", encoding="utf-8") as stream:
        capture = json.load(stream)

    from core import config
    from core import robot_cell as rc
    import pybullet as p

    if rc.is_pb_running():
        rc.stop_pb_client()

    use_gui = not args.headless
    print(f"debug_ik_collisions: launching PyBullet ({'GUI' if use_gui else 'DIRECT'}) ...")
    rc.start_pb_client(use_gui=use_gui, verbose=True)
    rcell = rc.get_or_load_robot_cell()
    template = rc.default_cell_state()
    _client, planner = rc.get_planner()
    client_id = planner.client.client_id

    # Build IK targets exactly the same way RSIKKeyframe does.
    ocf_l = np.array(capture["left"]["ocf_world_mm"], dtype=float)
    ocf_r = np.array(capture["right"]["ocf_world_mm"], dtype=float)
    base = np.array(capture["base_frame_world_mm"], dtype=float)
    lm = float(capture["lm_distance_mm"])
    table = config.MALE_JOINT_OCF_TO_TOOL0
    tool0_L = _tool0_from_ocf(ocf_l, capture["left"]["block_name"], capture["left"]["arm_side"], table)
    tool0_R = _tool0_from_ocf(ocf_r, capture["right"]["block_name"], capture["right"]["arm_side"], table)
    if args.phase == "approach":
        z_avg = (ocf_l[:3, 2] + ocf_r[:3, 2]) / 2.0
        offset = -_unit(z_avg) * lm
        tool0_L[:3, 3] += offset
        tool0_R[:3, 3] += offset
    print(
        f"debug_ik_collisions: phase={args.phase}, "
        f"base_xy=({base[0, 3]:.1f}, {base[1, 3]:.1f}) mm, "
        f"include_self={capture['include_self_collision']}, "
        f"include_env={capture['include_env_collision']}"
    )

    print("\n--- IK with check_collision=True (production path) ---")
    state_with = rc.solve_dual_arm_ik(planner, template, base, tool0_L, tool0_R, check_collision=True)
    if state_with is not None:
        print(f"[OK] {args.phase} IK PASSED with collision check.")
    else:
        print(f"[X] {args.phase} IK REJECTED with collision check.")

    print("\n--- IK with check_collision=False (always returns a config) ---")
    state_open = rc.solve_dual_arm_ik(planner, template, base, tool0_L, tool0_R, check_collision=False)
    if state_open is None:
        print("[X] IK failed even WITHOUT collision check; target is geometrically unreachable.")
        print("    Cannot inspect collisions. Closing.")
        return 1

    rc.set_cell_state(planner, state_open)

    # Force pybullet to compute current contacts at this configuration.
    p.performCollisionDetection(physicsClientId=client_id)
    cps = p.getContactPoints(physicsClientId=client_id)

    # Group by sorted (body, link) pair so reverse duplicates collapse.
    pairs: dict = {}
    for cp in cps:
        body_a, body_b, link_a, link_b = cp[1], cp[2], cp[3], cp[4]
        a = (body_a, link_a)
        b = (body_b, link_b)
        key = tuple(sorted([a, b]))
        pairs[key] = pairs.get(key, 0) + 1

    bodies = _enumerate_bodies(p, client_id)
    print(f"\n=== Bodies in PyBullet world: {len(bodies)} ===")
    for uid, label in bodies:
        print(f"  - body[{uid}] {label}")

    # Introspect compas_fab's allowed-collision wiring.
    pb_client = planner.client
    print(f"\n=== compas_fab.client.unordered_disabled_collisions: {len(pb_client.unordered_disabled_collisions)} pairs ===")
    for pair in list(pb_client.unordered_disabled_collisions)[:10]:
        print(f"  - {sorted(pair)}")
    if len(pb_client.unordered_disabled_collisions) > 10:
        print(f"  ... +{len(pb_client.unordered_disabled_collisions) - 10} more")

    print("\n=== tool_states touch_links (in state_open used for inspection) ===")
    for tool_name, ts in state_open.tool_states.items():
        print(f"  - {tool_name}: attached_to_group={ts.attached_to_group} "
              f"touch_links={list(ts.touch_links)} is_hidden={ts.is_hidden}")

    # Run compas_fab's own check_collision with verbose=True so we see exactly
    # which CC.x step (link-link / link-tool / etc.) flags or skips each pair.
    print("\n=== compas_fab.check_collision(state_open, verbose=True, full_report=True) ===")
    try:
        planner.check_collision(state_open, options={"verbose": True, "full_report": True})
        print("[OK] check_collision passed with no collisions (production IK should accept).")
    except Exception as exc:
        # CollisionCheckError carries a detailed message; print as-is.
        print(f"[X] check_collision raised:\n{exc}")

    print(f"\n=== Active contact pairs at the no-check config: {len(pairs)} ===")
    if not pairs:
        print("  (no contact points)")
        if state_with is None:
            print("  IK was REJECTED but PyBullet shows no contacts at this config.")
            print("  Likely cause: the solver tried a different seed that did clip,")
            print("  not the no-check seed shown here. Re-run with IK_MAX_RESULTS=1")
            print("  in scripts/core/config.py to force a single attempt and align them.")
    else:
        for (a, b), count in pairs.items():
            body_a_id, link_a_idx = a
            body_b_id, link_b_idx = b
            a_label = f"{_body_label(p, body_a_id)} / {_link_label(p, body_a_id, link_a_idx)}"
            b_label = f"{_body_label(p, body_b_id)} / {_link_label(p, body_b_id, link_b_idx)}"
            print(f"  - {a_label}   <->   {b_label}   ({count} pt)")
            _safe_change_color(p, body_a_id, link_a_idx, _RED, client_id)
            _safe_change_color(p, body_b_id, link_b_idx, _RED, client_id)

    print()
    if state_with is None and pairs:
        print("Diagnosis: production IK rejected this target AND PyBullet sees clipping at the")
        print("           no-check config. The pairs above are likely what's blocking IK.")
        print("           Cross-reference with the SRDF disable_collisions list to confirm")
        print("           none of these *should* be ignored.")
    elif state_with is not None and pairs:
        print("Diagnosis: production IK ACCEPTED this target but PyBullet still sees clipping.")
        print("           The pairs above are NOT being treated as collisions by compas_fab.")
        print("           Either an SRDF entry is incorrectly disabling them, or the tool/")
        print("           rigid-body plumbing isn't registering them as collidable.")
    elif state_with is not None and not pairs:
        print("Diagnosis: IK accepted, no contacts. Healthy result.")
    else:
        print("Diagnosis: IK rejected, no contacts found. Likely seed mismatch (see above).")

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
