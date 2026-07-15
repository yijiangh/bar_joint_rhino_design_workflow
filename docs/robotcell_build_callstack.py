"""Generate `docs/robotcell_build_callstack.png`.

Call-stack view of stages 1 (static RobotCell) and 2 (per-bar template
state) from `robotcell_state_lifecycle.png`, framed as a Rhino workflow:
which toolbar button the user clicks, and how the call propagates down into
`core.robot_cell` / `core.ik_collision_setup` / `core.bar_action`.

Each panel is one click. `*` marks where a durable object is created or a
cell/state is mutated. Line numbers are `<module>.py:<line>` at time of writing.

Run with the plain (non-Rhino) Python that has matplotlib:
    python docs/robotcell_build_callstack.py
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch


C_STATIC   = ("#e8f0fb", "#2f6fb0")   # blue   -- stage 1 (static cell)
C_TEMPLATE = ("#fdf0e0", "#d98a2b")   # orange -- stage 2 (template state)
C_NOTE     = ("#f4f6f8", "#4a5568")
INK        = "#222222"

LINE_H = 1.62          # data-units per text line
HEADER_H = 2.6
PAD_TOP = 1.1
PAD_BOT = 1.2
BODY_FS = 9.6          # monospace body font size
X = 2.0
W = 66.0               # call-tree panel width
NOTE_X = 70.0
NOTE_W = 28.0


def panel(ax, y_top, header, sub, lines, colors, note):
    """Draw one click panel (call tree) + its right-side 'PRODUCES' note.

    Returns the y coordinate of the panel bottom (for stacking)."""
    face, edge = colors
    h = PAD_TOP + HEADER_H + len(lines) * LINE_H + PAD_BOT
    y = y_top - h

    ax.add_patch(FancyBboxPatch(
        (X, y), W, h, boxstyle="round,pad=0.15,rounding_size=0.5",
        linewidth=1.9, edgecolor=edge, facecolor=face, zorder=2))
    ax.text(X + 0.7, y_top - PAD_TOP, header, ha="left", va="top",
            fontsize=12.5, fontweight="bold", color=edge, zorder=3)
    ax.text(X + W - 0.7, y_top - PAD_TOP - 0.15, sub, ha="right", va="top",
            fontsize=9, style="italic", color=edge, zorder=3)
    ax.text(X + 0.9, y_top - PAD_TOP - HEADER_H, "\n".join(lines),
            ha="left", va="top", fontsize=BODY_FS, family="monospace",
            color=INK, zorder=3, linespacing=1.32)

    # Right-side note box, vertically centred on the panel.
    nh = min(h, PAD_TOP + PAD_BOT + len(note) * 2.05)
    ny = y + (h - nh) / 2.0
    ax.add_patch(FancyBboxPatch(
        (NOTE_X, ny), NOTE_W, nh, boxstyle="round,pad=0.12,rounding_size=0.4",
        linewidth=1.4, edgecolor=C_NOTE[1], facecolor=C_NOTE[0], zorder=2))
    ax.text(NOTE_X + NOTE_W / 2, ny + nh - 0.7, "PRODUCES", ha="center",
            va="top", fontsize=9, fontweight="bold", color=C_NOTE[1], zorder=3)
    ax.text(NOTE_X + 0.6, ny + nh - 2.4, "\n".join(note), ha="left", va="top",
            fontsize=8.7, color=INK, zorder=3, linespacing=1.35)
    return y


def main():
    fig, ax = plt.subplots(figsize=(24, 19))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    ax.text(50, 98.6,
            "Building the RobotCell + template state  —  the Rhino click-to-call-stack",
            ha="center", va="top", fontsize=18, fontweight="bold", color=INK)
    ax.text(50, 95.6,
            "One-time per session: RSPBStart.   After any geometry edit: RSRebuildRobotCell.   "
            "Then per bar: RSIKKeyframe / RSShowIK (builds the template state).",
            ha="center", va="top", fontsize=10.5, style="italic", color="#555")

    y = 93.0

    # ---- CLICK 1 : RSPBStart -------------------------------------------
    y = panel(
        ax, y,
        "CLICK 1 · RSPBStart", "rs_pb_start.py  (once per Rhino session)",
        [
            "main(use_gui=False)",
            "└─ robot_cell.start_pb_client(use_gui)                 robot_cell.py:266",
            "   ├─ _ensure_submodule_compas_fab_loaded()      # pin in-repo compas_fab",
            "   ├─ PyBulletClient(\"direct\").__enter__()         # connect PyBullet",
            "   ├─ PyBulletPlanner(client)",
            "   ├─ get_or_load_robot_cell()                        robot_cell.py:208",
            "   │    ├─ LocalPackageMeshLoader.load_urdf(HUSKY_URDF)",
            "   │    ├─ RobotModel.from_urdf_string() → load_geometry(x4 loaders)",
            "   │    ├─ RobotSemantics.from_srdf_file()",
            "   │    └─ * RobotCell(robot_model, semantics) → sticky[ROBOT_CELL]",
            "   └─ planner.set_robot_cell(rcell)  → sticky[CELL_KIND]=\"dual_arm\"",
        ],
        C_STATIC,
        [
            "• cached PyBullet client",
            "  + planner  (sc.sticky)",
            "• a BARE RobotCell:",
            "  robot_model + semantics",
            "  only — NO tools, bars,",
            "  or obstacles yet.",
        ],
    )
    arrow_y = y
    y -= 1.8

    # ---- CLICK 2 : RSRebuildRobotCell ----------------------------------
    y = panel(
        ax, y,
        "CLICK 2 · RSRebuildRobotCell", "rs_rebuild_robot_cell.py  (after ADD / MOVE / RESIZE geometry)",
        [
            "main()",
            "├─ reload(robot_cell / config / env_collision)",
            "├─ is_pb_running()                         # guard: needs RSPBStart",
            "├─ get_planner()                           → cached (client, planner)  robot_cell.py:345",
            "├─ get_or_load_robot_cell()                → cached BARE cell",
            "├─ repair_on_entry(BAR_RADIUS)             # bar-registry hygiene",
            "└─ robot_cell.rebuild_assembly_cell(rcell, planner)   robot_cell.py:1359",
            "   ├─ get_bar_seq_map()",
            "   ├─ env_collision.collect_assembly_geometry(seq_map)  → bar_<id>, joint_<jid>_<sub>",
            "   ├─ env_collision.collect_environment_geometry()      → obstacle_<name>",
            "   ├─ build_arm_tool_models()                          robot_cell.py:1218",
            "   │    └─ * rcell.tool_models[AT3L/AT3R] = ToolModel(collision OBJ)",
            "   ├─ * rcell.rigid_body_models[name] = rb   # replace managed bar_/joint_/obstacle_ set",
            "   ├─ planner.set_robot_cell(rcell)          # push once (tools + bodies)",
            "   └─ * sticky[ASSEMBLY_SNAPSHOT] = {collision_bodies, tool_ids}",
            "        sticky[ASSEMBLY_FINGERPRINT] = _live_assembly_fingerprint()",
        ],
        C_STATIC,
        [
            "STAGE 1 complete:",
            "• the STATIC geometry",
            "  registry on rcell —",
            "  tool_models +",
            "  rigid_body_models.",
            "• cached collision_bodies",
            "  {name: body_info}",
            "  snapshot + fingerprint",
            "  (reused, not re-scanned).",
        ],
    )
    arrow_y2 = y
    y -= 1.8

    # ---- CLICK 3 : RSIKKeyframe / RSShowIK -----------------------------
    panel(
        ax, y,
        "CLICK 3 · RSIKKeyframe  /  RSShowIK", "per bar — this is where the TEMPLATE state is built",
        [
            "main()",
            "├─ get_planner() / get_or_load_robot_cell()      → cached",
            "├─ prompt_if_cell_stale(rcell, planner)          # fingerprint: Rebuild / Proceed / Abort",
            "├─ default_cell_state()                          # neutral state (base-preview only)",
            "├─ … pick Ln bar + resolve L/R tools …",
            "└─ bar_action.build_assembly_movements(rcell, planner, bar_id, base, tool0L, tool0R)   bar_action.py:892",
            "   ├─ slim = default_cell_state(); _set_robot_base_frame(slim, base)",
            "   └─ ik_collision_setup.prepare_assembly_collision_state(rcell, planner, slim, bar_id)   ik_collision_setup.py:244",
            "      ├─ robot_cell.ensure_assembly_cell(rcell, planner)          robot_cell.py:1499",
            "      │    └─ snapshot present → return cached collision_bodies  (else rebuild_assembly_cell)",
            "      ├─ * robot_cell.base_assembly_cell_state()                  robot_cell.py:1286   ← ATTACH tools",
            "      │    └─ tool_states[AT3L]→LEFT_GROUP, [AT3R]→RIGHT_GROUP;",
            "      │       touch_links = wrist_2/3 ; attachment_frame = worldXY",
            "      ├─ base.robot_base_frame / robot_configuration ← slim",
            "      └─ build_full_assembly_state(base, collision_bodies, seq_map, bar_id)   ik_collision_setup.py:175",
            "           └─ * per body: RigidBodyState(static @ world; is_hidden if seq > active bar)",
            "      ⇒ returns (template_state, env_geom)",
            "   → _build_m0 … _build_m4  each do template_state.copy()   (Stage 3)",
        ],
        C_TEMPLATE,
        [
            "STAGE 2 complete:",
            "• the per-bar",
            "  template_state:",
            "  tools attached,",
            "  every obstacle placed",
            "  static @ world,",
            "  unbuilt bars hidden,",
            "  active bar still",
            "  static (each Mi",
            "  re-classes it).",
            "• env_geom = the",
            "  collision-body dict.",
        ],
    )

    # Session-order arrows down the left gutter.
    arrow_kw = dict(arrowstyle="-|>", mutation_scale=18, linewidth=2.2, color="#888")
    ax.add_patch(FancyArrowPatch((0.9, arrow_y + 1.7), (0.9, arrow_y - 0.1), **arrow_kw))
    ax.add_patch(FancyArrowPatch((0.9, arrow_y2 + 1.7), (0.9, arrow_y2 - 0.1), **arrow_kw))

    ax.text(50, 2.2,
            "`*` = a durable object is created / mutated.   sticky = sc.sticky (survives between commands).   "
            "rcell = the one cached RobotCell.   Stage 3 (M0–M4) begins where Click 3 ends.",
            ha="center", va="center", fontsize=9, style="italic", color="#444")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "robotcell_build_callstack.png")
    fig.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
