"""Generate `docs/robotcell_state_lifecycle.png`.

A companion to `planning_call_stacks.png`. Where that diagram maps entry
points -> public API -> planning primitive, THIS one maps the data-flow of
the RobotCell + RobotCellState: how the static geometry cell is built, how a
per-bar template state is stamped with tool attachments + obstacle
visibility, how each of the four BarAction movements (M1-M4) re-classes the
grasped bar and sets its own attachment + Allowed-Collision (touch_bodies)
policy, and how `rs_ik_keyframe.py` (solver) and `rs_show_ik.py` (viewer)
consume those movement start states.

Run with the plain (non-Rhino) Python that has matplotlib:
    python docs/robotcell_state_lifecycle.py
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch


# ---------------------------------------------------------------------------
# Palette (mirrors the planning_call_stacks.png colour language)
# ---------------------------------------------------------------------------
C_STATIC   = ("#e8f0fb", "#2f6fb0")   # blue   -- static RobotCell geometry
C_TEMPLATE = ("#fdf0e0", "#d98a2b")   # orange -- per-bar template state
C_GRIP     = ("#e6f3e6", "#3a8f3a")   # green  -- gripped movements (M1/M2)
C_RELEASE  = ("#f3ece2", "#9a6b2f")   # brown  -- released movements (M3/M4)
C_CONSUMER = ("#eceff2", "#4a5568")   # grey   -- consumer scripts
INK        = "#222222"


def box(ax, x, y, w, h, title, lines, colors, title_size=11, body_size=8.5, file=None):
    """Draw one rounded box: bold title + (optional) source-file + body lines."""
    face, edge = colors
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.15,rounding_size=0.5",
        linewidth=1.8, edgecolor=edge, facecolor=face, zorder=2,
    ))
    ax.text(x + w / 2, y + h - 0.55, title, ha="center", va="top",
            fontsize=title_size, fontweight="bold", color=edge, zorder=3)
    body_top = y + h - 1.5
    if file:
        # Small grey monospace line naming the .py file the function lives in.
        ax.text(x + w / 2, y + h - 1.45, file, ha="center", va="top",
                fontsize=7.4, family="monospace", style="italic",
                color="#6a7480", zorder=3)
        body_top = y + h - 2.75
    ax.text(x + 0.45, body_top, "\n".join(lines), ha="left", va="top",
            fontsize=body_size, color=INK, zorder=3, linespacing=1.35)


def arrow(ax, p0, p1, color=INK, style="-", lw=2.0, rad=0.0):
    ax.add_patch(FancyArrowPatch(
        p0, p1, connectionstyle=f"arc3,rad={rad}",
        arrowstyle="-|>", mutation_scale=16, linewidth=lw,
        linestyle=style, color=color, zorder=1,
    ))


def main():
    fig, ax = plt.subplots(figsize=(23, 14))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    ax.text(50, 98.3, "RobotCell / RobotCellState lifecycle  —  bar_joint_rhino_design_workflow",
            ha="center", va="top", fontsize=19, fontweight="bold", color=INK)

    # Column headers.
    heads = [
        (12.5, "1 · STATIC RobotCell\n(geometry registry, built once)"),
        (37.5, "2 · TEMPLATE cell state\n(per bar, per command)"),
        (63.0, "3 · PER-MOVEMENT start states\nbar_action.build_assembly_movements"),
        (87.5, "4 · CONSUMERS"),
    ]
    for hx, ht in heads:
        ax.text(hx, 94.2, ht, ha="center", va="top", fontsize=12.5,
                fontweight="bold", color="#333333")

    # -------------------------------------------------------------------
    # Column 1 — static RobotCell geometry registry
    # -------------------------------------------------------------------
    box(ax, 2, 84.0, 21, 7.5,
        "get_or_load_robot_cell()",
        ["RobotCell(robot_model, semantics)",
         "loaded from URDF + SRDF, cached in sticky.",
         "NO tools / bars / obstacles yet."],
        C_STATIC, file="core/robot_cell.py")

    box(ax, 2, 55.5, 21, 26.0,
        "rebuild_assembly_cell()   [RSRebuildRobotCell]",
        ["build_arm_tool_models()   [robot_cell.py]",
         "   -> tool_models = {AT3L, AT3R}",
         "      (collision OBJ + tool0->tcp frame)",
         "",
         "env_collision.collect_assembly_geometry()",
         "   -> rigid_body_models: bar_<id>,",
         "      joint_<jid>_<male|female>",
         "env_collision.collect_environment_geometry()",
         "   -> rigid_body_models: obstacle_<name>",
         "",
         "planner.set_robot_cell(rcell)",
         "caches snapshot: collision_bodies",
         "   = {name: body_info(frame, kind,",
         "      parent_bar_id, rigid_body)}"],
        C_STATIC, title_size=10,
        file="core/robot_cell.py  ·  core/env_collision.py")

    box(ax, 2, 45.0, 21, 8.2,
        "ensure_assembly_cell()",
        ["per-command: returns the CACHED",
         "collision_bodies (no re-scan).",
         "fingerprint check -> stale warning only."],
        C_STATIC, file="core/robot_cell.py")

    arrow(ax, (12.5, 84.0), (12.5, 81.5), C_STATIC[1])
    arrow(ax, (12.5, 55.5), (12.5, 53.2), C_STATIC[1])

    # -------------------------------------------------------------------
    # Column 2 — template state (per command)
    # -------------------------------------------------------------------
    box(ax, 27, 78.0, 21, 9.0,
        "base_assembly_cell_state()",
        ["default_cell_state() + ATTACH tools:",
         "tool_states AT3L->LEFT_GROUP,",
         "AT3R->RIGHT_GROUP  (wrist touch_links,",
         "attachment_frame = worldXY)"],
        C_TEMPLATE, title_size=10, file="core/robot_cell.py")

    box(ax, 27, 53.5, 21, 22.0,
        "prepare_assembly_collision_state()",
        ["-> then build_full_assembly_state():",
         "copy base state; set robot_base_frame",
         "+ robot_configuration.",
         "",
         "one RigidBodyState per collision body,",
         "STATIC (unattached) at its world frame:",
         "  • obstacle_*  -> visible static",
         "  • bars/joints seq <= active bar",
         "        -> visible static obstacle",
         "  • bars/joints seq >  active bar",
         "        -> is_hidden = True",
         "  • ACTIVE bar -> still static here",
         "        (each Mi re-classes it)",
         "",
         "returns (template_state, env_geom)"],
        C_TEMPLATE, title_size=10, file="core/ik_collision_setup.py")

    arrow(ax, (37.5, 78.0), (37.5, 75.5), C_TEMPLATE[1])
    # static cell feeds the template.
    arrow(ax, (23, 62), (27, 66), C_STATIC[1], style="--", rad=-0.15)
    ax.text(24.7, 58.5, "collision_bodies\n+ tool_models", ha="center",
            va="center", fontsize=7.5, style="italic", color=C_STATIC[1])

    # -------------------------------------------------------------------
    # Column 3 — per-movement start states M1..M4
    # -------------------------------------------------------------------
    mx = 52.5
    mw = 22.5
    def movement_box(y, h, title, lines, colors):
        box(ax, mx, y, mw, h, title, lines, colors, title_size=9.5, body_size=8,
            file="core/bar_action.py")

    movement_box(73.5, 13.5,
        "M1  DualArmConstrained · home->approach",
        ["config = HOME (both arms)",
         "ATTACH to tool0 (frame = inv(tool0)@body):",
         "  bar tube + females -> LEFT arm",
         "     (left_ur_arm_tool0)",
         "  each male -> ITS arm (L-tool->left,",
         "     R-tool->right ur_arm_tool0)",
         "ACM: male<->its-own-arm tool, male<->bar,",
         "  female<->bar, bar<->BOTH tools (AT3L+AT3R)",
         "target_ee = approach frames (geom offset)"],
        C_GRIP)

    movement_box(59.0, 12.0,
        "M2  Linear · approach->assembled  (mate)",
        ["config = approach_groups  (None pre-IK)",
         "ATTACH: same as M1 (bar+females->LEFT,",
         "  each male->its own arm; still gripped)",
         "ACM: M1 + male<->its built mate female",
         "target_ee = assembled frames"],
        C_GRIP)

    movement_box(44.0, 12.0,
        "M3  Linear · assembled->retreat",
        ["config = assembled_groups",
         "DETACH: bar+joints -> static @ world",
         "  (no arm attachment)",
         "ACM: male<->its-own-arm tool; bar<->both tools",
         "target_ee = per-arm retreat (joint -Z)"],
        C_RELEASE)

    movement_box(30.5, 11.0,
        "M4  Free · retreat->home",
        ["config = None (planner fills from M3 end)",
         "DETACH: bar+joints static @ world",
         "ACM: none  (bar<->tools cleared)",
         "target_configuration = HOME"],
        C_RELEASE)

    # template -> each movement (copy + re-class).
    for yc in (80.6, 65.5, 50.5, 36.5):
        arrow(ax, (48, 65), (52.5, yc), C_TEMPLATE[1], style="--", rad=0.12, lw=1.4)
    ax.text(50.2, 71.0, "template_state\n.copy()  per Mi", ha="center",
            va="center", fontsize=7.5, style="italic", color=C_TEMPLATE[1])

    # -------------------------------------------------------------------
    # Column 4 — consumers
    # -------------------------------------------------------------------
    box(ax, 76, 66.5, 22.5, 20,
        "rs_ik_keyframe.py   (SOLVER)",
        ["build_assembly_movements(groups=None)  [bar_action.py]",
         "   -> configs left UNSOLVED",
         "",
         "_solve_chain_with_sampling()",
         " -> ik_keyframe.solve_keyframe_chain(",
         "        [M1, M2, M3], base_frame)",
         "    each: start_state.copy(); chain seed",
         "    prev config; solve_dual_arm_ik on",
         "    movement.target_ee_frames",
         "",
         "M1->approach  M2->assembled  M3->retreat",
         "preview: set_cell_state + ik_viz.update",
         "accept -> write bar user-text:",
         "  KEY_ASSEMBLY_BASE_FRAME / IK_APPROACH",
         "  / IK_ASSEMBLED / IK_RETREAT"],
        C_CONSUMER, title_size=10, body_size=8)

    box(ax, 76, 40.5, 22.5, 22,
        "rs_show_ik.py   (VIEWER)",
        ["_load_assembly_payload(bar) -> saved",
         "  base + approach/assembled/retreat",
         "",
         "_build_movements():",
         "  build_assembly_movements(",
         "     approach_groups=saved.approach,",
         "     assembled_groups=saved.assembled)",
         "",
         "cycle POSES M1->M2->M3->M4:",
         "  show movement.start_state",
         "  (M4: fill cfg from M3, apply retreat)",
         "  set_cell_state + ik_viz.update_state",
         "",
         "CheckCollision / jog dialog:",
         "  planner.check_collision(start_state)",
         "  uses that Mi's touch_bodies (ACM)",
         "  -> red-highlight offenders"],
        C_CONSUMER, title_size=10, body_size=8)

    # movements -> consumers.
    arrow(ax, (75, 74), (76, 76), C_GRIP[1], rad=-0.1)
    arrow(ax, (75, 50), (76, 51), C_RELEASE[1], rad=0.1)
    ax.text(75.5, 62.5, "start_state\n+ target_ee\n+ touch_bodies",
            ha="center", va="center", fontsize=7.3, style="italic", color="#555")

    # ik_keyframe writes -> show_ik reads (user-text round trip).
    arrow(ax, (87.2, 66.5), (87.2, 62.5), "#b03a5b", style="--", lw=1.6)
    ax.text(89.6, 64.5, "bar user-text\n(KEY_ASSEMBLY_*)", ha="left",
            va="center", fontsize=7.5, style="italic", color="#b03a5b")

    # -------------------------------------------------------------------
    # Legend
    # -------------------------------------------------------------------
    leg = [
        (C_STATIC,   "Static RobotCell geometry (tool_models + rigid_body_models)"),
        (C_TEMPLATE, "Per-bar template state (tools attached, obstacles placed)"),
        (C_GRIP,     "Gripped movement start state (bar attached to tool0)"),
        (C_RELEASE,  "Released movement start state (bar static at world)"),
        (C_CONSUMER, "Consumer script (solver / viewer)"),
    ]
    lx, ly = 3, 3.0
    for i, (colors, label) in enumerate(leg):
        yy = ly + (len(leg) - 1 - i) * 3.4
        ax.add_patch(FancyBboxPatch(
            (lx, yy), 3.2, 2.0, boxstyle="round,pad=0.05,rounding_size=0.4",
            linewidth=1.6, edgecolor=colors[1], facecolor=colors[0], zorder=2))
        ax.text(lx + 4.2, yy + 1.0, label, ha="left", va="center",
                fontsize=9, color=INK)

    # ---- "where each function lives" map box (lower-right empty area).
    ax.add_patch(FancyBboxPatch(
        (34, 8.5), 64, 15.5, boxstyle="round,pad=0.2,rounding_size=0.5",
        linewidth=1.4, edgecolor="#6a7480", facecolor="#f6f7f9", zorder=2))
    ax.text(66, 22.9, "WHERE EACH FUNCTION LIVES", ha="center", va="top",
            fontsize=10, fontweight="bold", color="#4a5568", zorder=3)
    ax.text(35.2, 20.6,
            "core/robot_cell.py     get_or_load_robot_cell · default_cell_state · rebuild_assembly_cell ·\n"
            "                       ensure_assembly_cell · build_arm_tool_models · base_assembly_cell_state ·\n"
            "                       solve_dual_arm_ik · set_cell_state · extract_group_config\n"
            "core/env_collision.py  collect_assembly_geometry · collect_environment_geometry\n"
            "core/ik_collision_setup.py  prepare_assembly_collision_state · build_full_assembly_state\n"
            "core/bar_action.py     build_assembly_movements · _set_active_attachments ·\n"
            "                       _apply_movement_touch_policy · _build_m1 … _build_m4\n"
            "core/ik_keyframe.py    solve_keyframe_chain          core/ik_viz.py  update_state · begin_session\n"
            "scripts/rs_ik_keyframe.py · scripts/rs_show_ik.py   the two consumer commands (column 4)",
            ha="left", va="top", fontsize=8.2, family="monospace",
            color=INK, zorder=3, linespacing=1.4)

    ax.text(63, 5.6,
            "Solid arrow = builds / produces.   Dashed = supplies data (no new object).   "
            "Each Mi = template_state.copy() with only the grasped bar + its joints re-classed;\n"
            "attachment_frame + touch_bodies (allowed-collision whitelist) stamped per movement inside "
            "bar_action._set_active_attachments / _apply_movement_touch_policy.   "
            "Bar + females attach to the LEFT arm (bar_arm_side='left', the default; parameterizable);\n"
            "each male attaches to the arm whose L/R-suffixed tool sits on it.",
            ha="center", va="center", fontsize=8.3, style="italic", color="#444")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "robotcell_state_lifecycle.png")
    fig.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
