"""Generate `docs/three_module_pipeline.png`.

The whole three-module pipeline: (1) the Rhino 8 design frontend in this repo
solves IK keyframes and exports a schema 2 design folder through the shared core
`bar_assembly_core`, (2) `scripts/plan_design.py` plans the movements with the
`external/husky_assembly_tamp` motion planner into `solutions/`, and (3) the
live execution stack (`husky-assembly-teleop`, ROS2) runs the solved plans on
the real robot, replanning the arms live against the original tool0 targets once
the mocap-tracked base stops. Solutions also flow BACK into Rhino. Every module
builds its compas_fab cells from the design with the core.

Run with the plain (non-Rhino) Python that has matplotlib:
    python docs/three_module_pipeline.py
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch


# ---------------------------------------------------------------------------
# Palette
# ---------------------------------------------------------------------------
C_DESIGN   = ("#e8f0fb", "#2f6fb0")   # blue   -- Module 1: Rhino design frontend
C_PLANNER  = ("#fdf0e0", "#d98a2b")   # orange -- Module 2: offline planner
C_LIVE     = ("#e6f3e6", "#3a8f3a")   # green  -- Module 3: live execution
C_ARTIFACT = ("#f3ece2", "#9a6b2f")   # brown  -- JSON artifacts between modules
C_AUX      = ("#eceff2", "#4a5568")   # grey   -- auxiliary (viewer, schema note)
INK        = "#222222"


def box(ax, x, y, w, h, title, lines, colors, title_size=11, body_size=8.5, file=None):
    """Draw one rounded box: bold title + (optional) source-file + body lines.

    Args:
        ax: Matplotlib axes to draw on.
        x (float): Left edge of the box in the 0-100 canvas.
        y (float): Bottom edge of the box in the 0-100 canvas.
        w (float): Box width.
        h (float): Box height.
        title (str): Bold heading drawn at the top of the box.
        lines (list[str]): Body text, one string per line.
        colors (tuple[str, str]): (face colour, edge colour) pair.
        title_size (float): Font size for the title.
        body_size (float): Font size for the body lines.
        file (str): Optional grey monospace line naming the source file(s).
    """
    face, edge = colors
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.15,rounding_size=0.5",
        linewidth=1.8, edgecolor=edge, facecolor=face, zorder=2,
    ))
    ax.text(x + w / 2, y + h - 0.55, title, ha="center", va="top",
            fontsize=title_size, fontweight="bold", color=edge, zorder=3)
    # ! Vertical offsets leave a full line of clearance under the title (and
    # ! under the file line) so nothing overlaps at the canvas scale used here.
    body_top = y + h - 2.3
    if file:
        # Small grey monospace line naming where the code lives.
        ax.text(x + w / 2, y + h - 2.05, file, ha="center", va="top",
                fontsize=7.4, family="monospace", style="italic",
                color="#6a7480", zorder=3)
        body_top = y + h - 3.4
    ax.text(x + 0.45, body_top, "\n".join(lines), ha="left", va="top",
            fontsize=body_size, color=INK, zorder=3, linespacing=1.35)


def arrow(ax, p0, p1, color=INK, style="-", lw=2.0, rad=0.0):
    """Draw one arrow from p0 to p1.

    Solid = produces / writes an artifact; dashed = supplies data (reads).

    Args:
        ax: Matplotlib axes to draw on.
        p0 (tuple[float, float]): Arrow tail (x, y).
        p1 (tuple[float, float]): Arrow head (x, y).
        color (str): Arrow colour.
        style (str): Line style, "-" solid or "--" dashed.
        lw (float): Line width.
        rad (float): Curvature; 0 = straight, sign picks the bend side.
    """
    ax.add_patch(FancyArrowPatch(
        p0, p1, connectionstyle=f"arc3,rad={rad}",
        arrowstyle="-|>", mutation_scale=16, linewidth=lw,
        linestyle=style, color=color, zorder=1,
    ))


def main():
    fig, ax = plt.subplots(figsize=(24, 13.5))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    ax.text(50, 99.0, "Three-module pipeline  —  design → offline planning → live execution",
            ha="center", va="top", fontsize=19, fontweight="bold", color=INK)

    # ! Column headers: one per module, left to right.
    heads = [
        (12.5, "1 · DESIGN\nRhino 8 (Windows)"),
        (51.5, "2 · OFFLINE PLANNING\nplan_design.py + husky_assembly_tamp"),
        (88.5, "3 · LIVE EXECUTION\nhusky-assembly-teleop (ROS2, Ubuntu)"),
    ]
    for hx, ht in heads:
        ax.text(hx, 95.0, ht, ha="center", va="top", fontsize=12.5,
                fontweight="bold", color="#333333")

    # -------------------------------------------------------------------
    # * Column 1 — design frontend (this repo, Rhino 8)
    # -------------------------------------------------------------------
    box(ax, 2, 81, 21, 8,
        "Design scripts (bars / joints)",
        ["place scaffolding bars + connector blocks,",
         "assign assembly sequence + walkable ground"],
        C_DESIGN, title_size=10.5,
        file="rs_create_bar.py · rs_joint_place.py · …")

    box(ax, 2, 67, 21, 10,
        "IK keyframes   [RSIKKeyframe / All]",
        ["pick a base, solve approach→assembled→",
         "retreat on the core's mirror (tamp",
         "solvers), preview, save keyframes",
         "on the bar"],
        C_DESIGN, title_size=10.5, file="rs_ik_keyframe.py")

    box(ax, 2, 55, 21, 8,
        "Export   [RSExportDesign]",
        ["write one schema 2 design folder",
         "(right) through bar_assembly_core"],
        C_DESIGN, title_size=10.5, file="rs_export_design.py")

    # Grey auxiliary box: solved plans come back into Rhino for viewing.
    box(ax, 2, 25, 21, 9,
        "Solved-plan viewer (back in Rhino)",
        ["write solved keyframes onto the bars,",
         "step through poses / trajectories"],
        C_AUX, title_size=10.5,
        file="RSLoadSolvedBarAction · RSShowBarActionPlan")

    # Flow inside column 1: design -> quick IK check -> export.
    arrow(ax, (12.5, 81), (12.5, 77.4), C_DESIGN[1])
    arrow(ax, (12.5, 67), (12.5, 63.4), C_DESIGN[1])

    # -------------------------------------------------------------------
    # * Interface 1|2 — the clean exports (brown JSON artifacts)
    # -------------------------------------------------------------------
    box(ax, 25.5, 51, 13, 16,
        "DESIGN FOLDER",
        ["design.json",
         "  robots, tools, bodies,",
         "  mates, schedule",
         "",
         "actions/<id>.json",
         "  movements: start states,",
         "  link / joint targets",
         "",
         "meshes/ · robots/",
         "  geometry, URDF/SRDF"],
        C_ARTIFACT, title_size=10, body_size=7.8)

    # Export box WRITES the clean JSONs (solid).
    arrow(ax, (23, 59.5), (25.5, 61), C_DESIGN[1], rad=-0.1)
    # Offline planner READS them (dashed) — JSON is the source of truth.
    arrow(ax, (38.5, 64), (40.5, 79), C_ARTIFACT[1], style="--", rad=-0.2)
    ax.text(37.6, 73.0, "reads (design =\nsource of truth)", ha="center",
            va="center", fontsize=7.5, style="italic", color=C_ARTIFACT[1])

    # -------------------------------------------------------------------
    # * Column 2 — offline planner (husky_assembly_tamp submodule)
    # -------------------------------------------------------------------
    box(ax, 40.5, 75.5, 22, 11,
        "Scenes → compas_fab cells",
        ["Design.scene_at(movement) → the core's",
         "CompasFabMirror (one PyBullet world per",
         "robot); the tamp solvers plan on its",
         "lent planner (Rhino's IK check too)"],
        C_PLANNER, title_size=10.5,
        file="bar_assembly_core/mirrors/")

    box(ax, 40.5, 61, 22, 10,
        "Motion planning",
        ["transfer, insert, retreat and home of",
         "the assembly robot (tamp motion API);",
         "support-robot moves not planned yet"],
        C_PLANNER, title_size=10.5,
        file="husky_assembly_tamp/motion_planner/")

    box(ax, 40.5, 44, 22, 12.5,
        "CLI entry point (drives both steps above)",
        ["plan_design.py <design folder>",
         "  plan the movements, write solutions/",
         "  --bars B3,B10 plans some bars only",
         "runs on Windows or Ubuntu"],
        C_PLANNER, title_size=9.5,
        file="scripts/plan_design.py  (this repo)")

    # Keyframe configs seed the motion planner (solid: produces the seeds).
    arrow(ax, (51.5, 75.5), (51.5, 71.4), C_PLANNER[1])
    ax.text(52.3, 73.5, "one cell per robot, kept", ha="left",
            va="center", fontsize=7.5, style="italic", color=C_PLANNER[1])

    # Both planner stages WRITE their solved sidecars (solid).
    arrow(ax, (62.5, 79.5), (67.5, 65), C_PLANNER[1], rad=-0.2)
    arrow(ax, (62.5, 65.5), (64.5, 61.5), C_PLANNER[1], rad=-0.1)

    # -------------------------------------------------------------------
    # * Interface 2|3 — the solved sidecars (brown JSON artifacts)
    # -------------------------------------------------------------------
    box(ax, 64.5, 51, 13, 14,
        "SOLUTIONS",
        ["solutions/<action>.json",
         "  start / end joints,",
         "  trajectories, bases",
         "",
         "stale once the design",
         "  changes (hashes)"],
        C_ARTIFACT, title_size=10, body_size=7.8)

    # Live executor READS the solved plans (dashed).
    arrow(ax, (77.5, 62), (79.5, 79), C_ARTIFACT[1], style="--", rad=-0.2)
    ax.text(77.3, 72.5, "loaded on\nthe robot", ha="center", va="center",
            fontsize=7.5, style="italic", color=C_ARTIFACT[1])

    # -------------------------------------------------------------------
    # * Column 3 — live execution (husky-assembly-teleop, ROS2)
    # -------------------------------------------------------------------
    box(ax, 79.5, 79, 18.5, 7.5,
        "Load + live monitor",
        ["load the design + solutions; monitor",
         "and execute the assembly bar by bar"],
        C_LIVE, title_size=10.5)

    box(ax, 79.5, 67, 18.5, 8,
        "Mocap base localization",
        ["motion capture senses the real base",
         "pose; the base tracks the planned",
         "pose as closely as it can"],
        C_LIVE, title_size=10.5)

    box(ax, 79.5, 52, 18.5, 11,
        "Live arm replanning",
        ["the base never lands exactly on the",
         "plan — once it stops, the ARM motion",
         "is replanned live against the ORIGINAL",
         "tool0 targets from the mocap-sensed",
         "base pose"],
        C_LIVE, title_size=10.5)

    # Execution order inside column 3: load -> drive base -> replan arms.
    arrow(ax, (88.75, 79), (88.75, 75.4), C_LIVE[1])
    arrow(ax, (88.75, 67), (88.75, 63.4), C_LIVE[1])

    # Replanning still READS the original tool0 targets out of the solved plan.
    arrow(ax, (77.5, 53.5), (79.5, 57), C_ARTIFACT[1], style="--", rad=0.15)
    ax.text(75.5, 48.5, "original tool0\ntargets", ha="center", va="center",
            fontsize=7.5, style="italic", color=C_ARTIFACT[1])

    # ! Feedback loop: solved sidecars flow BACK into Rhino for visualization.
    arrow(ax, (70, 51), (23, 30), C_AUX[1], style="--", lw=1.6, rad=0.3)
    ax.text(49, 28.5, "solved plans flow back into Rhino for visualization",
            ha="center", va="center", fontsize=8, style="italic",
            color=C_AUX[1])

    # -------------------------------------------------------------------
    # * Shared schema note (grey): the contract every artifact speaks.
    # -------------------------------------------------------------------
    box(ax, 36, 8, 38, 9,
        "Shared core — the interchange contract",
        ["design format (schema 2), scenes and compas_fab cells;",
         "every module reads and builds its cells with it"],
        C_AUX, title_size=10,
        file="external/bar_assembly_core  (git submodule)")

    # -------------------------------------------------------------------
    # Legend
    # -------------------------------------------------------------------
    leg = [
        (C_DESIGN,   "Module 1 — design frontend (this repo, Rhino 8 scripts)"),
        (C_PLANNER,  "Module 2 — offline planner (plan_design.py + husky_assembly_tamp)"),
        (C_LIVE,     "Module 3 — live execution (husky-assembly-teleop, ROS2)"),
        (C_ARTIFACT, "JSON artifacts — the interfaces between the modules"),
        (C_AUX,      "Auxiliary (Rhino plan viewers · shared core note)"),
    ]
    lx, ly = 3, 3.0
    for i, (colors, label) in enumerate(leg):
        yy = ly + (len(leg) - 1 - i) * 3.4
        ax.add_patch(FancyBboxPatch(
            (lx, yy), 3.2, 2.0, boxstyle="round,pad=0.05,rounding_size=0.4",
            linewidth=1.6, edgecolor=colors[1], facecolor=colors[0], zorder=2))
        ax.text(lx + 4.2, yy + 1.0, label, ha="left", va="center",
                fontsize=9, color=INK)

    # Footnote: arrow semantics + the live-replanning story in one breath.
    ax.text(63, 4.0,
            "Solid arrow = produces / writes.   Dashed arrow = supplies data (reads).   "
            "The robot never reaches the planned base pose exactly — mocap tracks the real base,\n"
            "and once the base stops the ARM motion is replanned live against the ORIGINAL tool0 targets.   "
            "The design format and its cells live in the shared core, external/bar_assembly_core.",
            ha="center", va="center", fontsize=8.3, style="italic", color="#444")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "three_module_pipeline.png")
    fig.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
