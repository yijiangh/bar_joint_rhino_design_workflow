"""Generate the action flowchart of the CURRENT action / movement implementation.

Writes two files next to this script:

- ``action_flowchart_2026.drawio`` -- an editable draw.io diagram that reuses
  the shape styles of ``action_flowchart_draft_2025.drawio.xml`` (same colors,
  same rounded boxes, rhombus conditionals, circle start / end).
- ``action_flowchart_2026.png`` -- a matplotlib render of the SAME node and
  edge list, so the picture never drifts from the draw.io file.

The flowchart follows the method of the SCF 2021 paper ("The new analog"):
high-level actions are rounded containers, each broken down into atomic
movements colored by movement type. Every box carries the movement id used in
the code (``J_M3``, ``H_M2``, ...) so the picture can be read next to
``external/rs_data_structure`` and ``scripts/core/bar_action.py`` /
``scripts/core/hold_action_builder.py``.

Run with the plain (non-Rhino) Python that has matplotlib:
    python docs/action_flowchart_2026.py
"""

from __future__ import annotations

import os
import textwrap
from xml.sax.saxutils import escape

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import to_rgb
from matplotlib.patches import Ellipse, FancyBboxPatch, Polygon


# * ---------------------------------------------------------------- styles
# Colors copied from the 2025 draft so the two diagrams read the same way.
# Each entry: (fill color, stroke color, gradient color or None, dashed?)
KIND = {
    "fm": ("#dae8fc", "#6c8ebf", None, False),            # free robotic movement
    "lm": ("#d5e8d4", "#82b366", None, False),            # linear robotic movement
    "fm_c": ("#dae8fc", "#6c8ebf", "#E6D0DE", False),     # constrained dual-arm free
    "lm_c": ("#d5e8d4", "#82b366", "#E6D0DE", False),     # constrained dual-arm linear
    "manual": ("#d0f0f0", "#000000", None, False),        # manual movement
    "tool": ("#f5f5f5", "#666666", None, False),          # tool movement
    "tool_overlap": ("#f5f5f5", "#666666", None, True),   # tool movement overlapping the next one
    "plain": ("#ffffff", "#000000", None, False),         # flow-control boxes (i = 0, ...)
    "container": ("#ffffff", "#000000", None, False),     # high-level action
    "note": ("#ffffff", "#000000", None, False),          # free text note with a border
}
INK = "#000000"

# draw.io style strings, one per kind (the draft's own strings, trimmed).
DRAWIO_STYLE = {
    "fm": "whiteSpace=wrap;html=1;rounded=1;fontSize=10;fillColor=#dae8fc;strokeColor=#6c8ebf;align=center;",
    "lm": "whiteSpace=wrap;html=1;rounded=1;fontSize=10;fillColor=#d5e8d4;strokeColor=#82b366;align=center;",
    "fm_c": "whiteSpace=wrap;html=1;rounded=1;fontSize=10;fillColor=#dae8fc;strokeColor=#6c8ebf;align=center;gradientColor=#E6D0DE;gradientDirection=east;",
    "lm_c": "whiteSpace=wrap;html=1;rounded=1;fontSize=10;fillColor=#d5e8d4;strokeColor=#82b366;align=center;gradientColor=#E6D0DE;gradientDirection=east;",
    "manual": "rounded=1;whiteSpace=wrap;html=1;fontSize=10;fillColor=light-dark(#d0f0f0, #ededed);",
    "tool": "whiteSpace=wrap;html=1;rounded=1;fontSize=10;fillColor=#f5f5f5;fontColor=#333333;strokeColor=#666666;align=center;",
    "tool_overlap": "whiteSpace=wrap;html=1;rounded=1;fontSize=10;fillColor=#f5f5f5;fontColor=#333333;strokeColor=#666666;align=center;dashed=1;dashPattern=5 3;",
    "plain": "whiteSpace=wrap;html=1;rounded=1;fontSize=10;",
    "container": "rounded=1;whiteSpace=wrap;html=1;fontSize=10;fontStyle=1;verticalAlign=top;align=left;spacingLeft=8;spacingTop=2;fillColor=#ffffff;",
    "note": "whiteSpace=wrap;html=1;fontSize=9;align=left;verticalAlign=top;spacing=5;",
    "rhombus": "rhombus;whiteSpace=wrap;html=1;rounded=1;fontSize=10;",
    "ellipse": "ellipse;whiteSpace=wrap;html=1;aspect=fixed;fontStyle=1;fontSize=10;",
    "text": "text;html=1;strokeColor=none;fillColor=none;align=left;verticalAlign=middle;whiteSpace=wrap;rounded=0;fontSize=10;",
    "edge": "edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;fontSize=10;",
}


# * ---------------------------------------------------------------- the diagram
# Every node: dict(id, shape, kind, x, y, w, h, title, text).
#   shape: "box" | "rhombus" | "ellipse" | "text"
#   title: bold first line (movement id or container name), may be ""
#   text:  wrapped body text, may be ""
NODES = []
EDGES = []   # dict(src, dst, exit=(ex, ey), entry=(ex, ey), points=[(x, y), ...])


def node(nid, shape, kind, x, y, w, h, title="", text=""):
    """Register one node (see NODES for the field meaning)."""
    NODES.append(dict(id=nid, shape=shape, kind=kind, x=x, y=y, w=w, h=h, title=title, text=text))


def edge(src, dst, exit_pt, entry_pt, points=()):
    """Register one arrow from node ``src`` to node ``dst``.

    ``exit_pt`` / ``entry_pt`` are relative anchors on the node's bounding box
    (0..1 in x and y), the same numbers draw.io uses as exitX/exitY/entryX/
    entryY. ``points`` are absolute waypoints between the two anchors.
    """
    EDGES.append(dict(src=src, dst=dst, exit=exit_pt, entry=entry_pt, points=list(points)))


# Column geometry shared by all containers.
COL_A = 100          # assembly robot (Cindy) column
COL_B = 500          # support robot (X) column
COL_C = 880          # notes column
BOX_W, BOX_H, PITCH = 230, 62, 72   # movement boxes inside a container
CONT_W = 270
MID_A = COL_A + CONT_W / 2   # x of column A's center line (235)
MID_B = COL_B + CONT_W / 2   # x of column B's center line (635)


def container(nid, x, y, title, movements):
    """A high-level action box holding a vertical list of movements.

    Args:
        nid (str): id prefix for the container and its movements.
        x, y (float): top-left corner of the container.
        title (str): container label (robot + action class + bar).
        movements (list): ``(movement_id, kind, text)`` tuples in order.

    Returns:
        float: the container's bottom y, so the caller can place the next node.
    """
    h = 30 + len(movements) * PITCH + 4
    node(nid, "box", "container", x, y, CONT_W, h, title=title)
    for k, (mid, kind, text) in enumerate(movements):
        by = y + 30 + k * PITCH
        node(f"{nid}.{mid}", "box", kind, x + 20, by, BOX_W, BOX_H, title=mid, text=text)
        if k > 0:
            prev = movements[k - 1][0]
            edge(f"{nid}.{prev}", f"{nid}.{mid}", (0.5, 1), (0.5, 0))
    return y + h


# * ---- legend (rebuilt from real shapes; the draft embedded it as an image)
node("leg1", "box", "plain", 100, 0, 300, 122)
node("leg1.circle", "ellipse", "plain", 112, 10, 22, 22)
node("leg1.t1", "text", "text", 145, 8, 240, 26, text="Flow control (start / end)")
node("leg1.rhombus", "rhombus", "plain", 108, 44, 30, 26)
node("leg1.t2", "text", "text", 145, 44, 240, 26, text="Flow control (conditional / loop over holds)")
node("leg1.box", "box", "plain", 110, 84, 26, 22)
node("leg1.t3", "text", "text", 145, 82, 240, 26, text="High-level action (one robot, one bar)")

node("leg2", "box", "plain", 420, 0, 740, 122)
LEGEND_SWATCHES = [
    (432, 10, "fm", "Free robotic movement (FM)"),
    (432, 40, "lm", "Linear robotic movement (LM)"),
    (432, 70, "fm_c", "Constrained dual-arm FM (both arms hold one bar, fixed relative flanges)"),
    (432, 100, "lm_c", "Constrained dual-arm LM (same constraint, straight-line flanges)"),
    (830, 10, "manual", "Manual movement (operator step, no planning)"),
    (830, 40, "tool", "Tool movement (screws / gripper, no planning)"),
    (830, 70, "tool_overlap", "Tool movement that keeps running through the NEXT movement"),
]
for k, (sx, sy, kind, label) in enumerate(LEGEND_SWATCHES):
    node(f"leg2.s{k}", "box", kind, sx, sy, 40, 20)
    node(f"leg2.t{k}", "text", "text", sx + 50, sy - 4, 300, 28, text=label)

# * ---- top row: process loop over the assembly sequence
node("start", "ellipse", "plain", 36, 178, 64, 64, text="Process starts")
node("init", "box", "plain", 140, 195, 70, 30, text="i = 0")
node("loop", "rhombus", "plain", 250, 170, 100, 80, title="i < N")
node("done", "ellipse", "plain", 1118, 178, 64, 64, text="Process complete")
node("inc", "box", "plain", 880, 260, 100, 30, text="Increment i")
node("t_seq", "text", "text", 110, 140, 290, 30,
     text="Assembly sequence + supported_until lists come from the Rhino document")
node("t_iN", "text", "text", 560, 176, 420, 26,
     text="i = index of the current bar in assembly_seq;  N = number of bars")
node("t_no1", "text", "text", 356, 190, 30, 20, text="No")
node("t_yes1", "text", "text", 268, 252, 30, 20, text="Yes")

edge("start", "init", (1, 0.5), (0, 0.5))
edge("init", "loop", (1, 0.5), (0, 0.5))
edge("loop", "done", (1, 0.5), (0, 0.5))

# * ---- column A, row 1: Cindy joints bar i (BarAssemblyJointingAction)
Y_J = 320
bottom_j = container("J", COL_A, Y_J, "Cindy: BarAssemblyJointingAction (bar i)", [
    ("J_M0", "fm", "Cindy's two arms travel from wherever they are to the bar loading pose "
                   "(FM, arms independent, planned live)"),
    ("J_M1", "manual", "Operator mounts bar i, joint halves already installed, into Cindy's "
                       "two end effectors (Manual)"),
    ("J_M2", "tool", "Both arm tools drive their grasping screws until they stall: the bar is clamped (TM)"),
    ("J_M3", "fm_c", "Cindy carries the bar to the approach pose, 15 mm off the assembled pose "
                     "(constrained dual-arm FM)  ·  target = approach keyframe"),
    ("J_M4", "tool_overlap", "Jointing screws tighten through J_M5 until they stall (TM, overlaps "
                             "next); only the arms whose male has its female built. Ground bar: see note"),
    ("J_M5", "lm_c", "Straight-line insert, approach to assembled, on the Cartesian compliant "
                     "controller; the screw stall ends it (constrained dual-arm LM)  ·  target = assembled keyframe"),
])
# Arrows end on the container border (as in the draft), not on the first box.
edge("loop", "J", (0.5, 1), (0.5, 0), points=[(300, 290), (MID_A, 290)])

# * ---- conditional: does bar i need a support robot?
RH_W, RH_H = 240, 120
y_q1 = bottom_j + 40
c_q1 = y_q1 + RH_H / 2
node("q_hold", "rhombus", "plain", MID_A - RH_W / 2, y_q1, RH_W, RH_H,
     text="Does bar i need holding? (a bar in its supported_until list is assembled LATER)")
edge("J", "q_hold", (0.5, 1), (0.5, 0))
node("t_yes2", "text", "text", MID_A + RH_W / 2 + 8, c_q1 - 22, 30, 20, text="Yes")
node("t_no2", "text", "text", MID_A + 8, y_q1 + RH_H + 6, 30, 20, text="No")

# * ---- column B, row 1: support robot X holds bar i (BarHoldingAction)
y_pick = c_q1 - 20
node("pick_x", "box", "plain", COL_B, y_pick, CONT_W, 40,
     text="Assign X = first free support robot (Alice before Belle); derived from the sequence, never stored")
edge("q_hold", "pick_x", (1, 0.5), (0, 0.5))
Y_H = y_pick + 70
bottom_h = container("H", COL_B, Y_H, "X: BarHoldingAction (bar i)", [
    ("H_M0", "fm", "X's arm travels from wherever it is to the approach pose, 100 mm off the grasp "
                   "along the gripper axis (single-arm FM, planned live)  ·  target = approach keyframe"),
    ("H_M1", "tool", "X's gripper opens to receive bar i (TM)"),
    ("H_M2", "lm", "Straight-line approach onto the hand-picked grasp pose on bar i "
                   "(single-arm LM)  ·  target = held keyframe"),
    ("H_M3", "tool", "X's gripper closes: bar i is held. X stays frozen here until every bar in "
                     "supported_until is built (TM)"),
])
edge("pick_x", "H", (0.5, 1), (0.5, 0))

# * ---- column A, row 2: Cindy lets go of bar i (BarAssemblyReleaseAction)
y_merge = bottom_h + 30
Y_R = y_merge + 40
bottom_r = container("R", COL_A, Y_R, "Cindy: BarAssemblyReleaseAction (bar i)", [
    ("R_M0", "tool", "Both tools run the grasping screws backwards: bar i becomes part of the "
                     "structure (TM). The jointing screws are never reversed"),
    ("R_M1", "lm", "Each arm retreats 50 mm along its joint's -Z axis, arms independent "
                   "(LM)  ·  target = retreat keyframe"),
    ("R_M2", "fm", "Both arms travel to the fixed dual-arm home configuration (FM, arms independent)"),
])
# "No" branch straight down; the hold branch merges into the same line.
edge("q_hold", "R", (0.5, 1), (0.5, 0))
edge("H", "R", (0.5, 1), (0.5, 0), points=[(MID_B, y_merge), (MID_A, y_merge)])
node("t_wait", "text", "text", 300, y_merge - 28, 330, 24,
     text="Cindy waits, still gripping bar i, until X has closed its gripper")

# * ---- loop: every hold whose LAST stabilizing bar is bar i releases now
RH2_W, RH2_H = 270, 130
y_q2 = bottom_r + 40
c_q2 = y_q2 + RH2_H / 2
node("q_rel", "rhombus", "plain", MID_A - RH2_W / 2, y_q2, RH2_W, RH2_H,
     text="Is there a hold h whose LAST stabilizing bar is bar i and that is not released yet? (hold-start order)")
edge("R", "q_rel", (0.5, 1), (0.5, 0))
node("t_yes3", "text", "text", MID_A + RH2_W / 2 + 8, c_q2 - 22, 30, 20, text="Yes")
node("t_no3", "text", "text", MID_A + 8, y_q2 + RH2_H + 6, 110, 20, text="No (all released)")

# * ---- column B, row 2: support robot X_h releases bar h (BarHoldingReleaseAction)
# Centered on the rhombus so the "Yes" arrow runs horizontally into the container.
HR_H = 30 + 2 * PITCH + 4
Y_HR = c_q2 - HR_H / 2
bottom_hr = container("HR", COL_B, Y_HR, "X_h: BarHoldingReleaseAction (bar h)", [
    ("HR_M0", "tool", "X_h's gripper opens, letting go of the now stable bar h (TM)"),
    ("HR_M1", "lm", "X_h's arm retreats straight back to its approach pose, the approach line "
                    "reversed (single-arm LM)  ·  target = approach keyframe"),
])
edge("q_rel", "HR", (1, 0.5), (0, 0.5))
node("n_away", "text", "text", COL_B + CONT_W + 15, c_q2 - 30, 180, 80,
     text="X_h then drives far away. Not a movement in the data model: X_h is simply "
          "dropped from every later planning scene.")
# Back into the loop rhombus at its lower-right edge, i.e. the (0.75, 0.75)
# anchor, coming from below so the return never crosses the "Yes" arrow.
y_back = bottom_hr + 40
entry_y = y_q2 + RH2_H * 0.75
edge("HR", "q_rel", (0.5, 1), (0.75, 0.75), points=[(MID_B, y_back), (420, y_back), (420, entry_y)])

# * ---- the "No" exit: next bar
y_next = bottom_hr + 100
edge("q_rel", "inc", (0.5, 1), (1, 0.5), points=[(MID_A, y_next), (1200, y_next), (1200, 275)])
edge("inc", "loop", (0, 0.5), (0.75, 0.75), points=[(325, 275)])

# * ---- notes column (what the picture cannot show)
NOTES = [
    ("Robot base pose",
     "Every action is solved at one robot base frame per bar (saved on the bar as user text). "
     "Driving the base there is done by the live stack before the first movement; it is not a "
     "movement in the data model, so the draft's base-movement (BM) boxes are gone.", 104),
    ("Start states and chaining",
     "Every movement stores a full RobotCellState snapshot: base frame, arm configuration, which "
     "bodies are attached to which flange, and which contacts are allowed. Arm movements chain: one "
     "movement's target configuration is the next one's start. J_M0 and H_M0 start configurations "
     "are None and filled live.", 128),
    ("Keyframes solved in Rhino (RSIKKeyframe)",
     "Assembly flow: approach (J_M3 target), assembled (insert target) and retreat (R_M1 target), all "
     "at one base. Support flow: held (H_M2 target) and approach (H_M0 target, reused as HR_M1's "
     "target) at X's own base. Free-motion trajectories are left to the headless planner.", 128),
    ("One planning scene per robot",
     "Each robot plans in its own PyBullet session. The other robots stand frozen as obstacles: "
     "Cindy at her assembled pose while X grasps; every X still holding stands at its held pose "
     "during all later bars until its release.", 104),
    ("Ground bars (foundation)",
     "No jointing motor runs. J_M0 to J_M3 as shown; then J_M4 = the straight-line insert, ended "
     "when the target is reached, and J_M5 = the operator tapes / fixes the foundation to the "
     "ground while Cindy keeps holding the bar (Manual). The tools grasp the ground joints and the "
     "insert follows the walkable ground's normal. A bar mixing ground and male joints is refused.", 152),
    ("Floor and allowed contacts",
     "Every robot's scene carries one floor slab per walkable ground the bars use. The wheels and "
     "the frozen robots always touch it; a ground joint may touch it during its insert and the "
     "fix step only.", 104),
]
ny = 320
for k, (title, text, h) in enumerate(NOTES):
    node(f"note{k}", "box", "note", COL_C, ny, 260, h, title=title, text=text)
    ny += h + 16

CANVAS_W, CANVAS_H = 1230, y_next + 40


# * ---------------------------------------------------------------- helpers
def anchor(n, rel):
    """Absolute point on node ``n``'s bounding box at relative position ``rel``."""
    return (n["x"] + n["w"] * rel[0], n["y"] + n["h"] * rel[1])


def edge_polyline(e, by_id):
    """All points of an edge in order: exit anchor, waypoints, entry anchor."""
    src, dst = by_id[e["src"]], by_id[e["dst"]]
    return [anchor(src, e["exit"])] + e["points"] + [anchor(dst, e["entry"])]


# * ---------------------------------------------------------------- draw.io writer
def write_drawio(path):
    """Write the node / edge lists as a draw.io (mxGraph) XML file."""
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<mxfile host="app.diagrams.net">',
        '  <diagram name="Page-1" id="action_flowchart_2026">',
        '    <mxGraphModel dx="1400" dy="900" grid="1" gridSize="10" guides="1" tooltips="1" '
        'connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="1240" pageHeight="1754" '
        'math="0" shadow="0">',
        "      <root>",
        '        <mxCell id="0" />',
        '        <mxCell id="1" parent="0" />',
    ]

    def attr(s):
        """Escape a string for use inside a double-quoted XML attribute."""
        return escape(s, {'"': "&quot;"})

    for n in NODES:
        if n["shape"] == "rhombus":
            style = DRAWIO_STYLE["rhombus"]
        elif n["shape"] == "ellipse":
            style = DRAWIO_STYLE["ellipse"]
        elif n["shape"] == "text":
            style = DRAWIO_STYLE["text"]
        else:
            style = DRAWIO_STYLE[n["kind"]]
        # Bold title on its own line, then the body text (HTML labels).
        parts = []
        if n["title"]:
            parts.append(f"<b>{escape(n['title'])}</b>")
        if n["text"]:
            parts.append(escape(n["text"]))
        value = "<br>".join(parts)
        lines.append(
            f'        <mxCell id="{attr(n["id"])}" value="{attr(value)}" style="{attr(style)}" '
            f'vertex="1" parent="1">'
        )
        lines.append(
            f'          <mxGeometry x="{n["x"]:g}" y="{n["y"]:g}" width="{n["w"]:g}" '
            f'height="{n["h"]:g}" as="geometry" />'
        )
        lines.append("        </mxCell>")

    for k, e in enumerate(EDGES):
        ex, ey = e["exit"]
        nx, ny_ = e["entry"]
        style = (
            DRAWIO_STYLE["edge"]
            + f"exitX={ex:g};exitY={ey:g};exitDx=0;exitDy=0;"
            + f"entryX={nx:g};entryY={ny_:g};entryDx=0;entryDy=0;"
        )
        lines.append(
            f'        <mxCell id="e{k}" style="{attr(style)}" edge="1" parent="1" '
            f'source="{attr(e["src"])}" target="{attr(e["dst"])}">'
        )
        lines.append('          <mxGeometry relative="1" as="geometry">')
        if e["points"]:
            lines.append('            <Array as="points">')
            for (px, py) in e["points"]:
                lines.append(f'              <mxPoint x="{px:g}" y="{py:g}" />')
            lines.append("            </Array>")
        lines.append("          </mxGeometry>")
        lines.append("        </mxCell>")

    lines += ["      </root>", "    </mxGraphModel>", "  </diagram>", "</mxfile>", ""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


# * ---------------------------------------------------------------- matplotlib render
PX2PT = 0.72   # 1 draw.io pixel = 1/100 inch on the figure = 0.72 pt


def _wrap(text, width_px, font_px):
    """Wrap ``text`` to fit ``width_px`` at ``font_px`` (rough average glyph width)."""
    chars = max(8, int((width_px - 12) / (font_px * 0.56)))
    return textwrap.wrap(text, width=chars)


def _draw_label(ax, n, font_px=10, align="center", valign="middle"):
    """Draw the bold title + wrapped body text of node ``n``."""
    x, y, w, h = n["x"], n["y"], n["w"], n["h"]
    # Notes, movement bodies and rhombus questions use a 9 px body so they
    # fit; titles (movement ids, container names) stay 10 px.
    body_px = 9 if (n["kind"] == "note" or n["title"] or n["shape"] == "rhombus") else font_px
    # A rhombus only offers about 60% of its bounding width to text.
    wrap_w = w * 0.62 if n["shape"] == "rhombus" else w
    lines = []
    if n["title"]:
        lines.append((n["title"], True))
    for ln in _wrap(n["text"], wrap_w, body_px) if n["text"] else []:
        lines.append((ln, False))
    if not lines:
        return
    line_h = body_px * 1.25
    total = line_h * len(lines)
    if valign == "top":
        y0 = y + 6 + line_h / 2
    else:
        y0 = y + h / 2 - total / 2 + line_h / 2
    if align == "left":
        tx, ha = x + 6, "left"
    else:
        tx, ha = x + w / 2, "center"
    color = "#333333" if n["kind"] in ("tool", "tool_overlap") else INK
    for k, (ln, bold) in enumerate(lines):
        ax.text(tx, y0 + k * line_h, ln, fontsize=(font_px if bold else body_px) * PX2PT,
                ha=ha, va="center", color=color, weight="bold" if bold else "normal", zorder=5)


def _draw_node(ax, n):
    """Draw one node's shape and label."""
    x, y, w, h = n["x"], n["y"], n["w"], n["h"]
    fill, stroke, grad, dashed = KIND.get(n["kind"], KIND["plain"])
    ls = (0, (4, 2.5)) if dashed else "-"
    if n["shape"] == "text":
        _draw_label(ax, n, align="left")
        return
    if n["shape"] == "ellipse":
        patch = Ellipse((x + w / 2, y + h / 2), w, h, facecolor=fill, edgecolor=stroke, lw=0.8, zorder=2)
    elif n["shape"] == "rhombus":
        pts = [(x + w / 2, y), (x + w, y + h / 2), (x + w / 2, y + h), (x, y + h / 2)]
        patch = Polygon(pts, closed=True, facecolor=fill, edgecolor=stroke, lw=0.8, zorder=2)
    else:
        patch = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=6",
                               facecolor=fill, edgecolor=stroke, lw=0.8, ls=ls,
                               zorder=1 if n["kind"] == "container" else 2)
    ax.add_patch(patch)
    if grad is not None:
        # Horizontal gradient from the fill color to the gradient color, clipped to the box.
        c0, c1 = np.array(to_rgb(fill)), np.array(to_rgb(grad))
        t = np.linspace(0, 1, 128)[None, :, None]
        img = c0 * (1 - t) + c1 * t
        im = ax.imshow(img, extent=(x, x + w, y + h, y), aspect="auto", zorder=2, interpolation="bilinear")
        im.set_clip_path(patch)
    if n["kind"] in ("container", "note"):
        _draw_label(ax, n, align="left", valign="top")
    else:
        _draw_label(ax, n)


def _draw_edge(ax, e, by_id):
    """Draw one edge as an orthogonal polyline with an arrowhead at the end."""
    pts = edge_polyline(e, by_id)
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=INK, lw=0.8, zorder=3, solid_joinstyle="miter")
    ax.annotate("", xy=pts[-1], xytext=pts[-2],
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.8, mutation_scale=7,
                                shrinkA=0, shrinkB=0), zorder=4)


def render_png(path):
    """Render the node / edge lists to a PNG with matplotlib."""
    by_id = {n["id"]: n for n in NODES}
    fig = plt.figure(figsize=(CANVAS_W / 100.0, CANVAS_H / 100.0), dpi=220)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, CANVAS_W)
    ax.set_ylim(CANVAS_H, 0)
    ax.axis("off")
    # Containers first so the movement boxes sit on top of them.
    for n in sorted(NODES, key=lambda n: 0 if n["kind"] == "container" else 1):
        _draw_node(ax, n)
    for e in EDGES:
        _draw_edge(ax, e, by_id)
    fig.savefig(path, dpi=220, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    drawio_path = os.path.join(here, "action_flowchart_2026.drawio")
    png_path = os.path.join(here, "action_flowchart_2026.png")
    write_drawio(drawio_path)
    render_png(png_path)
    print(f"wrote {drawio_path}\nwrote {png_path}")
