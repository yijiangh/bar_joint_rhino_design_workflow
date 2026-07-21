# Defining tool candidates and testing them in IK keyframing

How to define two new assembly-tool candidate sets (example: `AT4` and `AT5`)
and A/B-test their kinematic reachability. Each set is one physical tool design
with a hand-made Left and Right version.

## 1. Model the geometry in Rhino (per set)

For each candidate set, prepare **per side** (left and right — tools are not
symmetric):

1. **A block definition** with the tool's visual geometry. The block's local
   origin/orientation must be the **robot flange (tool0) frame** — same
   convention as the existing `LeftHandTool_SimpleVis` / `RightHandTool_SimpleVis`
   blocks. These are source block names only; exported assets and imported
   definitions are named from the registered tool name.
2. **A low-poly collision mesh**, hand-modeled and positioned on the tool
   instance. If the block itself is already a low-poly mesh, skip this — you
   will pick the block as the collision source. Breps are never auto-meshed.
3. **Three TCP points**: origin, +X tip, +Y tip. This is the frame of the male
   joint while held. Your reachability experiment lives here: model the tool /
   place the TCP so the held joint sits wherever you want relative to the
   flange (that offset becomes `M_tcp_from_block`).

## 2. Define each tool — `RSDefineRoboticTool`, 4 runs total

Run **RSDefineRoboticTool → AssemblyTool** once per side, per set:

1. Pick the tool block instance.
2. Pick TCP origin, +X tip, +Y tip (previous picks auto-hide).
3. Pick the collision mesh source(s): loose mesh(es) **and/or** the tool block
   itself (it stays visible for this pick). All picks merge into one OBJ, so
   don't pick both the block and a coincident loose mesh.
4. Type the tool name. **It must end in `L` or `R`**, and the pair shares a
   prefix: `AT4L`/`AT4R`, then `AT5L`/`AT5R`. The suffix assigns the arm side;
   the prefix pairs the two.

Each run writes `asset/<tool-name>.3dm` + `asset/<tool-name>.obj` and the registry
entry in `scripts/core/robotic_tools.json`. Reusing an exact tool name replaces
its existing entry instead of adding a duplicate. Defining does **not** activate
anything yet — the console reminds you when a pair is complete.

## 3. Activate one set — `RSSwapRoboticTool`, type one name

Click **RSSwapRoboticTool**. The command prints all names registered in
`robotic_tools.json`; type the exact name of either side (for example `AT4L` or
`AT4R`). No candidate block needs to exist in the active Rhino document. The
command swaps both sides:

- registry `active` entry → `AT4L` + `AT4R`
- every placed tool instance in the doc is re-placed with the matching side
  (each joint keeps the arm side it had)
- both tool block definitions are imported/force-refreshed from `asset/`, even
  when the active Rhino document currently has no placed tool instances
- bars carrying already-solved IK keyframes are listed in a **warning** — those
  were solved with the old tools and must be re-solved

It refuses cleanly (nothing changes) if the pair partner is missing or a
`.3dm`/`.obj` is absent.

## 4. Rebuild the collision cell and re-solve IK

1. **RSPBStart** (if PyBullet isn't running).
2. Accept the rebuild prompt at the end of the swap, or click
   **RSRebuildRobotCell**. The summary should list `AT4L`/`AT4R`.
   (If you forget, the next IK command shows a staleness prompt — the active
   tool pair is part of the cell fingerprint.)
3. **RSIKKeyframe** on the bars you care about and check solve success /
   collision results as usual. **RSClearIKKeyframe** to erase stale solves.

## 5. Compare the two sets

Repeat steps 3–4 with the other set (`RSSwapRoboticTool` → type `AT5L` or
`AT5R` → rebuild → re-solve the same bars). Whichever pair solves more bars /
with better margins wins. The registry remembers only one active pair at a
time.

After settling on a tool, re-export (`RSExportBarAction` / RobotCell JSON) —
exported files embed the tool geometry and names, so exports made with the
old pair are stale.
