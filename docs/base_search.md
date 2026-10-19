# Base search: math and algorithm

How RSIKKeyframe and RSIKKeyframeAll choose where to try the IK chain when the picked base fails
(`BaseSearch = Optimised`). Code: `scripts/core/base_region.py` (geometry and cost),
`scripts/core/base_search.py` (search), `scripts/core/base_search_viz.py` (debug trace).
Default values are from `scripts/core/config.py` (`IK_BASE_SEARCH_*`).

The search only proposes bases. Every proposal is accepted or rejected by the full IK chain solve
(approach, assembled, retreat), exactly as for the picked base.

## 1. Order of events

1. Solve the IK chain at the **seed** (the picked or placed base). Success ends here; the search never runs.
2. **Descent from the seed** (section 5). If it moves the base, that is proposal 1; solve the chain there.
3. Only now the **ground grid** is computed (section 4): every cell within the search radius, at its cheapest
   allowed heading, sorted by arm cost. Closed form, no planner, a few milliseconds.
4. **Walk the sorted cells**, cheapest first. Per cell, one collision check with the arms at home:
   - collides: skip it (red circle) and keep the cells within 75 mm out of the walk;
   - free: run a descent from it, propose the result, solve the chain there.
5. If the chain fails again, continue the walk from where it stopped, skipping cells near earlier proposals.

Nothing is random. The next start is the cheapest unused cell, not a random location.

Limits: 10 proposals (`MAX_PROPOSALS`), 300 grid-cell checks (`MAX_CHECKS`; the descents' own checks come on top).

## 2. Geometry of one arm and one target

Per arm, from the URDF (`wrist_reach`, computed once per session):

- **Pivot** $p$: the origin of the arm's first moving joint, in the base frame. Turning that joint does not move
  the pivot.
- **Wrist point**: the point on the last joint's axis nearest the axis of the joint before it (on a UR, where the
  wrist 2 and wrist 3 axes cross, about 99.5 mm behind the flange). A flange target fixes this point in the world,
  because the point is on the last axis and the last joint does not move it. Written $w$ for a given target.
- **Reach** $R$: the largest distance from pivot to wrist point the joints in between allow, found numerically
  (400 random joint sets, the best 8 refined with Powell's method) plus a 20 mm margin. Joint limits are ignored,
  so $R$ is generous (about 0.947 m for these arms).

A base pose is reachable for a target only if $\lVert w - \text{pivot}_\text{world} \rVert \le R$. This is
necessary, not sufficient: joint limits, orientation of the inner joints and collisions can still fail.

## 3. The base pose and the closed-form heading test

A base is a ground point $o$ with the ground's unit normal $n$, and a heading $\psi$. Heading 0 is world X laid
flat on the ground ($a$, "ahead"); $l = n \times a$ is heading 90 degrees.

Write the pivot in the base frame as $p = (p_x, p_y, p_z)$, with $r = \sqrt{p_x^2 + p_y^2}$ and
$\varphi = \operatorname{atan2}(p_y, p_x)$. In the world the pivot sits at

$$
\text{pivot}(\psi) = o + p_z\,n + r\,\big(\cos(\psi+\varphi)\,a + \sin(\psi+\varphi)\,l\big).
$$

Let $d = w - o - p_z n$, with components $x = d\cdot a$, $y = d\cdot l$, $h = d\cdot n$, and
$\rho = \sqrt{x^2+y^2}$, $\beta = \operatorname{atan2}(y, x)$. By the law of cosines

$$
\lVert w - \text{pivot}(\psi) \rVert^2 = h^2 + \rho^2 + r^2 - 2 r \rho \cos(\psi + \varphi - \beta).
$$

So the reach condition at one ground point is

$$
\cos(\psi + \varphi - \beta) \;\ge\; \frac{h^2 + \rho^2 + r^2 - R^2}{2 r \rho},
$$

which is a single arc of headings centred on $\psi = \beta - \varphi$ (or no heading, or all of them).
`feasible_yaws` tests 72 headings (5 degrees apart) against every target and keeps those that pass all.
The intersection of several arcs can be two separate runs; `widest_run` finds the longest one.

## 4. The cost

**Arm extension** (`extension_cost`). Per target, $e = \lVert w - \text{pivot}(\psi)\rVert / R$, from the
formula above. With the preferred extension $e^* = 0.65$:

$$
c(e) = \begin{cases}
\left(\dfrac{e - e^*}{1 - e^*}\right)^2 & e > e^* \\[2ex]
\left(\dfrac{e^* - e}{e^*}\right)^2 & e \le e^*
\end{cases}
\qquad
E = \frac{1}{6}\sum_{\text{2 arms} \times \text{3 keyframes}} c(e).
$$

$c$ is 0 at the preferred extension and 1 both fully stretched and folded onto the pivot.

**Clearance** (`crowding`). With the arms at home, PyBullet's closest distance $\delta$ from the robot to the
scene's standing bodies, measured up to $\delta_0 = 0.3$ m. With weight $k = 0.25$:

$$
C = k \left(1 - \frac{\min(\delta, \delta_0)}{\delta_0}\right)^2 .
$$

**Total**: $J = E + C$. A base that cannot be used gets $J = 1000$ (section 5).

**Grid ranking** (`ranked_cells`). Ground points on a 50 mm grid (where grounds overlap in plan, the highest),
within 1000 mm of the seed in plan. Per cell, headings within 90 degrees of the seed's heading and passing the
reach test; the cell's score is $\min_\psi E$ over those, with $C$ left out (no planner here). Cells with no
allowed heading are dropped.

## 5. The descent

Nelder-Mead on three scaled variables $u$ from a start base $(x_0, y_0, \psi_0)$:

$$
x = x_0 + 50\,u_1\ \text{mm},\quad y = y_0 + 50\,u_2\ \text{mm},\quad \psi = \psi_0 + 5^\circ u_3 .
$$

The initial simplex is the start and one step of one grid cell or one heading step along each variable; at most
40 cost evaluations. Derivative-free because $C$ has kinks wherever another pair of bodies becomes the nearest,
and each evaluation costs a planner call.

Each evaluation, in order (the first that applies returns 1000 and stops):

| Rule | Cost of checking | Trace mark |
|---|---|---|
| more than 1000 mm from the seed in plan | none | none |
| within 200 mm of an earlier proposal (not for the seed's descent) | none | none |
| turned more than 90 degrees from the seed's heading | none | none |
| off the ground (the ground point is more than 1 mm away in plan) | snap to ground | none |
| a target out of reach (section 3) | closed form | none |
| collides with the arms at home | one planner call | red circle |
| otherwise: compute $J$ | planner call + clearance | green circle, long arrow |

The descent returns the cheapest base it evaluated. A descent whose start is already the cheapest returns the
start; for the seed's descent nothing is then proposed.

**Home check** (`HomeCheck`). The robot with its arms at the home configuration, on the retreat scene (the new
bar in place and let go). Two checks:

- **Once, at the first base**: the robot against itself and its own tools (compas_fab's CC.1 and CC.2). This does
  not depend on the base. If the home pose collides here, every base is ruled out.
- **Per base**: the robot and its tools against the scene's bodies: bars, joints, ground (CC.3 to CC.5).

**Known limitation: other robots are not checked by the base search.** A support robot standing where this robot
would stand, or in its tools' way, does not rule a base out here. Only the keyframe IK solved at the proposed base
checks against other robots (CC.2 for the arms, CC.6 for the tools), and fails there. The once-only check skips
CC.6 on purpose (`_skip_cc6`): the other robots stand still while the base moves, so one answer at the first base
would be wrong for the rest. (Its CC.2 does pair the robot's links with the other robots too, but only as they
stand from the first base: a link hitting another robot there rules out every base.)

## 6. Reading the debug trace

`DEBUG_BASE_SEARCH_TRACE = True` draws, for the last search:

- **Short arrows** (only with `DEBUG_BASE_SEARCH_GRID = True`): the ground grid, at each cell's cheapest heading,
  shaded by $E$ (dark = cheap).
- **Long arrows**: every base whose full cost $J$ was computed, shaded on the same scale.
- **Circles**: every collision check. Green = free, red = collides.
- **Lines**: one per descent, from its start through each base that was **cheaper than every base before it** in
  that descent.
- **Big dots**: the proposals (the chain was solved there).

The circles are not one connected walk:

- Each descent is drawn as its own line; descents are not joined to each other or to the seed.
- Within a descent, Nelder-Mead also evaluates bases that turn out worse (reflections, contractions). Those get a
  circle and an arrow but no line.
- A base that collides inside a descent gets a red circle and no line.
- Grid cells checked during the walk that collide are single red circles. They start no descent.
- Bases ruled out by geometry (radius, heading, off ground, reach) get no mark at all.
