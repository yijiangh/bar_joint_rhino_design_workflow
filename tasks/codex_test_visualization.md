# Task: Add Visual Debugging to All Tests

## Approach

Add a `--viz` pytest flag. When passed, tests that have visual debugging show a matplotlib or PyBullet window. Without the flag, tests run normally (no windows, no blocking).

```
pytest tests/ -v              # normal, no visuals
pytest tests/ -v --viz        # shows plots after each visualizable test
pytest tests/test_s2_t1.py -v --viz   # only S2 visuals
```

## Files to Modify / Create

| File | Action |
|------|--------|
| `tests/conftest.py` | Add `--viz` flag + `viz` fixture |
| `tests/viz_helpers.py` | **New** — shared matplotlib plotting utilities |
| `tests/test_geometry.py` | Add viz calls to 6 tests |
| `tests/test_kinematics.py` | Add viz calls to 10 tests |
| `tests/test_s2_t1.py` | Add viz calls to 5 tests |
| `tests/test_urdf_chain.py` | Switch to PyBullet GUI mode when `--viz` |

---

## 1. `tests/conftest.py`

Add the `--viz` CLI option and a `viz` fixture.

```python
import sys
import os
import pytest

# existing path setup
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


def pytest_addoption(parser):
    parser.addoption("--viz", action="store_true", default=False,
                     help="Show visual plots for tests that support it")


@pytest.fixture
def viz(request):
    """Fixture that returns a VizContext if --viz is passed, or a no-op otherwise."""
    if request.config.getoption("--viz"):
        from viz_helpers import VizContext
        ctx = VizContext(enabled=True)
        yield ctx
        ctx.show()  # called after the test body runs
    else:
        from viz_helpers import VizContext
        yield VizContext(enabled=False)


@pytest.fixture(scope="module")
def viz_enabled(request):
    """Module-level flag for tests that need to know early (e.g., PyBullet mode)."""
    return request.config.getoption("--viz")
```

---

## 2. `tests/viz_helpers.py` (NEW file)

Shared matplotlib utilities. All drawing is no-op when `enabled=False`.

```python
"""Visualization helpers for test debugging.

Usage in tests:
    def test_something(self, viz):
        # ... run assertions ...
        viz.plot_line(start, end, color='grey', label='Le')
        viz.plot_point(pt, color='red', label='closest')
        viz.plot_frame(frame_4x4, scale=20, label='female OCF')
        viz.set_title("test_something")
"""

import numpy as np


class VizContext:
    """Collects draw commands during a test. Shows a matplotlib figure at the end."""

    def __init__(self, enabled: bool):
        self.enabled = enabled
        self._commands = []

    # ---- Drawing commands (no-op if disabled) ----

    def plot_line(self, start, end, color='black', linewidth=2, label=None, linestyle='-'):
        """Draw a 3D line segment."""
        if not self.enabled:
            return
        self._commands.append(('line', {
            'start': np.asarray(start, dtype=float),
            'end': np.asarray(end, dtype=float),
            'color': color, 'linewidth': linewidth,
            'label': label, 'linestyle': linestyle,
        }))

    def plot_point(self, point, color='red', size=40, label=None):
        """Draw a 3D point."""
        if not self.enabled:
            return
        self._commands.append(('point', {
            'point': np.asarray(point, dtype=float),
            'color': color, 'size': size, 'label': label,
        }))

    def plot_segment(self, start, end, color='blue', linewidth=1.5, linestyle='--', label=None):
        """Draw a dashed segment (e.g., shortest distance line)."""
        if not self.enabled:
            return
        self._commands.append(('line', {
            'start': np.asarray(start, dtype=float),
            'end': np.asarray(end, dtype=float),
            'color': color, 'linewidth': linewidth,
            'label': label, 'linestyle': linestyle,
        }))

    def plot_frame(self, frame_4x4, scale=20.0, label=None):
        """Draw a 4x4 homogeneous frame as RGB axis tripod (X=red, Y=green, Z=blue)."""
        if not self.enabled:
            return
        self._commands.append(('frame', {
            'frame': np.asarray(frame_4x4, dtype=float),
            'scale': scale, 'label': label,
        }))

    def plot_box_wireframe(self, frame_4x4, size_xyz, color='pink', label=None):
        """Draw a wireframe box at the given frame with given extents."""
        if not self.enabled:
            return
        self._commands.append(('box', {
            'frame': np.asarray(frame_4x4, dtype=float),
            'size': np.asarray(size_xyz, dtype=float),
            'color': color, 'label': label,
        }))

    def plot_text(self, point, text, color='black', fontsize=10):
        """Place text at a 3D point."""
        if not self.enabled:
            return
        self._commands.append(('text', {
            'point': np.asarray(point, dtype=float),
            'text': text, 'color': color, 'fontsize': fontsize,
        }))

    def set_title(self, title: str):
        if not self.enabled:
            return
        self._commands.append(('title', {'title': title}))

    # ---- Rendering ----

    def show(self):
        """Render all accumulated commands in a matplotlib 3D plot."""
        if not self.enabled or not self._commands:
            return

        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        all_points = []
        title = ""

        for cmd_type, data in self._commands:
            if cmd_type == 'line':
                s, e = data['start'], data['end']
                ax.plot([s[0], e[0]], [s[1], e[1]], [s[2], e[2]],
                        color=data['color'], linewidth=data['linewidth'],
                        linestyle=data['linestyle'], label=data.get('label'))
                all_points.extend([s, e])

            elif cmd_type == 'point':
                p = data['point']
                ax.scatter(*p, color=data['color'], s=data['size'],
                           label=data.get('label'), zorder=5)
                all_points.append(p)

            elif cmd_type == 'frame':
                f = data['frame']
                origin = f[:3, 3]
                scale = data['scale']
                colors = ['red', 'green', 'blue']
                axis_labels = ['X', 'Y', 'Z']
                for i in range(3):
                    tip = origin + scale * f[:3, i]
                    ax.plot([origin[0], tip[0]], [origin[1], tip[1]], [origin[2], tip[2]],
                            color=colors[i], linewidth=2)
                    ax.text(*tip, axis_labels[i], color=colors[i], fontsize=8)
                if data.get('label'):
                    ax.text(*origin, data['label'], fontsize=9, color='black')
                all_points.append(origin)

            elif cmd_type == 'box':
                f = data['frame']
                sz = data['size']
                origin = f[:3, 3]
                R = f[:3, :3]
                half = sz / 2.0
                # 8 corners of the box in local frame
                corners_local = np.array([
                    [-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                    [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]
                ], dtype=float) * half
                corners_world = (R @ corners_local.T).T + origin
                # Draw 12 edges
                edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
                         (0,4),(1,5),(2,6),(3,7)]
                for i, j in edges:
                    ax.plot([corners_world[i,0], corners_world[j,0]],
                            [corners_world[i,1], corners_world[j,1]],
                            [corners_world[i,2], corners_world[j,2]],
                            color=data['color'], linewidth=0.8)
                all_points.extend(corners_world.tolist())

            elif cmd_type == 'text':
                ax.text(*data['point'], data['text'],
                        color=data['color'], fontsize=data['fontsize'])

            elif cmd_type == 'title':
                title = data['title']

        # Set equal aspect ratio
        if all_points:
            pts = np.array(all_points)
            center = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
            radius = max(0.55 * np.max(pts.max(axis=0) - pts.min(axis=0)), 1.0)
            ax.set_xlim(center[0] - radius, center[0] + radius)
            ax.set_ylim(center[1] - radius, center[1] + radius)
            ax.set_zlim(center[2] - radius, center[2] + radius)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        if title:
            ax.set_title(title)
        ax.legend(loc='upper left', fontsize=8)
        plt.tight_layout()
        plt.show()
```

---

## 3. `tests/test_geometry.py` — Add viz calls

Add `viz` fixture parameter to every test method that benefits from visualization. Add viz calls **after** the assertions (so the test still passes/fails normally, the plot is just extra).

### TestClosestParamsInfiniteLines

For each test, visualize: both infinite lines (as long segments), closest points, and the shortest segment between them.

Helper to add at top of file:
```python
def _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_i, t_j, title=""):
    """Visualize two infinite lines, their closest points, and the connecting segment."""
    # Draw lines as long segments centered at their anchor points
    extent = max(abs(t_i), abs(t_j), 5) * 1.5
    viz.plot_line(x_i - extent * n_i, x_i + extent * n_i,
                  color='grey', linewidth=3, label='Line A')
    viz.plot_line(x_j - extent * n_j, x_j + extent * n_j,
                  color='orange', linewidth=3, label='Line B')
    # Closest points
    p_a = x_i + t_i * n_i
    p_b = x_j + t_j * n_j
    viz.plot_point(p_a, color='red', size=60, label=f'Closest on A (t={t_i:.2f})')
    viz.plot_point(p_b, color='blue', size=60, label=f'Closest on B (t={t_j:.2f})')
    # Shortest segment
    dist = np.linalg.norm(p_a - p_b)
    viz.plot_segment(p_a, p_b, color='magenta', label=f'Dist={dist:.3f}')
    # Anchor points
    viz.plot_point(x_i, color='grey', size=20, label='Anchor A')
    viz.plot_point(x_j, color='orange', size=20, label='Anchor B')
    viz.set_title(title)
```

Then in each test (example for `test_skew_lines_offset`):
```python
def test_skew_lines_offset(self, viz):
    x_i, n_i = np.array([0.,0,0]), np.array([1.,0,0])
    x_j, n_j = np.array([1.,2,3]), np.array([0.,1,0])
    tA, tB = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
    assert abs(tA - 1.0) < TOL
    assert abs(tB - (-2.0)) < TOL
    # ... existing assertions ...
    _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, tA, tB,
                        title="test_skew_lines_offset")
```

**Apply this pattern to all 5 tests in TestClosestParamsInfiniteLines** (perpendicular_at_origins, skew_lines_offset, 45_degree_lines, symmetric_config, parallel tests).

### TestClosestParamsFiniteSegments

Similar but draw segments with endpoints instead of infinite lines:

```python
def _viz_finite_segments(viz, p1_start, p1_end, p2_start, p2_end, t1, t2, title=""):
    d1 = p1_end - p1_start
    d2 = p2_end - p2_start
    viz.plot_line(p1_start, p1_end, color='grey', linewidth=4, label='Segment 1')
    viz.plot_line(p2_start, p2_end, color='orange', linewidth=4, label='Segment 2')
    # Closest points
    cp1 = p1_start + t1 * d1
    cp2 = p2_start + t2 * d2
    viz.plot_point(cp1, color='red', size=60, label=f'Closest on S1 (t={t1:.2f})')
    viz.plot_point(cp2, color='blue', size=60, label=f'Closest on S2 (t={t2:.2f})')
    viz.plot_segment(cp1, cp2, color='magenta',
                     label=f'Dist={np.linalg.norm(cp1-cp2):.3f}')
    # Endpoints
    for pt in [p1_start, p1_end]:
        viz.plot_point(pt, color='grey', size=15)
    for pt in [p2_start, p2_end]:
        viz.plot_point(pt, color='orange', size=15)
    viz.set_title(title)
```

**Apply to**: test_interior_solution, test_boundary_clipping, test_coincides_with_infinite_when_inside.

### TestDistanceInfiniteLines

Reuse `_viz_infinite_lines` — just compute t values from `closest_params_infinite_lines` and plot. Apply to the 3 tests that have geometry (not the error-case tests).

---

## 4. `tests/test_kinematics.py` — Add viz calls

### TestFKSanity (4 tests)

For each test, visualize the bar and the resulting FK frame:

```python
def _viz_fk_result(viz, bar_start, bar_end, frame, bar_color='grey',
                   frame_label='OCF', title=""):
    viz.plot_line(bar_start, bar_end, color=bar_color, linewidth=5,
                  label=f'Bar ({bar_color})')
    viz.plot_frame(frame, scale=15, label=frame_label)
    # Show bar axis as thin dashed line extended beyond endpoints
    bar_dir = bar_end - bar_start
    bar_unit = bar_dir / np.linalg.norm(bar_dir)
    viz.plot_line(bar_start - 20*bar_unit, bar_end + 20*bar_unit,
                  color=bar_color, linewidth=0.5, linestyle='--')
    viz.set_title(title)
```

Example for `test_fk_female_origin_on_correct_side`:
```python
def test_fk_female_origin_on_correct_side(self, viz):
    Le_start = np.array([-100., 0, 0])
    Le_end = np.array([100., 0, 0])
    frame = fk_female(Le_start, Le_end, 50.0, 0.0, 0.0, config)
    # ... existing assertions ...
    _viz_fk_result(viz, Le_start, Le_end, frame,
                   bar_color='grey', frame_label='Female OCF',
                   title="test_fk_female_origin_on_correct_side")
```

For `test_fjr_rotates_radial_direction`, plot both frames (fjr=0 and fjr=pi):
```python
def test_fjr_rotates_radial_direction(self, viz):
    Le_start, Le_end = np.array([0.,0,0]), np.array([200.,0,0])
    f0 = fk_female(Le_start, Le_end, 100.0, 0.0, 0.0, config)
    f_pi = fk_female(Le_start, Le_end, 100.0, np.pi, 0.0, config)
    # ... existing assertions ...
    viz.plot_line(Le_start, Le_end, color='grey', linewidth=5, label='Le')
    viz.plot_frame(f0, scale=15, label='FJR=0')
    viz.plot_frame(f_pi, scale=15, label='FJR=pi')
    viz.plot_point([100, 0, 0], color='black', size=30, label='Bar contact pt')
    viz.set_title("test_fjr_rotates_radial_direction")
```

### TestOptimizeJointPlacement (5 tests)

These are the most important to visualize. For each test, plot both bars + both resulting joint frames:

```python
def _viz_optimization_result(viz, Le_start, Le_end, Ln_start, Ln_end, result, title=""):
    viz.plot_line(Le_start, Le_end, color='grey', linewidth=5, label='Le (existing)')
    viz.plot_line(Ln_start, Ln_end, color='orange', linewidth=5, label='Ln (new)')
    viz.plot_frame(result['female_frame'], scale=12, label='Female OCF')
    viz.plot_frame(result['male_frame'], scale=12, label='Male OCF')
    # Connector line between the two OCF origins
    f_origin = result['female_frame'][:3, 3]
    m_origin = result['male_frame'][:3, 3]
    viz.plot_segment(f_origin, m_origin, color='purple',
                     label=f"Residual={result['residual']:.2e}")
    viz.plot_text(f_origin + [0, 5, 0],
                  f"FJP={result['fjp']:.1f} FJR={np.degrees(result['fjr']):.0f}deg",
                  fontsize=8)
    viz.plot_text(m_origin + [0, 5, 0],
                  f"MJP={result['mjp']:.1f} MJR={np.degrees(result['mjr']):.0f}deg",
                  fontsize=8)
    viz.set_title(title)
```

Apply to all 5 optimization tests: perpendicular_bars, angled_bars, skew_bars_3d, dof_values_within_bounds, fjp_mjp_near_contact_point.

---

## 5. `tests/test_s2_t1.py` — Add viz calls

### TestS2T1 (4 tests)

For each test, plot existing bars, contact points, and the resulting new bar + contact normals:

```python
def _viz_s2_result(viz, n1, Ce1, n2, Ce2, nn, P1, P2, D, title=""):
    # Existing bars as long lines through contact points
    viz.plot_line(Ce1 - 60*n1, Ce1 + 60*n1, color='grey', linewidth=4, label='Le1')
    viz.plot_line(Ce2 - 60*n2, Ce2 + 60*n2, color='dimgrey', linewidth=4, label='Le2')
    # Contact points on existing bars
    viz.plot_point(Ce1, color='red', size=60, label='Ce1')
    viz.plot_point(Ce2, color='blue', size=60, label='Ce2')
    # New bar through P1 and P2
    mid = 0.5 * (P1 + P2)
    viz.plot_line(mid - 80*nn, mid + 80*nn, color='orange', linewidth=4, label='Ln (result)')
    # Points on Ln
    viz.plot_point(P1, color='salmon', size=40, label='P1 (on Ln)')
    viz.plot_point(P2, color='cornflowerblue', size=40, label='P2 (on Ln)')
    # Contact normal segments (Ce1→P1, Ce2→P2)
    viz.plot_segment(Ce1, P1, color='red', label=f'd1={np.linalg.norm(P1-Ce1):.2f}')
    viz.plot_segment(Ce2, P2, color='blue', label=f'd2={np.linalg.norm(P2-Ce2):.2f}')
    viz.set_title(title)
```

### TestS2T1AllSolutions

Visualize all solutions overlaid with different colors:

```python
def _viz_s2_all_solutions(viz, n1, Ce1, n2, Ce2, solutions, D, title=""):
    # Existing bars
    viz.plot_line(Ce1 - 60*n1, Ce1 + 60*n1, color='grey', linewidth=4, label='Le1')
    viz.plot_line(Ce2 - 60*n2, Ce2 + 60*n2, color='dimgrey', linewidth=4, label='Le2')
    viz.plot_point(Ce1, color='red', size=60, label='Ce1')
    viz.plot_point(Ce2, color='blue', size=60, label='Ce2')
    # Each solution in a different color
    sol_colors = ['orange', 'cyan', 'lime', 'magenta']
    for i, sol in enumerate(solutions):
        mid = 0.5 * (sol['p1'] + sol['p2'])
        nn = sol['nn']
        c = sol_colors[i % len(sol_colors)]
        viz.plot_line(mid - 80*nn, mid + 80*nn, color=c, linewidth=3,
                      label=f"Sol {i+1} signs={sol['signs']} res={sol['residual']:.1e}")
        viz.plot_segment(Ce1, sol['p1'], color=c, linestyle=':')
        viz.plot_segment(Ce2, sol['p2'], color=c, linestyle=':')
    viz.set_title(title)
```

---

## 6. `tests/test_urdf_chain.py` — PyBullet GUI mode

For URDF tests, the visualization approach is different: switch PyBullet from `DIRECT` to `GUI` mode and pause for inspection.

Modify the `robot` fixture to accept `viz_enabled`:

```python
@pytest.fixture(scope="module")
def robot(request, viz_enabled):
    """Load URDF. Use GUI mode if --viz is passed."""
    assert os.path.exists(URDF_PATH), "Run generate_urdf.py first"

    if viz_enabled:
        cid = p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.8, cameraYaw=45, cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.15])
    else:
        cid = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(URDF_PATH, basePosition=[0, 0, 0], useFixedBase=True)

    # ... existing joint_map/link_map setup ...

    yield {"id": robot_id, "joints": joint_map, "links": link_map,
           "client": cid, "viz": viz_enabled}
    p.disconnect(cid)
```

Then add a helper used after each test's assertions:

```python
def _pause_if_viz(robot, title=""):
    """If in GUI mode, add a debug text and wait for user to press Enter."""
    if robot.get("viz"):
        p.addUserDebugText(title, [0, 0, 0.5], textColorRGB=[0, 0, 0], textSize=1.5)
        input(f"  [VIZ] {title} — press Enter to continue...")
```

Add `_pause_if_viz(robot, "test_name")` at the end of each test that modifies joint values. For example:

```python
def test_fjp_moves_female_along_le(self, robot):
    set_joints(robot, {"fjp_joint": 0, ...})
    pos0 = get_link_pos(robot, "female_link").copy()
    set_joints(robot, {"fjp_joint": 100, ...})
    pos1 = get_link_pos(robot, "female_link")
    # ... assertions ...
    _pause_if_viz(robot, "test_fjp_moves_female_along_le (FJP=100)")
```

**Important**: Since the `robot` fixture is `scope="module"`, all tests in the file share one PyBullet session. This means in GUI mode the user sees the state evolve from test to test, which is actually ideal for debugging.

---

## Implementation Order

1. `tests/conftest.py` — add `--viz` flag and fixtures
2. `tests/viz_helpers.py` — create the VizContext class
3. `tests/test_geometry.py` — add `viz` parameter + helper calls to 8 tests
4. `tests/test_kinematics.py` — add `viz` parameter + helper calls to 9 tests
5. `tests/test_s2_t1.py` — add `viz` parameter + helper calls to 5 tests
6. `tests/test_urdf_chain.py` — modify `robot` fixture + add `_pause_if_viz` to 13 tests

## Key Rules

- **Never break existing tests.** All viz calls go AFTER assertions. Adding `viz` as a fixture parameter must not change test behavior when `--viz` is not passed.
- **VizContext.show() is called by the fixture teardown**, not by the test. This means each test accumulates draw commands, and the plot appears after the test finishes.
- **No `plt.show()` inside test bodies.** Only VizContext.show() calls it.
- For **test_urdf_chain.py**, use `input()` to pause (not `plt.show()`), since PyBullet has its own GUI.
