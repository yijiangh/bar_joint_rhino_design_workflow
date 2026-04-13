# Task: S2-T1 Return All Solutions + Rhino Pick UI

## Context

`solve_s2_t1` in `scripts/core/geometry.py` tries 4 sign combinations (±D for each contact normal) but returns only the best. The user needs to see all valid solutions and pick one visually in Rhino.

## Changes Required

### 1. `scripts/core/geometry.py` — Add `solve_s2_t1_all()`

Add a new function that returns all converged solutions. Keep the existing `solve_s2_t1()` as a thin wrapper that calls `solve_s2_t1_all()` and returns the best.

```python
def solve_s2_t1_all(
    n1, x_ce1, n2, x_ce2, distance,
    nn_init_hint=None, *, tol=1e-6, residual_threshold=1e-4,
) -> list[dict]:
    """Solve S2-T1 for all 4 sign combinations.

    Returns a list of dicts, one per converged solution (residual < residual_threshold):
        {
            "nn": np.ndarray,       # unit direction of Ln
            "p1": np.ndarray,       # point on Ln nearest to Le1
            "p2": np.ndarray,       # point on Ln nearest to Le2
            "residual": float,      # optimization residual
            "signs": (float, float) # the (s1, s2) sign pair used
        }
    Sorted by residual ascending (best first).
    """
```

**Implementation**: refactor the existing loop in `solve_s2_t1` (lines 299-361). Instead of tracking only `best_cost`/`best_state`, collect all results per sign pair into a list. For each sign pair, keep the best result across all initial directions. Then filter by `residual < residual_threshold` and sort.

After adding `solve_s2_t1_all`, update `solve_s2_t1` to delegate:

```python
def solve_s2_t1(n1, x_ce1, n2, x_ce2, distance, nn_init_hint=None, *, tol=1e-6):
    """Original API — returns the single best solution."""
    solutions = solve_s2_t1_all(n1, x_ce1, n2, x_ce2, distance, nn_init_hint, tol=tol)
    if not solutions:
        raise RuntimeError("Failed to solve the S2 bar-axis problem.")
    best = solutions[0]  # already sorted by residual
    return best["nn"], best["p1"], best["p2"]
```

### 2. `scripts/t1_bar_axis.py` — Update `run_s2_t1()` with preview-and-pick UI

Replace the current `run_s2_t1` function (lines 96-114). The new flow:

```python
from core.geometry import solve_s2_t1_all

# Colors for up to 4 solutions (R, G, B tuples)
_S2_PREVIEW_COLORS = [
    (230, 80, 80),    # red
    (80, 80, 230),    # blue
    (80, 200, 80),    # green
    (200, 160, 50),   # amber
]

def run_s2_t1(inputs):
    le1_start, le1_end = _curve_endpoints(inputs["le1_id"])
    le2_start, le2_end = _curve_endpoints(inputs["le2_id"])
    ce1 = np.array(inputs["ce1"], dtype=float)
    ce2 = np.array(inputs["ce2"], dtype=float)

    n1 = le1_end - le1_start
    n2 = le2_end - le2_start

    # --- Step 1: compute all valid solutions ---
    try:
        solutions = solve_s2_t1_all(
            n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE,
            nn_init_hint=ce2 - ce1,
        )
    except Exception as exc:
        rs.MessageBox(f"Error while solving S2-T1: {exc}")
        return None

    if not solutions:
        rs.MessageBox("No valid S2-T1 solution found for any sign combination.")
        return None

    # --- Step 2: if only one solution, use it directly ---
    if len(solutions) == 1:
        sol = solutions[0]
        midpoint = 0.5 * (sol["p1"] + sol["p2"])
        line_id = _add_centered_line(midpoint, sol["nn"])
        rs.SelectObject(line_id)
        print(f"S2-T1: Single solution, bar placed. Residual: {sol['residual']:.2e}")
        return line_id

    # --- Step 3: draw all solutions as preview lines with labels ---
    half_length = DEFAULT_NEW_BAR_LENGTH / 2.0
    preview_items = []  # list of (solution_index, line_id, dot_id)

    for i, sol in enumerate(solutions):
        midpoint = 0.5 * (sol["p1"] + sol["p2"])
        nn_unit = sol["nn"] / np.linalg.norm(sol["nn"])
        pt_a = midpoint - half_length * nn_unit
        pt_b = midpoint + half_length * nn_unit

        line_id = rs.AddLine(pt_a, pt_b)
        color = _S2_PREVIEW_COLORS[i % len(_S2_PREVIEW_COLORS)]
        rs.ObjectColor(line_id, color)

        label = f"{i + 1}"
        dot_id = rs.AddTextDot(label, midpoint)
        rs.ObjectColor(dot_id, color)

        preview_items.append((i, line_id, dot_id))

    rs.Redraw()

    # --- Step 4: user picks one line ---
    picked_id = rs.GetObject(
        f"Click one of the {len(solutions)} candidate bars (numbered 1-{len(solutions)}):",
        rs.filter.curve,
    )

    # --- Step 5: determine which solution was picked, clean up all previews ---
    chosen_index = None
    for i, line_id, dot_id in preview_items:
        if line_id == picked_id:
            chosen_index = i
        rs.DeleteObject(dot_id)
        rs.DeleteObject(line_id)

    if chosen_index is None:
        # User cancelled or clicked something else
        print("S2-T1: No solution selected.")
        return None

    # --- Step 6: add the final chosen line ---
    sol = solutions[chosen_index]
    midpoint = 0.5 * (sol["p1"] + sol["p2"])
    line_id = _add_centered_line(midpoint, sol["nn"])
    rs.SelectObject(line_id)
    signs = sol["signs"]
    print(
        f"S2-T1: Solution {chosen_index + 1} selected. "
        f"Signs=({signs[0]:+.0f},{signs[1]:+.0f}), "
        f"Residual: {sol['residual']:.2e}"
    )
    return line_id
```

### 3. Update `scripts/t1_bar_axis.py` imports

Change line 18:
```python
# Before:
from core.geometry import closest_params_infinite_lines, solve_s2_t1

# After:
from core.geometry import closest_params_infinite_lines, solve_s2_t1_all
```

(Keep `solve_s2_t1` import too if `run_s1_t1` or tests still use it.)

### 4. Update tests in `tests/test_s2_t1.py`

Add a test for `solve_s2_t1_all`:

```python
from core.geometry import solve_s2_t1_all

class TestS2T1AllSolutions:
    def test_returns_list(self):
        """solve_s2_t1_all should return a list of dicts."""
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([5., 0, 0])
        n2 = np.array([0., 1, 0])
        Ce2 = np.array([0., 8, 0])
        solutions = solve_s2_t1_all(n1, Ce1, n2, Ce2, D)
        assert isinstance(solutions, list)
        assert len(solutions) >= 1

    def test_each_solution_valid(self):
        """Every returned solution should satisfy the geometric constraints."""
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([5., 0, 0])
        n2 = np.array([0., 1, 0])
        Ce2 = np.array([0., 8, 0])
        solutions = solve_s2_t1_all(n1, Ce1, n2, Ce2, D)
        for sol in solutions:
            assert "nn" in sol and "p1" in sol and "p2" in sol
            assert "residual" in sol and "signs" in sol
            # nn should be unit vector
            assert abs(np.linalg.norm(sol["nn"]) - 1.0) < 1e-8
            # Each solution passes the shared geometric verifier
            self._verify_s2_result(sol["nn"], sol["p1"], sol["p2"], n1, Ce1, n2, Ce2, D)

    def test_sorted_by_residual(self):
        """Solutions should be sorted best-first."""
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([5., 0, 0])
        n2 = np.array([0., 1, 0])
        Ce2 = np.array([0., 8, 0])
        solutions = solve_s2_t1_all(n1, Ce1, n2, Ce2, D)
        residuals = [s["residual"] for s in solutions]
        assert residuals == sorted(residuals)

    def test_solutions_are_distinct(self):
        """Different sign pairs should produce geometrically different bars."""
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([5., 0, 0])
        n2 = np.array([0., 1, 0])
        Ce2 = np.array([0., 8, 0])
        solutions = solve_s2_t1_all(n1, Ce1, n2, Ce2, D)
        if len(solutions) >= 2:
            # At least two solutions should have different directions or positions
            mid0 = 0.5 * (solutions[0]["p1"] + solutions[0]["p2"])
            mid1 = 0.5 * (solutions[1]["p1"] + solutions[1]["p2"])
            dir_diff = 1.0 - abs(np.dot(solutions[0]["nn"], solutions[1]["nn"]))
            pos_diff = np.linalg.norm(mid0 - mid1)
            assert dir_diff > 1e-3 or pos_diff > 1e-3, \
                "Two solutions should be geometrically distinct"

    def _verify_s2_result(self, nn, P1, P2, n1, Ce1, n2, Ce2, D):
        """Check geometric constraints (reuse from existing test_s2_t1.py)."""
        from core.geometry import distance_infinite_lines, closest_params_infinite_lines
        TOL = 1e-3
        # Distance from Ln to Le1 should be D
        d1 = abs(distance_infinite_lines(P1, nn, Ce1, n1))
        assert abs(d1 - D) < TOL, f"Distance to Le1: {d1}, expected {D}"
        # Distance from Ln to Le2 should be D
        d2 = abs(distance_infinite_lines(P1, nn, Ce2, n2))
        assert abs(d2 - D) < TOL, f"Distance to Le2: {d2}, expected {D}"
```

## Files Modified

| File | Change |
|------|--------|
| `scripts/core/geometry.py` | Add `solve_s2_t1_all()`, refactor `solve_s2_t1()` to delegate |
| `scripts/t1_bar_axis.py` | Rewrite `run_s2_t1()` with preview-and-pick UI |
| `tests/test_s2_t1.py` | Add `TestS2T1AllSolutions` class |
