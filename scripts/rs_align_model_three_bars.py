#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""Align all managed-layer geometry to three baked mocap lines.

Workflow
--------
1. ``rs_read_mocap_bar`` has already baked one or more straight lines on the
   ``MoCap_Retrieval`` layer; each line passes through the centers of the
   two marker pairs of a real bar in the world.
2. The user picks three of those baked lines and pairs each with one model
   bar from the ``Bar Centerlines`` layer.
3. We solve a 6-DoF rigid transform ``(R, t)`` that, when applied to the
    three model bars, minimises the perpendicular distance from each line's
    two endpoints (i.e. the marker-pair centers) to the corresponding bar's
    infinite axis.  Initial guess comes from a closed-form direction fit,
    then nonlinear refinement runs in stages with early stop on tiny gains.
4. We report the per-pair and RMS fitting error and ask the user to
   confirm.  On confirm, the transform is applied to **every** object on
   the managed layers (bars, joints, tools, ground, previews).

Notes
-----
* Lines / bars have no inherent direction; the cost uses perpendicular
  distance to the infinite axis, so either orientation of either line
  produces the same residual.
* Markers are placed along the bar (not at the ends), so we treat the
  bar's centerline as infinite for the residual.  No end-cap penalty.
"""

import math
import os
import sys

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc
import numpy as np
from scipy.optimize import least_squares

# Make the core/ sibling importable when run from ScriptEditor.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from core.config import LAYER_BAR_CENTERLINES, MANAGED_LAYERS  # noqa: E402
from core.rhino_helpers import curve_endpoints  # noqa: E402


_MOCAP_LAYER = "MoCap_Retrieval"
_NUM_PAIRS = 3

# Solver tuning (validated via tests/_dev_align_three_bars_benchmark.py on the
# 2026-05-17 real-world capture).  `lm` with x0=identity got stuck when the
# model was dragged far from the lines.  `trf` + jac-scaled steps + augmented
# direction residual + closed-form initial guess converges in <200 nfev for
# both near and far starts to the same global minimum (RMSE ~1.5 mm).
_SOLVER_METHOD = "trf"
_SOLVER_X_SCALE = "jac"
_SOLVER_MAX_NFEV = 20000
# Solve in chunks and stop when a chunk does not improve perpendicular RMSE
# beyond this threshold (model units). This gives explicit, user-visible
# convergence criteria on top of SciPy's internal ftol/xtol/gtol checks.
_SOLVER_STAGE_MAX_NFEV = 500
_SOLVER_MIN_IMPROVEMENT_UNITS = 0.0001
# Direction residual weight (mm).  ``1 - |cos(angle)|`` is dimensionless;
# multiplying by a length puts it on the same scale as the perpendicular
# distance residuals so the Jacobian is well conditioned.  200 mm worked well
# for bars 0.6-2 m long; only adjust if much longer / shorter bars change the
# balance between rotation and translation.
_DIRECTION_WEIGHT_MM = 200.0


# ---------------------------------------------------------------------------
# Picking
# ---------------------------------------------------------------------------

def _make_layer_filter(layer_name):
    """Return a custom_filter for rs.GetObject that only accepts curves on
    the given layer (or any of its sub-layers)."""
    prefix = layer_name + "::"

    def _filter(rhino_object, geometry, component_index):
        try:
            obj_layer = sc.doc.Layers[rhino_object.Attributes.LayerIndex].FullPath
        except Exception:
            return False
        if obj_layer == layer_name or obj_layer.startswith(prefix):
            return isinstance(geometry, Rhino.Geometry.Curve)
        return False

    return _filter


def _pick_pairs():
    """Prompt the user for 3 (mocap line, model bar) pairs.

    Returns a list of (line_id, bar_id) or ``None`` on cancel.
    """
    pairs = []
    for i in range(_NUM_PAIRS):
        line_id = rs.GetObject(
            "Pick mocap line #{} (from '{}')".format(i + 1, _MOCAP_LAYER),
            filter=rs.filter.curve,
            preselect=False,
            select=False,
            custom_filter=_make_layer_filter(_MOCAP_LAYER),
        )
        if line_id is None:
            return None
        bar_id = rs.GetObject(
            "Pick model bar #{} (from '{}') to align onto that line".format(
                i + 1, LAYER_BAR_CENTERLINES
            ),
            filter=rs.filter.curve,
            preselect=False,
            select=False,
            custom_filter=_make_layer_filter(LAYER_BAR_CENTERLINES),
        )
        if bar_id is None:
            return None
        pairs.append((line_id, bar_id))
    return pairs


# ---------------------------------------------------------------------------
# Geometry math
# ---------------------------------------------------------------------------

def _rodrigues(rvec):
    """Axis-angle (3-vector) -> 3x3 rotation matrix."""
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3)
    k = np.asarray(rvec, dtype=float) / theta
    K = np.array([
        [0.0, -k[2], k[1]],
        [k[2], 0.0, -k[0]],
        [-k[1], k[0], 0.0],
    ])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def _point_to_line_distance(p, a, b):
    """Perpendicular distance from point *p* to the infinite line through
    *a*, *b*.  All args are 3-vectors (numpy)."""
    d = b - a
    n = float(np.linalg.norm(d))
    if n < 1e-12:
        return float(np.linalg.norm(p - a))
    d = d / n
    v = p - a
    proj = float(np.dot(v, d))
    perp = v - proj * d
    return float(np.linalg.norm(perp))


def _line_unit_direction(a, b):
    d = b - a
    n = float(np.linalg.norm(d))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0], dtype=float), 0.0
    return d / n, n


def _residual_metrics(residuals):
    res = np.asarray(residuals, dtype=float)
    n = int(res.size)
    if n == 0:
        return {
            "n": 0,
            "sse": 0.0,
            "mse": 0.0,
            "rmse": 0.0,
            "max_abs": 0.0,
        }
    sq = res ** 2
    sse = float(np.sum(sq))
    mse = float(sse / n)
    rmse = float(math.sqrt(mse))
    max_abs = float(np.max(np.abs(res)))
    return {
        "n": n,
        "sse": sse,
        "mse": mse,
        "rmse": rmse,
        "max_abs": max_abs,
    }


def _build_residuals(pair_data):
    """Perpendicular-distance only residual (2 per pair, 6 total).

    Used for both the original objective and for *scoring* the solved
    parameters in reporting / metrics regardless of which objective the
    solver itself optimised.
    """
    def _residuals(params):
        rvec = params[:3]
        tvec = params[3:6]
        R = _rodrigues(rvec)
        out = []
        for pd in pair_data:
            a, b = pd["bar_endpoints"]
            a_t = R @ a + tvec
            b_t = R @ b + tvec
            for p in pd["line_endpoints"]:
                out.append(_point_to_line_distance(p, a_t, b_t))
        return np.asarray(out, dtype=float)

    return _residuals


def _build_residuals_augmented(pair_data, dir_weight_mm=_DIRECTION_WEIGHT_MM):
    """Perpendicular distance + direction-alignment residual (3 per pair).

    Adds ``dir_weight_mm * (1 - |cos(theta)|)`` per pair, where ``theta`` is
    the angle between the line direction and the *transformed* bar
    direction.  The absolute value handles the bar/line sign ambiguity.
    Breaks rotational degeneracy and prevents the solver from stalling when
    the initial pose is far from the answer.
    """
    def _residuals(params):
        rvec = params[:3]
        tvec = params[3:6]
        R = _rodrigues(rvec)
        out = []
        for pd in pair_data:
            a, b = pd["bar_endpoints"]
            a_t = R @ a + tvec
            b_t = R @ b + tvec
            bar_dir = b_t - a_t
            bar_n = float(np.linalg.norm(bar_dir))
            if bar_n > 1e-12:
                bar_dir = bar_dir / bar_n
            line_dir = pd["line_endpoints"][1] - pd["line_endpoints"][0]
            line_n = float(np.linalg.norm(line_dir))
            if line_n > 1e-12:
                line_dir = line_dir / line_n
            for p in pd["line_endpoints"]:
                out.append(_point_to_line_distance(p, a_t, b_t))
            cos_abs = abs(float(np.dot(bar_dir, line_dir)))
            cos_abs = max(-1.0, min(1.0, cos_abs))
            out.append(dir_weight_mm * (1.0 - cos_abs))
        return np.asarray(out, dtype=float)

    return _residuals


def _rotation_matrix_to_rvec(R):
    """3x3 rotation matrix -> axis-angle 3-vector."""
    cos_t = max(-1.0, min(1.0, 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)))
    theta = math.acos(cos_t)
    if theta < 1e-9:
        return np.zeros(3)
    if math.pi - theta < 1e-6:
        eigvals, eigvecs = np.linalg.eigh(R + np.eye(3))
        axis = eigvecs[:, -1]
        return axis * theta
    sin_t = math.sin(theta)
    rx = (R[2, 1] - R[1, 2]) / (2.0 * sin_t)
    ry = (R[0, 2] - R[2, 0]) / (2.0 * sin_t)
    rz = (R[1, 0] - R[0, 1]) / (2.0 * sin_t)
    return np.array([rx, ry, rz]) * theta


def _closed_form_initial_guess(pair_data):
    """Wahba-on-directions + centroid translation initial guess.

    Not a great solution on its own (when bars and lines have different
    lengths the centroid mapping is biased), but it puts the nonlinear
    refinement in the correct rotational basin so convergence is fast even
    when the model has been dragged far from the mocap lines.
    """
    M = np.zeros((3, 3))
    for pd in pair_data:
        bd, _ = _line_unit_direction(pd["bar_endpoints"][0],
                                     pd["bar_endpoints"][1])
        ld, _ = _line_unit_direction(pd["line_endpoints"][0],
                                     pd["line_endpoints"][1])
        if np.dot(bd, ld) < 0:
            ld = -ld
        M += np.outer(ld, bd)
    U, _S, Vt = np.linalg.svd(M)
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1.0
    R = U @ D @ Vt

    line_mids = np.array([
        0.5 * (pd["line_endpoints"][0] + pd["line_endpoints"][1])
        for pd in pair_data
    ])
    bar_mids = np.array([
        0.5 * (pd["bar_endpoints"][0] + pd["bar_endpoints"][1])
        for pd in pair_data
    ])
    t = line_mids.mean(axis=0) - (R @ bar_mids.T).T.mean(axis=0)
    return np.hstack([_rotation_matrix_to_rvec(R), t])


def _solve_transform(pair_data):
    """Return solve details for downstream reporting."""
    f_full = _build_residuals_augmented(pair_data)
    f_perp = _build_residuals(pair_data)
    x0 = _closed_form_initial_guess(pair_data)

    x_curr = np.asarray(x0, dtype=float)
    perp_curr = f_perp(x_curr)
    rmse_curr = _residual_metrics(perp_curr)["rmse"]
    nfev_total = 0
    stage_count = 0
    last_improvement = None
    stop_reason = "max_nfev_budget"
    result = None

    while nfev_total < _SOLVER_MAX_NFEV:
        stage_count += 1
        stage_budget = min(_SOLVER_STAGE_MAX_NFEV, _SOLVER_MAX_NFEV - nfev_total)
        result = least_squares(
            f_full,
            x_curr,
            method=_SOLVER_METHOD,
            x_scale=_SOLVER_X_SCALE,
            max_nfev=stage_budget,
        )

        stage_nfev = int(getattr(result, "nfev", stage_budget) or stage_budget)
        nfev_total += stage_nfev
        x_next = np.asarray(result.x, dtype=float)
        perp_next = f_perp(x_next)
        rmse_next = _residual_metrics(perp_next)["rmse"]
        improvement = float(rmse_curr - rmse_next)
        last_improvement = improvement

        x_curr = x_next
        rmse_curr = rmse_next

        if improvement <= _SOLVER_MIN_IMPROVEMENT_UNITS:
            stop_reason = "improvement_threshold"
            break

        # Respect SciPy's own convergence/termination if it reports success.
        if bool(getattr(result, "success", False)):
            stop_reason = "solver_converged"
            break

    R = _rodrigues(x_curr[:3])
    rvec = x_curr[:3]
    t = x_curr[3:6]
    # Score with the perpendicular-distance objective so reported metrics are
    # unaffected by the augmented direction term.
    res = f_perp(x_curr)
    per_pair_rms = []
    for i in range(len(pair_data)):
        chunk = res[2 * i:2 * i + 2]
        per_pair_rms.append(float(math.sqrt(float(np.mean(chunk ** 2)))))

    return {
        "R": R,
        "rvec": rvec,
        "t": t,
        "residuals": res,
        "per_pair_rms": per_pair_rms,
        "result": result,
        "x0": x0,
        "stage_count": stage_count,
        "nfev_total": nfev_total,
        "stop_reason": stop_reason,
        "improvement_threshold": _SOLVER_MIN_IMPROVEMENT_UNITS,
        "last_improvement": last_improvement,
    }


# ---------------------------------------------------------------------------
# Apply transform to managed layers
# ---------------------------------------------------------------------------

def _numpy_to_rhino_transform(R, t):
    xform = Rhino.Geometry.Transform(1.0)
    for r in range(3):
        for c in range(3):
            xform[r, c] = float(R[r, c])
        xform[r, 3] = float(t[r])
    xform[3, 0] = 0.0
    xform[3, 1] = 0.0
    xform[3, 2] = 0.0
    xform[3, 3] = 1.0
    return xform


def _all_managed_layer_objects():
    """Collect object ids on every managed layer (including sub-layers)."""
    seen = set()
    ids = []
    for layer in MANAGED_LAYERS:
        # rs.ObjectsByLayer returns objects on exactly that full layer path.
        for oid in (rs.ObjectsByLayer(layer) or []):
            key = str(oid)
            if key in seen:
                continue
            seen.add(key)
            ids.append(oid)
        # Also include any sub-layers.
        prefix = layer + "::"
        all_layers = rs.LayerNames() or []
        for full in all_layers:
            if full.startswith(prefix):
                for oid in (rs.ObjectsByLayer(full) or []):
                    key = str(oid)
                    if key in seen:
                        continue
                    seen.add(key)
                    ids.append(oid)
    return ids


def _apply_transform(xform):
    ids = _all_managed_layer_objects()
    moved = 0
    for oid in ids:
        try:
            new_id = rs.TransformObject(oid, xform, copy=False)
            if new_id is not None:
                moved += 1
        except Exception as exc:
            print("[align] failed to transform {}: {}".format(oid, exc))
    return moved, len(ids)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _format_pair_report(pair_data, residuals, units, per_pair_rms=None):
    lines = []
    for i, pd in enumerate(pair_data):
        d0, d1 = residuals[2 * i], residuals[2 * i + 1]
        if per_pair_rms is None:
            rms = float(math.sqrt((float(d0) ** 2 + float(d1) ** 2) * 0.5))
        else:
            rms = per_pair_rms[i]
        lines.append(
            "  pair {}: line='{}' bar='{}'  endpoint dists = "
            "{:.4f}, {:.4f} {}  (RMS {:.4f})".format(
                i + 1, pd["line_name"], pd["bar_name"], d0, d1, units, rms,
            )
        )
    return "\n".join(lines)


def _safe_int_or_na(value):
    if value is None:
        return "n/a"
    try:
        return str(int(value))
    except (TypeError, ValueError):
        return str(value)


def _curve_name_or_id(curve_id):
    name = rs.ObjectName(curve_id)
    return name if name else str(curve_id)


def main():
    pairs = _pick_pairs()
    if pairs is None:
        print("[align] Cancelled.")
        return

    pair_data = []
    for line_id, bar_id in pairs:
        l_start, l_end = curve_endpoints(line_id)
        b_start, b_end = curve_endpoints(bar_id)
        pair_data.append({
            "line_id": line_id,
            "bar_id": bar_id,
            "line_name": _curve_name_or_id(line_id),
            "bar_name": _curve_name_or_id(bar_id),
            "line_endpoints": np.vstack([l_start, l_end]),
            "bar_endpoints": np.vstack([b_start, b_end]),
        })

    # Initial (identity) residuals = current-pose error.
    f = _build_residuals(pair_data)
    res0 = f(np.zeros(6))
    metrics0 = _residual_metrics(res0)

    solve = _solve_transform(pair_data)
    R = solve["R"]
    rvec = solve["rvec"]
    t = solve["t"]
    res = solve["residuals"]
    per_pair_rms = solve["per_pair_rms"]
    result = solve["result"]
    stage_count = solve["stage_count"]
    nfev_total = solve["nfev_total"]
    stop_reason = solve["stop_reason"]
    improvement_threshold = solve["improvement_threshold"]
    last_improvement = solve["last_improvement"]
    metrics = _residual_metrics(res)
    units = str(sc.doc.ModelUnitSystem).split(".")[-1].lower()

    if last_improvement is None:
        last_improvement_txt = "n/a"
    else:
        last_improvement_txt = "{:.9f}".format(last_improvement)

    print(
        "[align] fit rmse: {:.4f} -> {:.4f} {}".format(
            metrics0["rmse"], metrics["rmse"], units
        )
    )
    print(
        "[align] staged stop: reason='{}' stages={} total_nfev={} threshold={:.9f} {} last_improvement={} {}".format(
            stop_reason,
            stage_count,
            nfev_total,
            float(improvement_threshold),
            units,
            last_improvement_txt,
            units,
        )
    )
    print(
        "[align] solver: success={} status={} nfev={} njev={} message='{}'".format(
            bool(result.success),
            int(result.status),
            _safe_int_or_na(getattr(result, "nfev", None)),
            _safe_int_or_na(getattr(result, "njev", None)),
            str(result.message).strip(),
        )
    )

    report = (
        "Three-bar alignment solved.\n"
        "  initial RMS: {:.4f} {}\n"
        "  solved RMS:  {:.4f} {}\n"
        "  solved SSE:  {:.4f} {}^2\n"
        "  solved MSE:  {:.4f} {}^2\n"
        "  solved max endpoint error: {:.4f} {}\n\n"
        "  staged stop reason: {}\n"
        "  staged iterations: {}\n"
        "  total nfev used: {} / {}\n"
        "  min improvement threshold: {:.9f} {}\n"
        "  last iteration improvement: {} {}\n\n"
        "{}\n\nApply this transform to all managed-layer objects?"
    ).format(
        metrics0["rmse"], units,
        metrics["rmse"], units,
        metrics["sse"], units,
        metrics["mse"], units,
        metrics["max_abs"], units,
        stop_reason,
        stage_count,
        nfev_total,
        _SOLVER_MAX_NFEV,
        float(improvement_threshold),
        units,
        last_improvement_txt,
        units,
        _format_pair_report(pair_data, res, units, per_pair_rms=per_pair_rms),
    )

    answer = rs.MessageBox(
        report, 4 | 32, "RSAlignModelThreeBars"
    )  # 4=YesNo, 32=question icon
    if answer != 6:  # 6 == Yes
        print("[align] Aborted by user.")
        return

    xform = _numpy_to_rhino_transform(R, t)
    moved, total = _apply_transform(xform)
    sc.doc.Views.Redraw()
    print("[align] Moved {} / {} managed-layer objects.".format(moved, total))


if __name__ == "__main__":
    main()
