"""Endpoint-preserving cubic paths and continuous curvature diagnostics.

No optimization package is required. Cubic curvature extrema are checked at
stationary polynomial roots, not merely at a sparse set of display vertices.
Arc-length inversion is numerical and explicitly resolution controlled.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.polynomial import Polynomial as Poly

from capacity_policy.geometry import Corridor


def _unit_roots(poly: Poly) -> list[float]:
    coefficients = poly.coef
    magnitude = np.max(np.abs(coefficients))
    if magnitude == 0:
        return []
    roots = Poly(coefficients / magnitude).trim(tol=1e-12).roots()
    return [float(r.real) for r in roots if abs(r.imag) < 1e-7 and 0 < r.real < 1]


@dataclass(frozen=True)
class SmoothPath:
    """Natural cubic spline through uniformly parameterized control points."""

    control_xy_m: np.ndarray
    crs: str
    arc_step_m: float = 10.0
    source: str = "endpoint-fixed natural cubic research path"

    def __post_init__(self):
        points = np.asarray(self.control_xy_m, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError("finite XY control points required")
        if not np.isfinite(self.arc_step_m) or self.arc_step_m <= 0:
            raise ValueError("arc_step_m must be positive")
        n = len(points)
        # Work relative to the first coordinate to avoid cancellation in UTM.
        relative = points - points[0]
        matrix = np.eye(n)
        rhs = np.zeros_like(relative)
        for i in range(1, n - 1):
            matrix[i, i - 1:i + 2] = [1, 4, 1]
            rhs[i] = 6 * (relative[i + 1] - 2 * relative[i] + relative[i - 1])
        second = np.linalg.solve(matrix, rhs)
        coefficients = np.stack([
            relative[:-1],
            np.diff(relative, axis=0) - (2 * second[:-1] + second[1:]) / 6,
            second[:-1] / 2,
            np.diff(second, axis=0) / 6,
        ], axis=1)
        object.__setattr__(self, "control_xy_m", points)
        object.__setattr__(self, "coefficients", coefficients)
        count = max(1001, int(np.ceil(np.linalg.norm(np.diff(points, axis=0), axis=1).sum() / self.arc_step_m)) + 1)
        parameter = np.linspace(0, n - 1, count)
        xy = self.at_parameter(parameter)
        cumulative = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        if np.any(np.diff(cumulative) <= 0):
            raise ValueError("degenerate path")
        object.__setattr__(self, "arc_parameter", parameter)
        object.__setattr__(self, "arc_distance_m", cumulative)
        object.__setattr__(self, "xy_m", xy)

    @property
    def length_m(self) -> float:
        return float(self.arc_distance_m[-1])

    def at_parameter(self, parameter) -> np.ndarray:
        t = np.clip(np.asarray(parameter, dtype=float), 0, len(self.control_xy_m) - 1)
        index = np.minimum(t.astype(int), len(self.control_xy_m) - 2)
        u = t - index
        c = self.coefficients[index]
        return self.control_xy_m[0] + c[..., 0, :] + u[..., None] * (
            c[..., 1, :] + u[..., None] * (c[..., 2, :] + u[..., None] * c[..., 3, :])
        )

    def interpolate(self, s_m, lateral_m=0.0) -> np.ndarray:
        if np.any(np.asarray(lateral_m) != 0):
            raise ValueError("SmoothPath embeds offsets; a second lateral offset is not supported")
        parameter = np.interp(np.asarray(s_m), self.arc_distance_m, self.arc_parameter)
        return self.at_parameter(parameter)

    def curvature_report(self) -> dict:
        max_curvature = 0.0
        regular = True
        for segment in self.coefficients:
            scale = max(float(np.abs(segment[1:]).max()), 1.0)
            dx = Poly(np.r_[0, segment[1:, 0] / scale]).deriv()
            dy = Poly(np.r_[0, segment[1:, 1] / scale]).deriv()
            speed2 = dx * dx + dy * dy
            cross = dx * dy.deriv() - dy * dx.deriv()
            speed_points = [0.0, 1.0, *_unit_roots(speed2.deriv())]
            if min(speed2(t) for t in speed_points) <= 1e-12:
                regular = False
                continue
            # d(kappa^2)/du=0 => 2*N'*V - 3*N*V'=0 (N=0 is a zero-curvature minimum).
            stationary = 2 * cross.deriv() * speed2 - 3 * cross * speed2.deriv()
            points = [0.0, 1.0, *_unit_roots(stationary), *np.linspace(0, 1, 33)]
            maximum = max(abs(cross(t)) / speed2(t)**1.5 / scale for t in points)
            max_curvature = max(max_curvature, float(maximum))
        return {
            "regular": regular,
            "max_curvature_per_m": max_curvature if regular else None,
            "min_radius_m": 1 / max_curvature if regular and max_curvature > 1e-14 else None,
            "method": "all cubic-segment stationary roots and endpoints, numerical floating-point check",
        }


def offset_path(corridor: Corridor, offsets_m, *, arc_step_m=10.0) -> SmoothPath:
    offsets = np.asarray(offsets_m, dtype=float)
    if offsets.ndim != 1 or len(offsets) < 4 or not np.isfinite(offsets).all():
        raise ValueError("at least four finite offset controls required")
    if offsets[0] != 0 or offsets[-1] != 0:
        raise ValueError("optimized paths must preserve both endpoints")
    s = np.linspace(0, corridor.length_m, len(offsets))
    centers = corridor.interpolate(s)
    tangent = np.gradient(centers, axis=0)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    lengths = np.linalg.norm(normal, axis=1)
    if np.any(lengths == 0):
        raise ValueError("undefined control-point normal")
    controls = centers + offsets[:, None] * normal / lengths[:, None]
    return SmoothPath(controls, corridor.crs, arc_step_m)


def path_constraints(
    path: SmoothPath, corridor: Corridor, *, minimum_radius_m=8000.0,
    maximum_deviation_m=2000.0, maximum_length_ratio=1.1,
) -> dict:
    """Check curvature continuously and a conservative correspondence envelope.

Deviation is distance to the reference point at equal normalized parameter,
not nearest-point distance; it is sampled at <=25m plus all source vertices.
This envelope check is a numerical screening, not a certified airspace bound.
"""
    if not np.isfinite([minimum_radius_m, maximum_deviation_m, maximum_length_ratio]).all() or min(minimum_radius_m, maximum_deviation_m) <= 0 or maximum_length_ratio < 1:
        raise ValueError("invalid path constraints")
    fraction = np.unique(np.r_[
        np.linspace(0, 1, max(1001, int(np.ceil(corridor.length_m / 25)) + 1)),
        corridor.cumulative_m / corridor.length_m,
    ])
    deviation = np.linalg.norm(
        path.at_parameter(fraction * (len(path.control_xy_m) - 1))
        - corridor.interpolate(fraction * corridor.length_m), axis=1,
    )
    report = path.curvature_report()
    kappa = report["max_curvature_per_m"]
    max_deviation = float(deviation.max())
    length_ratio = path.length_m / corridor.length_m
    endpoint_error = float(np.linalg.norm(
        path.at_parameter([0, len(path.control_xy_m) - 1]) - corridor.xy_m[[0, -1]], axis=1
    ).max())
    violations = [
        max(0.0, kappa * minimum_radius_m - 1) if report["regular"] else 1e6,
        max(0.0, max_deviation / maximum_deviation_m - 1),
        max(0.0, length_ratio / maximum_length_ratio - 1),
        max(0.0, endpoint_error / 1e-6 - 1),
    ]
    return {
        **report, "maximum_deviation_m": max_deviation, "length_ratio": length_ratio,
        "endpoint_error_m": endpoint_error,
        "violation": float(sum(violations)),
        "feasible": bool(report["regular"] and max(violations) <= 1e-9),
        "envelope_method": "sampled equal-reference-progress Euclidean deviation (<=25m plus source vertices)",
    }


def polyline_curvature_diagnostic(corridor: Corridor) -> dict:
    a = corridor.xy_m[1:-1] - corridor.xy_m[:-2]
    b = corridor.xy_m[2:] - corridor.xy_m[1:-1]
    cross = np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
    denominator = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(a + b, axis=1)
    kappa = np.divide(2 * cross, denominator, out=np.zeros_like(cross), where=denominator > 0)
    maximum = float(kappa.max(initial=0))
    return {
        "vertex_count": len(corridor.xy_m),
        "minimum_three_point_radius_m": 1 / maximum if maximum else None,
        "triples_below_8000m": int((kappa > 1 / 8000).sum()),
        "flyability_certified": False,
        "interpretation": "three-point diagnostic only; a polyline has discontinuous heading at noncollinear vertices",
    }


def search_offsets(corridor: Corridor, objective, *, control_count=6,
                   max_offset_m=2000.0, seed_offsets_m=(-1000.0, 0.0, 1000.0),
                   steps_m=(1000.0, 500.0, 250.0), passes_per_step=2,
                   max_evaluations=120, arc_step_m=10.0, **constraints) -> dict:
    """Deterministic feasibility-first coordinate search over bounded controls.

The best tested feasible path is returned, never a claimed global optimum.
Capacity is evaluated only for geometrically feasible candidates. Infeasible
search termination remains explicit and must not silently fall back to raw GIS.
"""
    if control_count < 4 or not np.isfinite(max_offset_m) or max_offset_m <= 0 or max_evaluations < 1 or passes_per_step < 1:
        raise ValueError("invalid search budget or geometry")
    if not steps_m or any(not np.isfinite(v) or v <= 0 for v in steps_m):
        raise ValueError("search steps must be finite and positive")
    cache = {}
    records = []
    best = None

    def evaluate(offsets):
        nonlocal best
        key = tuple(map(float, offsets))
        if key in cache:
            return cache[key]
        if len(records) >= max_evaluations:
            return None
        path = offset_path(corridor, offsets, arc_step_m=arc_step_m)
        report = path_constraints(path, corridor, **constraints)
        score = float(objective(path)) if report["feasible"] else None
        if score is not None and not np.isfinite(score):
            raise ValueError("objective must return a finite score")
        record = {"candidate_id": len(records), "offset_controls_m": list(key), **report, "objective_q_rho_uam_h": score}
        records.append(record)
        cache[key] = (path, record)
        if score is not None and (best is None or score > best[1]["objective_q_rho_uam_h"] + 1e-12):
            best = (path, record)
        return path, record

    def rank(item):
        record = item[1]
        return (1, record["objective_q_rho_uam_h"]) if record["feasible"] else (0, -record["violation"])

    starts = []
    for seed in seed_offsets_m:
        if not np.isfinite(seed) or abs(seed) > max_offset_m:
            raise ValueError("seed lies outside offset bound")
        offsets = np.full(control_count, seed, dtype=float)
        offsets[[0, -1]] = 0
        candidate = evaluate(offsets)
        if candidate is not None:
            starts.append(candidate)
    if not starts:
        raise ValueError("at least one seed required")
    current = max(starts, key=rank)
    for step in steps_m:
        for _ in range(passes_per_step):
            improved = False
            for index in range(1, control_count - 1):
                origin = np.asarray(current[1]["offset_controls_m"])
                options = [current]
                for direction in (-1, 1):
                    offsets = origin.copy()
                    offsets[index] = np.clip(offsets[index] + direction * step, -max_offset_m, max_offset_m)
                    candidate = evaluate(offsets)
                    if candidate is not None:
                        options.append(candidate)
                chosen = max(options, key=rank)
                if rank(chosen) > rank(current):
                    current, improved = chosen, True
            if not improved:
                break
    return {
        "status": "feasible_candidate_found" if best is not None else "no_feasible_candidate_found",
        "best_path": None if best is None else best[0],
        "best": None if best is None else best[1],
        "best_feasible_seed": max((s[1] for s in starts if s[1]["feasible"]),
                                  key=lambda r: r["objective_q_rho_uam_h"], default=None),
        "first_feasible_candidate": next((r for r in records if r["feasible"]), None),
        "records": records,
        "budget_exhausted": len(records) >= max_evaluations,
        "search_method": "deterministic feasibility-first bounded coordinate pattern search",
        "optimality": "best tested feasible candidate; no global optimality or infeasibility proof",
    }
