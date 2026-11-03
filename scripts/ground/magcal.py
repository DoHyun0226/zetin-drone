#!/usr/bin/env python3
"""Fit BMM350 hard/soft-iron calibration from a telemetry CSV capture.

The firmware exposes uncalibrated ``Mag_X/Y/Z`` while
``Mag_Cal_Active == 1``. This tool selects that interval, fits a constrained
Li & Griffiths ellipsoid, removes radial outliers with three MAD clipping
passes, validates directional coverage, and prints constants ready for the
flight sketch.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


MIN_SAMPLES = 30
CLIP_PASSES = 3
MAD_SIGMA_LIMIT = 3.0
MIN_INLIER_FRACTION = 0.70
COVERAGE_WARN_RATIO = 10.0
COVERAGE_REJECT_RATIO = 100.0
MAX_CORRECTED_SPREAD_PCT = 5.0
EXISTING_MAG_COMP = np.array([0.007497, -0.001218, -0.000640])


class CalibrationError(ValueError):
    """Base class for calibration input or result failures."""


class CoverageError(CalibrationError):
    """Raised when sample directions cannot constrain a 3-D ellipsoid."""


class FitError(CalibrationError):
    """Raised when the data cannot produce a physical ellipsoid."""


@dataclass(frozen=True)
class CalibrationResult:
    """Validated hard/soft-iron calibration in raw BMM350 µT units."""

    hard_iron: np.ndarray
    soft_iron: np.ndarray
    radii: np.ndarray
    inlier_mask: np.ndarray
    coverage_eigenvalues: np.ndarray
    coverage_ratio: float
    raw_spread_pct: float
    corrected_spread_pct: float
    radial_residual_95_ut: float
    target_radius_ut: float

    @property
    def sample_count(self) -> int:
        return int(self.inlier_mask.size)

    @property
    def inlier_count(self) -> int:
        return int(np.count_nonzero(self.inlier_mask))

    @property
    def axis_ratio(self) -> float:
        return float(np.max(self.radii) / np.min(self.radii))


@dataclass(frozen=True)
class _UnitEllipsoid:
    center: np.ndarray
    whitening: np.ndarray
    radii: np.ndarray


def _as_samples(samples: Iterable[Iterable[float]]) -> np.ndarray:
    values = np.asarray(samples, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3:
        raise CalibrationError("magnetometer samples must have shape (N, 3)")
    if values.shape[0] < MIN_SAMPLES:
        raise CalibrationError(
            f"need at least {MIN_SAMPLES} finite samples; got {values.shape[0]}"
        )
    if not np.all(np.isfinite(values)):
        raise CalibrationError("magnetometer samples must all be finite")
    return values


def _unit_directions(values: np.ndarray) -> np.ndarray:
    centered = values - np.mean(values, axis=0)
    lengths = np.linalg.norm(centered, axis=1)
    positive = lengths > np.finfo(float).eps
    if np.count_nonzero(positive) < MIN_SAMPLES:
        raise CoverageError("too few distinct directions for calibration")
    return centered[positive] / lengths[positive, None]


def _coverage(values: np.ndarray) -> tuple[np.ndarray, float]:
    directions = _unit_directions(values)
    covariance = np.cov(directions, rowvar=False, bias=True)
    eigenvalues, _ = np.linalg.eigh(0.5 * (covariance + covariance.T))
    largest = float(eigenvalues[-1])
    smallest = float(eigenvalues[0])
    tolerance = max(np.finfo(float).eps, largest * 1e-12)
    if smallest <= tolerance:
        raise CoverageError("direction coverage is rank-deficient")
    ratio = largest / smallest
    if ratio >= COVERAGE_REJECT_RATIO:
        raise CoverageError(
            "direction coverage is degenerate: "
            f"covariance eigenvalue ratio {ratio:.1f} >= "
            f"{COVERAGE_REJECT_RATIO:.1f}"
        )
    return eigenvalues, ratio


def _design_matrix(values: np.ndarray) -> np.ndarray:
    x, y, z = values.T
    return np.column_stack(
        (
            x * x,
            y * y,
            z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
            np.ones(values.shape[0]),
        )
    )


def _constraint_matrix() -> np.ndarray:
    # beta[:6] = A, B, C, D, E, F for
    # Ax²+By²+Cz²+2Dxy+2Exz+2Fyz. This is 4J-I²=1.
    return np.array(
        (
            (-1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
            (1.0, -1.0, 1.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, -1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, -4.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, -4.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, -4.0),
        )
    )


def _positive_generalized_eigenvector(
    scatter: np.ndarray, constraint: np.ndarray
) -> np.ndarray:
    """Solve scatter*v=lambda*constraint*v with symmetric ``eigh``.

    The constraint has one positive and five negative directions, so it cannot
    be passed as the positive-definite metric required by common generalized
    ``eigh`` APIs. Whitening the positive scatter matrix instead gives the
    symmetric problem ``L^-1 C L^-T y = (1/lambda)y``.
    """

    scatter = 0.5 * (scatter + scatter.T)
    try:
        lower = np.linalg.cholesky(scatter)
    except np.linalg.LinAlgError as error:
        raise FitError("quadratic scatter matrix is singular") from error

    left = np.linalg.solve(lower, constraint)
    whitened_constraint = np.linalg.solve(lower, left.T).T
    whitened_constraint = 0.5 * (
        whitened_constraint + whitened_constraint.T
    )
    eigenvalues, eigenvectors = np.linalg.eigh(whitened_constraint)
    tolerance = max(1.0, float(np.max(np.abs(eigenvalues)))) * 1e-12
    candidates = np.flatnonzero(eigenvalues > tolerance)
    if candidates.size != 1:
        raise FitError(
            "Li & Griffiths constraint did not yield one ellipsoid candidate"
        )
    vector = np.linalg.solve(lower.T, eigenvectors[:, candidates[0]])
    norm = float(vector @ constraint @ vector)
    if not math.isfinite(norm) or norm <= tolerance:
        raise FitError("ellipsoid candidate violates 4J-I²=1")
    return vector / math.sqrt(norm)


def _fit_unit_ellipsoid(samples: np.ndarray) -> _UnitEllipsoid:
    origin = np.mean(samples, axis=0)
    centered = samples - origin
    scale = float(np.sqrt(np.mean(np.sum(centered * centered, axis=1))))
    if not math.isfinite(scale) or scale <= np.finfo(float).eps:
        raise FitError("sample RMS radius is zero")
    normalized = centered / scale

    design = _design_matrix(normalized)
    scatter = (design.T @ design) / normalized.shape[0]
    s11 = scatter[:6, :6]
    s12 = scatter[:6, 6:]
    s22 = scatter[6:, 6:]
    try:
        eliminated = np.linalg.solve(s22, s12.T)
    except np.linalg.LinAlgError as error:
        raise FitError("linear ellipsoid terms are singular") from error
    reduced = s11 - s12 @ eliminated
    quadratic = _positive_generalized_eigenvector(
        reduced, _constraint_matrix()
    )
    linear_constant = -eliminated @ quadratic
    beta = np.concatenate((quadratic, linear_constant))

    matrix = np.array(
        (
            (beta[0], beta[3], beta[4]),
            (beta[3], beta[1], beta[5]),
            (beta[4], beta[5], beta[2]),
        )
    )
    linear = beta[6:9]
    constant = float(beta[9])
    if np.trace(matrix) < 0.0:
        matrix = -matrix
        linear = -linear
        constant = -constant

    matrix_eigenvalues, _ = np.linalg.eigh(matrix)
    matrix_tolerance = max(
        1.0, float(np.max(np.abs(matrix_eigenvalues)))
    ) * 1e-12
    if matrix_eigenvalues[0] <= matrix_tolerance:
        raise FitError("constrained quadratic is not a positive ellipsoid")
    try:
        center_normalized = -np.linalg.solve(matrix, linear)
    except np.linalg.LinAlgError as error:
        raise FitError("ellipsoid center is singular") from error
    k = float(center_normalized @ matrix @ center_normalized - constant)
    if not math.isfinite(k) or k <= matrix_tolerance:
        raise FitError("ellipsoid has a non-positive radius scale")

    shape = 0.5 * (matrix / k + (matrix / k).T)
    shape_eigenvalues, eigenvectors = np.linalg.eigh(shape)
    shape_tolerance = max(
        1.0, float(np.max(np.abs(shape_eigenvalues)))
    ) * 1e-12
    if shape_eigenvalues[0] <= shape_tolerance:
        raise FitError("ellipsoid whitening matrix is not positive definite")
    whitening_normalized = (
        eigenvectors
        @ np.diag(np.sqrt(shape_eigenvalues))
        @ eigenvectors.T
    )
    whitening = whitening_normalized / scale
    radii = scale / np.sqrt(shape_eigenvalues)
    center = origin + scale * center_normalized
    if not (
        np.all(np.isfinite(center))
        and np.all(np.isfinite(whitening))
        and np.all(np.isfinite(radii))
    ):
        raise FitError("ellipsoid fit produced non-finite calibration values")
    return _UnitEllipsoid(center=center, whitening=whitening, radii=radii)


def _unit_radii(samples: np.ndarray, fit: _UnitEllipsoid) -> np.ndarray:
    return np.linalg.norm(
        (fit.whitening @ (samples - fit.center).T).T, axis=1
    )


def _spread_pct(lengths: np.ndarray) -> float:
    mean = float(np.mean(lengths))
    if not math.isfinite(mean) or mean <= np.finfo(float).eps:
        raise FitError("field magnitude mean is zero")
    return float(np.std(lengths) / mean * 100.0)


def fit_calibration(samples: Iterable[Iterable[float]]) -> CalibrationResult:
    """Fit and validate hard/soft-iron calibration for raw XYZ samples."""

    values = _as_samples(samples)
    _coverage(values)
    inlier_mask = np.ones(values.shape[0], dtype=bool)
    minimum_inliers = max(
        MIN_SAMPLES, int(math.ceil(values.shape[0] * MIN_INLIER_FRACTION))
    )

    for _ in range(CLIP_PASSES):
        fit = _fit_unit_ellipsoid(values[inlier_mask])
        radii = _unit_radii(values[inlier_mask], fit)
        residual = radii - np.median(radii)
        mad = float(np.median(np.abs(residual)))
        if not math.isfinite(mad):
            raise FitError("MAD residual is not finite")
        sigma = 1.4826 * mad
        if sigma <= np.finfo(float).eps:
            continue
        keep_local = np.abs(residual) <= MAD_SIGMA_LIMIT * sigma
        current_indices = np.flatnonzero(inlier_mask)
        next_mask = np.zeros_like(inlier_mask)
        next_mask[current_indices[keep_local]] = True
        if np.count_nonzero(next_mask) < minimum_inliers:
            raise FitError(
                "MAD clipping rejected too many samples: "
                f"{np.count_nonzero(next_mask)}/{values.shape[0]} remain"
            )
        inlier_mask = next_mask

    fit = _fit_unit_ellipsoid(values[inlier_mask])
    target_radius = float(np.mean(fit.radii))
    soft_iron = target_radius * fit.whitening
    corrected = (soft_iron @ (values[inlier_mask] - fit.center).T).T
    corrected_norms = np.linalg.norm(corrected, axis=1)
    raw_norms = np.linalg.norm(values[inlier_mask], axis=1)
    corrected_spread = _spread_pct(corrected_norms)
    if corrected_spread > MAX_CORRECTED_SPREAD_PCT:
        raise FitError(
            "samples do not fit one ellipsoid: corrected |B| spread "
            f"{corrected_spread:.2f}% > {MAX_CORRECTED_SPREAD_PCT:.2f}%"
        )

    coverage_eigenvalues, coverage_ratio = _coverage(corrected)
    radial_residual_95_ut = float(
        np.percentile(np.abs(corrected_norms - target_radius), 95.0)
    )
    return CalibrationResult(
        hard_iron=fit.center,
        soft_iron=soft_iron,
        radii=fit.radii,
        inlier_mask=inlier_mask,
        coverage_eigenvalues=coverage_eigenvalues,
        coverage_ratio=coverage_ratio,
        raw_spread_pct=_spread_pct(raw_norms),
        corrected_spread_pct=corrected_spread,
        radial_residual_95_ut=radial_residual_95_ut,
        target_radius_ut=target_radius,
    )


def apply_calibration(
    samples: Iterable[Iterable[float]], result: CalibrationResult
) -> np.ndarray:
    """Apply ``soft_iron @ (raw - hard_iron)`` to one or more samples."""

    values = np.asarray(samples, dtype=float)
    if values.ndim == 1:
        if values.shape != (3,):
            raise CalibrationError("one magnetometer sample must have shape (3,)")
        return result.soft_iron @ (values - result.hard_iron)
    if values.ndim != 2 or values.shape[1] != 3:
        raise CalibrationError("magnetometer samples must have shape (N, 3)")
    return (result.soft_iron @ (values - result.hard_iron).T).T


def load_calibration_csv(path: str | Path, use_all: bool = False) -> np.ndarray:
    """Load finite raw XYZ rows, selecting ``Mag_Cal_Active == 1`` by default."""

    csv_path = Path(path)
    try:
        csv_file = csv_path.open(newline="", encoding="utf-8-sig")
    except OSError as error:
        raise CalibrationError(f"cannot open {csv_path}: {error}") from error
    with csv_file:
        reader = csv.DictReader(csv_file)
        fields = set(reader.fieldnames or ())
        required = {"Mag_X", "Mag_Y", "Mag_Z"}
        missing = sorted(required - fields)
        if missing:
            raise CalibrationError(
                f"CSV is missing required columns: {', '.join(missing)}"
            )
        if not use_all and "Mag_Cal_Active" not in fields:
            raise CalibrationError(
                "CSV has no Mag_Cal_Active column; use --all only for an "
                "independently isolated raw capture"
            )

        samples = []
        for row_number, row in enumerate(reader, start=2):
            if not use_all:
                active_raw = (row.get("Mag_Cal_Active") or "").strip()
                if active_raw == "":
                    continue
                try:
                    active = float(active_raw)
                except ValueError as error:
                    raise CalibrationError(
                        f"row {row_number}: Mag_Cal_Active is not numeric"
                    ) from error
                if not math.isfinite(active) or active != 1.0:
                    continue
            try:
                sample = np.array(
                    [float(row[name]) for name in ("Mag_X", "Mag_Y", "Mag_Z")]
                )
            except (TypeError, ValueError):
                continue
            if not np.all(np.isfinite(sample)) or np.all(sample == 0.0):
                continue
            samples.append(sample)
    if not samples:
        selector = "all rows" if use_all else "Mag_Cal_Active == 1"
        raise CalibrationError(f"no finite nonzero magnetic samples in {selector}")
    return np.asarray(samples)


def format_firmware_constants(result: CalibrationResult) -> str:
    """Return paste-ready flight-sketch hard/soft-iron constant definitions."""

    hard = result.hard_iron
    soft = result.soft_iron
    rows = ",\n".join(
        "  {" + ", ".join(f"{value:.9f}f" for value in row) + "}"
        for row in soft
    )
    return (
        "const float MAG_HARD_IRON[3] = {"
        + ", ".join(f"{value:.9f}f" for value in hard)
        + "};\n"
        + "const float MAG_SOFT_IRON[3][3] = {\n"
        + rows
        + "\n};"
    )


def recalibrate_mag_comp(
    soft_iron: Iterable[Iterable[float]],
    coefficients: Iterable[float] = EXISTING_MAG_COMP,
) -> np.ndarray:
    """Transform post-calibration throttle coefficients as ``W @ old``."""

    matrix = np.asarray(soft_iron, dtype=float)
    vector = np.asarray(coefficients, dtype=float)
    if matrix.shape != (3, 3) or vector.shape != (3,):
        raise CalibrationError("mag_comp conversion needs a 3x3 matrix and 3-vector")
    transformed = matrix @ vector
    if not np.all(np.isfinite(transformed)):
        raise CalibrationError("mag_comp conversion produced non-finite values")
    return transformed


def format_report(result: CalibrationResult) -> str:
    """Return a concise calibration quality report."""

    radial_residual_95_pct = (
        100.0 * result.radial_residual_95_ut / result.target_radius_ut
    )
    warning = ""
    if result.coverage_ratio > COVERAGE_WARN_RATIO:
        warning = (
            "\nWARNING: directional coverage is uneven; recapture with more "
            "roll/pitch rotation before trusting these constants."
        )
    return (
        f"samples: selected={result.sample_count} inliers={result.inlier_count} "
        f"rejected={result.sample_count - result.inlier_count}\n"
        "coverage covariance eigenvalues: "
        + " ".join(f"{value:.6f}" for value in result.coverage_eigenvalues)
        + f"  ratio={result.coverage_ratio:.2f}\n"
        + "fitted radii (uT): "
        + " ".join(f"{value:.4f}" for value in result.radii)
        + f"  axis_ratio={result.axis_ratio:.4f}\n"
        + f"|B| spread: raw={result.raw_spread_pct:.3f}% "
        + f"corrected={result.corrected_spread_pct:.3f}%\n"
        + "95p |B| radial residual: "
        + f"{result.radial_residual_95_ut:.4f} uT "
        + f"({radial_residual_95_pct:.3f}%)"
        + warning
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit Li & Griffiths constrained BMM350 hard/soft-iron "
            "calibration from a flight-log CSV"
        )
    )
    parser.add_argument("csv", type=Path, help="flight-log CSV to fit")
    parser.add_argument(
        "--all",
        action="store_true",
        help="fit all finite Mag_X/Y/Z rows instead of Mag_Cal_Active == 1",
    )
    parser.add_argument(
        "--mag-comp",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=EXISTING_MAG_COMP,
        help="existing post-calibration mag_comp coefficients in uT/us",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        samples = load_calibration_csv(args.csv, use_all=args.all)
        result = fit_calibration(samples)
        transformed_comp = recalibrate_mag_comp(
            result.soft_iron, args.mag_comp
        )
    except CalibrationError as error:
        print(f"magcal_fit: error: {error}", file=sys.stderr)
        return 2

    print(format_report(result))
    print("\nFirmware calibration constants:\n")
    print(format_firmware_constants(result))
    print("\nTransformed post-calibration mag_comp (W @ existing):")
    print(
        "mag_comp_x={:.9f}f mag_comp_y={:.9f}f mag_comp_z={:.9f}f".format(
            *transformed_comp
        )
    )
    print(
        "Revalidate mag_comp with a motor/throttle capture after baking the "
        "new hard/soft-iron constants."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
