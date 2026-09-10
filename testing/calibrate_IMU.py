"""Orientation-free static 6-DOF IMU calibration.

Each input CSV is one *stationary* orientation. Input acceleration values must
be in **milli-g (mg)**, where stationary gravity has magnitude 1000 mg, and
input gyro values must be in **milli-degrees per second (mdps)**. Acceleration
is fitted to the gravity sphere with a full affine correction:

    a_calibrated = A_accel @ (a_raw - b_accel)
    ||a_calibrated|| = 1000 mg

The fit deliberately does not need to know the orientation of any file.  The
matrix is constrained to an upper-triangular, positive-diagonal form.  This
chooses one unambiguous representative of the otherwise rotationally
ambiguous ellipsoid correction.

Important: static data can estimate gyro bias, but it *cannot* estimate gyro
scale or axis misalignment: the true angular rate is zero in every file, so
any gyro scale matrix produces zero after bias removal.  Accordingly, this
program returns the identity as ``gyro_scale_matrix``.  Use a known rotation
rate experiment to calibrate that matrix.

CSV files may have a header with columns such as ax, ay, az, gx, gy, gz
(common longer names such as accel_x and gyro_z are also accepted), or six
numeric columns in that order.  Extra columns are ignored.

Example:
    python IMU_calibration.py data/static_*.csv

Use ``--gravity`` only when the local gravity magnitude in mg is known and a
more precise value is desired. Reported ``accel_bias`` values are in mg and
reported ``gyro_bias`` values are in mdps. The acceleration correction matrix
is dimensionless (it maps mg to mg).
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.optimize import least_squares


ACCEL_ALIASES = (("ax", "accel_x", "accelerometer_x", "acc_x"),
                 ("ay", "accel_y", "accelerometer_y", "acc_y"),
                 ("az", "accel_z", "accelerometer_z", "acc_z"))
GYRO_ALIASES = (("gx", "gyro_x", "gyroscope_x", "gyr_x"),
                ("gy", "gyro_y", "gyroscope_y", "gyr_y"),
                ("gz", "gyro_z", "gyroscope_z", "gyr_z"))


def _normalise_name(name: str) -> str:
    return name.strip().lower().replace("-", "_").replace(" ", "_")


def _column_indices(header: list[str]) -> list[int] | None:
    """Return ax..gz column positions, or None when this has no usable header."""
    names = {_normalise_name(name): i for i, name in enumerate(header)}
    indices: list[int] = []
    for aliases in ACCEL_ALIASES + GYRO_ALIASES:
        match = next((names[name] for name in aliases if name in names), None)
        if match is None:
            return None
        indices.append(match)
    return indices


def read_imu_csv(path: Path) -> np.ndarray:
    """Read one CSV and return finite rows shaped (samples, 6)."""
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        rows = [row for row in csv.reader(handle) if row and any(cell.strip() for cell in row)]
    if not rows:
        raise ValueError("file is empty")

    indices = _column_indices(rows[0])
    data_rows = rows[1:] if indices is not None else rows
    if indices is None:
        indices = list(range(6))

    values: list[list[float]] = []
    for line_number, row in enumerate(data_rows, start=2 if data_rows is not rows else 1):
        try:
            value = [float(row[index]) for index in indices]
        except (IndexError, ValueError) as exc:
            raise ValueError(f"invalid IMU row at line {line_number}") from exc
        if np.all(np.isfinite(value)):
            values.append(value)
    if not values:
        raise ValueError("file contains no finite IMU samples")
    return np.asarray(values, dtype=float)


def unpack_upper_matrix(parameters: np.ndarray) -> np.ndarray:
    """Make a nonsingular 3x3 upper matrix; diagonal entries are exp(log d)."""
    matrix = np.zeros((3, 3))
    matrix[0, 0], matrix[1, 1], matrix[2, 2] = np.exp(parameters[:3])
    matrix[0, 1], matrix[0, 2], matrix[1, 2] = parameters[3:]
    return matrix


def pack_upper_matrix(matrix: np.ndarray) -> np.ndarray:
    return np.array([
        np.log(matrix[0, 0]), np.log(matrix[1, 1]), np.log(matrix[2, 2]),
        matrix[0, 1], matrix[0, 2], matrix[1, 2],
    ])


def ellipsoid_initial_guess(orientation_means: np.ndarray, gravity: float) -> tuple[np.ndarray, np.ndarray]:
    """Algebraic ellipsoid estimate used only to initialize nonlinear fitting."""
    x, y, z = orientation_means.T
    design = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, x, y, z, np.ones(len(x))))
    _, _, vh = np.linalg.svd(design, full_matrices=False)
    p = vh[-1]
    q_matrix = np.array(((p[0], p[3], p[4]), (p[3], p[1], p[5]), (p[4], p[5], p[2])))
    linear = p[6:9]
    constant = p[9]
    # Either sign represents the same algebraic surface.
    if np.linalg.eigvalsh(q_matrix).mean() < 0:
        q_matrix, linear, constant = -q_matrix, -linear, -constant
    bias = -0.5 * np.linalg.solve(q_matrix, linear)
    level = bias @ q_matrix @ bias - constant
    if level <= 0:
        raise ValueError("orientations do not form a valid 3-D ellipsoid")
    metric = gravity**2 * q_matrix / level
    if np.any(np.linalg.eigvalsh(metric) <= 0):
        raise ValueError("orientations do not form a valid 3-D ellipsoid")
    # C.T @ C = metric; NumPy's Cholesky returns L where metric = L @ L.T.
    correction = np.linalg.cholesky(metric).T
    return bias, correction


def calibrate_accelerometer(orientation_means: np.ndarray, gravity: float) -> tuple[np.ndarray, np.ndarray, object]:
    """Fit accel bias and full correction matrix from arbitrary static poses."""
    try:
        initial_bias, initial_matrix = ellipsoid_initial_guess(orientation_means, gravity)
    except (np.linalg.LinAlgError, ValueError):
        # A useful fallback for imperfect datasets; the solver will refine it.
        initial_bias = orientation_means.mean(axis=0)
        initial_matrix = np.eye(3) * gravity / np.mean(np.linalg.norm(orientation_means - initial_bias, axis=1))

    initial = np.r_[initial_bias, pack_upper_matrix(initial_matrix)]

    def residuals(parameters: np.ndarray) -> np.ndarray:
        bias = parameters[:3]
        correction = unpack_upper_matrix(parameters[3:])
        calibrated = (correction @ (orientation_means - bias).T).T
        return (np.linalg.norm(calibrated, axis=1) - gravity) / gravity

    result = least_squares(residuals, initial, method="trf", loss="soft_l1", f_scale=0.01, max_nfev=20_000)
    if not result.success:
        raise RuntimeError(f"accelerometer fit failed: {result.message}")
    return result.x[:3], unpack_upper_matrix(result.x[3:]), result


def expand_patterns(patterns: Iterable[str]) -> list[Path]:
    files: list[Path] = []
    for pattern in patterns:
        matches = glob.glob(pattern)
        files.extend(Path(match) for match in (matches or [pattern]))
    # Preserve order, while avoiding accidentally fitting a file twice.
    return list(dict.fromkeys(files))


def main() -> int:
    parser = argparse.ArgumentParser(description="Orientation-free static IMU ellipsoid calibration")
    parser.add_argument("files", nargs="+", help="CSV paths or glob patterns; one file per static orientation")
    parser.add_argument("--gravity", type=float, default=1000.0,
                        help="local gravity magnitude in mg (default: 1000)")
    parser.add_argument("--output", type=Path, help="optional JSON output path")
    args = parser.parse_args()
    if args.gravity <= 0:
        parser.error("--gravity must be positive")

    files = expand_patterns(args.files)
    if len(files) < 9:
        parser.error("at least 9 distinct static orientations are required for a full 3x3 ellipsoid fit")

    blocks = []
    for file in files:
        try:
            blocks.append(read_imu_csv(file))
        except (OSError, ValueError) as exc:
            parser.error(f"{file}: {exc}")

    # Equal orientation weighting prevents a long recording in one pose from dominating.
    orientation_means = np.vstack([block[:, :3].mean(axis=0) for block in blocks])
    gyro_orientation_means = np.vstack([block[:, 3:].mean(axis=0) for block in blocks])
    all_samples = np.vstack(blocks)
    accel_bias, accel_matrix, fit = calibrate_accelerometer(orientation_means, args.gravity)
    gyro_bias = gyro_orientation_means.mean(axis=0)
    gyro_matrix = np.eye(3)

    calibrated_norms = np.linalg.norm((accel_matrix @ (orientation_means - accel_bias).T).T, axis=1)
    report = {
        "input_files": [str(file) for file in files],
        "orientation_count": len(files),
        "samples_per_orientation": [len(block) for block in blocks],
        "previous_calibration_state": {
            "accel_average_magnitude": float(np.mean(np.linalg.norm(all_samples[:, :3], axis=1))),
            "gyro_average_magnitude": float(np.mean(np.linalg.norm(all_samples[:, 3:], axis=1))),
            "accel_orientation_mean_magnitudes": np.linalg.norm(orientation_means, axis=1).tolist(),
        },
        "estimated_calibration": {
            "accel_bias": accel_bias.tolist(),
            "accel_scale_matrix": accel_matrix.tolist(),
            "gyro_bias": gyro_bias.tolist(),
            "gyro_scale_matrix": gyro_matrix.tolist(),
            "gyro_scale_note": "Identity: gyro scale/misalignment is unobservable from stationary data alone.",
        },
        "fit_quality": {
            "gravity": args.gravity,
            "orientation_calibrated_magnitude_mean": float(calibrated_norms.mean()),
            "orientation_calibrated_magnitude_std": float(calibrated_norms.std(ddof=0)),
            "optimizer_cost": float(fit.cost),
            "optimizer_success": bool(fit.success),
        },
    }
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
