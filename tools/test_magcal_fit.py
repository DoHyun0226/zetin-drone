import csv
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ground.magcal import (  # noqa: E402
    CoverageError,
    FitError,
    apply_calibration,
    fit_calibration,
    format_report,
    format_firmware_constants,
    load_calibration_csv,
    recalibrate_mag_comp,
)


FIELD_UT = 50.4
TRUE_HARD_IRON = np.array([-1.8, 9.3, -12.5])
TRUE_SENSOR_SOFT_IRON = np.array(
    [
        [1.12, 0.05, -0.03],
        [0.05, 0.92, 0.04],
        [-0.03, 0.04, 1.05],
    ]
)


def make_samples(seed, sample_count=1500, spike_count=0, coverage="drone"):
    rng = np.random.default_rng(seed)
    if coverage == "full":
        directions = rng.normal(size=(sample_count, 3))
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    elif coverage == "planar":
        yaw = rng.uniform(0.0, 2.0 * np.pi, sample_count)
        directions = np.column_stack(
            (np.cos(yaw), np.sin(yaw), np.zeros(sample_count))
        )
    else:
        yaw = rng.uniform(0.0, 2.0 * np.pi, sample_count)
        tilt = np.radians(rng.normal(-50.0, 22.0, sample_count))
        directions = np.column_stack(
            (
                np.cos(tilt) * np.cos(yaw),
                np.cos(tilt) * np.sin(yaw),
                np.sin(tilt),
            )
        )

    true_field = FIELD_UT * directions
    samples = (
        TRUE_SENSOR_SOFT_IRON @ true_field.T
    ).T + TRUE_HARD_IRON
    samples += rng.normal(0.0, 0.3, size=samples.shape)
    clean_mask = np.ones(sample_count, dtype=bool)
    if spike_count:
        indices = rng.choice(sample_count, size=spike_count, replace=False)
        samples[indices] += rng.normal(0.0, 40.0, size=(spike_count, 3))
        clean_mask[indices] = False
    return samples, true_field, clean_mask


def relative_heading_error_95(samples, clean_mask, result):
    corrected = apply_calibration(samples[clean_mask], result)
    expected = np.linalg.solve(
        TRUE_SENSOR_SOFT_IRON,
        (samples[clean_mask] - TRUE_HARD_IRON).T,
    ).T
    measured_heading = np.degrees(np.arctan2(corrected[:, 1], corrected[:, 0]))
    true_heading = np.degrees(np.arctan2(expected[:, 1], expected[:, 0]))
    error = (measured_heading - true_heading + 180.0) % 360.0 - 180.0
    error -= np.median(error)
    error = (error + 180.0) % 360.0 - 180.0
    return float(np.percentile(np.abs(error), 95.0))


class MagnetometerCalibrationFitTests(unittest.TestCase):
    def test_report_exposes_only_radial_fit_residual(self):
        samples, _, _ = make_samples(23, coverage="full")

        result = fit_calibration(samples)
        corrected = apply_calibration(samples[result.inlier_mask], result)
        corrected_norms = np.linalg.norm(corrected, axis=1)
        want_ut = float(
            np.percentile(
                np.abs(corrected_norms - result.target_radius_ut), 95.0
            )
        )

        angle = np.radians(30.0)
        rotation = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        rotated = (rotation @ corrected.T).T
        np.testing.assert_allclose(
            np.linalg.norm(rotated, axis=1), corrected_norms, atol=1e-12
        )
        original_heading = np.degrees(
            np.arctan2(corrected[:, 1], corrected[:, 0])
        )
        rotated_heading = np.degrees(
            np.arctan2(rotated[:, 1], rotated[:, 0])
        )
        heading_error = (
            rotated_heading - original_heading + 180.0
        ) % 360.0 - 180.0
        self.assertAlmostEqual(
            30.0, float(np.percentile(np.abs(heading_error), 95.0)), places=10
        )

        self.assertAlmostEqual(
            want_ut, result.radial_residual_95_ut, places=10
        )
        report = format_report(result)
        self.assertIn("95p |B| radial residual", report)
        self.assertIn("uT", report)
        self.assertIn("%", report)
        self.assertNotIn("heading error", report.lower())

    def test_recovers_known_ellipsoid_and_relative_heading(self):
        samples, _, clean_mask = make_samples(7, coverage="full")

        result = fit_calibration(samples)

        center_error = np.linalg.norm(result.hard_iron - TRUE_HARD_IRON)
        heading_error = relative_heading_error_95(samples, clean_mask, result)
        corrected_norms = np.linalg.norm(apply_calibration(samples, result), axis=1)
        expected_scale = float(np.mean(np.linalg.eigvalsh(TRUE_SENSOR_SOFT_IRON)))
        expected_correction = expected_scale * np.linalg.inv(
            TRUE_SENSOR_SOFT_IRON
        )
        self.assertLess(center_error, 0.35)
        np.testing.assert_allclose(
            result.soft_iron, expected_correction, rtol=0.02, atol=0.002
        )
        self.assertLess(heading_error, 1.5)
        self.assertLess(np.std(corrected_norms) / np.mean(corrected_norms), 0.02)
        self.assertTrue(np.all(np.linalg.eigvalsh(result.soft_iron) > 0.0))

    def test_seventy_five_spikes_never_break_twenty_fits(self):
        failures = []
        worst_heading_error = 0.0
        for seed in range(20):
            samples, _, clean_mask = make_samples(seed, spike_count=75)
            try:
                result = fit_calibration(samples)
            except (CoverageError, FitError) as error:
                failures.append((seed, str(error)))
                continue
            heading_error = relative_heading_error_95(samples, clean_mask, result)
            worst_heading_error = max(worst_heading_error, heading_error)
            if heading_error >= 1.5:
                failures.append((seed, f"heading95={heading_error:.3f}"))

        self.assertEqual([], failures)
        self.assertLess(worst_heading_error, 1.5)

    def test_degenerate_planar_coverage_is_rejected(self):
        samples, _, _ = make_samples(3, coverage="planar")

        with self.assertRaises(CoverageError):
            fit_calibration(samples)

    def test_hyperboloid_samples_are_rejected(self):
        rng = np.random.default_rng(11)
        u = rng.uniform(-1.6, 1.6, 1200)
        v = rng.uniform(0.0, 2.0 * np.pi, 1200)
        samples = np.column_stack(
            (
                24.0 * np.cosh(u) * np.cos(v),
                31.0 * np.cosh(u) * np.sin(v),
                20.0 * np.sinh(u),
            )
        )
        samples += np.array([4.0, -7.0, 2.0])

        with self.assertRaises(FitError):
            fit_calibration(samples)

    def test_csv_defaults_to_active_rows_and_all_bypasses_selector(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flight.csv"
            with path.open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=(
                        "Timestamp",
                        "Mag_X",
                        "Mag_Y",
                        "Mag_Z",
                        "Mag_Cal_Active",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    (
                        {
                            "Timestamp": "00:00:00.000",
                            "Mag_X": "1.0",
                            "Mag_Y": "2.0",
                            "Mag_Z": "3.0",
                            "Mag_Cal_Active": "0",
                        },
                        {
                            "Timestamp": "00:00:00.050",
                            "Mag_X": "4.0",
                            "Mag_Y": "5.0",
                            "Mag_Z": "6.0",
                            "Mag_Cal_Active": "1",
                        },
                        {
                            "Timestamp": "00:00:00.100",
                            "Mag_X": "7.0",
                            "Mag_Y": "8.0",
                            "Mag_Z": "9.0",
                            "Mag_Cal_Active": "1",
                        },
                    )
                )

            selected = load_calibration_csv(path)
            all_rows = load_calibration_csv(path, use_all=True)

        np.testing.assert_array_equal(
            selected, np.array([[4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
        )
        self.assertEqual((3, 3), all_rows.shape)

    def test_firmware_and_mag_comp_output_uses_matrix_rows(self):
        samples, _, _ = make_samples(19, coverage="full")
        result = fit_calibration(samples)
        block = format_firmware_constants(result)

        self.assertIn("const float MAG_HARD_IRON[3]", block)
        self.assertIn("const float MAG_SOFT_IRON[3][3]", block)
        self.assertNotIn("nan", block.lower())
        custom_matrix = np.array(
            [[1.0, 0.2, 0.0], [-0.1, 0.9, 0.3], [0.05, -0.2, 1.1]]
        )
        transformed = recalibrate_mag_comp(
            custom_matrix, np.array([0.007497, -0.001218, -0.000640])
        )
        np.testing.assert_allclose(
            transformed,
            np.array([0.0072534, -0.0020379, -0.00008555]),
            atol=1e-10,
        )


if __name__ == "__main__":
    unittest.main()
