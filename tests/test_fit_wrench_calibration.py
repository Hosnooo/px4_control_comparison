import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
FITTER = ROOT / "experiment" / "calibration" / "fit_wrench_calibration.py"
COLUMNS = [
    "normalized_thrust",
    "torque_x",
    "torque_y",
    "torque_z",
    "collective_thrust_n",
    "moment_x_nm",
    "moment_y_nm",
    "moment_z_nm",
]
COEFFICIENTS = [
    [1.0, 30.0, 0.2, 0.1, 0.05],
    [0.01, 0.02, 5.0, 0.1, 0.2],
    [-0.02, 0.01, 0.1, 4.0, 0.3],
    [0.0, 0.005, 0.2, 0.3, 2.0],
]


def synthetic_rows():
    rows = []
    for index in range(80):
        thrust = 0.2 + 0.6 * ((index * 17) % 79) / 78.0
        tx = -0.3 + 0.6 * ((index * 23 + 3) % 79) / 78.0
        ty = -0.3 + 0.6 * ((index * 31 + 7) % 79) / 78.0
        tz = -0.3 + 0.6 * ((index * 43 + 11) % 79) / 78.0
        inputs = [1.0, thrust, tx, ty, tz]
        outputs = [sum(a * b for a, b in zip(row, inputs)) for row in COEFFICIENTS]
        rows.append([thrust, tx, ty, tz, *outputs])
    return rows


def parse_record(path):
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, value = line.split("=", 1)
        fields[key] = value
    return fields


class FitWrenchCalibrationTest(unittest.TestCase):
    def run_fitter(self, rows, columns=COLUMNS, minimum_samples=50,
                   maximum_condition=100.0, maximum_rmse=1e-9,
                   maximum_residual=1e-8):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        source = directory / "measurements.csv"
        output = directory / "calibration.txt"
        with source.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            writer.writerows(rows)
        command = [
            sys.executable, str(FITTER), str(source), str(output),
            "--vehicle-id", "f450_lab_01",
            "--calibration-date-utc", "2026-09-01",
            "--method", "static_wrench_stand",
            "--source-data-sha256",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
            "--minimum-samples", str(minimum_samples),
            "--maximum-condition-number", str(maximum_condition),
            "--maximum-collective-rmse-n", str(maximum_rmse),
            "--maximum-moment-rmse-nm", str(maximum_rmse),
            "--maximum-collective-residual-n", str(maximum_residual),
            "--maximum-moment-residual-nm", str(maximum_residual),
        ]
        return subprocess.run(command, text=True, capture_output=True), output

    def test_exact_affine_fit_writes_loader_record(self):
        completed, output = self.run_fitter(synthetic_rows())
        self.assertEqual(completed.returncode, 0, completed.stderr)
        fields = parse_record(output)
        self.assertEqual(fields["authority"], "measured_hardware")
        self.assertEqual(fields["sample_count"], "80")
        for field, expected in zip(
                ["coefficient_collective", "coefficient_moment_x",
                 "coefficient_moment_y", "coefficient_moment_z"],
                COEFFICIENTS):
            actual = [float(value) for value in fields[field].split(",")]
            for actual_value, expected_value in zip(actual, expected):
                self.assertAlmostEqual(actual_value, expected_value, places=11)
        self.assertLess(float(fields["rmse_collective_n"]), 1e-11)
        self.assertAlmostEqual(float(fields["thrust_min"]), 0.2)
        self.assertAlmostEqual(float(fields["thrust_max"]), 0.8)

        runner_source = output.parent / "load_generated_record.cpp"
        runner_binary = output.parent / "load_generated_record"
        runner_source.write_text(r'''#include "control/hardware_wrench_calibration.hpp"
#include <cmath>
int main(int argc, char **argv) {
  if (argc != 2) return 1;
  const control::HardwareCalibrationLimits limits{
      50, 100.0, 1e-9, {1e-9, 1e-9, 1e-9},
      1e-8, {1e-8, 1e-8, 1e-8}, 30, {1e-10, 1e-10, 1e-10}};
  const auto calibration = control::HardwareWrenchCalibration::loadFromFile(
      argv[1], "f450_lab_01", "2026-09-10", limits);
  const auto result = calibration.normalize(19.0, {0.121, -0.129, 0.018}, 0.60);
  return result.ok && std::abs(result.normalized_torque_frd.x - 0.02) < 1e-11 &&
         std::abs(result.normalized_torque_frd.y + 0.03) < 1e-11 &&
         std::abs(result.normalized_torque_frd.z - 0.01) < 1e-11 ? 0 : 2;
}
''', encoding="utf-8")
        compile_result = subprocess.run([
            "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
            f"-I{ROOT / 'control' / 'include'}",
            str(ROOT / "control" / "src" / "hardware_wrench_calibration.cpp"),
            str(runner_source), "-o", str(runner_binary),
        ], text=True, capture_output=True)
        self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
        load_result = subprocess.run([str(runner_binary), str(output)],
                                     text=True, capture_output=True)
        self.assertEqual(load_result.returncode, 0, load_result.stderr)

    def test_rejects_invalid_datasets_and_quality(self):
        rows = synthetic_rows()
        completed, _ = self.run_fitter(rows, columns=COLUMNS[:-1])
        self.assertNotEqual(completed.returncode, 0, "missing column")

        nonfinite = [row[:] for row in rows]
        nonfinite[0][0] = math.nan
        completed, _ = self.run_fitter(nonfinite)
        self.assertNotEqual(completed.returncode, 0, "non-finite sample")

        completed, _ = self.run_fitter(rows[:10], minimum_samples=50)
        self.assertNotEqual(completed.returncode, 0, "insufficient samples")

        singular = [row[:] for row in rows]
        for row in singular:
            row[3] = row[2]
        completed, _ = self.run_fitter(singular)
        self.assertNotEqual(completed.returncode, 0, "rank-deficient design")

        ill_conditioned = [row[:] for row in rows]
        for row in ill_conditioned:
            row[3] *= 1e-8
        completed, _ = self.run_fitter(ill_conditioned, maximum_condition=100.0)
        self.assertNotEqual(completed.returncode, 0, "poorly conditioned design")

        noisy = [row[:] for row in rows]
        noisy[0][4] += 0.1
        completed, _ = self.run_fitter(noisy, maximum_rmse=1e-4,
                                       maximum_residual=1e-4)
        self.assertNotEqual(completed.returncode, 0, "excessive residual")


if __name__ == "__main__":
    unittest.main()
