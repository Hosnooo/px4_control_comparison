#!/usr/bin/env python3
"""Fit the measured affine wrench model consumed by the C++ hardware gate."""

import argparse
import csv
from datetime import date
import math
from pathlib import Path
import re
import sys

import numpy as np


INPUT_COLUMNS = ["normalized_thrust", "torque_x", "torque_y", "torque_z"]
OUTPUT_COLUMNS = [
    "collective_thrust_n", "moment_x_nm", "moment_y_nm", "moment_z_nm"
]
ALL_COLUMNS = INPUT_COLUMNS + OUTPUT_COLUMNS


def positive_finite(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return value


def nonnegative_finite(text):
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("must be a non-negative finite number")
    return value


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_record", type=Path)
    parser.add_argument("--vehicle-id", required=True)
    parser.add_argument("--calibration-date-utc", required=True)
    parser.add_argument("--method", required=True)
    parser.add_argument("--source-data-sha256", required=True)
    parser.add_argument("--minimum-samples", type=int, required=True)
    parser.add_argument("--maximum-condition-number", type=positive_finite, required=True)
    parser.add_argument("--maximum-collective-rmse-n", type=nonnegative_finite, required=True)
    parser.add_argument("--maximum-moment-rmse-nm", type=nonnegative_finite, required=True)
    parser.add_argument("--maximum-collective-residual-n", type=nonnegative_finite,
                        required=True)
    parser.add_argument("--maximum-moment-residual-nm", type=nonnegative_finite,
                        required=True)
    return parser.parse_args(argv)


def validate_provenance(args):
    if not args.vehicle_id.strip() or args.vehicle_id != args.vehicle_id.strip():
        raise ValueError("vehicle ID must be non-empty without surrounding whitespace")
    if not args.method.strip() or args.method != args.method.strip():
        raise ValueError("method must be non-empty without surrounding whitespace")
    try:
        date.fromisoformat(args.calibration_date_utc)
    except ValueError as error:
        raise ValueError("calibration date must be a valid YYYY-MM-DD date") from error
    if not re.fullmatch(r"[0-9a-fA-F]{64}", args.source_data_sha256):
        raise ValueError("source-data SHA-256 must contain exactly 64 hex digits")
    if args.minimum_samples < 5:
        raise ValueError("minimum samples must be at least five")
    if args.maximum_condition_number <= 1.0:
        raise ValueError("maximum condition number must exceed one")


def read_samples(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ALL_COLUMNS:
            raise ValueError("CSV columns must exactly match the documented order")
        samples = []
        for line_number, row in enumerate(reader, start=2):
            try:
                sample = [float(row[column]) for column in ALL_COLUMNS]
            except (TypeError, ValueError) as error:
                raise ValueError(f"invalid numeric sample on line {line_number}") from error
            if not all(math.isfinite(value) for value in sample):
                raise ValueError(f"non-finite sample on line {line_number}")
            samples.append(sample)
    return np.asarray(samples, dtype=float)


def format_number(value):
    return format(float(value), ".17g")


def fit(samples, args):
    if samples.shape[0] < args.minimum_samples:
        raise ValueError("insufficient measured samples")
    normalized = samples[:, :4]
    if (np.any(normalized[:, 0] < 0.0) or np.any(normalized[:, 0] > 1.0) or
            np.any(normalized[:, 1:] < -1.0) or np.any(normalized[:, 1:] > 1.0)):
        raise ValueError("normalized inputs lie outside PX4 bounds")
    ranges = np.ptp(normalized, axis=0)
    if np.any(ranges <= 0.0):
        raise ValueError("each normalized input must span a measured range")

    design = np.column_stack((np.ones(samples.shape[0]), normalized))
    outputs = samples[:, 4:]
    coefficients, _, rank, singular_values = np.linalg.lstsq(design, outputs, rcond=None)
    if rank != design.shape[1] or singular_values[-1] <= 0.0:
        raise ValueError("measured design matrix is rank deficient")
    condition = float(singular_values[0] / singular_values[-1])
    if not math.isfinite(condition) or condition > args.maximum_condition_number:
        raise ValueError("measured design matrix exceeds the condition limit")

    residual = design @ coefficients - outputs
    rmse = np.sqrt(np.mean(np.square(residual), axis=0))
    maximum_residual = np.max(np.abs(residual), axis=0)
    rmse_limits = np.array([
        args.maximum_collective_rmse_n,
        args.maximum_moment_rmse_nm,
        args.maximum_moment_rmse_nm,
        args.maximum_moment_rmse_nm,
    ])
    residual_limits = np.array([
        args.maximum_collective_residual_n,
        args.maximum_moment_residual_nm,
        args.maximum_moment_residual_nm,
        args.maximum_moment_residual_nm,
    ])
    if np.any(rmse > rmse_limits) or np.any(maximum_residual > residual_limits):
        raise ValueError("measured fit residual exceeds an acceptance limit")
    return coefficients.T, condition, rmse, maximum_residual


def write_record(path, args, samples, coefficients, condition, rmse,
                 maximum_residual):
    normalized = samples[:, :4]
    coefficient_names = [
        "coefficient_collective", "coefficient_moment_x",
        "coefficient_moment_y", "coefficient_moment_z"
    ]
    fields = [
        ("schema_version", "1"),
        ("authority", "measured_hardware"),
        ("vehicle_id", args.vehicle_id),
        ("calibration_date_utc", args.calibration_date_utc),
        ("method", args.method),
        ("source_data_sha256", args.source_data_sha256.lower()),
        ("input_units", "px4_normalized_thrust_torque"),
        ("output_units", "N_Nm"),
        ("accept_minimum_sample_count", str(args.minimum_samples)),
        ("accept_maximum_condition_number", format_number(args.maximum_condition_number)),
        ("accept_maximum_collective_rmse_n",
         format_number(args.maximum_collective_rmse_n)),
        ("accept_maximum_moment_rmse_nm", format_number(args.maximum_moment_rmse_nm)),
        ("accept_maximum_collective_residual_n",
         format_number(args.maximum_collective_residual_n)),
        ("accept_maximum_moment_residual_nm",
         format_number(args.maximum_moment_residual_nm)),
        ("thrust_min", format_number(np.min(normalized[:, 0]))),
        ("thrust_max", format_number(np.max(normalized[:, 0]))),
        ("torque_x_min", format_number(np.min(normalized[:, 1]))),
        ("torque_x_max", format_number(np.max(normalized[:, 1]))),
        ("torque_y_min", format_number(np.min(normalized[:, 2]))),
        ("torque_y_max", format_number(np.max(normalized[:, 2]))),
        ("torque_z_min", format_number(np.min(normalized[:, 3]))),
        ("torque_z_max", format_number(np.max(normalized[:, 3]))),
        ("sample_count", str(samples.shape[0])),
        ("design_condition_number", format_number(condition)),
        ("rmse_collective_n", format_number(rmse[0])),
        ("rmse_moment_x_nm", format_number(rmse[1])),
        ("rmse_moment_y_nm", format_number(rmse[2])),
        ("rmse_moment_z_nm", format_number(rmse[3])),
        ("max_residual_collective_n", format_number(maximum_residual[0])),
        ("max_residual_moment_x_nm", format_number(maximum_residual[1])),
        ("max_residual_moment_y_nm", format_number(maximum_residual[2])),
        ("max_residual_moment_z_nm", format_number(maximum_residual[3])),
    ]
    for name, row in zip(coefficient_names, coefficients):
        fields.append((name, ",".join(format_number(value) for value in row)))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(f"{key}={value}\n" for key, value in fields),
                    encoding="utf-8")


def main(argv=None):
    args = arguments(argv)
    try:
        validate_provenance(args)
        samples = read_samples(args.input_csv)
        coefficients, condition, rmse, maximum_residual = fit(samples, args)
        write_record(args.output_record, args, samples, coefficients, condition,
                     rmse, maximum_residual)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
