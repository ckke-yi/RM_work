#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


def as_float(value):
    return float(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_csv")
    parser.add_argument("ground_truth_csv")
    parser.add_argument("output_video")
    args = parser.parse_args()

    with open(args.result_csv, newline="", encoding="utf-8") as stream:
      results = list(csv.DictReader(stream))
    with open(args.ground_truth_csv, newline="", encoding="utf-8") as stream:
      truth = list(csv.DictReader(stream))
    if len(results) != len(truth) or len(results) != 60:
        raise ValueError("expected exactly 60 result rows")

    visible_count = 0
    detected_count = 0
    measurement_errors = []
    tracking_errors = []
    predicted_occlusion = 0
    for result, expected in zip(results, truth):
        if result["frame_id"] != expected["frame_id"]:
            raise ValueError("frame order mismatch")
        visible = expected["visible"] == "1"
        detected = result["detected"] == "1"
        x = float(expected["x"])
        y = float(expected["y"])
        if visible:
            visible_count += 1
            if detected:
                detected_count += 1
                measurement_errors.append(
                    math.hypot(as_float(result["measure_x"]) - x,
                               as_float(result["measure_y"]) - y)
                )
        if result["track_x"].lower() != "nan":
            tracking_errors.append(
                math.hypot(as_float(result["track_x"]) - x,
                           as_float(result["track_y"]) - y)
            )
        frame_id = int(result["frame_id"])
        if 25 <= frame_id <= 28 and result["predicted"] == "1":
            predicted_occlusion += 1

    detection_rate = detected_count / visible_count
    mean_measurement_error = sum(measurement_errors) / len(measurement_errors)
    mean_tracking_error = sum(tracking_errors) / len(tracking_errors)
    if detection_rate < 0.95:
        raise ValueError(f"detection rate too low: {detection_rate:.3f}")
    if mean_measurement_error > 3.0:
        raise ValueError(f"measurement error too high: {mean_measurement_error:.3f}")
    if mean_tracking_error > 15.0:
        raise ValueError(f"tracking error too high: {mean_tracking_error:.3f}")
    if predicted_occlusion != 4:
        raise ValueError("all four occluded frames must be predicted")
    if not Path(args.output_video).is_file() or Path(args.output_video).stat().st_size < 1000:
        raise ValueError("visualization video is missing or empty")

    print(
        f"Vision result valid: detection_rate={detection_rate:.3f}, "
        f"measurement_error={mean_measurement_error:.3f}, "
        f"tracking_error={mean_tracking_error:.3f}"
    )


if __name__ == "__main__":
    main()
