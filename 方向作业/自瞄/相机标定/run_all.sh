#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
cmake -S . -B build
cmake --build build --parallel
mkdir -p output
./build/rm_camera_calibrate config/calibration.conf
./build/rm_undistort config/calibration.conf
echo "Camera calibration and undistortion completed."
echo "Calibration data: output/camera.yaml"
echo "Comparison video: output/undistorted_comparison.avi"
