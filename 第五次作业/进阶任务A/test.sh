#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
cmake -S . -B build
cmake --build build --parallel
mkdir -p output
./build/rm_vision_sim config/vision.conf
python3 tools/check_result.py \
  output/result.csv data/ground_truth_public.csv output/result.avi
echo "Public vision pipeline test passed."
