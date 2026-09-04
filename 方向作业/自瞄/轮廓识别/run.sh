#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
cmake -S . -B build
cmake --build build --parallel
mkdir -p output
./build/energy_contour_recognition config/energy.conf
