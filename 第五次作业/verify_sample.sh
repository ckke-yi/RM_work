#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${project_dir}/build"

cmake -S "${project_dir}" -B "${build_dir}"
cmake --build "${build_dir}" --parallel
"${build_dir}/sensor_sync" "${project_dir}/config/sensor_sync.conf"
diff -u "${project_dir}/data/expected_result.txt" "${project_dir}/data/result.txt"

echo "Sample verification passed."
