#!/usr/bin/env bash
set -euo pipefail

# Ensure PICO_SDK_PATH is defined
if [ -z "${PICO_SDK_PATH:-}" ]; then
    echo "Error: PICO_SDK_PATH is not set."
    echo "Please export PICO_SDK_PATH to point at your pico-sdk clone."
    exit 1
fi

# Locate script dir and create build folder
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "=== Building Stage1 in ${BUILD_DIR} ==="
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure
echo "-- Running CMake"
cmake ..

# Compile
echo "-- Running Make"
make -j"$(nproc)"

# Report artifact
echo "=== Build complete ==="
echo "  UF2 output is in: ${BUILD_DIR}/*.uf2"

