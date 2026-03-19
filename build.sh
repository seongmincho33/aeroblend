#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"

echo "=== AeroBlend Native Build ==="

# CMake configure + build (Corrosion handles Rust build automatically)
echo "--- CMake configure ---"
cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release

echo "--- CMake build ---"
cmake --build "$BUILD_DIR" -j "$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

echo "=== Build complete ==="
echo "Run: ./$BUILD_DIR/cpp/aeroblend --model <path-to-model.glb>"
