#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

echo "========================================="
echo "  AeroBlend Docker 빌드 (Mac)"
echo "  처음엔 Rust+C++ 컴파일로 오래 걸림"
echo "========================================="
echo ""

docker compose build

echo ""
echo "========================================="
echo "  빌드 완료! DDALKAK_MAC.sh 로 실행"
echo "========================================="
