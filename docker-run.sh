#!/usr/bin/env bash
set -euo pipefail

echo "=== AeroBlend Docker ==="
echo "빌드 중... (처음엔 Rust + C++ 컴파일 때문에 시간이 걸립니다)"
docker compose build

echo ""
echo "실행 중..."
docker compose up -d

echo ""
echo "============================================"
echo "  브라우저에서 http://localhost:6080 접속"
echo "============================================"
echo ""
echo "종료: docker compose down"
echo "로그: docker compose logs -f"
