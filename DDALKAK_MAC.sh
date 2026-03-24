#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

ADDON_SRC="$(pwd)/scripts/generate_example_aircraft.py"
PROJECT_ROOT="$(pwd)"

echo "========================================="
echo "  AeroBlend 딸깍 (Mac)"
echo "========================================="
echo ""

# ── 1. Blender 찾기 ──
BLENDER=""
if [ -d "/Applications/Blender.app" ]; then
    BLENDER="/Applications/Blender.app/Contents/MacOS/Blender"
elif command -v blender &>/dev/null; then
    BLENDER="blender"
fi

if [ -z "$BLENDER" ]; then
    echo "[오류] Blender를 찾을 수 없습니다."
    echo "  - brew install --cask blender"
    echo "  - 또는 https://blender.org 에서 설치"
    exit 1
fi

echo "Blender: $BLENDER"

# ── 2. Blender 버전 감지 → addons 디렉토리 찾기 ──
BLENDER_VER=$("$BLENDER" --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
ADDONS_DIR="$HOME/Library/Application Support/Blender/$BLENDER_VER/scripts/addons"
STARTUP_DIR="$HOME/Library/Application Support/Blender/$BLENDER_VER/scripts/startup"

echo "Blender $BLENDER_VER 감지"

# ── 3. 애드온 설치 (심링크) ──
mkdir -p "$ADDONS_DIR"
ADDON_DST="$ADDONS_DIR/generate_example_aircraft.py"

if [ -L "$ADDON_DST" ] || [ -f "$ADDON_DST" ]; then
    rm "$ADDON_DST"
fi

ln -s "$ADDON_SRC" "$ADDON_DST"
echo "애드온 심링크 완료"

# ── 4. 자동 활성화 스타트업 스크립트 설치 ──
mkdir -p "$STARTUP_DIR"
cat > "$STARTUP_DIR/aeroblend_autostart.py" << PYEOF
"""AeroBlend 애드온 자동 활성화 (DDALKAK이 생성)"""
import bpy
import addon_utils

def _enable_aeroblend():
    addon_utils.enable('generate_example_aircraft', default_set=True, persistent=True)
    prefs = bpy.context.preferences.addons.get('generate_example_aircraft')
    if prefs:
        prefs.preferences.project_root = r"${PROJECT_ROOT}"
    bpy.ops.wm.save_userpref()
    # 한 번 실행 후 타이머 제거
    return None

# Blender 완전히 초기화된 후 실행
bpy.app.timers.register(_enable_aeroblend, first_interval=1.0)
PYEOF

echo "자동 활성화 스크립트 설치 완료"

# ── 5. Blender 실행 ──
echo ""
echo "========================================="
echo "  Blender 여는 중..."
echo "  사이드바(N) > AeroBlend 탭"
echo "========================================="

open -a Blender
