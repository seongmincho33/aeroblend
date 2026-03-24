#!/usr/bin/env bash
set -euo pipefail

# --- 가상 디스플레이 시작 ---
echo "[aeroblend] Xvfb 시작 (해상도: ${RESOLUTION:-1280x720x24})"
Xvfb :99 -screen 0 "${RESOLUTION:-1280x720x24}" -ac +extension GLX &
sleep 1

# --- VNC 서버 시작 ---
echo "[aeroblend] x11vnc 시작"
x11vnc -display :99 -forever -nopw -shared -rfbport 5900 -bg -o /tmp/x11vnc.log

# --- noVNC (브라우저 → VNC 프록시) ---
echo "[aeroblend] noVNC 시작 → http://localhost:6080"
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &

# --- AeroBlend 실행 ---
echo "[aeroblend] 시뮬레이터 시작: $*"
exec /app/aeroblend "$@"
