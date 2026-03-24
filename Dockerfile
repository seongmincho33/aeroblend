# ============================================================
# AeroBlend Docker — macOS / Windows / Linux 어디서든 실행
# 브라우저로 localhost:6080 접속
# ============================================================

# ── Stage 1: 빌드 ──
FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    curl \
    ca-certificates \
    libsdl2-dev \
    libgl-dev \
    libglu1-mesa-dev \
    mesa-common-dev \
    && rm -rf /var/lib/apt/lists/*

# Rust 설치
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /src
COPY . .

# CMake 빌드 (Corrosion이 Rust를 자동 빌드)
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"

# ── Stage 2: 런타임 ──
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # SDL2 런타임
    libsdl2-2.0-0 \
    # Mesa 소프트웨어 렌더링 (llvmpipe)
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl-mesa0 \
    # 가상 디스플레이 + VNC
    xvfb \
    x11vnc \
    # noVNC (브라우저 접속)
    novnc \
    websockify \
    # 폰트 (HUD용)
    fonts-noto-cjk \
    # 기타
    supervisor \
    && rm -rf /var/lib/apt/lists/*

# 실행 파일 복사
COPY --from=builder /src/build/cpp/aeroblend /app/aeroblend
COPY --from=builder /src/build/cpp/shaders /app/shaders
COPY assets /app/assets

# noVNC index.html 심링크 (배포판마다 경로가 다를 수 있음)
RUN if [ ! -f /usr/share/novnc/index.html ] && [ -f /usr/share/novnc/vnc.html ]; then \
        ln -s /usr/share/novnc/vnc.html /usr/share/novnc/index.html; \
    fi

# entrypoint 스크립트
COPY docker/entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# supervisord 설정
COPY docker/supervisord.conf /etc/supervisor/conf.d/aeroblend.conf

WORKDIR /app

# 환경변수: Mesa 소프트웨어 렌더링 강제
ENV DISPLAY=:99
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV SDL_VIDEODRIVER=x11

# noVNC 포트
EXPOSE 6080

ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["--model", "/app/assets/models/example_aircraft.glb"]
