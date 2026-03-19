# AeroBlend 초보자 실행 가이드

이 문서는 AeroBlend를 처음 접하는 분이 **설치부터 실행, 조종까지** 따라할 수 있도록 만든 단계별 안내서입니다.

---

## 목차

1. [AeroBlend가 뭔가요?](#1-aeroblend가-뭔가요)
2. [내 컴퓨터에서 돌아갈까요?](#2-내-컴퓨터에서-돌아갈까요)
3. [필요한 프로그램 설치하기](#3-필요한-프로그램-설치하기)
4. [프로젝트 빌드하기](#4-프로젝트-빌드하기)
5. [실행하기](#5-실행하기)
6. [비행기 조종법](#6-비행기-조종법)
7. [나만의 비행기 만들기](#7-나만의-비행기-만들기)
8. [화면 읽는 법 (HUD 계기판)](#8-화면-읽는-법-hud-계기판)
9. [문제가 생겼을 때](#9-문제가-생겼을-때)

---

## 1. AeroBlend가 뭔가요?

AeroBlend는 **항공 물리 시뮬레이터**입니다.

- Blender에서 직접 만든 비행기 3D 모델(`.glb` 파일)을 불러와서 날릴 수 있습니다
- 실제 물리 법칙(대기압, 양력, 항력)을 계산해서 비행기가 현실적으로 움직입니다
- War Thunder 게임처럼 마우스로 방향을 잡으면 비행기가 따라갑니다

### 어떤 기술로 만들어졌나요?

```
비행기 물리/수학  →  Rust 언어로 작성 (빠르고 안전)
화면 그리기      →  C++ / OpenGL로 작성 (3D 렌더링)
두 언어 연결     →  FFI라는 인터페이스로 자동 연결
```

초보자분은 이 내부 구조를 몰라도 됩니다. 아래 순서대로 따라하면 실행할 수 있습니다.

---

## 2. 내 컴퓨터에서 돌아갈까요?

### 지원 환경

| 항목 | 최소 사양 |
|------|-----------|
| OS | macOS 12+, Linux (Ubuntu 20.04+) |
| GPU | OpenGL 3.3 지원 (2010년 이후 대부분의 GPU) |
| RAM | 4GB 이상 |
| 디스크 | 약 2GB (빌드 포함) |

> **Windows는요?** 현재는 macOS와 Linux를 주 대상으로 합니다. Windows에서는 WSL2를 통해 사용할 수 있습니다.

---

## 3. 필요한 프로그램 설치하기

빌드하려면 4가지 도구가 필요합니다. 아래 순서대로 설치하세요.

### macOS (Homebrew 사용)

터미널(Terminal.app)을 열고 아래 명령어를 **한 줄씩** 입력하세요.

**Step 1) Homebrew가 없다면 먼저 설치:**

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

**Step 2) CMake와 SDL2 설치:**

```bash
brew install cmake sdl2
```

**Step 3) Rust 설치:**

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

설치 중 물어보면 `1) Proceed with standard installation` 선택 후 Enter.

설치 후 터미널을 **껐다가 다시** 열어야 `cargo` 명령이 인식됩니다.

**Step 4) 설치 확인:**

```bash
cmake --version     # 3.20 이상이면 OK
sdl2-config --version  # 2.0 이상이면 OK
rustc --version     # 1.75 이상이면 OK
cargo --version     # rustc와 함께 설치됨
```

4개 모두 버전이 출력되면 준비 완료입니다.

### Linux (Ubuntu/Debian)

```bash
# CMake, SDL2, 빌드 도구
sudo apt update
sudo apt install -y cmake build-essential pkg-config libsdl2-dev

# Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

---

## 4. 프로젝트 빌드하기

"빌드"란 소스 코드를 실행 가능한 프로그램으로 변환하는 과정입니다.

### 간단한 방법 (빌드 스크립트)

프로젝트 폴더에서:

```bash
cd /path/to/aeroblend    # 프로젝트 폴더로 이동
./build.sh               # 자동 빌드
```

> `Permission denied` 오류가 나면: `chmod +x build.sh` 를 먼저 실행하세요.

### 수동 빌드 (무슨 일이 일어나는지 알고 싶다면)

```bash
cd /path/to/aeroblend

# 1) build 폴더 생성 + CMake 설정
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 2) 컴파일 (CPU 코어 수만큼 병렬로)
cmake --build build -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)
```

**무슨 일이 일어나나요?**

```
cmake -B build ...
  → Rust 코드 자동 컴파일 (Corrosion이 cargo build를 대신 해줌)
  → SDL2, OpenGL 라이브러리를 찾음
  → C++ 코드 컴파일 + Rust 라이브러리와 링크
  → 최종 실행 파일 생성: build/cpp/aeroblend
```

### 빌드 시간

- 첫 빌드: 약 1~3분 (Rust 의존성 다운로드 + 컴파일)
- 이후 재빌드: 약 10~30초 (변경된 파일만)

### Rust 테스트 실행 (선택)

물리/수학 엔진이 제대로 동작하는지 확인하고 싶다면:

```bash
cd rust
cargo test
```

`test result: ok. 50 passed` 가 나오면 모든 테스트 통과입니다.

---

## 5. 실행하기

### 모델 없이 실행 (환경만 보기)

```bash
./build/cpp/aeroblend
```

하늘, 지면, 활주로만 보입니다. 물리 시뮬레이션은 동작하지 않지만 카메라와 HUD를 확인할 수 있습니다.

### 비행기 모델과 함께 실행

```bash
./build/cpp/aeroblend --model /path/to/your-aircraft.glb
```

비행기가 화면에 나타나고, 물리 시뮬레이션이 시작됩니다.

### 예제 비행기 만들어서 바로 실행

비행기 모델이 없다면 예제를 자동 생성할 수 있습니다. (Blender 필요)

```bash
# 예제 비행기 생성
blender --background --python scripts/generate_example_aircraft.py

# 생성된 모델로 실행
./build/cpp/aeroblend --model assets/models/example_aircraft.glb
```

### 빌드부터 실행까지 한 번에

```bash
./build.sh && ./build/cpp/aeroblend --model assets/models/example_aircraft.glb
```

### Blender에서 바로 실행하기 (추천)

터미널 없이 **Blender 안에서** 빌드와 실행을 모두 할 수 있습니다.

#### 애드온 설치

1. Blender 실행
2. **Edit > Preferences > Add-ons** 열기
3. 우측 상단 **▼ 드롭다운** 클릭 > **Install from Disk** 선택
4. 파일 브라우저에서 `scripts/generate_example_aircraft.py` 선택

#### 애드온 활성화

1. Blender 실행 (설치 후 재시작)
2. **Edit > Preferences > Add-ons**
3. 검색창에 `AeroBlend` 입력
4. **"AeroBlend: Example Aircraft Generator"** 체크하여 활성화
5. 3D 뷰포트에서 `N` 키 → 사이드바 **AeroBlend** 탭

사이드바에 세 개의 섹션이 나타납니다:

```
┌──────────────────────────┐
│  Quick Start             │
│  [🐵 Example Aircraft]   │  ← 원클릭: 예시 모델 생성 + 내보내기
│                          │
│  Model                   │
│  [+ Generate Aircraft]   │  ← 파라미터 조절 가능
│  [↗ Export Aircraft GLB] │  ← .glb 파일로 내보내기
│                          │
│  Simulator               │
│  [▶ Build & Run]         │  ← 빌드 + 실행 (원클릭)
│  [Build]  [Run]          │  ← 개별 실행
└──────────────────────────┘
```

#### 가장 빠른 워크플로우

1. **Example Aircraft** 클릭 → 예시 비행기 생성 + GLB 자동 내보내기
2. **Build & Run** 클릭 → 빌드 + 시뮬레이터 실행

이 두 번의 클릭만으로 모델 생성부터 비행까지 Blender 안에서 끝낼 수 있습니다.

#### 커스텀 워크플로우

1. **Generate Aircraft** 클릭 → 파라미터 다이얼로그에서 날개/동체 크기 조절
2. Blender에서 직접 편집 (색상, 형태 수정 등)
3. **Build & Run** 클릭 → GLB 내보내기 + 빌드 + 시뮬레이터 실행

> **프로젝트 경로 설정:** 스크립트가 자동으로 프로젝트 위치를 찾지만, 찾지 못하면 **Edit > Preferences > Add-ons** 에서 AeroBlend를 펼쳐 **Project Root** 경로를 직접 지정하세요.

---

## 6. 비행기 조종법

AeroBlend는 **War Thunder 스타일** 조종입니다.
마우스로 "저기로 가고 싶다"는 방향을 가리키면, 비행기가 자동으로 그 방향으로 회전합니다.

### 마우스

| 동작 | 설명 |
|------|------|
| 마우스 이동 | 에임 방향 지정 (비행기가 따라감) |

### 엔진

| 키 | 동작 | 설명 |
|----|------|------|
| `Shift` | 스로틀 증가 | 엔진 출력을 올립니다 (가속) |
| `Ctrl` | 스로틀 감소 | 엔진 출력을 내립니다 (감속) |

> 이륙하려면 `Shift`를 꾹 눌러서 스로틀을 100%까지 올리세요.

### 수동 조종 (키보드)

마우스 에임 외에 키보드로 직접 조종할 수도 있습니다.

| 키 | 동작 | 설명 |
|----|------|------|
| `W` | 기수 내리기 (피치 다운) | 비행기 코를 아래로 |
| `S` | 기수 올리기 (피치 업) | 비행기 코를 위로 |
| `A` | 좌측 롤 | 비행기를 왼쪽으로 기울임 |
| `D` | 우측 롤 | 비행기를 오른쪽으로 기울임 |
| `Q` | 좌측 요 | 비행기 코를 왼쪽으로 |
| `E` | 우측 요 | 비행기 코를 오른쪽으로 |

### 장비 토글

| 키 | 동작 | 설명 |
|----|------|------|
| `G` | 랜딩기어 | 바퀴를 접거나 펼칩니다 |
| `F` | 플랩 | 날개의 플랩을 내리면 양력이 증가합니다 (이착륙 시 사용) |
| `B` | 브레이크 | 지상에서 정지, 공중에서 에어브레이크 |

### 화면/기타

| 키 | 동작 | 설명 |
|----|------|------|
| `V` | 카메라 전환 | 추적 카메라 ↔ 조종석 카메라 |
| `H` | HUD 토글 | 계기판 표시/숨기기 |
| `ESC` | 종료 | 프로그램을 닫습니다 |

### 이륙 순서 (처음 해보는 분)

1. `Shift` 꾹 누르기 → 스로틀 100%까지 올리기
2. 마우스를 화면 약간 위로 → 기수가 살짝 들어감
3. 속도가 붙으면 자연스럽게 이륙!
4. 이륙 후 `G`를 눌러 랜딩기어 접기 (항력 감소)

### 착륙 순서

1. `Ctrl`로 스로틀을 30% 정도로 줄이기
2. `F`로 플랩 내리기 (양력 증가, 저속에서도 뜰 수 있음)
3. 마우스를 살짝 아래로 → 서서히 하강
4. `G`로 랜딩기어 펼치기
5. 지면에 닿으면 `B`로 브레이크

---

## 7. 나만의 비행기 만들기

Blender에서 만든 3D 모델을 바로 가져올 수 있습니다.

### 핵심 규칙: 메시 이름 짓기

AeroBlend는 **메시(부품) 이름**을 보고 "이건 날개다", "이건 엔진이다"를 자동으로 판단합니다.
이름만 잘 지으면 물리 파라미터가 자동으로 설정됩니다.

| 이 이름을 포함하면 | 이걸로 인식됨 | 예시 |
|---------------------|---------------|------|
| `Wing_Left`, `wing_l` | 왼쪽 날개 | `Wing_Left_Main` |
| `Wing_Right`, `wing_r` | 오른쪽 날개 | `Wing_Right_01` |
| `H_Tail`, `stabilizer` | 수평 꼬리날개 | `H_Tail_Left` |
| `V_Tail`, `rudder` | 수직 꼬리날개 | `Vertical_Stabilizer` |
| `Jet_Engine`, `turbine` | 제트 엔진 | `Jet_Engine_R` |
| `Propeller`, `blade` | 프로펠러 | `Propeller_Main` |
| `Fuselage`, `body` | 동체 | `Fuselage` |
| `Gear`, `wheel` | 랜딩기어 | `Gear_Front` |

### Blender에서 내보내기

1. Blender에서 비행기 모델링
2. 각 부품의 메시 이름을 위 규칙에 맞게 설정
3. 메뉴: **File > Export > glTF 2.0 (.glb/.gltf)** 클릭
4. 포맷을 `.glb` (단일 파일)로 선택, 저장
5. 실행:

```bash
./build/cpp/aeroblend --model /path/to/my-plane.glb
```

### 예제 비행기 스크립트 사용

직접 모델링하기 어렵다면 Python 스크립트로 예제 비행기를 자동 생성할 수 있습니다.

**방법 A: 터미널에서 (Blender 화면 없이)**

```bash
blender --background --python scripts/generate_example_aircraft.py
```

**방법 B: Blender 안에서 (파라미터 조절 가능)**

1. Blender 실행
2. **Edit > Preferences > Add-ons > Install from Disk** 클릭
3. `scripts/generate_example_aircraft.py` 선택
4. 목록에서 **"AeroBlend: Example Aircraft Generator"** 체크
5. 3D 뷰포트에서 `N` 키 → 사이드바 **AeroBlend** 탭
6. **Generate Aircraft** 클릭 → 파라미터 조절 (날개 길이, 동체 크기 등)
7. **Export Aircraft GLB** 클릭 → 파일 저장

---

## 8. 화면 읽는 법 (HUD 계기판)

`H` 키로 HUD를 켜면 화면에 비행 정보가 표시됩니다.

```
┌─────────────────────────────────────────────────┐
│                                                 │
│  [속도]          [자세계]          [고도]        │
│  ┌────┐    ┌───────────────┐     ┌────┐        │
│  │ 250│    │    ─── 10 ─── │     │5000│        │
│  │ 240│    │   /         \ │     │4900│        │
│  │►230│    │  ─── 0 ───── │     │►4850│       │
│  │ 220│    │   \         / │     │4800│        │
│  │ 210│    │    ───-10─── │     │4700│        │
│  └────┘    └───────────────┘     └────┘        │
│                                                 │
│  [방향]  ◄─── N ── 030 ── 060 ── E ───►        │
│                                                 │
│  THR [████████░░] 80%     G-Force: 1.2G         │
│  GEAR: UP  |  FLAP: OFF  |  BRK: OFF           │
│                                        FPS: 60  │
└─────────────────────────────────────────────────┘
```

| 계기 | 위치 | 읽는 법 |
|------|------|---------|
| **속도 (Airspeed)** | 왼쪽 | 현재 속도 (knots). 숫자가 클수록 빠름 |
| **고도 (Altimeter)** | 오른쪽 | 현재 높이 (feet). 0이면 지면 |
| **자세계 (Attitude)** | 가운데 | 비행기 기울기. 수평선이 가운데면 수평 비행 |
| **방향 (Heading)** | 아래쪽 | 바라보는 방향. N=북, E=동, S=남, W=서 |
| **스로틀 (Throttle)** | 하단 좌측 | 엔진 출력 (0~100%) |
| **G-Force** | 하단 우측 | 받는 중력 배수. 1.0이 일반, 급기동시 증가 |
| **상태 표시** | 최하단 | GEAR(바퀴), FLAP(플랩), BRK(브레이크) 상태 |
| **FPS** | 우측 하단 | 초당 프레임 수. 30 이상이면 부드러움 |

---

## 9. 문제가 생겼을 때

### 빌드 에러

| 증상 | 원인 | 해결 |
|------|------|------|
| `cmake: command not found` | CMake 미설치 | `brew install cmake` |
| `sdl2 not found` | SDL2 미설치 | `brew install sdl2` |
| `cargo: command not found` | Rust 미설치 또는 터미널 재시작 필요 | Rust 설치 후 터미널 재시작 |
| `linker error ... CoreFoundation` | macOS 프레임워크 누락 | CMakeLists.txt에 이미 설정됨. `build/` 삭제 후 재빌드 |
| Rust 컴파일 에러 | Rust 버전이 낮음 | `rustup update` 실행 |

### 실행 에러

| 증상 | 원인 | 해결 |
|------|------|------|
| `No such file or directory` | 실행 파일 경로 오류 | `./build/cpp/aeroblend` 확인 |
| `Failed to load model` | glb 파일 경로 오류 | `--model` 뒤의 경로 확인 |
| 화면이 검게 나옴 | 셰이더 파일 누락 | `build/cpp/shaders/` 폴더에 `.vert`, `.frag` 파일이 있는지 확인 |
| FPS가 매우 낮음 | GPU 드라이버 문제 | GPU 드라이버 업데이트 |

### 처음부터 다시 빌드하고 싶을 때

```bash
rm -rf build
./build.sh
```

`build/` 폴더를 삭제하면 완전 초기 상태로 돌아갑니다.

### Rust 테스트로 엔진 검증

물리 엔진이 이상하게 동작한다면:

```bash
cd rust
cargo test -- --nocapture
```

`--nocapture`를 붙이면 테스트 중 출력되는 숫자값도 볼 수 있습니다.

---

## 부록: 프로젝트 폴더 구조

```
aeroblend/
├── build.sh                 ← 빌드 스크립트 (이것만 실행하면 됨)
├── CMakeLists.txt           ← 빌드 설정 파일
├── GETTING_STARTED.md       ← 이 문서
├── README.md                ← 기술 문서 (개발자용)
│
├── rust/                    ← 물리 엔진 (Rust)
│   ├── aeroblend-math/      ←   수학 (벡터, 행렬, 쿼터니언)
│   ├── aeroblend-physics/   ←   물리 (대기, 양력, 엔진, 6DoF)
│   ├── aeroblend-importer/  ←   모델 로더 (glTF 파싱)
│   ├── aeroblend-core/      ←   C++와 공유하는 데이터 타입
│   └── aeroblend-ffi/       ←   C++에서 호출하는 함수들
│
├── cpp/                     ← 화면 (C++)
│   ├── src/                 ←   메인 루프, 렌더링, 카메라, HUD
│   ├── shaders/             ←   GPU 셰이더 (하늘, 조명 등)
│   └── third_party/         ←   외부 라이브러리 (glad, ImGui)
│
├── assets/
│   ├── models/              ← 비행기 모델 (.glb 파일)
│   └── fonts/               ← HUD 폰트
│
└── scripts/
    └── generate_example_aircraft.py  ← 예제 비행기 생성 스크립트
```

---

## 부록: 빠른 참조 카드

```
빌드:   ./build.sh
실행:   ./build/cpp/aeroblend --model <파일.glb>
테스트: cd rust && cargo test

W/S     스로틀      G  기어      V  카메라
마우스  방향 조준    F  플랩      H  HUD
A/D     롤          B  브레이크  ESC 종료
```
