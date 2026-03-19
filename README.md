# AeroBlend

Blender에서 만든 항공기 모델(.glb/.gltf)을 바로 임포트하여 조종할 수 있는 항공 물리 시뮬레이터.
ISA 대기 모델, 에어포일 양력/항력, 6자유도 강체 동역학을 실시간으로 계산하며, War Thunder 스타일의 마우스 에임 조종을 지원한다.

## 아키텍처

```
┌──────────────────────────────────────────────────┐
│  C++ Application (SDL2 + OpenGL 3.3 + Dear ImGui)│
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐  │
│  │ Renderer │ │  Input   │ │   HUD (ImGui)    │  │
│  │ Sky/Terr │ │  SDL2    │ │  9 instruments   │  │
│  │ Aircraft │ │ KB+Mouse │ │                  │  │
│  └────┬─────┘ └────┬─────┘ └────────┬─────────┘  │
│       │            │                │             │
│  ─────┴────────────┴────────────────┴──────────── │
│              FFI (C ABI, POD structs)             │
│  ────────────────────────────────────────────────  │
│  ┌──────────────────────────────────────────────┐ │
│  │  Rust Static Library (.a)                    │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────────┐ │ │
│  │  │ Physics  │ │ Importer │ │    Math      │ │ │
│  │  │ 6DoF RK4 │ │ glTF/glb │ │ Quat/Mat/Vec│ │ │
│  │  │ Aero/Atm │ │ Parts    │ │             │ │ │
│  │  └──────────┘ └──────────┘ └──────────────┘ │ │
│  └──────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────┘
```

| 계층 | 언어 | 역할 |
|------|------|------|
| **Physics / Math / Importer** | Rust | ISA 대기, CL/CD 에어포일, RK4 적분, 6DoF 엔진, glTF 파싱, 부품 분류 |
| **FFI** | Rust (`extern "C"`) | `#[repr(C)]` POD 구조체, cbindgen 자동 헤더 생성 |
| **Rendering / Input / HUD** | C++ | SDL2 윈도우, OpenGL 3.3 Core, Phong + Sky + Flat 셰이더, Dear ImGui |

## 빌드

### 의존성

| 도구 | 버전 |
|------|------|
| Rust (rustc + cargo) | 1.75+ |
| CMake | 3.20+ |
| SDL2 | 2.0+ |
| C++17 컴파일러 | Clang/GCC/MSVC |

macOS (Homebrew):
```bash
brew install cmake sdl2
```

### 빌드 명령

```bash
cd aeroblend-native
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(sysctl -n hw.ncpu)
```

또는 빌드 스크립트:
```bash
./build.sh
```

CMake가 [Corrosion](https://github.com/corrosion-rs/corrosion)을 통해 Rust를 자동 빌드하므로, 별도 `cargo build`는 필요 없다.

### 테스트 (Rust)

```bash
cd rust && cargo test
```

39개 유닛 테스트: 수학 10 + 물리 20 + 임포터 9

## 실행

### 빈 월드 (모델 없이)

```bash
./build/cpp/aeroblend
```

스카이 + 지형 + 활주로만 표시된다. 물리 시뮬레이션 없이 카메라와 HUD를 확인할 수 있다.

### 항공기 모델과 함께

```bash
./build/cpp/aeroblend --model /path/to/aircraft.glb
```

Blender에서 `.glb` 또는 `.gltf`로 export한 항공기 모델을 로드한다.
모델이 로드되면 물리 시뮬레이션이 시작되며, 키보드와 마우스로 조종 가능하다.

### Blender 모델 준비

1. Blender에서 항공기를 모델링한다
2. 메시 이름을 아래 규칙에 맞게 지정한다 (자동 부품 분류):
   - 날개: `Wing_Left`, `Wing_Right`, `wing_l`, `wing_r`
   - 미익: `H_Tail`, `V_Tail`, `stabilizer`, `rudder`
   - 엔진: `Jet_Engine`, `turbine`, `Propeller`, `blade`
   - 동체: `Fuselage`, `body`
   - 랜딩기어: `Gear`, `wheel`
3. **File > Export > glTF 2.0 (.glb/.gltf)** 로 export한다
4. 위 명령어로 실행한다

### 예시 항공기 생성

`assets/models/`에 테스트용 항공기가 없다면, Blender Python 스크립트로 바로 생성할 수 있다.
이 스크립트는 **Blender 애드온**으로도, **CLI 스탠드얼론**으로도 사용 가능하다.

#### 방법 1: CLI (headless)

```bash
blender --background --python scripts/generate_example_aircraft.py
```

`assets/models/example_aircraft.glb`가 기본 파라미터로 바로 생성된다.

#### 방법 2: Blender 애드온 설치

1. Blender 열기
2. **Edit > Preferences > Add-ons > Install from Disk**
3. `scripts/generate_example_aircraft.py` 선택
4. **"AeroBlend: Example Aircraft Generator"** 체크하여 활성화
5. 3D Viewport에서 **N키** → 사이드바 **AeroBlend** 탭
6. **Generate Aircraft** 버튼 클릭 → 파라미터 조정 다이얼로그
7. **Export Aircraft GLB** 버튼으로 파일 브라우저에서 내보내기

애드온에서 조절 가능한 파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| Airfoil Points | 40 | 에어포일 단면 해상도 |
| Span Sections | 10 | 날개 스팬 방향 분할 수 |
| Circ. Segments | 24 | 동체/나셀 원주 분할 수 |
| Wing Chord | 2.0m | 날개 시위 |
| Wing Half-Span | 5.0m | 날개 반 스팬 |
| Wing Taper Ratio | 0.5 | 날개 테이퍼비 |
| Fuselage Length | 12.0m | 동체 길이 |
| Fuselage Radius | 0.8m | 동체 반경 |

#### 생성되는 파트

NACA 에어포일 단면을 가진 8개 파트 (parts.rs 분류기와 매칭):

| 파트 이름 | 분류 | 형상 |
|-----------|------|------|
| `Wing_Left` / `Wing_Right` | wing_left / wing_right | NACA 2412, span 5m, chord 2m, taper 0.5 |
| `H_Tail_Left` / `H_Tail_Right` | h_tail | NACA 0009, span 2.5m, chord 1.2m |
| `Vertical_Stabilizer` | v_tail | NACA 0009, height 1.8m |
| `Fuselage` | fuselage | 테이퍼드 실린더, 길이 12m |
| `Engine_Left` / `Engine_Right` | engine | 나셀, 반경 0.25m |

생성 후 바로 실행:
```bash
./build/cpp/aeroblend --model assets/models/example_aircraft.glb
```

### 빠른 테스트

빌드부터 실행까지 한 줄로:
```bash
./build.sh && ./build/cpp/aeroblend
```

## 조종법 (War Thunder 스타일)

| 입력 | 동작 |
|------|------|
| **마우스 이동** | 에임 방향 → 자동 피치/롤/요 |
| **W / S** | 피치 다운 / 업 (키보드 오버라이드) |
| **Shift / Ctrl** | 스로틀 증가 / 감소 |
| **A / D** | 롤 좌 / 우 |
| **Q / E** | 요 좌 / 우 |
| **G** | 랜딩기어 토글 |
| **F** | 플랩 토글 |
| **B** | 브레이크 / 에어브레이크 |
| **V** | 카메라 모드 전환 (추적 / 조종석) |
| **H** | HUD 표시 토글 |
| **ESC** | 종료 |

## HUD 계기

- **Airspeed Indicator** — 속도 테이프 (knots)
- **Altimeter** — 고도 테이프 (feet)
- **Attitude Indicator** — 인공 수평의 (피치 래더 + 롤)
- **Heading Indicator** — 컴퍼스 테이프
- **Throttle Gauge** — 스로틀 바
- **G-Force Indicator** — G 하중 표시
- **Status Indicators** — 기어/플랩/브레이크 상태
- **Crosshair** — 에임 십자선
- **FPS Counter** — 프레임 레이트

## 물리 엔진

- **대기 모델**: ISA(International Standard Atmosphere), 대류권 + 성층권
- **공기역학**: 에어포일 CL/CD, 실속 모델, 유도항력(Oswald), 에어브레이크
- **적분**: RK4 (4차 Runge-Kutta), 120Hz 고정 타임스텝
- **6DoF**: 쿼터니언 기반 자세 표현 (짐벌락 없음)
- **지면 충돌**: 고도 0 이하 시 속도 리셋

### glTF 부품 분류

모델 로딩 시 메시 이름을 정규식으로 자동 분류하여 물리 파라미터를 추출한다:

| 패턴 | 분류 | 역할 |
|------|------|------|
| `Wing_Left`, `wing_r` | wing_left/right | 양력면 (AeroSurface) |
| `H_Tail`, `stabilizer` | h_tail | 수평 미익 |
| `V_Tail`, `rudder` | v_tail | 수직 미익 |
| `Jet_Engine`, `turbine` | engine | 제트 추력 (EngineSpec) |
| `Propeller`, `blade` | propeller | 프로펠러 추력 |
| `Fuselage`, `body` | fuselage | 동체 |
| `Gear`, `wheel` | gear | 랜딩기어 |

## 프로젝트 구조

```
aeroblend-native/
├── CMakeLists.txt                  # 최상위 CMake (Corrosion + SDL2 + OpenGL)
├── build.sh                        # 빌드 스크립트
├── rust/
│   ├── Cargo.toml                  # Workspace
│   ├── aeroblend-math/             # glam 래핑, 쿼터니언, 행렬 (345 LOC)
│   ├── aeroblend-physics/          # 대기, 에어포일, RK4, 6DoF 엔진 (1431 LOC)
│   ├── aeroblend-importer/         # glTF 로더, 부품 분류 (480 LOC)
│   ├── aeroblend-core/             # #[repr(C)] FFI 타입 (287 LOC)
│   └── aeroblend-ffi/              # extern "C" 함수 + cbindgen (312 LOC)
└── cpp/
    ├── CMakeLists.txt
    ├── include/aeroblend/
    │   ├── ffi.h                   # cbindgen 자동 생성
    │   ├── window.h, shader.h, gpu_mesh.h
    │   ├── renderer.h, sky.h, terrain.h
    │   ├── camera.h, input.h, hud.h
    ├── src/                        # C++ 구현 (1213 LOC)
    │   ├── main.cpp                # 게임 루프 (120Hz 물리 + 가변 렌더)
    │   ├── renderer.cpp            # 씬 오케스트레이션
    │   ├── sky.cpp, terrain.cpp    # 환경 렌더링
    │   ├── camera.cpp              # 추적/조종석 카메라
    │   ├── input.cpp               # 마우스 에임 + 키보드 스무딩
    │   └── hud.cpp                 # ImGui 9개 계기
    ├── shaders/                    # GLSL 3.30 (phong, sky, flat)
    └── third_party/                # glad (GL 3.3), Dear ImGui (v1.91.8)
```

## FFI 경계

Rust가 `extern "C"` 함수를 export하고, cbindgen이 `ffi.h`를 자동 생성한다.

```c
// 라이프사이클
PhysicsEngine* aeroblend_physics_create(void);
void            aeroblend_physics_destroy(PhysicsEngine*);
LoadedModel*    aeroblend_model_load(const char* path);
void            aeroblend_model_destroy(LoadedModel*);

// 시뮬레이션
AircraftPhysicsStateC aeroblend_physics_init(PhysicsEngine*, const LoadedModel*);
AircraftPhysicsStateC aeroblend_physics_step(PhysicsEngine*, const ControlStateC*, double dt);

// 카메라 + 행렬
CameraStateC aeroblend_camera_update_chase(const AircraftPhysicsStateC*, const CameraStateC*, double dt);
CameraStateC aeroblend_camera_update_cockpit(const AircraftPhysicsStateC*);
Mat4C        aeroblend_build_view_matrix(const CameraStateC*);
Mat4C        aeroblend_build_projection_matrix(const CameraStateC*, double aspect);
Mat4C        aeroblend_build_model_matrix(const Mat3C*, const Vec3C*);
```

소유권: Rust가 PhysicsEngine/LoadedModel/메시 버퍼를 소유하고, C++은 GL 리소스(VAO/VBO/셰이더)와 SDL 윈도우를 소유한다.

## 라이선스

Private project.
