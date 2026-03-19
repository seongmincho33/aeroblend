# AeroBlend

Flight physics simulator: Blender glTF models → real-time aerodynamics with War Thunder-style mouse aim.

## Architecture

- **Rust** (static library): physics, math, glTF importer, FFI boundary
- **C++17** (SDL2 + OpenGL 3.3 + Dear ImGui): rendering, input, HUD
- **FFI**: `extern "C"` functions with `#[repr(C)]` POD structs, cbindgen auto-generates `cpp/include/aeroblend/ffi.h`

## Build

```bash
./build.sh                    # or: cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
```

Dependencies: Rust 1.75+, CMake 3.20+, SDL2, C++17 compiler.
CMake uses Corrosion to auto-build Rust via Cargo — no separate `cargo build` needed.

## Test

```bash
cd rust && cargo test          # Rust unit tests (math, physics, importer)
cd build && ctest              # C++ integration tests (mouse flight)
```

## Run

```bash
./build/cpp/aeroblend --model path/to/aircraft.glb
```

## Project Layout

```
rust/
  aeroblend-math/       # glam wrappers, quaternion, matrix (10 tests)
  aeroblend-physics/    # ISA atmosphere, airfoil CL/CD, RK4, 6DoF engine
  aeroblend-importer/   # glTF loader, regex part classification (9 tests)
  aeroblend-core/       # #[repr(C)] POD types for FFI
  aeroblend-ffi/        # extern "C" exports, cbindgen header gen
cpp/
  src/                  # main loop, renderer, sky, terrain, camera, input, hud
  include/aeroblend/    # ffi.h (auto-gen), C++ headers
  shaders/              # GLSL 3.30 (phong, sky, flat)
  third_party/          # glad2 (GL 3.3), Dear ImGui v1.91.8
  tests/                # mouse flight integration tests
scripts/                # Blender aircraft generator addon
assets/                 # models, fonts
```

## Code Conventions

- Language: Korean comments and docs are used throughout. Follow existing style.
- Rust: edition 2021, workspace with shared dependencies in root Cargo.toml
- C++: C++17, headers in `cpp/include/aeroblend/`, sources in `cpp/src/`
- FFI types: all in `aeroblend-core` with `#[repr(C)]`, cbindgen config in `aeroblend-ffi/cbindgen.toml`
- Shaders: GLSL 3.30 core profile

## Technical Documents

- `docs/mouse-aim-system.md` — 마우스 에임 조종 시스템 기술서 (인스트럭터, PD 제어, 적응형 게인 스케줄링, 비행 보호, 3-body convergence)
- `docs/mouse-aim-math-formulas.md` — 마우스 에임 수학 공식집 (20개 섹션, 1634줄: 전달함수, 감쇠비, 최적 게인, 안정성 증명, 협조선회, 에너지 관리, 실속 보호, 항력 극선, 개선 권고 P0-P3)

## Key Technical Details

- Physics runs at 120Hz fixed timestep (RK4), rendering at variable rate
- Quaternion-based attitude (no gimbal lock)
- glad2 (not glad1): `gladLoadGL((GLADloadfunc)SDL_GL_GetProcAddress)`
- Each Rust crate using `glam::DVec3` etc. must have glam as a direct dependency
- cbindgen needs `with_parse_include(&["aeroblend-core"])` to resolve shared types
- macOS linking requires `-framework CoreFoundation -framework Security`
- Regex pattern order matters in `aeroblend-importer/src/parts.rs` (v_tail before h_tail)
- Engine thrust scaled to T/W=0.35 in importer (`lib.rs`)
