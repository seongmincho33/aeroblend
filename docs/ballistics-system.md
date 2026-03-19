# AeroBlend 탄도학 시스템 기술서

> 이 문서는 AeroBlend의 기총/탄도학(Ballistics) 시스템의 **아키텍처, 물리 모델, 구현 세부사항**을 정의한다.
> 탄환 비행 시뮬레이션, 기총 메커닉, 렌더링 파이프라인의 기준 문서(baseline reference)이다.

---

## 1. 개요

### 1.1 시스템 구조

탄도학 시스템은 기존 AeroBlend 아키텍처와 동일한 Rust + C++ + FFI 구조를 따른다:

| 계층 | 역할 | 위치 |
|------|------|------|
| **Rust** (`aeroblend-ballistics`) | 탄도 물리, 기총 메커닉, 탄환 풀 관리 | `rust/aeroblend-ballistics/` |
| **FFI** (`aeroblend-core`/`aeroblend-ffi`) | `#[repr(C)]` POD 타입, `extern "C"` 함수 | `rust/aeroblend-core/`, `rust/aeroblend-ffi/` |
| **C++** (SDL2/OpenGL) | 트레이서 렌더링, 발사 입력 처리, HUD 표시 | `cpp/src/` |

### 1.2 데이터 흐름

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         Game Loop (main.cpp)                            │
│                                                                         │
│  SDL Event ──→ InputHandler ──→ WeaponSystem::update()                  │
│  (마우스 좌클릭)       │               │                                │
│                       │               ▼                                 │
│                       │        GunStateC (from Rust FFI)                │
│                       │        ┌──────────────────┐                     │
│                       │        │ fire_rate         │                    │
│                       │        │ ammo_remaining    │                    │
│                       │        │ muzzle_velocity   │                    │
│                       │        │ spread_rad        │                    │
│                       │        └──────────────────┘                     │
│                       │               │                                 │
│                       │               ▼                                 │
│                       │        Projectile::step() ← Rust Physics       │
│                       │        (중력 + 공기저항 + 분산)                 │
│                       │               │                                 │
│                       │               ▼                                 │
│                       │        ProjectileRenderer                       │
│                       │        (GL_LINES 트레이서)                      │
│                       │               │                                 │
│                       ▼               ▼                                 │
│                  ControlStateC   HUD (잔탄수, 조준점)                   │
└──────────────────────────────────────────────────────────────────────────┘
```

**요약**: 마우스 좌클릭 → `InputHandler` → `WeaponSystem::update()` → `Projectile::step()` → `ProjectileRenderer`

---

## 2. 탄도학 수학 모델

### 2.1 외탄도학 (External Ballistics)

탄환 비행 중 작용하는 힘은 두 가지이다:

**중력**:

```
F_g = (0, -mg, 0)
```

**공기 저항** (항력):

```
F_drag = -0.5 × ρ(h) × |v|² × C_d × A × v̂
```

여기서:
- `ρ(h)` — ISA 대기 밀도 (고도 `h`에서), 단위: kg/m³
- `C_d` — 항력계수 (탄종별 상수, 무차원)
- `A` — 탄환 단면적 = `π(d/2)²`, 단위: m²
- `v̂` — 속도 단위벡터 = `v / |v|`
- `m` — 탄환 질량, 단위: kg
- `g` — 중력가속도 = 9.80665 m/s²

**합력과 가속도**:

```
F_total = F_g + F_drag
a = F_total / m
```

### 2.2 ISA 대기 모델

기존 `aeroblend-physics`의 ISA 대기 모델을 공유한다.

**대류권** (h < 11,000m):

```
T(h) = T₀ + L × h = 288.15 - 0.0065 × h  [K]
ρ(h) = ρ₀ × (T(h)/T₀)^(g₀/(L×R) - 1)
```

**간략화 공식** (계산 효율):

```
ρ(h) ≈ 1.225 × (1 - 2.26×10⁻⁵ × h)^4.256  [kg/m³]
```

여기서:
- `T₀` = 288.15 K (해수면 기온)
- `ρ₀` = 1.225 kg/m³ (해수면 밀도)
- `L` = -0.0065 K/m (기온감률)
- `g₀` = 9.80665 m/s²
- `R` = 287.058 J/(kg·K) (건조 공기 기체상수)

> **구현 참조**: `aeroblend-physics/src/atmosphere.rs`의 `isa_density(altitude)` 함수를 직접 호출한다. 중복 구현하지 않는다.

### 2.3 적분법

**탄환: Euler 적분** (120Hz에서 충분한 정확도)

```
v(t+dt) = v(t) + a(t) × dt
x(t+dt) = x(t) + v(t) × dt
```

탄환에 Euler 적분을 사용하는 이유:
1. 탄환은 **짧은 생존시간** (최대 5초)으로 오차 누적이 제한적
2. **강체 회전 없음** — 질점(point mass) 모델이므로 자세 불안정 문제 없음
3. 동시 최대 512발 처리 시 **계산 효율**이 중요

**항공기: RK4 적분** (6DoF 안정성 요구)

```
항공기는 기존 aeroblend-physics의 RK4 적분기를 사용한다.
탄환과 항공기의 물리 갱신은 동일한 120Hz 타임스텝에서 동기 실행된다.
```

### 2.4 산탄 분포 (Dispersion)

총구 이탈 시 탄환 방향에 **가우시안 편차**를 적용하여 현실적인 분산을 모사한다.

**수학 모델**:

```
σ = spread_rad          (기총별 상수, 단위: rad)
θ ~ N(0, σ²)            (편향각 — 정규분포)
φ ~ U(0, 2π)            (방위각 — 균등분포)
```

**적용 방법**:

```
1. 기총 방향 벡터 d를 기준으로 직교 좌표계 (d, u, r) 구성
2. 편향벡터 = u × sin(θ) × cos(φ) + r × sin(θ) × sin(φ)
3. 최종 탄환 방향 = normalize(d + 편향벡터)
```

여기서 `u`, `r`은 기총 방향 `d`에 수직인 단위벡터 쌍이다.

---

## 3. 기총 스펙

### 3.1 탄종별 파라미터

| 파라미터 | .50 cal M2 | 20mm AN/M2 | 7.7mm Vickers |
|---------|-----------|------------|--------------|
| 구경 (caliber) | 12.7mm | 20mm | 7.7mm |
| 탄환 질량 (mass) | 45g | 130g | 11g |
| 초구속도 (muzzle velocity) | 890 m/s | 840 m/s | 745 m/s |
| 항력계수 (C_d) | 0.295 | 0.32 | 0.30 |
| 단면적 (A) | 1.27×10⁻⁴ m² | 3.14×10⁻⁴ m² | 4.66×10⁻⁵ m² |
| 발사율 (fire rate) | 750 RPM | 600 RPM | 1000 RPM |
| 장탄수 (ammo capacity) | 500 | 200 | 600 |
| 산탄각 (dispersion) | ±3 mrad | ±5 mrad | ±4 mrad |

### 3.2 파라미터 설명

- **초구속도**: 탄환이 총구를 이탈하는 순간의 속도. 항공기 속도와 벡터 합산된다.
- **항력계수**: 탄환 형상에 의한 공기 저항 상수. 값이 클수록 속도 감쇠가 빠르다.
- **단면적**: `A = π(d/2)²` 로 계산. 항력 크기에 직접 비례한다.
- **발사율**: 분당 발수(RPM). 탄환 간 최소 간격 = `60 / fire_rate` 초.
- **산탄각**: 총구 이탈 시 편향 범위. mrad = 밀리라디안 (1 mrad ≈ 1m 편향 at 1km).

### 3.3 탄환 초기 속도 계산

탄환은 항공기의 속도에 총구 속도를 더한 값으로 발사된다:

```
v_projectile = v_aircraft + muzzle_velocity × gun_direction
```

여기서 `gun_direction`은 기총 장착 방향(항공기 기체 좌표 → 월드 좌표 변환 후)이다.

---

## 4. 시스템 제약

### 4.1 성능 제한

| 제약 | 값 | 근거 |
|------|-----|------|
| 최대 동시 탄환 | 512발 | 프레임당 512 × Euler step 비용 허용 범위 |
| 탄환 생존시간 | 5.0초 | 최대 사거리 ≈ 890 × 5 = 4,450m (.50 cal 기준, 공기저항 미포함) |
| 물리 갱신 주기 | 120Hz | 항공기 물리와 동기 (dt = 1/120 ≈ 0.00833초) |
| 탄환 풀 방식 | 고정 배열 (pre-allocated) | 동적 할당 없음, 캐시 친화적 |

**탄환 풀 관리**:

```
탄환 배열 [512] — 고정 크기
├── active = true:  물리 갱신 + 렌더링 대상
└── active = false: 재사용 가능 슬롯

발사 시: 비활성 슬롯 검색 → 활성화 (없으면 발사 무시)
제거 조건: 생존시간 초과 OR 지면 충돌 (y < 0)
```

### 4.2 렌더링

**트레이서 렌더링**:

| 항목 | 값 |
|------|-----|
| 프리미티브 | `GL_LINES` |
| 트레이서 길이 | 속도 × 0.02초 |
| 트레이서 색상 | `#FFAA00` (노란/주황) |
| 배치 처리 | 단일 draw call (활성 탄환 전체) |

**구현 방식**:

```
1. 활성 탄환의 (position, position - velocity × 0.02) 쌍을 정점 버퍼에 적재
2. 단일 glDrawArrays(GL_LINES, 0, active_count × 2) 호출
3. flat 셰이더 사용 (조명 불필요)
```

> **성능 참조**: 512발 × 2 정점 × 3 float = 12,288 bytes. 매 프레임 업로드해도 GPU 부담 무시 가능.

### 4.3 입력 처리

| 입력 | 동작 |
|------|------|
| 마우스 좌클릭 (누름) | 발사 시작 |
| 마우스 좌클릭 (해제) | 발사 중지 |
| 잔탄 0 | 자동 발사 중지, HUD에 "AMMO DEPLETED" 표시 |

발사 간격은 `fire_interval = 60.0 / fire_rate` 초로 계산하며, `time_since_last_shot >= fire_interval` 일 때 탄환을 생성한다.

---

## 5. FFI 인터페이스

### 5.1 C 호환 타입 (`aeroblend-core`)

```rust
#[repr(C)]
pub struct ProjectileC {
    pub position: [f64; 3],     // 월드 좌표 (m)
    pub velocity: [f64; 3],     // 속도 벡터 (m/s)
    pub active: bool,           // 활성 여부
    pub time_alive: f64,        // 생존 시간 (초)
}

#[repr(C)]
pub struct GunSpecC {
    pub caliber_mm: f64,        // 구경 (mm)
    pub mass_kg: f64,           // 탄환 질량 (kg)
    pub muzzle_velocity: f64,   // 초구속도 (m/s)
    pub drag_coeff: f64,        // 항력계수 (무차원)
    pub cross_section: f64,     // 단면적 (m²)
    pub fire_rate_rpm: f64,     // 발사율 (RPM)
    pub ammo_capacity: i32,     // 장탄수
    pub spread_rad: f64,        // 산탄각 (rad)
}

#[repr(C)]
pub struct GunStateC {
    pub ammo_remaining: i32,    // 잔탄수
    pub time_since_last_shot: f64, // 마지막 발사 후 경과 시간 (초)
    pub is_firing: bool,        // 발사 중 여부
}
```

### 5.2 FFI 함수 (`aeroblend-ffi`)

```c
// 탄도 시스템 생성/소멸
BallisticsHandle* aeroblend_ballistics_create(const GunSpecC* spec);
void aeroblend_ballistics_destroy(BallisticsHandle* handle);

// 탄환 발사
void aeroblend_ballistics_fire(
    BallisticsHandle* handle,
    const double position[3],    // 총구 위치 (월드 좌표)
    const double direction[3],   // 발사 방향 (단위벡터)
    const double aircraft_vel[3] // 항공기 속도 (벡터 합산용)
);

// 물리 갱신 (120Hz)
void aeroblend_ballistics_step(BallisticsHandle* handle, double dt);

// 상태 조회
GunStateC aeroblend_ballistics_get_state(const BallisticsHandle* handle);
int aeroblend_ballistics_get_active_projectiles(
    const BallisticsHandle* handle,
    ProjectileC* out_buffer,     // 호출자 제공 버퍼
    int buffer_size              // 버퍼 크기
);
```

---

## 6. 향후 계획

### 6.1 피격 판정 (Collision Detection)

- **AABB/OBB 충돌**: 탄환 위치 vs 항공기 메시의 바운딩 박스
- **관통 판정**: 운동 에너지 `E_k = 0.5 × m × |v|²` vs 장갑 두께로 관통 여부 결정
- **레이캐스트**: 탄환의 이전 위치 → 현재 위치 구간으로 빠른 탄환도 검출

### 6.2 손상 모델 (Damage Model)

- **서피스별 HP 시스템**: glTF 메시의 파트 분류(`aeroblend-importer`)를 활용
- **부위별 효과**:

| 부위 | 피격 효과 |
|------|----------|
| 날개 (wing) | 양력 감소, 롤 비대칭 |
| 엔진 (engine) | 추력 감소, 화재 위험 |
| 조종면 (control surface) | 조종력 감소, 응답성 저하 |
| 동체 (fuselage) | HP 감소 |
| 꼬리 (tail) | 요/피치 안정성 감소 |

- **화재/폭발 이벤트**: 엔진 피격 + 연료 탱크 근접 시 확률적 화재 발생

### 6.3 확장 무장

| 무장 유형 | 물리 모델 | 우선순위 |
|----------|----------|---------|
| 로켓 (비유도) | 추력 + 항력 + 중력 | 높음 |
| 폭탄 (자유낙하) | 항력 + 중력 (추력 없음) | 높음 |
| 미사일 (열추적) | 추력 + 항력 + 비례항법 유도 | 중간 |
| 미사일 (레이더) | 추력 + 항력 + 중점유도 | 낮음 |

- **다중 무장 슬롯**: 항공기별 무장 장착점(hardpoint) 정의
- **무장 선택 시스템**: 숫자키(1-4)로 무장 전환, HUD에 현재 선택 무장 표시

### 6.4 시각 효과

| 효과 | 구현 방법 |
|------|----------|
| 총구 화염 (Muzzle flash) | 빌보드 스프라이트, 발사 시 2-3프레임 표시 |
| 피탄 효과 (Hit sparks) | 파티클 시스템, 충돌 지점에서 방출 |
| 연기 트레일 (Smoke trail) | 트레이서 뒤 알파 감쇠 파티클 |
| 탄흔 (Bullet holes) | 데칼 텍스처 (향후 고려) |

---

## 부록: 용어 정리

| 용어 | 영문 | 설명 |
|------|------|------|
| 초구속도 | Muzzle velocity | 탄환이 총구를 떠나는 순간의 속도 |
| 항력계수 | Drag coefficient (C_d) | 형상에 의한 공기 저항 상수 |
| 산탄각 | Dispersion angle | 탄환 방향의 랜덤 편향 범위 |
| 외탄도학 | External ballistics | 총구 이탈 후 탄환의 비행 역학 |
| 발사율 | Rate of fire (RPM) | 분당 발사 가능 탄수 |
| 트레이서 | Tracer | 탄환 궤적을 시각적으로 표시하는 렌더링 요소 |
| ISA | International Standard Atmosphere | 국제 표준 대기 모델 |
| Euler 적분 | Euler integration | 1차 수치 적분법, 계산 효율 우수 |
| RK4 | Runge-Kutta 4th order | 4차 수치 적분법, 정확도 우수 |
| 탄환 풀 | Projectile pool | 사전 할당된 고정 크기 탄환 배열 |
