# AeroBlend 마우스 에임 시스템 기술서

> 이 문서는 AeroBlend의 마우스 에임(Mouse Aim) 조종 방식의 **설계 원리와 구현 구조**를 정의한다.
> 향후 개선 작업의 기준 문서(baseline reference)이며, "마우스 에임 조종이란 무엇인가"에 대한 정의서이다.

---

## 1. 마우스 에임 조종이란

### 1.1 개념

마우스 에임(Mouse Aim)은 **플레이어가 마우스로 "가고 싶은 방향"을 화면에 가리키면, 소프트웨어 인스트럭터(instructor)가 자동으로 조종면(pitch/roll/yaw)을 계산하여 항공기를 그 방향으로 조향하는** 간접 조종 방식이다.

핵심 아이디어: **플레이어는 "어디로 갈지"만 결정하고, "어떻게 갈지"는 인스트럭터가 결정한다.**

이 방식은 War Thunder, World of Warplanes 등의 항공 게임에서 대중화되었으며, 항공역학 지식이 없는 사용자도 직관적으로 비행기를 조종할 수 있게 해준다.

### 1.2 직접 조종과의 차이

| 구분 | 직접 조종 (Keyboard/Joystick) | 마우스 에임 (Mouse Aim) |
|------|-------------------------------|------------------------|
| 입력 의미 | "엘리베이터를 위로 당겨라" | "저 방향으로 가라" |
| 제어 대상 | 조종면 편향각 (deflection) | 목표 방향 (direction) |
| 축 매핑 | 1 입력 → 1 축 직접 대응 | 1 입력 → 3축 자동 조합 |
| 협조선회 | 플레이어가 직접 수행 | 인스트럭터가 자동 수행 |
| 실속 방지 | 플레이어 책임 | 인스트럭터가 제한 |
| 스로틀 | 플레이어 수동 | 오토스로틀 보조 가능 |

### 1.3 시스템의 3가지 핵심 역할

1. **목표 방향 추적 (Target Tracking)**: 마우스 입력을 월드 공간의 목표 방향으로 누적
2. **비행 제어 계산 (Flight Control Computation)**: 목표 방향과 현재 방향의 오차를 PD 제어기로 조종면 명령으로 변환
3. **비행 보호 (Flight Protection)**: 실속, 과도한 받음각(AoA), 저속 등 위험 상황에서 자동 보호

---

## 2. 시스템 아키텍처

### 2.1 전체 데이터 흐름

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Game Loop (main.cpp)                         │
│                                                                        │
│  SDL Event ──→ InputHandler ──→ ControlStateC ──→ Physics Engine(Rust) │
│   (mouse dx,dy)    │                                    │              │
│                    │                                    │              │
│              MouseAimComputer                           │              │
│              ┌──────────────┐                           ▼              │
│              │ target_yaw   │              AircraftPhysicsStateC       │
│              │ target_pitch │◄────────── (position, velocity,         │
│              │              │             orientation, airspeed,       │
│              │ compute_     │             aoa, angular_velocity,      │
│              │ instructor() │             flight_chars)               │
│              │   │          │                           │              │
│              │   ▼          │                           │              │
│              │ pitch/roll/  │                           │              │
│              │ yaw commands │                           │              │
│              └──────────────┘                           │              │
│                    │                                    │              │
│                    ▼                                    ▼              │
│              ControlStateC                         Camera             │
│              (pitch, roll, yaw,              (cam_yaw, cam_pitch      │
│               throttle, ...)                  aim display)            │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 세 가지 독립 추적 객체 (3-Body Convergence)

마우스 에임 시스템에는 **서로 다른 속도로 움직이는 세 가지 방향**이 존재한다:

```
                  응답 속도
   마우스 커서     즉시        ──→  target_yaw / target_pitch
       │
       ▼
   카메라 방향     빠름 (6.0)  ──→  cam_yaw / cam_pitch
       │
       ▼
   항공기 방향     느림 (물리)  ──→  orientation matrix
```

| 객체 | 속도 | 역할 | 위치 |
|------|------|------|------|
| **마우스 타겟** | 즉시 반응 | 플레이어 의도 표현 (에임 원) | `MouseAimComputer::target_yaw/pitch` |
| **카메라** | 지수 추적 (rate=6.0) | 부드러운 시점 전환 | `Camera::cam_yaw_/cam_pitch_` |
| **항공기** | 물리 시뮬레이션 | 기총 조준선 (gunsight) | `AircraftPhysicsStateC::orientation` |

이 3단계 수렴 구조가 마우스 에임의 "느낌"을 결정한다:
- 카메라는 마우스보다 느리게 따라가므로, **마우스를 움직이면 에임 원이 화면 안에서 이동**한다.
- 항공기는 카메라보다 느리게 따라가므로, **기총 조준선(초록 십자선)이 에임 원을 향해 수렴**한다.
- 세 개가 일치하면 안정 비행(사격 가능), 벌어지면 기동 중(수렴선이 길다)임을 직관적으로 알 수 있다.

### 2.3 HUD 표시: 3개의 마커

```
         ○ ← 에임 원 (마우스 타겟 방향 — 하얀색)
        ╱
       ╱  ← 수렴선 (추적 오차 시각화 — 흰색 반투명)
      ╱
     ＋ ← 기총 조준선 (gunsight — 초록색)
      \
       ◉ ← 속도벡터 (실제 비행경로 — 시안색)
```

| 마커 | 의미 | 색상 | 소스 |
|------|------|------|------|
| **에임 원 (Circle)** | 플레이어가 마우스로 가리키는 목표 방향 | 하얀색 | `mouse_target_yaw/pitch - cam_yaw/pitch` |
| **기총 조준선 (Gunsight)** | 항공기의 기총선 — 기총을 발사하면 이 방향으로 날아감 | 초록색 | `orientation forward vector - cam direction` |
| **속도벡터 (Velocity)** | 항공기가 실제로 이동하는 방향 (비행 경로) | 시안색 | `velocity vector - cam direction` |

**기총 조준선의 핵심**: 초록색 십자선은 항공기의 **기총선(gun line)**이다. 비행기가 기총을 쏘면 이 십자선 방향으로 탄환이 날아간다. 인스트럭터가 항공기를 에임 원 방향으로 조향하므로, **기총 조준선은 하얀 에임 원을 향해 수렴**한다.

**수렴선(Convergence Line)**: 기총 조준선과 에임 원을 잇는 흰색 반투명 선이다. 이 선의 길이가 곧 **추적 오차**를 나타낸다:
- **선이 짧다** = 항공기가 목표 방향을 잘 추적 중 → 기총 사격 정확도 높음
- **선이 길다** = 기동 중이거나 수렴하는 과정 → 사격 부정확

플레이어가 마우스를 움직인 후 **가만히 있으면**, 인스트럭터가 항공기를 에임 원 방향으로 수렴시키고, 기총 조준선이 에임 원에 도달하면서 **수렴선 길이가 0에 수렴**한다. 이것이 "조준이 완료되었다"는 시각적 피드백이다.

이 3개 마커의 위치 관계로 비행 상태를 읽을 수 있다:
- 에임 원 ≈ 기총 조준선 ≈ 속도벡터: **안정 직진 비행, 사격 가능**
- 에임 원 ≠ 기총 조준선: **기동 중** (인스트럭터가 기총선을 에임 원 쪽으로 수렴시키는 중)
- 기총 조준선 ≠ 속도벡터: **사이드슬립 또는 고받음각** (기총이 가리키는 곳과 실제 이동 방향이 다름)

---

## 3. 마우스 입력 처리

### 3.1 마우스 → 목표 방향 변환

```
파일: cpp/src/input.cpp — MouseAimComputer::update_from_mouse()
```

마우스의 상대 이동량(dx, dy)을 **월드 공간 절대 각도**로 누적한다:

```
target_yaw   += dx * sensitivity     (좌우 → 요 각도)
target_pitch -= dy * sensitivity     (위아래 → 피치 각도, 반전)
target_pitch  = clamp(target_pitch, -1.2, 1.2)   (±70° 제한)
```

핵심 설계 결정:
- **sensitivity = 0.005 rad/px**: 마우스 1픽셀 = 0.29° 회전
- **절대 각도 누적**: 마우스를 멈추면 목표 방향이 유지됨 (FPS 게임의 에임과 동일)
- **피치 제한 ±70°**: 수직 루프를 방지하여 비현실적 기동 억제
- **y축 반전**: 마우스 위 = 피치 업 (자연스러운 매핑)

### 3.2 초기화: 현재 방향으로 시작

```
파일: cpp/src/input.cpp — MouseAimComputer::compute_instructor() 초기화 블록
```

첫 프레임에서 항공기의 현재 forward 벡터로부터 target_yaw/pitch를 역계산한다:

```
target_yaw   = atan2(forward.x, forward.z)
target_pitch = asin(forward.y) + 0.08      (← 트림 오프셋 ~5°)
```

**트림 오프셋 (0.08 rad ≈ 5°)**: 수평 비행에는 약간의 양의 받음각이 필요하다.
초기 마우스 위치가 수평 비행을 유지하는 방향을 가리키도록 약간 위로 보정한다.

---

## 4. 인스트럭터 (Instructor)

인스트럭터는 마우스 에임 시스템의 핵심이다. 목표 방향과 현재 항공기 방향 사이의 오차를 계산하고, PD 제어기로 조종면 명령(-1.0~+1.0)을 생성한다.

```
파일: cpp/src/input.cpp — MouseAimComputer::compute_instructor()
```

### 4.1 오차 계산: 월드 → 로컬 변환

목표 방향 벡터를 항공기 로컬 좌표계로 변환하여 각도 오차를 구한다:

```
1) 목표 방향 벡터 (월드)
   tx = sin(target_yaw) * cos(target_pitch)
   ty = sin(target_pitch)
   tz = cos(target_yaw) * cos(target_pitch)

2) 항공기 로컬 좌표로 변환
   lx = dot(target, right)      ← 좌우 오차
   ly = dot(target, up)         ← 상하 오차
   lz = dot(target, forward)    ← 전후 (양수면 앞쪽)

3) 각도 오차 추출
   yaw_err   = atan2(lx, lz)                        ← 좌우 각도 오차
   pitch_err = atan2(ly, sqrt(lx² + lz²))           ← 상하 각도 오차
```

이 방식의 장점:
- 짐벌락 없음 (오일러 각이 아니라 벡터 내적 기반)
- 모든 자세에서 정확한 오차 계산
- 자연스럽게 최단 경로 선택

### 4.2 PD 제어기

**비례-미분(PD) 제어기**로 조종면 명령을 생성한다:

```
pitch = clamp( Kp_pitch * pitch_err  -  D_filtered_pitch,  -1, 1)
roll  = clamp( Kp_roll  * bank_err   -  D_filtered_roll,   -1, 1)
yaw   = clamp( Kp_yaw   * yaw_err    -  D_filtered_yaw,    -1, 1)
```

| 항 | 역할 | 효과 |
|----|------|------|
| **Kp × error** (비례항) | 오차에 비례한 조종 | 오차가 크면 세게, 작으면 약하게 |
| **Kd × ω** (미분항) | 각속도에 비례한 감쇠 | 진동/오버슈트 억제 |

#### D-term 저역통과 필터 (P2-7)

미분항에 1차 저역통과 필터(LPF)를 적용하여 고주파 노이즈(60Hz 시뮬레이션)를 억제한다:

```
τ = 0.01s  (필터 시간 상수)
α_f = 1 - exp(-dt / τ)

D_filtered += α_f × (Kd × ω - D_filtered)
```

효과: 60Hz 노이즈를 ~75% 감쇠하면서 실제 각속도 변화는 충실히 추적. 제어 응답에 0.01초 미만의 지연만 추가.

기본 게인값:

| 축 | Kp (비례) | Kd (미분) | P/D 비율 | 목표 감쇠비 |
|----|-----------|-----------|----------|------------|
| Pitch | 2.5 | 1.52 | 1.64 | ζ ≈ 0.7 |
| Roll | 3.0 | 0.8 | 3.75 | - |
| Yaw | 0.5 | 0.4 | 1.25 | - |

> **Kd 피치 변경 이력 (P2-1)**: 기존 1.2에서 1.52로 상향. 감쇠비 ζ=0.47(오버슈트 19.5%)에서 ζ=0.7(오버슈트 4.6%)로 개선하여 최적 감쇠 달성.

설계 의도:
- **롤이 가장 공격적** (P=3.0): 선회 진입을 빠르게
- **피치는 중간** (P=2.5): 안정적 상승/하강, 최적 감쇠비로 오버슈트 최소화
- **요는 보수적** (P=0.5): 과도한 사이드슬립 방지

### 4.3 협조선회 (Coordinated Turn)

#### 물리 기반 뱅크 각도 계산 (P1-2)

요 오차로부터 물리적으로 정확한 뱅크 각도를 계산한다. 기존의 단순 비례(`yaw_err × 1.5`)를 대체하여, 실제 선회 역학에 기반한 공식을 사용한다:

```
K_turn_rate = 0.268                          (표준 선회율 상수)
ψ̇_desired  = K_turn_rate × yaw_err          (목표 선회율 [rad/s])
desired_bank = atan(ψ̇_desired × V / g)      (물리적 뱅크 각도)
desired_bank = clamp(desired_bank, -max_bank, max_bank)
current_bank = atan2(-right.y, up.y)
bank_err     = desired_bank - current_bank
```

K_turn_rate = 0.268의 의미: 1 라디안(57°) 요 오차에서 15.4°/s 선회율을 생성. 이는 표준 선회(2분에 360°)와 일치한다.

이 공식의 장점:
1. 속도에 비례하여 뱅크 각도가 자동 조절됨 (고속→큰 뱅크, 저속→작은 뱅크)
2. `atan()` 함수가 자연스러운 포화(saturation) 제공 — 극단적 입력에도 안정적
3. 양력 수평 성분 = 원심력 조건을 정확히 충족하는 조화 선회

**max_bank**는 속도에 따라 제한된다:
```
max_bank = 60° × clamp(speed_ratio, 0.3, 1.0)
```
- 고속: 최대 60° 뱅크 (급선회 가능)
- 저속: 18° 뱅크 제한 (에너지 보존)

#### 사이드슬립 보정 (P1-4)

협조선회 중 발생하는 사이드슬립(β)을 PD 댐퍼로 자동 보정한다:

```
β     = sideslip_rad                          (현재 사이드슬립 [rad])
β̇     = (β - prev_sideslip) / dt             (사이드슬립 변화율)

rudder_correction = -0.8 × β  -  0.3 × β̇    (PD 보정)
yaw += rudder_correction × gain_scale
yaw  = clamp(yaw, -1.0, 1.0)
```

효과: 선회 시 "볼이 가운데"에 오도록 러더를 자동 보정. 사이드슬립이 +이면 반대쪽 러더, 변화율이 크면 예측 보정.

### 4.4 속도 기반 게인 스케일링 (P1-3)

모든 PD 게인에 **역제곱(inverse-square)** 속도 보상이 적용된다:

```
gain_scale = clamp( (V_ref / V)², 0.3, 3.0 )

pitch *= gain_scale
roll  *= gain_scale
yaw   *= gain_scale
```

> **변경 이력 (P1-3)**: 기존 선형 스케일링(`speed_ratio`)에서 역제곱으로 변경.
>
> 이유: 조종면 모멘트가 동압(q = ½ρV²)에 비례하므로 plant gain ∝ V². 기존 선형 보상(∝V¹)에서는 루프 게인이 V³에 비례하여 고속에서 과도하게 공격적이 되는 문제가 있었다. 역제곱 보상으로 **속도에 무관한 일정한 루프 게인**을 달성한다.

| 속도 | V_ref/V | gain_scale | 효과 |
|------|---------|------------|------|
| 0.5 × V_ref | 2.0 | 3.0 (클램프) | 저속 보상 최대 |
| V_ref | 1.0 | 1.0 | 기준 |
| 2 × V_ref | 0.5 | 0.25 → 0.3 (클램프) | 고속 감쇠 |

---

## 5. 적응형 게인 스케줄링 (Adaptive Gain Scheduling)

### 5.1 문제: 다양한 항공기

복엽기와 제트기는 관성, 조종면 크기, 비행 속도가 완전히 다르다.
하나의 고정 게인으로는 모든 항공기에서 좋은 조종감을 제공할 수 없다.

### 5.2 해결: FlightCharacteristics 기반 자동 튜닝

```
파일: cpp/src/input.cpp — MouseAimComputer::configure()
파일: rust/aeroblend-physics/src/aerodynamics.rs — compute_flight_characteristics()
```

Rust 물리 엔진이 항공기 형상에서 **12개의 비행 특성값**을 자동 계산하고, C++ 인스트럭터가 이를 받아 게인을 자동 조정한다:

```
┌──────────────┐     FFI     ┌──────────────────┐
│  Rust        │   ────→     │  C++             │
│  glTF 모델   │             │  MouseAimComputer │
│  ↓           │             │  ↓               │
│  부품 분류    │             │  configure()     │
│  ↓           │             │  ↓               │
│  면적/질량   │             │  PD 게인 조정     │
│  ↓           │             │  실속 임계값 설정  │
│  FlightChars │             │  속도 임계값 설정  │
└──────────────┘             └──────────────────┘
```

### 5.3 FlightCharacteristicsC: 12개 파라미터

> **변경 (P3-1)**: 기존 9개에서 12개로 확장. 축별 control_authority(`ca_pitch`, `ca_roll`, `ca_yaw`)가 추가되었다.

| 파라미터 | 단위 | 계산식 | 인스트럭터에서의 용도 |
|----------|------|--------|----------------------|
| `stall_speed_ms` | m/s | √(2W / ρSCL_max) | 실속 보호, 오토스로틀 임계값 |
| `ref_speed_ms` | m/s | 1.3 × stall_speed | 속도 기반 게인 스케일링 기준 |
| `max_aoa_rad` | rad | max(surface.alpha_stall) | 실속 보호 각도 기준 |
| `total_wing_area_m2` | m² | Σ(surface.area) | 비행 특성 계산의 기초 |
| `control_authority` | - | ref_torque / avg_inertia | 레거시 호환 (평균값) |
| **`ca_pitch`** | - | pitch_torque / Ixx | **피치 축 게인 스케줄링** |
| **`ca_roll`** | - | roll_torque / Izz | **롤 축 게인 스케줄링** |
| **`ca_yaw`** | - | yaw_torque / Iyy | **요 축 게인 스케줄링** |
| `wing_loading_kg_m2` | kg/m² | mass / wing_area | 비행 특성 참조 |
| `thrust_to_weight` | - | total_thrust / weight | 비행 특성 참조 |
| `max_pitch_rate_rad_s` | rad/s | pitch_torque / Ixx | 비행 특성 참조 |
| `max_roll_rate_rad_s` | rad/s | roll_torque / Izz | 비행 특성 참조 |

### 5.4 축별 control_authority 기반 게인 스케줄링 (P3-1, P1-1)

control_authority(제어 권한)는 **항공기가 얼마나 민첩한가**를 나타내는 무차원 값이다.

> **변경 (P3-1)**: 기존에는 평균 관성 모멘트 하나로 모든 축의 게인을 결정했으나, 축별 관성이 크게 다른 항공기(예: F-16은 피치/롤 관성 차이 4.2배)에서 심각한 오류가 발생했다. 이제 **각 축별로 독립적인 control_authority**를 사용한다.

```
축별 control_authority 계산:

ca_pitch = (q_ref × S × chord × Cm_elevator) / Ixx    (피치: 엘리베이터 토크 / 피치 관성)
ca_roll  = (q_ref × S × span  × Cl_aileron)  / Izz    (롤:   에일러론 토크 / 롤 관성)
ca_yaw   = (q_ref × S × span  × Cn_rudder)   / Iyy    (요:   러더 토크 / 요 관성)

여기서:
  q_ref       = 기준 속도에서의 동압 (0.5 × ρ × V_ref²)
  S           = 총 날개 면적
  chord/span  = 기준 시위 / 기준 스팬
  Cm/Cl/Cn    = 각 조종면의 모멘트 계수
  Ixx/Iyy/Izz = 축별 관성 모멘트
```

높은 ca = 민첩한 축 (가벼운 관성) → P 게인 ↓, D 게인 ↑
낮은 ca = 둔한 축 (무거운 관성)  → P 게인 ↑, D 게인 ↓

스케일링 공식:

```
ca_reference = 0.25  (기준값: 일반 항공기)

P_scale(ca) = √(ca_reference / ca)          ← 둔한 축일수록 비례 게인 증가
D_scale(ca) = (ca_reference / ca)^(3/4)     ← 둔한 축일수록 미분 게인도 증가 (비대칭)

Kp_pitch = 2.5 × P_scale(ca_pitch),   Kd_pitch = 1.52 × D_scale(ca_pitch)
Kp_roll  = 3.0 × P_scale(ca_roll),    Kd_roll  = 0.8  × D_scale(ca_roll)
Kp_yaw   = 0.5 × P_scale(ca_yaw),     Kd_yaw   = 0.4  × D_scale(ca_yaw)
```

> **D_scale 공식 변경 (P1-1)**: 기존 `√(ca/ca_ref)` → `(ca_ref/ca)^(3/4)`.
>
> 기존 공식은 방향이 반전되어 있었다: 민첩한 기체에 오히려 D를 줄이고 둔한 기체에 늘리는 오류. `3/4` 지수는 감쇠비 ζ가 제어 권한 변화에 걸쳐 일정하게 유지되도록 수학적으로 도출된 값이다 (`ζ ∝ Kd × √(ca)` → `Kd ∝ ca^(-3/4)`로 ζ 일정).

직관:
- **둔한 기체**: 더 세게 밀어야 움직이므로 P ↑ / 관성이 커서 진동이 느리므로 D도 ↑
- **민첩한 기체**: 약하게 밀어도 움직이므로 P ↓ / 이미 빠른 감쇠가 있으므로 D ↓

게인 안전 범위 (클램핑):

| 축 | Kp 범위 | Kd 범위 |
|----|---------|---------|
| Pitch | [1.0, 8.0] | [0.3, 4.0] |
| Roll | [1.0, 10.0] | [0.2, 3.0] |
| Yaw | [0.2, 2.0] | [0.1, 1.5] |

---

## 6. 비행 보호 시스템

인스트럭터는 단순히 방향을 맞추는 것 외에, 위험한 비행 상태를 자동으로 방지한다.

### 6.1 실속 보호 (Stall Protection)

받음각(AoA)이 임계값을 초과하면 피치업을 억제하고 노즈다운을 강제한다:

```
stall_aoa    = max_aoa × 0.85   (보호 시작: 최대 받음각의 85%)
critical_aoa = max_aoa × 0.95   (강제 노즈다운: 95%)
```

> **임계값 변경 (P2-2)**: 기존 70%/90% → 85%/95%. 기존 임계값은 비행 성능의 15~20%를 낭비하는 과보호였다. 조종사가 최대 AoA의 85%까지 자유롭게 기동할 수 있도록 보호 시작 시점을 늦추었다.

**2단계 보호:**

```
AoA 범위              pitch_up_authority    stall_nose_down
─────────────────────────────────────────────────────────
< stall_aoa           1.0 (제한 없음)       0.0
stall ~ critical      1.0 → 0.0 (선형)      0 → -0.6 (선형)
> critical_aoa        0.0 (피치업 차단)      -1.0 (풀 노즈다운)
```

동작:
1. 플레이어가 마우스를 위로 올려도, AoA > 85%이면 피치업 명령이 점진적으로 줄어듦
2. AoA > 95%이면 피치업이 완전 차단되고, 노즈다운 바이어스가 최대로 적용
3. 속도가 회복되면 자동으로 보호 해제

#### Reference Windup 방지 (Anti-Windup, P2-3)

실속 보호가 피치업을 차단하는 동안에도 마우스 타겟(target_pitch)은 계속 누적되는 문제가 있었다. 이로 인해 보호 해제 시 누적된 피치 명령이 한꺼번에 방출되어 **급격한 피치업 → 재실속**을 유발했다.

**Back-calculation 방식 anti-windup**으로 해결한다:

```
if pitch_up_authority < 1.0:
    current_pitch = asin(clamp(forward.y, -1, 1))
    error_excess  = target_pitch - current_pitch
    k_bc          = 2.0  (back-calculation 게인)

    target_pitch -= k_bc × (1 - pitch_up_authority) × error_excess × dt
```

동작: 실속 보호가 강할수록(`pitch_up_authority`가 낮을수록) 마우스 타겟을 실제 항공기 피치 방향으로 끌어당긴다. 보호 해제 시 타겟과 실제 방향의 차이가 최소화되어 부드러운 회복을 보장한다.

### 6.2 속도 기반 피치 제한

속도가 낮을수록 허용 피치 각도가 줄어든다:

```
speed_frac = clamp((airspeed - min_speed) / (ref_speed - min_speed), 0, 1)
max_pitch_for_speed = 0.09 + 0.60 × speed_frac

저속 (stall 근처):  max_pitch ≈ 5° → 거의 수평만 허용
고속 (ref_speed):   max_pitch ≈ 40° → 급상승 허용
```

### 6.3 저속 보호

AoA가 높지 않더라도, 속도 자체가 위험할 때:

```
if airspeed < min_speed:
    low_factor = airspeed / min_speed
    pitch_up_authority *= low_factor    ← 피치업 점진 제한
    stall_nose_down -= 0.4 × (1 - low_factor)  ← 노즈다운 바이어스
```

### 6.4 하중배수 보정 오토스로틀 (n-Adjusted Auto-Throttle, P0)

속도가 위험하게 낮을 때 자동으로 스로틀을 올린다.

> **치명적 버그 수정 (P0)**: 기존 오토스로틀은 1g 실속 속도 기준으로만 작동했다. 그러나 60° 뱅크 선회에서는 하중배수 n=2.0이 되어 실속 속도가 √2 ≈ 1.41배로 증가한다. 기존에는 이 증가분을 인식하지 못해 **선회 중 실속 속도가 오토스로틀 작동 속도를 초과**하는 치명적 상황이 발생했다.

```
파일: cpp/src/input.cpp — InputHandler::update()

// P0: 현재 뱅크 각도에서 실제 하중배수 계산
bank_angle = |atan2(-right.y, up.y)|       (항공기 자세에서 뱅크 추출)
n_current  = 1 / max(cos(bank_angle), 0.1) (하중배수: 1g ~ 10g 클램프)

// 실속 속도를 하중배수로 보정
Vs_n = stall_speed × √(n_current)          (n-보정 실속 속도)

auto_throttle_speed = Vs_n × 1.30          (보정된 130% 임계값)
critical_speed      = Vs_n × 0.85          (보정된 85% 비상 임계값)

if airspeed < auto_throttle_speed:
    urgency = 1 - (airspeed - critical_speed) / (auto_throttle_speed - critical_speed)
    min_throttle = 60% + 40% × urgency    (60% → 100% 선형)
    throttle = max(player_throttle, min_throttle)
```

| 뱅크 | n | Vs_n (biplane) | 오토스로틀 시작 |
|------|---|----------------|----------------|
| 0° | 1.0 | 18.3 m/s | 23.8 m/s |
| 45° | 1.41 | 21.8 m/s | 28.3 m/s |
| 60° | 2.0 | 25.9 m/s | 33.7 m/s |

동작:
- 직선 비행: 기존과 동일하게 작동
- 선회 중: 뱅크가 커질수록 오토스로틀이 더 일찍, 더 강하게 개입
- 급선회에서도 안전한 에너지 관리 보장

---

## 7. 키보드 오버라이드

키보드 입력은 인스트럭터를 **즉시 무시**하고 직접 조종면을 제어한다:

```
final_pitch = (kb_pitch ≠ 0) ? kb_pitch : inst_pitch
final_roll  = (kb_roll  ≠ 0) ? kb_roll  : inst_roll
final_yaw   = (kb_yaw   ≠ 0) ? kb_yaw   : inst_yaw
```

| 키 | 조종면 | 값 |
|----|--------|-----|
| W | pitch | -1.0 (노즈 다운) |
| S | pitch | +1.0 (노즈 업) |
| A | roll | -1.0 (좌 롤) |
| D | roll | +1.0 (우 롤) |
| Q | yaw | -1.0 (좌 요) |
| E | yaw | +1.0 (우 요) |

스로틀은 Shift/Ctrl로 제어한다:

| 키 | 기능 | 효과 |
|----|------|------|
| Shift | 스로틀 증가 | +1.5/초 |
| Ctrl | 스로틀 감소 | -1.5/초 |

키보드 입력에는 **InputSmoother**가 적용되어 부드러운 전환을 제공한다:
```
rate_up   = 4.0 (누를 때)
rate_down = 6.0 (뗄 때, 더 빠르게 원복)
```

인스트럭터 출력은 PD 제어기의 미분항이 이미 스무딩 역할을 하므로 스무더 없이 직접 전달된다.

키보드를 놓으면 스무더의 현재값이 인스트럭터 출력으로 동기화되어 **끊김 없는 전환**을 보장한다.

---

## 8. 카메라 시스템과의 연동

### 8.1 궤도 카메라 (Orbit Camera)

```
파일: cpp/src/camera.cpp — Camera::update()
```

카메라는 **항공기를 중심으로 궤도를 돌며 관찰하는** 방식으로 동작한다:

```
       ┌── 에임 방향 (하얀 원) ──┐
       │                         │
       │    Aircraft             │
       │    ┌───┐                ▼
Camera ←────┤   ├────────────→  ○ (하얀 원)
  👁️        └───┘
       ◄─── kCamDistance ───►◄─ sphere ─►
```

핵심 원리:
- **하얀 원(에임 타겟)**은 항공기 중심 구면 위에서 마우스로 이동
- **카메라**는 항공기를 사이에 두고 하얀 원의 **반대편**에 위치
- **카메라는 항상 수평**을 유지 (up = world Y)

```
aim_dir = (sin(cam_yaw) × cos(cam_pitch),     ← 에임 방향 벡터
           sin(cam_pitch),
           cos(cam_yaw) × cos(cam_pitch))

camera_pos = aircraft_pos - aim_dir × distance × zoom   ← 반대편
camera_target = aircraft_pos                              ← 항공기를 바라봄
camera_up = (0, 1, 0)                                     ← 항상 수평
```

| 하얀 원 위치 | 카메라 위치 | 화면에 보이는 것 |
|-------------|-----------|----------------|
| 항공기 앞 | 항공기 뒤 | 뒤에서 따라가는 시점 (기본) |
| 항공기 아래 | 항공기 위 | 위에서 내려다보는 시점 |
| 항공기 위 | 항공기 아래 | 아래에서 올려다보는 시점 |
| 항공기 오른쪽 | 항공기 왼쪽 | 왼쪽에서 관찰하는 시점 |

카메라 궤도 방향은 마우스 타겟을 **지수 감쇠(exponential decay)**로 부드럽게 추적한다:

```
kCamTrackRate = 6.0
alpha = 1 - exp(-kCamTrackRate × dt)

cam_yaw   += alpha × (target_yaw - cam_yaw)
cam_pitch += alpha × (target_pitch - cam_pitch)
```

추적 속도가 6.0으로, 물리 응답보다 빠르다. 이것이 **"카메라가 먼저 궤도를 돌고, 항공기가 따라 방향을 바꾼다"**는 3-body convergence의 핵심이다.

#### G-force 카메라 감속 (P2-6)

고 G 기동 시 카메라 추적 속도를 감소시켜 **시각적 G-feel**을 제공한다:

```
n_z = g_force                                   (현재 하중배수, 부호 있음)
kCamTrackRate_eff = kCamTrackRate / (0.5 + 0.5 × |n_z|)
alpha = 1 - exp(-kCamTrackRate_eff × dt)
```

| 하중배수 | 유효 추적 속도 | 체감 효과 |
|---------|---------------|----------|
| 1g (직선) | 6.0 | 정상 추적 |
| 3g (급선회) | 3.0 | 카메라 궤도 이동이 느려져 G-feel |
| 6g (전투기동) | 1.7 | 강한 G-feel, 터널 비전 효과 |

이 효과로 플레이어가 급기동 시 "몸이 무거워지는" 느낌을 시각적으로 체험할 수 있다.

### 8.2 에임 디스플레이 계산

```
파일: cpp/src/camera.cpp — Camera::compute_aim_display()
```

모든 마커는 **항공기 중심 구체** 위의 점이다. 카메라도 이 구체 위에 위치하며(에임 반대편), 화면에는 카메라에서 바라본 구체의 앞쪽 반구가 보인다. 각 마커의 화면 위치는 **카메라 시선 방향(cam_yaw/cam_pitch) 기준 각도 오프셋**으로 계산한다:

```
1) 화면 해상도에서 px_per_rad 계산
   vfov_rad = fov_deg × π / 180
   px_per_rad = (screen_h × 0.5) / tan(vfov_rad × 0.5)

2) 마커의 yaw/pitch와 카메라의 cam_yaw/cam_pitch 차이를 구함
   Δyaw   = marker_yaw - cam_yaw      ([-π, π] 범위로 정규화)
   Δpitch = marker_pitch - cam_pitch

3) 화면 중앙 기준 픽셀 오프셋
   screen_x = screen_w/2 + Δyaw   × px_per_rad
   screen_y = screen_h/2 - Δpitch × px_per_rad
```

| 마커 | 구체 위의 위치 | yaw/pitch 산출 |
|------|-------------|---------------|
| 에임 원 (흰색) | 사용자가 가리킨 방향 | `mouse_target_yaw`, `mouse_target_pitch` |
| 기총 조준선 (초록) | 콕핏이 바라보는 방향 | `atan2(fwd_x, fwd_z)`, `asin(fwd_y)` |
| 속도벡터 (시안) | 실제 비행 경로 방향 | `atan2(vx/spd, vz/spd)`, `asin(vy/spd)` |

구체 모델의 구조:
- **카메라**: 구체 뒤쪽 (에임 반대편, `aircraft_pos - aim_dir × distance`)
- **에임 원**: 구체 앞쪽 (사용자가 마우스로 이동)
- **기총 조준선**: 구체 앞쪽 (항공기 노즈 방향, 물리 엔진이 결정)
- 항공기는 구체 중심에 위치하며, 카메라와 에임 원 사이에 있음

---

## 9. Rust 물리 엔진과의 연결

### 9.1 제어 입력 → 물리적 힘

인스트럭터가 생성한 pitch/roll/yaw 명령은 Rust 물리 엔진에서 **조종면 모멘트**로 변환된다:

```
파일: rust/aeroblend-physics/src/aerodynamics.rs — compute_control_moments()

q = 0.5 × ρ × V²   (동압)

pitch_moment = q × S × chord × Cm_elevator × controls.pitch
roll_moment  = q × S × span  × Cl_aileron  × controls.roll
yaw_moment   = q × S × span  × Cn_rudder   × controls.yaw
```

모멘트 계수:
| 조종면 | 계수 | 값 |
|--------|------|----|
| 엘리베이터 | Cm_elevator | -0.025 |
| 에일러론 | Cl_aileron | 0.020 |
| 러더 | Cn_rudder | -0.012 |

모멘트는 동압에 비례하므로, **고속에서 조종면 효과가 크고 저속에서 작다**. 이것이 인스트럭터의 속도 기반 게인 스케일링과 조화를 이룬다.

### 9.2 부호 있는 G-force 계산 (P2-5)

```
파일: rust/aeroblend-physics/src/engine.rs — step()
```

G-force는 **body up 축(동체 위쪽)을 따른 공기역학적 하중배수**로 계산된다:

```
F_aero = total_force - gravity               (공기역학 힘만 추출)
body_up = orientation의 y축 (위쪽 벡터)
g_force = dot(F_aero, body_up) / (m × g)     (부호 있는 하중배수)
```

> **변경 (P2-5)**: 기존 `|a_no_grav| / g`(항상 양수)에서 부호 있는 body-axis 하중으로 변경.
>
> 기존 방식의 문제: 항상 양수여서 역 G(-G)를 표현할 수 없었다. 노즈다운 푸시에서도 항상 +G로 표시되었고, 카메라 G-feel이 방향을 구분하지 못했다. 새 방식은 수평 비행에서 +1g, 역전 비행에서 -1g, 급상승에서 +3~5g 등 정확한 하중을 반영한다.

### 9.3 통합 루프

```
파일: rust/aeroblend-physics/src/engine.rs — step()

1. ControlStateC 수신 (인스트럭터 출력)
2. RK4 적분 (120Hz 고정 타임스텝)
   - 중력 + 공기역학 + 조종면 모멘트 + 안정성 모멘트 + 추력
3. 부호 있는 G-force 계산 (body up 축 투영)
4. 지면 충돌 처리
5. AircraftPhysicsStateC 반환 (다음 프레임 인스트럭터 입력)
```

---

## 10. 전체 처리 순서 (Per Frame)

```
1. SDL 이벤트 수집
   └→ 마우스 dx,dy → MouseAimComputer::update_from_mouse()
   └→ 키보드 → key_w_, key_s_, ...

2. InputHandler::update(dt, aircraft_state)
   ├→ n-보정 오토스로틀 계산 (P0: 뱅크 → n → Vs_n → 임계값)
   ├→ MouseAimComputer::compute_instructor()
   │   ├→ 초기화 + configure() (첫 프레임: 축별 게인 자동 설정)
   │   ├→ 속도 기반 피치 제한
   │   ├→ 목표 방향 → 로컬 오차 계산
   │   ├→ 에임 디스플레이 값 계산
   │   ├→ 실속 보호 적용 (P2-2: 85%/95% 임계)
   │   ├→ anti-windup back-calculation (P2-3: 타겟 보정)
   │   ├→ 물리 기반 협조선회 (P1-2: atan 뱅크)
   │   ├→ 역제곱 게인 스케일링 (P1-3)
   │   ├→ PD 제어 + D-term 저역통과 필터 (P2-7)
   │   └→ 사이드슬립 PD 보정 (P1-4)
   └→ 키보드 오버라이드 + 스무딩 적용
      └→ ControlStateC 생성

3. Physics Step (Rust, 120Hz)
   └→ ControlStateC → 조종면 모멘트 → RK4 → 부호 G-force(P2-5)
      → AircraftPhysicsStateC

4. Camera::update()
   └→ mouse_target → cam direction (지수 추적 + G-force 감속 P2-6)

5. Renderer::render()
   └→ 3D 장면 그리기

6. Camera::compute_aim_display()
   └→ 에임 원, 기총 조준선, 속도벡터 위치 계산

7. HudRenderer::render()
   └→ 에임 마커 + 계기판 그리기
```

---

## 부록 A: P0~P3 개선사항 요약

수학 연구 기반 분석(`docs/mouse-aim-math-formulas.md`)을 통해 도출된 개선 사항 목록. 2026-02-22 구현 완료.

| 우선순위 | 코드 | 개선 내용 | 핵심 공식 | 구현 파일 |
|---------|------|----------|----------|----------|
| **P0** | n-adj auto-throttle | 선회 시 하중배수 보정 오토스로틀 | `Vs_n = Vs × √n` | `input.cpp` |
| **P1-1** | D_scale 수정 | 미분 게인 스케일 방향 오류 수정 | `(ca_ref/ca)^(3/4)` | `input.cpp` |
| **P1-2** | 물리 기반 선회 | 선회율 기반 뱅크 각도 계산 | `atan(K × ψ̇ × V/g)` | `input.cpp` |
| **P1-3** | 역제곱 gain_scale | 속도 보상을 inverse-square로 변경 | `(V_ref/V)²` | `input.cpp` |
| **P1-4** | 사이드슬립 보정 | PD 댐퍼로 러더 자동 보정 | `-0.8β - 0.3β̇` | `input.cpp` |
| **P2-1** | Kd 기본값 상향 | 최적 감쇠비 ζ=0.7 달성 | `Kd_pitch = 1.52` | `input.h` |
| **P2-2** | 실속 임계 상향 | 과보호 완화 (70/90→85/95%) | `0.85 × max_aoa` | `input.cpp` |
| **P2-3** | Anti-windup | Back-calculation 방식 적분 포화 방지 | `target -= k_bc × (1-auth) × err × dt` | `input.cpp` |
| **P2-5** | 부호 G-force | Body up 축 투영 하중배수 | `dot(F_aero, up)/(mg)` | `engine.rs` |
| **P2-6** | 카메라 G-feel | G-force로 카메라 추적 감속 | `rate/(0.5+0.5×|n|)` | `camera.cpp` |
| **P2-7** | D-term LPF | 미분항 저역통과 필터 | `α_f = 1-exp(-dt/τ), τ=0.01` | `input.cpp` |
| **P3-1** | 축별 ca | 피치/롤/요 독립 제어 권한 | `ca = torque/I_axis` | `aerodynamics.rs`, `ffi.h` |

미구현 (미래 과제):
- **P2-4**: 에너지 관리 (총 에너지 제어 — TECS 방식)
- **P3-2**: 형상 기반 안정성 계수 (glTF 형상에서 Cmα, Cnβ 직접 추정)

---

## 부록 B: 소스 파일 참조

| 파일 | 역할 |
|------|------|
| `cpp/include/aeroblend/input.h` | MouseAimComputer (D-term 필터, anti-windup, sideslip 상태 포함), InputSmoother, InputHandler 선언 |
| `cpp/src/input.cpp` | 인스트럭터, 축별 게인 스케줄링, 실속 보호, n-보정 오토스로틀, 사이드슬립 보정 구현 |
| `cpp/include/aeroblend/camera.h` | Camera, AimDisplay 선언 |
| `cpp/src/camera.cpp` | 3-body 추적, G-force 카메라 감속, 에임 디스플레이 계산 |
| `cpp/include/aeroblend/ffi.h` | cbindgen 생성 FFI 헤더 (FlightCharacteristicsC에 축별 ca 포함) |
| `cpp/src/hud.cpp` | 에임 원/기총 조준선/속도벡터 렌더링 |
| `cpp/src/main.cpp` | 게임 루프, 120Hz 물리 타임스텝 |
| `cpp/tests/test_mouse_flight.cpp` | 인스트럭터 통합 테스트 (17개) |
| `rust/aeroblend-physics/src/aerodynamics.rs` | FlightCharacteristics 계산 (축별 ca), 조종면 모멘트, 안정성 모멘트 |
| `rust/aeroblend-physics/src/engine.rs` | 6DoF 엔진, RK4 적분, 부호 있는 G-force 계산 |
| `rust/aeroblend-physics/src/types.rs` | FlightCharacteristics 내부 타입 (ca_pitch/roll/yaw 필드) |
| `rust/aeroblend-core/src/lib.rs` | FFI 타입 정의 (FlightCharacteristicsC 12개 필드, ControlStateC 등) |
| `docs/mouse-aim-math-formulas.md` | P0~P3 수학적 도출 근거 (수학 연구 보고서) |

## 부록 C: 용어 정리

| 용어 | 설명 |
|------|------|
| **인스트럭터 (Instructor)** | 목표 방향 → 조종면 명령을 자동 생성하는 소프트웨어 파일럿 |
| **PD 제어기** | 비례(P) + 미분(D) 피드백 제어기. 오차와 변화율을 모두 사용 |
| **협조선회 (Coordinated Turn)** | 롤, 피치, 요를 조합하여 사이드슬립 없는 깨끗한 선회를 만드는 기법 |
| **받음각 (AoA)** | 기류와 날개 시위 사이의 각도. 임계각 초과 시 실속 |
| **동압 (Dynamic Pressure, q)** | 0.5 × ρ × V². 공력의 크기를 결정하는 핵심 파라미터 |
| **control_authority (ca)** | 단위 제어 입력 당 각가속도. 높을수록 민첩함. 축별(ca_pitch/roll/yaw) 또는 평균값 |
| **3-body convergence** | 에임 원(마우스)→카메라→기총 조준선(항공기)이 서로 다른 속도로 수렴하는 시각적 구조. 수렴 완료 시 수렴선 길이 → 0 |
| **수렴선 (Convergence Line)** | 기총 조준선과 에임 원을 잇는 선. 길이 = 추적 오차. 0에 가까우면 조준 완료(사격 가능) |
| **기총 조준선 (Gunsight)** | 항공기의 기총선(gun line) 방향을 화면에 투영한 초록색 십자선. 기총 발사 시 탄환이 날아가는 방향 |
| **오토스로틀** | 속도가 위험하게 낮을 때 자동으로 스로틀을 올리는 보호 시스템 |
| **트림 오프셋** | 수평 비행에 필요한 기본 피치 보정값 (~5°) |
| **하중배수 (n, Load Factor)** | 항공기에 작용하는 양력/중력 비율. 수평 1g, 60° 뱅크 2g |
| **감쇠비 (ζ, Damping Ratio)** | PD 제어기의 오버슈트/진동 특성. ζ=0.7이 최적 (4.6% 오버슈트) |
| **anti-windup** | 제어 포화(saturation) 시 적분/참조값 누적을 방지하는 기법 |
| **사이드슬립 (β, Sideslip)** | 기류가 항공기 측면에서 오는 각도. 0이면 조화 비행 |
| **G-feel** | 하중배수 변화를 시각/촉각으로 체감시키는 효과. 카메라 추적 감속으로 구현 |
| **저역통과 필터 (LPF)** | 고주파 노이즈를 제거하고 저주파 신호만 통과. D-term에 τ=0.01s 적용 |
