# AeroBlend 현재 마우스 에임 시스템 수학 모델 분석

작성일: 2026-03-01
분석자: math-lead / control-engineer (CEO 감수)
소스 경로: `cpp/src/input.cpp`, `cpp/include/aeroblend/input.h`
참조 문서: `docs/mouse-aim-system.md`, `docs/mouse-aim-math-formulas.md`

---

## 1. 시스템 아키텍처

AeroBlend의 마우스 에임 시스템은 War Thunder 스타일 "인스트럭터" 패턴을 구현한다.

### 1.1 신호 흐름도

```
마우스 입력 (dx, dy)
    │
    ▼
MouseAimComputer::update_from_mouse()
    │  target_yaw += dx × sensitivity
    │  target_pitch -= dy × sensitivity
    ▼
목표 방향 벡터 (tx, ty, tz) [월드 좌표]
    │
    ▼
로컬 프레임 변환 (항공기 orientation 행렬)
    │
    ▼
오차 계산 (pitch_err, yaw_err, bank_err)
    │
    ├──→ 실속 보호 (AoA 기반)
    ├──→ 속도 보호 (저속 제한)
    ├──→ Anti-windup (back-calculation)
    │
    ▼
PD 제어기 (축별 적응 게인)
    │  pitch = (Kp × pitch_err - D_filtered_pitch) × gain_scale
    │  roll  = (Kp × bank_err  - D_filtered_roll)  × gain_scale
    │  yaw   = (Kp × yaw_err   - D_filtered_yaw)   × gain_scale
    │
    ├──→ 사이드슬립 PD 보정 (yaw 채널)
    │
    ▼
조종면 출력 [-1, 1]
```

---

## 2. 마우스 입력 처리

### 2.1 목표 방향 갱신

```
target_yaw   += dx × sensitivity     // sensitivity = 0.005 rad/pixel
target_pitch -= dy × sensitivity     // 마우스 위 = pitch up (부호 반전)
target_pitch  = clamp(target_pitch, -1.2, max_pitch_for_speed)
```

### 2.2 속도 기반 피치 제한

```
speed_frac = clamp((V - V_min) / (V_ref - V_min), 0, 1)
max_pitch_for_speed = 0.09 + 0.60 × speed_frac
```

- 기준속도(V_ref): 순항 속도 → 최대 피치 ≈ 0.69 rad (≈ 40°)
- 최소속도(V_min): 실속 × 1.05 → 최대 피치 ≈ 0.09 rad (≈ 5°)
- 저속에서 자동으로 상승 각도를 제한하여 에너지 손실 방지

### 2.3 목표 방향 벡터 구성

구면 좌표 → 직교 좌표 변환:

```
tx = sin(target_yaw) × cos(target_pitch)
ty = sin(target_pitch)
tz = cos(target_yaw) × cos(target_pitch)
```

### 2.4 초기화

첫 프레임에서 항공기 전방 벡터로부터 초기 목표 설정:

```
target_yaw   = atan2(fwd_x, fwd_z)
target_pitch = asin(clamp(fwd_y, -1, 1)) + 0.08   // +0.08 rad ≈ 4.6° 트림 오프셋
```

트림 오프셋은 수평 비행을 유지하기 위한 양력 받음각 보상이다.

---

## 3. 오차 계산

### 3.1 로컬 프레임 변환

목표 방향 벡터를 항공기 orientation 행렬(열 우선)으로 변환:

```
lx = tx × right_x + ty × right_y + tz × right_z   // 로컬 X (right)
ly = tx × up_x    + ty × up_y    + tz × up_z       // 로컬 Y (up)
lz = tx × fwd_x   + ty × fwd_y   + tz × fwd_z      // 로컬 Z (forward)
```

### 3.2 각도 오차

```
yaw_err   = atan2(lx, lz)                          // 수평면 편향 [rad]
pitch_err = atan2(ly, √(lx² + lz²))               // 수직 편향 [rad]
```

이 방식은 MouseFlight의 InverseTransformPoint와 유사하지만, 정규화된 방향 벡터에 대해 atan2로 정확한 각도를 추출한다는 점에서 수학적으로 더 정밀하다.

### 3.3 뱅크 오차 (물리 기반 협조선회)

```
ψ̇_desired = K_turn_rate × yaw_err          // K_turn_rate = 0.268 [1/s]
desired_bank = atan(ψ̇_desired × V / g)     // g = 9.81 m/s²
max_bank = (π/3) × clamp(V/V_ref, 0.3, 1.0)   // 최대 60°, 저속에서 축소

bank_err = desired_bank - current_bank
current_bank = atan2(-right_y, up_y)
```

**수학적 근거**: 정상 협조선회에서 tan(φ) = V²/(Rg) = Vψ̇/g. 따라서 원하는 선회율 ψ̇에 대응하는 뱅크각은 φ = atan(Vψ̇/g).

---

## 4. 비행 보호 시스템

### 4.1 실속 보호 (P2-2)

```
stall_aoa    = max_aoa × 0.85    // 보호 시작 (soft limit)
critical_aoa = max_aoa × 0.95    // 강제 노즈다운 (hard limit)

if AoA > stall_aoa:
    excess = clamp((AoA - stall_aoa) / (critical_aoa - stall_aoa), 0, 1)
    pitch_up_authority = 1 - excess     // 점진적 피치업 제한
    stall_nose_down = -0.6 × excess     // 점진적 노즈다운 편향

if AoA > critical_aoa:
    pitch_up_authority = 0              // 피치업 완전 차단
    stall_nose_down = -1.0              // 최대 노즈다운
```

### 4.2 저속 보호

```
if V < V_min:
    low_factor = clamp(V / V_min, 0, 1)
    pitch_up_authority *= low_factor
    stall_nose_down -= 0.4 × (1 - low_factor)
```

### 4.3 Anti-windup (P2-3: Back-calculation)

실속 보호로 피치 권한이 제한될 때, 목표 피치를 현재 항공기 피치 방향으로 점진 복귀:

```
if pitch_up_authority < 1.0:
    current_pitch = asin(clamp(fwd_y, -1, 1))
    error_excess = target_pitch - current_pitch
    target_pitch -= k_bc × (1 - pitch_up_authority) × error_excess × dt
    // k_bc = 2.0, dt ≈ 1/120
```

이는 적분기가 없는 PD 시스템에서의 "위치 windup" 방지 메커니즘이다. 실속 중 목표 피치가 현재 자세에서 크게 벗어나지 않도록 한다.

### 4.4 피치 오차에 보호 적용

```
if pitch_err > 0:
    pitch_err *= pitch_up_authority    // 피치업만 제한 (다운은 허용)
pitch_err += stall_nose_down           // 노즈다운 편향 추가
```

---

## 5. PD 제어기

### 5.1 적응 게인 스케줄링 (P3-1)

configure() 함수에서 FlightCharacteristics 기반으로 게인 자동 설정:

```
ca_reference = 0.25    // 기준 제어 권한 (일반 항공기)

P_scale(ca) = √(ca_ref / ca)          // 저 ca → 높은 P 게인
D_scale(ca) = (ca_ref / ca)^0.75       // 저 ca → 높은 D 게인

Kp_pitch = 2.5 × P_scale(ca_pitch)    Kd_pitch = 1.52 × D_scale(ca_pitch)
Kp_roll  = 3.0 × P_scale(ca_roll)     Kd_roll  = 0.80 × D_scale(ca_roll)
Kp_yaw   = 0.5 × P_scale(ca_yaw)      Kd_yaw   = 0.40 × D_scale(ca_yaw)
```

**게인 클램핑 범위:**

| 축 | Kp 범위 | Kd 범위 |
|----|---------|---------|
| pitch | [1.0, 8.0] | [0.3, 5.0] |
| roll | [1.0, 10.0] | [0.2, 4.0] |
| yaw | [0.2, 2.0] | [0.1, 2.0] |

**수학적 근거**: 제어 권한(ca)이 낮은 기체(대형/저속)는 같은 응답을 얻기 위해 더 높은 게인이 필요하다. P_scale ∝ 1/√ca 는 2차 시스템의 고유진동수를 일정하게 유지하기 위한 스케일링이다.

### 5.2 D-term 저역통과 필터 (P2-7)

```
alpha_f = 1 - exp(-dt / tau)     // tau = 0.01s, dt = 1/120s
D_filtered += alpha_f × (Kd × omega - D_filtered)
```

이 1차 IIR 필터는 미분항의 고주파 노이즈를 억제한다:
- 차단 주파수: f_c = 1/(2πτ) ≈ 15.9 Hz
- 120Hz 물리 루프에서의 필터 계수: alpha_f ≈ 0.565

### 5.3 속도 기반 게인 스케일링 (P1-3)

```
gain_scale = clamp((V_ref / V)², 0.3, 3.0)
```

**수학적 근거**: 공기역학적 조종면 효과는 동압 q = ½ρV²에 비례한다. 따라서 플랜트 게인이 V²에 비례하고, 이를 상쇄하기 위해 컨트롤러 게인을 1/V²로 스케일링한다. 결과적으로 루프 게인 L = K_controller × G_plant 이 속도에 무관하게 일정해진다.

### 5.4 최종 PD 출력

```
pitch = clamp((Kp_pitch × pitch_err - D_filtered_pitch) × gain_scale, -1, 1)
roll  = clamp((Kp_roll  × bank_err  - D_filtered_roll)  × gain_scale, -1, 1)
yaw   = clamp((Kp_yaw   × yaw_err   - D_filtered_yaw)   × gain_scale, -1, 1)
```

### 5.5 사이드슬립 보정 (P1-4)

별도의 PD 댐퍼가 요 채널에 추가:

```
beta_dot = (beta - prev_beta) / dt
rudder_correction = -0.8 × beta - 0.3 × beta_dot
yaw += rudder_correction × gain_scale
yaw = clamp(yaw, -1, 1)
```

---

## 6. 자동 스로틀 (P0: n-adjusted)

### 6.1 n-조정 실속 속도

뱅크 각도에 따른 하중 계수 보정:

```
n = 1 / max(cos(bank), 0.1)          // 하중 계수
Vs_n = Vs × √n                        // n-조정 실속 속도

auto_throttle_speed = Vs_n × 1.30     // 자동스로틀 작동 시작
critical_speed = Vs_n × 0.85          // 긴급 풀파워
```

### 6.2 자동 스로틀 로직

```
if V < auto_throttle_speed:
    urgency = 1 - clamp((V - critical_speed) / (at_speed - critical_speed), 0, 1)
    min_throttle = 0.6 + 0.4 × urgency    // 60% → 100%
    throttle = max(throttle, min_throttle)
```

---

## 7. 키보드 오버라이드 및 스무딩

### 7.1 키보드 우선

```
final_axis = keyboard_input ≠ 0 ? keyboard_input : instructor_output
```

키보드 입력이 있으면 인스트럭터를 완전히 오버라이드한다.

### 7.2 InputSmoother

키보드 입력에만 적용되는 비대칭 레이트 리미터:

```
if target > current:
    current += rate_up × dt
else:
    current -= rate_down × dt
```

| 축 | rate_up | rate_down |
|----|---------|-----------|
| throttle | 1.5 | 2.0 |
| pitch/roll | 4.0 | 6.0 |
| yaw | 3.0 | 5.0 |

인스트럭터 PD 출력은 미분항이 이미 스무딩 역할을 하므로 직접 전달된다.

---

## 8. HUD 에임 표시

```
aim_x = clamp(yaw_err / 1.0, -1, 1)
aim_y = clamp(-pitch_err / 1.0, -1, 1)
```

오차가 줄어들면 에임 커서가 보어사이트에 수렴한다.

---

## 9. 레퍼런스 대비 AeroBlend 고유 기능 요약

| 기능 | MouseFlight | MouseAimFlight (KSP) | AeroBlend |
|-----|------------|---------------------|-----------|
| 제어 구조 | P-only | P-only (new) / PID (old) | PD |
| 게인 적응 | 없음 | speedFactor (old) | 축별 ca 기반 + 속도 스케일링 |
| 실속 보호 | 없음 | 없음 | 85%/95% AoA, progressive |
| Anti-windup | 없음 | 포화 시 적분 리셋 (old) | Back-calculation |
| 협조선회 | 없음 | upWeighting (old) | 물리 기반 desired_bank |
| 사이드슬립 보정 | 없음 | 롤 오차 직접 차감 (old) | 별도 PD 댐퍼 |
| D-term 필터 | 없음 | 없음 | 1차 IIR (tau=0.01s) |
| 자동스로틀 | 없음 | 없음 | n-adjusted |
| 비행 모드 | 없음 | Normal/Cruise/Aggressive (old) | 단일 모드 |
| 피치 제한 | 없음 | 없음 | 속도 기반 동적 제한 |
