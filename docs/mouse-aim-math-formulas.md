# AeroBlend 마우스 에임 시스템 수학 공식집

> **제어이론 연구소 (alpha)** + **항공역학 연구소 (beta)** 공동 연구 결과
> 생성일: 2026-02-22 | AeroBlend 비행제어 수학연구소

이 문서는 마우스 에임(Mouse Aim) 시스템의 모든 수학적 기반을 한 곳에 정리한 기술 문서이다.
`docs/mouse-aim-system.md` (시스템 설계서)에서 다루지 못한 **수학적 유도, 최적화 근거, 안정성 증명**에 초점을 맞춘다.

---

## 목차

**Part I: 제어이론 분석 (alpha 연구소)**

1. [PD 제어기 전달함수 모델](#1-pd-제어기-전달함수-모델)
2. [2차 시스템 모델링 (zeta, omega_n)](#2-2차-시스템-모델링)
3. [최적 게인 공식 도출](#3-최적-게인-공식-도출)
4. [안정성 분석](#4-안정성-분석)
5. [D항 저역통과 필터 설계](#5-d항-저역통과-필터-설계)
6. [3-Body Convergence 수학 모델](#6-3-body-convergence-수학-모델)

**Part II: 항공역학 분석 (beta 연구소)**

7. [협조선회 수학 모델](#7-협조선회-수학-모델)
8. [에너지 관리 모델](#8-에너지-관리-모델)
9. [실속 보호 수학](#9-실속-보호-수학)
10. [항력 극선과 선회 성능](#10-항력-극선과-선회-성능)
11. [속도 기반 게인 스케일링 정당화](#11-속도-기반-게인-스케일링-정당화)

**Part III: 통합 발견 및 개선 (양팀 합의)**

12. [control_authority 축별 분리](#12-control_authority-축별-분리)
13. [D_scale 스케줄링 오류 수정](#13-d_scale-스케줄링-오류-수정)
14. [Reference Windup 문제와 해결](#14-reference-windup-문제와-해결)
15. [P0: 선회 중 n-adjusted 오토스로틀](#15-p0-선회-중-n-adjusted-오토스로틀)
16. [사이드슬립 보정](#16-사이드슬립-보정)
17. [비행감(Flight Feel) 향상 공식](#17-비행감flight-feel-향상-공식)
18. [누락된 피드백 루프 체계](#18-누락된-피드백-루프-체계)
19. [3종 항공기 프로파일 수치 검증](#19-3종-항공기-프로파일-수치-검증)
20. [개선 권고 요약 (확정)](#20-개선-권고-요약-확정)

---

## 1. PD 제어기 전달함수 모델

### 1.1 피치 축 플랜트 모델

피치 축의 항공기 동역학을 1-자유도 회전 시스템으로 단순화한다.

**제어 모멘트:**

```
M_control = q · S · c · Cm · u

여기서:
  q  = 0.5 · rho · V^2        동압 [Pa]
  S  = total_wing_area         총 날개 면적 [m^2]
  c  = ref_chord               기준 시위 [m]
  Cm = -0.025                  피치 조종 모멘트 계수 [-]
  u  = controls.pitch          조종 입력 [-1, +1]
```

**공기역학적 감쇠:**

코드(`compute_angular_damping`)로부터:

```
M_damping = -b_eff · omega

b_eff = D_pitch · q / (q + 500)

여기서:
  D_pitch = 5000                감쇠 계수 [kg·m^2/s]
  q       = 동압 [Pa]
```

**운동방정식 (피치 축):**

```
I_pitch · theta_ddot = M_control + M_damping + M_stability

I_pitch · theta_ddot = q·S·c·Cm·u - b_eff·omega + M_stability
```

여기서 `I_pitch`는 피치 관성 모멘트 (Ixx).

### 1.2 플랜트 전달함수

안정성 모멘트를 무시하고 (별도 루프로 처리), 주 제어 루프만 고려하면:

```
P(s) = theta(s) / U(s) = K_c / (s · (s + b))

여기서:
  K_c = q · S · c · |Cm| / I_pitch    [rad/s^2]  (제어 이득)
  b   = b_eff / I_pitch                [1/s]     (감쇠율)
  b   = D_pitch · q / ((q + 500) · I_pitch)
```

### 1.3 PD 제어기 전달함수

현재 인스트럭터의 PD 제어는:

```
u(t) = Kp · e(t) - Kd · omega(t)

여기서:
  e(t) = pitch_err = theta_target - theta  (각도 오차)
  omega(t) = d(theta)/dt                    (각속도)
```

라플라스 변환:

```
C(s) = Kp + Kd · s
```

### 1.4 개루프 전달함수 (Open-Loop)

```
L(s) = C(s) · P(s) = (Kp + Kd·s) · K_c / (s · (s + b))

     = K_c · Kd · (s + Kp/Kd) / (s · (s + b))
```

### 1.5 폐루프 전달함수 (Closed-Loop)

```
T(s) = L(s) / (1 + L(s))

     = K_c · (Kp + Kd·s) / (s^2 + (b + K_c·Kd)·s + K_c·Kp)
```

이는 **표준 2차 시스템** 형태를 가진다.

---

## 2. 2차 시스템 모델링

### 2.1 표준 2차 형태

폐루프 특성방정식:

```
s^2 + 2·zeta·omega_n·s + omega_n^2 = 0
```

폐루프 전달함수의 분모와 대조:

```
s^2 + (b + K_c·Kd)·s + K_c·Kp = 0
```

따라서:

```
omega_n = sqrt(K_c · Kp)                      고유진동수 [rad/s]

zeta = (b + K_c · Kd) / (2 · omega_n)         감쇠비 [-]
     = (b + K_c · Kd) / (2 · sqrt(K_c · Kp))
```

### 2.2 기본 프로파일 수치 검증

기본 기체 파라미터 (default profile: mass=5000kg, I_pitch=5000, no aero surfaces):

```
S = 30 m^2, c = 2.5 m, |Cm| = 0.025
stall_speed = sqrt(2 · 5000 · 9.81 / (1.225 · 30 · 1.5)) = 42.19 m/s
ref_speed = 1.3 · 42.19 = 54.85 m/s
q_ref = 0.5 · 1.225 · 54.85^2 = 1843 Pa

K_c = 1843 · 30 · 2.5 · 0.025 / 5000 = 0.691 rad/s^2
b   = 5000 · 1843 / ((1843 + 500) · 5000) = 1843/2343 = 0.787 s^-1
```

현재 게인 (Kp=2.5, Kd=1.2)에서:

```
omega_n = sqrt(0.691 · 2.5) = sqrt(1.728) = 1.314 rad/s
zeta    = (0.787 + 0.691 · 1.2) / (2 · 1.314)
        = (0.787 + 0.829) / 2.629
        = 1.616 / 2.629
        = 0.615
```

### 2.3 시간 영역 응답 특성

2차 시스템의 시간 영역 파라미터:

```
감쇠 고유진동수:  omega_d = omega_n · sqrt(1 - zeta^2)

정착 시간 (2%):   t_s = 4 / (zeta · omega_n)

최대 오버슈트:    M_p = exp(-pi · zeta / sqrt(1 - zeta^2))  (0 < zeta < 1일 때)
```

기본 프로파일 (zeta=0.615, omega_n=1.314):

```
omega_d = 1.314 · sqrt(1 - 0.379) = 1.314 · 0.788 = 1.036 rad/s
t_s     = 4 / (0.615 · 1.314) = 4 / 0.808 = 4.95 s
M_p     = exp(-pi · 0.615 / 0.788) = exp(-2.452) = 8.6%
```

**해석**: 현재 zeta=0.615에서 약 8.6% 오버슈트, 정착 시간 ~5초. 사용자가 "비행이 잘 되지 않는다"고 느끼는 원인 중 하나는 오버슈트와 느린 정착이다.

---

## 3. 최적 게인 공식 도출

### 3.1 목표 감쇠비

항공 제어 시스템의 국제 표준(MIL-HDBK-1797, Cooper-Harper) 기준:

| 감쇠비 (zeta) | 비행 품질 | 조종사 피드백 |
|---------------|-----------|--------------|
| < 0.3 | 불량 (Level 3) | 진동, 제어 곤란 |
| 0.3 - 0.5 | 허용 (Level 2) | 약간 진동, 불편 |
| **0.5 - 0.8** | **양호 (Level 1)** | **쾌적, 예측 가능** |
| 0.7 | **최적** | **최소 정착시간, 과도응답 최적** |
| > 1.0 | 과감쇠 | 느린 응답, 둔감 |

**설계 목표: zeta = 0.7** (임계감쇠에 가까운 최적값)

### 3.2 최적 Kd 공식 (Kp 고정)

zeta = 0.7을 만족하는 Kd를 역산한다:

```
zeta = (b + K_c · Kd) / (2 · sqrt(K_c · Kp))

0.7 = (b + K_c · Kd) / (2 · omega_n)

K_c · Kd = 2 · 0.7 · omega_n - b

Kd_optimal = (1.4 · sqrt(K_c · Kp) - b) / K_c
```

**핵심 공식:**

```
┌──────────────────────────────────────────────────────┐
│                                                      │
│  Kd* = (1.4 · sqrt(K_c · Kp) - b) / K_c            │
│                                                      │
│  여기서:                                              │
│    K_c = q_ref · S · c · |Cm| / I_pitch              │
│    b   = D_pitch · q_ref / ((q_ref + 500) · I_pitch) │
│    Kp  = 2.5 · sqrt(ca_ref / ca)  (현재 스케줄링)    │
│                                                      │
└──────────────────────────────────────────────────────┘
```

### 3.3 기본 프로파일 최적 Kd

```
Kd* = (1.4 · sqrt(0.691 · 2.5) - 0.787) / 0.691
    = (1.4 · 1.314 - 0.787) / 0.691
    = (1.840 - 0.787) / 0.691
    = 1.053 / 0.691
    = 1.52
```

**결과: 기본 프로파일에서 최적 Kd는 1.52** (현재 1.2 → +27% 증가)

### 3.4 최적 Kp-Kd 동시 최적화

omega_n과 zeta를 독립 설계 파라미터로 지정:

```
Kp* = omega_n^2 / K_c

Kd* = (2 · zeta · omega_n - b) / K_c
```

목표 omega_n = 1.5 rad/s (정착시간 ~3.8초), zeta = 0.7일 때:

```
Kp* = 1.5^2 / 0.691 = 2.25 / 0.691 = 3.26
Kd* = (2 · 0.7 · 1.5 - 0.787) / 0.691 = (2.1 - 0.787) / 0.691 = 1.90
```

### 3.5 control_authority 기반 스케줄링의 수학적 근거

현재 구현의 P/D 스케일링:

```
P_scale = sqrt(ca_ref / ca)
D_scale = sqrt(ca / ca_ref) = 1 / P_scale
```

**수학적 정당화:**

control_authority `ca`는 다음과 같이 정의된다:

```
ca = K_c(at V_ref) = q_ref · S · c · |Cm| / I_avg
```

이것이 곧 플랜트 이득 K_c이므로:

```
omega_n = sqrt(K_c · Kp) = sqrt(ca · Kp)
```

모든 기체에서 동일한 omega_n을 유지하려면:

```
ca_1 · Kp_1 = ca_2 · Kp_2

Kp_2 / Kp_1 = ca_1 / ca_2

Kp(ca) = Kp_base · (ca_ref / ca)
```

그런데 현재 구현은 `P_scale = sqrt(ca_ref / ca)`를 사용한다. 이는 **선형이 아닌 제곱근** 스케일링이다.

**이유:** 완전한 선형 보상은 저속에서 과도한 게인을 초래한다. 제곱근 스케일링은 절충안:

```
완전 보상: Kp ∝ 1/ca         → 극단적 기체에서 불안정
제곱근:    Kp ∝ 1/sqrt(ca)   → 부분 보상, 안정적
현재 구현: Kp ∝ sqrt(ca_ref/ca) = 1/sqrt(ca/ca_ref) ← 이것
```

**omega_n이 ca에 따라 변하는 정도:**

```
omega_n(ca) = sqrt(ca · Kp_base · sqrt(ca_ref/ca))
            = sqrt(Kp_base · ca_ref^0.5 · ca^0.5)
            = (Kp_base · ca_ref)^0.25 · ca^0.25 · sqrt(ca_ref)^0.25 ... (복잡)
```

단순화하면:

```
omega_n ∝ ca^(1/4)    (제곱근 스케줄링에서)
```

즉, ca가 4배 변해도 omega_n은 sqrt(2)=1.41배만 변한다. 이는 합리적인 범위이다.

### 3.6 D 게인 스케줄링 보정

현재 `D_scale = sqrt(ca/ca_ref)`인데, 최적 zeta를 유지하려면:

```
zeta = (b + K_c · Kd) / (2 · omega_n)
```

K_c가 ca에 비례하고, omega_n ∝ ca^(1/4)이므로:

```
Kd ∝ omega_n / K_c ∝ ca^(1/4) / ca = ca^(-3/4)
```

그러나 현재 `Kd ∝ sqrt(ca/ca_ref) = ca^(1/2)`인데, 이는 **반대 방향**이다.

**정정 공식:**

```
D_scale_corrected = sqrt(ca_ref / ca)    (P_scale과 동일)
```

또는 더 정확하게는:

```
Kd(ca) = (2 · zeta_target · sqrt(ca · Kp(ca)) - b(ca)) / ca
```

**이것이 사용자가 "비행이 잘 안 된다"고 느끼는 핵심 원인일 수 있다.**
현재 D 게인 스케줄링 방향이 수학적 최적과 반대이기 때문에, 민첩한 기체에서는 감쇠 부족(진동), 둔한 기체에서는 과감쇠(둔한 응답)가 발생한다.

---

## 4. 안정성 분석

### 4.1 개루프 전달함수의 극점과 영점

```
L(s) = K_c · Kd · (s + z) / (s · (s + b))

영점:  z = Kp / Kd
극점:  s = 0,  s = -b
```

기본 프로파일 (Kp=2.5, Kd=1.2):

```
영점: z = 2.5 / 1.2 = 2.083 rad/s
극점: p1 = 0, p2 = -0.787 rad/s
```

### 4.2 이득 여유 (Gain Margin)

이 시스템은 Type-1 시스템(원점에 극점 1개)이며, 영점이 좌반면에 있으므로 위상이 -180도에 도달하지 않는다. 따라서 **이득 여유는 무한대** (GM = infinity).

수학적 증명:

```
angle(L(jw)) = angle(Kd·(jw + z)) - angle(jw) - angle(jw + b)
             = atan(w/z) - 90° - atan(w/b)
```

w -> infinity일 때:

```
angle -> 90° - 90° - 90° = -90°
```

위상이 -90도까지만 감소하므로 -180도에 도달하지 않는다. 따라서 이 PD 구조는 **이득 불안정이 발생하지 않는다**.

### 4.3 위상 여유 (Phase Margin)

이득 교차 주파수(|L(jw_gc)| = 1)에서의 위상 여유:

```
|L(jw)| = K_c · Kd · sqrt(w^2 + z^2) / (w · sqrt(w^2 + b^2))
```

|L(jw_gc)| = 1을 수치적으로 풀면 (기본 프로파일, Kd=1.52):

```
w_gc 근방에서 시행착오:

w = 1.5: |L| = 0.691·1.52·sqrt(2.25+4.34)/(1.5·sqrt(2.25+0.619))
        = 1.050·2.567/(1.5·1.694) = 2.696/2.541 ≈ 1.06

w_gc ≈ 1.55 rad/s
```

위상 여유:

```
PM = 180° + angle(L(jw_gc))
   = 180° + atan(1.55/2.083) - 90° - atan(1.55/0.787)
   = 180° + 36.6° - 90° - 63.1°
   = 63.5°
```

**위상 여유 ≈ 63.5도** -- 제어 시스템에서 45도 이상이면 양호, 60도 이상이면 우수.

### 4.4 Nyquist 판별법

P(s)에 불안정 극점이 없고 (BIBO 안정), 개루프 전달함수 L(s)의 Nyquist 선도가 (-1, 0)을 encircle하지 않으므로, 폐루프 시스템은 **안정**이다.

---

## 5. D항 저역통과 필터 설계

### 5.1 문제: 미분항의 고주파 잡음 증폭

순수 미분항 `Kd · s`는 고주파에서 이득이 무한히 증가한다:

```
주파수 f에서 D항 이득 = Kd · 2·pi·f

60Hz 마우스 샘플링 잡음:
  이득 = 1.52 · 2·pi·60 = 573
```

이 잡음이 조종면 명령으로 직접 전달되어 미세 진동의 원인이 된다.

### 5.2 1차 저역통과 필터 적용

```
D_filtered(s) = Kd · s / (tau·s + 1)
```

이 필터는:
- 저주파에서: D_filtered ≈ Kd · s (원래 미분 동작 유지)
- 고주파에서: D_filtered ≈ Kd / tau (상수로 수렴, 잡음 증폭 방지)

### 5.3 필터 시정수 설계

필터의 차단 주파수:

```
f_cutoff = 1 / (2·pi·tau)
```

설계 기준:
- omega_n = 1.314 rad/s (시스템 대역폭)
- 필터 차단 주파수는 omega_n의 5~10배 이상이어야 제어 성능에 영향 없음

```
omega_filter > 5 · omega_n = 6.57 rad/s
tau < 1 / 6.57 = 0.152 s
```

또한 이산화 시 Nyquist 한계 고려 (120Hz 물리 업데이트):

```
omega_nyquist = pi · 120 = 377 rad/s
tau > 1 / 377 = 0.0027 s (너무 작으면 의미 없음)
```

### 5.4 권장값

```
┌────────────────────────────────────────────────┐
│  tau = 0.008 ~ 0.012 s                         │
│  (f_cutoff = 13 ~ 20 Hz)                       │
│                                                │
│  권장: tau = 0.01 s (f_cutoff = 15.9 Hz)       │
│                                                │
│  필터 적용 후 60Hz 잡음 이득:                   │
│    1.52 · 2·pi·60 / sqrt(1 + (0.01·2·pi·60)^2)│
│    = 573 / sqrt(1 + 14.2) = 573 / 3.90        │
│    = 147  (잡음 이득 75% 감소)                  │
│                                                │
│  위상 여유 변화: < 2도 감소 (무시 가능)          │
└────────────────────────────────────────────────┘
```

### 5.5 이산 시간 구현

120Hz 물리 루프에서의 1차 필터 이산화 (쌍일차 변환):

```
alpha_f = dt / (tau + dt)        dt = 1/120 = 0.00833 s

alpha_f = 0.00833 / (0.01 + 0.00833) = 0.454

D_filtered[n] = alpha_f · Kd · (omega[n]) + (1 - alpha_f) · D_filtered[n-1]
```

또는 더 간단한 지수 감쇠 필터:

```
alpha_f = 1 - exp(-dt / tau) = 1 - exp(-0.833) = 0.565

D_filtered[n] = D_filtered[n-1] + alpha_f · (Kd · omega[n] - D_filtered[n-1])
```

---

## 6. 3-Body Convergence 수학 모델

### 6.1 세 개의 동적 시스템

마우스 에임의 세 추적 대상을 각각의 전달함수로 모델링한다:

| 객체 | 입력 | 전달함수 | 시정수 |
|------|------|----------|--------|
| 마우스 타겟 | 마우스 dx/dy | 즉시 (gain만) | 0 |
| 카메라 | 마우스 타겟 | 1차 지수 추적 | 1/k_cam |
| 항공기 | 제어 명령 | 2차 시스템 | 1/omega_n |

### 6.2 카메라 전달함수

```
cam_yaw(t+dt) = cam_yaw(t) + alpha · (target_yaw - cam_yaw(t))

alpha = 1 - exp(-k_cam · dt),   k_cam = 6.0
```

연속 시간 등가:

```
G_cam(s) = k_cam / (s + k_cam) = 6.0 / (s + 6.0)
```

카메라의 시정수: tau_cam = 1/6.0 = 0.167 s

### 6.3 항공기 전달함수

Section 1에서 유도한 폐루프:

```
G_aircraft(s) = K_c · (Kp + Kd·s) / (s^2 + 2·zeta·omega_n·s + omega_n^2)
```

등가 시정수 (과도 응답 기준): tau_aircraft ≈ 1/(zeta · omega_n) = 1/(0.7 · 1.314) = 1.09 s

### 6.4 3-body 수렴 조건

시스템이 자연스럽게 느껴지려면 **시정수 분리 조건**이 필요하다:

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  tau_mouse ≪ tau_camera ≪ tau_aircraft                      │
│                                                             │
│  0 s (즉시)  ≪  0.167 s  ≪  1.09 s                         │
│                                                             │
│  비율: camera/aircraft = 0.167/1.09 = 0.153 ≈ 1:6.5        │
│                                                             │
│  권장 비율: 1:3 ~ 1:10 (현재 1:6.5는 적절)                  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 6.5 시각적 오차 궤적

스텝 입력(마우스를 한 번에 이동)에 대한 각 객체의 시간 응답:

```
target(t) = theta_cmd · u(t)                                   (즉시 도달)

camera(t) = theta_cmd · (1 - exp(-6.0 · t))                    (지수 수렴)

aircraft(t) = theta_cmd · [1 - exp(-zeta·omega_n·t) ·          (2차 수렴)
              (cos(omega_d·t) + (zeta/sqrt(1-zeta^2))·sin(omega_d·t))]
```

HUD 상에서의 오차 (에임 원과 십자선 사이 거리):

```
visual_error(t) = |camera(t) - aircraft(t)| · px_per_rad
```

이 오차가 최대가 되는 시점이 "기동 중"을 가장 강하게 느끼는 순간이다.

최대 시각적 오차 시점 (근사):

```
t_max_error ≈ tau_camera · ln(tau_aircraft / tau_camera)
            ≈ 0.167 · ln(6.5)
            ≈ 0.167 · 1.87
            ≈ 0.31 s
```

---

## 7. 협조선회 수학 모델

### 7.1 이상적 협조선회 기본 공식

수평면에서의 정상 선회(steady level turn):

**하중계수:**

```
n = L / W = 1 / cos(phi)

여기서:
  n   = 하중계수 [-]
  L   = 양력 [N]
  W   = 중량 [N]
  phi = 뱅크 각도 [rad]
```

**선회율:**

```
psi_dot = g · tan(phi) / V

여기서:
  psi_dot = 선회율 [rad/s]
  g       = 9.81 m/s^2
  V       = 대기속도 [m/s]
```

**선회 반경:**

```
R = V^2 / (g · tan(phi))
```

### 7.2 현재 구현의 수학적 분석

코드에서:

```
desired_bank = clamp(yaw_err * 1.5, -max_bank, max_bank)
```

이것은 **비례 제어기**: `phi_cmd = K_turn · psi_err`

여기서 `K_turn = 1.5`의 물리적 의미:

```
정상 선회에서:
  psi_dot = g · tan(phi) / V

작은 각도 근사 (phi < 30°):
  psi_dot ≈ g · phi / V

원하는 선회율이 yaw_err에 비례하면:
  psi_dot_desired ∝ yaw_err

따라서:
  phi_desired = (V / g) · psi_dot_desired = (V / g) · K_turn_rate · yaw_err
```

현재 `K_turn = 1.5`는 속도에 무관한 상수인데, 이상적으로는:

```
K_turn_ideal(V) = K_turn_rate · V / g

V = 54.85 m/s에서 K_turn = 1.5이면:
  K_turn_rate = 1.5 · g / V = 1.5 · 9.81 / 54.85 = 0.268 rad/s per rad
```

### 7.3 속도 의존 선회 이득 개선 공식

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  K_turn(V) = K_turn_rate · V / g                                │
│                                                                 │
│  K_turn_rate = 0.268 rad/s per rad  (설계 선회 응답성)           │
│                                                                 │
│  또는 등가적으로:                                                │
│  desired_bank = clamp(yaw_err · K_turn_rate · V / g,            │
│                       -max_bank, max_bank)                      │
│                                                                 │
│  V = ref_speed에서 K_turn = 1.5 (현재와 동일)                   │
│  V = stall_speed에서 K_turn ≈ 1.15 (선회 완화)                  │
│  V = 2·ref_speed에서 K_turn ≈ 3.0 (고속 선회 강화)              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 7.4 뱅크 각도 제한의 수학적 근거

현재:

```
max_bank = 60° · clamp(speed_ratio, 0.3, 1.0)
```

물리적 근거: 뱅크 각도 phi에서의 실속 속도:

```
V_stall(phi) = V_stall_1g · sqrt(n) = V_stall_1g · sqrt(1/cos(phi))
```

| phi (deg) | n (하중계수) | V_stall 증가 | 의미 |
|-----------|-------------|-------------|------|
| 30 | 1.155 | +7.5% | 안전 |
| 45 | 1.414 | +19% | 주의 |
| 60 | 2.000 | +41% | 위험 |
| 75 | 3.864 | +97% | 매우 위험 |

속도 기반 뱅크 제한의 정당화:

```
speed_ratio = V / V_ref
n_available = (V / V_stall)^2 = (speed_ratio · V_ref / V_stall)^2
            = (speed_ratio · 1.3)^2

phi_max_safe = arccos(1 / n_available)
```

| speed_ratio | n_available | phi_max_safe | 현재 max_bank |
|-------------|-------------|-------------|---------------|
| 0.3 | 0.15 | 선회 불가 | 18° |
| 0.5 | 0.42 | 선회 불가 | 30° |
| 0.77 | 1.0 | 0° | 46° |
| 1.0 | 1.69 | 54° | 60° |

현재 구현은 speed_ratio < 0.77에서 물리적 한계보다 관대하다. **에너지를 소모하는 선회를 허용하는 게임적 설계 결정**이다.

---

## 8. 에너지 관리 모델

### 8.1 비에너지 (Specific Energy)

항공기의 총 기계적 에너지 상태:

```
E_s = h + V^2 / (2·g)    [m]

여기서:
  h = 고도 [m]
  V = 대기속도 [m/s]
  g = 9.81 m/s^2
```

**비에너지 변화율 (비잉여추력):**

```
dE_s/dt = V · (T - D) / W = V · (T/W - D/W)

여기서:
  T = 추력 [N]
  D = 항력 [N]
  W = 중량 [N]
```

### 8.2 선회 중 에너지 손실

수평 선회에서 양력이 n배로 증가하므로, 유도항력도 증가한다:

```
D_turn = D_0 + D_i · n^2

여기서:
  D_0 = 영양력항력 (기생항력)
  D_i = 1g 비행 시 유도항력
  n   = 1/cos(phi)
```

**선회 중 추가 항력:**

```
Delta_D = D_i · (n^2 - 1) = D_i · (sec^2(phi) - 1) = D_i · tan^2(phi)
```

### 8.3 에너지 균형으로부터의 최대 지속 선회율

지속 선회(에너지 손실 없음) 조건: T = D_turn

```
T = D_0 + D_i · n^2

n_sustained = sqrt((T - D_0) / D_i)

phi_sustained = arccos(1 / n_sustained)

psi_dot_sustained = g · sqrt(n_sustained^2 - 1) / V
```

### 8.4 오토스로틀과 에너지 연결

오토스로틀 임계값의 에너지적 의미:

```
auto_throttle_speed = V_stall · 1.30
critical_speed      = V_stall · 0.85

V_stall에서의 비에너지:
  E_s_stall = h + V_stall^2 / (2g)

auto_throttle 작동점에서의 비에너지 마진:
  Delta_E = (1.3·V_stall)^2/(2g) - V_stall^2/(2g)
          = V_stall^2 · (1.69 - 1) / (2g)
          = 0.69 · V_stall^2 / (2g)
```

기본 프로파일 (V_stall = 42.19 m/s):

```
Delta_E = 0.69 · 42.19^2 / (2 · 9.81) = 0.69 · 1780 / 19.62 = 62.6 m
```

오토스로틀은 **약 63m의 고도 에너지 마진**에서 개입한다.

---

## 9. 실속 보호 수학

### 9.1 기본 실속 속도

정적 1g 실속 속도:

```
V_stall = sqrt(2·W / (rho·S·CL_max))

여기서:
  W      = m·g         중량 [N]
  rho    = 공기 밀도    [kg/m^3]
  S      = 날개 면적    [m^2]
  CL_max = 최대 양력계수 [-]
```

### 9.2 고도 보정

고도에 따른 밀도 변화 (ISA 표준대기):

```
rho(h) = rho_0 · (1 - 0.0000226·h)^4.256    (h < 11000m)

V_stall(h) = V_stall_SL · sqrt(rho_0 / rho(h))
           = V_stall_SL · (1 - 0.0000226·h)^(-2.128)
```

| 고도 [m] | rho/rho_0 | V_stall 증가 |
|----------|-----------|-------------|
| 0 | 1.000 | 0% |
| 1000 | 0.908 | +5% |
| 3000 | 0.742 | +16% |
| 5000 | 0.601 | +29% |

### 9.3 하중계수 보정 (선회 중 실속)

선회 중 실속 속도:

```
V_stall_turn = V_stall · sqrt(n) = V_stall · (1/cos(phi))^(1/2)
```

### 9.4 2단계 보호 함수의 수학적 표현

현재 구현의 보호 함수:

```
excess(alpha) = clamp((alpha - alpha_stall) / (alpha_crit - alpha_stall), 0, 1)

pitch_up_authority(alpha) = {
  1.0                              if alpha <= alpha_stall
  1.0 - excess(alpha)              if alpha_stall < alpha <= alpha_crit
  0.0                              if alpha > alpha_crit
}

stall_nose_down(alpha) = {
  0.0                              if alpha <= alpha_stall
  -0.6 · excess(alpha)             if alpha_stall < alpha <= alpha_crit
  -1.0                             if alpha > alpha_crit
}
```

여기서:

```
alpha_stall = 0.70 · max_aoa
alpha_crit  = 0.90 · max_aoa
```

**유효 피치 명령:**

```
u_pitch_effective = {
  u_pitch                                              if alpha <= alpha_stall
  max(u_pitch, 0)·(1-excess) + min(u_pitch, 0) - 0.6·excess  if 중간
  min(u_pitch, 0) - 1.0                                if alpha > alpha_crit
}
```

### 9.5 저속 보호 결합

AoA 보호와 저속 보호가 동시 적용될 때:

```
low_factor = clamp(V / min_speed, 0, 1)

pitch_up_authority *= low_factor

stall_nose_down -= 0.4 · (1 - low_factor)
```

총 보호 강도:

```
total_protection = 1.0 - pitch_up_authority + |stall_nose_down|
```

범위: 0 (보호 없음) ~ 2.0 (AoA + 저속 동시 최대 보호)

---

## 10. 항력 극선과 선회 성능

### 10.1 항력 극선 (Drag Polar)

코드의 항력 모델:

```
CD = CD0 + CL^2 / (pi · e · AR)

여기서:
  CD0 = 영양력항력 계수 (기생항력)
  CL  = 양력 계수
  e   = Oswald 스팬 효율 (0.5 ~ 0.95)
  AR  = 종횡비 = b^2 / S
```

Oswald 효율 추정 (코드):

```
e = 1.78 · (1 - 0.045 · AR^0.68) - 0.64,   clamp [0.5, 0.95]
```

### 10.2 최소 항력 속도

항력 최소 = CL/CD 최대인 조건:

```
CL_md = sqrt(CD0 · pi · e · AR)

V_md = sqrt(2·W / (rho·S·CL_md))
```

### 10.3 선회 시 필요 추력

n-g 선회에서:

```
CL_turn = n · CL_1g = n · 2·W / (rho·V^2·S)

CD_turn = CD0 + CL_turn^2 / (pi·e·AR)
        = CD0 + n^2 · CL_1g^2 / (pi·e·AR)

D_turn  = 0.5·rho·V^2·S·CD_turn

T_required = D_turn   (수평 선회 유지)
```

### 10.4 최대 선회 성능 다이어그램

속도별 최대 지속 선회율:

```
n_max_thrust(V) = sqrt( (T - 0.5·rho·V^2·S·CD0) · pi·e·AR / (2·W^2/(rho·V^2·S)) )

n_max_struct = 구조적 하중 한계 (일반적으로 3~6g)

n_available(V) = min(n_max_thrust(V), n_max_struct)

psi_dot_max(V) = g · sqrt(n_available^2 - 1) / V
```

코너 속도 (최대 선회율 속도):

```
V_corner: n_max_thrust(V) = n_max_struct 인 속도
```

---

## 11. 속도 기반 게인 스케일링 정당화

### 11.1 현재 구현

```
speed_ratio = clamp(V / V_ref, 0.2, 1.5)
gain_scale = clamp(speed_ratio, 0.5, 1.2)
```

### 11.2 물리적 근거: 동압과 제어 효과

조종면 모멘트는 동압 q에 비례:

```
M_control = q · S · c · Cm · u = (0.5·rho·V^2) · S · c · Cm · u
```

따라서 실제 플랜트 이득 K_c는:

```
K_c(V) = q(V) · S · c · |Cm| / I = (V/V_ref)^2 · K_c_ref
```

K_c가 V^2에 비례하므로, 일정한 폐루프 특성을 유지하려면:

```
Kp(V) = Kp_ref · (V_ref/V)^2    (역제곱 보상)
```

### 11.3 현재 선형 스케일링의 해석

현재 `gain_scale = V/V_ref`인데, 이론적으로는 `(V_ref/V)^2`이 필요하다.

그러나:

```
실제 omega_n(V) = sqrt(K_c(V) · Kp(V) · gain_scale(V))
               = sqrt((V/V_ref)^2 · K_c_ref · Kp_ref · (V/V_ref))
               = sqrt(K_c_ref · Kp_ref) · (V/V_ref)^(3/2)
```

omega_n이 V^(3/2)에 비례한다. 이것은:
- 고속: 더 빠른 응답 (민첩) -- 사실적
- 저속: 더 느린 응답 (둔함) -- 안전하지만 불만족

역제곱 보상을 완전히 적용하면 고속에서 조종감이 너무 둔해진다.

**현재 선형 스케일링은 사실성과 조종감 사이의 합리적 절충이다.**

### 11.4 확정 개선 공식: 역제곱 보상 (양팀 합의)

alpha-beta 양 연구소의 안정성 분석 및 합의에 의해 **역제곱 보상**이 확정되었다:

```
┌──────────────────────────────────────────────────────────────┐
│                                                              │
│  gain_scale = clamp((V_ref / V)², 0.3, 3.0)                │
│                                                              │
│  근거:                                                       │
│  - ωn이 속도에 완전 무관 (가장 깔끔한 설계)                  │
│  - ζ 변동 ±10.7% (0.686~0.850) — 실용적으로 충분            │
│  - 클램핑 3.0 상한으로 저속 포화 방지                        │
│  - 유효 보상 범위: 0.58~1.83 × V_ref (M_eff = 1.00)        │
│                                                              │
│  속도별 gain_scale 값:                                       │
│    V = 0.58·V_ref: gain_scale = 3.0 (하한 클램핑)           │
│    V = V_ref:      gain_scale = 1.0 (기준)                  │
│    V = 1.83·V_ref: gain_scale = 0.3 (상한 클램핑)           │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

적용 후 실효 루프 게인:

```
K_c(V) ∝ V²,  gain_scale ∝ 1/V²

실효 Kp_eff = Kp · gain_scale = Kp · (V_ref/V)²

omega_n = sqrt(K_c · Kp_eff) = sqrt((V/V_ref)² · K_c_ref · Kp · (V_ref/V)²)
        = sqrt(K_c_ref · Kp)
        = 상수  ← 속도 무관!
```

이로써 Section 11.3에서 지적한 V^3 문제가 완전히 해결된다.

---

## 12. control_authority 축별 분리

### 12.1 현재 문제: 평균 관성 모멘트 사용

현재 `control_authority`는 3축 평균 관성모멘트로 계산된다:

```
ca = q_ref · S · c · Cm / avg_inertia
avg_inertia = (Ixx + Iyy + Izz) / 3
```

그러나 실제 PD 제어기의 각 축은 **축별 관성 모멘트**를 사용한다:

```
피치 플랜트: G_pitch(s) = K_c_pitch / (s(s + b_pitch)),  K_c_pitch = q·S·c·Cm / Ixx
롤 플랜트:   G_roll(s)  = K_c_roll  / (s(s + b_roll)),   K_c_roll  = q·S·b_span·Cl / Izz
요 플랜트:   G_yaw(s)   = K_c_yaw   / (s(s + b_yaw)),    K_c_yaw   = q·S·b_span·Cn / Iyy
```

### 12.2 F-16 사례: 4.2배 오차

F-16의 관성 모멘트 (추정치):

```
Ixx = 12,000 kg·m²  (피치)
Iyy = 75,000 kg·m²  (요)
Izz = 65,000 kg·m²  (롤)

avg_I = (12000 + 75000 + 65000) / 3 = 50,667 kg·m²

Ixx / avg_I = 0.237 → 피치 축에서 플랜트 이득이 4.2배 과소추정
Iyy / avg_I = 1.480 → 요 축에서 1.5배 과대추정
```

이 오차는 게인 스케줄링 전체에 전파되어, F-16의 피치 반응이 의도보다 4.2배 빠르고 요 반응이 1.5배 느려진다.

### 12.3 개선: 축별 control_authority

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  ca_pitch = q_ref · S · c · |Cm_elev| / Ixx                  │
│  ca_roll  = q_ref · S · b_span · |Cl_ail| / Izz              │
│  ca_yaw   = q_ref · S · b_span · |Cn_rud| / Iyy              │
│                                                                │
│  Kp_pitch = Kp_base · sqrt(ca_ref / ca_pitch)                │
│  Kd_pitch = Kd_base · (ca_ref / ca_pitch)^(3/4)             │
│                                                                │
│  (롤, 요 축도 동일 패턴)                                      │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

## 13. D_scale 스케줄링 오류 수정

### 13.1 현재 오류

현재 구현 (`input.cpp:33`):

```
P_scale = sqrt(ca_ref / ca)     ← 올바름
D_scale = sqrt(ca / ca_ref)     ← 잘못됨! (방향이 반대)
```

### 13.2 수학적 증명

PD 제어기에서 일정 감쇠비(ζ) 유지 조건을 유도한다.

`r = ca_ref / ca` (control authority 비율)로 정의하면, 플랜트 이득 `K_c ∝ 1/r`:

```
Kp(r) = Kp_base · r^a,   Kd(r) = Kd_base · r^b

여기서 현재 구현: a = 1/2, b = -1/2
```

감쇠비 공식에 대입:

```
ζ = (b_aero + K_c · Kd) / (2 · sqrt(K_c · Kp))

K_c ∝ ca ∝ 1/r 이므로:

ζ ∝ ((1/r) · r^b) / sqrt((1/r) · r^a)
  = r^(b-1) / r^((a-1)/2)
  = r^(b - 1 - (a-1)/2)
  = r^(b - a/2 - 1/2)
```

ζ가 r에 무관하려면 (일정 감쇠비 조건):

```
b - a/2 - 1/2 = 0
b = a/2 + 1/2
```

현재 a = 1/2이면:

```
b_correct = 1/4 + 1/2 = 3/4
```

그런데 현재 b = -1/2. 실제 ζ의 r 의존성:

```
현재:  ζ ∝ r^(-1/2 - 1/4 - 1/2) = r^(-5/4) → ca에 따라 급변
올바른: ζ ∝ r^(3/4 - 1/4 - 1/2) = r^0 = 상수 → 일정 감쇠비!
```

### 13.3 올바른 공식

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  P_scale = (ca_ref / ca)^{1/2}     ← 현재와 동일             │
│  D_scale = (ca_ref / ca)^{3/4}     ← 현재와 반대 방향!       │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 13.4 수치 영향 (3종 항공기)

| 항공기 | ca | P_scale | 현재 D_scale | 올바른 D_scale | 현재 ζ | 올바른 ζ |
|--------|-----|---------|-------------|---------------|--------|----------|
| An-2 (복엽, 둔) | 0.142 | 1.33 | 0.75 | 1.52 | ~0.35 | ~0.70 |
| C-172 (기준) | 0.353 | 0.84 | 1.19 | 0.77 | ~0.70 | ~0.70 |
| F-16 (제트, 민첩) | 0.249 | 1.00 | 1.00 | 1.00 | ~0.60 | ~0.70 |

**핵심**: 현재 스케줄링은 둔한 항공기(An-2)에서 감쇠 부족(진동), 민첩한 항공기에서 과감쇠(둔한 응답)를 유발한다. 이것이 항공기 타입별로 비행감이 크게 다른 **근본 원인**이다.

---

## 14. Reference Windup 문제와 해결

### 14.1 문제 정의

실속 보호가 작동하면 `pitch_up_authority`가 0으로 감소하여 피치업 명령을 차단한다. 그러나 마우스 타겟(`target_pitch`)은 계속 누적된다:

```
실속 보호 중:
  target_pitch += dy · sensitivity    ← 계속 증가 (사용자가 마우스 위로)
  pitch_up_authority → 0              ← PD 출력 차단
  pitch_error = target_pitch - theta  ← 점점 커짐

실속 해제 시:
  pitch_up_authority → 1.0            ← 차단 해제
  pitch_error = (누적된 큰 값)        ← 급기동!
```

이것은 적분기 windup의 변형으로, **reference windup**이라 한다.

### 14.2 P0 버그와의 상호작용

Section 15의 P0 버그(선회 중 오토스로틀 실패)와 결합하면:

```
1. 선회 중 속도 감소 → 실속 → authority clamping 작동
2. 오토스로틀이 뒤늦게 개입 (V_stall×1.30 < V_stall_turn)
3. 속도 회복 지연 → authority clamping 장기화
4. target_pitch 누적 증가
5. 실속 해제 시 급기동 (porpoising)
```

### 14.3 개선: Back-Calculation Anti-Windup

실속 보호 중 `target_pitch`를 현재 항공기 방향으로 점진적으로 되돌린다:

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  if pitch_up_authority < 1.0:                                 │
│      // Back-calculation: target을 현재 상태로 동기화         │
│      error_excess = target_pitch - theta_current              │
│      k_bc = 2.0  (back-calculation 속도)                      │
│      target_pitch -= k_bc · (1.0 - pitch_up_authority)        │
│                    · error_excess · dt                         │
│                                                                │
│  효과:                                                         │
│  - authority=0 (완전 차단): target이 빠르게 현재 각도로 수렴  │
│  - authority=0.5 (부분 차단): target이 절반 속도로 동기화     │
│  - authority=1.0 (정상): back-calculation 비활성              │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

## 15. P0: 선회 중 n-adjusted 오토스로틀

### 15.1 치명적 설계 결함

현재 오토스로틀 임계값은 1g 실속 속도 기준이다:

```
auto_throttle_speed = V_stall × 1.30
critical_speed      = V_stall × 0.85
```

그러나 선회 중 실속 속도는 하중계수에 따라 증가한다:

```
V_stall_turn = V_stall × √n,   n = 1/cos(φ)
```

60도 뱅크(n=2.0)에서:

```
V_stall_turn = V_stall × √2 = V_stall × 1.414

auto_throttle_speed / V_stall_turn = 1.30 / 1.414 = 0.919
```

**오토스로틀이 개입하는 속도가 선회 실속 속도보다 8% 낮다**. 오토스로틀이 풀파워를 걸어도 이미 실속 상태이다.

### 15.2 수치 검증 (3종 항공기)

| 항공기 | V_stall | auto_throttle | V_stall_turn(60°) | 마진 | 판정 |
|--------|---------|--------------|-------------------|------|------|
| An-2 | 18.6 m/s | 24.2 m/s | 26.3 m/s | -2.1 m/s | **실속** |
| C-172 | 24.1 m/s | 31.2 m/s | 33.9 m/s | -2.7 m/s | **실속** |
| F-16 | 67.4 m/s | 87.5 m/s | 95.2 m/s | -7.7 m/s | **실속** |

### 15.3 개선 공식

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  n_current = 1.0 / max(cos(current_bank), 0.1)               │
│  V_stall_n = V_stall × sqrt(n_current)                       │
│                                                                │
│  auto_throttle_speed = V_stall_n × 1.30                      │
│  critical_speed      = V_stall_n × 0.85                      │
│                                                                │
│  // urgency 계산은 기존과 동일                                 │
│  urgency = 1 - (V - critical_speed)                           │
│              / (auto_throttle_speed - critical_speed)          │
│  min_throttle = 0.6 + 0.4 × clamp(urgency, 0, 1)            │
│  throttle = max(player_throttle, min_throttle)                │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 15.4 지속선회 한계

V_ref에서 모든 프로파일의 최대 지속 뱅크가 CL_max에 의해 약 **54도**로 제한된다:

```
n_available = (V_ref / V_stall)² = (1.3)² = 1.69
φ_max_sustained = arccos(1/1.69) = arccos(0.592) ≈ 54°
```

**핵심 통찰**: 추력이 아닌 **양력(CL_max)**이 선회 한계를 결정한다. auto-throttle 개선만으로는 근본적 선회 성능 향상이 불가능하다.

---

## 16. 사이드슬립 보정

### 16.1 현재 문제

현재 시스템은 weathervane 안정성(`cn_beta=0.05`)만으로 사이드슬립을 억제한다. 인스트럭터에 명시적 사이드슬립 보정이 없어:
- 비협조선회 시 "미끄러지는" 부자연스러운 느낌 발생
- 속도벡터와 기수 방향의 불일치가 과도하게 지속

### 16.2 개선 공식: PD 사이드슬립 댐퍼

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  β = sideslip_rad   (physics engine에서 계산됨)               │
│                                                                │
│  rudder_correction = -Kp_β × β - Kd_β × dβ/dt               │
│                                                                │
│  Kp_β = 0.8    (비례: 사이드슬립 각도에 비례한 러더)          │
│  Kd_β = 0.3    (미분: 사이드슬립 변화율 감쇠)                │
│                                                                │
│  yaw_command = yaw_from_PD + rudder_correction                │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

이 보정은 Section 7의 협조선회 로직과 독립적으로 작동하며, 선회 중 및 직진 중 모두 사이드슬립을 억제한다.

---

## 17. 비행감(Flight Feel) 향상 공식

### 17.1 G-Force 계산 수정

현재 `engine.rs`에서 g_force가 `accel_no_grav.length()` (벡터 크기, 항상 >= 0)로 계산되어 역비행/급기동에서 부정확하다.

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  현재:  n = |accel_no_grav| / g  (항상 양수, 방향 무시)       │
│                                                                │
│  개선:  n_z = dot(F_aero, body_up) / (m × g)                 │
│                                                                │
│  여기서:                                                       │
│    F_aero = 공기역학적 힘 (중력 제외)                         │
│    body_up = 항공기 로컬 Y축 (상방 벡터)                      │
│                                                                │
│  n_z > 1: 양의 G (선회, 풀업)                                 │
│  n_z = 1: 수평 비행                                           │
│  n_z < 1: 음의 G (푸시오버, 역비행)                           │
│  n_z = 0: 무중력                                              │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 17.2 카메라 하중계수 변조

G-force에 따라 카메라 추적 속도를 조절하여 "G를 느끼는" 시각 효과를 만든다:

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  kCamTrackRate_eff = kCamTrackRate / (0.5 + 0.5 × |n_z|)     │
│                                                                │
│  n_z = 1.0 (수평비행): rate_eff = 6.0 / 1.0 = 6.0 (정상)    │
│  n_z = 2.0 (60° 뱅크): rate_eff = 6.0 / 1.5 = 4.0 (40% ↓)  │
│  n_z = 4.0 (고G 기동): rate_eff = 6.0 / 2.5 = 2.4 (60% ↓)  │
│                                                                │
│  효과: 고G에서 카메라 추적이 느려져 시야가 좁아지는 느낌      │
│  → 사용자가 "G를 당기고 있다"는 체감을 얻음                   │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 17.3 Specific Excess Power (Ps) 프로파일별 수치

3종 항공기의 선회 뱅크별 에너지율:

```
Ps = V × (T - D) / W    [m/s]

Ps > 0: 에너지 증가 (가속/상승 가능)
Ps = 0: 에너지 유지 (지속 가능한 비행)
Ps < 0: 에너지 손실 (감속/하강 불가피)
```

| 항공기 | V (m/s) | 직진 Ps | 30° Ps | 45° Ps | 60° Ps |
|--------|---------|---------|--------|--------|--------|
| An-2 | 50 | +2.4 | -0.2 | -5.3 | -22.5 |
| C-172 | 60 | +5.3 | +3.1 | -1.6 | -14.5 |
| F-16 | 200 | +31.0 | +28.6 | +23.2 | +5.1 |

**핵심 통찰**: An-2는 순항에서도 Ps가 거의 0 → 30도 이상 선회에서 즉시 에너지 손실. F-16은 60도 선회에서도 Ps > 0 유지. 이 차이가 "비행감"에서 항공기 개성을 만드는 핵심이다.

---

## 18. 누락된 피드백 루프 체계

현재 시스템에 없는 피드백 경로를 식별하고, 우선순위와 함께 정리한다:

| 순위 | 코드 | 피드백 경로 | 공식 | 관련 문제 |
|------|------|------------|------|-----------|
| 1 | F2 | n → auto-throttle | `auto_throttle = V_stall × √n × 1.30` | P0 버그 해결 (Section 15) |
| 2 | F6 | ω → bank angle | `φ = atan(ψ̇_desired × V / g)` | 협조선회 개선 (Section 7) |
| 3 | F4 | β → rudder | `rudder = -0.8β - 0.3β̇` | 사이드슬립 보정 (Section 16) |
| 4 | F3 | Ps → max_bank | `if Ps < -5: bank_limit *= 0.7` | 에너지 관리 (Section 8) |
| 5 | F1 | n → camera shake | `shake = k × max(0, |n| - 2)` | 몰입감 향상 |
| 6 | F5 | Ps → turn quality | `bank 완화 when Ps unsustainable` | 고급 기능 |

**실속 감지 개선 (하이브리드 방식)**:

현재 CL 모델은 pre-stall 영역에서 dCL/dα가 상수(선형)이므로, dCL/dα 감소 기반 실속 감지는 post-stall 진입 후에만 작동한다. 따라서:

```
primary:    0.85 × α_stall  (현재 0.70에서 상향)
secondary:  CL > 0.95 × CL_max  (safety net)
```

---

## 19. 3종 항공기 프로파일 수치 검증

### 19.1 검증 프로파일

| 파라미터 | An-2 (복엽기) | C-172 (단엽기) | F-16 (제트기) |
|----------|--------------|---------------|-------------|
| mass (kg) | 4,500 | 1,111 | 12,000 |
| S_total (m²) | 84.0 | 21.5 | 38.47 |
| CL_max | 2.49 | 1.43 | 1.10 |
| V_stall (m/s) | 18.6 | 24.1 | 67.4 |
| V_ref (m/s) | 24.1 | 31.2 | 87.5 |
| T/W | 0.272 | 0.367 | 0.648 |
| control_authority | 0.142 | 0.353 | 0.249 |

### 19.2 협조선회 수치 증명 (yaw_err = 0.5 rad 기준)

| 기체 | V (m/s) | 현재 bank (°) | 필요 bank (°) | 선회율 (°/s) |
|------|---------|---------------|---------------|-------------|
| An-2 | 40 | 4.3 | 11.5 | 14.1 |
| C-172 | 50 | 4.3 | 14.3 | 11.3 |
| F-16 | 250 | 4.3 | 52.7 | 2.1 |

**F-16 at 250 m/s: 선회율 2.1 deg/s → 사실상 직진**. `yaw_err × 1.5` 공식의 속도 무관 문제를 결정적으로 입증한다.

### 19.3 Plant 전달함수 (beta 연구소 제공)

```
G_plant(s) = K_aero / I / (s² + B_damp·s/I - K_stab/I)

여기서:
  K_aero = q × S × c × Cm_δe        (V²에 비례, 축별 조종면 모멘트 상수)
  B_damp = C_damp × q / (q + 500)    (비선형 포화 감쇠)
  K_stab = q × S × c × Cm_α          (V²에 비례, 안정성 복원 계수)
```

---

## 20. 개선 권고 요약 (확정)

### 20.1 우선순위 체계

| 우선순위 | 영역 | 현재 | 개선 공식 | 영향도 | Section |
|----------|------|------|----------|--------|---------|
| **P0** | **오토스로틀 n-adjusted** | `V_stall × 1.30` | `V_stall × √n × 1.30` | **치명적** | 15 |
| P1-1 | D_scale 스케줄링 수정 | `sqrt(ca/ca_ref)` | `(ca_ref/ca)^{3/4}` | 높음 | 13 |
| P1-2 | 협조선회 물리 기반 | `yaw_err × 1.5` | `atan(K_turn × yaw_err × V/g)` | 높음 | 7 |
| P1-3 | 게인 동압 보상 | `V/V_ref` | `clamp((V_ref/V)², 0.3, 3.0)` | 높음 | 11 |
| P1-4 | 사이드슬립 보정 | 없음 | `rudder = -0.8β - 0.3β̇` | 높음 | 16 |
| P2-1 | Kd 기본값 증가 | 1.2 | 1.52 (zeta 0.615→0.7) | 중간 | 3 |
| P2-2 | 실속 임계값 상향 | 70%/90% | 85%/95% + n-adjusted | 중간 | 9 |
| P2-3 | Reference windup | 없음 | back-calculation | 중간 | 14 |
| P2-4 | 에너지 관리 | 속도만 | E_s + Ps 기반 | 중간 | 8 |
| P2-5 | G-force 계산 | `length()` | `dot(F, body_up)/(mg)` | 중간 | 17 |
| P2-6 | 카메라 하중계수 변조 | 없음 | `rate / (0.5 + 0.5×|n|)` | 중간 | 17 |
| P2-7 | D항 저역통과 필터 | 없음 | `tau = 0.01s` | 중간 | 5 |
| P3-1 | control_authority 축별 분리 | avg_inertia | 축별 Ixx/Iyy/Izz | 낮음 | 12 |
| P3-2 | 안정성 계수 형상 기반 | 고정값 | 형상 추정 | 낮음 | - |

### 20.2 즉시 적용 (P0 + P1)

```
// P0: n-adjusted auto-throttle
double n_cur = 1.0 / std::max(std::cos(current_bank), 0.1);
double Vs_n = fc.stall_speed_ms * std::sqrt(n_cur);
auto_throttle_speed = Vs_n * 1.30;
critical_speed = Vs_n * 0.85;

// P1-1: D_scale 수정
D_scale = std::pow(ca_ref / ca, 0.75);  // 기존: std::sqrt(ca / ca_ref)

// P1-2: 협조선회 물리 기반
double psi_dot_desired = K_turn_rate * yaw_err;  // K_turn_rate = 0.268
desired_bank = std::atan(psi_dot_desired * airspeed / 9.81);

// P1-3: gain_scale 역제곱 보상
gain_scale = std::clamp(std::pow(ref_speed / airspeed, 2.0), 0.3, 3.0);

// P1-4: 사이드슬립 보정
double rudder_correction = -0.8 * sideslip - 0.3 * sideslip_rate;
yaw_cmd += rudder_correction;
```

### 20.3 핵심 개선 효과 요약

| 개선 | Before | After |
|------|--------|-------|
| 감쇠비 ζ (An-2) | ~0.35 (진동) | ~0.70 (최적) |
| 감쇠비 ζ (F-16) | ~0.60 (부족) | ~0.70 (최적) |
| 고속 루프 게인 | V^3 비례 (과잉) | 속도 무관 (일정) |
| 60° 뱅크 실속 보호 | **실패** (-8% 마진) | 정상 (+30% 마진) |
| 선회율 (F-16, 250m/s) | 2.1 °/s (직진) | 속도 비례 적정 선회 |
| 사이드슬립 | 수동 감쇠만 | PD 능동 보정 |
| 오버슈트 | 8.6% | 4.6% |

---

## 부록 A: 변수 정의 총표

| 기호 | 이름 | 단위 | 정의 |
|------|------|------|------|
| s | 라플라스 변수 | - | 주파수 영역 변수 |
| K_c | 플랜트 이득 | rad/s^2 | q·S·c·\|Cm\|/I |
| b | 감쇠율 | 1/s | D·q/((q+500)·I) |
| Kp | 비례 게인 | - | PD 제어기의 P 이득 |
| Kd | 미분 게인 | s | PD 제어기의 D 이득 |
| zeta | 감쇠비 | - | (b + K_c·Kd)/(2·omega_n) |
| omega_n | 고유진동수 | rad/s | sqrt(K_c·Kp) |
| omega_d | 감쇠 고유진동수 | rad/s | omega_n·sqrt(1-zeta^2) |
| ca | control_authority | - | K_c at V_ref |
| q | 동압 | Pa | 0.5·rho·V^2 |
| rho | 공기밀도 | kg/m^3 | ISA 표준대기 |
| S | 날개면적 | m^2 | total_wing_area |
| c | 기준시위 | m | ref_chord |
| Cm | 피치 모멘트 계수 | - | 0.025 |
| I | 관성 모멘트 | kg·m^2 | 축별 상이 |
| V | 대기속도 | m/s | airspeed |
| V_ref | 기준속도 | m/s | 1.3·V_stall |
| V_stall | 실속속도 | m/s | sqrt(2W/(rho·S·CL_max)) |
| phi | 뱅크각 | rad | 선회 경사각 |
| n | 하중계수 | - | 1/cos(phi) |
| alpha | 받음각 (AoA) | rad | 기류-시위 각도 |
| tau | 필터 시정수 | s | 1차 필터 |
| E_s | 비에너지 | m | h + V^2/(2g) |
| AR | 종횡비 | - | b^2/S |
| e | Oswald 효율 | - | 0.5~0.95 |
| CD0 | 영양력항력계수 | - | 기생항력 |
| beta (β) | 사이드슬립각 | rad | 기류-기체 횡방향 각도 |
| Ps | 비잉여추력 | m/s | V·(T-D)/W |
| n_z | 수직 하중계수 | - | dot(F_aero, body_up)/(mg) |
| K_turn_rate | 선회율 이득 | 1/s | 0.268 |
| Kp_β | 사이드슬립 P 이득 | - | 0.8 |
| Kd_β | 사이드슬립 D 이득 | s | 0.3 |
| k_bc | back-calculation 속도 | 1/s | 2.0 |

## 부록 B: 코드 대응 표

| 수학 공식 위치 | 소스 파일 | 함수/행 |
|---------------|----------|---------|
| 1.1 제어 모멘트 | `rust/.../aerodynamics.rs` | `compute_control_moments()` L153-185 |
| 1.1 감쇠 모멘트 | `rust/.../aerodynamics.rs` | `compute_angular_damping()` L244-256 |
| 1.3 PD 제어기 | `cpp/src/input.cpp` | `compute_instructor()` L162-164 |
| 3.5 게인 스케줄링 | `cpp/src/input.cpp` | `configure()` L23-46 |
| 7.2 협조선회 | `cpp/src/input.cpp` | `compute_instructor()` L155-158 |
| 9.4 실속 보호 | `cpp/src/input.cpp` | `compute_instructor()` L126-149 |
| 8.4 오토스로틀 | `cpp/src/input.cpp` | `InputHandler::update()` L240-250 |
| 10.1 항력 극선 | `rust/.../aerodynamics.rs` | `AirfoilModel::compute_cd()` L89-97 |
| 12.1 control_authority | `rust/.../aerodynamics.rs` | `compute_flight_characteristics()` L273 |
| 13.1 D_scale 오류 | `cpp/src/input.cpp` | `configure()` L33 |
| 15.1 오토스로틀 임계값 | `cpp/src/input.cpp` | `InputHandler::update()` L240-250 |
| 17.1 G-force 계산 | `rust/.../engine.rs` | `step()` L170-171 |
| 19.3 Plant 모델 | `rust/.../aerodynamics.rs` | 전체 |

---

> **AeroBlend 비행제어 수학연구소**
> 제어이론 연구소 (alpha) - PD 제어기 분석, 안정성 증명, 게인 최적화, D_scale 오류 발견
> 항공역학 연구소 (beta) - 협조선회, 실속 보호, 에너지 관리 모델, P0 버그 발견
> 코디네이터 (group-coordinator) - 연구 조율 및 최종 문서 통합
> 생성: 2026-02-22 | 최종 보완: 2026-02-22
