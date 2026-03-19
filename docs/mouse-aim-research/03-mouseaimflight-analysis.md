# MouseAimFlight (KSP) 소스 코드 수학적 분석

작성일: 2026-03-01
분석자: code-analyst
소스: `references/MouseAimFlight/MouseAimFlight/`

---

## 1. 시스템 개요

MouseAimFlight는 KSP용 C# 마우스 에임 비행 플러그인이다 (BahamutoD, ferram4, tetryds).

두 버전이 존재한다:
- **Old 버전** (`Old/`): AdaptivePID + 3가지 비행 모드 (NormalFlight, CruiseFlight, AggressiveFlight)
- **리팩터링 버전** (현재): TargetModule + BasicFlight + InputPid/ControlFilter

---

## 2. 리팩터링 버전 — TargetModule의 SignedAngle 기반 오차 계산

### 2.1 피치/요/롤 오차 (pitch/yaw/roll 분리)

```csharp
// 피치 오차
Vector3 targetYZ = Vector3.ProjectOnPlane(targetDir, vessel.right);
float pitchErr = -Vector3.SignedAngle(vessel.up, targetYZ, vessel.right);

// 요 오차
Vector3 targetXY = Vector3.ProjectOnPlane(targetDir, -vessel.forward);
float yawErr = Vector3.SignedAngle(vessel.up, targetXY, -vessel.forward);

// 롤 오차
Vector3 targetXZ = Vector3.ProjectOnPlane(targetDir, vessel.up);
float rollErr = -Vector3.SignedAngle(-vessel.forward, targetXZ, vessel.up);
```

각 축의 오차를 독립적으로 계산하기 위해 해당 축에 수직인 평면에 타깃을 투영한 뒤
SignedAngle로 부호 있는 각도를 측정한다.

피치: vessel.up 기준으로 right-YZ 평면의 타깃 각도
요:   vessel.up 기준으로 forward-XY 평면의 타깃 각도
롤:   -vessel.forward 기준으로 up-XZ 평면의 타깃 각도

---

## 3. 리팩터링 버전 — BasicFlight의 rollYawScale = sqrt(|yawErr|)

```csharp
float rollYawScale = Mathf.Sqrt(Mathf.Abs(yawErr));
float rollErr = targetData.rollErr * rollYawScale;
```

roll_cmd = rollErr * sqrt(|yawErr|)

sqrt(|yawErr|) 사용 이유:
- yawErr = 0  -> rollYawScale = 0 -> 롤 없음 (정면 향할 때 날개 수평)
- yawErr = 4  -> rollYawScale = 2
- yawErr = 16 -> rollYawScale = 4
- 선형 대비: 소 오차에서 빠른 반응, 대 오차에서 완만한 증가 -> 과도 뱅크 억제

---

## 4. 리팩터링 버전 — InputPid에서 D/I항 주석처리 (P-only)

### 4.1 ControlFilter 파라미터

```
pitch: kp=0.2,  ki=0.1,  kd=0.08
roll:  kp=0.01, ki=0.0,  kd=0.005
yaw:   kp=0.035,ki=0.1,  kd=0.04
```

### 4.2 InputPid.ComputeValue 실제 동작

```csharp
float direct     = error * kp;
float derivErr   = (error - previousErr) / deltaTime;
float derivative = derivErr * kd;   // 계산되지만 사용 안 함

float value = 0;
value += direct;
// value += derivative;   // ← 주석 처리됨
// value += integral;     // ← 주석 처리됨

previousErr = error;
return value;
```

실제 출력: u = Kp * e (P-only)

D/I항 주석 처리 이유 (추정):
1. KSP 물리의 자체 감쇠 특성으로 D항 없이도 동작 가능
2. deltaTime 기반 수치 미분의 노이즈 증폭 문제 회피
3. 단순화 목적으로 제거 후 미복원

한계: 정상상태 오차 잔류, 과진동 미억제

---

## 5. Old 버전 — 3가지 비행 모드 수학적 차이

### 5.1 공통 오차 계산

```
pitchError = asin(dot(-back, ProjectOnPlane(targetDir, right))) * Rad2Deg
yawError   = asin(dot( right, ProjectOnPlane(targetDir, forward))) * Rad2Deg
sideslip   = asin(dot(vessel.right, vel.normalized)) * Rad2Deg
```

### 5.2 NormalFlight — 롤 계산

```
rollTarget = (targetPos
    + max(upWeighting * (100 - |yawErr*1.6| - pitchErr*2.8), 0) * upDir) - CoM
rollTarget = ProjectOnPlane(rollTarget, vessel.up)
rollError  = SignedAngle(-vessel.forward, rollTarget, vessel.right)
             - sideslip * sqrt(velocity) / 5

pitchDownFactor = pitchErr * (10/(yawErr^2 + 10) - 0.1)
rollError  += sign(rollError) * |clamp(pitchDownFactor, -15, 0)|
pitchError -= |clamp(rollError, -pitchError, pitchError)| / 3
```

upWeighting 동적 감소: 요/피치 오차 클수록 W_eff 감소 -> 날개 수평 유지력 약화 -> 빠른 선회
사이드슬립 보정: rollError -= beta * sqrt(V) / 5

### 5.3 CruiseFlight — 복합 피치 오차

```
pitchErrorHorizon = (acos(dot(vessel.up, upAxis)) - acos(dot(target, upAxis))) * Rad2Deg
pitchErrorTarget  = asin(dot(-back, ProjectOnPlane(targetDir, right))) * Rad2Deg
yawError          = 1.5 * asin(dot(right, ProjectOnPlane(targetDir, forward))) * Rad2Deg
pitchError = (pitchErrorHorizon + pitchErrorTarget) / 2
```

pitchError = 수평 기준 오차와 타깃 방향 오차의 평균 -> 완만한 상승/하강
yawError에 1.5 배율 -> 더 빠른 방위각 수정
롤 upWeighting: 2 * upWeighting * (100 - |yaw*1.8|) (피치 항 없음, 2배 계수)

### 5.4 AggressiveFlight — 피치 비선형 부스트

NormalFlight와 동일하지만 pitchError 감소 항 제거 + 비선형 부스트 추가:

```csharp
// pitchError -= |clamp(rollError, -pitchError, pitchError)| / 3;  // Normal에 있음
pitchError += 1 - Mathf.Exp(-pitchError);  // Aggressive 추가
```

pitchError_boosted = pitchError + (1 - exp(-pitchError))

| pitchError | 부스트 | 유효 오차 |
|-----------|-------|---------|
| 0.0 | 0.000 | 0.000 |
| 0.5 | 0.394 | 0.894 |
| 1.0 | 0.632 | 1.632 |
| 2.0 | 0.865 | 2.865 |

효과: 피치 오차가 클수록 최대 +1 추가 피치 명령 -> 급기동 응답성 향상

---

## 6. Old 버전 — AdaptivePID의 speedFactor 및 Anti-windup

### 6.1 속도 보상 팩터

```csharp
float speedFactor = vel / dynPress / 16;   // dynPress = 0.5 * rho * V^2
if (speedFactor > 1.2f) speedFactor = 1.2f;
float trimFactor  = Mathf.Sqrt(speedFactor);
```

speedFactor = V / (0.5 * rho * V^2) / 16 = 2 / (16 * rho * V) = 1 / (8 * rho * V)

고속(V 증가) -> speedFactor 감소 -> 전체 게인 감소 -> 공력 과민 방지

### 6.2 PID.Simulate 전체 공식

```csharp
integral += error * timeStep;
if (ki != 0) Clamp(ref integral, integralLimit / (ki * speedFactor));
else         ZeroIntegral();

output  = error * kp;
output += derivError * kd;      // derivError = angular velocity (각속도 직접 입력)
output *= speedFactor;
output += integral * ki * speedFactor;

if (output >= 1) ZeroIntegral();  // 포화 시 적분 리셋 (anti-windup)
Clamp(ref output, 1);
```

u = (Kp*e + Kd*omega) * f_speed + Ki * integral * f_speed

Anti-windup 방식: |u| >= 1 시 integral = 0 즉시 리셋
적분 클램프: |integral| <= integralLimit / (Ki * f_speed)

각속도 D항: 계산된 de/dt 대신 angular velocity 직접 사용 -> 수치 미분 노이즈 없음

### 6.3 AdaptivePID 기본 게인 파라미터

| 축 | kp | ki | kd | intLimit |
|----|----|----|-----|---------|
| pitch | 0.2 | 0.1 | 0.08 | 0.2 |
| roll | 0.01 | 0.0 | 0.005 | 0.2 |
| yaw | 0.035 | 0.1 | 0.04 | 0.1 |

---

## 7. 두 버전 비교 요약

| 항목 | Old 버전 | 리팩터링 버전 |
|-----|---------|------------|
| 비행 모드 수 | 3 (Normal, Cruise, Aggressive) | 1 (Basic) |
| 오차 계산 | asin 기반 각도 | SignedAngle (더 안정적) |
| 롤 전략 | 복합 공식 + upWeighting | rollErr * sqrt(|yawErr|) |
| 속도 보상 | V / q / 16 | 없음 |
| 제어기 | PID (D=각속도, Anti-windup 포함) | P-only (D/I 주석처리) |
| 피치 부스트 | 1 - exp(-e) (Aggressive 모드) | 없음 |
| 사이드슬립 보정 | rollErr -= beta * sqrt(V) / 5 | 없음 |
