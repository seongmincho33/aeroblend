# MouseFlight (Unity) 소스 코드 수학적 분석

작성일: 2026-03-01
분석자: code-analyst
소스: `references/MouseFlight/Assets/MouseFlight/Scripts/MouseFlightController.cs`
      `references/MouseFlight/Assets/MouseFlight/Demo/Scripts/Plane.cs`

---

## 1. 시스템 개요

MouseFlight는 Unity용 War Thunder 스타일 마우스 에임 조종 시스템의 레퍼런스 구현이다.
두 컴포넌트로 역할이 분리되어 있다:

- **MouseFlightController.cs** — 마우스 입력 → mouseAim Transform 회전 → 카메라 릭 추종
- **Plane.cs** — MouseAimPos → 항공기 조종면 출력 (오토파일럿)

이 분리 구조 덕분에 카메라와 비행 로직이 독립적으로 동작한다.

---

## 2. 마우스 입력 → mouseAim Transform 회전 메커니즘

### 2.1 입력 수집

```csharp
float mouseX =  Input.GetAxis("Mouse X") * mouseSensitivity;   // mouseSensitivity = 3
float mouseY = -Input.GetAxis("Mouse Y") * mouseSensitivity;   // 위 방향 = 양수
```

### 2.2 카메라 축 기반 World Space 회전

```csharp
mouseAim.Rotate(cam.right, mouseY, Space.World);  // 카메라 right축 기준 피치
mouseAim.Rotate(cam.up,    mouseX, Space.World);  // 카메라 up축 기준 요
```

**핵심**: `Space.World`로 회전하고 **카메라의 축**을 사용한다.
- 로컬 축 사용 시 카메라가 기울면 마우스 조작이 비직관적이 됨
- 카메라 world-space 축 사용으로 화면 기준 상하좌우가 항상 직관적으로 매핑됨

수학적으로 이는 두 번의 쿼터니언 곱이다:

$$q_{\text{new}} = R(\hat{u}_{\text{cam}},\, \delta x) \cdot R(\hat{r}_{\text{cam}},\, \delta y) \cdot q_{\text{old}}$$

여기서 $\hat{r}_{\text{cam}}$은 카메라 right 벡터, $\hat{u}_{\text{cam}}$은 카메라 up 벡터.

### 2.3 에임 위치 계산

$$\vec{p}_{\text{aim}} = \vec{p}_{\text{mouseAim}} + \hat{f}_{\text{mouseAim}} \cdot d$$

- $d$ = `aimDistance` = 500 m
- mouseAim Transform이 가리키는 방향으로 500m 투영한 월드 좌표

---

## 3. 프레임 독립 감쇠 SLERP (Rory Driscoll 방식)

### 3.1 upVec 결정

```csharp
Vector3 upVec = (Mathf.Abs(mouseAim.forward.y) > 0.9f)
              ? cameraRig.up   // 거의 수직 → 자기 up 사용
              : Vector3.up;    // 일반적인 경우 → 월드 up 사용
```

수직 방향 극점에서의 gimbal 불안정 방지.

### 3.2 지수 감쇠 SLERP

```csharp
cameraRig.rotation = Quaternion.Slerp(
    cameraRig.rotation,
    Quaternion.LookRotation(mouseAim.forward, upVec),
    1f - Mathf.Exp(-camSmoothSpeed * Time.deltaTime)  // camSmoothSpeed = 5
);
```

$$\alpha = 1 - e^{-\lambda \Delta t}, \quad \lambda = 5$$

$$q(t + \Delta t) = \text{Slerp}\!\left(q(t),\; q_{\text{target}},\; 1 - e^{-\lambda \Delta t}\right)$$

**프레임레이트 독립성 증명**:
$n$회 적용 후 잔류 오차 비율:

$$\left(1 - \alpha\right)^n = \left(e^{-\lambda \Delta t}\right)^n = e^{-\lambda \cdot n\Delta t} = e^{-\lambda T}$$

총 경과 시간 $T = n\Delta t$에만 의존 → 프레임레이트 불변.

**시상수 특성** ($\lambda = 5$):

| 경과 시간 | 수렴도 |
|---------|-------|
| $\tau = 0.2$ s | 63.2% |
| $3\tau = 0.6$ s | 95.0% |
| $5\tau = 1.0$ s | 99.3% |

---

## 4. 오토파일럿 — InverseTransformPoint 기반 비례 제어 (P-only)

### 4.1 로컬 좌표 변환

```csharp
var localFlyTarget = transform.InverseTransformPoint(flyTarget).normalized * sensitivity;
var angleOffTarget = Vector3.Angle(transform.forward, flyTarget - transform.position);
```

$$\vec{l} = M_{\text{inv}} \cdot (\vec{p}_{\text{aim}} - \vec{p}_{\text{aircraft}}), \quad \hat{l} = \frac{\vec{l}}{|\vec{l}|} \cdot s$$

$M_{\text{inv}}$: 항공기 월드→로컬 변환 행렬, $s$ = `sensitivity` = 5

### 4.2 피치/요 비례 제어 (P-only)

```csharp
yaw   =  Mathf.Clamp(localFlyTarget.x, -1f, 1f);
pitch = -Mathf.Clamp(localFlyTarget.y, -1f, 1f);
```

$$u_{\text{yaw}} = \text{clamp}(l_x,\, -1,\, 1)$$
$$u_{\text{pitch}} = \text{clamp}(-l_y,\, -1,\, 1)$$

**P-only 한계**: 코드 주석에 명시됨 —
> "Use of a PID controller for each axis is highly recommended."
- 미분항 없음 → 과진동(overshoot) 발생 가능
- 적분항 없음 → 정상상태 오차(steady-state error) 잔류 가능

---

## 5. 롤 전략 — Aggressive vs Wings-Level 블렌딩

### 5.1 두 가지 롤 명령

```csharp
// 뱅크 기동: 타깃 방향으로 롤 (localFlyTarget.x가 타깃의 수평 편차)
var aggressiveRoll = Mathf.Clamp(localFlyTarget.x, -1f, 1f);

// 날개 수평: right 벡터의 y성분 = 0이 되도록 (수평 유지)
var wingsLevelRoll = transform.right.y;
```

### 5.2 InverseLerp 기반 angleOffTarget 블렌딩

```csharp
var wingsLevelInfluence = Mathf.InverseLerp(0f, aggressiveTurnAngle, angleOffTarget);
// aggressiveTurnAngle = 10°
roll = Mathf.Lerp(wingsLevelRoll, aggressiveRoll, wingsLevelInfluence);
```

$$w = \text{clamp}\!\left(\frac{\theta_{\text{off}}}{10°},\; 0,\; 1\right)$$

$$u_{\text{roll}} = (1 - w) \cdot r_{\text{level}} + w \cdot r_{\text{aggressive}}$$

**블렌딩 거동**:

| $\theta_{\text{off}}$ | $w$ | 우세 모드 |
|----------------------|-----|---------|
| 0° | 0.0 | Wings-level (날개 수평) |
| 5° | 0.5 | 50/50 혼합 |
| ≥10° | 1.0 | Aggressive (뱅크 기동) |

**설계 의도**: 타깃이 거의 정면에 있을 때는 날개를 수평으로 유지하고,
크게 벗어났을 때는 강하게 뱅크하여 타깃으로 빠르게 선회.

---

## 6. 물리 적용 파라미터

```csharp
rigid.AddRelativeTorque(
    new Vector3(turnTorque.x * pitch, turnTorque.y * yaw, -turnTorque.z * roll) * forceMult
);
```

| 파라미터 | 값 |
|---------|---|
| `turnTorque` (pitch, yaw, roll) | (90, 25, 45) |
| `forceMult` | 1000 |
| `thrust` | 100 |

---

## 7. 핵심 수학 정리

| 구성요소 | 수식 |
|--------|------|
| mouseAim 회전 | $q = R(\hat{u}_\text{cam}, \delta x) \cdot R(\hat{r}_\text{cam}, \delta y) \cdot q_\text{old}$ |
| 에임 위치 | $\vec{p}_\text{aim} = \vec{p} + \hat{f} \cdot 500$ |
| 카메라 감쇠 | $\alpha = 1 - e^{-5\Delta t}$ |
| 피치/요 | $u = \text{clamp}(l_{x,y} \cdot s,\, -1,\, 1)$ |
| 롤 블렌딩 | $u_\text{roll} = \text{lerp}(r_\text{level},\, r_\text{aggressive},\, \theta_\text{off}/10°)$ |

---

## 8. 한계점

| 항목 | 현황 | 영향 |
|-----|------|------|
| 제어 구조 | P-only | 과진동, 정상상태 오차 |
| 속도 보상 | 없음 | 고속/저속 응답성 불균일 |
| 실속 보호 | 없음 | 저속 기동 시 실속 위험 |
| 기체 특성 반영 | 없음 | 모든 기체 동일 파라미터 |
| 협조선회 | 없음 | 요잉 기동 시 사이드슬립 발생 |
