# 레퍼런스 vs AeroBlend 비교 분석 및 개선안 도출

작성일: 2026-03-01
분석자: math-lead / code-analysis-lead (CEO 종합)
입력 문서: 01~04 분석 보고서

---

## 1. 비교 대상 시스템 요약

| 시스템 | 플랫폼 | 제어 구조 | 주요 특징 |
|--------|--------|----------|----------|
| MouseFlight | Unity | P-only | 최소 구현, 교육용 레퍼런스 |
| MouseFlightReloaded | Unity | P-only | MouseFlight와 동일 (변경 없음) |
| MouseAimFlight Old | KSP (C#) | PID + FlightModes | speedFactor, 3가지 비행 모드, 적분항 |
| MouseAimFlight New | KSP (C#) | P-only | 리팩터링, D/I 비활성, sqrt(yaw) 롤 |
| AeroBlend | Rust+C++ | PD + 적응형 | 축별 ca 스케줄링, 비행 보호, 협조선회 |

---

## 2. 제어 구조 비교

### 2.1 제어기 타입

```
MouseFlight:     u = Kp × e_local
MAF Old:         u = (Kp×e + Kd×ω + Ki×∫e) × speedFactor
MAF New:         u = Kp × e  (D/I 주석처리)
AeroBlend:       u = (Kp×e - LPF(Kd×ω)) × gain_scale + sideslip_correction
```

**분석**: AeroBlend은 레퍼런스 중 유일하게 실효적인 미분항(D)을 사용하며, LPF로 노이즈를 억제한다. MAF Old는 PID를 구현했으나, 새 버전에서 D/I를 비활성화한 것은 튜닝 어려움을 시사한다.

### 2.2 게인 스케줄링

| 방식 | 시스템 | 수식 | 물리적 정확성 |
|------|--------|------|-------------|
| 없음 | MouseFlight, MAF New | 고정 게인 | 낮음 |
| speedFactor | MAF Old | V/(q×16) ≈ 2/(ρV) | 중간 (근사) |
| Inverse-square | AeroBlend | (V_ref/V)² | 높음 (동압 역수) |
| 축별 ca 스케줄링 | AeroBlend | Kp ∝ 1/√ca | 높음 |

**분석**: AeroBlend의 이중 스케줄링(속도 + 기체 특성)은 가장 정교하다. MAF Old의 speedFactor는 개념적으로 유사하지만 물리적 정확성이 낮다.

---

## 3. 에러 계산 방식 비교

### 3.1 피치/요 오차

| 시스템 | 방법 | 좌표계 |
|--------|------|--------|
| MouseFlight | InverseTransformPoint → local XYZ | 항공기 로컬 |
| MAF Old | asin(dot(axis, ProjectOnPlane(...))) | 항공기 로컬 (각도) |
| MAF New | SignedAngle(vessel.up, projected, axis) | 항공기 로컬 (각도) |
| AeroBlend | atan2(ly, √(lx²+lz²)), atan2(lx, lz) | 항공기 로컬 (각도) |

**분석**: AeroBlend과 MAF의 접근법은 수학적으로 동치이다. MouseFlight는 normalized position 벡터를 직접 사용하여 비선형 왜곡이 있지만, sensitivity 파라미터로 보상한다.

### 3.2 롤/뱅크 계산

| 시스템 | 방법 | 협조선회 |
|--------|------|---------|
| MouseFlight | aggressive/wingsLevel 혼합 | 없음 |
| MAF Old | upWeighting 벡터 + sideslip 보정 | 간접적 (경험적) |
| MAF New | rollErr × √\|yawErr\| | 없음 |
| AeroBlend | atan(K×yaw_err×V/g) 물리 기반 | 직접적 (물리 공식) |

**핵심 차이**: AeroBlend만이 정상 선회의 물리 공식 `tan(φ) = Vψ̇/g`에서 desired_bank를 유도한다. MAF Old의 upWeighting은 경험적 튜닝에 의존하며, MouseFlight의 혼합 방식은 최소 구현이다.

---

## 4. 비행 보호 비교

| 기능 | MouseFlight | MAF Old | MAF New | AeroBlend |
|-----|------------|---------|---------|-----------|
| 실속 AoA 보호 | 없음 | 없음 | 없음 | 85%/95% progressive |
| 저속 보호 | 없음 | 없음 | 없음 | 피치업 제한 + 노즈다운 |
| Anti-windup | 없음 | 포화시 적분=0 | 없음 | Back-calculation |
| 자동 스로틀 | 없음 | 없음 | 없음 | n-adjusted |
| 피치 제한 | 없음 | 없음 | 없음 | 속도 기반 동적 |

**분석**: 비행 보호는 AeroBlend의 가장 큰 차별점이다. 레퍼런스 시스템은 어느 것도 실속 보호를 구현하지 않았다.

---

## 5. 레퍼런스에서 발견된 AeroBlend 미반영 기법

### 5.1 MAF Old: 비행 모드 시스템

MAF Old는 3가지 비행 모드(Normal/Cruise/Aggressive)를 제공한다:

- **Normal**: 균형 잡힌 기본 비행, pitchError -= |rollError|/3 (피치-롤 커플링 보정)
- **Cruise**: 수평선 기준 피치 = (horizon_pitch + target_pitch)/2 (고도 안정화)
- **Aggressive**: pitchError += 1 - exp(-pitchError) (비선형 피치 부스트)

**AeroBlend 적용 가능성**: **높음**
- 현재 AeroBlend는 단일 모드만 지원
- 상황별 비행 모드 전환은 사용자 경험(UX)을 크게 향상시킬 수 있음
- 구현 난이도: 중간 (기존 인스트럭터에 모드 분기 추가)

### 5.2 MAF Old: upWeighting (수평 유지력 가변)

```
rollTarget = targetPos + max(upWeighting × (100 - |yaw×1.6| - pitch×2.8), 0) × upDir
```

큰 기동 중에는 "날개 수평 유지" 힘을 줄여 더 자유로운 선회를 허용한다.

**AeroBlend 적용 가능성**: **중간**
- AeroBlend의 물리 기반 desired_bank가 이미 유사한 효과를 제공
- 단, 수직 기동(루프 등)에서의 수평 복귀 동작은 별도 고려 가능

### 5.3 MAF New: sqrt(yaw) 롤 스케일링

```
rollErr = targetData.rollErr × √|yawErr|
```

요 오차가 없으면 롤 명령 = 0 (자동 날개 수평), 요 오차에 비례하여 점진적 뱅크.

**AeroBlend 적용 가능성**: **낮음**
- AeroBlend의 물리 기반 desired_bank가 이미 더 정교한 메커니즘 제공
- sqrt 스케일링은 단순화된 근사이므로 물리 기반 접근이 우월

### 5.4 MouseFlight: 프레임 독립 카메라 감쇠

```
α = 1 - exp(-λ × dt)    // λ = 5.0
cameraRig.rotation = Slerp(current, target, α)
```

**AeroBlend 적용 가능성**: **확인 필요**
- AeroBlend의 카메라가 유사한 스무딩을 사용하는지 검증 필요
- 카메라 G-feel (tracking rate / (0.5 + 0.5|n|)) 이미 구현됨

### 5.5 MAF Old: 각속도 기반 D항

MAF Old는 `derivError = angularVelocity` (직접 측정)를 사용하고, AeroBlend도 `omega_pitch = state.angular_velocity.x`를 사용한다. 이 부분은 이미 동일하게 구현됨.

### 5.6 MAF Old: 적분항 (I)

MAF Old PID에서 적분항:
```
integral += error × dt
integral = clamp(integral, integralLimit / (ki × speedFactor))
output += integral × ki × speedFactor
```

**AeroBlend 적용 가능성**: **중간-높음**
- AeroBlend는 현재 I항 없음 (PD만)
- 정상상태 오차 제거에 적분항이 유용할 수 있음
- 단, anti-windup이 필수이며 MAF Old의 "포화시 리셋"보다 AeroBlend의 back-calculation이 더 안정적
- 적분항 추가 시 기존 back-calculation을 확장하여 사용 권장

---

## 6. 개선안 우선순위

### P4-1: 비행 모드 시스템 (우선도: 높음)

**개요**: Normal/Cruise/Aggressive 모드 도입

**구현 방안**:
1. MouseAimComputer에 `FlightMode` enum 추가 (Normal, Cruise, Aggressive)
2. Normal: 현재 동작 유지
3. Cruise: pitch_err에 수평선 기준 피치 혼합
   ```
   horizon_pitch = acos(dot(fwd, up_world)) - acos(dot(target, up_world))
   pitch_err = (horizon_pitch + target_pitch_err) / 2
   ```
4. Aggressive: 피치 응답 비선형 부스트
   ```
   pitch_err += sign(pitch_err) × (1 - exp(-|pitch_err|))
   ```
5. 키 바인딩: Tab 또는 별도 키로 모드 전환

**기대 효과**: 순항 시 안정성 향상, 공중전 시 민첩성 향상

### P4-2: 적분항(I) 도입 (우선도: 중간)

**개요**: PD → PID 확장으로 정상상태 오차 제거

**구현 방안**:
1. 축별 적분기 상태 변수 추가
2. 기존 back-calculation anti-windup 확장
3. 게인: Ki_pitch = 0.3, Ki_roll = 0.1, Ki_yaw = 0.15 (MAF Old 참고하되 스케일 조정)
4. 적분 제한: 출력의 20%까지 (MAF Old의 intLimit = 0.2와 일치)
5. 속도 스케일링: 적분항에도 gain_scale 적용

**주의사항**: 적분항은 진동을 유발할 수 있으므로 보수적 게인으로 시작

### P4-3: 피치-롤 커플링 보정 (우선도: 중간)

**개요**: MAF NormalFlight의 `pitchError -= |rollError|/3` 적용

**수학적 근거**: 대각도 뱅크 중 피치업 명령은 실질적으로 수평면에서의 선회를 증가시킨다. 롤 오차에 비례하여 피치를 줄이면 과도한 G-force를 방지한다.

**구현 방안**:
```
pitch_err -= abs(bank_err) × 0.3   // 기존 compute_instructor()에 추가
```

### P4-4: 적응형 감쇠비 (우선도: 낮음)

**개요**: ca 기반 감쇠비를 ζ=0.7(임계감쇠 근방)로 자동 유지

**현재**: Kd_base 고정값 사용
**개선**: `ζ = Kd / (2 × √(Kp × J/ca))` 에서 역산하여 Kd 조정

### P4-5: 에너지 관리 모드 (우선도: 낮음)

**개요**: 속도/고도 에너지 상태에 따른 자동 기동 제한

**구현 방안**: specific energy `Es = h + V²/(2g)` 모니터링, 에너지 부족 시 기동 제한 강화

---

## 7. 종합 평가

### 7.1 AeroBlend의 강점 (레퍼런스 대비)
1. **물리 기반 설계**: 모든 공식이 공기역학 원리에서 유도 (vs 경험적 튜닝)
2. **적응형 시스템**: 기체 특성 자동 감지 및 게인 조정
3. **비행 보호 체계**: 실속/저속/windup 보호가 통합된 유일한 구현
4. **협조선회**: 물리 공식 기반 desired_bank (vs upWeighting 경험적 방식)
5. **D-term 품질**: LPF 필터링된 미분항 (레퍼런스는 노이즈 필터 없음)

### 7.2 AeroBlend의 약점 (레퍼런스 대비)
1. **단일 비행 모드**: MAF의 3모드 시스템 대비 유연성 부족
2. **적분항 부재**: 정상상태 오차 존재 가능 (바람 등 외란 시)
3. **피치-롤 커플링 미보정**: 대각도 선회 시 과도 G 가능

### 7.3 결론

AeroBlend의 현재 구현(P0-P3)은 레퍼런스 시스템들을 제어 이론 및 비행 역학 관점에서 크게 앞선다. 레퍼런스에서 차용할 수 있는 핵심 아이디어는:

1. **비행 모드 시스템** (P4-1) — 사용자 경험 향상에 가장 큰 영향
2. **적분항** (P4-2) — 정밀도 향상, 단 신중한 튜닝 필요
3. **피치-롤 커플링 보정** (P4-3) — 간단한 구현으로 즉각적 효과

이 외의 레퍼런스 기법(sqrt 롤 스케일링, speedFactor 등)은 AeroBlend의 기존 물리 기반 접근이 이미 우월하므로 차용 불필요하다.
