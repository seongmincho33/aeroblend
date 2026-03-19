# MouseAim 연구소 최종 종합 보고서

작성일: 2026-03-01
작성: CEO / 전 부서 종합

---

## Executive Summary

3개의 오픈소스 마우스 에임 비행 제어 시스템(MouseFlight, MouseFlightReloaded, MouseAimFlight)을 수학적으로 분석하고, AeroBlend의 현재 구현(P0-P3)과 비교하였다. 분석 결과, AeroBlend는 제어 이론 및 비행 역학 관점에서 모든 레퍼런스를 크게 앞서고 있으며, 레퍼런스에서 차용 가능한 핵심 아이디어 3가지(비행 모드, 적분항, 피치-롤 커플링 보정)를 도출하였다.

---

## 1. 분석 범위 및 방법론

### 1.1 분석 대상

| # | 시스템 | 출처 | 분석 문서 |
|---|--------|------|----------|
| 1 | MouseFlight | Unity, Brian Hernandez (MIT) | 01-mouseflight-analysis.md |
| 2 | MouseFlightReloaded | Unity, Salzian (fork) | 02-mouseflight-reloaded-analysis.md |
| 3 | MouseAimFlight | KSP, BahamutoD/ferram4/tetryds (BSD) | 03-mouseaimflight-analysis.md |
| 4 | AeroBlend (현재) | Rust+C++, 자체 개발 | 04-aeroblend-current-analysis.md |

### 1.2 분석 방법

- 전 소스 코드 줄 단위 리뷰
- diff 비교 (MouseFlight vs Reloaded: 100% 동일 확인)
- 수학 공식 추출 및 검증
- 제어 이론 관점 평가 (안정성, 강인성, 적응성)

---

## 2. 핵심 발견사항

### 2.1 MouseFlight / MouseFlightReloaded

**아키텍처**: 카메라 리그 + 오토파일럿 2계층 분리

**수학 핵심**:
- 마우스 → 에임 방향: 카메라 축 기반 월드 회전
- 카메라 스무딩: `Slerp(a, b, 1 - exp(-5dt))` (Rory Driscoll 프레임 독립 감쇠)
- 오토파일럿: `InverseTransformPoint(target).normalized × sensitivity` → P-only 비례 제어
- 롤: aggressive/wingsLevel 혼합 (`InverseLerp` 블렌딩, threshold 10°)

**한계**: PID 미사용, 속도 보상 없음, 실속 보호 없음, 협조선회 없음

**참고**: MouseFlightReloaded는 원본과 바이트 단위 동일 (코드 변경 없는 포크)

### 2.2 MouseAimFlight (KSP)

**아키텍처**: 2세대 존재 (Old: AdaptivePID + 3 FlightModes, New: TargetModule + P-only)

**Old 버전 수학 핵심** (더 정교):
- 3가지 비행 모드: Normal(균형), Cruise(고도 안정), Aggressive(비선형 부스트)
- PID with speedFactor: `speedFactor = V/(q×16)`, 포화 시 적분 리셋
- 사이드슬립 보정: `rollError -= sideslip × √V / 5`
- upWeighting: 큰 기동 중 수평 유지력 동적 감소
- 피치-롤 커플링: `pitchError -= |rollError|/3` (NormalFlight)
- 비선형 피치 부스트: `pitchError += 1 - exp(-e)` (AggressiveFlight)

**New 버전 수학 핵심** (단순화):
- TargetModule: `ProjectOnPlane` + `SignedAngle`로 3축 오차 분리
- BasicFlight: `rollErr × √|yawErr|` (요 오차 비례 뱅크)
- InputPid: D/I 항 주석처리 → 실질 P-only

### 2.3 AeroBlend (현재 구현)

**아키텍처**: MouseAimComputer (C++) + FlightCharacteristics 적응형

**수학 핵심**:
- PD 제어: 축별 적응 게인 + D-term LPF (tau=0.01s)
- 게인 스케줄링: `Kp ∝ √(ca_ref/ca)`, `Kd ∝ (ca_ref/ca)^0.75`
- 속도 게인: `(V_ref/V)²` (동압 역수, 물리적 정확)
- 협조선회: `desired_bank = atan(K×yaw_err×V/g)` (정상선회 물리 공식)
- 실속 보호: 85%/95% AoA progressive, back-calculation anti-windup
- 자동스로틀: n-adjusted (`Vs_n = Vs × √n`)
- 사이드슬립 PD: `-0.8β - 0.3dβ/dt`

---

## 3. 정량적 비교 매트릭스

| 평가 항목 | MouseFlight | MAF Old | MAF New | AeroBlend |
|-----------|:-----------:|:-------:|:-------:|:---------:|
| 제어기 정교도 | 1/5 (P-only) | 3/5 (PID) | 1/5 (P-only) | 4/5 (PD+LPF) |
| 게인 적응성 | 0/5 | 2/5 (speedFactor) | 0/5 | 5/5 (ca+V²) |
| 비행 보호 | 0/5 | 0/5 | 0/5 | 5/5 |
| 선회 품질 | 1/5 | 3/5 (upWeighting) | 1/5 | 5/5 (물리기반) |
| 비행 모드 | 0/5 | 4/5 (3모드) | 0/5 | 0/5 |
| 사이드슬립 보정 | 0/5 | 3/5 | 0/5 | 4/5 (PD) |
| 물리적 정확성 | 1/5 | 2/5 | 1/5 | 5/5 |
| **종합** | **3/35** | **17/35** | **3/35** | **28/35** |

---

## 4. 개선안 (우선순위순)

### P4-1: 비행 모드 시스템 (우선도: 높음, 효과: 높음)

**배경**: MAF Old의 Normal/Cruise/Aggressive 3모드가 상황별 비행 특성을 효과적으로 제공
**구현**: MouseAimComputer에 FlightMode enum 추가, 모드별 pitch_err 변환
- Normal: 현재 동작 유지
- Cruise: horizon_pitch 혼합으로 고도 안정화
- Aggressive: `1 - exp(-|e|)` 비선형 부스트로 기동성 강화

### P4-2: 적분항 도입 (우선도: 중간, 효과: 중간)

**배경**: 바람 등 외란에 대한 정상상태 오차 제거
**구현**: 축별 적분기 + 기존 back-calculation anti-windup 확장
**주의**: 보수적 게인(Ki ≈ 0.1~0.3)으로 시작, 진동 모니터링 필수

### P4-3: 피치-롤 커플링 보정 (우선도: 중간, 효과: 중간)

**배경**: MAF NormalFlight의 `pitchError -= |rollError|/3`
**구현**: `pitch_err -= abs(bank_err) × 0.3` 한 줄 추가
**효과**: 대각도 선회 시 과도한 G-force 방지

### P4-4: 적응형 감쇠비 (우선도: 낮음)

**배경**: ca 기반 ζ=0.7 자동 유지
**구현**: `Kd = 2ζ√(Kp×J/ca)` 역산

### P4-5: 에너지 관리 (우선도: 낮음)

**배경**: specific energy 모니터링으로 에너지 고갈 시 기동 제한
**구현**: `Es = h + V²/(2g)` 추적

---

## 5. 결론

### 5.1 AeroBlend의 현재 위치

AeroBlend의 마우스 에임 시스템(P0-P3)은 분석한 모든 레퍼런스를 **제어 이론적 정교도, 물리적 정확성, 안전성** 면에서 크게 앞선다. 특히:

- 유일한 **물리 기반 협조선회** 구현
- 유일한 **비행 보호 체계** (실속/저속/anti-windup)
- 가장 정교한 **적응형 게인 스케줄링** (축별 ca + 속도²)
- 유일한 **D-term 필터링** (1차 IIR LPF)

### 5.2 레퍼런스에서 차용할 가치가 있는 기능

레퍼런스 시스템이 AeroBlend보다 나은 영역은 **하나**: MAF Old의 비행 모드 시스템이다.
이를 AeroBlend에 적용하면 사용자 경험을 크게 향상시킬 수 있다.

적분항 도입(P4-2)과 피치-롤 커플링 보정(P4-3)은 레퍼런스에서 힌트를 얻은 독립적 개선이며, AeroBlend의 기존 아키텍처에 자연스럽게 통합 가능하다.

### 5.3 권고 실행 계획

| 단계 | 작업 | 예상 영향 |
|------|------|----------|
| 1 | P4-3 피치-롤 커플링 보정 (1줄 추가) | 즉각적 선회 품질 향상 |
| 2 | P4-1 비행 모드 시스템 | 사용자 경험 대폭 향상 |
| 3 | P4-2 적분항 도입 | 정밀도 향상 (외란 보상) |
| 4 | P4-4, P4-5 (선택적) | 장기 품질 개선 |

---

## 부록: 보고서 목록

| 파일 | 내용 |
|------|------|
| `00-final-report.md` | 본 종합 보고서 |
| `01-mouseflight-analysis.md` | MouseFlight (Unity) 수학적 분석 |
| `02-mouseflight-reloaded-analysis.md` | MouseFlightReloaded 비교 분석 |
| `03-mouseaimflight-analysis.md` | MouseAimFlight (KSP) 수학적 분석 |
| `04-aeroblend-current-analysis.md` | AeroBlend 현재 시스템 분석 |
| `05-comparative-analysis.md` | 비교 분석 및 개선안 도출 |
