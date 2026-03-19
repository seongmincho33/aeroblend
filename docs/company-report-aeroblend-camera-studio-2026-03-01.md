# 에어로블렌드 카메라 스튜디오 - 최종 보고서

> **회사명**: 에어로블렌드 카메라 스튜디오
> **팀 ID**: company-aeroblend-camera-20260301
> **완료 시각**: 2026-03-01 00:48:28
> **미션**: 카메라가 항공기 뒤를 따라다니며 촬영하는 카메라맨처럼 동적으로 움직이도록 개선
> **실행 모드**: 실시간 (realtime)
> **투입 인력**: 5명

## 참여 팀원

| 역할 | 이름 | 에이전트 타입 |
|------|------|---------------|
| CEO | ceo | general-purpose |
| 카메라 연구부 부서장 | research-lead | general-purpose |
| 수학 연구원 | camera-math | general-purpose |
| 카메라 구현부 부서장 | impl-lead | systems-programming:cpp-pro |
| 코드 리뷰어 | reviewer | code-refactoring:code-reviewer |

---

## 조직 구조

```
CEO (ceo)
├─ 카메라 연구부
│   ├─ research-lead (부서장)
│   └─ camera-math (수학 연구원)
└─ 카메라 구현부
    ├─ impl-lead (부서장)
    └─ reviewer (코드 리뷰어)
```

## 작업 결과 요약

### 문제 정의
현재 체이스 카메라는 항공기 heading을 exponential smoothing으로 추적하지만,
카메라 **위치**가 매 프레임 목표점에 직접 배치되어 "카메라맨이 우두커니 서있는" 정적 느낌.
항공기 급기동 시 카메라가 관성으로 뒤처지고 다시 따라잡는 동적인 느낌이 필요.

### 연구 결과 — 채택된 수학적 모델 (7가지)

| 우선순위 | 효과 | 수학적 모델 | 핵심 파라미터 |
|----------|------|-------------|---------------|
| **P0-1** | 스프링-댐퍼 위치 추적 | 2차 ODE: `a = ω_n²(target-pos) - 2ζω_n·vel` | ω_n=5.0, ζ=0.75 |
| **P0-2** | 속도 기반 동적 거리 | 선형함수 + 1차 LPF | 15m(저속)~25m(고속), rate=2.0 |
| **P0-3** | 턴/롤 측면 오프셋 | `-gain·sin(roll)·turn_intensity` + LPF | gain=4.0m, rate=3.0 |
| **P1** | 가속도 look-ahead | `dot(accel,fwd)·gain` + LPF | base=30m, 범위 20~50m |
| **P2-1** | G-force FOV | G당 1.5도 증가 + LPF | 75~82도, rate=3.0 |
| **보완1** | yaw/pitch 축별 스프링-댐퍼 | 기존 exp smoothing → 2차 ODE | yaw: ω=4.0/ζ=0.7, pitch: ω=3.5/ζ=0.8 |
| **보완2** | 피치 기반 동적 높이 | `base + pitch_gain·sin(pitch)` + LPF | base=3m, gain=4m, rate=2.5 |
| **보완3** | G-feel ω_n 변조 | `g_scale = 1/√(0.5+0.5|G|)` | 모든 스프링에 적용 |

모든 스무딩: `α = 1 - exp(-rate·dt)` (dt 독립적, 가변 프레임레이트 안전)

### 구현 결과

**camera.h** — 멤버 변수 11개 추가:
- `cam_yaw_vel_`, `cam_pitch_vel_` — yaw/pitch 스프링-댐퍼 속도
- `cam_pos_x/y/z_`, `cam_vel_x/y/z_` — 위치 스프링-댐퍼 상태 (6개)
- `spring_initialized_` — 스프링 초기화 플래그
- `smooth_dist_`, `smooth_lateral_`, `smooth_height_` — 동적 거리/측면/높이
- `smooth_look_ahead_`, `smooth_fov_` — look-ahead/FOV

**camera.cpp** — 핵심 변경:
- `init()`: 새 변수 전체 초기화
- `update()`: dt clamp max 0.05s + 7가지 동적 효과
- Semi-implicit Euler 적분 (symplectic, 에너지 보존)
- 오빗 컨벤션 X-negation 유지 (`arm_x = -sy*cp`)
- Camera::update() 시그니처 변경 없음 (main.cpp 수정 불필요)

### 코드 리뷰: APPROVE

- 연구부 사양과 파라미터 일치 확인
- dt clamp 정상 동작 확인
- 오빗 컨벤션 유지 확인
- 코드 스타일 일관성 확인

### 빌드/테스트

- 빌드: `cmake --build .` 성공 (warning 없음)
- 테스트: 16/17 통과 (keyboard_w는 기존 실패, 이번 변경 무관)

## 수정 파일 목록

| 파일 | 변경 유형 |
|------|-----------|
| `cpp/include/aeroblend/camera.h` | 수정 — 멤버 변수 11개 추가 |
| `cpp/src/camera.cpp` | 수정 — init() 초기화 + update() 동적 카메라 로직 7가지 |

## 예상 효과

| Before | After |
|--------|-------|
| 위치: 매 프레임 목표점 직접 배치 | 스프링-댐퍼 관성 추적 (뒤처짐 + 오버슈트) |
| 거리: 고정 20m | 속도 연동 15~25m (고속 줌아웃) |
| 측면: 정후방 고정 | 선회 시 외곽 4m 밀림 |
| 높이: 고정 3m | 피치 연동 -1~7m |
| Look-ahead: 고정 30m | 가속도 연동 20~50m |
| FOV: 고정 75도 | G-force 연동 75~82도 |
| 방향: exponential smoothing | 축별 스프링-댐퍼 (미세 오버슈트) |

정적 카메라 → 동적 카메라맨 완성.

---

> 생성: tensw-company (에어로블렌드 카메라 스튜디오) | 완료: 2026-03-01 00:48:28
