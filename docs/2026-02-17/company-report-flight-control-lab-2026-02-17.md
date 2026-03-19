# 비행조종연구소 - 최종 보고서

> **회사명**: 비행조종연구소 (Flight Control Research Lab)
> **팀 ID**: company-flight-control-lab-20260217
> **완료 시각**: 2026-02-17 04:22:40
> **미션**: AeroBlend 마우스 비행 조종 시스템 개선 - 비행기 형상별 조종 특성 차이를 수용하면서 공통된 마우스 에임 방식 제공
> **실행 모드**: 실시간 회사 (realtime)
> **투입 인력**: 7명

## 참여 팀원

| 역할 | 이름 | 에이전트 타입 |
|------|------|---------------|
| CEO (연구소장) | ceo | general-purpose |
| 물리엔진연구부장 | rust-physics-lead | systems-programming:rust-pro |
| 공력연구원 | aero-researcher | feature-dev:code-explorer |
| 조종시스템연구부장 | cpp-controls-lead | systems-programming:cpp-pro |
| 튜닝엔지니어 | tuning-engineer | application-performance:performance-engineer |
| 통합검증부장 | test-lead | unit-testing:test-automator |
| 비행디버거 | flight-debugger | unit-testing:debugger |

---

## 조직 구조

```
CEO (ceo)
├── 물리엔진연구부
│   ├── rust-physics-lead (부서장)
│   └── aero-researcher (공력연구원)
├── 조종시스템연구부
│   ├── cpp-controls-lead (부서장)
│   └── tuning-engineer (튜닝엔지니어)
└── 통합검증부
    ├── test-lead (부서장)
    └── flight-debugger (비행디버거)
```

## 작업 결과 요약

### Task 1: 비행 특성 매개변수 자동 계산 (물리엔진연구부)

비행기 형상(날개면적, AR, 질량, 관성)에서 instructor가 사용할 핵심 매개변수를 자동 계산하는 `compute_flight_characteristics()` 함수를 구현했습니다.

**FlightCharacteristics 구조체 (9개 매개변수):**
- `stall_speed_ms`: V_stall = sqrt(2*W/(rho*S*CL_max))
- `ref_speed_ms`: 순항속도 = 1.3 * stall_speed
- `max_aoa_rad`: 최대 AoA (AeroSurface에서 도출)
- `total_wing_area_m2`: 총 날개면적
- `control_authority`: ref_control_torque / avg_inertia (제어 민감도 지표)
- `wing_loading_kg_m2`: 익면하중
- `thrust_to_weight`: 추력/중량비
- `max_pitch_rate_rad_s`: 최대 피치 각속도
- `max_roll_rate_rad_s`: 최대 롤 각속도

**FFI 경로:** `FlightCharacteristicsC` (#[repr(C)]) → `AircraftPhysicsStateC.flight_chars` → C++ instructor

### Task 2: 적응형 Instructor 구현 (조종시스템연구부)

하드코딩된 9개 상수를 비행기 특성에 따라 자동 조절하는 `configure(FlightCharacteristicsC)` 메서드를 구현했습니다.

**PD 게인 스케줄링:**
- `control_authority` 기준 (baseline ca_reference = 0.25)
- P 게인: `sqrt(ca_reference / ca)` — 둔감한 비행기에 강한 P
- D 게인: `sqrt(1.0 / ca_ratio)` — 민첩한 비행기에 강한 D
- P/D 비율이 항상 ~2.08 유지 → 모든 비행기에서 일관된 감쇠 특성

**적응형 스톨 보호:**
- `stall_aoa = max_aoa * 0.70` (보호 시작)
- `critical_aoa = max_aoa * 0.90` (강제 노즈다운)
- 속도 임계값: `min_speed = stall_speed * 1.05`, `auto_throttle_speed = stall_speed * 1.30`

**자동 구성:** 첫 프레임에서 `state.flight_chars`를 읽어 자동 configure

### Task 3: 다중 프로파일 통합 테스트 (통합검증부)

3개 비행기 프로파일로 포괄적 검증을 수행했습니다:

**테스트 프로파일:**
| 프로파일 | 모델 | 질량 | 날개면적 | 특성 |
|---------|------|------|---------|------|
| Biplane | An-2 | 5250kg | 71.5m² | 큰 날개, 저속, 높은 항력 |
| Monoplane | Cessna 172 | 1111kg | 16.2m² | 일반 GA, 중간 속도 |
| Jet | F-16 | 12000kg | 27.9m² | 작은 날개, 고속, 높은 추력 |

**Rust 테스트 (65개 통과):**
- `test_init_biplane/monoplane/jet_flight_chars`: 특성 계산 검증
- `test_biplane/monoplane/jet_level_flight`: 수평비행 유지
- `test_biplane/monoplane/jet_pitch_up_recovery`: 스톨 복구
- `test_all_profiles_respond_to_roll`: 롤 응답 비교

**C++ 테스트 (17개 통과):**
- 각 프로파일별 configure() 검증
- 적응형 게인 범위 검증
- 스톨 보호 동작 검증

## 수정 파일 목록

### Rust (물리엔진)
- `rust/aeroblend-physics/src/types.rs` — `FlightCharacteristics` 구조체 추가
- `rust/aeroblend-physics/src/aerodynamics.rs` — `compute_flight_characteristics()` 함수 추가
- `rust/aeroblend-physics/src/engine.rs` — FlightCharacteristics 통합, 다중 프로파일 테스트, biplane/monoplane/jet 생성자
- `rust/aeroblend-core/src/lib.rs` — `FlightCharacteristicsC` FFI 구조체 추가, `AircraftPhysicsStateC`에 포함
- `rust/aeroblend-ffi/src/lib.rs` — `aeroblend_physics_get_flight_chars()` FFI 함수 추가
- `rust/aeroblend-importer/src/lib.rs` — 엔진 추력 스케일링 (T/W=0.35 목표)

### C++ (조종 시스템)
- `cpp/include/aeroblend/input.h` — `MouseAimComputer::configure()` 메서드, 적응형 멤버 변수 추가
- `cpp/src/input.cpp` — 적응형 PD 게인 스케줄링, 스톨 보호, 자동 스로틀 (모두 동적 임계값)

## 핵심 설계 원리

### 1. 통합 제어 권한 지표 (control_authority)
```
control_authority = ref_control_torque / avg_inertia
```
- 단일 스칼라로 비행기의 "민첩성" 정량화
- 복엽기 (큰 날개, 큰 관성): ca ≈ 0.15 → P 강화, D 약화
- 전투기 (작은 날개, 높은 추력): ca ≈ 0.8 → P 약화, D 강화

### 2. sqrt 스케일링
```
P_scale = sqrt(ca_reference / ca)
D_scale = sqrt(1.0 / ca_ratio)
```
- 선형 스케일링보다 부드러운 전환
- P/D 비율이 비행기 간에 일정하게 유지
- 오버슈트 없이 안정적 수렴

### 3. 다단계 스톨 보호
```
70% max_aoa: 피치업 권한 점진 감소
90% max_aoa: 강제 노즈다운
stall_speed × 1.30: 자동 스로틀 개입
stall_speed × 0.85: 비상 스로틀 100%
```

## 예상 효과

1. **범용성**: 복엽기부터 제트기까지 모든 형상의 비행기에서 동일한 마우스 조작 방식
2. **안정성**: 비행기별 적응형 스톨 보호로 초보자도 안전한 비행 가능
3. **일관성**: PD 게인 스케줄링으로 어떤 비행기든 유사한 반응 속도와 감쇠
4. **확장성**: 새 비행기 추가 시 geometry만 올바르면 instructor 자동 적응
5. **검증 완료**: 82개 테스트 (Rust 65 + C++ 17)로 3개 프로파일 포괄 검증

---

> 생성: tensw-company (비행조종연구소) | 완료: 2026-02-17 04:22:40
