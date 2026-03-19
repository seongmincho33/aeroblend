# MouseAim 연구소 - 최종 보고서

> **회사명**: MouseAim 연구소
> **팀 ID**: company-mouseaim-research-20260301
> **완료 시각**: 2026-03-01 02:12:54
> **미션**: 레퍼런스 소스들을 수학적으로 분석해서 마우스 에임 조종방식을 획기적으로 향상시키고 퀄리티를 높일 수 있는지 연구
> **실행 모드**: 실시간 회사 (realtime)
> **투입 인력**: 6명

## 참여 팀원

| 역할 | 이름 | 에이전트 타입 |
|------|------|---------------|
| CEO | ceo | general-purpose |
| 코드분석부서 부서장 | code-analysis-lead | feature-dev:code-explorer |
| 코드분석부서 코드분석가 | code-analyst | feature-dev:code-explorer |
| 수학/제어공학부서 부서장 | math-lead | general-purpose |
| 수학/제어공학부서 제어공학자 | control-engineer | general-purpose |
| 문서화부서 보고서작성가 | report-writer | documentation-generation:docs-architect |

---

## 조직 구조

```
CEO (ceo)
├── 코드분석부서
│   ├── code-analysis-lead (부서장)
│   └── code-analyst (코드분석가)
├── 수학/제어공학부서
│   ├── math-lead (부서장)
│   └── control-engineer (제어공학자)
└── 문서화부서
    └── report-writer (보고서작성가)
```

## 생성 보고서 목록

| 파일 | 내용 |
|------|------|
| `docs/mouse-aim-research/00-final-report.md` | 최종 종합 보고서 |
| `docs/mouse-aim-research/01-mouseflight-analysis.md` | MouseFlight (Unity) 수학적 분석 |
| `docs/mouse-aim-research/02-mouseflight-reloaded-analysis.md` | MouseFlightReloaded 비교 분석 |
| `docs/mouse-aim-research/03-mouseaimflight-analysis.md` | MouseAimFlight (KSP) 수학적 분석 |
| `docs/mouse-aim-research/04-aeroblend-current-analysis.md` | AeroBlend 현재 마우스 에임 수학 모델 |
| `docs/mouse-aim-research/05-comparative-analysis.md` | 비교 분석 및 개선안 도출 |

## 작업 결과 요약

**AeroBlend의 현재 구현(P0-P3)은 모든 레퍼런스를 제어 이론/비행 역학 관점에서 크게 앞섬** (종합 점수 28/35 vs 최고 레퍼런스 17/35).

### 핵심 개선안 3가지

1. **P4-1 비행 모드 시스템** (우선도: 높음) — MAF Old의 Normal/Cruise/Aggressive 3모드 도입. 순항 시 안정성, 공중전 시 민첩성 향상.

2. **P4-2 적분항(I) 도입** (우선도: 중간) — PD → PID 확장. 바람 등 외란에 대한 정상상태 오차 제거. 기존 back-calculation anti-windup 확장 가능.

3. **P4-3 피치-롤 커플링 보정** (우선도: 중간) — `pitch_err -= abs(bank_err) × 0.3` 한 줄 추가로 대각도 선회 시 과도 G-force 방지.

### 주요 발견

- MouseFlightReloaded는 원본 MouseFlight와 100% 동일한 코드 (차이 없음)
- MouseAimFlight New 버전은 PID의 D/I를 주석처리 → 튜닝 어려움 시사
- AeroBlend만이 물리 기반 협조선회 및 비행 보호 체계를 구현

## 예상 효과

- P4-1 비행 모드: 상황별 최적 응답성 → 조종 편의성 30%+ 향상 예상
- P4-2 I항 추가: 정상상태 오차 제거 → 정밀 조준 시 안정성 향상
- P4-3 커플링 보정: 대각도 선회 시 G-force 과도 방지 → 안전성 향상

---

> 생성: tensw-company (MouseAim 연구소) | 완료: 2026-03-01 02:12:54
