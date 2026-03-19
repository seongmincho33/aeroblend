# 참고 소스 목록

AeroBlend 개발에 참고할 수 있는 오픈소스 프로젝트 및 리소스 정리.

## 비행 시뮬레이터

| 프로젝트 | 언어 | 설명 | 링크 |
|----------|------|------|------|
| FlightGear | C++ | 가장 유명한 오픈소스 플라이트 심. 400+ 항공기, 전 세계 지형, 멀티플레이어. OpenSceneGraph 렌더링, JSBSim/YASim 물리 | https://github.com/FlightGear/flightgear |
| JSBSim | C++ | 6DoF 비행 역학 모델(FDM) 라이브러리. FlightGear 기본 물리 엔진. 단독 사용 가능 | https://github.com/JSBSim-Team/jsbsim |
| AirSim | C++ | Microsoft 드론/항공기 시뮬레이터. Unreal Engine 기반, 자율주행/AI 연구용 | https://github.com/microsoft/AirSim |

## 마우스 에임 조종 구현체

War Thunder 스타일 마우스 에임을 내장한 오픈소스 비행 시뮬레이터는 없음. 아래는 별도 구현체들.

| 프로젝트 | 엔진 | 설명 | 링크 |
|----------|------|------|------|
| MouseFlight | Unity | War Thunder 스타일 마우스 비행 조종. 가장 유명한 레퍼런스 구현 | https://github.com/brihernandez/MouseFlight |
| MouseFlightReloaded | Unity | MouseFlight 포크, 최신 Unity 대응 | https://github.com/Salzian/MouseFlightReloaded |
| MouseAimFlight | KSP (C#) | Kerbal Space Program용 마우스 에임 모드 | https://github.com/tetryds/MouseAimFlight |

## AeroBlend 포지셔닝

마우스 에임 + 실제 비행 물리(6DoF, RK4, ISA 대기 모델)를 결합한 오픈소스 시뮬레이터는 AeroBlend가 현재 유일.

- FlightGear: 리얼리즘 중심, 마우스 에임 없음
- MouseFlight: 마우스 에임만, 물리 엔진 없음 (Unity 내장 물리 사용)
- AeroBlend: 마우스 에임 + 자체 비행 물리 + glTF 모델 임포트
