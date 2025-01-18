
# Bayesian Optimization을 이용한 스마트 팩토리 장비 배치 최적화

이 레포지토리는 Digital Twin(DT) 플랫폼을 기반으로 스마트 팩토리 환경에서 장비 배치를 최적화하는 연구와 구현 내용을 담고 있습니다. NVIDIA Omniverse Isaac Sim을 활용해 고품질 시뮬레이션을 수행했으며, Bayesian Optimization(BO)을 통해 작업 시간(takt time)을 획기적으로 단축하고 생산성을 향상시켰습니다. 2023 한국CDE 동계학술대회에서 연구의 가치를 인정받아 우수 포스터상을 수상하였습니다.

---

## 연구 배경
4차 산업혁명의 발달로 Digital Twin 기술은 스마트 팩토리의 제조 공정을 개선하는 핵심 도구로 자리 잡았습니다. 그러나 기존의 장비 배치 최적화 방법은 전문가의 경험에 의존하여 효율성이 낮았으며, 시장 변화에 유연하게 대응하지 못하는 문제가 있었습니다.

### 기존 방법의 한계
- 장비 배치 설계가 전문가의 직관에 의존.
- 변화하는 시장 수요에 따라 생산 공정을 변경할 때 발생하는 비효율성.

---

## 연구 목표
- Digital Twin을 활용한 장비 배치 시뮬레이션 및 최적화.
- Bayesian Optimization을 통해 작업 시간(takt time)과 운영 비용 최소화.
- NVIDIA Omniverse Isaac Sim을 사용해 정밀하고 확장 가능한 시뮬레이션 워크플로우 구축.

---

## 시스템 개요

### 데모 팩토리 구성
- **장비 구성**: 로봇 암, 선형 스테이지, 3D 프린터 2대, 밀링 머신.
- **작업 시나리오**:
  - Task #1: 순차 작업 `3DP_1 → 3DP_2 → 밀링 머신`.
  - Task #2: 반복 작업 `3DP_1 → 밀링 머신 → 3DP_2 → 밀링 머신`.

### NVIDIA Omniverse Isaac Sim 통합
- **플랫폼 특징**:
  - 로봇 동작과 물리적 상호작용을 정밀하게 재현하는 고품질 시뮬레이션 제공.
  - 확장성과 높은 반복성을 지원하는 시뮬레이션 환경 구축.
- **활용 사례**:
  - Digital Twin 모델 설계 및 시뮬레이션 실행.
  - Bayesian Optimization과의 통합을 통해 반복적 최적화 수행.

### Omniverse 사용법 문서화
- NVIDIA Omniverse Isaac Sim 설치 및 사용법을 Notion에 정리해 팀원들과 공유했습니다.
- 주요 내용:
  1. **설치 가이드**: Omniverse Isaac Sim 설치 단계별 안내.
  2. **시뮬레이션 구성**: Digital Twin 환경 설계 및 관리 방법.
  3. **BO 통합**: Bayesian Optimization과 Omniverse를 연동하는 코드 및 워크플로우.

---

## 결과 및 주요 발견
- **최적화 성과**:
  - Task #1: 작업 시간(takt time) **28.8%** 단축.
  - Task #2: 작업 시간(takt time) **40.2%** 단축.
- **결과 요약**:
  - 초기 배치 대비 스마트 팩토리 생산성 대폭 향상.
- **Omniverse 기여**:
  - 다양한 공장 배치 시나리오를 신속히 테스트하고 평가.
  - 실제 환경에 적용 가능한 유용한 통찰 제공.


---

## 레포지토리 구조

```plaintext
.
├── README.md                  # 프로젝트 문서
├── src/                       # Bayesian Optimization 소스 코드
├── assets/                    
│   ├── poster.pdf             # 연구 포스터
│   ├── certificate.pdf        # 한국CDE 학회 우수 포스터상
│   └── simulation_video.mp4   # 시뮬레이션 데모 비디오
