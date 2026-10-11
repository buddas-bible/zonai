# 관절 제한 렉돌

Ragdoll은 새 solver나 별도 skeleton subsystem을 추가하지 않고 기존 Revolute Joint를 여러 개 조합하는 통합 실험이다. 목적은 하나의 Joint가 아니라 여러 개의 각도 제한이 동시에 작동할 때 현재 graph/island/wake/sleep/solver 흐름이 안정적으로 유지되는지 확인하는 것이다.

## 구성

Sandbox의 **연결 장치 / 관절 제한 렉돌**은 정적 바닥과 11개의 Dynamic body로 구성한다.

- 머리, 몸통, 골반
- 좌우 위팔 / 아래팔
- 좌우 허벅지 / 종아리

10개의 Revolute Joint가 목, 허리, 양쪽 어깨/팔꿈치/엉덩이/무릎을 연결한다. 모든 Joint는 motor 없이 `enableLimit = true`인 수동 관절이다. 연결된 두 body의 collision은 기존 Joint 기본 정책대로 제외하고, 다른 body part끼리의 contact는 일반 collision 경로를 사용한다.

초기 body rotation은 모두 0이고 각 local anchor는 초기 월드 위치가 정확히 일치하도록 배치한다. 따라서 시작 자세에서 constraint error를 일부러 넣지 않고, 중력과 외부 입력으로부터 여러 관절 제한이 함께 반응하는 모습을 관찰한다.

## 각도 제한

범위는 실제 인체 모델을 재현하기 위한 데이터가 아니라 2D 학습 데모에서 각 joint가 서로 다른 역할을 보이도록 잡은 근사치다.

- 목: ±25도
- 허리: ±20도
- 어깨: ±90도
- 팔꿈치: 좌우가 반대 방향으로 주로 접히도록 비대칭 범위
- 엉덩이: ±55도
- 무릎: 좌우가 반대 방향으로 주로 접히도록 비대칭 범위

Revolute의 `referenceAngle`은 0을 사용하므로 초기 상대 회전이 각 제한의 기준 자세다. 제한은 기존 Revolute의 predictive unilateral impulse와 warm start/store 경로를 그대로 사용한다.

## Sandbox 조작

몸통을 공통 실험 대상으로 사용한다.

- A / D 유지: 몸통을 좌우로 밀기
- Space: 위쪽 선형 임펄스
- S: 몸통 회전 임펄스
- 왼쪽 드래그: 기존 Mouse Joint로 원하는 body part 잡기
- 오른쪽 클릭: 몸통을 커서 방향으로 밀기

기존 오브젝트 Inspector, Contact/Tree/AABB 표시와 충돌 설정을 그대로 사용할 수 있다. 별도 Ragdoll 전용 UI나 skeleton 편집기는 추가하지 않는다.

## 검증

`sandboxJointDemoTests`는 Ragdoll 등록과 함께 다음을 확인한다.

- 정적 바닥 + 11 body part + 10 Revolute Joint 구성
- 모든 body/joint handle의 유효성
- 모든 Ragdoll Joint가 motor 없는 angular limit인지 확인
- 초기 anchor/angle 상태가 finite인지 확인
- 몸통에 큰 선형 임펄스를 준 뒤 4 substep으로 여러 초 동안 시뮬레이션해 모든 body transform이 finite인지 확인
- 안정화 뒤 각 관절 상대각도가 허용 범위 근처에 머무는지 확인

이 데모는 기존 Revolute Joint를 실제 조합에 사용해보는 단계다. 별도 Ragdoll solver, animation pose drive, IK, muscle motor, breakable joint는 이번 범위에 포함하지 않는다.
