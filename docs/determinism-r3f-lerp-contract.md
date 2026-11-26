# R3f — 결정론적 선형 보간 후보의 수학 계약

기반: [R3e #346](https://github.com/buddas-bible/zonai/pull/346), HEAD `ec1c8f713e0120af7b0c4ecd372fbdbe8e9b0e5e`.

## 문제와 범위

[R3e CI](https://github.com/buddas-bible/zonai/actions/runs/45273361046)의 최초 차이는 동일한 U(3,4) 보간 입력에 대한 두 번째 `std::lerp` 출력이다. 따라서 플랫폼 독립 연산 규약을 먼저 검증한다. 이 PR은 생산 Fluid의 보간 호출부, 이류·압력 솔버, 시뮬레이션 순서, strict 비교 정책을 변경하지 않는다.

## 후보: deterministicLerp(float a, float b, float t)

- 유효 정의역: 유한 binary32 끝점 a,b, 유한 t ∈ [0,1]. 외삽 미지원.
- t=0/1은 각각 a/b의 원래 비트 그대로 반환. 같은 끝점은 a를 반환(부호 있는 0 포함).
- 잘못된 정의역은 canonical qNaN `0x7fc00000` 반환.
- 나머지는 명시적 binary64 곱·합 순서 `(1-t)*a + t*b`, 마지막 binary32 변환. float에서 b-a를 계산할 때의 범위 초과를 피하고, 표준 라이브러리의 보간 구현 차이를 제거한다.
- IEEE binary32/binary64, round-to-nearest, gradual underflow, 비수축(FMA 금지), fast-math 금지라는 기존 FP 정책을 전제로 한다. 모든 실수에 대한 올바른 반올림 또는 모든 컴파일러/CPU에서의 일치를 증명하는 함수라고 주장하지 않는다.

## 검증

기존 `fpMathAuditTests`의 일반 CTest 호출과 `--emit` 모드 모두에 다음 검사를 추가한다.

- 15개 exact-bit 사례: 원래 U 보간 실패 입력, 끝점, signed zero, 반대 부호 최대 유한값, subnormal 및 tie-to-even.
- 7개 잘못된 정의역: NaN, 무한대, 외삽.
- 고정 정수 seed로 생성하는 1,024개 finite binary32 끝점 쌍 및 가중치. 끝점 정확성, 유한 범위 보존, 5개 고정 가중치 단조성, 독립 double difference-form 참고값과의 오차 한계를 검사.
- 위 1,046개 결과를 `zonai.lerp*` 엔진 소유 fingerprint로 추가. 기존 strict 비교가 오차 없이 모든 비트를 비교한다. 수치 정확성 검사의 허용 한계는 cross-OS 비트 비교에는 적용하지 않는다.

R3e 재현 입력: `a=0x3d094d24`, `b=0x3f12fd58`, `t=0x3f763360`. 후보의 기대 출력은 `0x3f0db10a`. 이 결과를 확인하는 것과 Fluid 전체 결정론 확보는 별개다.

## 다음 수용 단위

R3f math CI와 회귀를 확인한 후 별도 R3g에서 production `sampleComponent`만 후보로 교체하는 A/B를 수행한다. 기존 R3e 진단의 재구성도 같은 연산을 추적하도록 맞춘다. 실제 32 Tick 상태·반력, 첫 불일치 위치, 수치 안정성과 성능을 비교한다. 다른 geometry/libm 차이가 남을 수 있으므로 전체 strict gate가 녹색일 것이라고 가정하지 않는다. 현재 후보 PR은 기존 Fluid gate가 실패해도 정상이며, 병합하지 않는다.
