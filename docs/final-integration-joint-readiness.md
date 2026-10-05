# 최종 통합 확인 — Joint 개발로 전환

기준: 2026-10-06, master `8fb0ae5c6fa29fc1115be16f823f1f4a69739b12`. Box2D main을 다시 fetch했으며 기준은 여전히 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`이다.

이 프로젝트의 목적은 물리 엔진 학습이다. 이미 검토한 범위를 반복해서 정리하기보다, 기존 구현의 불변식과 다음 제약의 수학·수명을 연결해 이해하는 데 초점을 둔다. 이번 작업은 Part 1–10 병합 상태, subsystem 연결과 Joint 진입 조건을 확인한다. 모든 함수를 다시 감사하거나 Box2D 전체 동등성을 주장하지 않는다.

## 현재 구현 연결

| 경계 | 이번 확인 | 다음 Joint에서 필요한 확장 |
| --- | --- | --- |
| World 수명 / handle | `id.h`의 worldToken + index1 + generation, free slot과 IsValid 규칙. Body/Contact 파괴 경로와 기존 lifetime 회귀를 확인함. | 같은 규칙의 jointId, stable slot, Body 삭제 시 연결 Joint 제거. |
| BroadPhase → Contact | pair callback은 후보만 전달하고 실제 Contact 생성에서 pairSet을 등록함. DestroyContact가 pair와 양쪽 Body edge를 함께 제거함. | Body joint edge를 검사해 연결된 pair를 차단. 마지막 차단 Joint 삭제 뒤 stationary proxy도 TouchProxy로 다시 후보화함. |
| Contact → Island | `BuildIslands`는 pointCount > 0인 Contact를 edge로 삼고 awake non-static Body만 union함. Static이 무관한 Dynamic들을 연결하지 않음. | Contact가 없어도 Joint edge로 non-static Body를 union하고 island별 Joint 구간을 구성함. |
| Wake / sleep | WakeBodyByIndex, SleepBodyByIndex, WakeSleepingBodiesFromContacts가 Contact graph를 순회함. Static 변경은 이웃을 깨우되 Static을 graph bridge로 쓰지 않음. | Joint graph 순회와 Step 전 awake/sleep 경계 동기화. 생성/삭제/pose 변경도 연결 Body를 깨워야 함. |
| Solver / delta | Step에서 prepare → substep force/velocity → warm start → bias solve → delta 적분 → relax → Contact restitution → CCD → impulse store/commit → sleep 순서를 확인함. COM anchor와 bodyState delta는 별도임. | 같은 subStepTime과 delta 좌표계로 Joint pass를 연결. Joint에는 Contact restitution을 적용하지 않음. |
| Sensor / CCD | Sensor는 별도 overlap 단계이며 CCD는 shape filter와 TOI 경로를 사용함. Contact와 Sensor를 같은 정책이라고 가정하지 않음. | Joint의 Body collision filter가 CCD에서도 적용되는지 검사. 아래 Box2D 정책 차이를 명시하고 회귀로 고정함. |

기존 구현된 경로에서 이번 연결 확인만으로 새 correctness 수정을 정당화할 재현 사례는 발견하지 않았다. Joint가 없어서 빠진 edge·filter·solver 경로는 기존 기능 결함이 아니라 다음 구현의 요구 사항이다. Solver sets/constraint graph, persistent island, worker/SIMD를 먼저 추가할 필요는 없다.

## Box2D 비교에서 추가로 확인한 경계

- [Body collision filter](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/body.c): 같은 Body pair에 여러 Joint가 있으면 하나라도 collideConnected=false일 때 Contact를 차단함. Shape의 양수 groupIndex가 Joint 차단을 무효화하는 정책으로 만들지 않음.
- [Joint 생성/파괴](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/joint.c): 기존 Contact 제거, Body 양쪽 intrusive edge, impulse 상태, 생성/파괴 시 wake가 서로 연결돼 있음. Zonai의 삭제 후 proxy touch는 현재 moved-only 후보 탐색 구조에서 stationary pair 복원을 위해 필요함.
- [Sensor overlap](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/sensor.c)은 Joint Body filter를 조회하지 않고 shape filter를 사용하지만, [continuous candidate](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/solver.c)는 Sensor 후보에도 Body collision filter를 적용함. 첫 Joint에서도 discrete/continuous 경로의 차이를 의도적으로 유지하고 검사해야 함.
- [Distance solver](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/distance_joint.c)의 기본 rigid 경로만 먼저 학습 대상으로 삼음. 물리 spring, 거리 limit, motor는 별도 단계로 둠. Rigid 제약의 수치 안정화용 softness와 물리 spring 옵션을 혼동하지 않음.

## 확인한 검증과 한계

Windows 기본 설정에서 Sandbox 포함 Debug/Release 전체 build와 CTest 실행을 다시 수행해 각각 38/38 통과했다. 기존 8개 Release runtime 검사에는 Body/Shape/Contact 수명, solver/restitution, Island 연결, wake/sleep/static 경계, substep force/delta, CCD/Sensor, query callback과 cache/mass 회귀가 포함된다. 이번에 Joint를 만들어 검증한 것은 아니다.

Part 10의 Windows/Ubuntu CI와 외부 consumer 검증은 [PR #18](https://github.com/buddas-bible/zonai/pull/18)에 기록돼 있다. 이번 문서의 독립 리뷰와 원격 CI 결과는 별도 PR에 기록한다. Sandbox UI 시각 검증은 하지 않았다. 기존 assert 기반 29개 테스트의 Release 검사 공백, 16-bit generation wrap, 고정 stack 경계, moving-target CCD bounds와 Sensor hit budget 등은 [감사 현황](audit-status.md)의 제한을 유지한다.

## 다음 실제 구현

후보 설계는 [기본 Distance Joint](superpowers/specs/2026-10-06-distance-joint-design.md)에 있다. 이 문서는 새 subsystem의 검토 가능한 제안이며 구현 완료나 사용자 설계 승인 기록은 아니다. 성공 기준은 고정 거리 제약의 수학과 lifecycle/graph/filter 통합을 실제 테스트와 작은 Sandbox 실험으로 확인하는 것이다. 더 넓은 감사·Release 일괄 전환을 새 선행 조건으로 추가하지 않는다.
