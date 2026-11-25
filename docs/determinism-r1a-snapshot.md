# R1a — 최소 World/Body 결정론 스냅샷 (테스트 전용)

> PR: R1a. 기준 `master` `3f6dc695`. 기존 [R0 코드 감사](https://github.com/buddas-bible/zonai/pull/328)를 후속한다.  
> **현재 범위:** `tests/determinism/world_state_snapshot.h`의 **공개 World/Body 관찰 스냅샷**만 제공한다. 엔진 내부 알고리즘, 계산 순서, API, 기본값, Sandbox는 변경하지 않는다.  
> **비목표:** 이것은 아직 완전한 권위(authoritative) 상태 직렬화, 저장/로드, 네트워크 롤백 또는 플랫폼 간 결정론 통과 인증이 아니다.

## 1. 의도

먼저 서로 다른 World 인스턴스가 **같은 입력과 Tick**에서 공개적으로 관찰 가능한 World/Body 상태를 비트 수준으로 재현하는지 확인할 최소 기반을 만든다. 현재 `bodyId.worldToken`은 각 World 수명에서 고유하지만 다른 세계의 같은 물리 상태를 비교할 때는 제외한다. 비교 전에 `world.IsValid(bodyId)`로 현재 World에 속한 올바른 핸들인지 검증한다.

## 2. Snapshot format v1

필드별 record를 순서대로 이어붙인 바이트 스트림:

```text
[u16 field_name_bytes little-endian]
[field_name UTF-8 bytes]
[u8 value_byte_width: 1, 2, 4 or 8]
[value bits little-endian]
```

- `schema.magic`, `schema.version=1`, 정수 `tick`, `dt` float32 bits, World substep 수를 먼저 기록한다.
- 모든 `float`는 `std::bit_cast<uint32_t>`로 원시 IEEE-754 bits를 보존하며 `-0`과 `+0`을 구분한다. 비유한 값(NaN/Inf)은 이 단계에서 안정성 실패로 거절한다.
- `worldToken`, 포인터, padding, 메모리 주소, 벽시계 기록 등은 수집하지 않는다.
- 등록한 Body handles는 `index1`, `generation` 기준 정렬한다. 중복·무효·다른 World 핸들은 거절한다. 모든 Body를 자동 발견하는 기능은 아직 없다.
- World: 중력, 최대 선속도, Contact 재활용 거리, Hit 속도 임계값, sleep/continuous 플래그, Body/Shape/Contact/Joint 개수.
- Body: slot/generation, type, position, rotation cos/sin, linear/angular velocity, mass/inertia/local center, damping, gravity scale, sleep 관련 상태, CCD/bullet/fast/TOI 플래그, Contact recycling, Body별 Shape/Contact/Joint/Chain 카운트와 head link.
- 필드의 이름과 폭도 바이트 스트림에 포함해 같은 값이더라도 다른 스키마가 동일하게 간주되지 않는다. `FNV-1a-64`는 출력용 비암호학적 체크섬이며 **정확한 일치 여부는 전체 bytes/field-by-field 비교**로 검사한다.
- `firstDifference`는 첫 서로 다른 필드의 이름·raw bits·폭을 반환한다.

## 3. R1a tests

`worldStateSnapshotTests` (Windows/Linux Debug CI의 CTest로 등록):

1. 리틀엔디언 정수·float32 바이트, signed zero 구분, Inf 거절.
2. 서로 다른 World 수명 토큰을 가진 같은 장면끼리 첫 Tick 바이트 일치.
3. Body 입력 목록의 다른 순서에서 동일한 직렬화 결과.
4. 중복·다른 World 핸들 거절.
5. 생성된 두 독립 World를 같은 고정 `dt=1/60`, 4 substeps, 같은 간헐적 Force 입력으로 50 Tick 진행해 매 Tick 전체 바이트를 비교.
6. Body 선속도를 변경하면 첫 차이 필드가 `body[2].linearVelocity.x`로 보고됨을 검사.
7. Tick 번호 변화가 먼저 차이로 보고됨을 검사.

실행 예시:

```sh
cmake -S . -B build -DZONAI_BUILD_TESTS=ON -DZONAI_BUILD_SANDBOX=OFF
cmake --build build --target worldStateSnapshotTests
ctest --test-dir build -R '^worldStateSnapshotTests$' --output-on-failure
```

Windows 다중 구성 빌드는 적절한 `--config Debug`, `ctest -C Debug`를 추가한다.

## 4. 현재 검사하지 않는 권위 상태 — 반드시 확장

R1a가 같은 hash를 반환하더라도 엔진 **전체 결정론**을 증명하지 못한다:

- 추적 목록에 등록하지 않은 Body, Shape/Chain geometry, Collision Filter/Matrix, 재사용 가능한 free slots 및 generation.
- persistent Contact manifold, 각 점의 cached normal/tangent impulse, Contact/Joint 연결 그래프의 모든 필드.
- Joint 종류별 solver cache, Island/CCD/TOI 내부 scratch의 다음 Tick 관련 상태.
- DynamicTree 구조 및 proxy fat AABB, broad-phase pair membership.
- BodySim의 pending force/torque, 내부 Sleep/Constraint 캐시 중 공개 관찰에 없는 부분.
- Fluid Field의 U/V, Pressure, Cut-cell fraction, aperture, swept history, tracer, PCG warm start 및 coupling response.
- 이벤트 순서, 모든 외부 입력과 PRNG seed/상태.
- 실제 Windows/Linux·MSVC/GCC/Clang **출력 파일 간 교차 비교**.

위 항목은 R1b/R1c 또는 세부 subsystem 단계에서 내부 read-only snapshot 접근을 설계하고 test schema를 확장한다. 공개 production API에 무조건 debug getter를 증식시키지 않으며, 안정적인 field ownership을 먼저 정한다.

## 5. 후속 순서와 승인 조건

- R1a **완료**: 저장소에 CMake/CTest 포함 테스트 코드 등록, Windows/Linux CI 결과 기록. 성공해도 동일 실행 파일/동일 플랫폼 재현성만 확인한 것.
- R1b: Tick별 입력 로그와 전역 Replay runner, 파일로 출력되는 canonical snapshots와 최초 차이 보고.
- R1c: Windows/Linux 동일 장면 **파일 비교** 및 컴파일러/FP 설정 계약. 불일치 시 합격으로 표시하지 않는다.
- R2: FP/삼각함수와 BroadPhase/World/Joint/CCD의 실제 첫 차이를 찾아 작은 PR로 수정.
- 이후 R3/R4의 코드 분할 및 이전 경로 삭제는 이 도구의 회귀와 물리 정확성/성능 검증이 확보된 뒤 실행한다.

**중요:** R1a에선 게임용 알고리즘·유체 문제를 수정하지 않는다. [Issue #324](https://github.com/buddas-bible/zonai/issues/324)의 움직이는 Cut-cell 압력 스파이크는 여전히 미해결이다.
