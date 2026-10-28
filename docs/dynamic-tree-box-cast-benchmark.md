# DynamicTree BoxCast 후보 탐색 벤치마크

`dynamicTreeBoxCastBenchmark`는 동일한 DynamicTree와 선형 이동 AABB에서
기존 swept-AABB `Query`와 `BoxCast`의 **Tree leaf 후보 수**와 **조회 시간**을 비교한다.
엔진의 narrowphase `ShapeCast`(GJK), Sensor 필터, 전체 `world::castShape`
호출 비용 및 실제 프레임 시간은 측정하지 않는다.

## 실행

Windows (Visual Studio generator):

```powershell
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DZONAI_BUILD_TESTS=ON
cmake --build build --config Release --target dynamicTreeBoxCastBenchmark
.\build\tests\Release\dynamicTreeBoxCastBenchmark.exe
.\build\tests\Release\dynamicTreeBoxCastBenchmark.exe --csv > box-cast.csv
```

Linux/macOS (CMake 4.2+, Release):

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DZONAI_BUILD_TESTS=ON -DZONAI_BUILD_SANDBOX=OFF
cmake --build build-release --target dynamicTreeBoxCastBenchmark
./build-release/tests/dynamicTreeBoxCastBenchmark
./build-release/tests/dynamicTreeBoxCastBenchmark --csv > box-cast.csv
```

CI에서는 `dynamicTreeBoxCastBenchmarkOracle`가 `--verify`만 사용한다.
조회 시간은 PC의 CPU/컴파일러/빌드 설정/열 상태에 민감하므로 **CI 통과 기준에 사용하지 않는다.**
Windows Release CI는 `--csv` 표본 측정을 추가로 출력하지만, 이 값은 runner 환경에서의 참고값이며 사용자 PC의 성능을 대표하지 않는다.

## 비교 조건

- xorshift32 고정 seed로 32×32 = 1,024개 AABB를 만들고 Tree를 한 번 full rebuild한다.
  그 다음부터 Tree와 입력은 두 방법 모두 동일하게 유지한다.
- 다섯 장면: 대각선 장거리 이동, 수평 장거리 이동, 짧은 임의 이동,
  넓은 물체의 대각선 이동, 최대 fraction=0.25로 잘린 대각선 이동.
  각 장면은 128개 sweep이다.
- 기존 방식은 `Union(startAABB, startAABB + translation * maxFraction)`으로
  `Query`를 호출한다. 새 방식은 `BoxCast(startAABB, translation, maxFraction)`을 호출한다.
  콜백에서는 **모두 clip을 변경하지 않는다**. 따라서 처음부터 같은 구간에서
  Tree가 호출한 leaf 후보를 직접 대조한다.
- 정합성 검증은 각 leaf에 대해 swept union AABB 겹침, 독립적인 축별 교차 시간
  oracle과 대조한다. BoxCast 결과는 Query 후보의 부분집합이어야 한다.
  모든 비교는 측정 전에 이루어지며 틀리면 Debug/Release 모두 실패한다.
- 시간은 같은 장면을 각 방법별로 warm-up하고, 순서를 번갈아 9회 측정하며,
  회당 12회 × 128 sweep을 실행한 뒤 **1회 sweep당 ns의 중앙값**을 보고한다.
  출력의 후보 수는 128회 sweep 합계이고, 값이 작을수록 GJK로 넘어갈
  가능성이 있는 BroadPhase leaf가 적음을 의미한다.
- `box_over_query_ratio`는 BoxCast ns / swept Query ns이다.
  1 미만이면 그 **실행 환경·장면·Tree 조회 범위**에서 빠르게 측정됐다는 뜻이다.
  후보 감소가 반드시 조회 시간 감소를 뜻하지는 않는다.

`--csv`는 각 장면의 후보 수, 제거율, 시간, 비율을 헤더와 함께 출력해
외부 스프레드시트나 비교 스크립트로 분석할 수 있다.
전체 `World` ShapeCast 속도를 주장하려면 좁은 단계(GJK) 호출 수와
실제 게임 장면에서의 프레임 시간도 별도로 측정해야 한다.


## Tree 순회 연산 분포 계측

`dynamicTreeBoxCastBenchmark --profile --csv`로 고정 장면의 Query/BoxCast 노드 방문,
탈락 축, 초기 겹침 및 slab 연산 횟수를 출력한다. 기존 기본 모드의 실행 시간
측정과 분리된 **비타이밍 진단 기능**이며, 엔진 코드는 변경하지 않는다.

- `query_nodes` / `box_nodes`: 각 탐색에서 방문한 실재 node 수(leaf 포함).
- `query_rejected` / `box_rejected`: 각 조회에서 AABB/시간 구간 검사로 제외된 node 수.
- `x_tests` / `y_tests`: BoxCast의 X/Y slab 검사 횟수.
- `x_rejected` / `y_rejected`: 처음으로 제외 판정이 난 축의 횟수.
- `stationary_axes`: 이동 성분이 0이어서 나눗셈 없이 비교한 축 검사 횟수.
- `reciprocal_divisions`: 0이 아닌 이동 축의 역수를 query 시작 시 계산하는
  **소스상 나눗셈 횟수**(축당 최대 1회).
- `slab_multiplications`: node별 교차 시간 계산에 double 역수를 곱하는
  **소스상 곱셈 횟수**(이동 축당 최대 2회). 실제 CPU 명령어나 cycle 수는 아니다.
- `start_overlap_nodes`: 방문 시 시작 AABB와 node AABB가 이미 겹쳤던 수.
- `root_exits`: Tree 전체 범위에서 멀어지는 경로에 대한 조기 종료 횟수.

각 값은 해당 장면의 128회 sweep을 합산한다. `VisitNodes`에서 얻은 읽기 전용
node 구조로 실제 순회 수식을 복제하며, 매 입력에 대해 **production
`Query` / `BoxCast`와 leaf ID 집합이 정확히 일치**하는지 검증한다.
이 교차 검증은 `--verify`와 CI에도 포함되고, 실패하면 즉시 종료한다.
계측은 callback에서 clip을 갱신하지 않는 기존 벤치마크 장면에만 적용된다.

계측 수치는 정량적인 분포 설명이지 CPU 병목의 직접적인 증거가 아니다.
성능은 `--csv`(기존 실행 시간 모드)와 함께 분석해야 한다.

### 역수 사전 계산 실험

BoxCast의 노드마다 하던 float 나눗셈 대신, 이동 축별 double 역수를
조회 시작 시 한 번 계산하고 float 경계 차이에 곱한 뒤 float 시간으로
변환한다. 0인 이동 축은 기존 정지 축 판정을 그대로 사용한다.
일반 무작위 후보 검증 외에 경계 접촉, callback fraction, 음수 이동,
아주 작은 subnormal 이동을 비교한다.

최적화의 실제 성능과 후보 집합 보존 여부는 Windows/Ubuntu CI와
Release Tree/World 벤치마크로 판단한다. 나눗셈 경로 감소만으로
빠르다고 단정하거나 CI가 통과했다는 이유만으로 병합하지 않는다.
