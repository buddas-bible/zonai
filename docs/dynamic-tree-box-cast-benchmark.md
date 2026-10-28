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
