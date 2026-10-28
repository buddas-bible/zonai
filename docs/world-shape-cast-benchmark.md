# World ShapeCast: Query / BoxCast + GJK 비교

이 벤치마크는 기존 swept-AABB `Query`와 현재 `BoxCast`에 **동일한 narrowphase
`ShapeCast`(GJK 반복)**을 연결해 비교한다. 두 방식의 결과는 실제
`world::castShapeClosest`와 대조한다.

## 실행

Windows Release:

```powershell
cmake --build build --config Release --target worldShapeCastBenchmark
.\build\tests\Release\worldShapeCastBenchmark.exe
.\build\tests\Release\worldShapeCastBenchmark.exe --csv > world-shape-cast.csv
```

Ubuntu/Linux Release:

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DZONAI_BUILD_TESTS=ON -DZONAI_BUILD_SANDBOX=OFF
cmake --build build-release --target worldShapeCastBenchmark
./build-release/tests/worldShapeCastBenchmark --csv
```

`--verify`는 6개 장면에서 각각 64번의 sweep을 수행한 뒤
동일한 최단 hit 여부와 fraction을 확인하고 **시간 측정은 하지 않는다**.
CI의 Debug/Release CTest는 이 경로를 사용한다.

## 비교 기준

- 20×20 도형 400개를 생성해 Static/Kinematic/Dynamic 세 Tree에 나누어 넣는다.
  Circle, Capsule, Box와 Sensor, QueryFilter를 포함한다.
  각 proxy의 fat AABB와 후보 shape의 speculative AABB는 같은 World에서 얻는다.
- 동일 입력·geometry·filter로 기존 swept-AABB Tree `Query` 경로와
  현재의 Tree `BoxCast` 경로를 실행한다. 두 경로의 callback에서
  동일하게 QueryFilter, clip한 speculative AABB 겹침, `MakeShapeProxy`,
  relative transform, `ShapeCast`, closest의 초기 겹침 제외 규칙을 적용한다.
- `BoxCast`의 클리핑은 Tree 순회 중 이루어지고, `Query`의 클리핑은
  후보 callback의 speculative AABB 검사 및 GJK 단계에서 적용된다.
  즉 각 구현의 실제 알고리즘 차이는 유지한다.
- 장면: diagonal, horizontal, short, wide-box, capsule, miss.
  각 장면의 입력 일부에는 마스크 제한이 있고 capsule은 canEncroach도 사용한다.
- `query_leaves` / `box_leaves`: Tree 후보 callback 호출 횟수.
  `query_gjk` / `box_gjk`: 실제 narrowphase ShapeCast 호출 수.
  `query_gjk_iterations` / `box_gjk_iterations`: 내부 GJK 거리 단계 반복 합계.
  세 수치는 각 장면 **64회 조회의 합계**다.
- `query_ns` / `box_ns`: Tree 조회 + 동일한 GJK 후보 판정까지 포함한
  복제된 closest 파이프라인의 1회 조회당 ns 중앙값.
  `world_ns`: 엔진의 실제 World::castShapeClosest 호출 시간으로 **현재 BoxCast만** 측정한다.
  이전 버전 World 코드 전체를 실제로 되돌리는 A/B 실행은 아니다.
- 속도는 7회 라운드, 라운드마다 동일한 64개 sweep을 4회 반복 측정한 중앙값이다.
  방법 실행 순서는 번갈아 수행한다. warm-up 결과는 버린다.
- CI의 Windows Release는 CSV를 참고값으로 출력하며, 시간 임계값은 CI pass/fail에 사용하지 않는다.
  runner의 CPU·부하·컴파일러가 달라지면 결과도 달라진다.

기존의 [후보 단계만 비교한 실험](dynamic-tree-box-cast-benchmark.md)과 결과를
같이 보고 판단해야 한다. 파이프라인 복제 경로의 수치는 실제 World 메서드의
추가 디스패치·메모리 접근 비용과 다를 수 있으므로 두 경로의 ns만 직접 비교한다.
실제 게임 프레임 FPS나 CCD 전체 성능은 아직 이 실험의 측정 범위가 아니다.
