# BroadPhase Shape Pair hashSet 성능 비교

BroadPhase는 활성 Contact pair를 \`hashSet\`에 보관하고, 새로운 후보 쌍의
\`MakeShapePairKey\`를 검사해 이미 존재하는 Contact의 재생성을 막는다.
기존 \`hash_set_test.cpp\`에는 \`std::unordered_set\`과의 정확성 대조가
있었지만, 연산별 시간 비교는 없었다.

\`hashSetPairBenchmark\`는 **동일한 Shape Pair key와 동일한 연산 순서**를
두 자료구조에 재생하는 독립 벤치마크다. World/Tree/solver 구현을
교체하지 않으며 BroadPhase 전체 수행 시간 측정도 아니다.

| 장면 | 연산 | 의도 |
| --- | --- | --- |
| \`cold_growth\` | 4096개 삽입, 각 key의 hit/miss 조회, 절반 삭제 | 초기에 Contact가 증가하며 리해시되는 비용 |
| \`steady_lookup\` | 3072개 사전 삽입, 24000회 포함 검사(hit 75%, miss 25%) | 기존 Contact 후보 조회 비용 |
| \`contact_churn\` | 2048개 사전 삽입, 32000회 add/remove/contains 혼합 | pair의 생성·파괴·재등록 비용 |

모든 key는 \`MakeShapePairKey\`로 만들며 \`hashSet\`의 빈 슬롯 sentinel인
0을 사용하지 않는다. 검증은 각 연산의 반환 값, 크기, 마지막 전체 key의
멤버십을 \`std::unordered_set\`과 비교한다. 특히 custom \`Add\`의 반환값은
**이미 존재하면 true**이므로 표준 컨테이너 \`insert().second\`와 반대다.

## 실행

CMake에서 \`ZONAI_BUILD_TESTS=ON\`으로 빌드한 뒤:

\`\`\`sh
ctest --test-dir build -R hashSetPairBenchmarkOracle --output-on-failure
# build 폴더/구성에 맞는 실행 파일을 직접 선택
./build/tests/hashSetPairBenchmark --verify
./build/tests/hashSetPairBenchmark --csv
\`\`\`

Windows 멀티 구성 생성기에서는 예를 들어
\`build/tests/Release/hashSetPairBenchmark.exe --csv\`를 사용한다.
출력 필드: \`scenario,operations,initial_pairs,custom_ns_per_op,std_ns_per_op,custom_over_std\`.
\`custom_over_std\`가 1보다 작으면 해당 측정에서 custom 구현이 더 빠르다.

성능 측정은 사전 삽입과 난수 입력 생성을 제외하고 실제 연산열만
\`steady_clock\`으로 측정하며, 각 반복은 독립된 새 Set으로 시작한다.
한 차례 워밍업 후 7개 시간의 중앙값을 사용한다.
조회·혼합 장면은 두 구현의 최대 적재율을 0.5 근처로 맞춰 예약하고,
초기 성장 장면은 재할당 비용을 포함한다. \`std::unordered_set\`의 노드
할당 비용과 custom의 연속 배열 특성은 실제 비교 대상에 포함된다.

**CI에서는 \`--verify\`만 실행한다.** 실행기 부하/Debug 최적화 차이가
성능 실패를 만들지 않도록 시간 임계값을 두지 않는다. Release에서
동일 컴퓨터·동일 컴파일러·같은 입력을 여러 번 측정한 수치만 비교하고,
이 미시 벤치마크의 승패만으로 실제 물리 시뮬레이션의 성능을 단정하거나
기본 BroadPhase 컨테이너를 교체하지 않는다.
