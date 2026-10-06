# Inspector와 공통 충돌 설정

현재 화면은 한글이다. 공통 설정의 기본 표시는 레이어 01이며, 표시할 레이어 목록에서 64개 중 필요한 레이어를 최대 8개씩 골라 편집한다. 서로 다른 번호 구간도 함께 선택할 수 있다. 대칭 관계는 표의 위쪽에서 한 번만 편집하고, 표시에서 숨기는 것은 충돌 규칙을 바꾸지 않는다. 매트릭스 설정과 표시할 레이어는 데모 전환/초기화에도 유지된다.

소속 레이어는 Inspector의 충돌 마스크에서 바로 편집한다. 개별 마스크/그룹은 추가 제한 (고급)을 펼쳐 편집한다. 월드·마우스 조인트·물리 정보 표시 설정도 필요할 때 펼친다. 기본 화면에 64×64 관계나 모든 고급 설정을 전개하지 않는다.

Object Inspector는 Transform, Physics quantities, Collision material, Collision mask, Sleep and CCD, Sensors로 나눈다. 필요한 분류를 펼쳐 편집하며 기존 World setter를 사용한다.

Collision mask의 Category membership은 shape가 속한 레이어, Collision partners는 충돌을 허용할 상대 레이어다. 숫자 입력 대신 체크 목록을 사용한다. Layer 01은 bit 0, Layer 64는 bit 63이며 All/None으로 전체 선택/해제를 할 수 있다. 여러 레이어에 속할 수도 있다. 기본 Category는 Layer 01, Mask는 전체 허용이다.

Project collision matrix는 개별 Inspector 밖의 공통 설정이다. 행과 열의 레이어 쌍을 체크하면 허용하고 해제하면 제외한다. 대칭 관계이므로 한쪽을 바꾸면 반대쪽도 바뀐다. 같은 레이어끼리의 충돌도 설정할 수 있다. 이 설정은 현재 실행의 모든 데모에 공유되며 데모 전환과 Reset에도 유지된다. 프로그램 종료 후 파일로 저장하는 프로젝트 기능은 아직 없다.

충돌 판정은 공통 매트릭스와 기존 shape filter를 모두 통과해야 한다. 여러 Category가 있다면 매트릭스에서 허용된 쌍이 하나 이상 있어야 한다. 이어서 양쪽 Mask가 상대 Category를 허용해야 한다. 같은 non-zero Group index는 기존 Box2D 방식으로 개별 Category/Mask를 우선하지만, Category가 지정된 쌍의 공통 금지를 우회하지 않는다. Category가 0인 shape는 레이어 쌍 규칙의 대상이 아니며 기존 Group/Mask 판정만 따른다. 공통 설정의 기본값은 모두 허용하여 기존 동작을 유지한다.

World는 매트릭스 변경 시 기존 contact/cache를 제거하고 proxy를 다시 검사하며 움직이는 body를 깨운다. 정지한 shape도 새로 허용된 접촉을 찾는다. 센서와 CCD에도 같은 규칙을 적용한다. 동일한 설정을 다시 적용하면 수면이나 cache를 바꾸지 않는다. 이 매트릭스는 zonai의 공통 정책이며 Box2D의 shape filter 자체를 변경한 기능은 아니다.

`worldQuerySensorTests`는 접촉 제거/재생성, object Mask와 Group 우선순위, 정지 shape 재검사, no-op 수면, CCD와 continuous sensor를 검사한다. `sandboxDemoTests`는 데모 전환/Reset의 설정 보존과 실제 ImGui headless frame의 Layer 64 토글 및 매트릭스 대칭을 검사한다. 창/GPU/OS 입력과 화면 배치는 별도 확인이 필요하다. 기존 assert 기반 테스트의 Release 공백은 [테스트 문서](../tests/README.md)를 따른다.
