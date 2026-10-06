# 코딩 스타일

zonai는 물리 알고리즘을 직접 구현하며 학습하는 프로젝트다. 작성자가 직접 고친 현재 코드와 이전 HyruleEngine, Phyzzle C++ 원본에서 반복되는 패턴을 기준으로 삼는다. 공백만 맞추는 것보다 계산과 제어 흐름이 읽히는 구조를 우선한다.

## 참고한 코드

| 원본 | 확인한 패턴 |
| --- | --- |
| HyrulePhysics/CollisionSystem.cpp, ComputeImpulse | 물리 개념 설명 → 작용점 벡터 → 점의 속도 → 상대속도 → 임펄스 크기 → 적용 |
| HyrulePhysics/RigidBody.cpp, ApplyImpulse | 선형·회전 계산 구분, P = F * dt와 dV/dW의 유도 설명 |
| HyrulePhysics/RigidBody.h | 물리량 옆의 짧은 의미 주석, 관련 데이터와 GetSet 묶음 |
| HyrulePhysics/Manifold.cpp | 예외 조건 처리 후 계산, 관련 조회·변경 함수 배치 |
| HyruleMath/Matrix3x3.cpp | 행렬식·역행렬의 중간값, 퇴화 조건의 이유 설명 |
| HyruleEngine/HRigidBody.cpp, Test/PlayerController.cpp | 물리 호출과 입력 처리 구분, 직접 읽히는 동작 이름 |
| Phyzzle/ZonaiPhysicsBase/ZnDistanceJoint.h | 현재 거리·최소/최대 거리·스프링 계수의 의미를 헤더에서 설명 |
| Phyzzle/ZonaiPhysicsX/DistanceJoint.cpp, RigidBodyHelper.h | 선언에 맞춘 구현 배치, 검증과 실제 동작 사이의 여백, 질량·속도·힘 묶음 |
| Phyzzle/ZeldaClient/Player.h/.cpp | 입력·카메라·상태·초기화 region, 데이터의 짧은 주석 |
| Phyzzle/ZeldaClient/AttachHoldState.h/.cpp | 목표점 계산·스프링 계산·속도 적용의 단계, local/world 구분 |
| Phyzzle/ZeldaClient/AttachSystem.h | 작용점 좌표 변환의 이유, 긴 흐름은 블록 주석으로 설명 |
| Phyzzle/ZeldaClient/RewindSystem.cpp, Spring.h | 초기 조건 검사, 누적값과 중간 계산값을 드러내는 변수 |
| zonai의 d17a388, cad8e608, 7289565 커밋 | 함수·수식의 불필요한 줄바꿈 제거, 한국어 계산 주석, r_a/v_p2 등의 수학 표기 |

Phyzzle 기준은 포트폴리오의 원래 C++ 물리엔진·게임 클라이언트 소스다. 현재 Unity 이식 코드, 외부 라이브러리, 과거의 미완성 코드와 주석 처리된 실험을 일괄적으로 작성자의 스타일이라고 간주하지 않는다. 원본마다 명명·들여쓰기 차이는 있으므로 현재 zonai의 명시적 규칙을 우선한다.

## 이름과 배치

- 새 공개 함수와 일반 변수는 lowerCamelCase, private 멤버는 현재의 trailing underscore 표기를 유지한다. 기존 공개 API의 이름 변경은 별도의 변경으로 검토한다.
- 물리 계산에서는 작성자가 직접 쓴 r_a, r_b, v_a, w_a, v_p2, v_r 같은 수학 표기를 사용한다. 일반 API에 이 표기를 확장하지 않는다.
- bodySimA와 bodyStateA처럼 물리 상태의 종류와 A/B 역할을 이름에 드러낸다. localAnchor와 world target의 좌표계도 분명히 한다.
- 헤더의 함수 선언과 CPP 정의 순서를 맞춘다. 관련 초기화·입력·계산·조회 함수를 가까이 둔다.
- 긴 파일의 의미 있는 코드 묶음에 #pragma region을 사용하고 endregion에도 이름을 붙인다. 짧은 구조체나 함수마다 형식적인 region을 만들지 않는다. 작성자가 직접 제거한 Mouse Joint·조인트 슬롯의 region도 이 기준에 반영했다.
- 기존 필드 순서와 객체 레이아웃을 스타일 때문에 바꾸지 않는다. 예전 프로젝트의 PascalCase, 인자 앞 underscore, tab 규칙보다 현재 zonai 규칙을 우선한다.

## 함수와 계산

- CPP 함수와 실제 작업이 있는 제어 블록은 여는 중괄호를 다음 줄에 둔다. 단순한 inline 접근자는 기존 한 줄 형식을 유지할 수 있다.
- 단순한 return·continue guard는 중괄호 없이 짧게 쓴다. 조건이 짧으면 한 줄, 조건이 길면 다음 줄에 동작을 둔다. else가 연결된 분기나 여러 문장의 작업은 블록을 유지한다.
- 한 줄에 여러 대입이나 분기를 몰아쓰지 않는다. 계산 단계 사이에 여백을 둔다.
- 함수 인자와 수식을 항목마다 나누지 않는다. Contact의 Body A/B 상태처럼 의미가 같은 인자는 같은 줄에 묶고, 서로 다른 역할은 줄을 나눌 수 있다. 독립적인 의미를 가진 계산 단계라면 이름 있는 중간값으로 나타낸다.
- 유효성 검사, 좌표 변환, 제약 계산, 상태 적용이 순서대로 읽히게 한다. 작은 실험을 위해 범용 상태 머신이나 추가 인터페이스를 만들지는 않는다.
- 수식 분해 시 부동소수점 연산 순서와 각 제약이 읽는 최신 속도를 유지한다. 임펄스를 적용한 뒤 다음 제약의 속도를 다시 계산한다.
- 짧은 함수도 guard 뒤와 반환 전에 필요한 여백을 두며, 현재 코드의 들여쓰기를 유지한다.
- Body·Shape handle 조회는 먼저 bodyIndex·shapeIndex를 지역 변수로 구한 뒤 사용한다. 조회의 실행 순서와 횟수는 유지한다.
- 검증 묶음, 참조 취득, 계산, 적용·반환 사이를 빈 줄로 구분한다. 관련 변수마다 무조건 빈 줄을 넣지는 않는다.
- 함수 인자로 전달하는 lambda는 콜백 자체의 흐름이 보이도록 배치한다. 여러 데모·검사 입력·메뉴 항목의 데이터 목록은 항목별 행을 유지한다.

## 주석

- 헤더의 짧은 주석은 물리량의 의미, 단위나 범위, 좌표계를 설명한다. 긴 제약 조건은 선언 위에 둔다.
- 계산 주석은 한국어로 개념과 이유를 설명한다. 작용점 속도, 임펄스 분모, 선형·회전 적용처럼 물리 단계와 대응시킨다.
- 수식 유도나 여러 단계의 흐름은 블록 주석으로 묶는다. 한 줄이면 충분한 이유를 장문으로 늘리지 않는다.
- Box2D를 따르는 이유는 해당 코드 옆에서 설명한다. warm start, bias, softness 등의 용어는 코드와 대응되도록 필요한 곳에 남긴다.
- 대입을 그대로 읽어주는 주석, 모든 함수에 같은 설명을 붙이는 방식, 영어·한국어를 불필요하게 섞는 문장은 피한다.
- 데모는 입력 → 목표점 갱신 → 물리 적용 → 취소/수명 흐름이 읽히게 한다. 사용자 화면에 구현 용어를 추가하지 않는다.

스타일 변경 후에는 기존 조인트 단위·World·데모 검사와 필요한 빌드를 실행한다. 새 동작이 없으면 포맷만 검사하는 테스트를 저장소에 추가하지 않는다.

## 저장소 적용

작성한 src·sandbox·tests C++ 전체에 위 기준을 적용한다. .clang-format은 중괄호·들여쓰기·인자·수식 배치의 기본값을 제공한다. 80/120열에 맞추기 위해 물리 수식과 인자를 잘게 나누지 않도록 넓은 열 한도를 사용한다. 의미별 여백, Body A/B 인자 묶음, 데이터 표의 행, region의 실제 용도는 코드 검토로 맞춘다. third_party와 build는 .clang-format-ignore에 지정한다.

2026-10-06 작성자가 추가로 수정한 World 조회·guard, bodyShape의 참조/검증 여백, Contact 인자 묶음, constraintSoftness의 반환 및 짧은 조인트 region 제거를 이 기준에 반영했다.

작성자가 직접 구분한 작용점·거리·임펄스 적용 사이의 여백과 guard의 줄바꿈은 자동 포맷보다 우선한다. 조건의 이유를 설명하는 주석은 조건 앞에 둔다. testShapePoint의 inside 갱신처럼 작성자가 중괄호 없이 다음 줄에 둔 단일 대입도 그 배치를 유지한다.
