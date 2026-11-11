# zonai 환경 물리 시스템 통합 로드맵 (제안안 v0.1)

- 작성 기준: 2026-11-09, master d2d55482a0c07784cb9737d2cf82301b34543169 (PR #317 병합)
- 상태: 설계 검토용 초안. 이 문서의 신규 기능은 아직 구현되지 않았으며, 승인·검증 없이 기본 동작을 바꾸지 않는다.
- **다음 대화의 시작점:** [environment-status.md](environment-status.md) → 이 로드맵 → 관련 구현 및 테스트 → 최신 Git/CI.
- 이 문서는 기능의 목적·범위·불변식·검증 기준을 정의한다. 실제 완료 여부는 최신 코드와 PR/CI 결과로만 판정한다.

## A. 목표와 비목표

### 최종 목표
C++20 기반 2D 물리 엔진 zonai에서 환경이 게임플레이에 실질적으로 개입한다.

1. 바람이 돛·강체·Joint·장차 Cloth를 움직인다. 풍차, 로프 및 여러 부품에 응력이 전달된다.
2. 2D 뗏목은 물에 뜨고 물살·파도·바람에 반응한다. 바람과 물의 상호작용을 각각 제어하고 합성한다.
3. 움직이는 Body·지형은 바람과 물을 방해·변형한다. 선택한 Dynamic Body만 양방향 유체 반력을 받는다.
4. 장기적으로 모래·침식·퇴적·물길 변경·지형 파괴를 연계한다.
5. GPU는 필요한 고해상도 계산을 가속하지만 CPU 강체/Joint의 정확성과 안정성은 희생하지 않는다.
6. 2D 실험으로 검증된 설계만 장기적인 2.5D 지형 물/3D 엔진에 이식한다.

### 확정 지향 원칙 (구현의 성공 기준)
- **안정성 > 물리적으로 의미 있는 일관성 > 성능 > 시각적 사실성.** 시각화 전용 유체는 게임플레이의 힘을 마음대로 바꾸지 않는다.
- World/Joint solver는 초기 단계의 **권위 있는(authoritative) CPU 상태**다. 유체는 고유 속도/압력 상태를 유지하고 지정된 Step 경계에서 힘·충격량을 주고받는다.
- 유체 장애물 경계와 유체 반력을 받는 조건을 구분한다. 모든 Shape가 반드시 양방향 Schur 연산자에 들어갈 이유는 없다.
- 고정 Step, 명시적 단위, 보존/잔차 장부, 실패 상태, 재현 가능한 장면을 모든 단계의 일부로 둔다.
- 현재 구현의 기본 동작은 변경 후 대조 가능한 참조로 남긴다. 기존 옵션/제약/테스트는 합리적 사유와 회귀 없이는 삭제하지 않는다.
- 기능 구현 전에 사용 사례/물리 모델/비용/검증 기준을 한정하며, 기계적 복제보다 Box2D의 데이터 수명·접촉/Joint 설계 의도를 따른다.

### 이번 로드맵에서 의도적으로 제외
- 3D 전체 Navier–Stokes, 범용 오픈월드 실시간 완전 유체 보존, 모든 장애물에 대한 고정밀 자유수면/물보라 계산을 단일 Solver로 강제하지 않는다.
- GPU에 CPU World 상태를 매 substep 왕복시키는 설계를 성능 근거 없이 채택하지 않는다.
- 현재 엔진을 생산용 ECS/렌더 프레임워크로 전면 재작성하지 않는다.
- 2D 단면의 Sailing 데모를 3D 실제 요트 항해(풍상 항해/킬 측면력/양력)와 동일하다고 주장하지 않는다.
- 편의상 잘려진 힘/속도로 수치 폭주를 은폐하지 않는다. Clamp가 필요하면 계측·원인·물리 의미와 별도 옵션을 문서화한다.

## B. 현재 구현 감사 (구현 완료 ≠ 최종 모델 완성)

2026-11-09 master의 README, docs/fluid-field.md, docs/fluid-schur-profile.md, sandbox/rigidBodyDemo.cpp, src/fluid/fluidField2.h 및 최근 CI를 기준으로 확인:

- C++20 serial 2D World, 충돌/BroadPhase, Contact Solver, Island/Sleep/CCD, 다수 Joint, Ragdoll, 자동차, Chain/Spline 지형 데모 존재.
- 독립 fluidField2: 2D MAC 속도장, Semi-Lagrangian 및 선택형 보존 이류, Tracer, 점유/부분 Cut-cell, 이동 장애물 swept flux 근사, 압력/힘/운동량 진단 존재.
- 선택형 결합 Cut-cell PCG Schur (L + B M_body^-1 B^T), IC(0), bounded/indexed analytic geometry, Cold/previous-pressure Warm Start, 실제 월드 비교 회귀 존재. Warm Start는 **기본 OFF**.
- Sandbox는 World와 Fluid Step 횟수를 분리하고 그리드 크기/Static 장애물을 설정할 수 있음. 다만 결합 활성화 여부가 Fluid Step 스케줄에도 영향을 줌.
- 현재의 대표 Sandbox 경로는 형태별로 배열의 선두 3 Shape/Body에 대한 압력·Drag 경로가 고정되어 있음. 복합 Body의 여러 Shape를 Body 단위로 일관되게 결합하는 정식 정책은 미완성.
- windObstacleCouplingEnabled와 windSynchronizedCouplingEnabled가 경계 포함, 반력·시간 분할을 한꺼번에 좌우하는 부분이 있음. 개별 Body 정책이 아직 없음.
- 현 Drag는 4개 AABB 근사점에서 계산. 압력 traction과 별개이며 실제 표면 면적/양력/항력의 검증된 공기역학 모델은 아님.
- Cut-cell 이동계면의 완전한 질량·운동량/에너지 보존, 자유 수면, Cloth/Soft Body, 지형 침식, GPU 유체 Backend는 미완성.
- PR #316/#317은 opt-in previous-pressure Warm Start와 채택 경로의 World/Fluid 회귀를 병합. **수렴 실패 시 Warm→Cold 재시도는 아직 없음.**
- 지난 CI에서 Ubuntu 112/112, Windows Debug 115/115, Windows Release 52/52가 통과했음(해당 PR의 시점별 검증). 테스트 개수는 최신 CI로 재확인.
- 기존 docs/audit-status.md 및 docs/README.md 일부의 '다음 Joint는 Ragdoll' 문구는 역사 기록이며 최신 진행 상태를 대표하지 않음.

## C. 목표 아키텍처와 데이터 소유권

구조적 방향 (새 모듈 이름은 구현 시 재검토):

    [game/environment controls]   [terrain geometry and revisions]
                    |                     |
                    v                     v
      [environment sampling/scheduler; fixed tick]
                    |
      +-------------+------------------+
      |                                |
      v                                v
    [windField2 / fluidField2]       [water model 2D]
      |                                |
      +---------------+----------------+
                      |
      [obstacle snapshots and force/impulse feedback]
                      |
              [CPU World / Joint]
                      |
          [render/particle/GPU detail]

### C1. 소유권과 ID
- World가 Body/Shape ID, 실제 Pose/속도/질량/관성, Joint와 충돌을 소유. 유체는 World 포인터나 Body 수명을 영구 소유하지 않음.
- 별도 Fluid Interaction Registry(호스트/환경 계층, 구현 위치는 단계 S1에서 확정)는 stable bodyId + generation/world 수명 검사를 사용하고, 삭제·disable·scene reset·ID 재사용 시 오래된 설정을 안전하게 정리한다.
- 단일 Body에 여러 Shape가 있으면 **경계 기하는 Shape별**, 질량/관성/받는 총 반력은 **Body별** 집계한다. 동일 Body에 역질량을 Shape마다 중복 자유도로 더하지 않는다.
- Static geometry는 변경 revision으로 캐싱하고, Kinematic/Dynamic 경계는 매 Fluid tick 최신 Pose/속도를 가져온다. Static이 힘을 받지 않는다고 경계 계산이 무료가 되지 않는다.
- 복합/관절 Body 사이에는 유체가 직접 임의의 Joint 힘을 주입하지 않는다. force/torque 적용 후 기존 Joint solver가 반력을 전파한다.

### C2. 유체 상호작용 정책 (Body별)
정식 사용자 정책의 기본 모드:

| 정책 | 유체에 경계로 영향 | 유체가 Body에 힘/토크 | Schur Body 역질량 | 기본 대상 |
| --- | --- | --- | --- | --- |
| Disabled | 없음 | 없음 | 0 | 명시적으로 제외 |
| ReceiveOnly | 없음 | 있음(비보존 일방향) | 0 | Dynamic 기본 (제안) |
| ObstacleOnly | 있음(현재 Pose/속도 경계) | 없음 | 0 | Static/Kinematic 기본 |
| TwoWay | 있음 | 있음(정상 결합) | 실제 Dynamic M/I | Dynamic 명시적 opt-in |

- Static/Kinematic은 질량/역관성 0인 **운동학적 경계**이지 실제 양방향 힘 응답체가 아니다. Kinematic은 움직이며 유체를 밀어내지만 피드백으로 속도를 변경하지 않는다.
- Dynamic의 ObstacleOnly는 실제로 움직여도 유체 반력은 무시하는 **비보존 근사**다. 연출 또는 비용 절감을 위한 고의적 설정임을 Inspector에 표시한다.
- ReceiveOnly는 유체에서 속도를 읽어 힘을 얻되 장면의 유체 보존 장부가 닫힌 계가 아님을 표시한다. 근처 유체 운동량 주입은 하지 않는다.
- TwoWay는 Pressure + Drag의 사용 여부를 별도 정책 필드로 명시할 수 있으나 압력·운동량 교환이 중복 계산되지 않도록 통합 장부로 검증한다.
- 제안 내부 구성: mode + pressure/drag 옵션 + solver applicability + 진단. C++ 자료형·명칭은 S1 설계/검증에서 확정. 불필요한 bool 조합 폭발을 피한다.
- Body type을 변경했을 때 자동으로 기존 커스텀 정책을 덮어쓸지 여부는 명시적으로 정의해야 한다. 첫 구현안은 **기본값은 type별, 명시 설정은 존중하되 유효하지 않은 TwoWay는 거부/안전 강등 및 진단**이다.
- 동일 모드라도 공기/물별 물리 계수·매질 경계 참여는 독립 정책이 될 수 있다. 처음에는 하나의 Wind 정책으로 검증하고 공통 확장점만 확보한다.

### C3. 물리 단위·매질
- 위치 m, 시간 s, 선속도 m/s, 질량 kg, 힘 N, 충격량 N·s, 토크 N·m, 압력 Pa의 의미를 문서화한다.
- 현재 2D 솔버의 unit-depth(단위 두께) 가정을 유지하고, 면적/체적 및 실제 공기·물 밀도 적용에 필요한 유효 두께/스케일 규칙을 명시한다. 2D 선체 면적을 실제 3D 체적과 혼동하지 않는다.
- Wind와 Water의 밀도, 점성/감쇠 및 항력계수는 서로 구분한다. 임의 바람 힘 보정과 물리적 재료 파라미터를 분리한다.
- Dynamic 질량이 증가하면 동일한 힘에 의한 가속도가 감소해야 한다(F=ma). 정확한 풍하중은 물체의 투영 면적, 상대속도, 방향/형상에 따라 달라져야 하므로 질량만으로 바람 힘을 스케일하지 않는다.

### C4. 스케줄러 계약
- 고정 외부 Physics tick = dt. World substepCount와 Fluid substepCount는 독립 설정이고 실제 실행 횟수를 Diagnostics에 기록한다.
- 명시적 호출 순서를 고정한다: (1) 최신 World Shape/Body snapshot → (2) 경계/유체 Step 및 압력 풀이 → (3) Force/Torque/Impulse 집계 → (4) World/Joint Step → (5) 다음 snapshot. 내부 실제 그룹핑은 동등성 검증 후 결정.
- **연성 ON/OFF 비교에서 시간 분할을 동일하게 유지**하는 실험 경로를 제공하여 coupling과 scheduler 변화를 별도 평가한다.
- 두 매질을 함께 사용하게 되면 같은 물체에 공기/물 힘을 더한 뒤 한 번 World Step을 실행하는 일관된 순서를 정의한다. 같은 위치에 반력 두 번 적용을 금지한다.
- GPU 계산 결과를 매 World substep 동기 읽기하지 않는다. 게임플레이용 힘의 latency 계약(같은 tick/지연 tick)을 Backend별로 명시한다.

## D. 단계별 실행 로드맵 (작은 PR, 의존성 중심)

모든 단계 상태는 [environment-status.md](environment-status.md)에서만 변경한다.
아래 각 단계는 **착수 전 검증 장면 → 독립 구현 PR → 회귀/성능 증거 → 병합 → 상태 문서 갱신**의 순서.

### S0 — 기준선 확보·튀는 현상 분류 [최우선]
범위:
- 동일한 풍속·geometry·Body 초기 조건의 고정 재현 시나리오 제작.
- 양방향 ON/OFF × 동기화 ON/OFF × Drag=0/양수 × 0.5m/0.25m × World/Fluid substeps를 분리 계측.
- 프레임별 최대 MAC 속도 및 delta, 국소 발산, 압력 peak, PCG status/iterations/진짜 residual, pressure reaction/Drag impulse, Body velocity/ω, Cut-cell volume/swept mismatch, 부분 에너지/운동량 장부를 CSV로 출력.
- 실제 유체 급변 vs 힘 적용에 따른 Body 점프 vs 렌더 tracer 보간 문제를 따로 판단.
- PR #317의 World Cold/Warm differential을 참조 baseline으로 사용.
완료 조건: 재현 scene과 최초 비정상 프레임/관련 수치 기록; 어느 단계가 원인인지 확인 또는 가설별 격리 실험 완료.
금지: 원인 미확인 상태에서 무조건 속도/압력 clamp.

### S1 — Body별 Fluid Interaction 정책 / 수명 / 복합 Shape
범위:
- C2의 네 가지 모드와 API를 구현하고 Sandbox Inspector에 노출. 기존 global toggle은 호환성 shim/프리셋으로 축소.
- Static/Kinematic/ Dynamic 유형 및 Body enable/disable, destroy/ID reuse, Body type change, reset, Shape attach/detach에 대한 설정 유지/정리 규칙 정의.
- 하나의 Body에 여러 Shape가 붙었을 때 같은 질량/관성을 공유하며 총 pressure/drag feedback을 Body별 정확히 집계.
- obstacle snapshot builder, geometry dirty/static cache, force receiver 및 diagnostic record의 책임 분리.
- Dynamic ReceiveOnly가 유체를 막지 않는다는 의도와 TwoWay 선택형 비용을 명시.
완료 조건: 모드 4종의 유량·힘 매트릭스, static moving Kinematic 경계, compound Body 교차·회전, stale handle, 여러 Body 순서 불변, 기존 테스트 green.
독립 PR 후보: S1a 정책/수명 → S1b 모드별 boundary 생성 → S1c multi-shape Body aggregation → S1d Sandbox UI/성능.

### S2 — 압력·Drag·Warm Start 안정화
범위:
- Warm Start가 **채택된 후 수치 실패했을 때만** 같은 assembled operator/RHS로 Cold PCG를 재시도; Cold 성공/실패 양쪽 telemetry. Double pressure feedback 금지.
- 최초 시도/재시도 반복·elapsed·실제 잔차, topology change와 pressure spike 상관관계 기록.
- Dynamic 추가질량과 매우 가벼운 Body의 힘/속도 변동, 소형 Cut-cell, 단절 pocket, 회전·고속 경계, 정적 다수 장애물 회귀.
- Pressure Schur traction과 경험적 4점 Drag의 역할/중복 여부를 분리 실험. 더 정확한 표면 샘플/양력 모델은 S3로 이관.
- 수치 실패에 대한 현재 pressure 반력 미주입 정책 유지. 실패한 Warm 결과를 그대로 Body에 적용하지 않음.
완료 조건: 재시도가 Cold 기준 결과/상태를 복원하고 실패 통계가 정확함; 해당 조건에서 NaN/비유한 force/velocity 전파 없음; solver 실패를 UI/로그로 발견할 수 있음.
독립 PR 후보: S2a Warm retry → S2b pressure/drag differential → S2c high-motion robustness.

### S3 — 게임플레이용 Wind Force / 돛·Joint 데모
범위:
- 속도장 샘플 API는 매질/단위가 명확한 형태로 표준화하고 고정 비용 샘플링·최대 영향 거리·Body별 계수 제공.
- 표면/세그먼트 샘플에 상대속도 v_air-v_surface, 방향/투영 면적을 사용하여 force/torque 계산. 시작은 검증된 drag-only 모델; lift는 별도 회귀를 갖춘 opt-in 확장.
- 간단한 횡스크롤 2D 선체(강체), 돛/돛대(강체), Revolute/Weld/기타 필요한 Joint를 연결. 물은 아직 없어도 됨.
- 바람 방향, 풍속, 돛 각도, Body 질량/관성, Joint stiffness/motor, 연성 모드를 실시간 설정.
- Wind Field에서 static 벽의 풍하류, moving kinematic vane, dynamic sail 힘과 torque를 관측.
완료 조건: 돛 장력/토크가 Joint 반력으로 전달, 풍속과 돛 각도 변화에 일관된 움직임, substep 변화에서 NaN/Joint 폭주 없음, drag-only lift 미포함을 분명히 표시.
독립 PR 후보: S3a Surface force model → S3b joint-driven sail → S3c wake/visual differential.

### S4 — 2D 수면/물살/부력 모델 (자유 수면의 최소 기능)
범위:
- **현재 2D side-view 데모에서는** 수면 높이 η(x,t) 또는 수심 h(x,t)·수평 flux를 가진 보존적 1D 수면/천수 계열 모델을 우선 평가. 바닥 지형, 체적 유량, 외곽 유입/유출과 Wave Source 정의.
- 이 모델은 3D 바다의 2D horizontal shallow-water(2.5D)와 다르며, 파도 overturn/breaking/비산은 표현하지 못함을 명시.
- 선체 Shape의 잠긴 단면적 × 유효 두께 → 배제된 물의 체적; 부력 F_b=ρ_water*g*V_submerged 와 부력 중심의 토크. 물살과 선체 상대속도에 따른 drag/damping을 별도 적용.
- Static shoreline 및 Kinematic 이동 경계, Water 전용 Body Interaction 정책, 수위/유량/물질 보존 장부.
- 진짜 2D 대량 액체가 흘러넘치는/뒤집히는 장면이 필요해지면 heightfield만 늘리지 말고 별도 자유수면 MAC/LevelSet/FLIP/APIC 프로토타입을 설계심사.
완료 조건: 정수 중 부유 평형, 질량 다른 물체의 잠김량 차이, 기울어진 선체의 복원 토크, 물살 방향 변경, 파도 외력, 물질 보존/경계 유출 장부.
독립 PR 후보: S4a water 1D state/boundaries → S4b buoyancy → S4c surface-wave/current → S4d boundaries.

### S5 — 뗏목 통합 MVP
범위:
- Wind(돛의 공기력) + Water(부력/유속항력) + Body/Joint(돛대/선체/연결물)를 **한 fixed tick**에서 합성.
- 관찰할 토크/힘 장부를 wind/water/buoyancy/contact/joint로 분류. 매질 간 동일 면/힘 이중적용과 강체 위치 직접 보정 회피.
- 뗏목이 얕은 물에 접근하거나 바닥에 닿을 때 Contact와 Buoyancy를 함께 처리.
- 표류·풍하 이동을 기본으로 하고 **실제 3D 풍상 항해, 킬/양력에 의한 횡방향 항해는 별도 범위**.
완료 조건: 정수 안정 부유→바람으로 이동→파도에 흔들림→물살 변화에 반응→지형 접촉까지 하나의 재현 장면, 1/4/8 substep 비교 및 모든 force/torque finite.

### S6 — 지형 변화, 침식·퇴적·흐름 피드백
범위:
- 현재 Chain/Spline collision과 별도로 침식 가능한 질량/지형 높이/재료 상태 정의. 2D 단면의 모래 이동과 장기 2.5D heightmap 침식을 구분.
- 물 전단/유속에 의한 침식량, 흐름에 의한 운반, 속도 감소 시 퇴적, 마른 모래의 wind transport를 단계별 opt-in 모델로 구현.
- 침식 계산이 만드는 지형 변경은 snapshot revision과 경계 invalidation으로 전파; 충돌 Chain/Spline 재구성, 재삽입/Contact 갱신, 정적 경계 캐시 무효화 원자적 처리.
- 지형 질량 ledger/침식·퇴적 budget, 아주 작은 지형 변화에 대한 threshold·배치 갱신; 임의 물질 생성/손실 진단.
완료 조건: 수로를 파면 물길 변경→주변 지형에 운반/퇴적→다음 Step 물길 재변화; 역으로 마른 모래에 바람을 가하면 퇴적 패턴; stable Body ID·Contact 오류 없음.
독립 PR 후보: S6a terrain material/grid → S6b erosion/transport → S6c collision regeneration → S6d sand/wind.

### S7 — Cloth/Soft Body와 Wind/Water
범위:
- Cloth/Soft Body 입자·변형 제약, 질량·충돌·부착/Joint coupling의 단순 reference 구현.
- 각 요소/삼각형/선분의 local wind sample와 상대속도를 사용해 힘 전달. rigid sail과 soft sail을 같은 바람에서 대조.
- 큰 변형체를 매 substep 정확한 Cut-cell 경계로 rasterize할지 vs ReceiveOnly 근사로 처리할지 모드별 선택.
- Water 부력/항력 결합은 cloth solver 안정화 후 별도 범위.
완료 조건: 바람 없는 정지/중력/장력·에너지 회귀, 강한 바람에서 유한 상태, 부착된 강체/Joint로 반력이 전달되는 검증.

### S8 — GPU 가속 (S0부터 자료 계약, S2 이후 독립 PoC 병행)
범위:
- **G0 (즉시):** CPU Field profiler, 데이터 layout·필수 buffer 목록, 동일 케이스 기준선, GPU 읽기/동기화 빈도 목표.
- **G1 (S2 이후):** D3D11 Compute Shader 기반 velocity advection/tracer/visual detail PoC. CPU reference와 입력 snapshot/장애물 경계/출력 비교; 권위 있는 CPU pressure coupling 변경 금지.
- **G2 (S4 이후):** 물 높이장/파도/연기 등 규칙적 grid의 GPU compute. Rendering과 물리 데이터가 같은 장치에 머무는 경로부터 선택.
- **G3 (측정으로만 결정):** GPU pressure projection/결합 Solver 검토. Jacobi/Multigrid/CG류 대비 현재 CPU IC(0)와 정확도·총 Frame 비용·readback·latency를 검증. 단순 코드 이식 금지.
- GPU/CPU 물리 경계는 호스트가 명시; GPU 데이터가 한 tick 늦을 경우 동일 tick과 물리적으로 다를 수 있음을 옵션으로 보존/표시.
- D3D11 전용 Backend는 Sandbox/renderer 영역에 두고 zonai CPU library의 Linux/헤드리스 빌드를 깨지 않는다.
완료 조건: 같은 장면에서 CPU/GPU의 결과 차이, Grid/Body scaling, upload/readback/synchronization, p50/p95 step, VRAM을 측정해 채택 근거 확보. 빨라지지 않으면 CPU 유지.
참고: NVIDIA GPU Gems의 grid-advection/pressure 및 고체 경계, Microsoft D3D11 staging readback 문서.

### S9 — 장기 확장 / 3D 이식
범위:
- 넓은 해안 및 2.5D terrain에는 horizontal 2D Shallow Water와 GPU detail waves를 별도로 설계.
- 자유수면/파쇄 파도/튀는 물체 효과는 요구와 성능에 따라 FLIP/APIC 등 국소 모델을 연구.
- 3D World/Collision/Joint, 3D 부력·돛 양력/측면력, 3D Terrain voxel/Marching Cubes/Dual Contouring는 **별도 상위 로드맵**에서 단계/검증 확정.
완료 조건: 2D 검증 모델과 이식 범위, 3D에선 더 이상 유효하지 않은 근사 항목을 명문화.

## E. 검증 매트릭스와 합격 규칙

### 공통 자동화 규칙
1. 수학 경계·정확한 상태 전이·이산 보존 불변식은 Unit Test, World 연동은 고정 Step Integration, 게임플레이는 Sandbox headless/visual, 비용은 전용 Release Benchmark로 구분.
2. Debug/Release runtime 검사 코드가 assert 제거로 사라지지 않아야 함. 새로운 검증에 side effect가 있는 assert 금지.
3. 기존 test의 수렴 tolerance를 변경할 때는 개선이나 퇴행을 정량화. CI 실행시간(ms)은 일반적으로 합격 기준이 아니라 정보 제공/회귀 조사의 계기.
4. PR 단위로 변경 전/후 결과와 baseline, 위상/방향, 수렴/잔차/힘/운동량, Windows/Ubuntu CI 상태를 기록.
5. 기본 옵션 변경은 기존 장면 differential과 설정 초기화/리사이즈/Body lifetime/부하까지 확인한 후 별도 PR.

### 필수 시나리오
| 분류 | 상황 | 핵심 증거 |
| --- | --- | --- |
| Body 타입 | Static, 이동 Kinematic, Dynamic, Body disable/enable | 경계 속도와 압력 반력 0/양수, stale ID 무효 |
| 연성 | Disabled/ReceiveOnly/ObstacleOnly/TwoWay | 경계 포함/힘 전달의 정확한 매트릭스 |
| 복합 형상 | 한 Body에 Circle+Capsule+Polygon, 공유 COM | 역질량 중복 없음, 힘/토크 합계 일치 |
| 질량 | 아주 가벼움/중간/무거움, 낮은 관성/빠른 회전 | 동일 힘 가속도와 torque, 솔버 finite |
| 속도/위상 | 고속 이동, 급격한 회전, 작은 Cut-cell, 분리 pocket | PCG 상태/진짜 residual/swept mismatch, 유속 spike |
| 시간 | World=1/4/8, Fluid=1/2/4/8, 동일 cadence 비교 | dt 보존, 실제 호출 횟수, endpoint states |
| 격자 | h=0.5m/0.25m, domain resize, obstacles 다수 | 물리 오차/Step/Memory scaling |
| Joint | 바람이 돛을 비스듬히 밀고 구조물이 구속 | joint reaction, Body torque, finite energy |
| 물 | 정수, 물살, 파도, 지형 경계, 외부 유입/유출 | buoyancy/flux/volume budget, 접촉 안정 |
| 지형 | 파임/쌓임, 경계 통과, 변경 후 충돌 재생성 | 물질 ledger, boundary revision, Body/Contact valid |
| GPU | 작은/큰 grid, 0/다수 Bodies, readback on/off | CPU 결과 차이, p50/p95 wall, sync/VRAM |

### 진단 데이터 계약 (제안)
- simulationTick, dt, worldSubsteps, fluidSubsteps, gridSize, activeCells, boundaryFaces
- bodyCount, pressureCoupledBodyCount, static/kinematic/dynamic obstacle faces, interaction policy count
- maxFluidSpeed, maxDeltaFluidSpeed, divergenceBefore/After, truePcgRelativeResidual, pcgStatus/Iterations
- pressureColdRetries, warmAccepted, perBodyPressureForce/Torque, dragForce/Impulse, netFluidMomentumDelta
- cutCellVolumeChange, sweptFluxMismatch, mass/energy budget residual와 사용된 근사 범위
- fluidGeometryMs, fluidPressureMs, fluidAdvectionMs, totalFluidMs, worldMs, renderGpuMs, readbackMs
- Stage/solver 버전 및 장면 파라미터를 CSV header와 함께 저장. 값이 유효하지 않을 때 0으로 숨기지 않고 상태 플래그 제공.

### 성능 합격 정책
- 60fps(16.67ms)는 참고 전체 프레임 예산이지 현재 솔버가 만족하는 실측값이 아님. Physics 예산은 목표 플랫폼/장면 규모를 먼저 정하고 확정.
- 성능 개선은 실제 **전체 Fluid + World + transfer** 기준으로 비교하며 낮은 PCG 반복 수만으로 성공 판단 금지.
- 두 격자, 여러 Body/Static 장애물 수, Fluid substeps, 1회/연속 프레임을 모두 비교.
- 정밀도 저하나 누락된 반력을 감춘 '빠른 실패'와 수렴한 계산의 실행 시간을 직접 비교하지 않음.
- GPU 도입의 채택 조건은 end-to-end 절감 + gameplay latency/force consistency 충족 + 개발/이식 비용의 실익임.

### 프로젝트 구현 컨벤션
- C++20, Box2D 알고리즘과 자료 수명 의도를 보존한다. 원시 배열/매크로를 무작정 새로 늘리지 않고 필요 시 std::array/std::span/constexpr·정확한 정수 타입을 사용한다.
- zonai에서 유지하는 camelCase 명명, 선언/정의 순서 일치, 기존 private member 규칙과 #pragma region 구획을 존중한다. 주석은 물리적 의도·불변식·검증 한계 위주로 짧고 명확하게 작성한다.
- 기존 소스의 파일 배치/줄바꿈/인코딩을 보존하고, unrelated rename·reformat이나 전체 리팩터링은 작은 기능 PR에 섞지 않는다.
- new public field/method는 단위·수명·default/disabled 동작, invalid input, reset·resize 동작을 명시한다.
- Windows CI와 로컬 빌드는 README의 CMake preset을 따르고, Linux headless/library/test를 계속 유지한다. D3D11 Compute 코드는 CPU library의 link dependency가 되지 않는다.

## F. 변경 관리 / PR 절차

- 우선순위: (1) 사용자 재현 버그와 물리 안정성 (2) 개별 Body 연성 (3) Joint+Wind 게임플레이 데모 (4) Water/부력 (5) Terrain/Cloth/GPU와 대규모 확장.
- 한 PR은 하나의 실험 질문/수치 가정만 검증하도록 나눔. 성능 실험은 기존 경로 옵션과 differential test 및 회귀 수치를 포함.
- 테스트 불통과/성능 근거 부족은 Draft 유지 또는 비교 실험 중단. 안전 근거 없이 단지 PR을 닫기 위해 기본 모드를 변경하지 않음.
- 병합 완료 시 [environment-status.md](environment-status.md)의 master SHA, 완료 PR, 검증 CI, 현 단계, 다음 원자 PR을 갱신한다.
- 코드/문서 충돌 시 최신 master의 실제 함수와 tests가 우선. 이 문서는 미래 의도를 나타내고 실행된 사실은 코드·PR·CI로 확인.
- 사양이 바뀌면 결정 이유/영향/이전 선택과의 차이를 Decision Log에 기록.

## G. 다음 작업 후보 (S0/S1 초기 쪼개기)

| 우선 | 작업 | 기존 위치/참조 | 합격 조건 |
| --- | --- | --- | --- |
| 1 | S0a 동일 cadence에서 연성 ON/OFF / Drag 스파이크 재현 및 CSV | sandbox/rigidBodyDemo.cpp, tests/sandbox/wind_flow_demo_test.cpp, fluidField2 | 재현 scenario + 최초 비정상 구간 데이터 |
| 2 | S2a Warm→Cold PCG retry 안전 장치 (S0a와 독립 가능) | src/fluid/fluidField2.cpp, tests/fluid/fluid_field_test.cpp | Warm 실패 후 Cold 결과 동등, feedback 1회 |
| 3 | S1a per-Body mode 정책과 stale-ID 수명 시험 | sandbox + 향후 환경 Adapter, World Body IDs | mode matrix, delete/reuse 안전 |
| 4 | S1b multi-shape aggregation·Static/Kinematic 경계 분리 | fluidObstacle snapshots, Schur body terms | 다중 Shape Body 물리 반력·mass 중복 없음 |
| 5 | S3a 바람 표면력 벤치마크 | wind sampling, World force & torque | 풍속/면적/질량·회전 일관성 |

이 순서는 **초안**이다. 요구 변경이나 S0 측정 결과에 따라 바뀔 수 있다. S0a의 첫 PR은 새로운 유체 모형이 아니라 비교 가능한 진단과 작은 재현 장면을 추가하는 것으로 제한한다.

## H. 새 대화 인수인계 프로토콜

새 대화의 작업자는:
1. 먼저 [environment-status.md](environment-status.md)와 **본 문서**를 읽는다.
2. GitHub master HEAD, 열려 있는 PR, 최근 CI와 변경 파일을 조회해 상태 문서를 검증한다. 과거 ChatGPT 대화나 오래된 docs/README만으로 완료를 가정하지 않는다.
3. 가장 높은 우선순위의 미완료 atomic task를 고르고 **범위/비범위/수정 예상 파일/물리 불변식/사전 테스트/완료 조건**을 간단히 보고한다.
4. 변경 전 최소 재현 또는 benchmark를 확보하고 작업→테스트→CI→PR→Merge/보류 결정을 수행한다.
5. PR 결과와 실패/한계·최신 HEAD 및 다음 1~3 작업을 status 문서에 기록한다.
6. 계획 문구만 업데이트한 PR에서는 엔진 코드가 바뀌거나 물리 성능이 개선됐다고 주장하지 않는다.

### 다음 대화에 붙여넣기 위한 최소 프롬프트
"zonai 환경 물리 계획을 이어가자. GitHub buddas-bible/zonai의 docs/roadmaps/environment-status.md와 environment-simulation.md, 현재 master/PR/CI를 먼저 확인하고, status의 Next atomic PR 하나만 범위·검증 기준대로 구현해. Box2D 의도, C++20 코딩 스타일, 기본 OFF/기존 동작 보존, CPU World 물리 기준을 준수하고 검증 후 status를 갱신해."

## I. Decision Log / 미결정 설계

- 2026-11-09: Body별 4모드, CPU gameplay + 선택 GPU detail, Wind/Water 별도 모듈, 실험 우선 방식 **제안**. 승인/최초 구현 전.
- 미결정 1: Registry의 실 소유 위치 (Sandbox prototype → reusable EnvironmentAdapter vs World extension).
- 미결정 2: Flow-Pressure와 Drag의 중첩 모델 및 압력 실패시 receiver 정책.
- 미결정 3: 2D Water 최소 모델의 상태량과 경계/파도 전달 방식.
- 미결정 4: GPU gameplay authoritative 전환의 latency와 동기화 예산.
- 미결정 5: 실제 목표 플랫폼/해상도/물체 수에 따른 physics frame budget과 품질 단계.
- 미결정 6: 2D 모래 모델과 향후 2.5D Sandcastle 스타일 erosion 모델의 데이터 공유 범위.

## 참고 자료 (공개 검증)
- Box2D simulation: https://box2d.org/documentation/md_simulation.html
- NVIDIA GPU Gems, GPU fluid: https://developer.nvidia.com/gpugems/gpugems/part-vi-beyond-triangles/chapter-38-fast-fluid-dynamics-simulation-gpu
- NVIDIA GPU Gems 3, obstacle boundaries and real-time fluids: https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-30-real-time-simulation-and-rendering-3d-fluids
- Microsoft D3D11 staging readback: https://learn.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_usage
- Crimson Desert engine public report (Shallow Water; GPU cloth/hair, no blanket claim water runs on GPU): https://crimsondesert.pearlabyss.com/en-US/News/Notice/Detail?_boardNo=40
- Sandcastle (Bubblebird Studio) public game description; no unverified proprietary solver claim: https://store.steampowered.com/app/3216520/Sandcastle/
