#include "demo.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>

namespace zonai::sandbox
{
namespace
{

constexpr float FIXED_TIME_STEP = 1.0f / 60.0f;
constexpr int MAX_STEPS_PER_FRAME = 8;
constexpr std::array entries
{
    demoEntry{ demoKind::playground, "강체", "강체 실험", "캔버스: A/D 유지로 원 밀기, 스페이스로 점프, S로 상자 회전. 왼쪽 드래그로 동적 오브젝트 잡기. 오른쪽 클릭으로 원을 커서 방향으로 밀기.", { 0.0f, 0.5f }, 55.0f },
    demoEntry{ demoKind::terrainChain, "지형", "체인 지형", "초록 화살표가 Chain의 충돌 앞면입니다. 원·캡슐·상자를 선택해 앞면·뒷면 접근과 이음새 통과를 비교하세요. 점을 드래그해 지형을 편집하고 시험 물체 복귀로 지형·물성을 유지하며 반복합니다. 물체는 왼쪽 드래그로 잡을 수 있습니다.", { 0.0f, 0.5f }, 55.0f },
    demoEntry{ demoKind::splineTerrain, "지형", "스플라인 지형", "보라색 제어점을 드래그해 곡선 지형을 편집합니다. 곡선에서 10px 이내를 Ctrl+왼쪽 클릭하면 제어점을 삽입합니다. 주황색은 선택한 점입니다. Shift+왼쪽 드래그로 시험 위치를 고릅니다. 원·캡슐·상자로 앞면·뒷면 접근과 이음새 통과를 비교하세요. 시험 물체 복귀로 반복하며 물체는 왼쪽 드래그로 잡을 수 있습니다.", { 0.0f, 0.5f }, 55.0f },
    demoEntry{ demoKind::distancePendulum, "조인트", "거리 조인트 진자", "캔버스: A/D 유지로 진자 밀기, 스페이스로 충격량 가하기. 왼쪽 드래그로 진자 잡기, 오른쪽 클릭으로 커서 방향으로 밀기. 스프링·주파수·감쇠·거리 제한·축 방향 충격량을 조절합니다. 보라색 선은 고정점을 잇고 초록·빨강은 거리 제한입니다.", { 0.0f, -0.5f }, 110.0f },
    demoEntry{ demoKind::revoluteHinge, "조인트", "회전축과 막대", "캔버스: A/D 유지로 막대 밀기, 스페이스로 위로 밀기, S로 회전 충격량 가하기. 왼쪽 드래그로 막대 잡기. 오른쪽 클릭으로 커서 방향으로 밀기. 연결점은 유지되고 상대 회전은 자유롭습니다.", { 0.0f, -0.8f }, 130.0f },
    demoEntry{ demoKind::wheelSuspension, "조인트", "서스펜션과 바퀴", "캔버스: A/D 유지로 축 옆으로 밀기, 스페이스로 바퀴 위로 밀기, S로 회전 충격량 가하기. 왼쪽 드래그로 바퀴 잡기. 오른쪽 클릭으로 커서 방향으로 밀기. 축 방향 스프링·감쇠와 자유 회전을 비교합니다.", { 0.0f, -0.2f }, 150.0f },
    demoEntry{ demoKind::prismaticRail, "조인트", "프리즈매틱 레일", "모터+제한으로 시작합니다. 수평·경사 레일을 전환하고 약한 모터·오르기·부하 제동을 비교하세요. A/D는 레일 축 방향 힘, Space·S는 수직·회전 충격입니다. 화면 밖으로 이동하면 시작 위치 복귀를 누르세요. 스프링은 위치, 모터는 속도, 제한은 범위를 제어합니다.", { 0.0f, 0.0f }, 110.0f },
    demoEntry{ demoKind::mouseJointPlayground, "조인트", "마우스 조인트", "캔버스: 질량이 다른 상자를 왼쪽 드래그로 잡아 움직입니다. 주파수·감쇠·최대 힘을 바꾸며 같은 설정에서 질량에 따른 추종 차이를 비교합니다.", { 0.0f, 0.0f }, 90.0f },
    demoEntry{ demoKind::weldPair, "조인트", "웰드 조인트", "자유 상자 쌍과 고정 지지·중력 하중 장면을 비교하세요. 강체·선형 탄성·회전 탄성을 전환해 연결점 처짐과 굽힘을 관찰합니다. A/D·Space·S로 오른쪽 상자에 충격을 주고 시작 상태 복귀로 반복 시험하세요.", { 0.0f, 0.0f }, 100.0f },
    demoEntry{ demoKind::motorJointPlayground, "조인트", "모터 조인트", "Motor Joint의 상대속도 actuator와 목표 transform spring을 비교합니다. 제동·선형·회전·스프링·결합 프리셋으로 속도 목표와 위치/각도 복원의 차이를 관찰합니다.", { 0.0f, 0.0f }, 95.0f },
    demoEntry{ demoKind::moverJointPlayground, "조인트", "무버 조인트", "Mover Joint는 두 Body의 COM 상대 선속도만 제어하고 회전은 건드리지 않습니다. 수평·수직·대각선·축별 힘 한도 프리셋으로 character mover용 velocity actuator를 비교합니다.", { 0.0f, 0.0f }, 95.0f },
    demoEntry{ demoKind::pogoJointPlayground, "조인트", "포고 조인트", "A/D로 평지·단차·경사를 이동하고 Space로 틈을 뛰어넘으세요. Pogo는 발밑 지면과 간격을 유지하고 Mover는 좌우 속도를 제어합니다. 공중에서는 지지가 사라지고 착지하면 다시 연결됩니다. 프리셋으로 지지대의 탄성과 힘 한도를 비교하세요.", { 3.5f, 1.0f }, 65.0f },
    demoEntry{ demoKind::filterJointPlayground, "조인트", "필터 조인트", "On/Off와 시작 상태 복귀로 A-B 충돌을 비교하세요. C 충돌 시험은 다른 물체와의 충돌이 유지됨을 보여줍니다. 수면 시험 준비 후 A를 깨워 Filter가 깨어남을 연결하지만 힘은 전달하지 않음을 관찰하세요.", { 0.0f, 1.0f }, 110.0f },
    demoEntry{ demoKind::motorCar, "연결 장치", "모터 자동차", "A/D로 주행, Space로 제동하고 키를 놓으면 관성으로 구릅니다. 카메라 추적을 선택하고 서스펜션·저마찰 프리셋을 비교하세요. 차량 복귀는 설정을 보존합니다. 왼쪽 드래그로 물체를 잡고, 추적을 끄면 가운데 드래그로 카메라를 이동합니다.", { 4.0f, 1.0f }, 50.0f },
    demoEntry{ demoKind::ragdoll, "연결 장치", "관절 제한 렉돌", "Revolute 관절의 각도 제한과 회전 마찰로 구성한 수동 렉돌입니다. 자유·마찰 관절과 자기 충돌을 비교하고 자세 복귀로 같은 낙하를 반복하세요. A/D·Space·S·마우스 드래그로 몸통에 힘을 주며 각 관절의 연결과 허용 각도를 관찰합니다.", { 0.0f, 3.5f }, 72.0f },
    demoEntry{ demoKind::splinePath, "경로", "스플라인 경로 이동", "편집 가능한 곡선을 따라 키네마틱 물체가 1m/s로 이동하고 접선 방향으로 회전합니다. 제어점 드래그와 곡선 10px 이내 Ctrl+클릭 삽입을 지원합니다.", { 0.0f, 1.0f }, 75.0f },
    demoEntry{ demoKind::splineRail, "연결 장치", "스플라인 레일", "다이나믹 물체를 편집 가능한 곡선 레일에 연결합니다. 스프링·리미트·모터와 회전 고정·접선 추종을 비교합니다. 양 끝은 물리적 정지점입니다.", { 0.0f, 1.0f }, 75.0f },
    demoEntry{ demoKind::collisionComparison, "강체", "충돌 비교", "원·상자·캡슐 쌍이 서로 마주 옵니다. 인스펙터에서 밀도·반발·감쇠를 바꾸고 시험 복귀로 같은 출발 조건을 비교하세요. 복귀는 편집값을 유지합니다.", { 0.0f, 0.0f }, 70.0f },
    demoEntry{ demoKind::frictionComparison, "강체", "마찰 비교", "같은 경사면의 상자 세 개가 서로 다른 마찰로 출발합니다. 물체와 경사면의 마찰을 편집해 미끄러짐을 비교하세요. 시험 복귀는 편집값을 유지합니다.", { 0.0f, 0.0f }, 65.0f },
    demoEntry{ demoKind::restitutionComparison, "강체", "반발 비교", "같은 높이의 원 세 개가 서로 다른 반발계수로 낙하합니다. 바닥과 원의 반발을 편집하고 튀어 오르는 높이를 비교하세요. 시험 복귀는 편집값을 유지합니다.", { 0.0f, 2.0f }, 85.0f },
    demoEntry{ demoKind::boxStack, "강체", "상자 쌓기", "상자 여덟 개의 쌓임과 수면을 관찰합니다. 인스펙터에서 마찰·밀도·감쇠·수면 설정을 편집하고 드래그로 건드려 보세요. 시험 복귀는 편집값을 유지합니다.", { 0.0f, 4.0f }, 65.0f },
    demoEntry{ demoKind::ccdComparison, "강체", "고속 충돌 CCD 비교", "정지 상태에서 시험 복귀 후 1프레임을 진행하세요. 월드 설정의 CCD 검사를 켜거나 끄고 고속 원의 얇은 벽 통과 여부를 비교합니다. Bullet 해제는 CCD 전체 끄기와 다릅니다. 복귀는 편집값과 CCD 설정을 유지합니다.", { 0.0f, 0.0f }, 180.0f },
    demoEntry{ demoKind::collisionFilterComparison, "강체", "충돌 필터 비교", "같은 원 쌍이 위·아래에서 마주 옵니다. 처음에는 위쪽이 충돌하고 아래쪽은 마스크로 제외되어 통과합니다. 인스펙터의 충돌 마스크를 편집하고 시험 복귀로 비교하세요. 공통 충돌 설정도 함께 적용됩니다.", { 0.0f, 0.0f }, 100.0f },
    demoEntry{ demoKind::sensorWallComparison, "강체", "센서·일반 벽 비교", "위쪽 일반 벽은 원을 막고 아래쪽 센서는 원을 통과시키며 겹침을 감지합니다. 센서와 원의 센서 이벤트·마스크를 인스펙터에서 편집하고 시험 복귀로 반복하세요. 복귀는 편집값을 유지합니다.", { 0.0f, 0.0f }, 100.0f },
    demoEntry{ demoKind::sleepWakeComparison, "강체", "수면·깨우기 비교", "같은 상자 두 개가 바닥에 내려앉습니다. 처음에는 왼쪽만 자동 수면을 허용합니다. 인스펙터의 수면 허용·속도 기준을 편집하고 왼쪽 드래그로 깨우세요. 시험 복귀는 편집값을 유지합니다.", { 0.0f, 1.0f }, 100.0f },
    demoEntry{ demoKind::dampingComparison, "강체", "선형·회전 감쇠 비교", "중력 없는 공간에서 같은 상자 세 개가 같은 속도로 이동·회전합니다. 초기 감쇠는 0·0.5·2입니다. 인스펙터에서 선형·회전 감쇠를 각각 편집하고 시험 복귀로 비교하세요. 처음에는 자동 수면을 끕니다.", { 0.0f, 2.0f }, 75.0f },
    demoEntry{ demoKind::gravityScaleComparison, "강체", "중력 배율 비교", "같은 높이의 상자 세 개가 정지 상태에서 출발합니다. 초기 중력 배율은 0·0.5·1입니다. 인스펙터에서 개별 배율과 월드 중력을 편집하고 시험 복귀로 비교하세요. 처음에는 감쇠와 자동 수면을 끕니다.", { 0.0f, 1.0f }, 65.0f },
    demoEntry{ demoKind::massImpulseComparison, "강체", "질량·충격량 비교", "같은 크기의 상자 세 개가 초기 밀도 1·2·4로 시작합니다. 같은 충격량 적용 버튼은 중심에 위쪽 2N·s를 추가합니다. 인스펙터에서 밀도를 편집하고 시험 복귀 후 다시 적용하세요. 처음에는 중력·감쇠·자동 수면을 끕니다.", { 0.0f, 1.0f }, 75.0f },
    demoEntry{ demoKind::impulsePointComparison, "강체", "충격량 작용점 비교", "같은 상자에 질량중심·중심의 왼쪽·오른쪽 0.5m 지점을 구분해 위쪽 2N·s를 추가합니다. 표시한 작용점은 물체와 함께 회전합니다. 인스펙터에서 회전·밀도를 편집하고 시험 복귀로 비교하세요.", { 0.0f, 1.0f }, 75.0f }
};

}

std::span<const demoEntry> getDemoEntries() noexcept
{
    return entries;
}

const demoEntry& getDemoEntry( demoKind kind )
{
    const auto entry = std::find_if( entries.begin(), entries.end(),
        [kind]( const demoEntry& value )
        {
            return value.kind == kind;
        } );
    assert( entry != entries.end() );

    return *entry;
}

#pragma region Lifetime

demoSession::demoSession( demoFactory factory ) : factory_( factory )
{
    assert( factory_ != nullptr );

    selectDemo( demoKind::playground );
}

void demoSession::selectDemo( demoKind kind )
{
    // 새 데모 생성 후 교체함. UI 선택/contact/World handle은 이전 데모와 함께 파괴됨.
    auto replacement = factory_( kind );
    assert( replacement != nullptr );

    replacement->setCollisionMatrix( collisionMatrix_ );
    if( demo_ )
    {
        demo_->cancelInput();
    }
    demo_ = std::move( replacement );
    kind_ = kind;
    playing_ = false;
    accumulator_ = 0.0f;
    stepCount_ = 0;
}

void demoSession::reset()
{
    selectDemo( kind_ );
}

void demoSession::setCollisionMatrix( const collisionMatrix& matrix )
{
    if( matrix == collisionMatrix_ ) return;

    demo_->setCollisionMatrix( matrix );
    collisionMatrix_ = matrix;
}

#pragma endregion Lifetime

#pragma region PlaybackAndInput

void demoSession::setPlaying( bool playing )
{
    if( playing_ == playing ) return;

    playing_ = playing;
    accumulator_ = 0.0f;
    demo_->cancelInput();
}

void demoSession::advance( float frameTime, int subStepCount )
{
    assert( std::isfinite( frameTime ) && frameTime >= 0.0f && subStepCount > 0 );

    if( !playing_ ) return;

    accumulator_ += std::min( frameTime, 0.25f );
    int count = 0;
    while( accumulator_ >= FIXED_TIME_STEP && count < MAX_STEPS_PER_FRAME )
    {
        demo_->step( FIXED_TIME_STEP, subStepCount );
        accumulator_ -= FIXED_TIME_STEP;
        ++stepCount_;
        ++count;
    }
    // 긴 frame 뒤 무제한 catch-up을 하지 않는 기존 Sandbox 정책을 유지함.
    if( count == MAX_STEPS_PER_FRAME )
    {
        accumulator_ = 0.0f;
    }
}

void demoSession::stepOnce( int subStepCount )
{
    assert( subStepCount > 0 );

    demo_->step( FIXED_TIME_STEP, subStepCount );
    ++stepCount_;
    accumulator_ = 0.0f;
}

void demoSession::handleInput( const demoInput& input, bool enabled )
{
    if( enabled )
    {
        demo_->handleInput( input );
    }
    else
    {
        demo_->cancelInput();
    }
}

#pragma endregion PlaybackAndInput

} // namespace zonai::sandbox
