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
    demoEntry{ demoKind::distancePendulum, "조인트", "거리 조인트 진자", "캔버스: A/D 유지로 진자 밀기, 스페이스로 충격량 가하기. 왼쪽 드래그로 진자 잡기, 오른쪽 클릭으로 커서 방향으로 밀기. 스프링·주파수·감쇠·거리 제한·축 방향 충격량을 조절합니다. 보라색 선은 고정점을 잇고 초록·빨강은 거리 제한입니다.", { 0.0f, -0.5f }, 110.0f },
    demoEntry{ demoKind::revoluteHinge, "조인트", "회전축과 막대", "캔버스: A/D 유지로 막대 밀기, 스페이스로 위로 밀기, S로 회전 충격량 가하기. 왼쪽 드래그로 막대 잡기. 오른쪽 클릭으로 커서 방향으로 밀기. 연결점은 유지되고 상대 회전은 자유롭습니다.", { 0.0f, -0.8f }, 130.0f },
    demoEntry{ demoKind::wheelSuspension, "조인트", "서스펜션과 바퀴", "캔버스: A/D 유지로 축 옆으로 밀기, 스페이스로 바퀴 위로 밀기, S로 회전 충격량 가하기. 왼쪽 드래그로 바퀴 잡기. 오른쪽 클릭으로 커서 방향으로 밀기. 축 방향 스프링·감쇠와 자유 회전을 비교합니다.", { 0.0f, -0.2f }, 150.0f },
    demoEntry{ demoKind::prismaticRail, "조인트", "프리즈매틱 레일", "캔버스: A/D 유지로 슬라이더를 레일 축 방향으로 밀기, 스페이스로 수직 충격량, S로 회전 충격량 가하기. 빠른 설정에서 자유·제한·위치 고정과 모터 정방향·역방향·제동·모터+제한을 비교합니다. 모터는 위치가 아니라 축 방향 목표 속도를 제어합니다.", { 0.0f, 0.0f }, 110.0f },
    demoEntry{ demoKind::mouseJointPlayground, "조인트", "마우스 조인트", "캔버스: 질량이 다른 상자를 왼쪽 드래그로 잡아 움직입니다. 주파수·감쇠·최대 힘을 바꾸며 같은 설정에서 질량에 따른 추종 차이를 비교합니다.", { 0.0f, 0.0f }, 90.0f },
    demoEntry{ demoKind::motorCar, "연결 장치", "모터 자동차", "캔버스: A/D 유지로 왼쪽/오른쪽 주행, 스페이스 유지로 제동. 키를 놓으면 자유 주행합니다. 왼쪽 드래그로 차체나 바퀴 잡기, 오른쪽 클릭으로 차체 밀기. 모터·마찰·서스펜션을 비교하고 가운데 드래그로 주행 경로를 따라갑니다.", { 4.0f, 1.0f }, 50.0f }
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
