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
    demoEntry{ demoKind::playground, "Rigid Bodies", "Playground", "Canvas: A/D hold = push circle, Space = jump, S = spin box. Left drag = grab dynamic body. Right click = kick circle toward cursor.", { 0.0f, 0.5f }, 55.0f },
    demoEntry{ demoKind::distancePendulum, "Joints", "Distance Pendulum", "Canvas: A/D hold = push pendulum, Space = kick. Left drag = grab pendulum. Right click = kick toward cursor. Controls: spring/Hertz/damping, limit/min/max, Radial kick. Purple line connects anchors; green/red mark limits.", { 0.0f, -0.5f }, 110.0f }
};
}

std::span<const demoEntry> getDemoEntries() noexcept { return entries; }

const demoEntry& getDemoEntry( demoKind kind )
{
    const auto entry = std::find_if( entries.begin(), entries.end(), [kind]( const demoEntry& value ) { return value.kind == kind; } );
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
    if( demo_ ) { demo_->cancelInput(); }
    demo_ = std::move( replacement );
    kind_ = kind;
    playing_ = false;
    accumulator_ = 0.0f;
    stepCount_ = 0;
}

void demoSession::reset() { selectDemo( kind_ ); }
#pragma endregion

#pragma region PlaybackAndInput
void demoSession::setPlaying( bool playing )
{
    if( playing_ == playing ) { return; }
    playing_ = playing;
    accumulator_ = 0.0f;
    demo_->cancelInput();
}

void demoSession::advance( float frameTime, int subStepCount )
{
    assert( std::isfinite( frameTime ) && frameTime >= 0.0f && subStepCount > 0 );
    if( !playing_ ) { return; }
    accumulator_ += std::min( frameTime, 0.25f );
    int count = 0;
    while( accumulator_ >= FIXED_TIME_STEP && count < MAX_STEPS_PER_FRAME )
    {
        demo_->step( FIXED_TIME_STEP, subStepCount );
        accumulator_ -= FIXED_TIME_STEP;
        ++stepCount_; ++count;
    }
    // 긴 frame 뒤 무제한 catch-up을 하지 않는 기존 Sandbox 정책을 유지함.
    if( count == MAX_STEPS_PER_FRAME ) { accumulator_ = 0.0f; }
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
    if( enabled ) { demo_->handleInput( input ); }
    else { demo_->cancelInput(); }
}
#pragma endregion
} // namespace zonai::sandbox
