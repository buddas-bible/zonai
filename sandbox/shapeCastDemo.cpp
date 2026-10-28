#include "shapeCastDemo.h"

#include <algorithm>
#include <cmath>

namespace zonai::sandbox
{

shapeCastDemo::shapeCastDemo()
{
    world_.SetGravity( {} );

    const bodyId circleBody = world_.CreateBody( bodyType::Static, { { -1.0f, 0.0f }, {} } );
    targets_[0] = { circleBody, world_.CreateShape( circleBody, circle2{ {}, 0.8f } ), "원" };

    const bodyId boxBody = world_.CreateBody( bodyType::Static, { { 2.0f, 0.0f }, {} } );
    targets_[1] = { boxBody, world_.CreateShape( boxBody, MakeBox( { 0.65f, 0.85f } ) ), "상자" };

    const bodyId capsuleBody = world_.CreateBody( bodyType::Static, { { 4.0f, -1.5f }, {} } );
    targets_[2] = { capsuleBody, world_.CreateShape( capsuleBody, capsule2{ { -0.8f, 0.0f }, { 0.8f, 0.0f }, 0.35f } ), "캡슐" };

    const bodyId sensorBody = world_.CreateBody( bodyType::Kinematic, { { 0.0f, 2.0f }, {} } );
    targets_[3] = { sensorBody, world_.CreateSensorShape( sensorBody, circle2{ {}, 0.65f }, collisionFilter{ 2, ~std::uint64_t{ 0 }, 0 } ), "센서" };

    input_.transform.position = { -5.0f, 0.0f };
    input_.translation = { 11.0f, 0.0f };
    setProbe( shapeCastProbe::circle );
}

void shapeCastDemo::step( float timeStep, int subStepCount )
{
    ( void )subStepCount;
    if( !( timeStep > 0.0f ) || !std::isfinite( timeStep ) ) return;

    const float limit = hit_.hit ? hit_.fraction : 1.0f;
    previewFraction_ = std::min( limit, previewFraction_ + timeStep * 0.4f );
}

void shapeCastDemo::handleInput( const demoInput& input )
{
    if( !input.mousePressed || !IsFinite( input.mousePosition ) ) return;

    // 기존 Canvas의 Shift 상태를 사용해 시작점 / 도착점 선택을 분리함.
    if( input.selectTrialPosition ) setStart( input.mousePosition );
    else setTarget( input.mousePosition );
}

void shapeCastDemo::setProbe( shapeCastProbe probe )
{
    probe_ = probe;

    switch( probe_ )
    {
    case shapeCastProbe::circle:
        input_.geometry = circle2{ {}, 0.45f };
        break;
    case shapeCastProbe::capsule:
        input_.geometry = capsule2{ { 0.0f, -0.45f }, { 0.0f, 0.45f }, 0.3f };
        break;
    case shapeCastProbe::box:
        input_.geometry = MakeBox( { 0.5f, 0.4f } );
        break;
    }

    updateHit();
}

void shapeCastDemo::setStart( vec2 start )
{
    if( !IsFinite( start ) ) return;

    const vec2 end = input_.transform.position + input_.translation;
    input_.transform.position = start;
    input_.translation = end - start;
    updateHit();
}

void shapeCastDemo::setTarget( vec2 target )
{
    if( !IsFinite( target ) ) return;

    input_.translation = target - input_.transform.position;
    updateHit();
}

void shapeCastDemo::setSensorIncluded( bool included )
{
    sensorIncluded_ = included;
    updateHit();
}

void shapeCastDemo::setCanEncroach( bool enabled )
{
    input_.canEncroach = enabled;
    updateHit();
}

void shapeCastDemo::resetPreview()
{
    previewFraction_ = 0.0f;
}

aabb2 shapeCastDemo::getSweepAABB() const noexcept
{
    const aabb2 start = ComputeShapeAABB( input_.geometry, input_.transform );
    return Union( start, { start.min + input_.translation, start.max + input_.translation } );
}

queryFilter shapeCastDemo::makeFilter() const noexcept
{
    queryFilter filter{};
    if( !sensorIncluded_ ) filter.maskBits &= ~std::uint64_t{ 2 };
    return filter;
}

void shapeCastDemo::updateHit()
{
    const queryFilter filter = makeFilter();

    // 동일한 경로와 Query Filter로 세 API를 비교함. AABB는 geometry 확정이 아닌 후보 조회임.
    hit_ = world_.castShapeClosest( input_, filter );
    rayHit_ = world_.castRayClosest( input_.transform.position, input_.translation, filter );

    aabbCandidateCount_ = 0;
    const aabb2 swept = getSweepAABB();

    if( IsValidAABB( swept ) )
    {
        world_.overlapAABB( swept, filter,
            [&]( shapeId )
            {
                ++aabbCandidateCount_;
                return true;
            } );
    }

    resetPreview();
}

} // namespace zonai::sandbox
