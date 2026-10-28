#include "shapeCastDemo.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "collision/sweep2.h"

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

void shapeCastDemo::setStartRotationRadians( float radians )
{
    constexpr float PI = 3.14159265358979323846f;
    if( !std::isfinite( radians ) || std::fabs( radians ) > PI ||
        radians == startRotationRadians_ ) return;

    startRotationRadians_ = radians;
    input_.transform.rotation = rot2::FromRadians( radians );
    updateHit();
}

void shapeCastDemo::setRotationRadians( float radians )
{
    constexpr float HALF_PI = 1.5707963267948966f;
    if( !std::isfinite( radians ) || std::fabs( radians ) > HALF_PI ||
        radians == rotationRadians_ ) return;

    rotationRadians_ = radians;
    updateHit();
}

void shapeCastDemo::setDenseTargets( bool enabled )
{
    if( enabled == hasDenseTargets() ) return;

    if( enabled )
    {
        denseTargets_.reserve( 40 );
        for( int y = 0; y < 5; ++y )
        {
            for( int x = 0; x < 8; ++x )
            {
                const int index = y * 8 + x;
                const bodyType type = static_cast<bodyType>( index % static_cast<int>( bodyType::Count ) );
                const transform2 pose{ { -7.0f + 2.0f * x, -4.0f + 2.0f * y },
                    rot2::FromRadians( 0.17f * ( index % 5 ) ) };
                const bodyId body = world_.CreateBody( type, pose );

                shapeGeometry geometry{};
                if( index % 3 == 0 ) geometry = circle2{ {}, 0.3f };
                else if( index % 3 == 1 ) geometry = MakeBox( { 0.36f, 0.28f } );
                else geometry = capsule2{ { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.18f };

                const shapeId id = index % 11 == 0
                    ? world_.CreateSensorShape( body, geometry,
                        collisionFilter{ 2, ~std::uint64_t{ 0 }, 0 } )
                    : world_.CreateShape( body, geometry );
                denseTargets_.push_back( { body, id, "" } );
            }
        }
    }
    else
    {
        for( const shapeCastTarget& target : denseTargets_ )
            world_.DestroyBody( target.body );
        denseTargets_.clear();
    }

    updateHit();
}

void shapeCastDemo::resetPreview()
{
    previewFraction_ = 0.0f;
}

transform2 shapeCastDemo::getPoseAtFraction( float fraction ) const noexcept
{
    const float time = std::clamp( fraction, 0.0f, 1.0f );
    const vec2 start = input_.transform.position;
    if( rotationRadians_ == 0.0f )
        return { start + input_.translation * time, input_.transform.rotation };

    const sweep2 sweep{ {}, start, start + input_.translation, input_.transform.rotation,
        input_.transform.rotation * rot2::FromRadians( rotationRadians_ ) };
    return GetSweepTransform( sweep, time );
}

aabb2 shapeCastDemo::getSweepAABB() const noexcept
{
    const aabb2 start = ComputeShapeAABB( input_.geometry, input_.transform );
    if( rotationRadians_ == 0.0f )
        return Union( start, { start.min + input_.translation, start.max + input_.translation } );

    // World의 회전 후보 조회와 같이 이동 중심을 최대 로컬 원점 반경으로 확장함.
    const shapeProxy2 proxy = MakeShapeProxy( input_.geometry );
    double maxRadius = proxy.radius;
    for( int i = 0; i < proxy.count; ++i )
    {
        const vec2 point = proxy.points[i];
        maxRadius = std::max( maxRadius,
            std::hypot( static_cast<double>( point.x ), static_cast<double>( point.y ) ) + proxy.radius );
    }

    if( !std::isfinite( maxRadius ) ||
        maxRadius >= static_cast<double>( std::numeric_limits<float>::max() ) ) return {};

    const float radius = std::nextafter( static_cast<float>( maxRadius ),
        std::numeric_limits<float>::infinity() );
    const float negativeInfinity = -std::numeric_limits<float>::infinity();
    const float positiveInfinity = std::numeric_limits<float>::infinity();
    const vec2 end = input_.transform.position + input_.translation;

    return {
        { std::nextafter( std::min( input_.transform.position.x, end.x ) - radius, negativeInfinity ),
          std::nextafter( std::min( input_.transform.position.y, end.y ) - radius, negativeInfinity ) },
        { std::nextafter( std::max( input_.transform.position.x, end.x ) + radius, positiveInfinity ),
          std::nextafter( std::max( input_.transform.position.y, end.y ) + radius, positiveInfinity ) }
    };
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
    sweepStats_ = {};

    // 동일한 경로와 Query Filter로 세 API를 비교함. AABB는 geometry 확정이 아닌 후보 조회임.
    if( rotationRadians_ == 0.0f )
        hit_ = world_.castShapeClosest( input_, filter );
    else
        hit_ = world_.castShapeSweepClosest(
            { input_.geometry, input_.transform, input_.translation, rotationRadians_ },
            filter, &sweepStats_ );

    // 중심 RayCast는 회전을 반영하지 않는 비교용 직선 조회임.
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
