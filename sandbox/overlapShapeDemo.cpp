#include "overlapShapeDemo.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace zonai::sandbox
{

overlapShapeDemo::overlapShapeDemo()
{
    world_.SetGravity( {} );

    const bodyId boxBody = world_.CreateBody( bodyType::Static );
    targets_[0] = { boxBody, world_.CreateShape( boxBody, MakeBox( { 1.0f, 1.0f } ) ), "상자" };

    const bodyId circleBody = world_.CreateBody( bodyType::Static, { { -3.0f, 0.0f }, {} } );
    targets_[1] = { circleBody, world_.CreateShape( circleBody, circle2{ {}, 0.6f } ), "원" };

    const bodyId movingBody = world_.CreateBody( bodyType::Dynamic, { { 3.0f, 0.0f }, {} } );
    targets_[2] = { movingBody, world_.CreateShape( movingBody, circle2{ {}, 0.7f } ), "동적 원" };

    const bodyId sensorBody = world_.CreateBody( bodyType::Kinematic, { { 0.0f, 2.8f }, {} } );
    targets_[3] = { sensorBody, world_.CreateSensorShape( sensorBody, circle2{ {}, 0.65f },
        collisionFilter{ 2, ~std::uint64_t{ 0 }, 0 } ), "센서" };

    queryTransform_.position = { 1.45f, 1.45f };
    setProbe( overlapProbe::circle );
}

void overlapShapeDemo::handleInput( const demoInput& input )
{
    if( input.mousePressed && IsFinite( input.mousePosition ) )
        setQueryPosition( input.mousePosition );
}

void overlapShapeDemo::setQueryMode( overlapQueryMode mode )
{
    if( mode_ == mode ) return;
    mode_ = mode;
    updateQueries();
}

void overlapShapeDemo::setProbe( overlapProbe probe )
{
    probe_ = probe;

    switch( probe_ )
    {
    case overlapProbe::circle:
        queryGeometry_ = circle2{ {}, 0.5f };
        break;

    case overlapProbe::capsule:
        queryGeometry_ = capsule2{ { 0.0f, -0.45f }, { 0.0f, 0.45f }, 0.25f };
        break;

    case overlapProbe::box:
        queryGeometry_ = MakeBox( { 0.5f, 0.35f } );
        break;
    }

    updateQueries();
}

void overlapShapeDemo::setQueryPosition( vec2 position )
{
    if( !IsFinite( position ) ) return;
    queryTransform_.position = position;
    updateQueries();
}

void overlapShapeDemo::setQueryRotation( float radians )
{
    if( !std::isfinite( radians ) ) return;
    queryRotation_ = radians;
    queryTransform_.rotation = rot2::FromRadians( radians );
    updateQueries();
}

void overlapShapeDemo::setSensorIncluded( bool included )
{
    sensorIncluded_ = included;
    updateQueries();
}

void overlapShapeDemo::setLineTargetsEnabled( bool enabled )
{
    if( enabled == hasLineTargets() ) return;

    if( enabled )
    {
        // 같은 높이의 양면 Segment와 ghost가 있는 Chain edge를 별개 Body에 배치함.
        const bodyId segmentBody = world_.CreateBody( bodyType::Static, { { -3.0f, -2.5f }, {} } );
        const shapeId segment = world_.CreateShape( segmentBody, segment2{ { -0.8f, 0.0f }, { 0.8f, 0.0f } } );
        lineTargets_.push_back( { segmentBody, segment, "양면 선분" } );

        const bodyId chainBody = world_.CreateBody( bodyType::Static, { { 3.0f, -2.5f }, {} } );
        const chainSegment2 chain{ { -1.6f, 0.0f },
            { { -0.8f, 0.0f }, { 0.8f, 0.0f } }, { 1.6f, 0.0f } };
        const shapeId chainShape = world_.CreateShape( chainBody, chain );
        lineTargets_.push_back( { chainBody, chainShape, "Chain 선분" } );
    }
    else
    {
        for( const overlapTarget& target : lineTargets_ )
            world_.DestroyBody( target.body );
        lineTargets_.clear();
    }

    updateQueries();
}

aabb2 overlapShapeDemo::getQueryAABB() const
{
    // 점 조회는 반경이 없는 AABB로 Tree를 탐색하므로 도형의 크기를 사용하지 않음.
    if( mode_ == overlapQueryMode::point ) return { queryTransform_.position, queryTransform_.position };

    return ComputeShapeAABB( queryGeometry_, queryTransform_ );
}

bool overlapShapeDemo::isCandidate( shapeId id ) const
{
    return std::find( candidates_.begin(), candidates_.end(), id ) != candidates_.end();
}

bool overlapShapeDemo::isExactOverlap( shapeId id ) const
{
    return std::find( exactOverlaps_.begin(), exactOverlaps_.end(), id ) != exactOverlaps_.end();
}

void overlapShapeDemo::updateQueries()
{
    candidates_.clear();
    exactOverlaps_.clear();

    queryFilter filter{};
    if( !sensorIncluded_ ) filter.maskBits &= ~std::uint64_t{ 2 };

    world_.overlapAABB( getQueryAABB(), filter,
        [&]( shapeId id )
        {
            candidates_.push_back( id );
            return true;
        } );

    const auto collectOverlap = [&]( shapeId id )
    {
        exactOverlaps_.push_back( id );
        return true;
    };

    // 같은 점 또는 도형의 AABB 후보를 먼저 기록하고, 실제 포함/겹침만 다시 분류함.
    if( mode_ == overlapQueryMode::point )
        world_.overlapPoint( queryTransform_.position, filter, collectOverlap );
    else
        world_.overlapShape( queryGeometry_, queryTransform_, filter, collectOverlap );
}

} // namespace zonai::sandbox
