#include "collision/rayCast2.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace zonai
{

namespace
{

rayCastHit2 CastCircle( const circle2& circle, vec2 origin, vec2 translation, float maxFraction )
{
    const vec2 offset = origin - circle.center;
    const float distanceSquared = LengthSquared( offset );
    const float radiusSquared = circle.radius * circle.radius;

    // Box2D의 solid ray 의미: 내부 또는 표면에서 출발하면 출구를 보고하지 않음.
    if( distanceSquared <= radiusSquared ) return {};

    const float a = LengthSquared( translation );
    const float b = Dot( offset, translation );
    const float discriminant = b * b - a * ( distanceSquared - radiusSquared );

    if( a <= 0.0f || discriminant < 0.0f ) return {};

    const float fraction = ( -b - std::sqrt( discriminant ) ) / a;

    if( fraction < 0.0f || fraction > maxFraction ) return {};

    const vec2 point = origin + translation * fraction;
    const vec2 normal = Normalize( point - circle.center );

    return { point, normal, fraction, true };
}

rayCastHit2 CastSegment( const segment2& segment, vec2 origin, vec2 translation, float maxFraction, bool oneSided )
{
    const vec2 edge = segment.b - segment.a;
    const float determinant = Cross( translation, edge );
    if( determinant == 0.0f ) return {};

    const vec2 difference = segment.a - origin;
    const float fraction = Cross( difference, edge ) / determinant;
    const float edgeFraction = Cross( difference, translation ) / determinant;

    if( fraction < 0.0f || fraction > maxFraction || edgeFraction < 0.0f || edgeFraction > 1.0f ) return {};

    // Chain의 앞면은 edge 방향의 오른쪽. 일반 Segment는 양쪽 면을 허용함.
    vec2 normal = Normalize( vec2{ edge.y, -edge.x } );
    if( LengthSquared( normal ) == 0.0f ) return {};

    if( Dot( normal, translation ) >= 0.0f )
    {
        if( oneSided ) return {};

        normal = -normal;
    }

    return { origin + translation * fraction, normal, fraction, true };
}

void KeepNearest( rayCastHit2& closest, const rayCastHit2& candidate )
{
    if( candidate.hit && ( !closest.hit || candidate.fraction < closest.fraction ) ) closest = candidate;
}

rayCastHit2 CastCapsule( const capsule2& capsule, vec2 origin, vec2 translation, float maxFraction )
{
    const vec2 axis = capsule.center2 - capsule.center1;
    const float axisLength = Length( axis );

    if( axisLength == 0.0f ) return CastCircle( { capsule.center1, capsule.radius }, origin, translation, maxFraction );
    if( capsule.radius == 0.0f ) return CastSegment( { capsule.center1, capsule.center2 }, origin, translation, maxFraction, false );

    if( Contains( capsule, origin ) ) return {};

    const vec2 tangent = axis / axisLength;
    const vec2 side{ tangent.y, -tangent.x };
    const float startDistance = Dot( origin - capsule.center1, side );
    const float normalSpeed = Dot( translation, side );

    rayCastHit2 closest{};

    // 직사각형의 양 옆면. 양 끝 원호는 아래 별도 검사함.
    if( normalSpeed != 0.0f )
    {
        for( const float sign : { -1.0f, 1.0f } )
        {
            const vec2 normal = side * sign;
            if( Dot( normal, translation ) >= 0.0f ) continue;

            const float fraction = ( sign * capsule.radius - startDistance ) / normalSpeed;
            if( fraction < 0.0f || fraction > maxFraction ) continue;

            const vec2 point = origin + translation * fraction;
            const float along = Dot( point - capsule.center1, tangent );
            if( along < 0.0f || along > axisLength ) continue;

            KeepNearest( closest, { point, normal, fraction, true } );
        }
    }

    const rayCastHit2 first = CastCircle( { capsule.center1, capsule.radius }, origin, translation, maxFraction );
    if( first.hit && Dot( first.point - capsule.center1, tangent ) <= 0.0f ) KeepNearest( closest, first );

    const rayCastHit2 second = CastCircle( { capsule.center2, capsule.radius }, origin, translation, maxFraction );
    if( second.hit && Dot( second.point - capsule.center2, tangent ) >= 0.0f ) KeepNearest( closest, second );

    return closest;
}

bool StartsInsidePolygon( const polygon2& polygon, vec2 origin )
{
    bool insideCore = true;
    float distanceSquared = std::numeric_limits<float>::max();

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        const int next = ( i + 1 ) % polygon.vertexCount;
        const float signedDistance = Dot( polygon.normals[i], origin - polygon.vertices[i] );
        if( signedDistance > 0.0f ) insideCore = false;

        if( polygon.radius > 0.0f )
            distanceSquared = std::min( distanceSquared, DistanceSquared( segment2{ polygon.vertices[i], polygon.vertices[next] }, origin ) );
    }

    return insideCore || ( polygon.radius > 0.0f && distanceSquared <= polygon.radius * polygon.radius );
}

rayCastHit2 CastPolygon( const polygon2& polygon, vec2 origin, vec2 translation, float maxFraction )
{
    if( polygon.vertexCount == 1 ) return CastCircle( { polygon.vertices[0], polygon.radius }, origin, translation, maxFraction );
    if( polygon.vertexCount == 2 ) return CastCapsule( { polygon.vertices[0], polygon.vertices[1], polygon.radius }, origin, translation, maxFraction );
    if( polygon.vertexCount < 3 || StartsInsidePolygon( polygon, origin ) ) return {};

    if( polygon.radius == 0.0f )
    {
        // 볼록 다각형의 모든 반평면에서 진입/이탈 fraction 구간을 자름.
        float enter = 0.0f;
        float exit = maxFraction;
        vec2 normal{};
        bool hasEntrance = false;

        for( int i = 0; i < polygon.vertexCount; ++i )
        {
            const vec2 edgeNormal = polygon.normals[i];
            const float distance = Dot( edgeNormal, origin - polygon.vertices[i] );
            const float speed = Dot( edgeNormal, translation );

            if( speed == 0.0f )
            {
                if( distance > 0.0f ) return {};
                continue;
            }

            const float fraction = -distance / speed;
            if( speed < 0.0f )
            {
                if( fraction >= enter )
                {
                    enter = fraction;
                    normal = edgeNormal;
                    hasEntrance = true;
                }
            }
            else
            {
                exit = std::min( exit, fraction );
            }

            if( enter > exit ) return {};
        }

        if( !hasEntrance || enter < 0.0f || enter > maxFraction ) return {};
        return { origin + translation * enter, normal, enter, true };
    }

    // Rounded polygon: 각 면의 외측 평행선과 꼭짓점 원호를 별도로 검사함.
    rayCastHit2 closest{};

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        const int next = ( i + 1 ) % polygon.vertexCount;
        const vec2 vertex = polygon.vertices[i];
        const vec2 edge = polygon.vertices[next] - vertex;
        const vec2 normal = polygon.normals[i];
        const float speed = Dot( normal, translation );

        if( speed < 0.0f )
        {
            const float fraction = ( polygon.radius - Dot( normal, origin - vertex ) ) / speed;
            if( fraction >= 0.0f && fraction <= maxFraction )
            {
                const vec2 point = origin + translation * fraction;
                const float projection = Dot( point - vertex, edge );
                if( projection >= 0.0f && projection <= LengthSquared( edge ) )
                    KeepNearest( closest, { point, normal, fraction, true } );
            }
        }

        const int previous = ( i + polygon.vertexCount - 1 ) % polygon.vertexCount;
        const vec2 previousEdge = vertex - polygon.vertices[previous];
        const rayCastHit2 corner = CastCircle( { vertex, polygon.radius }, origin, translation, maxFraction );

        if( corner.hit && Dot( corner.point - vertex, previousEdge ) >= 0.0f &&
            Dot( corner.point - vertex, edge ) <= 0.0f )
            KeepNearest( closest, corner );
    }

    return closest;
}

} // namespace

rayCastHit2 RayCastShapeGeometry( const shapeGeometry& geometry, const transform2& transform,
    vec2 origin, vec2 translation, float maxFraction )
{
    if( !IsFinite( origin ) || !IsFinite( translation ) || !std::isfinite( maxFraction ) ||
        maxFraction <= 0.0f || maxFraction > 1.0f || LengthSquared( translation ) <= 0.0f ) return {};

    // Geometry는 Body local space에 저장됨. 방향 벡터에는 translation을 적용하지 않음.
    const vec2 localOrigin = InverseTransformPoint( transform, origin );
    const vec2 localTranslation = InverseTransformVector( transform, translation );

    rayCastHit2 result = std::visit(
        [&]( const auto& local ) -> rayCastHit2
        {
            using Geometry = std::remove_cvref_t<decltype( local )>;

            if constexpr( std::is_same_v<Geometry, std::monostate> ) return {};
            else if constexpr( std::is_same_v<Geometry, circle2> ) return CastCircle( local, localOrigin, localTranslation, maxFraction );
            else if constexpr( std::is_same_v<Geometry, capsule2> ) return CastCapsule( local, localOrigin, localTranslation, maxFraction );
            else if constexpr( std::is_same_v<Geometry, polygon2> ) return CastPolygon( local, localOrigin, localTranslation, maxFraction );
            else if constexpr( std::is_same_v<Geometry, segment2> ) return CastSegment( local, localOrigin, localTranslation, maxFraction, false );
            else
            {
                static_assert( std::is_same_v<Geometry, chainSegment2> );
                return CastSegment( local.segment, localOrigin, localTranslation, maxFraction, true );
            }
        }, geometry );

    if( !result.hit ) return {};

    result.point = TransformPoint( transform, result.point );
    result.normal = TransformVector( transform, result.normal );

    if( !IsFinite( result.point ) || !IsFinite( result.normal ) ) return {};
    return result;
}

} // namespace zonai
