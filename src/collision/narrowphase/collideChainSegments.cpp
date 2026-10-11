#include "collision/narrowphase/collide.h"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>

#include "collision/constants.h"
#include "collision/distance2.h"

namespace zonai
{
namespace
{

enum class chainNormalType
{
    skip,
    admit,
    snap
};

struct chainSegmentParams
{
    vec2 edge{};
    vec2 normal0{};
    vec2 normal2{};
    bool convex1 = false;
    bool convex2 = false;
};

chainNormalType ClassifyChainNormal( const chainSegmentParams& params, const vec2& normal )
{
    constexpr float sinTolerance = 0.01f;

    if( Dot( normal, params.edge ) <= 0.0f )
    {
        // tail 쪽을 향하는 normal은 이전 edge와의 convex/concave 관계로 소유권을 정함.
        if( params.convex1 )
        {
            if( Cross( normal, params.normal0 ) > sinTolerance ) return chainNormalType::skip;

            return chainNormalType::admit;
        }

        return chainNormalType::snap;
    }

    // head 쪽도 같은 방식으로 다음 edge와 비교함.
    if( params.convex2 )
    {
        if( Cross( params.normal2, normal ) > sinTolerance ) return chainNormalType::skip;

        return chainNormalType::admit;
    }

    return chainNormalType::snap;
}

localManifold2 ClipChainSegments( const vec2& a1, const vec2& a2, const vec2& b1, const vec2& b2, const vec2& normal, float radiusA, float radiusB, std::uint16_t id1, std::uint16_t id2 )
{
    localManifold2 manifold{};

    const vec2 tangent = Cross( 1.0f, normal );

    // reference segment A를 tangent 좌표의 [0, upperA] 구간으로 둠.
    const float lowerA = 0.0f;
    const float upperA = Dot( a2 - a1, tangent );

    // CCW polygon의 incident edge는 tangent와 반대 방향으로 진행함.
    const float upperB = Dot( b1 - a1, tangent );
    const float lowerB = Dot( b2 - a1, tangent );

    if( upperB < lowerA || upperA < lowerB ) return manifold;

    vec2 lowerPoint = b2;

    if( lowerB < lowerA && upperB - lowerB > FLT_EPSILON )
    {
        const float fraction = ( lowerA - lowerB ) / ( upperB - lowerB );
        lowerPoint = b2 + ( b1 - b2 ) * fraction;
    }

    vec2 upperPoint = b1;

    if( upperB > upperA && upperB - lowerB > FLT_EPSILON )
    {
        const float fraction = ( upperA - lowerB ) / ( upperB - lowerB );
        upperPoint = b2 + ( b1 - b2 ) * fraction;
    }

    const float lowerSeparation = Dot( lowerPoint - a1, normal );
    const float upperSeparation = Dot( upperPoint - a1, normal );

    lowerPoint += 0.5f * ( radiusA - radiusB - lowerSeparation ) * normal;
    upperPoint += 0.5f * ( radiusA - radiusB - upperSeparation ) * normal;

    const float radius = radiusA + radiusB;

    manifold.normal = normal;
    manifold.points[0].point = lowerPoint;
    manifold.points[0].separation = lowerSeparation - radius;
    manifold.points[0].id = id1;
    manifold.points[1].point = upperPoint;
    manifold.points[1].separation = upperSeparation - radius;
    manifold.points[1].id = id2;
    manifold.pointCount = 2;

    return manifold;
}

} // namespace

localManifold2 CollideChainSegmentCapsule( const chainSegment2& chainSegment, const capsule2& capsule, const transform2& capsuleTransform, simplexCache2& cache )
{
    const polygon2 capsulePolygon = MakeCapsule( capsule.center1, capsule.center2, capsule.radius );

    return CollideChainSegmentPolygon( chainSegment, capsulePolygon, capsuleTransform, cache );
}

localManifold2 CollideChainSegmentPolygon( const chainSegment2& chainSegment, const polygon2& polygon, const transform2& polygonTransform, simplexCache2& cache )
{
    localManifold2 manifold{};

    if( polygon.vertexCount == 0 ) return manifold;

    const vec2 centroidB = TransformPoint( polygonTransform, polygon.centroid );
    const float radiusB = polygon.radius;

    const vec2 point1 = chainSegment.segment.a;
    const vec2 point2 = chainSegment.segment.b;
    const vec2 edge1 = Normalize( point2 - point1 );

    chainSegmentParams params{};
    params.edge = edge1;

    constexpr float convexTolerance = 0.01f;

    const vec2 edge0 = Normalize( point1 - chainSegment.ghost1 );
    params.normal0 = Cross( edge0, 1.0f );
    params.convex1 = Cross( edge0, edge1 ) >= convexTolerance;

    const vec2 edge2 = Normalize( chainSegment.ghost2 - point2 );
    params.normal2 = Cross( edge2, 1.0f );
    params.convex2 = Cross( edge1, edge2 ) >= convexTolerance;

    // point1 -> point2의 오른쪽이 chain의 충돌 면임.
    const vec2 normal1 = Cross( edge1, 1.0f );
    const bool behind1 = Dot( normal1, centroidB - point1 ) < 0.0f;
    bool behind0 = true;
    bool behind2 = true;

    if( params.convex1 ) behind0 = Dot( params.normal0, centroidB - point1 ) < 0.0f;

    if( params.convex2 ) behind2 = Dot( params.normal2, centroidB - point2 ) < 0.0f;

    if( behind1 && behind0 && behind2 ) return manifold;

    std::array<vec2, MAX_POLYGON_VERTICES> vertices{};
    std::array<vec2, MAX_POLYGON_VERTICES> normals{};

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        vertices[i] = TransformPoint( polygonTransform, polygon.vertices[i] );
        normals[i] = TransformVector( polygonTransform, polygon.normals[i] );
    }

    // Core shape의 최근접 feature를 GJK로 구함. cache는 다음 narrowphase 갱신에서 재사용 가능함.
    distanceInput2 input{};
    input.proxyA.points[0] = point1;
    input.proxyA.points[1] = point2;
    input.proxyA.count = 2;
    input.proxyB.count = polygon.vertexCount;

    for( int i = 0; i < polygon.vertexCount; ++i ) input.proxyB.points[i] = vertices[i];

    input.transform = {};
    input.useRadii = false;

    const distanceOutput2 output = ShapeDistance( input, cache );

    if( output.distance > radiusB + SPECULATIVE_DISTANCE ) return manifold;

    // Concave neighbor는 독립 normal을 허용하지 않고 현재 segment normal로 snap함.
    const vec2 normal0 = params.convex1 ? params.normal0 : normal1;
    const vec2 normal2 = params.convex2 ? params.normal2 : normal1;

    int incidentIndex = -1;
    int incidentNormal = -1;

    if( !behind1 && output.distance > 0.1f * LINEAR_SLOP )
    {
        if( cache.count == 1 )
        {
            const vec2 normal = Normalize( output.pointB - output.pointA );
            const chainNormalType type = ClassifyChainNormal( params, normal );

            if( type == chainNormalType::skip ) return manifold;

            if( type == chainNormalType::admit )
            {
                manifold.normal = normal;
                manifold.points[0].point = output.pointA;
                manifold.points[0].separation = output.distance - radiusB;
                manifold.points[0].id = MakeContactPointId( cache.indexA[0], cache.indexB[0] );
                manifold.pointCount = 1;

                return manifold;
            }

            incidentIndex = cache.indexB[0];
        }
        else
        {
            assert( cache.count == 2 );

            int indexA1 = cache.indexA[0];
            int indexA2 = cache.indexA[1];
            int indexB1 = cache.indexB[0];
            int indexB2 = cache.indexB[1];

            if( indexA1 == indexA2 )
            {
                assert( indexB1 != indexB2 );

                // Polygon normal 중 실제 최근접 방향과 가장 정렬된 face를 선택함.
                const vec2 normalBSearch = output.pointA - output.pointB;
                const float dot1 = Dot( normalBSearch, normals[indexB1] );
                const float dot2 = Dot( normalBSearch, normals[indexB2] );
                const int indexB = dot1 > dot2 ? indexB1 : indexB2;
                const vec2 normalB = normals[indexB];

                const chainNormalType type = ClassifyChainNormal( params, -normalB );

                if( type == chainNormalType::skip ) return manifold;

                if( type == chainNormalType::admit )
                {
                    indexB1 = indexB;
                    indexB2 = indexB < polygon.vertexCount - 1 ? indexB + 1 : 0;

                    const vec2 b1 = vertices[indexB1];
                    const vec2 b2 = vertices[indexB2];

                    const float separation1 = Dot( normalB, point1 - b1 );
                    const float separation2 = Dot( normalB, point2 - b1 );

                    if( separation1 < separation2 )
                    {
                        if( Dot( normal0, normalB ) < Dot( normal1, normalB ) ) return manifold;
                    }
                    else
                    {
                        if( Dot( normal2, normalB ) < Dot( normal1, normalB ) ) return manifold;
                    }

                    manifold = ClipChainSegments( b1, b2, point1, point2, normalB, radiusB, 0.0f, MakeContactPointId( indexB1, 1 ), MakeContactPointId( indexB2, 0 ) );

                    if( manifold.pointCount == 2 ) manifold.normal = -normalB;

                    return manifold;
                }

                incidentNormal = indexB;
            }
            else
            {
                // Segment edge와 polygon vertex가 최근접 feature인 경우.
                const float dot1 = Dot( normal1, vertices[indexB1] - point1 );
                const float dot2 = Dot( normal1, vertices[indexB2] - point2 );
                incidentIndex = dot1 < dot2 ? indexB1 : indexB2;
            }
        }
    }
    else
    {
        // Segment normal과 허용 가능한 polygon normal을 SAT로 비교함.
        float edgeSeparation = FLT_MAX;

        for( int i = 0; i < polygon.vertexCount; ++i )
        {
            const float separation = Dot( normal1, vertices[i] - point1 );

            if( separation < edgeSeparation )
            {
                edgeSeparation = separation;
                incidentIndex = i;
            }
        }

        if( params.convex1 )
        {
            float separation0 = FLT_MAX;

            for( int i = 0; i < polygon.vertexCount; ++i ) separation0 = std::min( separation0, Dot( params.normal0, vertices[i] - point1 ) );

            if( separation0 > edgeSeparation )
            {
                edgeSeparation = separation0;
                incidentIndex = -1;
            }
        }

        if( params.convex2 )
        {
            float separation2 = FLT_MAX;

            for( int i = 0; i < polygon.vertexCount; ++i ) separation2 = std::min( separation2, Dot( params.normal2, vertices[i] - point2 ) );

            if( separation2 > edgeSeparation )
            {
                edgeSeparation = separation2;
                incidentIndex = -1;
            }
        }

        float polygonSeparation = -FLT_MAX;
        int referenceIndex = -1;

        for( int i = 0; i < polygon.vertexCount; ++i )
        {
            const vec2 normal = normals[i];

            if( ClassifyChainNormal( params, -normal ) != chainNormalType::admit ) continue;

            const vec2 point = vertices[i];
            const float separation = std::min( Dot( normal, point2 - point ), Dot( normal, point1 - point ) );

            if( separation > polygonSeparation )
            {
                polygonSeparation = separation;
                referenceIndex = i;
            }
        }

        if( polygonSeparation > edgeSeparation )
        {
            const int indexA1 = referenceIndex;
            const int indexA2 = indexA1 < polygon.vertexCount - 1 ? indexA1 + 1 : 0;
            const vec2 a1 = vertices[indexA1];
            const vec2 a2 = vertices[indexA2];
            const vec2 normal = normals[indexA1];

            const float separation1 = Dot( normal, point1 - a1 );
            const float separation2 = Dot( normal, point2 - a1 );

            if( separation1 < separation2 )
            {
                if( Dot( normal0, normal ) < Dot( normal1, normal ) ) return manifold;
            }
            else
            {
                if( Dot( normal2, normal ) < Dot( normal1, normal ) ) return manifold;
            }

            manifold = ClipChainSegments( a1, a2, point1, point2, normal, radiusB, 0.0f, MakeContactPointId( indexA1, 1 ), MakeContactPointId( indexA2, 0 ) );

            if( manifold.pointCount == 2 ) manifold.normal = -normal;

            return manifold;
        }

        if( incidentIndex == -1 ) return manifold;
    }

    assert( incidentNormal != -1 || incidentIndex != -1 );

    // Segment가 reference일 때 deepest polygon vertex에 인접한 두 face 중
    // segment normal과 더 anti-parallel한 edge를 incident edge로 선택함.
    int indexB1 = 0;
    int indexB2 = 0;
    vec2 b1{};
    vec2 b2{};

    if( incidentNormal != -1 )
    {
        indexB1 = incidentNormal;
        indexB2 = indexB1 < polygon.vertexCount - 1 ? indexB1 + 1 : 0;
        b1 = vertices[indexB1];
        b2 = vertices[indexB2];
    }
    else
    {
        const int index2 = incidentIndex;
        const int index1 = index2 > 0 ? index2 - 1 : polygon.vertexCount - 1;
        const float dot1 = Dot( normal1, normals[index1] );
        const float dot2 = Dot( normal1, normals[index2] );

        if( dot1 < dot2 )
        {
            indexB1 = index1;
            indexB2 = index2;
        }
        else
        {
            indexB1 = index2;
            indexB2 = index2 < polygon.vertexCount - 1 ? index2 + 1 : 0;
        }

        b1 = vertices[indexB1];
        b2 = vertices[indexB2];
    }

    return ClipChainSegments( point1, point2, b1, b2, normal1, 0.0f, radiusB, MakeContactPointId( 0, indexB2 ), MakeContactPointId( 1, indexB1 ) );
}

} // namespace zonai
