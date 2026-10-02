#include "collision/distance2.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cfloat>
#include <cmath>

namespace zonai
{

namespace
{

struct simplexVertex2
{
    vec2 wA{};
    vec2 wB{};
    vec2 w{};

    float a = 0.0f;

    int indexA = 0;
    int indexB = 0;
};

struct simplex2
{
    simplexVertex2 v1{};
    simplexVertex2 v2{};
    simplexVertex2 v3{};

    int count = 0;
};

vec2 Weight(
    float a1,
    const vec2& w1,
    float a2,
    const vec2& w2 )
{
    return w1 * a1 + w2 * a2;
}

vec2 Weight(
    float a1,
    const vec2& w1,
    float a2,
    const vec2& w2,
    float a3,
    const vec2& w3 )
{
    return
        w1 * a1 +
        w2 * a2 +
        w3 * a3;
}

int FindSupport(
    const shapeProxy2& proxy,
    const vec2& direction )
{
    assert( proxy.count > 0 );

    int bestIndex = 0;
    float bestValue =
        Dot(
            proxy.points[0],
            direction
        );

    for( int i = 1; i < proxy.count; ++i )
    {
        const float value =
            Dot(
                proxy.points[i],
                direction
            );

        if( value > bestValue )
        {
            bestIndex = i;
            bestValue = value;
        }
    }

    return bestIndex;
}

simplex2 MakeSimplexFromCache(
    const simplexCache2& cache,
    const shapeProxy2& proxyA,
    const shapeProxy2& proxyB )
{
    assert( cache.count <= 3 );

    simplex2 simplex{};
    simplex.count = cache.count;

    std::array<simplexVertex2*, 3> vertices
    {
        &simplex.v1,
        &simplex.v2,
        &simplex.v3
    };

    for( int i = 0; i < simplex.count; ++i )
    {
        simplexVertex2& vertex =
            *vertices[i];

        vertex.indexA = cache.indexA[i];
        vertex.indexB = cache.indexB[i];

        assert( vertex.indexA >= 0 );
        assert( vertex.indexA < proxyA.count );
        assert( vertex.indexB >= 0 );
        assert( vertex.indexB < proxyB.count );

        vertex.wA =
            proxyA.points[vertex.indexA];

        vertex.wB =
            proxyB.points[vertex.indexB];

        vertex.w =
            vertex.wA - vertex.wB;

        vertex.a = -1.0f;
    }

    if( simplex.count == 0 )
    {
        simplexVertex2& vertex =
            *vertices[0];

        vertex.indexA = 0;
        vertex.indexB = 0;

        vertex.wA = proxyA.points[0];
        vertex.wB = proxyB.points[0];
        vertex.w = vertex.wA - vertex.wB;
        vertex.a = 1.0f;

        simplex.count = 1;
    }

    return simplex;
}

simplexCache2 MakeSimplexCache(
    const simplex2& simplex )
{
    simplexCache2 cache{};
    cache.count =
        static_cast<std::uint16_t>(
            simplex.count
        );

    const std::array<const simplexVertex2*, 3> vertices
    {
        &simplex.v1,
        &simplex.v2,
        &simplex.v3
    };

    for( int i = 0; i < simplex.count; ++i )
    {
        cache.indexA[i] =
            static_cast<std::uint8_t>(
                vertices[i]->indexA
            );

        cache.indexB[i] =
            static_cast<std::uint8_t>(
                vertices[i]->indexB
            );
    }

    return cache;
}

void ComputeWitnessPoints(
    const simplex2& simplex,
    vec2& pointA,
    vec2& pointB )
{
    switch( simplex.count )
    {
    case 1:
        pointA = simplex.v1.wA;
        pointB = simplex.v1.wB;
        break;

    case 2:
        pointA =
            Weight(
                simplex.v1.a,
                simplex.v1.wA,
                simplex.v2.a,
                simplex.v2.wA
            );

        pointB =
            Weight(
                simplex.v1.a,
                simplex.v1.wB,
                simplex.v2.a,
                simplex.v2.wB
            );
        break;

    case 3:
        pointA =
            Weight(
                simplex.v1.a,
                simplex.v1.wA,
                simplex.v2.a,
                simplex.v2.wA,
                simplex.v3.a,
                simplex.v3.wA
            );

        // origin이 Minkowski triangle 안에 있으므로 두 witness point는 동일함.
        pointB = pointA;
        break;

    default:
        assert( false );
        pointA = {};
        pointB = {};
        break;
    }
}

// 현재 2-point simplex에서 origin에 가장 가까운 Voronoi region을 선택하고
// 다음 support point를 찾을 방향을 반환함.
vec2 SolveSimplex2(
    simplex2& simplex )
{
    const vec2 w1 = simplex.v1.w;
    const vec2 w2 = simplex.v2.w;
    const vec2 e12 = w2 - w1;

    const float d12_2 =
        -Dot( w1, e12 );

    if( d12_2 <= 0.0f )
    {
        simplex.v1.a = 1.0f;
        simplex.count = 1;

        return -w1;
    }

    const float d12_1 =
        Dot( w2, e12 );

    if( d12_1 <= 0.0f )
    {
        simplex.v2.a = 1.0f;
        simplex.v1 = simplex.v2;
        simplex.count = 1;

        return -w2;
    }

    const float inverseD12 =
        1.0f /
        ( d12_1 + d12_2 );

    simplex.v1.a =
        d12_1 * inverseD12;

    simplex.v2.a =
        d12_2 * inverseD12;

    simplex.count = 2;

    return
        Cross(
            Cross(
                w1 + w2,
                e12
            ),
            e12
        );
}

vec2 SolveSimplex3(
    simplex2& simplex )
{
    const vec2 w1 = simplex.v1.w;
    const vec2 w2 = simplex.v2.w;
    const vec2 w3 = simplex.v3.w;

    const vec2 e12 = w2 - w1;
    const float w1e12 = Dot( w1, e12 );
    const float w2e12 = Dot( w2, e12 );
    const float d12_1 = w2e12;
    const float d12_2 = -w1e12;

    const vec2 e13 = w3 - w1;
    const float w1e13 = Dot( w1, e13 );
    const float w3e13 = Dot( w3, e13 );
    const float d13_1 = w3e13;
    const float d13_2 = -w1e13;

    const vec2 e23 = w3 - w2;
    const float w2e23 = Dot( w2, e23 );
    const float w3e23 = Dot( w3, e23 );
    const float d23_1 = w3e23;
    const float d23_2 = -w2e23;

    const float n123 =
        Cross(
            e12,
            e13
        );

    const float d123_1 =
        n123 *
        Cross(
            w2,
            w3
        );

    const float d123_2 =
        n123 *
        Cross(
            w3,
            w1
        );

    const float d123_3 =
        n123 *
        Cross(
            w1,
            w2
        );

    if( d12_2 <= 0.0f &&
        d13_2 <= 0.0f )
    {
        simplex.v1.a = 1.0f;
        simplex.count = 1;

        return -w1;
    }

    if( d12_1 > 0.0f &&
        d12_2 > 0.0f &&
        d123_3 <= 0.0f )
    {
        const float inverseD12 =
            1.0f /
            ( d12_1 + d12_2 );

        simplex.v1.a =
            d12_1 * inverseD12;

        simplex.v2.a =
            d12_2 * inverseD12;

        simplex.count = 2;

        return
            Cross(
                Cross(
                    w1 + w2,
                    e12
                ),
                e12
            );
    }

    if( d13_1 > 0.0f &&
        d13_2 > 0.0f &&
        d123_2 <= 0.0f )
    {
        const float inverseD13 =
            1.0f /
            ( d13_1 + d13_2 );

        simplex.v1.a =
            d13_1 * inverseD13;

        simplex.v3.a =
            d13_2 * inverseD13;

        simplex.v2 = simplex.v3;
        simplex.count = 2;

        return
            Cross(
                Cross(
                    w1 + w3,
                    e13
                ),
                e13
            );
    }

    if( d12_1 <= 0.0f &&
        d23_2 <= 0.0f )
    {
        simplex.v2.a = 1.0f;
        simplex.v1 = simplex.v2;
        simplex.count = 1;

        return -w2;
    }

    if( d13_1 <= 0.0f &&
        d23_1 <= 0.0f )
    {
        simplex.v3.a = 1.0f;
        simplex.v1 = simplex.v3;
        simplex.count = 1;

        return -w3;
    }

    if( d23_1 > 0.0f &&
        d23_2 > 0.0f &&
        d123_1 <= 0.0f )
    {
        const float inverseD23 =
            1.0f /
            ( d23_1 + d23_2 );

        simplex.v2.a =
            d23_1 * inverseD23;

        simplex.v3.a =
            d23_2 * inverseD23;

        simplex.v1 = simplex.v3;
        simplex.count = 2;

        return
            Cross(
                Cross(
                    w2 + w3,
                    e23
                ),
                e23
            );
    }

    const float inverseD123 =
        1.0f /
        (
            d123_1 +
            d123_2 +
            d123_3
        );

    simplex.v1.a =
        d123_1 * inverseD123;

    simplex.v2.a =
        d123_2 * inverseD123;

    simplex.v3.a =
        d123_3 * inverseD123;

    simplex.count = 3;

    return {};
}

} // namespace

distanceOutput2 ShapeDistance(
    const distanceInput2& input,
    simplexCache2& cache )
{
    assert( input.proxyA.count > 0 );
    assert( input.proxyB.count > 0 );
    assert( input.proxyA.radius >= 0.0f );
    assert( input.proxyB.radius >= 0.0f );

    shapeProxy2 localProxyB{};
    localProxyB.count = input.proxyB.count;
    localProxyB.radius = input.proxyB.radius;

    for( int i = 0;
         i < localProxyB.count;
         ++i )
    {
        localProxyB.points[i] =
            TransformPoint(
                input.transform,
                input.proxyB.points[i]
            );
    }

    simplex2 simplex =
        MakeSimplexFromCache(
            cache,
            input.proxyA,
            localProxyB
        );

    std::array<simplexVertex2*, 3> vertices
    {
        &simplex.v1,
        &simplex.v2,
        &simplex.v3
    };

    std::array<int, 3> savedA{};
    std::array<int, 3> savedB{};

    vec2 nonUnitNormal{};
    int iteration = 0;

    constexpr int MAX_ITERATIONS = 20;

    while( iteration < MAX_ITERATIONS )
    {
        const int savedCount =
            simplex.count;

        for( int i = 0;
             i < savedCount;
             ++i )
        {
            savedA[i] =
                vertices[i]->indexA;

            savedB[i] =
                vertices[i]->indexB;
        }

        vec2 direction{};

        switch( simplex.count )
        {
        case 1:
            direction =
                -simplex.v1.w;
            break;

        case 2:
            direction =
                SolveSimplex2(
                    simplex
                );
            break;

        case 3:
            direction =
                SolveSimplex3(
                    simplex
                );
            break;

        default:
            assert( false );
            break;
        }

        if( simplex.count == 3 )
        {
            cache =
                MakeSimplexCache(
                    simplex
                );

            distanceOutput2 output{};
            ComputeWitnessPoints(
                simplex,
                output.pointA,
                output.pointB
            );

            output.distance = 0.0f;
            output.iterations = iteration;

            return output;
        }

        if( LengthSquared( direction ) <
            1000.0f * FLT_MIN )
        {
            cache =
                MakeSimplexCache(
                    simplex
                );

            distanceOutput2 output{};
            ComputeWitnessPoints(
                simplex,
                output.pointA,
                output.pointB
            );

            output.distance = 0.0f;
            output.iterations = iteration;

            return output;
        }

        nonUnitNormal = direction;

        simplexVertex2& vertex =
            *vertices[simplex.count];

        vertex.indexA =
            FindSupport(
                input.proxyA,
                direction
            );

        vertex.wA =
            input.proxyA.points[
                vertex.indexA
            ];

        vertex.indexB =
            FindSupport(
                localProxyB,
                -direction
            );

        vertex.wB =
            localProxyB.points[
                vertex.indexB
            ];

        vertex.w =
            vertex.wA -
            vertex.wB;

        ++iteration;

        bool duplicate = false;

        for( int i = 0;
             i < savedCount;
             ++i )
        {
            if( vertex.indexA == savedA[i] &&
                vertex.indexB == savedB[i] )
            {
                duplicate = true;
                break;
            }
        }

        if( duplicate )
        {
            break;
        }

        ++simplex.count;
    }

    distanceOutput2 output{};

    output.normal =
        Normalize(
            nonUnitNormal
        );

    ComputeWitnessPoints(
        simplex,
        output.pointA,
        output.pointB
    );

    output.distance =
        Length(
            output.pointA -
            output.pointB
        );

    output.iterations = iteration;

    cache =
        MakeSimplexCache(
            simplex
        );

    if( input.useRadii )
    {
        const float radiusA =
            input.proxyA.radius;

        const float radiusB =
            input.proxyB.radius;

        output.distance =
            std::max(
                0.0f,
                output.distance -
                radiusA -
                radiusB
            );

        output.pointA +=
            output.normal *
            radiusA;

        output.pointB -=
            output.normal *
            radiusB;
    }

    return output;
}

} // namespace zonai
