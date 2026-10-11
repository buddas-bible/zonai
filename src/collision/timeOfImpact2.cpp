#include "collision/timeOfImpact2.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include "collision/constants.h"
#include "collision/distance2.h"

namespace zonai
{

namespace
{

enum class separationType2
{
    Points,
    FaceA,
    FaceB
};

struct separationFunction2
{
    const shapeProxy2* proxyA = nullptr;
    const shapeProxy2* proxyB = nullptr;

    sweep2 sweepA{};
    sweep2 sweepB{};

    vec2 localPoint{};
    vec2 axis{};

    separationType2 type = separationType2::Points;
};

int FindSupport( const shapeProxy2& proxy, const vec2& direction )
{
    assert( proxy.count > 0 );

    int bestIndex = 0;
    float bestValue = Dot( proxy.points[0], direction );

    for( int i = 1; i < proxy.count; ++i )
    {
        const float value = Dot( proxy.points[i], direction );

        if( value > bestValue )
        {
            bestIndex = i;
            bestValue = value;
        }
    }

    return bestIndex;
}

vec2 ComputeProxyCentroid( const shapeProxy2& proxy )
{
    assert( proxy.count > 0 );

    if( proxy.count == 1 ) return proxy.points[0];

    if( proxy.count == 2 ) return 0.5f * ( proxy.points[0] + proxy.points[1] );

    float area2 = 0.0f;
    vec2 centroid{};
    // Box2D처럼 첫 정점을 기준으로 계산해 큰 로컬 좌표의 상쇄 오차를 줄임.
    const vec2 origin = proxy.points[0];

    for( int i = 0; i < proxy.count; ++i )
    {
        const vec2 point1 = proxy.points[i] - origin;
        const vec2 point2 = proxy.points[( i + 1 ) % proxy.count] - origin;
        const float cross = Cross( point1, point2 );

        area2 += cross;
        centroid += cross * ( point1 + point2 );
    }

    if( std::fabs( area2 ) > 0.0f ) return origin + centroid / ( 3.0f * area2 );

    centroid = {};

    for( int i = 0; i < proxy.count; ++i ) centroid += proxy.points[i] - origin;

    return origin + centroid / static_cast<float>( proxy.count );
}

float ComputeProxyMinExtent( const shapeProxy2& proxy, const vec2& centroid )
{
    assert( proxy.count > 0 );

    if( proxy.count <= 2 ) return proxy.radius;

    float minExtent = std::numeric_limits<float>::max();

    for( int i = 0; i < proxy.count; ++i )
    {
        const vec2 point1 = proxy.points[i];
        const vec2 point2 = proxy.points[( i + 1 ) % proxy.count];
        const vec2 edge = point2 - point1;
        const float edgeLength = Length( edge );

        if( edgeLength <= 0.0f ) continue;

        const vec2 normal = Cross( edge, 1.0f ) / edgeLength;
        minExtent = std::min( minExtent, Dot( normal, point1 - centroid ) );
    }

    if( minExtent == std::numeric_limits<float>::max() ) return proxy.radius;

    return std::max( 0.0f, minExtent + proxy.radius );
}

bool ShouldSkipOneSidedToi( const toiInput2& input )
{
    if( !input.proxyA.oneSided || input.proxyA.count != 2 ) return false;

    // Box2D와 같이 one-sided chain은 generic convex TOI를 실행하기 전에
    // fast shape centroid가 뒷면에서 시작하거나 앞면을 향하지 않는 경우를 버림.
    const transform2 transformA = GetSweepTransform( input.sweepA, 0.0f );
    const vec2 point1 = TransformPoint( transformA, input.proxyA.points[0] );
    const vec2 point2 = TransformPoint( transformA, input.proxyA.points[1] );
    vec2 edge = point2 - point1;
    const float edgeLength = Length( edge );

    if( edgeLength <= LINEAR_SLOP ) return false;

    edge /= edgeLength;

    const vec2 localCentroidB = ComputeProxyCentroid( input.proxyB );
    const vec2 centroid1 = TransformPoint( GetSweepTransform( input.sweepB, 0.0f ), localCentroidB );
    const vec2 centroid2 = TransformPoint( GetSweepTransform( input.sweepB, input.maxFraction ), localCentroidB );
    const float separation1 = Cross( centroid1 - point1, edge );
    const float separation2 = Cross( centroid2 - point1, edge );
    const float coreDistance = 0.25f * ComputeProxyMinExtent( input.proxyB, localCentroidB );

    return separation1 < 0.0f || ( separation1 - separation2 < coreDistance && separation2 > coreDistance );
}

separationFunction2 MakeSeparationFunction( const simplexCache2& cache, const shapeProxy2& proxyA, const sweep2& sweepA, const shapeProxy2& proxyB, const sweep2& sweepB, float time )
{
    assert( cache.count > 0 );
    assert( cache.count < 3 );

    separationFunction2 function{};
    function.proxyA = &proxyA;
    function.proxyB = &proxyB;
    function.sweepA = sweepA;
    function.sweepB = sweepB;

    const transform2 transformA = GetSweepTransform( sweepA, time );

    const transform2 transformB = GetSweepTransform( sweepB, time );

    if( cache.count == 1 )
    {
        function.type = separationType2::Points;

        const vec2 pointA = TransformPoint( transformA, proxyA.points[cache.indexA[0]] );

        const vec2 pointB = TransformPoint( transformB, proxyB.points[cache.indexB[0]] );

        function.axis = Normalize( pointB - pointA );

        return function;
    }

    if( cache.indexA[0] == cache.indexA[1] )
    {
        function.type = separationType2::FaceB;

        const vec2 localPointB1 = proxyB.points[cache.indexB[0]];

        const vec2 localPointB2 = proxyB.points[cache.indexB[1]];

        function.axis = Normalize( Cross( localPointB2 - localPointB1, 1.0f ) );

        const vec2 normal = Rotate( transformB.rotation, function.axis );

        function.localPoint = ( localPointB1 + localPointB2 ) * 0.5f;

        const vec2 pointB = TransformPoint( transformB, function.localPoint );

        const vec2 pointA = TransformPoint( transformA, proxyA.points[cache.indexA[0]] );

        if( Dot( pointA - pointB, normal ) < 0.0f )
        {
            function.axis = -function.axis;
        }

        return function;
    }

    function.type = separationType2::FaceA;

    const vec2 localPointA1 = proxyA.points[cache.indexA[0]];

    const vec2 localPointA2 = proxyA.points[cache.indexA[1]];

    function.axis = Normalize( Cross( localPointA2 - localPointA1, 1.0f ) );

    const vec2 normal = Rotate( transformA.rotation, function.axis );

    function.localPoint = ( localPointA1 + localPointA2 ) * 0.5f;

    const vec2 pointA = TransformPoint( transformA, function.localPoint );

    const vec2 pointB = TransformPoint( transformB, proxyB.points[cache.indexB[0]] );

    if( Dot( pointB - pointA, normal ) < 0.0f )
    {
        function.axis = -function.axis;
    }

    return function;
}

float FindMinSeparation( const separationFunction2& function, int& indexA, int& indexB, float time )
{
    const transform2 transformA = GetSweepTransform( function.sweepA, time );

    const transform2 transformB = GetSweepTransform( function.sweepB, time );

    switch( function.type )
    {
    case separationType2::Points:
    {
        const vec2 axisA = InverseRotate( transformA.rotation, function.axis );

        const vec2 axisB = InverseRotate( transformB.rotation, -function.axis );

        indexA = FindSupport( *function.proxyA, axisA );

        indexB = FindSupport( *function.proxyB, axisB );

        const vec2 pointA = TransformPoint( transformA, function.proxyA->points[indexA] );

        const vec2 pointB = TransformPoint( transformB, function.proxyB->points[indexB] );

        return Dot( pointB - pointA, function.axis );
    }

    case separationType2::FaceA:
    {
        const vec2 normal = Rotate( transformA.rotation, function.axis );

        const vec2 pointA = TransformPoint( transformA, function.localPoint );

        const vec2 axisB = InverseRotate( transformB.rotation, -normal );

        indexA = -1;
        indexB = FindSupport( *function.proxyB, axisB );

        const vec2 pointB = TransformPoint( transformB, function.proxyB->points[indexB] );

        return Dot( pointB - pointA, normal );
    }

    case separationType2::FaceB:
    {
        const vec2 normal = Rotate( transformB.rotation, function.axis );

        const vec2 pointB = TransformPoint( transformB, function.localPoint );

        const vec2 axisA = InverseRotate( transformA.rotation, -normal );

        indexA = FindSupport( *function.proxyA, axisA );

        indexB = -1;

        const vec2 pointA = TransformPoint( transformA, function.proxyA->points[indexA] );

        return Dot( pointA - pointB, normal );
    }
    }

    assert( false );

    indexA = -1;
    indexB = -1;

    return 0.0f;
}

float EvaluateSeparation( const separationFunction2& function, int indexA, int indexB, float time )
{
    const transform2 transformA = GetSweepTransform( function.sweepA, time );

    const transform2 transformB = GetSweepTransform( function.sweepB, time );

    switch( function.type )
    {
    case separationType2::Points:
    {
        const vec2 pointA = TransformPoint( transformA, function.proxyA->points[indexA] );

        const vec2 pointB = TransformPoint( transformB, function.proxyB->points[indexB] );

        return Dot( pointB - pointA, function.axis );
    }

    case separationType2::FaceA:
    {
        const vec2 normal = Rotate( transformA.rotation, function.axis );

        const vec2 pointA = TransformPoint( transformA, function.localPoint );

        const vec2 pointB = TransformPoint( transformB, function.proxyB->points[indexB] );

        return Dot( pointB - pointA, normal );
    }

    case separationType2::FaceB:
    {
        const vec2 normal = Rotate( transformB.rotation, function.axis );

        const vec2 pointB = TransformPoint( transformB, function.localPoint );

        const vec2 pointA = TransformPoint( transformA, function.proxyA->points[indexA] );

        return Dot( pointA - pointB, normal );
    }
    }

    assert( false );

    return 0.0f;
}

} // namespace

toiOutput2 TimeOfImpact( const toiInput2& input )
{
    assert( input.proxyA.count > 0 );
    assert( input.proxyB.count > 0 );
    assert( input.proxyA.radius >= 0.0f );
    assert( input.proxyB.radius >= 0.0f );
    assert( std::isfinite( input.maxFraction ) );
    assert( input.maxFraction >= 0.0f );
    assert( input.maxFraction <= 1.0f );

    toiOutput2 output{};
    output.fraction = input.maxFraction;

    if( ShouldSkipOneSidedToi( input ) )
    {
        output.state = toiState2::Separated;
        return output;
    }

    const float totalRadius = input.proxyA.radius + input.proxyB.radius;

    const float target = std::max( LINEAR_SLOP, totalRadius - LINEAR_SLOP );

    const float tolerance = 0.25f * LINEAR_SLOP;

    assert( target > tolerance );

    float time1 = 0.0f;
    int distanceIterations = 0;

    constexpr int MAX_DISTANCE_ITERATIONS = 20;
    constexpr int MAX_ROOT_ITERATIONS = 50;

    simplexCache2 cache{};

    distanceInput2 distanceInput{};
    distanceInput.proxyA = input.proxyA;
    distanceInput.proxyB = input.proxyB;
    distanceInput.useRadii = false;

    for( ;; )
    {
        const transform2 transformA = GetSweepTransform( input.sweepA, time1 );

        const transform2 transformB = GetSweepTransform( input.sweepB, time1 );

        distanceInput.transform = InverseMul( transformA, transformB );

        const distanceOutput2 distanceOutput = ShapeDistance( distanceInput, cache );

        const vec2 worldNormal = Rotate( transformA.rotation, distanceOutput.normal );

        const vec2 worldPointA = TransformPoint( transformA, distanceOutput.pointA );

        const vec2 worldPointB = TransformPoint( transformA, distanceOutput.pointB );

        ++distanceIterations;

        if( distanceOutput.distance <= 0.0f )
        {
            output.state = toiState2::Overlapped;

            output.fraction = 0.0f;
            break;
        }

        if( distanceOutput.distance <= target + tolerance )
        {
            output.state = toiState2::Hit;

            const vec2 pointA = worldPointA + worldNormal * input.proxyA.radius;

            const vec2 pointB = worldPointB - worldNormal * input.proxyB.radius;

            output.point = ( pointA + pointB ) * 0.5f;

            output.normal = worldNormal;

            output.fraction = time1;

            break;
        }

        const separationFunction2 function = MakeSeparationFunction( cache, input.proxyA, input.sweepA, input.proxyB, input.sweepB, time1 );

        bool done = false;
        float time2 = input.maxFraction;

        int pushBackIterations = 0;

        for( ;; )
        {
            int indexA = -1;
            int indexB = -1;

            float separation2 = FindMinSeparation( function, indexA, indexB, time2 );

            if( separation2 > target + tolerance )
            {
                output.state = toiState2::Separated;

                output.fraction = input.maxFraction;

                done = true;
                break;
            }

            if( separation2 > target - tolerance )
            {
                time1 = time2;
                break;
            }

            float separation1 = EvaluateSeparation( function, indexA, indexB, time1 );

            if( separation1 < target - tolerance )
            {
                output.state = toiState2::Failed;

                output.fraction = time1;

                done = true;
                break;
            }

            if( separation1 <= target + tolerance )
            {
                output.state = toiState2::Hit;

                const vec2 pointA = worldPointA + worldNormal * input.proxyA.radius;

                const vec2 pointB = worldPointB - worldNormal * input.proxyB.radius;

                output.point = ( pointA + pointB ) * 0.5f;

                output.normal = worldNormal;

                output.fraction = time1;

                done = true;
                break;
            }

            float lowerTime = time1;

            float upperTime = time2;

            for( int rootIteration = 0; rootIteration < MAX_ROOT_ITERATIONS; ++rootIteration )
            {
                float time = 0.0f;

                if( rootIteration % 2 == 1 )
                {
                    time = lowerTime + ( target - separation1 ) * ( upperTime - lowerTime ) / ( separation2 - separation1 );
                }
                else
                {
                    time = 0.5f * ( lowerTime + upperTime );
                }

                const float separation = EvaluateSeparation( function, indexA, indexB, time );

                if( std::fabs( separation - target ) < tolerance )
                {
                    time2 = time;
                    break;
                }

                if( separation > target )
                {
                    lowerTime = time;
                    separation1 = separation;
                }
                else
                {
                    upperTime = time;
                    separation2 = separation;
                }
            }

            ++pushBackIterations;

            if( pushBackIterations == MAX_POLYGON_VERTICES )
            {
                break;
            }
        }

        if( done )
        {
            break;
        }

        if( distanceIterations == MAX_DISTANCE_ITERATIONS )
        {
            output.state = toiState2::Failed;

            const vec2 pointA = worldPointA + worldNormal * input.proxyA.radius;

            const vec2 pointB = worldPointB - worldNormal * input.proxyB.radius;

            output.point = ( pointA + pointB ) * 0.5f;

            output.normal = worldNormal;

            output.fraction = time1;

            break;
        }
    }

    return output;
}

} // namespace zonai
