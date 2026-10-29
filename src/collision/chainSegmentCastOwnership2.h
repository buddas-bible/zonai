#pragma once

#include "collision/constants.h"
#include "collision/distance2.h"
#include "collision/narrowphase/collide.h"
#include "collision/sweep2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <variant>

namespace zonai
{

// World의 Chain Circle cast는 solver와 같은 ghost/Voronoi 판정을 이용하되,
// 같은 접합 vertex의 endpoint contact가 두 이웃 edge에서 중복 보고되지 않게 함.
// hasPrevious는 실제 parent Chain 안에 이전 edge가 있는 경우에만 true.
inline bool IsChainCircleCastOwned( const chainSegment2& chain, const circle2& circle,
    const transform2& circleInChain, bool hasPrevious )
{
    if( CollideChainSegmentCircle( chain, circle, circleInChain ).pointCount == 0 ) return false;
    if( !hasPrevious ) return true;

    const vec2 relative = TransformPoint( circleInChain, circle.center ) - chain.segment.a;
    const vec2 edge = chain.segment.b - chain.segment.a;

    // 현재 edge의 시작 endpoint 밖이고 이전 edge의 끝 endpoint도 같은 위치라면,
    // 두 edge의 ghost 검사에서 해당 꼭짓점이 각각 인정될 수 있음.
    // 이전 edge의 실제 앞면 영역일 때만 이전 edge에 소유권을 줌.
    if( Dot( edge, relative ) <= 0.0f )
    {
        const vec2 previousEdge = chain.segment.a - chain.ghost1;
        if( Dot( previousEdge, relative ) > 0.0f &&
            Dot( Cross( previousEdge, 1.0f ), relative ) > 0.0f )
            return false;
    }

    return true;
}


 // Capsule과 Polygon에는 단일 중심의 Voronoi 판정을 적용하지 않음.
 // 기존 Chain narrowphase의 adjacency normal 분류/clip을 그대로 사용함.
 // 이 검사는 실제 ShapeCast/TOI hit 시점에만 수행되므로 일반 도형의
 // broadphase 순회나 Solver의 contact 생성 경로에는 영향을 주지 않음.
inline bool IsChainConvexCastOwned( const chainSegment2& chain, const shapeGeometry& geometry,
    const transform2& movingInChain, bool hasPrevious )
{
    if( const circle2* circle = std::get_if<circle2>( &geometry ) )
        return IsChainCircleCastOwned( chain, *circle, movingInChain, hasPrevious );

    if( const capsule2* capsule = std::get_if<capsule2>( &geometry ) )
    {
        // ShapeCast는 매우 짧은 segment core를 거절하지 않으므로
        // MakeCapsule의 퇴화 edge assert 대신 동등한 원으로 검증함.
        if( LengthSquared( capsule->center2 - capsule->center1 ) <=
            std::numeric_limits<float>::epsilon() )
        {
            const circle2 circle{ 0.5f * ( capsule->center1 + capsule->center2 ), capsule->radius };
            return IsChainCircleCastOwned( chain, circle, movingInChain, hasPrevious );
        }

        simplexCache2 cache{};
        return CollideChainSegmentCapsule( chain, *capsule, movingInChain, cache ).pointCount > 0;
    }

    if( const polygon2* polygon = std::get_if<polygon2>( &geometry ) )
    {
        simplexCache2 cache{};
        return CollideChainSegmentPolygon( chain, *polygon, movingInChain, cache ).pointCount > 0;
    }

    // 일반 양면 Segment는 기존의 Chain front-normal 검사 의미를 유지함.
    return true;
}



struct chainSweepContact2
{
    vec2 point{};
    vec2 normal{};
    float fraction = 1.0f;
    bool hit = false;
};

struct chainSweepRetryStats2
{
    int intervals = 0;
    int distanceCalls = 0;
    int manifoldCalls = 0;
    int pruned = 0;
    bool exhausted = false;
};

// Ghost 접촉이 먼저 나타난 회전 Capsule/Polygon에 한해서 소유한 첫 접촉을 찾음.
// 좌측 시간 구간을 먼저 확인하여 BFS에서 발생했던 뒤늦은 hit 우선 문제를 방지함.
// 정해진 탐색 깊이와 검사 횟수 이후에는 유효한 접촉만 반환하고, 완전성은 주장하지 않음.
inline chainSweepContact2 FindOwnedChainSweepContact( const chainSegment2& chain,
    const shapeGeometry& geometry, const shapeProxy2& movingProxy,
    const transform2& targetTransform, const sweep2& movingSweep,
    float afterFraction, float maxFraction, chainSweepRetryStats2* stats = nullptr )
{
    if( stats ) *stats = {};
    if( !std::holds_alternative<capsule2>( geometry ) &&
        !std::holds_alternative<polygon2>( geometry ) ) return {};
    if( !( afterFraction < maxFraction ) ) return {};

    const vec2 edge = chain.segment.b - chain.segment.a;
    const float edgeLength = Length( edge );
    if( edgeLength <= 0.0f ) return {};
    const vec2 axis = edge / edgeLength;
    const vec2 front = Cross( axis, 1.0f );

    double radius = movingProxy.radius;
    for( int i = 0; i < movingProxy.count; ++i )
        radius = std::max( radius,
            std::hypot( static_cast<double>( movingProxy.points[i].x ),
                static_cast<double>( movingProxy.points[i].y ) ) + movingProxy.radius );

    // Nlerp의 최소 회전 길이로 전체 sweep의 각속도 상한을 계산함.
    const double dc = movingSweep.q2.c - movingSweep.q1.c;
    const double ds = movingSweep.q2.s - movingSweep.q1.s;
    const double d2 = dc * dc + ds * ds;
    const double nearest = d2 > 0.0 ?
        std::clamp( -( movingSweep.q1.c * dc + movingSweep.q1.s * ds ) / d2, 0.0, 1.0 ) : 0.0;
    const double c = movingSweep.q1.c + nearest * dc;
    const double sn = movingSweep.q1.s + nearest * ds;
    const double minRotationLength2 = c * c + sn * sn;
    if( minRotationLength2 <= 0.0 ) return {};
    const double angularSpeed = std::fabs( movingSweep.q1.c * movingSweep.q2.s -
        movingSweep.q1.s * movingSweep.q2.c ) / minRotationLength2;
    const vec2 translation = movingSweep.c2 - movingSweep.c1;
    const double speed = 1.001 * ( std::hypot( static_cast<double>( translation.x ),
        static_cast<double>( translation.y ) ) + angularSpeed * radius );
    if( !std::isfinite( speed ) ) return {};

    const auto manifoldAt = [&]( float time ) -> chainSweepContact2
    {
        if( stats ) ++stats->manifoldCalls;
        const transform2 inChain = InverseMul( targetTransform,
            GetSweepTransform( movingSweep, time ) );
        simplexCache2 cache{};
        localManifold2 manifold{};
        if( const capsule2* capsule = std::get_if<capsule2>( &geometry ) )
            manifold = CollideChainSegmentCapsule( chain, *capsule, inChain, cache );
        else
            manifold = CollideChainSegmentPolygon( chain,
                std::get<polygon2>( geometry ), inChain, cache );

        if( manifold.pointCount == 0 || Dot( manifold.normal, front ) <= 0.0f ) return {};
        vec2 point{};
        for( int i = 0; i < manifold.pointCount; ++i ) point += manifold.points[i].point;
        point /= static_cast<float>( manifold.pointCount );
        return { TransformPoint( targetTransform, point ),
            TransformVector( targetTransform, manifold.normal ), time, true };
    };

    distanceInput2 distance{};
    distance.proxyA = MakeShapeProxy( chain );
    distance.proxyA.oneSided = false;
    distance.proxyB = movingProxy;
    distance.useRadii = false;

    struct interval
    {
        float lower = 0.0f;
        float upper = 1.0f;
        int depth = 0;
    };

    constexpr int MAX_DEPTH = 9;
    constexpr int MAX_INTERVALS = 1024;
    std::array<interval, MAX_DEPTH + 2> stack{};
    int count = 1;
    int visited = 0;
    stack[0] = { afterFraction, maxFraction, 0 };
    chainSweepContact2 earliest{};
    float lowerBound = afterFraction;

    // 오른쪽을 먼저 push해 왼쪽(과거) 구간부터 방문한다.
    // 이미 찾은 hit보다 뒤인 구간은 무시하므로 탐색 결과가 큐 순서에 좌우되지 않음.
    while( count > 0 && visited < MAX_INTERVALS )
    {
        const interval current = stack[--count];
        if( earliest.hit && current.lower >= earliest.fraction ) continue;
        const float middle = current.lower + 0.5f * ( current.upper - current.lower );
        if( middle <= current.lower || middle >= current.upper ) continue;
        ++visited;
        if( stats ) ++stats->intervals;

        const transform2 inChain = InverseMul( targetTransform,
            GetSweepTransform( movingSweep, middle ) );
        float minAxis = std::numeric_limits<float>::infinity();
        float maxAxis = -minAxis;
        float minFront = minAxis;
        float maxFront = -minAxis;
        for( int i = 0; i < movingProxy.count; ++i )
        {
            const vec2 relative = TransformPoint( inChain, movingProxy.points[i] ) - chain.segment.a;
            const float along = Dot( relative, axis );
            const float outward = Dot( relative, front );
            minAxis = std::min( minAxis, along );
            maxAxis = std::max( maxAxis, along );
            minFront = std::min( minFront, outward );
            maxFront = std::max( maxFront, outward );
        }

        const double travel = 0.5 * speed * ( current.upper - current.lower ) +
            32.0 * std::numeric_limits<float>::epsilon() * ( radius + edgeLength + 1.0 );
        const double extension = movingProxy.radius + SPECULATIVE_DISTANCE + travel;
        if( maxAxis + extension < 0.0 || minAxis - extension > edgeLength ||
            minFront - extension > 0.0 || maxFront + extension < 0.0 )
        {
            if( stats ) ++stats->pruned;
            continue;
        }

        // Projection이 겹치더라도 실제 유한 선분과 도형이 멀면 해당 구간
        // 전체를 거리 변화의 Lipschitz 상한으로 안전하게 제외할 수 있음.
        distance.transform = inChain;
        simplexCache2 cache{};
        const distanceOutput2 separation = ShapeDistance( distance, cache );
        if( stats ) ++stats->distanceCalls;
        if( std::isfinite( separation.distance ) &&
            separation.distance > extension )
        {
            if( stats ) ++stats->pruned;
            continue;
        }

        const chainSweepContact2 contact = manifoldAt( middle );
        if( contact.hit && ( !earliest.hit || contact.fraction < earliest.fraction ) )
        {
            earliest = contact;
            lowerBound = current.lower;
        }
        else if( !contact.hit && middle < ( earliest.hit ? earliest.fraction : maxFraction ) )
            lowerBound = std::max( lowerBound, middle );

        if( current.depth == MAX_DEPTH ) continue;
        if( !contact.hit && ( !earliest.hit || middle < earliest.fraction ) )
            stack[count++] = { middle, current.upper, current.depth + 1 };
        stack[count++] = { current.lower, middle, current.depth + 1 };
    }

    if( stats ) stats->exhausted = count > 0;
    if( !earliest.hit ) return {};

    // 최초 유효 샘플의 왼쪽 구간 안에서만 경계를 좁힘.
    // 전체 [afterFraction,hit]에 접촉 여부가 단조롭다고 가정하지 않음.
    float lower = std::min( lowerBound, earliest.fraction );
    float upper = earliest.fraction;
    for( int i = 0; i < 24; ++i )
    {
        const float middle = lower + 0.5f * ( upper - lower );
        if( middle <= lower || middle >= upper ) break;
        const chainSweepContact2 contact = manifoldAt( middle );
        if( contact.hit )
        {
            earliest = contact;
            upper = middle;
        }
        else lower = middle;
    }

    return earliest;
}

} // namespace zonai
