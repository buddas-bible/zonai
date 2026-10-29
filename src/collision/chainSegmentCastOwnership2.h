#pragma once

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

// Generic TOI가 끝점의 ghost 접촉을 먼저 보고 World가 거절한 경우에만 호출.
// 이동 속도 상한으로 접촉 가능성이 없는 시간 구간을 제거하고,
// 남은 구간에서 Chain narrowphase의 실제 앞면 manifold를 다시 확인함.
// 일정한 간격의 전체 sweep 샘플링이 아니라 제한된 구간 이분 탐색임.
inline chainSweepContact2 FindOwnedChainSweepContact( const chainSegment2& chain,
    const shapeGeometry& geometry, const shapeProxy2& movingProxy,
    const transform2& targetTransform, const sweep2& movingSweep,
    float afterFraction, float maxFraction )
{
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
    {
        radius = std::max( radius,
            std::hypot( static_cast<double>( movingProxy.points[i].x ),
                static_cast<double>( movingProxy.points[i].y ) ) + movingProxy.radius );
    }

    // Nlerp 각속도는 |cross(q1,q2)| / |(1-t)q1+tq2|². 최소 분모를
    // 전체 [0,1]에서 구하므로 각 support 점의 이동 거리 상한이 됨.
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

    struct interval
    {
        float lower = 0.0f;
        float upper = 1.0f;
        int depth = 0;
    };
    std::array<interval, 256> queue{};
    int first = 0;
    int last = 1;
    queue[0] = { afterFraction, maxFraction, 0 };
    chainSweepContact2 recovered{};

    // 간격 이분은 실제 도형의 support 점 경계로 선별함.
    // 재탐색 횟수 상한은 ghost 거절을 반복하는 병적 입력의 비용을 제한함.
    while( first < last && last < static_cast<int>( queue.size() ) )
    {
        const interval current = queue[first++];
        const float middle = current.lower + 0.5f * ( current.upper - current.lower );
        if( middle <= current.lower || middle >= current.upper ) continue;

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

        // 속도 상한으로 구간 전체가 선분의 투영 범위나 앞면에서 벗어난
        // 것이 증명될 때만 건너뜀. 끝점의 무한 연장선 hit은 사용하지 않음.
        if( maxAxis + extension < 0.0 ||
            minAxis - extension > edgeLength ||
            minFront - extension > 0.0 ||
            maxFront + extension < 0.0 ) continue;

        const chainSweepContact2 contact = manifoldAt( middle );
        if( contact.hit )
        {
            recovered = contact;
            break;
        }
        if( current.depth < 7 && last + 1 < static_cast<int>( queue.size() ) )
        {
            queue[last++] = { current.lower, middle, current.depth + 1 };
            queue[last++] = { middle, current.upper, current.depth + 1 };
        }
    }

    if( !recovered.hit ) return {};

    // 첫 인정 접촉에 가까운 fraction으로 좁힘. 반환 normal/point는
    // 무시된 TOI 값이 아닌 유효한 Chain manifold에서만 가져옴.
    float lower = afterFraction;
    float upper = recovered.fraction;
    for( int i = 0; i < 24; ++i )
    {
        const float middle = lower + 0.5f * ( upper - lower );
        if( middle <= lower || middle >= upper ) break;
        const chainSweepContact2 contact = manifoldAt( middle );
        if( contact.hit )
        {
            recovered = contact;
            upper = middle;
        }
        else lower = middle;
    }
    return recovered;
}

} // namespace zonai
