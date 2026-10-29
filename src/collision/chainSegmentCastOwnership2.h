#pragma once

#include "collision/distance2.h"
#include "collision/narrowphase/collide.h"

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

} // namespace zonai
