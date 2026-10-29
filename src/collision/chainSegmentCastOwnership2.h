#pragma once

#include "collision/narrowphase/collide.h"

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

} // namespace zonai
