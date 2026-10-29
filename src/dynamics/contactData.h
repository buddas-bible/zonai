#pragma once

#include "collision/narrowphase/manifold2.h"
#include "dynamics/id.h"

namespace zonai
{

// world 외부에 노출하는 Contact snapshot.
// 내부 linked-list / free-list / raw index 정보는 포함하지 않음.
// Shape pair 기반 transient Contact 이벤트. End 이벤트의 Shape handle은 이미 파괴됐을 수 있음.
struct contactBeginEvent2
{
    shapeId shapeA{};
    shapeId shapeB{};
};

struct contactEndEvent2
{
    shapeId shapeA{};
    shapeId shapeB{};
};

// Solver가 실제 충돌을 처리한 접점 중 접근 속도가 가장 큰 지점의 snapshot.
struct contactHitEvent2
{
    contactId contact{};
    shapeId shapeA{};
    shapeId shapeB{};
    vec2 point{};
    vec2 normal{};
    float approachSpeed = 0.0f;
    float normalImpulse = 0.0f;
};

struct contactData
{
    contactId id{};
    shapeId shapeA{};
    shapeId shapeB{};
    manifold2 manifold{};
};

} // namespace zonai
