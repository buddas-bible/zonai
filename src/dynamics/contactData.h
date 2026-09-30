#pragma once

#include "collision/narrowphase/manifold2.h"
#include "dynamics/id.h"

namespace zonai
{

// world 외부에 노출하는 Contact snapshot.
// 내부 linked-list / free-list / raw index 정보는 포함하지 않음.
struct contactData
{
    contactId id{};
    shapeId shapeA{};
    shapeId shapeB{};
    manifold2 manifold{};
};

} // namespace zonai
