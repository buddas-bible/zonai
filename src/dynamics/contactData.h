#pragma once

#include "collision/narrowphase/manifold2.h"
#include "dynamics/id.h"

namespace zonai
{

// World 외부에 노출하는 Contact snapshot.
// 내부 linked-list / free-list / raw index 정보는 포함하지 않음.
struct ContactData
{
    ContactId contactId{};
    ShapeId shapeIdA{};
    ShapeId shapeIdB{};
    manifold2 manifold{};
};

} // namespace zonai
