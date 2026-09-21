#pragma once

#include <array>
#include <cstddef>

#include "collision/broadphase/dynamicTree.h"
#include "dynamics/bodyType.h"

namespace zonai
{

class BroadPhase
{
public:
    DynamicTree& GetTree( BodyType type );
    const DynamicTree& GetTree( BodyType type ) const;

private:
    static constexpr std::size_t BODY_TYPE_COUNT = static_cast<std::size_t>( BodyType::Count );

    // body type마다 독립된 DynamicTree를 사용함.
    std::array<DynamicTree, BODY_TYPE_COUNT> trees_{};
};

} // namespace zonai
