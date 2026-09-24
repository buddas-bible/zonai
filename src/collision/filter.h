#pragma once

#include <cstdint>
#include <limits>

namespace zonai
{

struct Filter
{
    // shape가 속한 collision category를 나타냄.
    std::uint64_t categoryBits = 1;

    // 충돌을 허용할 상대 category bit를 나타냄.
    std::uint64_t maskBits = std::numeric_limits<std::uint64_t>::max();

    // 같은 non-zero group이면 mask보다 우선함. 양수는 항상 충돌, 음수는 충돌하지 않음.
    std::int32_t groupIndex = 0;
};

// 두 shape filter가 실제 충돌을 허용하는지 확인함.
constexpr bool ShouldShapesCollide( const Filter& filterA, const Filter& filterB )
{
    // 같은 non-zero group은 category / mask보다 우선함.
    if( filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 )
    {
        return filterA.groupIndex > 0;
    }

    // 양쪽 mask가 서로의 category를 모두 허용해야 충돌함.
    return
        ( filterA.maskBits & filterB.categoryBits ) != 0 &&
        ( filterA.categoryBits & filterB.maskBits ) != 0;
}

} // namespace zonai
