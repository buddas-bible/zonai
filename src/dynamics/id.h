#pragma once

#include <cstdint>

namespace zonai
{

// World 외부에서 Body를 참조하는 handle.
// index1은 0을 null로 남기기 위해 내부 index + 1을 저장함.
struct BodyId
{
    std::int32_t index1 = 0;
    std::uint16_t generation = 0;

    constexpr bool operator==( const BodyId& ) const = default;
};

// World 외부에서 Shape를 참조하는 handle.
// slot이 재사용되더라도 generation이 달라져 오래된 handle을 검출할 수 있음.
struct ShapeId
{
    std::int32_t index1 = 0;
    std::uint16_t generation = 0;

    constexpr bool operator==( const ShapeId& ) const = default;
};

constexpr bool IsNull( BodyId id ) noexcept
{
    return id.index1 == 0;
}

constexpr bool IsNull( ShapeId id ) noexcept
{
    return id.index1 == 0;
}

} // namespace zonai
