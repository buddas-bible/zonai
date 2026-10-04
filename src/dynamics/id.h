#pragma once

#include <cstdint>

namespace zonai
{

class world;

// world 외부에서 body를 참조하는 handle.
// index1은 0을 null로 남기기 위해 내부 index + 1을 저장함.
struct bodyId
{
    std::int32_t index1 = 0;

    // 같은 slot / generation이 다른 world에도 존재할 수 있으므로
    // handle을 만든 world instance도 함께 기억함.
    const world* owner = nullptr;

    std::uint16_t generation = 0;

    constexpr bool operator==( const bodyId& ) const = default;
};

// world 외부에서 shape를 참조하는 handle.
// slot이 재사용되더라도 generation이 달라져 오래된 handle을 검출할 수 있음.
struct shapeId
{
    std::int32_t index1 = 0;
    std::uint16_t generation = 0;

    constexpr bool operator==( const shapeId& ) const = default;
};

// world 외부에서 Contact를 참조하는 handle.
// Contact는 자동 생성/파괴 빈도가 높으므로 generation을 32bit로 유지함.
struct contactId
{
    std::int32_t index1 = 0;
    std::uint32_t generation = 0;

    constexpr bool operator==( const contactId& ) const = default;
};

constexpr bool IsNull( bodyId id ) noexcept
{
    return id.index1 == 0;
}

constexpr bool IsNull( shapeId id ) noexcept
{
    return id.index1 == 0;
}

constexpr bool IsNull( contactId id ) noexcept
{
    return id.index1 == 0;
}

} // namespace zonai
