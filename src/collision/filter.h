#pragma once

#include <cstdint>
#include <array>
#include <cassert>
#include <limits>

namespace zonai
{

// Shape mask와 별개인 category 쌍의 공통 허용 규칙. 기본값은 모든 쌍을 허용함.
class collisionMatrix
{
public:
    constexpr collisionMatrix() { rows_.fill( ~std::uint64_t{ 0 } ); }

    constexpr void setPair( int a, int b, bool allowed )
    {
        assert( a >= 0 && a < 64 && b >= 0 && b < 64 );

        const auto bitA = std::uint64_t{ 1 } << a, bitB = std::uint64_t{ 1 } << b;
        rows_[a] = allowed ? rows_[a] | bitB : rows_[a] & ~bitB;
        rows_[b] = allowed ? rows_[b] | bitA : rows_[b] & ~bitA;
    }

    [[nodiscard]] constexpr bool allows( std::uint64_t a, std::uint64_t b ) const
    {
        // Category가 없는 shape는 공통 쌍 규칙의 대상이 아님. 기존 group/mask 판정은 유지함.
        if( a == 0 || b == 0 ) return true;

        for( int bit = 0; a != 0; ++bit, a >>= 1 )
        {
            if( ( a & 1 ) != 0 && ( rows_[bit] & b ) != 0 ) return true;
        }
        return false;
    }

    friend bool operator==( const collisionMatrix&, const collisionMatrix& ) = default;

private:
    std::array<std::uint64_t, 64> rows_{};
};

struct collisionFilter
{
    // shape가 속한 collision category를 나타냄.
    std::uint64_t categoryBits = 1;

    // 충돌을 허용할 상대 category bit를 나타냄.
    std::uint64_t maskBits = std::numeric_limits<std::uint64_t>::max();

    // 같은 non-zero group이면 mask보다 우선함. 양수는 항상 충돌, 음수는 충돌하지 않음.
    std::int32_t groupIndex = 0;
};

// 두 shape filter가 실제 충돌을 허용하는지 확인함.
constexpr bool ShouldShapesCollide( const collisionFilter& filterA, const collisionFilter& filterB )
{
    // 같은 non-zero group은 category / mask보다 우선함.
    if( filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 ) return filterA.groupIndex > 0;

    // 양쪽 mask가 서로의 category를 모두 허용해야 충돌함.
    return ( filterA.maskBits & filterB.categoryBits ) != 0 && ( filterA.categoryBits & filterB.maskBits ) != 0;
}

} // namespace zonai
