#include <cassert>
#include <cstdint>

#include "collision/filter.h"

using namespace zonai;

int main()
{
    {
        const Filter filterA{};
        const Filter filterB{};

        assert( ShouldShapesCollide( filterA, filterB ) );
    }

    {
        Filter player{};
        player.categoryBits = 0x00000001;
        player.maskBits = 0x00000002;

        Filter enemy{};
        enemy.categoryBits = 0x00000002;
        enemy.maskBits = 0x00000001;

        assert( ShouldShapesCollide( player, enemy ) );

        enemy.maskBits = 0x00000004;

        // 한쪽이라도 상대 category를 허용하지 않으면 충돌하지 않음.
        assert( ShouldShapesCollide( player, enemy ) == false );
    }

    {
        Filter filterA{};
        filterA.categoryBits = 0x00000001;
        filterA.maskBits = 0;
        filterA.groupIndex = 7;

        Filter filterB{};
        filterB.categoryBits = 0x00000002;
        filterB.maskBits = 0;
        filterB.groupIndex = 7;

        // 같은 양수 group은 category / mask보다 우선해서 항상 충돌함.
        assert( ShouldShapesCollide( filterA, filterB ) );
    }

    {
        Filter filterA{};
        filterA.groupIndex = -3;

        Filter filterB{};
        filterB.groupIndex = -3;

        // 같은 음수 group은 category / mask보다 우선해서 충돌하지 않음.
        assert( ShouldShapesCollide( filterA, filterB ) == false );
    }

    {
        Filter filterA{};
        filterA.groupIndex = 1;

        Filter filterB{};
        filterB.groupIndex = 2;

        // 서로 다른 group은 override하지 않고 category / mask 규칙을 사용함.
        assert( ShouldShapesCollide( filterA, filterB ) );
    }

    return 0;
}
