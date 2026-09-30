#include <cassert>
#include <cstdint>

#include "collision/filter.h"

using namespace zonai;

int main()
{
    {
        const collisionFilter filterA{};
        const collisionFilter filterB{};

        assert( ShouldShapesCollide( filterA, filterB ) );
    }

    {
        collisionFilter player{};
        player.categoryBits = 0x00000001;
        player.maskBits = 0x00000002;

        collisionFilter enemy{};
        enemy.categoryBits = 0x00000002;
        enemy.maskBits = 0x00000001;

        assert( ShouldShapesCollide( player, enemy ) );

        enemy.maskBits = 0x00000004;

        // 한쪽이라도 상대 category를 허용하지 않으면 충돌하지 않음.
        assert( ShouldShapesCollide( player, enemy ) == false );
    }

    {
        collisionFilter filterA{};
        filterA.categoryBits = 0x00000001;
        filterA.maskBits = 0;
        filterA.groupIndex = 7;

        collisionFilter filterB{};
        filterB.categoryBits = 0x00000002;
        filterB.maskBits = 0;
        filterB.groupIndex = 7;

        // 같은 양수 group은 category / mask보다 우선해서 항상 충돌함.
        assert( ShouldShapesCollide( filterA, filterB ) );
    }

    {
        collisionFilter filterA{};
        filterA.groupIndex = -3;

        collisionFilter filterB{};
        filterB.groupIndex = -3;

        // 같은 음수 group은 category / mask보다 우선해서 충돌하지 않음.
        assert( ShouldShapesCollide( filterA, filterB ) == false );
    }

    {
        collisionFilter filterA{};
        filterA.groupIndex = 1;

        collisionFilter filterB{};
        filterB.groupIndex = 2;

        // 서로 다른 group은 override하지 않고 category / mask 규칙을 사용함.
        assert( ShouldShapesCollide( filterA, filterB ) );
    }

    return 0;
}
