#include <cassert>
#include <cstdint>
#include <vector>

#include "dynamics/island2.h"

using namespace zonai;

int main()
{
    // Contact가 없으면 각 non-static body는 독립 island가 됨.
    {
        std::vector<body> bodies( 3 );

        bodies[0].bodyId = 0;
        bodies[0].type = bodyType::Dynamic;

        bodies[1].bodyId = 1;
        bodies[1].type = bodyType::Kinematic;

        bodies[2].bodyId = 2;
        bodies[2].type = bodyType::Static;

        const std::vector<island2> islands =
            BuildIslands(
                bodies,
                std::span<const contactSim2>{}
            );

        assert( islands.size() == 2 );

        assert( islands[0].bodyIds.size() == 1 );
        assert( islands[0].bodyIds[0] == 0 );
        assert( islands[0].contactIds.empty() );

        assert( islands[1].bodyIds.size() == 1 );
        assert( islands[1].bodyIds[0] == 1 );
        assert( islands[1].contactIds.empty() );
    }

    // solver-active Contact가 두 non-static body를 하나의 island로 연결함.
    {
        std::vector<body> bodies( 2 );

        bodies[0].bodyId = 0;
        bodies[0].type = bodyType::Dynamic;

        bodies[1].bodyId = 1;
        bodies[1].type = bodyType::Dynamic;

        std::vector<contactSim2> contacts( 1 );

        contacts[0].contactId = 0;
        contacts[0].bodyIdA = 0;
        contacts[0].bodyIdB = 1;
        contacts[0].manifold.pointCount = 1;

        const std::vector<island2> islands =
            BuildIslands(
                bodies,
                contacts
            );

        assert( islands.size() == 1 );
        assert( islands[0].bodyIds.size() == 2 );
        assert( islands[0].bodyIds[0] == 0 );
        assert( islands[0].bodyIds[1] == 1 );
        assert( islands[0].contactIds.size() == 1 );
        assert( islands[0].contactIds[0] == 0 );
    }

    // Static body는 island body 목록에서 제외되지만 Contact constraint는 island에 포함됨.
    {
        std::vector<body> bodies( 2 );

        bodies[0].bodyId = 0;
        bodies[0].type = bodyType::Static;

        bodies[1].bodyId = 1;
        bodies[1].type = bodyType::Dynamic;

        std::vector<contactSim2> contacts( 1 );

        contacts[0].contactId = 0;
        contacts[0].bodyIdA = 0;
        contacts[0].bodyIdB = 1;
        contacts[0].manifold.pointCount = 1;

        const std::vector<island2> islands =
            BuildIslands(
                bodies,
                contacts
            );

        assert( islands.size() == 1 );
        assert( islands[0].bodyIds.size() == 1 );
        assert( islands[0].bodyIds[0] == 1 );
        assert( islands[0].contactIds.size() == 1 );
        assert( islands[0].contactIds[0] == 0 );
    }

    // AABB pair만 있고 manifold가 비어 있으면 island를 연결하지 않음.
    {
        std::vector<body> bodies( 2 );

        bodies[0].bodyId = 0;
        bodies[0].type = bodyType::Dynamic;

        bodies[1].bodyId = 1;
        bodies[1].type = bodyType::Dynamic;

        std::vector<contactSim2> contacts( 1 );

        contacts[0].contactId = 0;
        contacts[0].bodyIdA = 0;
        contacts[0].bodyIdB = 1;
        contacts[0].manifold.pointCount = 0;

        const std::vector<island2> islands =
            BuildIslands(
                bodies,
                contacts
            );

        assert( islands.size() == 2 );
        assert( islands[0].contactIds.empty() );
        assert( islands[1].contactIds.empty() );
    }

    // speculative point도 실제 solver constraint이므로 island를 연결해야 함.
    {
        std::vector<body> bodies( 2 );

        bodies[0].bodyId = 0;
        bodies[0].type = bodyType::Dynamic;

        bodies[1].bodyId = 1;
        bodies[1].type = bodyType::Dynamic;

        std::vector<contactSim2> contacts( 1 );

        contacts[0].contactId = 0;
        contacts[0].bodyIdA = 0;
        contacts[0].bodyIdB = 1;
        contacts[0].manifold.pointCount = 1;
        contacts[0].manifold.points[0].separation = 0.01f;

        const std::vector<island2> islands =
            BuildIslands(
                bodies,
                contacts
            );

        assert( islands.size() == 1 );
        assert( islands[0].bodyIds.size() == 2 );
        assert( islands[0].contactIds.size() == 1 );
    }

    return 0;
}
