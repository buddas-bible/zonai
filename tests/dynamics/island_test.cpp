#include <cstdio>
#include <cstdlib>
#include <source_location>
#include <cstdint>
#include <span>
#include <vector>

#include "dynamics/island2.h"

using namespace zonai;

namespace
{

// Release에서도 island의 flat 구간과 connectivity 검사를 실행함.
void check( bool condition, const std::source_location& location = std::source_location::current() )
{
    if( !condition )
    {
        std::fprintf( stderr, "%s:%u: island check failed\n", location.file_name(), location.line() );
        std::exit( EXIT_FAILURE );
    }
}

std::span<const std::int32_t> GetBodyIds(
    const islandGraph2& graph,
    const island2& island )
{
    return
    {
        graph.bodyIds.data() + island.bodyStart,
        island.bodyCount
    };
}

std::span<const std::int32_t> GetContactIds(
    const islandGraph2& graph,
    const island2& island )
{
    return
    {
        graph.contactIds.data() + island.contactStart,
        island.contactCount
    };
}

} // namespace

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

        const islandGraph2 graph =
            BuildIslands(
                bodies,
                std::span<const contactSim2>{}
            );

        check( graph.islands.size() == 2 );
        check( graph.bodyIds.size() == 2 );
        check( graph.contactIds.empty() );

        const std::span<const std::int32_t> bodies0 =
            GetBodyIds( graph, graph.islands[0] );

        const std::span<const std::int32_t> bodies1 =
            GetBodyIds( graph, graph.islands[1] );

        check( bodies0.size() == 1 );
        check( bodies0[0] == 0 );

        check( bodies1.size() == 1 );
        check( bodies1[0] == 1 );
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

        const islandGraph2 graph =
            BuildIslands(
                bodies,
                contacts
            );

        check( graph.islands.size() == 1 );

        const std::span<const std::int32_t> bodyIds =
            GetBodyIds( graph, graph.islands[0] );

        const std::span<const std::int32_t> contactIds =
            GetContactIds( graph, graph.islands[0] );

        check( bodyIds.size() == 2 );
        check( bodyIds[0] == 0 );
        check( bodyIds[1] == 1 );

        check( contactIds.size() == 1 );
        check( contactIds[0] == 0 );
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

        const islandGraph2 graph =
            BuildIslands(
                bodies,
                contacts
            );

        check( graph.islands.size() == 1 );

        const std::span<const std::int32_t> bodyIds =
            GetBodyIds( graph, graph.islands[0] );

        const std::span<const std::int32_t> contactIds =
            GetContactIds( graph, graph.islands[0] );

        check( bodyIds.size() == 1 );
        check( bodyIds[0] == 1 );

        check( contactIds.size() == 1 );
        check( contactIds[0] == 0 );
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

        const islandGraph2 graph =
            BuildIslands(
                bodies,
                contacts
            );

        check( graph.islands.size() == 2 );
        check( graph.contactIds.empty() );
    }

    // sleeping body는 이번 solver island graph에서 제외됨.
    {
        std::vector<body> bodies( 2 );

        bodies[0].bodyId = 0;
        bodies[0].type = bodyType::Dynamic;
        bodies[0].awake = false;

        bodies[1].bodyId = 1;
        bodies[1].type = bodyType::Dynamic;
        bodies[1].awake = true;

        const islandGraph2 graph =
            BuildIslands(
                bodies,
                std::span<const contactSim2>{}
            );

        check( graph.islands.size() == 1 );
        check( graph.bodyIds.size() == 1 );
        check( graph.bodyIds[0] == 1 );
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

        const islandGraph2 graph =
            BuildIslands(
                bodies,
                contacts
            );

        check( graph.islands.size() == 1 );

        const std::span<const std::int32_t> bodyIds =
            GetBodyIds( graph, graph.islands[0] );

        const std::span<const std::int32_t> contactIds =
            GetContactIds( graph, graph.islands[0] );

        check( bodyIds.size() == 2 );
        check( contactIds.size() == 1 );
    }

    return 0;
}
