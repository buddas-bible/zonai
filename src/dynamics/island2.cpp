#include "dynamics/island2.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace zonai
{
namespace
{

std::int32_t FindRoot(
    std::vector<std::int32_t>& parents,
    std::int32_t bodyId )
{
    assert( bodyId >= 0 );
    assert( static_cast<std::size_t>( bodyId ) < parents.size() );
    assert( parents[bodyId] != body::NULL_INDEX );

    std::int32_t root = bodyId;

    while( parents[root] != root )
    {
        root = parents[root];
    }

    while( parents[bodyId] != bodyId )
    {
        const std::int32_t parent = parents[bodyId];
        parents[bodyId] = root;
        bodyId = parent;
    }

    return root;
}

void UnionBodies(
    std::vector<std::int32_t>& parents,
    std::vector<std::uint8_t>& ranks,
    std::int32_t bodyIdA,
    std::int32_t bodyIdB )
{
    std::int32_t rootA =
        FindRoot( parents, bodyIdA );

    std::int32_t rootB =
        FindRoot( parents, bodyIdB );

    if( rootA == rootB )
    {
        return;
    }

    if( ranks[rootA] < ranks[rootB] )
    {
        parents[rootA] = rootB;
    }
    else if( ranks[rootA] > ranks[rootB] )
    {
        parents[rootB] = rootA;
    }
    else
    {
        parents[rootB] = rootA;
        ++ranks[rootA];
    }
}

} // namespace

islandGraph2 BuildIslands(
    std::span<const body> bodies,
    std::span<const contactSim2> contactSims,
    std::span<const joint2> jointSims )
{
    std::vector<std::int32_t> parents(
        bodies.size(),
        body::NULL_INDEX
    );

    std::vector<std::uint8_t> ranks(
        bodies.size(),
        0
    );

    std::size_t awakeBodyCount = 0;

    // Static / sleeping body는 이번 solver island에 들어가지 않음.
    // Awake Dynamic / Kinematic body는 Contact가 없어도 독립 island 하나를 가짐.
    for( std::int32_t bodyId = 0;
         bodyId < static_cast<std::int32_t>( bodies.size() );
         ++bodyId )
    {
        const body& currentBody =
            bodies[bodyId];

        if( currentBody.bodyId == body::NULL_INDEX ||
            currentBody.type == bodyType::Static ||
            !currentBody.awake )
        {
            continue;
        }

        assert( currentBody.bodyId == bodyId );

        parents[bodyId] = bodyId;
        ++awakeBodyCount;
    }

    /*
    * solver-active Contact를 graph edge로 사용함.
    *
    * pointCount == 0:
    *     broadPhase pair만 유지되는 non-touching Contact이므로 연결하지 않음.
    *
    * pointCount > 0:
    *     penetration / touching / speculative point 모두 solver constraint가
    *     존재하므로 같은 island에서 풀어야 함.
    *
    * Static body는 graph node가 아니므로 non-static body끼리만 union함.
    */
    for( const contactSim2& contactSim : contactSims )
    {
        if( contactSim.contactId == contactSim2::NULL_INDEX ||
            contactSim.manifold.pointCount == 0 )
        {
            continue;
        }

        assert( contactSim.bodyIdA >= 0 );
        assert( contactSim.bodyIdB >= 0 );
        assert( static_cast<std::size_t>( contactSim.bodyIdA ) < bodies.size() );
        assert( static_cast<std::size_t>( contactSim.bodyIdB ) < bodies.size() );

        const body& bodyA =
            bodies[contactSim.bodyIdA];

        const body& bodyB =
            bodies[contactSim.bodyIdB];

        if( bodyA.type != bodyType::Static &&
            bodyB.type != bodyType::Static )
        {
            // active Contact가 awake / sleeping 경계를 가로지르면 안 됨.
            assert( bodyA.awake == bodyB.awake );
        }

        const bool hasBodyA =
            parents[contactSim.bodyIdA] != body::NULL_INDEX;

        const bool hasBodyB =
            parents[contactSim.bodyIdB] != body::NULL_INDEX;

        if( hasBodyA && hasBodyB )
        {
            UnionBodies(
                parents,
                ranks,
                contactSim.bodyIdA,
                contactSim.bodyIdB
            );
        }
    }

    // Joint는 manifold 없이도 active edge임. Static은 공유 anchor여도 union하지 않음.
    for( const joint2& joint : jointSims )
    {
        if( joint.jointId == -1 ) { continue; }
        assert( joint.edges[0].bodyId >= 0 && static_cast<std::size_t>( joint.edges[0].bodyId ) < bodies.size() );
        assert( joint.edges[1].bodyId >= 0 && static_cast<std::size_t>( joint.edges[1].bodyId ) < bodies.size() );
        const body& a = bodies[joint.edges[0].bodyId]; const body& b = bodies[joint.edges[1].bodyId];
        assert( a.type == bodyType::Static || b.type == bodyType::Static || a.awake == b.awake );
        if( parents[joint.edges[0].bodyId] != -1 && parents[joint.edges[1].bodyId] != -1 )
        {
            UnionBodies( parents, ranks, joint.edges[0].bodyId, joint.edges[1].bodyId );
        }
    }

    std::vector<std::int32_t> islandIndices(
        bodies.size(),
        body::NULL_INDEX
    );

    islandGraph2 graph{};
    graph.islands.reserve( awakeBodyCount );

    // 먼저 island 수와 각 island의 body 개수를 결정함.
    for( std::int32_t bodyId = 0;
         bodyId < static_cast<std::int32_t>( bodies.size() );
         ++bodyId )
    {
        if( parents[bodyId] == body::NULL_INDEX )
        {
            continue;
        }

        const std::int32_t root =
            FindRoot( parents, bodyId );

        std::int32_t islandIndex =
            islandIndices[root];

        if( islandIndex == body::NULL_INDEX )
        {
            islandIndex =
                static_cast<std::int32_t>( graph.islands.size() );

            islandIndices[root] = islandIndex;
            graph.islands.push_back( {} );
        }

        ++graph.islands[islandIndex].bodyCount;
    }

    // Contact도 어느 island에 속하는지 먼저 세어 flat 배열 구간 크기를 확정함.
    for( const contactSim2& contactSim : contactSims )
    {
        if( contactSim.contactId == contactSim2::NULL_INDEX ||
            contactSim.manifold.pointCount == 0 )
        {
            continue;
        }

        const bool hasBodyA =
            parents[contactSim.bodyIdA] != body::NULL_INDEX;

        const bool hasBodyB =
            parents[contactSim.bodyIdB] != body::NULL_INDEX;

        // BroadPhase 구조상 static-static Contact는 생성되지 않지만
        // island builder 자체는 그런 입력도 안전하게 무시함.
        if( !hasBodyA && !hasBodyB )
        {
            continue;
        }

        const std::int32_t ownerBodyId =
            hasBodyA
                ? contactSim.bodyIdA
                : contactSim.bodyIdB;

        const std::int32_t root =
            FindRoot(
                parents,
                ownerBodyId
            );

        const std::int32_t islandIndex =
            islandIndices[root];

        assert( islandIndex >= 0 );
        assert( static_cast<std::size_t>( islandIndex ) < graph.islands.size() );

        ++graph.islands[islandIndex].contactCount;
    }

    for( const joint2& joint : jointSims )
    {
        if( joint.jointId == -1 ) { continue; }
        const std::int32_t owner = parents[joint.edges[0].bodyId] != -1 ? joint.edges[0].bodyId : joint.edges[1].bodyId;
        if( parents[owner] != -1 ) { ++graph.islands[islandIndices[FindRoot( parents, owner )]].jointCount; }
    }

    std::size_t totalJointCount = 0;
    std::size_t totalBodyCount = 0;
    std::size_t totalContactCount = 0;

    for( island2& island : graph.islands )
    {
        island.bodyStart = totalBodyCount;
        island.contactStart = totalContactCount;
        island.jointStart = totalJointCount;
        totalJointCount += island.jointCount;

        totalBodyCount += island.bodyCount;
        totalContactCount += island.contactCount;
    }

    assert( totalBodyCount == awakeBodyCount );

    graph.bodyIds.resize( totalBodyCount );
    graph.contactIds.resize( totalContactCount );
    graph.jointIds.resize( totalJointCount );
    std::vector<std::size_t> jointOffsets( graph.islands.size(), 0 );

    std::vector<std::size_t> bodyOffsets(
        graph.islands.size(),
        0
    );

    std::vector<std::size_t> contactOffsets(
        graph.islands.size(),
        0
    );

    // body index 순서대로 채워 island 내부 순서도 결정적으로 유지함.
    for( std::int32_t bodyId = 0;
         bodyId < static_cast<std::int32_t>( bodies.size() );
         ++bodyId )
    {
        if( parents[bodyId] == body::NULL_INDEX )
        {
            continue;
        }

        const std::int32_t root =
            FindRoot( parents, bodyId );

        const std::int32_t islandIndex =
            islandIndices[root];

        island2& island =
            graph.islands[islandIndex];

        const std::size_t writeIndex =
            island.bodyStart +
            bodyOffsets[islandIndex]++;

        assert( writeIndex < graph.bodyIds.size() );

        graph.bodyIds[writeIndex] =
            bodyId;
    }

    // Contact stable slot 순서대로 채워 solver constraint 순서도 결정적으로 유지함.
    for( const contactSim2& contactSim : contactSims )
    {
        if( contactSim.contactId == contactSim2::NULL_INDEX ||
            contactSim.manifold.pointCount == 0 )
        {
            continue;
        }

        const bool hasBodyA =
            parents[contactSim.bodyIdA] != body::NULL_INDEX;

        const bool hasBodyB =
            parents[contactSim.bodyIdB] != body::NULL_INDEX;

        if( !hasBodyA && !hasBodyB )
        {
            continue;
        }

        const std::int32_t ownerBodyId =
            hasBodyA
                ? contactSim.bodyIdA
                : contactSim.bodyIdB;

        const std::int32_t root =
            FindRoot(
                parents,
                ownerBodyId
            );

        const std::int32_t islandIndex =
            islandIndices[root];

        island2& island =
            graph.islands[islandIndex];

        const std::size_t writeIndex =
            island.contactStart +
            contactOffsets[islandIndex]++;

        assert( writeIndex < graph.contactIds.size() );

        graph.contactIds[writeIndex] =
            contactSim.contactId;
    }

    for( const joint2& joint : jointSims )
    {
        if( joint.jointId == -1 ) { continue; }
        const std::int32_t owner = parents[joint.edges[0].bodyId] != -1 ? joint.edges[0].bodyId : joint.edges[1].bodyId;
        if( parents[owner] == -1 ) { continue; }
        const std::int32_t islandIndex = islandIndices[FindRoot( parents, owner )];
        const island2& island = graph.islands[islandIndex];
        graph.jointIds[island.jointStart + jointOffsets[islandIndex]++] = joint.jointId;
    }
    return graph;
}

} // namespace zonai
