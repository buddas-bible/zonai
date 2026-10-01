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

std::vector<island2> BuildIslands(
    std::span<const body> bodies,
    std::span<const contactSim2> contactSims )
{
    std::vector<std::int32_t> parents(
        bodies.size(),
        body::NULL_INDEX
    );

    std::vector<std::uint8_t> ranks(
        bodies.size(),
        0
    );

    // Static body는 solver island에 들어가지 않음.
    // Dynamic / Kinematic body는 Contact가 없어도 독립 island 하나를 가짐.
    for( std::int32_t bodyId = 0;
         bodyId < static_cast<std::int32_t>( bodies.size() );
         ++bodyId )
    {
        const body& currentBody =
            bodies[bodyId];

        if( currentBody.bodyId == body::NULL_INDEX ||
            currentBody.type == bodyType::Static )
        {
            continue;
        }

        assert( currentBody.bodyId == bodyId );
        parents[bodyId] = bodyId;
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

    std::vector<std::int32_t> islandIndices(
        bodies.size(),
        body::NULL_INDEX
    );

    std::vector<island2> islands;

    // body index 순서대로 island와 body 목록을 만들어 결과 순서를 결정적으로 유지함.
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
                static_cast<std::int32_t>( islands.size() );

            islandIndices[root] = islandIndex;
            islands.push_back( {} );
        }

        islands[islandIndex].bodyIds.push_back(
            bodyId
        );
    }

    // 각 solver-active Contact를 두 non-static endpoint가 속한 island에 한 번만 추가함.
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
        assert( static_cast<std::size_t>( islandIndex ) < islands.size() );

        islands[islandIndex].contactIds.push_back(
            contactSim.contactId
        );
    }

    return islands;
}

} // namespace zonai
