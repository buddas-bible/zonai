#include <algorithm>
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

#include "collision/broadphase/broadPhase.h"
#include "collision/broadphase/dynamicTree.h"
#include "dynamics/bodyType.h"

using namespace zonai;

namespace
{

std::uint32_t randomState = 0xC0FFEEu;

float RandomFloat( float lower, float upper )
{
    randomState = 1664525u * randomState + 1013904223u;

    const float unit = static_cast<float>( randomState >> 8 ) * ( 1.0f / 16777216.0f );

    return lower + ( upper - lower ) * unit;
}

aabb2 RandomBox( float centerExtent, float maxHalfExtent )
{
    const float x = RandomFloat( -centerExtent, centerExtent );

    const float y = RandomFloat( -centerExtent, centerExtent );

    const float hx = RandomFloat( 0.25f, maxHalfExtent );

    const float hy = RandomFloat( 0.25f, maxHalfExtent );

    return { { x - hx, y - hy }, { x + hx, y + hy } };
}

void CheckQueryAgainstBruteForce( const dynamicTree& tree, const std::vector<std::int32_t>& proxyIds, const std::vector<aabb2>& boxes, const aabb2& query )
{
    std::vector<std::int32_t> actual;

    tree.Query( query,
        [&]( std::int32_t proxyId )
        {
            actual.push_back( proxyId );

            return true;
        } );

    std::sort( actual.begin(), actual.end() );

    std::vector<std::int32_t> expected;

    for( std::size_t i = 0; i < proxyIds.size(); ++i )
    {
        if( Overlaps( boxes[i], query ) )
        {
            expected.push_back( proxyIds[i] );
        }
    }

    std::sort( expected.begin(), expected.end() );

    assert( actual == expected );
}

bool CanGenerateBroadPhasePair( bodyType typeA, bodyType typeB )
{
    return typeA == bodyType::Dynamic || typeB == bodyType::Dynamic;
}

using pair = std::pair<std::int32_t, std::int32_t>;

std::vector<pair> BuildExpectedPairs( const std::vector<bodyType>& types, const std::vector<aabb2>& boxes, const std::vector<bool>* moved = nullptr )
{
    std::vector<pair> expected;

    for( std::int32_t i = 0; i < static_cast<std::int32_t>( boxes.size() ); ++i )
    {
        for( std::int32_t j = i + 1; j < static_cast<std::int32_t>( boxes.size() ); ++j )
        {
            if( !CanGenerateBroadPhasePair( types[i], types[j] ) ) continue;

            if( moved != nullptr && !( ( *moved )[i] || ( *moved )[j] ) ) continue;

            if( Overlaps( boxes[i], boxes[j] ) )
            {
                expected.emplace_back( i, j );
            }
        }
    }

    return expected;
}

} // namespace

int main()
{
    // Dynamic / Kinematic 이동 경로는 topology를 즉시 재삽입하지 않고
    // leaf bounds와 ancestor bounds만 갱신한 뒤 partial rebuild에서 정리함.
    // 여러 proxy를 반복 갱신해도 query 결과가 brute-force와 같아야 함.
    {
        constexpr std::int32_t PROXY_COUNT = 192;
        constexpr int ROUND_COUNT = 8;

        dynamicTree tree{};
        std::vector<std::int32_t> proxyIds;
        std::vector<aabb2> boxes;

        proxyIds.reserve( PROXY_COUNT );
        boxes.reserve( PROXY_COUNT );

        for( std::int32_t i = 0; i < PROXY_COUNT; ++i )
        {
            const aabb2 box = RandomBox( 50.0f, 2.0f );

            proxyIds.push_back( tree.CreateProxy( box, i ) );

            boxes.push_back( box );
        }

        tree.Rebuild( true );
        assert( tree.Validate() );

        for( int round = 0; round < ROUND_COUNT; ++round )
        {
            for( std::int32_t i = round % 3; i < PROXY_COUNT; i += 3 )
            {
                boxes[i] = RandomBox( 50.0f, 2.0f );

                tree.UpdateProxy( proxyIds[i], boxes[i] );
            }

            assert( tree.HasMoved() );
            assert( tree.NeedsRebuild() );
            assert( tree.Validate() );

            // Rebuild 전에도 ancestor bounds가 정확히 refit되어 query를 누락하면 안 됨.
            for( int queryIndex = 0; queryIndex < 48; ++queryIndex )
            {
                CheckQueryAgainstBruteForce( tree, proxyIds, boxes, RandomBox( 50.0f, 8.0f ) );
            }

            const std::size_t rebuiltCount = tree.Rebuild( false );

            assert( rebuiltCount > 0 );
            assert( !tree.HasMoved() );
            assert( !tree.NeedsRebuild() );
            assert( tree.Validate() );

            // retained subtree를 복사한 뒤에도 stable proxy mapping과 query 결과가 유지되어야 함.
            for( int queryIndex = 0; queryIndex < 48; ++queryIndex )
            {
                CheckQueryAgainstBruteForce( tree, proxyIds, boxes, RandomBox( 50.0f, 8.0f ) );
            }
        }
    }

    // 세 tree의 pair 탐색 결과를 brute-force와 비교함.
    // 허용되는 조합은 Dynamic-Self / Dynamic-Static / Dynamic-Kinematic뿐임.
    {
        constexpr std::int32_t PROXY_COUNT = 96;

        broadPhase phase{};
        std::vector<bodyType> types;
        std::vector<aabb2> boxes;
        std::vector<proxyKey> proxyKeys;

        types.reserve( PROXY_COUNT );
        boxes.reserve( PROXY_COUNT );
        proxyKeys.reserve( PROXY_COUNT );

        for( std::int32_t i = 0; i < PROXY_COUNT; ++i )
        {
            const bodyType type = static_cast<bodyType>( i % static_cast<std::int32_t>( bodyType::Count ) );

            const aabb2 box = RandomBox( 18.0f, 3.0f );

            types.push_back( type );
            boxes.push_back( box );

            // 초기 static proxy도 기존 Dynamic과 pair를 만들 수 있어야 하므로 moved를 강제로 표시함.
            proxyKeys.push_back( phase.CreateProxy( type, box, i, true ) );
        }

        std::vector<pair> actual;

        phase.UpdatePairs(
            [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
            {
                actual.emplace_back( shapeIndexA, shapeIndexB );
            } );

        std::sort( actual.begin(), actual.end() );

        const std::vector<pair> expected = BuildExpectedPairs( types, boxes );

        assert( actual == expected );
        assert( std::adjacent_find( actual.begin(), actual.end() ) == actual.end() );

        assert( !phase.GetTree( bodyType::Static ).HasMoved() );

        assert( !phase.GetTree( bodyType::Kinematic ).NeedsRebuild() );

        assert( !phase.GetTree( bodyType::Dynamic ).NeedsRebuild() );

        // 일부 proxy만 다시 이동시킨 뒤에는 moved proxy가 관여한 새 후보만 보고되어야 함.
        std::vector<bool> moved( PROXY_COUNT, false );

        for( std::int32_t i = 0; i < PROXY_COUNT; i += 4 )
        {
            boxes[i] = RandomBox( 18.0f, 3.0f );

            moved[i] = true;

            phase.MoveProxy( proxyKeys[i], boxes[i] );
        }

        actual.clear();

        phase.UpdatePairs(
            [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
            {
                actual.emplace_back( shapeIndexA, shapeIndexB );
            } );

        std::sort( actual.begin(), actual.end() );

        const std::vector<pair> movedExpected = BuildExpectedPairs( types, boxes, &moved );

        assert( actual == movedExpected );
        assert( std::adjacent_find( actual.begin(), actual.end() ) == actual.end() );

        assert( phase.GetTree( bodyType::Static ).Validate() );

        assert( phase.GetTree( bodyType::Kinematic ).Validate() );

        assert( phase.GetTree( bodyType::Dynamic ).Validate() );
    }

    return 0;
}
