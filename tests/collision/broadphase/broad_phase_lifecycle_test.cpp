#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "collision/broadphase/broadPhase.h"

using namespace zonai;

namespace
{

#pragma region ReferenceChecks

std::uint32_t randomState = 0;
int currentRound = 0;

// assert는 Release에서 사라지므로 reference 비교와 side effect를 항상 실행함.
void check( bool condition, const char* message )
{
    if( !condition )
    {
        std::fprintf( stderr, "round=%d state=%u: %s\n", currentRound, randomState, message );
        std::exit( EXIT_FAILURE );
    }
}

std::uint32_t nextRandom()
{
    randomState = 1664525u * randomState + 1013904223u;
    return randomState;
}

aabb2 randomBox()
{
    const float x = static_cast<float>( nextRandom() % 81 ) - 40.0f;
    const float y = static_cast<float>( nextRandom() % 81 ) - 40.0f;
    const float halfExtent = 0.25f + static_cast<float>( nextRandom() % 16 );
    return { { x - halfExtent, y - halfExtent }, { x + halfExtent, y + halfExtent } };
}

struct proxyRecord
{
    proxyKey key = -1;
    bodyType type = bodyType::Static;
    aabb2 box{};
    bool moved = false;
};

void checkQueries( const broadPhase& phase, const std::vector<proxyRecord>& records, const aabb2& query )
{
    for( int typeIndex = 0; typeIndex < static_cast<int>( bodyType::Count ); ++typeIndex )
    {
        const bodyType type = static_cast<bodyType>( typeIndex );
        const dynamicTree& tree = phase.GetTree( type );
        check( tree.Validate(), "tree invariant" );
        std::vector<std::int32_t> actual, expected;
        std::size_t liveCount = 0;

        tree.Query( query, [&]( std::int32_t proxyId )
        {
            actual.push_back( tree.GetProxyShapeIndex( proxyId ) );
            return true;
        } );

        for( std::size_t i = 0; i < records.size(); ++i )
        {
            const proxyRecord& record = records[i];
            if( record.key == -1 || record.type != type )
            {
                continue;
            }

            ++liveCount;
            const aabb2& stored = tree.GetProxyAABB( GetProxyId( record.key ) );
            check( ContainsAABB( stored, record.box ) && ContainsAABB( record.box, stored ), "proxy bounds" );
            check( tree.GetProxyShapeIndex( GetProxyId( record.key ) ) == static_cast<std::int32_t>( i ), "stable mapping" );
            if( Overlaps( record.box, query ) )
            {
                expected.push_back( static_cast<std::int32_t>( i ) );
            }
        }

        std::sort( actual.begin(), actual.end() );
        check( actual == expected, "query versus brute force" );
        check( tree.GetProxyCount() == liveCount, "live proxy count" );

        int callbackCount = 0;
        tree.Query( query, [&]( std::int32_t ) { ++callbackCount; return false; } );
        check( callbackCount == ( expected.empty() ? 0 : 1 ), "query early termination" );
    }
}

void checkPairs( broadPhase& phase, const std::vector<proxyRecord>& records, std::span<const shape> shapes )
{
    std::vector<shapePairKey> actual, expected;
    for( std::size_t i = 0; i < records.size(); ++i )
    {
        for( std::size_t j = i + 1; j < records.size(); ++j )
        {
            const proxyRecord& a = records[i];
            const proxyRecord& b = records[j];
            if( a.key == -1 || b.key == -1 || !( a.moved || b.moved ) ||
                ( a.type != bodyType::Dynamic && b.type != bodyType::Dynamic ) || !Overlaps( a.box, b.box ) )
            {
                continue;
            }

            const shapePairKey key = MakeShapePairKey( static_cast<std::int32_t>( i ), static_cast<std::int32_t>( j ) );
            if( phase.HasPair( key ) )
            {
                continue;
            }

            if( !shapes.empty() && ( shapes[i].bodyId == shapes[j].bodyId ||
                shapes[i].sensorIndex != shape::NULL_INDEX || shapes[j].sensorIndex != shape::NULL_INDEX ||
                !ShouldShapesCollide( shapes[i].filter, shapes[j].filter ) ) )
            {
                continue;
            }

            expected.push_back( key );
        }
    }

    auto collect = [&]( std::int32_t a, std::int32_t b ) { actual.push_back( MakeShapePairKey( a, b ) ); };
    // maintenance 전 두 overload가 같은 moved 상태를 읽게 함.
    phase.FindPairs( shapes, collect );
    std::sort( actual.begin(), actual.end() );
    check( actual == expected, "FindPairs versus brute force, including duplicates" );
    actual.clear();
    if( shapes.empty() )
    {
        phase.UpdatePairs( collect );
    }
    else
    {
        phase.UpdatePairs( shapes, collect );
    }

    std::sort( actual.begin(), actual.end() );
    check( actual == expected, "UpdatePairs versus brute force, including duplicates" );
    for( int type = 0; type < static_cast<int>( bodyType::Count ); ++type )
    {
        const dynamicTree& tree = phase.GetTree( static_cast<bodyType>( type ) );
        check( tree.Validate() && !tree.HasMoved(), "maintenance consumes moved flags" );
        check( type == static_cast<int>( bodyType::Static ) || !tree.NeedsRebuild(), "maintenance restores DFS order" );
    }
}

#pragma endregion

} // namespace

int main()
{
#pragma region MixedProxyLifecycle

    // Box2D의 pair 조건은 overlap && (movedA || movedB)임.
    // topology 변경과 retained subtree 복사를 섞어도 leaf 기준 reference와 같아야 함.
    for( const std::uint32_t seed : { 1u, 12345u, 0xC0FFEEu, 0xFFFFFFFFu } )
    {
        randomState = seed;
        broadPhase phase{};
        std::vector<proxyRecord> records( 192 );
        std::vector<shape> shapes( records.size() );
        const shapePairKey existingPair = MakeShapePairKey( 0, 1 );
        check( !phase.AddPair( existingPair ), "new contact pair" );
        check( phase.AddPair( existingPair ), "existing contact pair" );

        for( std::size_t i = 0; i < records.size(); ++i )
        {
            proxyRecord& record = records[i];
            record.type = static_cast<bodyType>( i % 3 );
            // 같은 center는 midpoint partition의 median fallback도 실행함.
            record.box = i < 96 ? aabb2{ { -1.0f, -1.0f }, { 1.0f, 1.0f } } : randomBox();
            record.key = phase.CreateProxy( record.type, record.box, static_cast<std::int32_t>( i ), true );
            record.moved = true;
            shapes[i].bodyId = static_cast<std::int32_t>( i / 2 );
            shapes[i].sensorIndex = i % 17 == 0 ? 0 : shape::NULL_INDEX;
            shapes[i].filter.categoryBits = std::uint64_t{ 1 } << ( i % 4 );
            shapes[i].filter.maskBits = i % 5 == 0 ? 1 : 15;
            shapes[i].filter.groupIndex = i % 7 == 0 ? -1 : ( i % 11 == 0 ? 1 : 0 );
        }

        for( currentRound = 0; currentRound < 80; ++currentRound )
        {
            for( int operation = 0; operation < 32 && currentRound > 0; ++operation )
            {
                const std::size_t index = ( nextRandom() >> 8 ) % records.size();
                proxyRecord& record = records[index];
                if( record.key == -1 )
                {
                    record.type = static_cast<bodyType>( nextRandom() % 3 );
                    record.box = randomBox();
                    record.key = phase.CreateProxy( record.type, record.box, static_cast<std::int32_t>( index ), true );
                    record.moved = true;
                    continue;
                }

                switch( ( nextRandom() >> 8 ) % 5 )
                {
                case 0:
                    phase.DestroyProxy( record.key );
                    record.key = -1;
                    record.moved = false;
                    break;
                case 1:
                    record.box = randomBox();
                    phase.MoveProxy( record.key, record.box );
                    record.moved = true;
                    break;
                case 2:
                    record.box = Union( record.box, randomBox() );
                    phase.EnlargeProxy( record.key, record.box );
                    record.moved = true;
                    break;
                case 3:
                    phase.TouchProxy( record.key );
                    record.moved = true;
                    break;
                case 4:
                    break;
                }
            }

            checkQueries( phase, records, randomBox() );
            checkPairs( phase, records, currentRound % 2 == 0 ? std::span<const shape>{} : std::span<const shape>{ shapes } );
            for( proxyRecord& record : records )
            {
                record.moved = false;
            }

            // partial rebuild 뒤 stable id / bounds / query 결과가 유지되어야 함.
            checkQueries( phase, records, { { -100.0f, -100.0f }, { 100.0f, 100.0f } } );
            checkPairs( phase, records, {} );
            if( currentRound % 9 == 0 )
            {
                for( int type = 0; type < 3; ++type )
                {
                    phase.GetTree( static_cast<bodyType>( type ) ).Rebuild( true );
                }
                checkQueries( phase, records, randomBox() );
            }
        }

        check( phase.RemovePair( existingPair ), "remove contact pair" );
        for( proxyRecord& record : records )
        {
            if( record.key != -1 )
            {
                phase.DestroyProxy( record.key );
                record.key = -1;
            }
        }
        checkQueries( phase, records, { { -1.0f, -1.0f }, { 1.0f, 1.0f } } );
        checkPairs( phase, records, {} );
    }

#pragma endregion
    return 0;
}
