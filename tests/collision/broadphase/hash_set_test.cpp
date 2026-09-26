#include <cassert>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include "collision/broadphase/hashSet.h"

using namespace zonai;

int main()
{
    HashSet set{ 1 };

    assert( set.GetCount() == 0 );
    assert( set.GetCapacity() == 16 );

    assert( set.Add( 42 ) == false );
    assert( set.Add( 123 ) == false );
    assert( set.GetCount() == 2 );

    assert( set.Add( 42 ) );
    assert( set.GetCount() == 2 );

    assert( set.Contains( 42 ) );
    assert( set.Contains( 123 ) );
    assert( set.Contains( 999 ) == false );

    for( std::uint64_t key = 1; key <= 16; ++key )
    {
        set.Add( 1000 + key );
    }

    assert( set.GetCapacity() >= 32 );
    assert( set.Contains( 42 ) );
    assert( set.Contains( 123 ) );

    for( std::uint64_t key = 1; key <= 16; ++key )
    {
        assert( set.Contains( 1000 + key ) );
    }

    HashSet removalSet{};

    // 1, 14, 27은 capacity 16에서 같은 initial slot을 사용해 probe chain을 만듦.
    removalSet.Add( 1 );
    removalSet.Add( 14 );
    removalSet.Add( 27 );

    assert( removalSet.GetCount() == 3 );

    assert( removalSet.Remove( 1 ) );
    assert( removalSet.GetCount() == 2 );
    assert( removalSet.Contains( 1 ) == false );
    assert( removalSet.Contains( 14 ) );
    assert( removalSet.Contains( 27 ) );

    assert( removalSet.Remove( 14 ) );
    assert( removalSet.GetCount() == 1 );
    assert( removalSet.Contains( 14 ) == false );
    assert( removalSet.Contains( 27 ) );

    assert( removalSet.Remove( 999 ) == false );
    assert( removalSet.GetCount() == 1 );

    {
        // Box2D와 같은 큰 key / growth / removal stress.
        HashSet stressSet{ 32 };

        constexpr std::size_t TEST_SIZE = 1000;
        std::vector<std::uint64_t> keys;
        keys.reserve( TEST_SIZE );

        for( std::size_t i = 0; i < TEST_SIZE; ++i )
        {
            const std::uint64_t key =
                static_cast<std::uint64_t>( i * 7 + 13 );

            keys.push_back( key );
            assert( stressSet.Add( key ) == false );
        }

        assert( stressSet.GetCount() == TEST_SIZE );

        for( const std::uint64_t key : keys )
        {
            assert( stressSet.Contains( key ) );
        }

        for( std::size_t i = 0; i < TEST_SIZE; i += 2 )
        {
            assert( stressSet.Remove( keys[i] ) );
        }

        assert( stressSet.GetCount() == TEST_SIZE / 2 );

        for( std::size_t i = 0; i < TEST_SIZE; ++i )
        {
            assert( stressSet.Contains( keys[i] ) == ( ( i & 1u ) != 0 ) );
        }

        const std::uint64_t largeKey = 0xFFFFFFFFFFFFFFFEull;
        assert( stressSet.Add( largeKey ) == false );
        assert( stressSet.Contains( largeKey ) );
    }

    {
        // custom HashSet을 std::unordered_set과 같은 연산열로 대조함.
        HashSet customSet{ 16 };
        std::unordered_set<std::uint64_t> referenceSet;

        std::uint32_t state = 0x12345678u;

        for( int step = 0; step < 10000; ++step )
        {
            state = state * 1664525u + 1013904223u;

            const std::uint64_t key =
                ( static_cast<std::uint64_t>( state ) << 32 ) |
                static_cast<std::uint64_t>( step + 1 );

            // key 0은 sentinel이므로 생성식 자체가 항상 non-zero여야 함.
            assert( key != 0 );

            switch( step % 3 )
            {
            case 0:
            {
                const bool customFound = customSet.Add( key );
                const bool referenceFound =
                    !referenceSet.insert( key ).second;

                assert( customFound == referenceFound );
                break;
            }

            case 1:
            {
                const bool customRemoved = customSet.Remove( key );
                const bool referenceRemoved =
                    referenceSet.erase( key ) != 0;

                assert( customRemoved == referenceRemoved );
                break;
            }

            default:
                assert(
                    customSet.Contains( key ) ==
                    referenceSet.contains( key )
                );
                break;
            }
        }

        assert( customSet.GetCount() == referenceSet.size() );

        for( const std::uint64_t key : referenceSet )
        {
            assert( customSet.Contains( key ) );
        }
    }

    {
        // Shape pair key는 순서가 바뀌어도 같고 서로 다른 pair끼리는 충돌하지 않아야 함.
        HashSet pairSet{};

        constexpr std::int32_t N = 128;
        std::size_t expectedCount = 0;

        for( std::int32_t i = 0; i < N; ++i )
        {
            for( std::int32_t j = i + 1; j < N; ++j )
            {
                const std::uint64_t key =
                    ( static_cast<std::uint64_t>( i ) << 32 ) |
                    static_cast<std::uint32_t>( j );

                assert( key != 0 );
                assert( pairSet.Add( key ) == false );
                ++expectedCount;
            }
        }

        assert( pairSet.GetCount() == expectedCount );

        for( std::int32_t i = 0; i < N; ++i )
        {
            for( std::int32_t j = i + 1; j < N; ++j )
            {
                const std::uint64_t key =
                    ( static_cast<std::uint64_t>( i ) << 32 ) |
                    static_cast<std::uint32_t>( j );

                assert( pairSet.Contains( key ) );
            }
        }
    }

    return 0;
}
