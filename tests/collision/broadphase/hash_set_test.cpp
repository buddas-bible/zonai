#include <cassert>
#include <cstdint>

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

    return 0;
}
