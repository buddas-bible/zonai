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

    return 0;
}
