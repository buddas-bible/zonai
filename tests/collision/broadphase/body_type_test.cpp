#include <cassert>
#include <cstddef>

#include "dynamics/bodyType.h"

using namespace zonai;

int main()
{
    static_assert( static_cast<std::size_t>( BodyType::Static ) == 0 );
    static_assert( static_cast<std::size_t>( BodyType::Kinematic ) == 1 );
    static_assert( static_cast<std::size_t>( BodyType::Dynamic ) == 2 );
    static_assert( static_cast<std::size_t>( BodyType::Count ) == 3 );

    return 0;
}
