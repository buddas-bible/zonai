#include <cassert>
#include <cstddef>

#include "dynamics/bodyType.h"

using namespace zonai;

int main()
{
    static_assert( static_cast<std::size_t>( bodyType::Static ) == 0 );
    static_assert( static_cast<std::size_t>( bodyType::Kinematic ) == 1 );
    static_assert( static_cast<std::size_t>( bodyType::Dynamic ) == 2 );
    static_assert( static_cast<std::size_t>( bodyType::Count ) == 3 );

    return 0;
}
