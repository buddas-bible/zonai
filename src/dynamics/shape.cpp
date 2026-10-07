#include "dynamics/shape.h"

#include <cassert>
#include <type_traits>

namespace zonai
{

massData2 ComputeShapeMass( const shape& shape )
{
    if( shape.density == 0.0f ) return {};

    return std::visit(
        [&]( const auto& geometry ) -> massData2
        {
            using Geometry = std::remove_cvref_t<decltype( geometry )>;

            if constexpr( std::is_same_v<Geometry, std::monostate> )
            {
                assert( false );

                return {};
            }
            else
            {
                return ComputeMass( geometry, shape.density );
            }
        },
        shape.geometry );
}

} // namespace zonai
