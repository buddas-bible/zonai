#include "collision/shapeProxy2.h"

#include <cassert>
#include <type_traits>

namespace zonai
{

shapeProxy2 MakeShapeProxy(
    const shapeGeometry& geometry )
{
    return std::visit(
        []( const auto& localGeometry ) -> shapeProxy2
        {
            using Geometry =
                std::remove_cvref_t<decltype( localGeometry )>;

            shapeProxy2 proxy{};

            if constexpr( std::is_same_v<Geometry, std::monostate> )
            {
                assert( false );
            }
            else if constexpr( std::is_same_v<Geometry, circle2> )
            {
                proxy.points[0] = localGeometry.center;
                proxy.count = 1;
                proxy.radius = localGeometry.radius;
            }
            else if constexpr( std::is_same_v<Geometry, capsule2> )
            {
                proxy.points[0] = localGeometry.center1;
                proxy.points[1] = localGeometry.center2;
                proxy.count = 2;
                proxy.radius = localGeometry.radius;
            }
            else if constexpr( std::is_same_v<Geometry, segment2> )
            {
                proxy.points[0] = localGeometry.a;
                proxy.points[1] = localGeometry.b;
                proxy.count = 2;
            }
            else
            {
                assert( localGeometry.vertexCount > 0 );
                assert(
                    localGeometry.vertexCount <=
                    static_cast<int>( proxy.points.size() )
                );

                for( int i = 0;
                     i < localGeometry.vertexCount;
                     ++i )
                {
                    proxy.points[i] =
                        localGeometry.vertices[i];
                }

                proxy.count = localGeometry.vertexCount;
                proxy.radius = localGeometry.radius;
            }

            return proxy;
        },
        geometry
    );
}

} // namespace zonai
