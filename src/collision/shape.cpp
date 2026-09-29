#include "collision/shape.h"

#include <cassert>
#include <type_traits>

#include "collision/massData2.h"

namespace zonai
{

aabb2 ComputeShapeAABB(
    const ShapeGeometry& geometry,
    const transform2& transform )
{
    return std::visit(
        [&]( const auto& localGeometry ) -> aabb2
        {
            using Geometry = std::remove_cvref_t<decltype( localGeometry )>;

            if constexpr( std::is_same_v<Geometry, std::monostate> )
            {
                assert( false );
                return {};
            }
            else if constexpr( std::is_same_v<Geometry, circle2> )
            {
                circle2 worldGeometry = localGeometry;
                worldGeometry.center =
                    TransformPoint( transform, localGeometry.center );

                return ComputeAABB( worldGeometry );
            }
            else if constexpr( std::is_same_v<Geometry, capsule2> )
            {
                capsule2 worldGeometry = localGeometry;
                worldGeometry.center1 =
                    TransformPoint( transform, localGeometry.center1 );
                worldGeometry.center2 =
                    TransformPoint( transform, localGeometry.center2 );

                return ComputeAABB( worldGeometry );
            }
            else if constexpr( std::is_same_v<Geometry, segment2> )
            {
                segment2 worldGeometry = localGeometry;
                worldGeometry.a =
                    TransformPoint( transform, localGeometry.a );
                worldGeometry.b =
                    TransformPoint( transform, localGeometry.b );

                return ComputeAABB( worldGeometry );
            }
            else
            {
                polygon2 worldGeometry = localGeometry;

                for( int i = 0; i < localGeometry.vertexCount; ++i )
                {
                    worldGeometry.vertices[i] =
                        TransformPoint( transform, localGeometry.vertices[i] );
                }

                return ComputeAABB( worldGeometry );
            }
        },
        geometry
    );
}

massData2 ComputeShapeMass( const Shape& shape )
{
    if( shape.density == 0.0f )
    {
        return {};
    }

    return std::visit(
        [&]( const auto& geometry ) -> massData2
        {
            using Geometry =
                std::remove_cvref_t<decltype( geometry )>;

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
        shape.geometry
    );
}

} // namespace zonai
