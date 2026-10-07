#include "collision/shapeGeometry.h"

#include "collision/constants.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <type_traits>

namespace zonai
{

aabb2 ComputeShapeAABB( const shapeGeometry& geometry, const transform2& transform )
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
                worldGeometry.center = TransformPoint( transform, localGeometry.center );

                return ComputeAABB( worldGeometry );
            }
            else if constexpr( std::is_same_v<Geometry, capsule2> )
            {
                capsule2 worldGeometry = localGeometry;
                worldGeometry.center1 = TransformPoint( transform, localGeometry.center1 );
                worldGeometry.center2 = TransformPoint( transform, localGeometry.center2 );

                return ComputeAABB( worldGeometry );
            }
            else if constexpr( std::is_same_v<Geometry, segment2> )
            {
                segment2 worldGeometry = localGeometry;
                worldGeometry.a = TransformPoint( transform, localGeometry.a );
                worldGeometry.b = TransformPoint( transform, localGeometry.b );

                return ComputeAABB( worldGeometry );
            }
            else
            {
                polygon2 worldGeometry = localGeometry;

                for( int i = 0; i < localGeometry.vertexCount; ++i )
                {
                    worldGeometry.vertices[i] = TransformPoint( transform, localGeometry.vertices[i] );
                }

                return ComputeAABB( worldGeometry );
            }
        },
        geometry );
}

float ComputeShapeAABBMargin( const shapeGeometry& geometry )
{
    const float shapeExtent = std::visit(
        []( const auto& localGeometry ) -> float
        {
            using Geometry = std::remove_cvref_t<decltype( localGeometry )>;

            if constexpr( std::is_same_v<Geometry, std::monostate> )
            {
                assert( false );

                return 0.0f;
            }
            else if constexpr( std::is_same_v<Geometry, circle2> )
            {
                return localGeometry.radius;
            }
            else if constexpr( std::is_same_v<Geometry, capsule2> )
            {
                return 0.5f * Length( localGeometry.center2 - localGeometry.center1 ) + localGeometry.radius;
            }
            else if constexpr( std::is_same_v<Geometry, segment2> )
            {
                return 0.5f * Length( localGeometry.b - localGeometry.a );
            }
            else
            {
                float maxExtentSquared = 0.0f;

                for( int i = 0; i < localGeometry.vertexCount; ++i )
                {
                    maxExtentSquared = std::max( maxExtentSquared, LengthSquared( localGeometry.vertices[i] - localGeometry.centroid ) );
                }

                return std::sqrt( maxExtentSquared );
            }
        },
        geometry );

    return std::min( MAX_AABB_MARGIN, AABB_MARGIN_FRACTION * shapeExtent );
}

shapeExtent2 ComputeShapeExtent( const shapeGeometry& geometry, const vec2& localCenter )
{
    return std::visit(
        [&]( const auto& localGeometry ) -> shapeExtent2
        {
            using Geometry = std::remove_cvref_t<decltype( localGeometry )>;

            if constexpr( std::is_same_v<Geometry, std::monostate> )
            {
                assert( false );

                return {};
            }
            else if constexpr( std::is_same_v<Geometry, circle2> )
            {
                return { localGeometry.radius, Length( localGeometry.center - localCenter ) + localGeometry.radius };
            }
            else if constexpr( std::is_same_v<Geometry, capsule2> )
            {
                const float distance1 = LengthSquared( localGeometry.center1 - localCenter );

                const float distance2 = LengthSquared( localGeometry.center2 - localCenter );

                return { localGeometry.radius, std::sqrt( std::max( distance1, distance2 ) ) + localGeometry.radius };
            }
            else if constexpr( std::is_same_v<Geometry, segment2> )
            {
                const float distance1 = LengthSquared( localGeometry.a - localCenter );

                const float distance2 = LengthSquared( localGeometry.b - localCenter );

                return { 0.0f, std::sqrt( std::max( distance1, distance2 ) ) };
            }
            else
            {
                assert( localGeometry.vertexCount > 0 );

                float minExtent = std::numeric_limits<float>::max();

                float maxExtentSquared = 0.0f;

                for( int i = 0; i < localGeometry.vertexCount; ++i )
                {
                    const vec2 vertex = localGeometry.vertices[i];

                    const float planeOffset = Dot( localGeometry.normals[i], vertex - localGeometry.centroid );

                    minExtent = std::min( minExtent, planeOffset );

                    maxExtentSquared = std::max( maxExtentSquared, LengthSquared( vertex - localCenter ) );
                }

                return { minExtent + localGeometry.radius, std::sqrt( maxExtentSquared ) + localGeometry.radius };
            }
        },
        geometry );
}

} // namespace zonai
