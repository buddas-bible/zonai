#include "geometry/polygon2.h"

#include <algorithm>

namespace zonai
{
    polygon2 MakePolygon(std::span<const vec2> vertices)
    {
        polygon2 polygon{};

        if (vertices.size() < 3 || vertices.size() > maxPolygonVertices)
        {
            assert(false);
            return polygon;
        }

        polygon.vertexCount = vertices.size();

        for (std::size_t i = 0; i < polygon.vertexCount; ++i)
        {
            polygon.vertices[i] = vertices[i];
        }

        // TODO:
        // 1. winding 확인/교정
        // 2. normals 계산
        // 3. centroid 계산

        return polygon;
    }

    aabb2 ComputeAABB(const polygon2& polygon)
    {
        if (polygon.vertexCount == 0)
        {
            return {};
        }

        vec2 min = polygon.vertices[0];
        vec2 max = polygon.vertices[0];

        for (std::size_t i = 0; i < polygon.vertexCount; ++i)
        {
            const vec2& vertex = polygon.vertices[i];

            min.x = std::min(min.x, vertex.x);
            min.y = std::min(min.y, vertex.y);

            max.x = std::max(max.x, vertex.x);
            max.y = std::max(max.y, vertex.y);
        }

        return { min, max };
    }
}