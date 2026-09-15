#include "geometry/polygon2.h"

#include <algorithm>

namespace zonai
{
    aabb2 ComputeAABB(const polygon2& polygon)
    {
        if (polygon.vertices.empty())
        {
            return {};
        }

        vec2 min = polygon.vertices[0];
        vec2 max = polygon.vertices[0];

        for (const vec2& vertex : polygon.vertices)
        {
            min.x = std::min(min.x, vertex.x);
            min.y = std::min(min.y, vertex.y);

            max.x = std::max(max.x, vertex.x);
            max.y = std::max(max.y, vertex.y);
        }

        return { min, max };
    }
}