#include <cassert>
#include <cmath>

#include "geometry/circle2.h"

using namespace zonai;

bool NearlyEqual(float a, float b, float epsilon = 1e-5f)
{
    return std::fabs(a - b) <= epsilon;
}

int main()
{
    // ComputeAABB - centered at origin
    {
        circle2 circle{
            { 0.0f, 0.0f },
            2.0f
        };

        aabb2 box = ComputeAABB(circle);

        assert(NearlyEqual(box.min.x, -2.0f));
        assert(NearlyEqual(box.min.y, -2.0f));
        assert(NearlyEqual(box.max.x, 2.0f));
        assert(NearlyEqual(box.max.y, 2.0f));
    }

    // ComputeAABB - translated circle
    {
        circle2 circle{
            { 3.0f, -4.0f },
            1.5f
        };

        aabb2 box = ComputeAABB(circle);

        assert(NearlyEqual(box.min.x, 1.5f));
        assert(NearlyEqual(box.min.y, -5.5f));
        assert(NearlyEqual(box.max.x, 4.5f));
        assert(NearlyEqual(box.max.y, -2.5f));
    }

    // Contains - center point
    {
        circle2 circle{
            { 2.0f, 3.0f },
            5.0f
        };

        assert(Contains(circle, { 2.0f, 3.0f }));
    }

    // Contains - point inside
    {
        circle2 circle{
            { 0.0f, 0.0f },
            5.0f
        };

        assert(Contains(circle, { 3.0f, 4.0f }));
    }

    // Contains - point on boundary
    {
        circle2 circle{
            { 0.0f, 0.0f },
            5.0f
        };

        assert(Contains(circle, { 5.0f, 0.0f }));
        assert(Contains(circle, { 0.0f, -5.0f }));
    }

    // Contains - point outside
    {
        circle2 circle{
            { 0.0f, 0.0f },
            5.0f
        };

        assert(!Contains(circle, { 5.1f, 0.0f }));
        assert(!Contains(circle, { 4.0f, 4.0f }));
    }

    // Contains - translated circle
    {
        circle2 circle{
            { 10.0f, -3.0f },
            2.0f
        };

        assert(Contains(circle, { 11.0f, -3.0f }));
        assert(Contains(circle, { 10.0f, -1.0f }));
        assert(!Contains(circle, { 12.1f, -3.0f }));
    }

    // Zero-radius circle
    {
        circle2 circle{
            { 1.0f, 2.0f },
            0.0f
        };

        aabb2 box = ComputeAABB(circle);

        assert(NearlyEqual(box.min.x, 1.0f));
        assert(NearlyEqual(box.min.y, 2.0f));
        assert(NearlyEqual(box.max.x, 1.0f));
        assert(NearlyEqual(box.max.y, 2.0f));

        assert(Contains(circle, { 1.0f, 2.0f }));
        assert(!Contains(circle, { 1.001f, 2.0f }));
    }

    return 0;
}