#include <cassert>
#include <cmath>

#include "math/vec2.h"
#include "collision/aabb2.h"

using namespace zonai;

bool NearlyEqual(float a, float b, float epsilon = 1e-5f)
{
    return std::fabs(a - b) <= epsilon;
}

int main()
{
    // Center
    {
        aabb2 box{
            { -2.0f, -4.0f },
            {  2.0f,  4.0f }
        };

        vec2 center = Center(box);

        assert(NearlyEqual(center.x, 0.0f));
        assert(NearlyEqual(center.y, 0.0f));
    }

    // Extents
    {
        aabb2 box{
            { -2.0f, -4.0f },
            {  2.0f,  4.0f }
        };

        vec2 extents = Extents(box);

        assert(NearlyEqual(extents.x, 2.0f));
        assert(NearlyEqual(extents.y, 4.0f));
    }

    // Point inside
    {
        aabb2 box{
            { -1.0f, -1.0f },
            {  1.0f,  1.0f }
        };

        assert(Contains(box, { 0.0f, 0.0f }));
        assert(Contains(box, { 0.5f, -0.5f }));
    }

    // Point outside
    {
        aabb2 box{
            { -1.0f, -1.0f },
            {  1.0f,  1.0f }
        };

        assert(!Contains(box, { 2.0f, 0.0f }));
        assert(!Contains(box, { 0.0f, -2.0f }));
    }

    // Point on boundary
    {
        aabb2 box{
            { -1.0f, -1.0f },
            {  1.0f,  1.0f }
        };

        assert(Contains(box, { 1.0f, 0.0f }));
        assert(Contains(box, { -1.0f, -1.0f }));
    }

    // Overlapping boxes
    {
        aabb2 a{
            { -1.0f, -1.0f },
            {  1.0f,  1.0f }
        };

        aabb2 b{
            { 0.5f, 0.5f },
            { 2.0f, 2.0f }
        };

        assert(Overlaps(a, b));
        assert(Overlaps(b, a));
    }

    // Completely separated on X axis
    {
        aabb2 a{
            { -1.0f, -1.0f },
            {  1.0f,  1.0f }
        };

        aabb2 b{
            { 2.0f, -1.0f },
            { 3.0f,  1.0f }
        };

        assert(!Overlaps(a, b));
        assert(!Overlaps(b, a));
    }

    // Completely separated on Y axis
    {
        aabb2 a{
            { -1.0f, -1.0f },
            {  1.0f,  1.0f }
        };

        aabb2 b{
            { -1.0f, 2.0f },
            {  1.0f, 3.0f }
        };

        assert(!Overlaps(a, b));
        assert(!Overlaps(b, a));
    }

    // Edge touching counts as overlap
    {
        aabb2 a{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        aabb2 b{
            { 1.0f, 0.0f },
            { 2.0f, 1.0f }
        };

        assert(Overlaps(a, b));
        assert(Overlaps(b, a));
    }

    // Corner touching also counts as overlap
    {
        aabb2 a{
            { 0.0f, 0.0f },
            { 1.0f, 1.0f }
        };

        aabb2 b{
            { 1.0f, 1.0f },
            { 2.0f, 2.0f }
        };

        assert(Overlaps(a, b));
    }

    // One box fully contains another
    {
        aabb2 outer{
            { -10.0f, -10.0f },
            {  10.0f,  10.0f }
        };

        aabb2 inner{
            { -1.0f, -2.0f },
            {  3.0f,  4.0f }
        };

        assert(Overlaps(outer, inner));
        assert(Overlaps(inner, outer));
    }

    return 0;
}