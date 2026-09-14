#include <cassert>
#include <cmath>

#include "math/vec2.h"
#include "math/rot2.h"
#include "math/transform2.h"

using namespace zonai;

bool NearlyEqual(float a, float b, float epsilon = 1e-5f)
{
    return std::fabs(a - b) <= epsilon;
}

int main()
{
    {
        // identity
        transform2 t{};

        vec2 p{ 2.0f, 3.0f };

        vec2 result = TransformPoint(t, p);

        assert(NearlyEqual(result.x, 2.0f));
        assert(NearlyEqual(result.y, 3.0f));
    }

    {
        // translation
        transform2 t{};
        t.position = { 10.0f, 5.0f };

        vec2 result = TransformPoint(t, { 1.0f, 2.0f });

        assert(NearlyEqual(result.x, 11.0f));
        assert(NearlyEqual(result.y, 7.0f));
    }

    {
        // translation + rotation
        transform2 t{};
        t.position = { 10.0f, 5.0f };
        t.rotation = rot2::FromRadians(3.1415926535f * 0.5f);

        vec2 result = TransformPoint(t, { 1.0f, 0.0f });

        assert(NearlyEqual(result.x, 10.0f));
        assert(NearlyEqual(result.y, 6.0f));
    }

    {
        transform2 t{};
        t.position = { 10.0f, -3.0f };
        t.rotation = rot2::FromRadians(0.7f);

        vec2 original{ 2.0f, 5.0f };

        vec2 world = TransformPoint(t, original);
        vec2 local = InverseTransformPoint(t, world);

        assert(NearlyEqual(local.x, original.x));
        assert(NearlyEqual(local.y, original.y));
    }

    return 0;
}