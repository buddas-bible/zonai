#include <cassert>
#include <cmath>

#include "math/vec2.h"
#include "math/rot2.h"

using namespace zonai;

bool NearlyEqual(float a, float b, float epsilon = 1e-5f)
{
    return std::fabs(a - b) <= epsilon;
}

int main()
{
    {
        // identity
        rot2 identity{};
        vec2 v{ 1.0f, 0.0f };

        vec2 same = Rotate(identity, v);
        assert(NearlyEqual(same.x, 1.0f));
        assert(NearlyEqual(same.y, 0.0f));

        // +90 degrees
        rot2 r = rot2::FromRadians(3.1415926535f * 0.5f);

        vec2 up = Rotate(r, { 1.0f, 0.0f });
        assert(NearlyEqual(up.x, 0.0f));
        assert(NearlyEqual(up.y, 1.0f));

        // inverse
        vec2 original = InverseRotate(r, up);
        assert(NearlyEqual(original.x, 1.0f));
        assert(NearlyEqual(original.y, 0.0f));

        // composition: 90 + 90 = 180
        rot2 r180 = r * r;

        vec2 left = Rotate(r180, { 1.0f, 0.0f });
        assert(NearlyEqual(left.x, -1.0f));
        assert(NearlyEqual(left.y, 0.0f));
    }

    return 0;
}