#include <cassert>
#include <cmath>

#include "math/vec2.h"

using namespace zonai;

bool NearlyEqual(float a, float b, float epsilon = 1e-5f)
{
    return std::fabs(a - b) <= epsilon;
}

int main()
{
    {
        vec2 a{ 1.0f, 2.0f };
        vec2 b{ 3.0f, 4.0f };

        vec2 result = a + b;

        assert(result.x == 4.0f);
        assert(result.y == 6.0f);
    }

    {
        vec2 a{ 4.0f, 6.0f };
        vec2 b{ 1.0f, 2.0f };

        vec2 result = a - b;

        assert(result.x == 3.0f);
        assert(result.y == 4.0f);
    }

    {
        vec2 v{ 3.0f, 4.0f };

        assert(NearlyEqual(Length(v), 5.0f));
    }

    {
        vec2 a{ 1.0f, 2.0f };
        vec2 b{ 3.0f, 4.0f };

        assert(NearlyEqual(Dot(a, b), 11.0f));
        assert(NearlyEqual(Cross(a, b), -2.0f));
    }

    {
        vec2 v{ 3.0f, 4.0f };

        vec2 normalized = Normalize(v);

        assert(NearlyEqual(normalized.x, 0.6f));
        assert(NearlyEqual(normalized.y, 0.8f));
        assert(NearlyEqual(Length(normalized), 1.0f));
    }

    {
        vec2 zero{};

        vec2 normalized = Normalize(zero);

        assert(normalized.x == 0.0f);
        assert(normalized.y == 0.0f);
    }

    return 0;
}