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

bool NearlyEqual(
    const vec2& a,
    const vec2& b,
    float epsilon = 1e-5f )
{
    return NearlyEqual( a.x, b.x, epsilon ) &&
        NearlyEqual( a.y, b.y, epsilon );
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

    // inverse
    {
        transform2 t{};
        t.position = { 10.0f, -3.0f };
        t.rotation = rot2::FromRadians( 0.7f );

        const transform2 inverse = Inverse( t );

        const vec2 point{ 2.0f, 5.0f };

        const vec2 world =
            TransformPoint( t, point );

        const vec2 restored =
            TransformPoint( inverse, world );

        assert( NearlyEqual( restored, point ) );
    }

    // mul
    {
        transform2 a{};
        a.position = { 3.0f, 4.0f };
        a.rotation = rot2::FromRadians( 0.5f );

        transform2 b{};
        b.position = { -2.0f, 1.0f };
        b.rotation = rot2::FromRadians( -0.3f );

        const vec2 point{ 2.0f, 3.0f };

        const vec2 expected =
            TransformPoint(
                a,
                TransformPoint( b, point )
            );

        const transform2 combined =
            Mul( a, b );

        const vec2 actual =
            TransformPoint( combined, point );

        assert( NearlyEqual( actual, expected ) );
    }

    // inverse mul
    {
        transform2 transformA{};
        transformA.position = { 4.0f, 2.0f };
        transformA.rotation =
            rot2::FromRadians( 0.7f );

        transform2 transformB{};
        transformB.position = { -3.0f, 6.0f };
        transformB.rotation =
            rot2::FromRadians( -0.4f );

        const vec2 pointB{ 1.0f, 2.0f };

        const vec2 worldPoint =
            TransformPoint(
                transformB,
                pointB
            );

        const vec2 expected =
            InverseTransformPoint(
                transformA,
                worldPoint
            );

        const transform2 relative =
            InverseMul(
                transformA,
                transformB
            );

        const vec2 actual =
            TransformPoint(
                relative,
                pointB
            );

        assert( NearlyEqual( actual, expected ) );
    }

    {
        transform2 t{};
        t.position = { 5.0f, -7.0f };
        t.rotation =
            rot2::FromRadians( 1.2f );

        const transform2 relative =
            InverseMul( t, t );

        assert( NearlyEqual(
            relative.position,
            vec2{}
        ) );

        assert( NearlyEqual(
            relative.rotation.c,
            1.0f
        ) );

        assert( NearlyEqual(
            relative.rotation.s,
            0.0f
        ) );
    }

    return 0;
}