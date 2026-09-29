#include "collision/massData2.h"

#include <array>
#include <cassert>
#include <cfloat>
#include <cmath>

namespace zonai
{

namespace
{

constexpr float PI = 3.14159265358979323846f;

}

massData2 ComputeMass( const circle2& circle, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );
    assert( circle.radius >= 0.0f );

    const float rr = circle.radius * circle.radius;
    const float mass = density * PI * rr;

    return
    {
        mass,
        circle.center,
        mass * 0.5f * rr
    };
}

massData2 ComputeMass( const capsule2& capsule, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );
    assert( capsule.radius >= 0.0f );

    const float radius = capsule.radius;
    const float rr = radius * radius;
    const float length = Length( capsule.center2 - capsule.center1 );
    const float ll = length * length;

    const float circleMass = density * PI * rr;
    const float boxMass = density * 2.0f * radius * length;
    const float mass = circleMass + boxMass;

    const vec2 center =
        ( capsule.center1 + capsule.center2 ) * 0.5f;

    // 두 반원의 질량은 합치면 하나의 원과 같고,
    // 직사각형 끝으로 이동한 만큼 parallel-axis theorem을 적용함.
    const float halfCircleCentroid =
        4.0f * radius / ( 3.0f * PI );
    const float halfLength = 0.5f * length;

    const float circleInertia =
        circleMass *
        (
            0.5f * rr +
            halfLength * halfLength +
            2.0f * halfLength * halfCircleCentroid
        );

    const float boxInertia =
        boxMass * ( 4.0f * rr + ll ) / 12.0f;

    return
    {
        mass,
        center,
        circleInertia + boxInertia
    };
}

massData2 ComputeMass( const polygon2& polygon, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );
    assert( polygon.vertexCount > 0 );

    if( polygon.vertexCount == 1 )
    {
        return ComputeMass(
            circle2{ polygon.vertices[0], polygon.radius },
            density
        );
    }

    if( polygon.vertexCount == 2 )
    {
        return ComputeMass(
            capsule2
            {
                polygon.vertices[0],
                polygon.vertices[1],
                polygon.radius
            },
            density
        );
    }

    std::array<vec2, MAX_POLYGON_VERTICES> vertices{};

    if( polygon.radius > 0.0f )
    {
        // Box2D와 같이 rounded polygon의 질량은 vertex를 바깥으로 밀어 근사함.
        constexpr float sqrt2 = 1.412f;

        for( int i = 0; i < polygon.vertexCount; ++i )
        {
            const int previous =
                i == 0 ? polygon.vertexCount - 1 : i - 1;

            const vec2 mid =
                Normalize(
                    polygon.normals[previous] +
                    polygon.normals[i]
                );

            vertices[i] =
                polygon.vertices[i] +
                mid * ( sqrt2 * polygon.radius );
        }
    }
    else
    {
        for( int i = 0; i < polygon.vertexCount; ++i )
        {
            vertices[i] = polygon.vertices[i];
        }
    }

    vec2 center{};
    float area = 0.0f;
    float rotationalInertia = 0.0f;

    // 첫 vertex를 기준점으로 삼아 큰 좌표에서의 round-off를 줄임.
    const vec2 reference = vertices[0];

    constexpr float inv3 = 1.0f / 3.0f;

    for( int i = 1; i < polygon.vertexCount - 1; ++i )
    {
        const vec2 e1 = vertices[i] - reference;
        const vec2 e2 = vertices[i + 1] - reference;

        const float D = Cross( e1, e2 );
        const float triangleArea = 0.5f * D;

        area += triangleArea;
        center +=
            ( e1 + e2 ) *
            ( triangleArea * inv3 );

        const float intx2 =
            e1.x * e1.x +
            e2.x * e1.x +
            e2.x * e2.x;

        const float inty2 =
            e1.y * e1.y +
            e2.y * e1.y +
            e2.y * e2.y;

        rotationalInertia +=
            ( 0.25f * inv3 * D ) *
            ( intx2 + inty2 );
    }

    assert( area > FLT_EPSILON );

    const float invArea = 1.0f / area;
    center *= invArea;

    massData2 massData{};
    massData.mass = density * area;
    massData.center = reference + center;
    massData.rotationalInertia =
        density * rotationalInertia -
        massData.mass * Dot( center, center );

    assert( massData.rotationalInertia >= 0.0f );

    return massData;
}

massData2 ComputeMass( const segment2&, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );

    return {};
}

} // namespace zonai
