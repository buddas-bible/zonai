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

/*
* shape 질량 특성 계산 메모
*
* [공통]
* - density(rho)는 단위 면적당 질량임.
* - 질량: M = rho * A
* - 질량 중심: C = (1 / M) * integral( p * dm )
*            = (1 / A) * integral( p * dA )  // density가 일정한 경우
* - 2D에서는 회전축이 화면에 수직인 z축 하나뿐이므로
*   3D 관성 텐서 행렬 대신 z축에 대한 스칼라 회전 관성 I만 저장함.
* - rotationalInertia는 shape 자신의 center of mass를 지나는 z축 기준 값임.
*
* [평행축 정리]
* - center of mass에서 거리 d만큼 떨어진 평행한 축으로 옮기면
*   I_shifted = I_center + M * d^2
* - 여러 shape를 하나의 body로 합칠 때 각 shape 관성을
*   body center of mass 기준으로 옮기는 데 사용함.
*/

massData2 ComputeMass( const circle2& circle, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );
    assert( circle.radius >= 0.0f );

    // 원의 면적 A = PI * r^2
    // 질량 M = density * A = density * PI * r^2
    const float rr = circle.radius * circle.radius;
    const float mass = density * PI * rr;

    // 균일한 원판의 중심 기준 회전 관성
    // I = (1 / 2) * M * r^2
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

    /*
    * Capsule을 아래 두 도형의 합으로 계산함.
    *
    *     반원 + 직사각형 + 반원
    *      )---------------(
    *
    * 두 반원의 면적을 합치면 반지름 r인 원 하나와 같고,
    * 가운데 직사각형 크기는 length * (2r)임.
    *
    * circleMass = density * PI * r^2
    * boxMass    = density * length * 2r
    * totalMass  = circleMass + boxMass
    */

    const float radius = capsule.radius;
    const float rr = radius * radius;
    const float length = Length( capsule.center2 - capsule.center1 );
    const float ll = length * length;

    const float circleMass = density * PI * rr;
    const float boxMass = density * 2.0f * radius * length;
    const float mass = circleMass + boxMass;

    // Capsule은 양 끝의 형상이 대칭이므로 질량 중심은 axis의 정확한 중점임.
    const vec2 center =
        ( capsule.center1 + capsule.center2 ) * 0.5f;

    /*
    * 반원 두 개의 회전 관성
    *
    * 반원의 질량 중심은 원의 중심에서 곡면 방향으로
    *
    *     lc = 4r / (3PI)
    *
    * 만큼 떨어져 있음.
    *
    * Capsule 중심에서 직사각형 끝까지의 거리:
    *
    *     h = length / 2
    *
    * 두 반원을 합친 자체 관성은 원 하나와 같은
    *
    *     I_circleCenter = (1 / 2) * circleMass * r^2
    *
    * 이고, 두 반원을 Capsule 중심까지 옮기는 평행축 항을 합치면
    *
    *     circleMass * (h^2 + 2h * lc)
    *
    * 가 추가됨.
    *
    * 따라서:
    *
    *     I_circle =
    *         circleMass * ( r^2 / 2 + h^2 + 2h * lc )
    */
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

    /*
    * 가운데 직사각형의 중심 기준 회전 관성
    *
    * 사각형의 폭  = length
    * 사각형의 높이 = 2r
    *
    *     I_box = M * ( width^2 + height^2 ) / 12
    *           = boxMass * ( length^2 + 4r^2 ) / 12
    */
    const float boxInertia =
        boxMass * ( 4.0f * rr + ll ) / 12.0f;

    // 두 부분 모두 같은 Capsule center를 기준으로 계산했으므로 단순 합산 가능함.
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

    // rounded polygon의 vertex가 하나면 원, 두 개면 capsule과 같은 형상임.
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

    /*
    * 일반 convex polygon은 첫 vertex를 기준점(reference)으로 잡고
    * triangle fan으로 분할해서 면적 / centroid / 회전 관성을 누적함.
    *
    *       v2 ------ v3
    *       / \      /
    *      /   \    /
    *     /     \  /
    *   reference--v1
    *
    * 각 triangle:
    *
    *     e1 = v[i]     - reference
    *     e2 = v[i + 1] - reference
    *
    *     D = Cross( e1, e2 )
    *     triangleArea = D / 2
    *
    * CCW convex polygon을 사용하므로 D와 면적은 양수임.
    */

    std::array<vec2, MAX_POLYGON_VERTICES> vertices{};

    if( polygon.radius > 0.0f )
    {
        /*
        * rounded polygon은 실제 경계에 radius가 추가되므로
        * 원래 vertex만 적분하면 질량이 실제 collision 형상보다 작아짐.
        *
        * Box2D 방식대로 인접한 두 normal의 중간 방향으로
        * vertex를 약 sqrt(2) * radius만큼 밀어낸 polygon을 사용해
        * rounded 영역의 질량을 근사함.
        */
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

    // center는 아직 실제 좌표가 아니라
    // reference를 원점으로 본 area-weighted centroid 누적값임.
    vec2 center{};

    // A = sum( triangleArea )
    float area = 0.0f;

    // reference를 지나는 z축에 대한 면적 관성 적분값.
    // density는 모든 triangle 적분이 끝난 뒤 한 번 곱함.
    float rotationalInertia = 0.0f;

    // 첫 vertex를 임시 원점으로 사용하면 큰 world/local 좌표에서도
    // 작은 edge vector를 적분하게 되어 부동소수점 오차가 줄어듦.
    const vec2 reference = vertices[0];

    constexpr float inv3 = 1.0f / 3.0f;

    for( int i = 1; i < polygon.vertexCount - 1; ++i )
    {
        const vec2 e1 = vertices[i] - reference;
        const vec2 e2 = vertices[i + 1] - reference;

        // 2D 외적의 크기는 두 edge가 만드는 평행사변형의 signed area임.
        // 따라서 triangle 면적은 그 절반:
        //
        //     A_triangle = Cross( e1, e2 ) / 2
        const float D = Cross( e1, e2 );
        const float triangleArea = 0.5f * D;

        area += triangleArea;

        /*
        * reference를 원점으로 본 triangle centroid:
        *
        *     C_triangle = ( 0 + e1 + e2 ) / 3
        *                = ( e1 + e2 ) / 3
        *
        * Polygon 전체 centroid는 각 triangle centroid의
        * area-weighted average이므로 분자를 먼저 누적함:
        *
        *     centerNumerator += A_triangle * C_triangle
        */
        center +=
            ( e1 + e2 ) *
            ( triangleArea * inv3 );

        /*
        * Triangle 내부에 대해 x^2, y^2를 적분한 결과.
        *
        *     integral( x^2 dA )
        *       = D / 12 *
        *         ( e1.x^2 + e1.x*e2.x + e2.x^2 )
        *
        *     integral( y^2 dA )
        *       = D / 12 *
        *         ( e1.y^2 + e1.y*e2.y + e2.y^2 )
        *
        * z축 회전 관성의 면적 적분은
        *
        *     integral( x^2 + y^2 ) dA
        *
        * 이므로 두 값을 더해서 누적함.
        */
        const float intx2 =
            e1.x * e1.x +
            e2.x * e1.x +
            e2.x * e2.x;

        const float inty2 =
            e1.y * e1.y +
            e2.y * e1.y +
            e2.y * e2.y;

        // 0.25 * (1 / 3) = 1 / 12
        rotationalInertia +=
            ( 0.25f * inv3 * D ) *
            ( intx2 + inty2 );
    }

    assert( area > FLT_EPSILON );

    /*
    * 지금까지 center에는
    *
    *     sum( A_triangle * C_triangle )
    *
    * 가 들어 있으므로 전체 면적으로 나누면
    * reference 기준 polygon centroid가 됨.
    */
    const float invArea = 1.0f / area;
    center *= invArea;

    massData2 massData{};

    // M = density * A
    massData.mass = density * area;

    // center는 reference 기준 상대 좌표였으므로 local 좌표로 다시 이동함.
    massData.center = reference + center;

    /*
    * 위 적분으로 구한 값은 reference를 지나는 축 기준 관성임.
    *
    *     I_reference =
    *         density * integral( x^2 + y^2 ) dA
    *
    * 반환값은 shape center of mass 기준이어야 하므로
    * 평행축 정리를 반대로 적용함:
    *
    *     I_reference = I_center + M * d^2
    *
    * 따라서:
    *
    *     I_center = I_reference - M * d^2
    *
    * 여기서 d = |center|이며 center는 reference 기준 centroid임.
    */
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

    // 선분은 길이는 있지만 2D 면적 A가 0이므로
    // M = density * A에서도 질량이 0이며 회전 관성도 만들지 않음.
    return {};
}

} // namespace zonai
