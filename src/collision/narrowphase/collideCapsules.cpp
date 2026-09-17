#include "collision/narrowphase/collide.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace zonai
{

localManifold2 CollideCapsules(
    const capsule2& a,
    const capsule2& b, const transform2& transformB )
{
    localManifold2 manifold = {};

    // 계산 오차를 줄이기 위해 capsule A의 첫 점을 원점으로 이동한다.
    const vec2 origin = a.center1;

    const vec2 point1A{};
    const vec2 point2A = a.center2 - origin;

    const vec2 point1B =
        zonai::TransformPoint( transformB, b.center1 ) - origin;
    const vec2 point2B =
        zonai::TransformPoint( transformB, b.center2 ) - origin;

    const vec2 directionA = point2A - point1A;
    const vec2 directionB = point2B - point1B;

    const float lengthSquaredA = zonai::Dot( directionA, directionA );
    const float lengthSquaredB = zonai::Dot( directionB, directionB );

    constexpr float epsilonSquared = FLT_EPSILON * FLT_EPSILON;

    if( lengthSquaredA <= epsilonSquared ||
        lengthSquaredB <= epsilonSquared )
    {
        return manifold;
    }

    // Ericson 5.1.9: 두 선분 위의 최근접점 계산
    const vec2 offset = point1A - point1B;
    const float offsetA = zonai::Dot( offset, directionA );
    const float offsetB = zonai::Dot( offset, directionB );
    const float directionsDot = zonai::Dot( directionA, directionB );

    const float denominator =
        lengthSquaredA * lengthSquaredB -
        directionsDot * directionsDot;

    float fractionA = 0.0f;

    if( denominator != 0.0f )
    {
        fractionA = std::clamp(
            ( directionsDot * offsetB -
              offsetA * lengthSquaredB ) / denominator,
            0.0f,
            1.0f
        );
    }

    float fractionB =
        ( directionsDot * fractionA + offsetB ) /
        lengthSquaredB;

    if( fractionB < 0.0f )
    {
        fractionB = 0.0f;
        fractionA = std::clamp(
            -offsetA / lengthSquaredA,
            0.0f,
            1.0f
        );
    }
    else if( fractionB > 1.0f )
    {
        fractionB = 1.0f;
        fractionA = std::clamp(
            ( directionsDot - offsetA ) / lengthSquaredA,
            0.0f,
            1.0f
        );
    }

    const vec2 closestA =
        point1A + directionA * fractionA;
    const vec2 closestB =
        point1B + directionB * fractionB;

    const vec2 closestDelta = closestB - closestA;
    const float distanceSquared =
        zonai::LengthSquared( closestDelta );

    const float radiusSum = a.radius + b.radius;

    // TODO: speculative contacts
    if( distanceSquared > radiusSum * radiusSum )
    {
        return manifold;
    }

    const float distance = std::sqrt( distanceSquared );

    const float lengthA = std::sqrt( lengthSquaredA );
    const float lengthB = std::sqrt( lengthSquaredB );
    const vec2 axisA = directionA / lengthA;
    const vec2 axisB = directionB / lengthB;

    // 두 선분의 투영이 겹치는 경우에는 2점 manifold를 시도한다.
    const float projectionB1 =
        zonai::Dot( point1B - point1A, axisA );
    const float projectionB2 =
        zonai::Dot( point2B - point1A, axisA );

    const bool outsideA =
        ( projectionB1 <= 0.0f && projectionB2 <= 0.0f ) ||
        ( projectionB1 >= lengthA && projectionB2 >= lengthA );

    const float projectionA1 =
        zonai::Dot( point1A - point1B, axisB );
    const float projectionA2 =
        zonai::Dot( point2A - point1B, axisB );

    const bool outsideB =
        ( projectionA1 <= 0.0f && projectionA2 <= 0.0f ) ||
        ( projectionA1 >= lengthB && projectionA2 >= lengthB );

    if( !outsideA && !outsideB )
    {
        vec2 normalA{ -axisA.y, axisA.x };

        const float sideA1 =
            zonai::Dot( point1B - point1A, normalA );
        const float sideA2 =
            zonai::Dot( point2B - point1A, normalA );

        const float positiveA = std::min( sideA1, sideA2 );
        const float negativeA = std::min( -sideA1, -sideA2 );

        float separationA = positiveA;

        if( positiveA <= negativeA )
        {
            separationA = negativeA;
            normalA = -normalA;
        }

        vec2 normalB{ -axisB.y, axisB.x };

        const float sideB1 =
            zonai::Dot( point1A - point1B, normalB );
        const float sideB2 =
            zonai::Dot( point2A - point1B, normalB );

        const float positiveB = std::min( sideB1, sideB2 );
        const float negativeB = std::min( -sideB1, -sideB2 );

        float separationB = positiveB;

        if( positiveB <= negativeB )
        {
            separationB = negativeB;
            normalB = -normalB;
        }

        constexpr float linearSlop = 0.005f;

        if( separationA + 0.1f * linearSlop >= separationB )
        {
            manifold.normal = normalA;

            vec2 contact1 = point1B;
            vec2 contact2 = point2B;

            if( projectionB1 < 0.0f && projectionB2 > 0.0f )
            {
                contact1 = point1B +
                    ( point2B - point1B ) *
                    ( -projectionB1 /
                      ( projectionB2 - projectionB1 ) );
            }
            else if( projectionB2 < 0.0f && projectionB1 > 0.0f )
            {
                contact2 = point2B +
                    ( point1B - point2B ) *
                    ( -projectionB2 /
                      ( projectionB1 - projectionB2 ) );
            }

            if( projectionB1 > lengthA && projectionB2 < lengthA )
            {
                contact1 = point1B +
                    ( point2B - point1B ) *
                    ( ( projectionB1 - lengthA ) /
                      ( projectionB1 - projectionB2 ) );
            }
            else if( projectionB2 > lengthA && projectionB1 < lengthA )
            {
                contact2 = point2B +
                    ( point1B - point2B ) *
                    ( ( projectionB2 - lengthA ) /
                      ( projectionB2 - projectionB1 ) );
            }

            const float separation1 =
                zonai::Dot( contact1 - point1A, normalA );
            const float separation2 =
                zonai::Dot( contact2 - point1A, normalA );

            if( separation1 <= distance + linearSlop ||
                separation2 <= distance + linearSlop )
            {
                manifold.points[0].point =
                    contact1 + normalA *
                    ( 0.5f *
                      ( a.radius - b.radius - separation1 ) );
                manifold.points[0].separation =
                    separation1 - radiusSum;

                manifold.points[1].point =
                    contact2 + normalA *
                    ( 0.5f *
                      ( a.radius - b.radius - separation2 ) );
                manifold.points[1].separation =
                    separation2 - radiusSum;

                manifold.pointCount = 2;
            }
        }
        else
        {
            // normal은 항상 A에서 B를 향한다.
            manifold.normal = -normalB;

            vec2 contact1 = point1A;
            vec2 contact2 = point2A;

            if( projectionA1 < 0.0f && projectionA2 > 0.0f )
            {
                contact1 = point1A +
                    ( point2A - point1A ) *
                    ( -projectionA1 /
                      ( projectionA2 - projectionA1 ) );
            }
            else if( projectionA2 < 0.0f && projectionA1 > 0.0f )
            {
                contact2 = point2A +
                    ( point1A - point2A ) *
                    ( -projectionA2 /
                      ( projectionA1 - projectionA2 ) );
            }

            if( projectionA1 > lengthB && projectionA2 < lengthB )
            {
                contact1 = point1A +
                    ( point2A - point1A ) *
                    ( ( projectionA1 - lengthB ) /
                      ( projectionA1 - projectionA2 ) );
            }
            else if( projectionA2 > lengthB && projectionA1 < lengthB )
            {
                contact2 = point2A +
                    ( point1A - point2A ) *
                    ( ( projectionA2 - lengthB ) /
                      ( projectionA2 - projectionA1 ) );
            }

            const float separation1 =
                zonai::Dot( contact1 - point1B, normalB );
            const float separation2 =
                zonai::Dot( contact2 - point1B, normalB );

            if( separation1 <= distance + linearSlop ||
                separation2 <= distance + linearSlop )
            {
                manifold.points[0].point =
                    contact1 + normalB *
                    ( 0.5f *
                      ( b.radius - a.radius - separation1 ) );
                manifold.points[0].separation =
                    separation1 - radiusSum;

                manifold.points[1].point =
                    contact2 + normalB *
                    ( 0.5f *
                      ( b.radius - a.radius - separation2 ) );
                manifold.points[1].separation =
                    separation2 - radiusSum;

                manifold.pointCount = 2;
            }
        }
    }

    if( manifold.pointCount == 0 )
    {
        vec2 normal = closestDelta;

        if( zonai::LengthSquared( normal ) > epsilonSquared )
        {
            normal = zonai::Normalize( normal );
        }
        else
        {
            normal = { -axisA.y, axisA.x };
        }

        const vec2 surfaceA =
            closestA + normal * a.radius;
        const vec2 surfaceB =
            closestB - normal * b.radius;

        manifold.normal = normal;
        manifold.points[0].point =
            ( surfaceA + surfaceB ) * 0.5f;
        manifold.points[0].separation =
            distance - radiusSum;
        manifold.pointCount = 1;
    }

    for( std::size_t i = 0; i < manifold.pointCount; ++i )
    {
        manifold.points[i].point += origin;
    }

    return manifold;
}

localManifold2 CollideSegmentCapsule(
    const segment2& segment,
    const capsule2& capsuleB, const transform2& capsuleBTransform )
{
    const capsule2 capsuleA =
    {
        segment.a,
        segment.b,
        0.0f
    };

    return CollideCapsules(
        capsuleA,
        capsuleB,
        capsuleBTransform
    );
}

} // namespace zonai
