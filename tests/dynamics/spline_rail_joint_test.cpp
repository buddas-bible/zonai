#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include "dynamics/joints/prismaticJointConstraint2.h"
#include "dynamics/joints/splineRailJointConstraint2.h"

using namespace zonai;

namespace
{

void check( bool ok, const char* message )
{
    if( !ok )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

bool near( float a, float b )
{
    return std::abs( a - b ) <= 1.0e-5f * std::max( { 1.0f, std::abs( a ), std::abs( b ) } );
}

}

int main()
{
    for( int mode = 0; mode < 8; ++mode )
        for( float inverseMass : { 1.0f, 1.0e-6f, 1.0e-8f } )
            for( bool dynamicA : { false, true } )
            {
                bodySim a{}, b{};
                a.bodyId = 0;
                b.bodyId = 1;
                a.invMass = dynamicA ? inverseMass : 0.0f;
                a.invInertia = a.invMass;
                b.invMass = inverseMass;
                b.invInertia = inverseMass;
                b.center = { 2.0f, 0.2f };
                b.transform.position = b.center;
                prismaticJointSim2 line{};
                line.bodyIdA = 0;
                line.bodyIdB = 1;
                line.localAnchorB = { 0.3f, 0.2f };
                line.enableSpring = mode == 1 || mode == 2 || mode == 7;
                line.hertz = mode == 2 ? 0.0f : 3.0f;
                line.targetTranslation = 1.0f;
                line.enableMotor = mode == 3 || mode == 4 || mode == 7;
                line.motorSpeed = mode == 4 ? 0.0f : 2.0f;
                line.maxMotorForce = 100.0f;
                line.enableLimit = mode >= 5;
                line.lowerTranslation = 1.0f;
                line.upperTranslation = mode == 6 ? 1.0f : 2.0f;
                splineRailJointSim2 rail{};
                rail.bodyIdA = 0;
                rail.bodyIdB = 1;
                rail.localAnchorB = line.localAnchorB;
                rail.enableSpring = line.enableSpring;
                rail.hertz = line.hertz;
                rail.targetTranslation = line.targetTranslation;
                rail.enableMotor = line.enableMotor;
                rail.motorSpeed = line.motorSpeed;
                rail.maxMotorForce = line.maxMotorForce;
                rail.enableLimit = line.enableLimit;
                rail.lowerTranslation = line.lowerTranslation;
                rail.upperTranslation = line.upperTranslation;
                const vec2 points[] = { { -100.0f, 0.0f }, { 100.0f, 0.0f } };
                rail.referenceDistance = 100.0;
                check( setSplineRailGeometry( rail, points ), "straight rail geometry builds" );
                check( rail.geometry.getSegmentCount() == 1 && rail.geometry.getSegments()[0].start.x == -100.0f, "rail preparation commits its owned prepared snapshot" );
                auto expected = preparePrismaticJointConstraint( line, a, b, 1.0f / 60.0f );
                auto actual = prepareSplineRailJointConstraint( rail, a, b, 1.0f / 60.0f );
                bodyState ea{}, eb{}, aa{}, ab{};
                eb.linearVelocity = { 3.0f, 2.0f };
                eb.angularVelocity = 4.0f;
                ab = eb;
                for( int i = 0; i < 6; ++i )
                {
                    solvePrismaticJointConstraint( expected, ea, eb, i < 3 );
                    solveSplineRailJointConstraint( actual, aa, ab, i < 3 );
                }
                check( near( ea.linearVelocity.x, aa.linearVelocity.x ) && near( ea.linearVelocity.y, aa.linearVelocity.y ) && near( ea.angularVelocity, aa.angularVelocity ) && near( eb.linearVelocity.x, ab.linearVelocity.x ) && near( eb.linearVelocity.y, ab.linearVelocity.y ) && near( eb.angularVelocity, ab.angularVelocity ), "straight spline solver matches prismatic spring motor limit and rotation response" );
                check( near( actual.impulse.x, expected.impulse.x ) && near( actual.impulse.y, expected.impulse.y ) && near( actual.springImpulse, expected.springImpulse ) && near( actual.motorImpulse, expected.motorImpulse ), "straight spline reaction impulses match prismatic" );
            }
    {
        splineRailJointSim2 rail{};
        const vec2 points[] = { { -2, 0 }, { 2, 0 } };
        check( setSplineRailGeometry( rail, points ), "finite failure fixture builds" );
        rail.enableSpring = true;
        rail.hertz = std::numeric_limits<float>::max();
        rail.targetTranslation = 1.0f;
        bodySim a{}, b{};
        b.invMass = b.invInertia = 1.0f;
        auto constraint = prepareSplineRailJointConstraint( rail, a, b, 1.0f / 60.0f );
        bodyState stateA{}, stateB{};
        stateB.linearVelocity = { 1, 2 };
        solveSplineRailJointConstraint( constraint, stateA, stateB, true );
        check( IsFinite( stateA.linearVelocity ) && IsFinite( stateB.linearVelocity ) && std::isfinite( stateB.angularVelocity ) && !constraint.trackingValid && LengthSquared( constraint.force ) == 0.0f, "overflowing spring computation skips impulses without poisoning body state" );
    }
    {
        const vec2 points[] = { { -4, 0 }, { -2, 0 }, { 0, 2 }, { 2, 0 }, { 4, 0 } };
        splineRailJointSim2 rail{};
        check( setSplineRailGeometry( rail, points ), "high-speed projection fixture builds" );
        const vec2 target = Evaluate( { points }, 2, 0.1f );
        const auto frame = getSplineRailFrame( rail, {}, { target, {} }, { 1, 0.05f }, true );
        check( frame.valid && frame.parameter.segmentIndex == 2 && std::abs( frame.parameter.t - 0.1f ) < 1.0e-4f, "tracked projection traverses adjacent segments after a large advance" );
        const vec2 tangent = Normalize( Derivative( { points }, 0, 0.0f ) );
        const auto outside = getSplineRailFrame( rail, {}, { points[0] - 0.2f * tangent, {} }, {}, true );
        check( outside.valid && outside.curvature == 0.0, "clamped endpoint angle has no curvature coupling beyond its physical stop" );
    }
    {
        splineRailJointSim2 rail{};
        const vec2 points[] = { { -2, 0 }, { 0, 1 }, { 2, 0 } };
        check( setSplineRailGeometry( rail, points ), "dynamic support fixture builds" );
        rail.rotationMode = splineRailRotationMode::followTangent;
        rail.localAnchorB = { 0.2f, -0.1f };
        rail.enableMotor = true;
        rail.motorSpeed = 2.0f;
        rail.maxMotorForce = 10.0f;
        bodySim a{}, b{};
        a.invMass = 0.5f;
        a.invInertia = 0.25f;
        b.invMass = 1.0f;
        b.invInertia = 0.5f;
        b.center = { -0.5f, 0.8f };
        b.transform.position = b.center;
        auto constraint = prepareSplineRailJointConstraint( rail, a, b, 1.0f / 60.0f );
        bodyState sa{}, sb{};
        for( int i = 0; i < 4; ++i )
            solveSplineRailJointConstraint( constraint, sa, sb, true );
        const vec2 momentum = sa.linearVelocity / a.invMass + sb.linearVelocity / b.invMass;
        const float angularMomentum = sa.angularVelocity / a.invInertia + sb.angularVelocity / b.invInertia + Cross( b.center, sb.linearVelocity / b.invMass );
        check( Length( momentum ) < 1.0e-4f && std::abs( angularMomentum ) < 1.0e-4f, "curved off-center rail transfers equal opposite force and torque to dynamic A" );
        const transform2 pose{ { 3, -1 }, rot2::FromRadians( 0.7f ) };
        a.center = TransformPoint( pose, a.center );
        a.transform = pose;
        b.center = TransformPoint( pose, b.center );
        b.transform = { b.center, pose.rotation };
        auto rotated = prepareSplineRailJointConstraint( rail, a, b, 1.0f / 60.0f );
        bodyState ra{}, rb{};
        for( int i = 0; i < 4; ++i )
            solveSplineRailJointConstraint( rotated, ra, rb, true );
        check( Length( ra.linearVelocity - Rotate( pose.rotation, sa.linearVelocity ) ) < 1.0e-4f && Length( rb.linearVelocity - Rotate( pose.rotation, sb.linearVelocity ) ) < 1.0e-4f && near( ra.angularVelocity, sa.angularVelocity ) && near( rb.angularVelocity, sb.angularVelocity ), "rotating and translating both bodies preserves the local curved rail response" );
    }
    {
        const vec2 points[] = { { -2, 0 }, { 0, 1 }, { 2, 0 } };
        const catmullRomSpline2 spline{ points };
        const vec2 derivative = Derivative( spline, 1, 0.0f );
        const vec2 second = SecondDerivative( spline, 1, 0.0f );
        const float speed = Length( derivative );
        const float curvature = Cross( derivative, second ) / ( speed * speed * speed );
        const vec2 target = Evaluate( spline, 1, 0.0f ) + Cross( 1.0f, derivative / speed ) * ( 2.0f / curvature );
        splineRailJointSim2 rail{};
        check( setSplineRailGeometry( rail, points ), "singular projection fixture builds" );
        rail.parameter = { 1, 0.0f };
        rail.trackingValid = true;
        bodySim a{}, b{};
        b.invMass = b.invInertia = 1.0f;
        b.center = target;
        b.transform.position = target;
        auto constraint = prepareSplineRailJointConstraint( rail, a, b, 1.0f / 60.0f );
        bodyState sa{}, sb{};
        sb.linearVelocity = { 1, 2 };
        solveSplineRailJointConstraint( constraint, sa, sb, true );
        check( !constraint.trackingValid && sb.linearVelocity.x == 1.0f && sb.linearVelocity.y == 2.0f && constraint.springImpulse == 0.0f, "singular projection clears caches and skips the entire physical impulse" );
        const vec2 cusp[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
        check( !setSplineRailGeometry( rail, cusp ) && rail.points.size() == 3 && rail.points[0].x == -2.0f, "zero-tangent candidate is rejected before replacing geometry" );
    }
    {
        const vec2 points[] = { { -2, 0 }, { 0, 1 }, { 2, 0 } };
        const vec2 edited[] = { { -2, 0 }, { 0, 1.5f }, { 2, 0 } };
        const vec2 invalid[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
        splineRailJointSim2 rail{};
        check( setSplineRailGeometry( rail, points ) && setSplineRailGeometry( rail, edited ), "rail snapshot accepts a regular geometry edit" );
        const vec2 before = rail.geometry.evaluate( 0, 0.5f );
        const double length = rail.length;
        const auto* address = rail.geometry.getSegments().data();
        check( !setSplineRailGeometry( rail, invalid ) && rail.geometry.getSegments().data() == address && rail.geometry.evaluate( 0, 0.5f ).y == before.y && rail.length == length, "failed rail edit preserves snapshot and arc metadata atomically" );
        auto copy = rail;
        rail = {};
        check( copy.geometry.getSegments().size() == copy.segments.size() && copy.geometry.evaluate( 0, 0.5f ).y == before.y, "copied rail geometry has no borrowed segment pointers" );
    }
    return 0;
}
