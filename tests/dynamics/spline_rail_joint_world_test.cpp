#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include "dynamics/world.h"

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

bool near( vec2 a, vec2 b, float error = 0.002f )
{
    return Length( a - b ) <= error;
}

}

int main()
{
    {
        world simulation{};
        simulation.SetGravity( {} );
        const auto a = simulation.CreateBody();
        const auto b = simulation.CreateBody( bodyType::Dynamic, { { 0, 0.25f }, rot2::FromRadians( 0.35f ) } );
        ( void )simulation.CreateShape( b, circle2{ {}, 0.25f } );
        const vec2 points[] = { { -2, 0 }, { 0, 1 }, { 2, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.rotationMode = splineRailRotationMode::followTangent;
        definition.enableAngularSpring = false;
        definition.angularHertz = 4.0f;
        definition.angularDampingRatio = 0.45f;
        const auto id = simulation.createSplineRailJoint( definition );
        auto data = simulation.getSplineRailJointData( id );
        check( !data.enableAngularSpring && data.angularHertz == 4.0f && data.angularDampingRatio == 0.45f, "World preserves explicit hard tangent selection and spring tuning at creation" );
        simulation.SetBodyTransform( b, { { 0, 0.25f }, rot2::FromRadians( 0.55f ) } );
        simulation.SetBodyLinearVelocity( b, { 1.0f, 0.5f } );
        simulation.SetBodyAngularVelocity( b, 2.0f );
        const auto before = simulation.getSplineRailJointData( id );
        const auto pose = simulation.GetBodyTransform( b );
        const auto velocity = simulation.GetBodyLinearVelocity( b );
        const auto angularVelocity = simulation.GetBodyAngularVelocity( b );
        for( bool spring : { true, false, true } )
        {
            simulation.SetBodyAwake( b, false );
            simulation.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.0f, 0.45f, spring );
            data = simulation.getSplineRailJointData( id );
            check( data.enableAngularSpring == spring && data.angularHertz == 4.0f && data.angularDampingRatio == 0.45f && std::abs( data.currentAngle - before.currentAngle ) < 1.0e-6f && data.referenceAngle == before.referenceAngle, "changing angular constraint preserves spring tuning and the original tangent reference" );
            check( simulation.IsBodyAwake( b ) && data.torque == 0.0f, "angular constraint changes wake the body and clear old impulses" );
            simulation.SetBodyAwake( b, false );
            simulation.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.0f, 0.45f, spring );
            check( !simulation.IsBodyAwake( b ), "unchanged angular constraint does not wake the body" );
        }
        simulation.SetBodyAwake( b, true );
        simulation.SetBodyLinearVelocity( b, velocity );
        simulation.SetBodyAngularVelocity( b, angularVelocity );
        simulation.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.0f, 0.45f, false );
        check( near( simulation.GetBodyTransform( b ).position, pose.position, 0.0f ) && simulation.GetBodyTransform( b ).rotation.s == pose.rotation.s && near( simulation.GetBodyLinearVelocity( b ), velocity, 0.0f ) && simulation.GetBodyAngularVelocity( b ) == angularVelocity, "angular constraint selection does not overwrite dynamic pose or velocity" );
        simulation.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, -1.0f, 0.45f, true );
        check( !simulation.getSplineRailJointData( id ).enableAngularSpring, "invalid tuning cannot change angular constraint selection" );
        simulation.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.0f, 0.45f );
        check( simulation.getSplineRailJointData( id ).enableAngularSpring, "existing rotation setter calls retain the legacy spring default" );
    }
    for( int mode = 0; mode < 7; ++mode )
    {
        world lineWorld{}, curveWorld{};
        lineWorld.SetGravity( {} );
        curveWorld.SetGravity( {} );
        const auto lineA = lineWorld.CreateBody(), curveA = curveWorld.CreateBody();
        const auto lineB = lineWorld.CreateBody( bodyType::Dynamic, { { 2.0f, 0.2f }, {} } ), curveB = curveWorld.CreateBody( bodyType::Dynamic, { { 2.0f, 0.2f }, {} } );
        ( void )lineWorld.CreateShape( lineB, circle2{ {}, 0.25f } );
        ( void )curveWorld.CreateShape( curveB, circle2{ {}, 0.25f } );
        prismaticJointDef line{};
        line.bodyA = lineA;
        line.bodyB = lineB;
        line.enableSpring = mode == 1 || mode == 2 || mode == 6;
        line.hertz = mode == 2 ? 0.0f : 3.0f;
        line.targetTranslation = 1.0f;
        line.enableMotor = mode == 3 || mode == 4 || mode == 6;
        line.motorSpeed = mode == 4 ? 0.0f : 2.0f;
        line.maxMotorForce = 50.0f;
        line.enableLimit = mode == 5 || mode == 6;
        line.lowerTranslation = 1.0f;
        line.upperTranslation = 3.0f;
        const auto lineId = lineWorld.createPrismaticJoint( line );
        const vec2 points[] = { { 0.0f, 0.0f }, { 10.0f, 0.0f } };
        splineRailJointDef rail{};
        rail.bodyA = curveA;
        rail.bodyB = curveB;
        rail.points = points;
        rail.enableSpring = line.enableSpring;
        rail.hertz = line.hertz;
        rail.targetTranslation = line.targetTranslation;
        rail.enableMotor = line.enableMotor;
        rail.motorSpeed = line.motorSpeed;
        rail.maxMotorForce = line.maxMotorForce;
        rail.enableLimit = line.enableLimit;
        rail.lowerTranslation = line.lowerTranslation;
        rail.upperTranslation = line.upperTranslation;
        const auto curveId = curveWorld.createSplineRailJoint( rail );
        check( curveWorld.IsValid( curveId ), "World creates a dynamic spline rail joint" );
        lineWorld.SetBodyLinearVelocity( lineB, { 1.0f, 2.0f } );
        curveWorld.SetBodyLinearVelocity( curveB, { 1.0f, 2.0f } );
        for( int i = 0; i < 120; ++i )
        {
            lineWorld.Step( 1.0f / 60.0f, 4 );
            curveWorld.Step( 1.0f / 60.0f, 4 );
        }
        check( near( lineWorld.GetBodyTransform( lineB ).position, curveWorld.GetBodyTransform( curveB ).position ) && near( lineWorld.GetBodyLinearVelocity( lineB ), curveWorld.GetBodyLinearVelocity( curveB ) ), "World straight spline follows the same prismatic trajectory" );
        const auto actual = curveWorld.getSplineRailJointData( curveId );
        const auto expected = lineWorld.getPrismaticJointData( lineId );
        check( actual.trackingValid && std::abs( actual.currentTranslation - expected.currentTranslation ) < 0.002 && std::abs( actual.currentAngle - expected.currentAngle ) < 0.0001f, "World straight spline reports matching translation and relative angle" );
        const auto pose = curveWorld.GetBodyTransform( curveB );
        const auto velocity = curveWorld.GetBodyLinearVelocity( curveB );
        const vec2 invalid[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 0.0f, 0.0f } };
        check( !curveWorld.setSplineRailJointPoints( curveId, invalid ) && curveWorld.getSplineRailJointPoints( curveId ).size() == 2 && near( pose.position, curveWorld.GetBodyTransform( curveB ).position, 0.0f ) && near( velocity, curveWorld.GetBodyLinearVelocity( curveB ), 0.0f ), "invalid rail edits preserve geometry and body state atomically" );
        curveWorld.DestroyBody( curveB );
        check( !curveWorld.IsValid( curveId ), "destroying a slider destroys its spline joint" );
    }
    const vec2 curve[] = { { -4.0f, 0.0f }, { -2.0f, 0.0f }, { 0.0f, 2.0f }, { 2.0f, 0.0f }, { 4.0f, 0.0f } };
    for( const auto [angularHertz, enableAngularSpring] : { std::pair{ 0.0f, true }, { 6.0f, true }, { 6.0f, false } } )
        for( auto mode : { splineRailRotationMode::fixed, splineRailRotationMode::followTangent } )
        {
            world simulation{};
            simulation.SetGravity( { 0.0f, -9.8f } );
            const auto railBody = simulation.CreateBody();
            const auto slider = simulation.CreateBody( bodyType::Dynamic, { curve[0], {} } );
            const auto sliderShape = simulation.CreateShape( slider, circle2{ {}, 0.2f } );
            splineRailJointDef definition{};
            definition.bodyA = railBody;
            definition.bodyB = slider;
            definition.points = curve;
            definition.rotationMode = mode;
            definition.angularHertz = angularHertz;
            definition.enableAngularSpring = enableAngularSpring;
            definition.enableMotor = true;
            definition.motorSpeed = 2.0f;
            definition.maxMotorForce = 20.0f;
            const auto joint = simulation.createSplineRailJoint( definition );
            check( simulation.IsValid( joint ), "World creates a curved spline rail" );
            double maximumDistance = 0.0;
            for( int i = 0; i < 420; ++i )
            {
                simulation.Step( 1.0f / 60.0f, 4 );
                const auto data = simulation.getSplineRailJointData( joint );
                check( data.trackingValid && std::abs( data.lateralError ) < 0.025f && IsFinite( data.force ) && std::isfinite( data.torque ), "curved motor rail stays tracked and physically constrained" );
                if( mode == splineRailRotationMode::fixed )
                    check( std::abs( data.currentAngle ) < 0.0001f, "curved fixed rail retains its initial relative rotation" );
                else
                    check( std::abs( data.currentAngle ) < 0.18f, "curved tangent constraint follows orientation without pose overwrites" );
                maximumDistance = std::max( maximumDistance, data.currentTranslation );
            }
            const auto stopped = simulation.getSplineRailJointData( joint );
            check( maximumDistance > stopped.length - 0.1 && stopped.currentTranslation <= stopped.length + 0.025 && Length( simulation.GetBodyLinearVelocity( slider ) ) < 0.02f, "outward motor stops at the finite curved endpoint" );
            simulation.setSplineRailJointMotor( joint, false, 0.0f, 0.0f );
            simulation.setSplineRailJointSpring( joint, true, 2.0f, 0.8f, 1.0f );
            for( int i = 0; i < 600; ++i )
                simulation.Step( 1.0f / 60.0f, 4 );
            check( std::abs( simulation.getSplineRailJointData( joint ).currentTranslation - 1.0 ) < 0.03 && Length( simulation.GetBodyLinearVelocity( slider ) ) < 0.03f, "curved rail spring returns and damps along arc distance" );
            simulation.SetShapeDensity( sliderShape, 2.0f );
            check( simulation.getSplineRailJointData( joint ).trackingValid, "mass cache reset retains the tracked spline branch" );
            const auto pose = simulation.GetBodyTransform( slider );
            const auto velocity = simulation.GetBodyLinearVelocity( slider );
            const float angularVelocity = simulation.GetBodyAngularVelocity( slider );
            const float mass = simulation.GetBodyMass( slider );
            const vec2 edited[] = { { -4.0f, 0.0f }, { -2.0f, 0.5f }, { 0.0f, 1.5f }, { 2.0f, -0.2f }, { 4.0f, 0.0f } };
            check( simulation.setSplineRailJointPoints( joint, edited ) && simulation.IsValid( joint ) && simulation.IsValid( slider ), "curved rail editing retains stable handles" );
            check( near( pose.position, simulation.GetBodyTransform( slider ).position, 0.0f ) && near( velocity, simulation.GetBodyLinearVelocity( slider ), 0.0f ) && angularVelocity == simulation.GetBodyAngularVelocity( slider ) && mass == simulation.GetBodyMass( slider ), "curve editing preserves dynamic pose velocity and mass" );
            check( simulation.getSplineRailJointData( joint ).springForce == 0.0f && simulation.IsBodyAwake( slider ), "curve editing clears impulses and wakes its slider" );
        }

    {
        world pair{};
        pair.SetGravity( {} );
        const auto a = pair.CreateBody( bodyType::Dynamic ), b = pair.CreateBody( bodyType::Dynamic );
        ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
        ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
        pair.Step( 1.0f / 60.0f, 1 );
        check( pair.GetContactCount() == 1, "rail collision fixture starts overlapped" );
        std::vector<vec2> points{ { -2, 0 }, { 2, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        const auto id = pair.createSplineRailJoint( definition );
        points.clear();
        check( pair.getSplineRailJointPoints( id ).size() == 2 && pair.GetBody( a ).jointCount == 1, "rail owns geometry and participates in the shared graph" );
        pair.Step( 1.0f / 60.0f, 1 );
        check( pair.GetContactCount() == 0, "rail suppresses connected collisions" );
        pair.SetBodyAwake( a, false );
        check( !pair.IsBodyAwake( b ), "rail bodies share sleep state" );
        pair.setSplineRailJointMotor( id, true, 0.0f, 2.0f );
        check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "rail setter wakes the connected component" );
        pair.SetBodyAwake( a, false );
        pair.setSplineRailJointMotor( id, true, 0.0f, 2.0f );
        check( !pair.IsBodyAwake( b ), "unchanged settings do not wake the rail" );
        pair.setSplineRailJointLimit( id, true, 10.0f, 20.0f );
        check( !pair.getSplineRailJointData( id ).enableLimit && !pair.IsBodyAwake( b ), "disjoint limits are rejected without state changes" );
        pair.setSplineRailJointSpring( id, true, -1.0f, 0.7f, 0.0f );
        check( !pair.getSplineRailJointData( id ).enableSpring, "invalid spring settings remain unapplied" );
        pair.SetBodyAwake( a, true );
        for( int count : { 1, 4, 8 } )
            pair.Step( 1.0f / 60.0f, count );
        check( pair.getSplineRailJointData( id ).trackingValid, "changed substep counts keep the rail finite" );
        pair.destroyJoint( id );
        check( !pair.IsValid( id ), "rail destruction invalidates its handle" );
        pair.Step( 1.0f / 60.0f, 1 );
        check( pair.GetContactCount() == 1, "rail destruction restores connected collisions" );
        const vec2 line[] = { { -2, 0 }, { 2, 0 } };
        definition.points = line;
        definition.collideConnected = true;
        const auto reused = pair.createSplineRailJoint( definition );
        world foreign{};
        check( pair.IsValid( reused ) && !pair.IsValid( id ) && !foreign.IsValid( reused ), "rail slot generations and world ownership remain checked" );
        pair.Step( 1.0f / 60.0f, 1 );
        check( pair.GetContactCount() == 1, "rail can explicitly enable connected collision" );
        pair.DestroyBody( a );
        check( !pair.IsValid( reused ) && pair.GetBody( b ).jointCount == 0, "destroying the rail owner detaches the slider" );
    }
    {
        const vec2 crossing[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
        world simulation{};
        simulation.SetGravity( {} );
        const auto a = simulation.CreateBody();
        const auto b = simulation.CreateBody( bodyType::Dynamic, { Evaluate( { crossing }, 3, 0.9f ), {} } );
        const auto shape = simulation.CreateShape( b, circle2{ {}, 0.2f } );
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = crossing;
        const auto id = simulation.createSplineRailJoint( definition );
        check( simulation.IsValid( id ) && simulation.getSplineRailJointData( id ).parameter.segmentIndex == 3, "crossing fixture starts on the later branch" );
        simulation.SetBodyTransform( b, { { 0, 0 }, {} } );
        simulation.SetShapeDensity( shape, 2.0f );
        check( simulation.getSplineRailJointData( id ).parameter.segmentIndex >= 3, "mass reset retains the later crossing branch" );
        simulation.Step( 1.0f / 60.0f, 4 );
        check( simulation.getSplineRailJointData( id ).trackingValid && simulation.getSplineRailJointData( id ).parameter.segmentIndex >= 3, "solver never globally switches branches at a crossing" );
    }
    return 0;
}
