#include <cmath>
#include <cstdio>
#include <cstdlib>
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

bool near( float a, float b )
{
    return std::abs( a - b ) < 0.00001f;
}

}

int main()
{
    const float h = 1.0f / 60.0f;

    for( const int subSteps : { 1, 4 } )
    {
        world simulation;
        simulation.SetGravity( {} );
        const bodyId rail = simulation.CreateBody();
        const bodyId slider = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, rot2::FromRadians( 0.3f ) } );
        ( void )simulation.CreateShape( slider, circle2{ {}, 0.25f } );

        prismaticJointDef definition{};
        definition.bodyA = rail;
        definition.bodyB = slider;
        definition.referenceAngle = 0.3f;
        const jointId id = simulation.createPrismaticJoint( definition );
        auto data = simulation.getPrismaticJointData( id );
        check( data.bodyA == rail && data.bodyB == slider && near( data.currentTranslation, 2.0f ) && near( data.lateralError, 0.0f ) && near( data.currentAngle, 0.0f ), "Prismatic query exposes rail frame and reference angle" );
        check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "Prismatic reaction starts empty before solve" );

        simulation.SetBodyLinearVelocity( slider, { 2.0f, 3.0f } );
        simulation.SetBodyAngularVelocity( slider, 4.0f );
        for( int i = 0; i < 60; ++i ) simulation.Step( h, subSteps );

        data = simulation.getPrismaticJointData( id );
        check( std::abs( simulation.GetBodyLinearVelocity( slider ).x - 2.0f ) < 0.0001f, "World Prismatic preserves axial velocity" );
        check( std::abs( data.lateralError ) < 0.002f && std::abs( data.currentAngle ) < 0.002f, "World Prismatic keeps slider on rail without relative rotation" );
        check( IsFinite( data.force ) && std::isfinite( data.torque ), "World Prismatic reports finite reaction" );

        simulation.SetBodyTransform( slider, { { 2.0f, 0.2f }, rot2::FromRadians( 0.4f ) } );
        data = simulation.getPrismaticJointData( id );
        check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "Prismatic pose change clears cached reaction" );

        simulation.destroyJoint( id );
        const jointId reused = simulation.createPrismaticJoint( definition );
        check( !simulation.IsValid( id ) && simulation.IsValid( reused ), "Prismatic slot reuse invalidates stale generation" );
        simulation.DestroyBody( slider );
        check( !simulation.IsValid( reused ) && simulation.GetBody( rail ).jointCount == 0, "Body destruction removes Prismatic from common joint graph" );
    }

    world pair;
    pair.SetGravity( {} );
    const bodyId a = pair.CreateBody( bodyType::Dynamic );
    const bodyId b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    prismaticJointDef pairDefinition{};
    pairDefinition.bodyA = a;
    pairDefinition.bodyB = b;
    const jointId pairJoint = pair.createPrismaticJoint( pairDefinition );
    pair.Step( h, 4 );
    check( pair.GetContactCount() == 0, "Prismatic uses common collideConnected exclusion" );

    world foreign;
    check( !foreign.IsValid( pairJoint ), "foreign World rejects Prismatic ID" );

    pair.destroyJoint( pairJoint );
    pair.Step( h, 4 );
    check( pair.GetContactCount() == 1, "Prismatic deletion restores stationary connected contact" );

    world limited;
    limited.SetGravity( {} );
    const bodyId limitRail = limited.CreateBody();
    const bodyId limitSlider = limited.CreateBody( bodyType::Dynamic, { { 0.9f, 0.0f }, {} } );
    ( void )limited.CreateShape( limitSlider, circle2{ {}, 0.25f } );
    prismaticJointDef limitDefinition{};
    limitDefinition.bodyA = limitRail;
    limitDefinition.bodyB = limitSlider;
    limitDefinition.enableLimit = true;
    limitDefinition.lowerTranslation = -1.0f;
    limitDefinition.upperTranslation = 1.0f;
    const jointId limitJoint = limited.createPrismaticJoint( limitDefinition );
    auto limitData = limited.getPrismaticJointData( limitJoint );
    check( limitData.enableLimit && near( limitData.lowerTranslation, -1.0f ) && near( limitData.upperTranslation, 1.0f ), "Prismatic create/query preserves translation limits" );

    limited.SetBodyLinearVelocity( limitSlider, { 20.0f, 0.0f } );
    limited.Step( h, 4 );
    limitData = limited.getPrismaticJointData( limitJoint );
    check( limitData.currentTranslation <= 1.002f, "Prismatic World predictive limit prevents upper boundary crossing" );

    limited.SetBodyTransform( limitSlider, { { 1.0f, 0.0f }, {} } );
    limited.SetBodyLinearVelocity( limitSlider, {} );
    limited.ApplyForceToCenter( limitSlider, { 100.0f, 0.0f } );
    limited.Step( h, 4 );
    limitData = limited.getPrismaticJointData( limitJoint );
    check( Dot( limitData.force, limitData.axis ) < 0.0f, "Prismatic upper limit reports negative axial reaction under sustained load" );

    limited.setPrismaticJointLimit( limitJoint, false, -1.0f, 1.0f );
    limitData = limited.getPrismaticJointData( limitJoint );
    check( !limitData.enableLimit && LengthSquared( limitData.force ) == 0.0f, "disabling Prismatic limit clears cached reaction" );
    limited.SetBodyLinearVelocity( limitSlider, { 20.0f, 0.0f } );
    limited.Step( h, 4 );
    check( limited.getPrismaticJointData( limitJoint ).currentTranslation > 1.05f, "disabled Prismatic limit restores free axial motion" );

    limited.SetBodyTransform( limitSlider, { { 0.4f, 0.0f }, {} } );
    limited.SetBodyAwake( limitSlider, false );
    limited.setPrismaticJointLimit( limitJoint, true, 0.0f, 0.0f );
    check( limited.IsBodyAwake( limitSlider ), "changing Prismatic limit wakes the connected component" );
    for( int i = 0; i < 60; ++i ) limited.Step( h, 4 );
    limitData = limited.getPrismaticJointData( limitJoint );
    check( std::abs( limitData.currentTranslation ) < 0.01f, "equal Prismatic limits hold the requested translation" );
    check( limitData.enableLimit && limitData.lowerTranslation == 0.0f && limitData.upperTranslation == 0.0f, "Prismatic setter updates queried limit range" );

    world motor;
    motor.SetGravity( {} );
    const bodyId motorRail = motor.CreateBody();
    const bodyId motorSlider = motor.CreateBody( bodyType::Dynamic );
    ( void )motor.CreateShape( motorSlider, circle2{ {}, 0.25f } );
    prismaticJointDef motorDefinition{};
    motorDefinition.bodyA = motorRail;
    motorDefinition.bodyB = motorSlider;
    motorDefinition.enableMotor = true;
    motorDefinition.motorSpeed = 2.0f;
    motorDefinition.maxMotorForce = 1000.0f;
    const jointId motorJoint = motor.createPrismaticJoint( motorDefinition );
    auto motorData = motor.getPrismaticJointData( motorJoint );
    check( motorData.enableMotor && near( motorData.motorSpeed, 2.0f ) && near( motorData.maxMotorForce, 1000.0f ) && motorData.motorForce == 0.0f, "Prismatic create/query preserves motor settings" );

    motor.Step( h, 4 );
    motorData = motor.getPrismaticJointData( motorJoint );
    check( std::abs( motor.GetBodyLinearVelocity( motorSlider ).x - 2.0f ) < 0.0001f, "Prismatic motor drives the requested positive axial speed" );

    motor.setPrismaticJointMotor( motorJoint, true, -1.0f, 1000.0f );
    motorData = motor.getPrismaticJointData( motorJoint );
    check( motorData.motorForce == 0.0f, "changing Prismatic motor clears cached motor reaction" );
    motor.Step( h, 4 );
    check( std::abs( motor.GetBodyLinearVelocity( motorSlider ).x + 1.0f ) < 0.0001f, "Prismatic motor reverses live" );

    motor.setPrismaticJointMotor( motorJoint, true, 0.0f, 1000.0f );
    motor.Step( h, 4 );
    check( std::abs( motor.GetBodyLinearVelocity( motorSlider ).x ) < 0.0001f, "zero-speed Prismatic motor brakes live axial motion" );

    motor.SetBodyAwake( motorSlider, false );
    motor.setPrismaticJointMotor( motorJoint, true, 1.0f, 1000.0f );
    check( motor.IsBodyAwake( motorSlider ), "changing Prismatic motor wakes the connected component" );

    motor.setPrismaticJointLimit( motorJoint, true, -0.5f, 0.5f );
    motor.SetBodyTransform( motorSlider, { { 0.5f, 0.0f }, {} } );
    motor.SetBodyLinearVelocity( motorSlider, {} );
    motor.setPrismaticJointMotor( motorJoint, true, 5.0f, 1000.0f );
    motor.Step( h, 4 );
    motorData = motor.getPrismaticJointData( motorJoint );
    check( std::abs( motor.GetBodyLinearVelocity( motorSlider ).x ) < 0.001f, "Prismatic upper limit removes motor-driven outward velocity" );
    check( motorData.currentTranslation < 0.53f, "Prismatic soft limit bounds the first motor-driven position error" );
    check( motorData.motorForce > 0.0f, "Prismatic reports motor effort even when the limit cancels motion" );
    for( int i = 0; i < 60; ++i ) motor.Step( h, 4 );
    motorData = motor.getPrismaticJointData( motorJoint );
    check( std::abs( motor.GetBodyLinearVelocity( motorSlider ).x ) < 0.001f, "Prismatic upper limit keeps sustained motor velocity blocked" );
    check( motorData.currentTranslation >= 0.5f && motorData.currentTranslation < 0.53f, "Prismatic soft limit keeps sustained motor position error bounded" );
    check( motorData.motorForce > 0.0f, "Prismatic motor keeps applying effort against active limit" );
    check( std::abs( Dot( motorData.force, motorData.axis ) ) < 1.0f, "Prismatic motor and upper limit reactions balance at steady state" );

    return EXIT_SUCCESS;
}
