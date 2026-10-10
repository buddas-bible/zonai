#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/motorJointConstraint2.h"
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
    return std::abs( a - b ) < 0.0001f;
}

}

int main()
{
    bodySim bodyA{};
    bodyA.bodyId = 0;
    bodyA.invMass = 1.0f;
    bodyA.invInertia = 1.0f;

    bodySim bodyB{};
    bodyB.bodyId = 1;
    bodyB.invMass = 1.0f;
    bodyB.invInertia = 1.0f;

    motorJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.linearVelocity = { 2.0f, -1.0f };
    joint.maxVelocityForce = 1000.0f;
    joint.angularVelocity = 3.0f;
    joint.maxVelocityTorque = 1000.0f;

    const float h = 1.0f / 60.0f;
    auto constraint = prepareMotorJointConstraint( joint, bodyA, bodyB, h );
    bodyState stateA{};
    bodyState stateB{};

    solveMotorJointConstraint( constraint, stateA, stateB );
    check( near( stateB.linearVelocity.x - stateA.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.y - stateA.linearVelocity.y, -1.0f ), "motor reaches desired relative linear velocity" );
    check( near( stateB.angularVelocity - stateA.angularVelocity, 3.0f ), "motor reaches desired relative angular velocity" );

    joint.linearVelocity = { 10.0f, 0.0f };
    joint.maxVelocityForce = 6.0f;
    joint.angularVelocity = 10.0f;
    joint.maxVelocityTorque = 12.0f;
    constraint = prepareMotorJointConstraint( joint, bodyA, bodyB, h );
    stateA = {};
    stateB = {};
    solveMotorJointConstraint( constraint, stateA, stateB );
    check( near( Length( constraint.linearVelocityImpulse ), joint.maxVelocityForce * h ), "linear motor impulse is limited by max force times h" );
    check( near( std::abs( constraint.angularVelocityImpulse ), joint.maxVelocityTorque * h ), "angular motor impulse is limited by max torque times h" );

    // Linear / angular Motor를 동시에 켜고 anchor가 COM 밖에 있으면 두 제약은 서로 결합됨.
    // 순차 angular -> linear solve만 두 번 호출해서는 linear impulse가 다시 만든 angular error가 남음.
    bodySim coupledBodyA{};
    coupledBodyA.bodyId = 2;
    bodySim coupledBodyB{};
    coupledBodyB.bodyId = 3;
    coupledBodyB.invMass = 1.0f;
    coupledBodyB.invInertia = 1.0f;

    motorJointSim2 coupledJoint{};
    coupledJoint.jointId = 1;
    coupledJoint.bodyIdA = 2;
    coupledJoint.bodyIdB = 3;
    coupledJoint.localAnchorB = { 0.0f, 1.0f };
    coupledJoint.linearVelocity = { 1.0f, 0.0f };
    coupledJoint.maxVelocityForce = 1000.0f;
    coupledJoint.angularVelocity = 0.0f;
    coupledJoint.maxVelocityTorque = 1000.0f;

    auto coupledConstraint = prepareMotorJointConstraint( coupledJoint, coupledBodyA, coupledBodyB, h );
    bodyState coupledStateA{};
    bodyState coupledStateB{};
    // World의 현재 Joint solve와 같은 두 번의 pass를 재현함.
    solveMotorJointConstraint( coupledConstraint, coupledStateA, coupledStateB );
    solveMotorJointConstraint( coupledConstraint, coupledStateA, coupledStateB );
    const vec2 coupledPointVelocity = coupledStateB.linearVelocity + Cross( coupledStateB.angularVelocity, coupledConstraint.anchorB );
    check( near( coupledPointVelocity.x, coupledJoint.linearVelocity.x ) && near( coupledPointVelocity.y, coupledJoint.linearVelocity.y ), "off-center combined Motor preserves linear target" );
    check( near( coupledStateB.angularVelocity - coupledStateA.angularVelocity, coupledJoint.angularVelocity ), "off-center combined Motor preserves angular target" );

    // World가 Motor를 공용 Joint graph / solver 경로에 넣고 query까지 되돌려주는지 검증함.
    world simulation;
    simulation.SetGravity( {} );
    const auto ground = simulation.CreateBody();
    const auto driven = simulation.CreateBody( bodyType::Dynamic );
    ( void )simulation.CreateShape( driven, circle2{ {}, 0.5f } );

    motorJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = driven;
    definition.linearVelocity = { 1.5f, -0.5f };
    definition.maxVelocityForce = 1000.0f;
    definition.angularVelocity = 2.0f;
    definition.maxVelocityTorque = 1000.0f;
    const auto motorId = simulation.createMotorJoint( definition );
    auto data = simulation.getMotorJointData( motorId );
    check( near( data.linearVelocity.x, definition.linearVelocity.x ) && near( data.linearVelocity.y, definition.linearVelocity.y ) && near( data.angularVelocity, definition.angularVelocity ), "Motor create/query preserves target velocities" );
    check( simulation.getJointCount() == 1 && simulation.GetBody( driven ).jointCount == 1, "Motor shares common joint graph" );

    simulation.Step( 1.0f / 60.0f, 1 );
    check( near( simulation.GetBodyLinearVelocity( driven ).x, definition.linearVelocity.x ) && near( simulation.GetBodyLinearVelocity( driven ).y, definition.linearVelocity.y ), "World Motor reaches target linear velocity" );
    check( near( simulation.GetBodyAngularVelocity( driven ), definition.angularVelocity ), "World Motor reaches target angular velocity" );
    data = simulation.getMotorJointData( motorId );
    check( IsFinite( data.force ) && std::isfinite( data.torque ) && LengthSquared( data.force ) > 0.0f && std::abs( data.torque ) > 0.0f, "Motor stores finite linear and angular reactions" );

    const float torqueBeforeLinearChange = data.torque;
    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointLinearVelocity( motorId, { -1.0f, 0.5f }, 500.0f );
    check( simulation.IsBodyAwake( driven ), "changing Motor linear settings wakes connected body" );
    data = simulation.getMotorJointData( motorId );
    check( near( data.linearVelocity.x, -1.0f ) && near( data.linearVelocity.y, 0.5f ) && near( data.maxVelocityForce, 500.0f ), "Motor linear setter updates query state" );
    check( LengthSquared( data.force ) == 0.0f && near( data.torque, torqueBeforeLinearChange ), "Motor linear setter clears only linear cache" );

    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointLinearVelocity( motorId, { -1.0f, 0.5f }, 500.0f );
    check( !simulation.IsBodyAwake( driven ), "unchanged Motor linear settings do not wake body" );

    simulation.SetBodyAwake( driven, true );
    simulation.Step( 1.0f / 60.0f, 1 );
    const auto rebuilt = simulation.getMotorJointData( motorId );
    check( LengthSquared( rebuilt.force ) > 0.0f, "Motor rebuilds linear reaction after setter change" );

    const vec2 forceBeforeAngularChange = rebuilt.force;
    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointAngularVelocity( motorId, -2.0f, 700.0f );
    check( simulation.IsBodyAwake( driven ), "changing Motor angular settings wakes connected body" );
    data = simulation.getMotorJointData( motorId );
    check( near( data.angularVelocity, -2.0f ) && near( data.maxVelocityTorque, 700.0f ), "Motor angular setter updates query state" );
    check( data.torque == 0.0f && near( data.force.x, forceBeforeAngularChange.x ) && near( data.force.y, forceBeforeAngularChange.y ), "Motor angular setter clears only angular cache" );

    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointAngularVelocity( motorId, -2.0f, 700.0f );
    check( !simulation.IsBodyAwake( driven ), "unchanged Motor angular settings do not wake body" );

    simulation.SetBodyTransform( driven, { { 1.0f, 0.0f }, {} } );
    data = simulation.getMotorJointData( motorId );
    check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "pose change clears Motor warm-start cache" );

    simulation.destroyJoint( motorId );
    check( !simulation.IsValid( motorId ) && simulation.getJointCount() == 0, "destroy invalidates Motor handle" );
    const auto reused = simulation.createMotorJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( motorId ), "Motor slot reuse preserves generation validation" );
    simulation.DestroyBody( driven );
    check( !simulation.IsValid( reused ) && simulation.GetBody( ground ).jointCount == 0, "body destruction removes connected Motor" );

    world pair;
    pair.SetGravity( {} );
    const auto a = pair.CreateBody( bodyType::Dynamic );
    const auto b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1, "overlapping bodies initially create a Contact" );

    definition = {};
    definition.bodyA = a;
    definition.bodyB = b;
    const auto pairJoint = pair.createMotorJoint( definition );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 0, "Motor connected collision suppression uses shared joint path" );

    pair.SetBodyAwake( a, false );
    check( !pair.IsBodyAwake( b ), "Motor connected dynamic bodies sleep together" );
    pair.setMotorJointLinearVelocity( pairJoint, { 1.0f, 0.0f }, 10.0f );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "Motor setter wake propagates through joint graph" );

    pair.destroyJoint( pairJoint );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1, "destroying Motor restores candidate contacts" );

    definition.collideConnected = true;
    const auto colliding = pair.createMotorJoint( definition );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1 && pair.getMotorJointData( colliding ).collideConnected, "explicit Motor connected collisions remain enabled" );

    world foreign;
    check( !foreign.IsValid( colliding ), "foreign World rejects Motor handle" );

    return EXIT_SUCCESS;
}
