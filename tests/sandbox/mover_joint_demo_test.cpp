#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "demo.h"
#include "rigidBodyDemo.h"

using namespace zonai;
using namespace zonai::sandbox;

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
    check( getDemoEntries().size() == 12, "Mover Joint demo is independently selectable" );
    check( std::string_view{ getDemoEntry( demoKind::moverJointPlayground ).category } == "조인트", "Mover Joint demo stays in joint category" );

    rigidBodyDemo mover{ demoKind::moverJointPlayground };
    check( mover.getWorld().GetBodyCount() == 2 && mover.getWorld().getJointCount() == 1 && mover.getShapes().size() == 2, "Mover scene has reference and driven bodies" );
    check( mover.getWorld().IsValid( mover.getMoverJoint() ), "Mover scene owns a valid Mover Joint" );

    mover.applyMoverJointPreset( moverJointDemoPreset::horizontal );
    auto data = mover.getWorld().getMoverJointData( mover.getMoverJoint() );
    check( data.linearVelocity.x > 0.0f && near( data.linearVelocity.y, 0.0f ), "Mover horizontal preset isolates x velocity" );
    check( data.maxVelocityForce.x > 0.0f && data.maxVelocityForce.y > 0.0f, "Mover horizontal preset keeps finite axis forces" );

    mover.applyMoverJointPreset( moverJointDemoPreset::vertical );
    data = mover.getWorld().getMoverJointData( mover.getMoverJoint() );
    check( near( data.linearVelocity.x, 0.0f ) && data.linearVelocity.y > 0.0f, "Mover vertical preset isolates y velocity" );

    mover.applyMoverJointPreset( moverJointDemoPreset::diagonal );
    data = mover.getWorld().getMoverJointData( mover.getMoverJoint() );
    check( data.linearVelocity.x > 0.0f && data.linearVelocity.y > 0.0f, "Mover diagonal preset drives both axes" );

    mover.applyMoverJointPreset( moverJointDemoPreset::anisotropic );
    data = mover.getWorld().getMoverJointData( mover.getMoverJoint() );
    check( data.linearVelocity.x > 0.0f && data.linearVelocity.y > 0.0f, "Mover anisotropic preset keeps a two-axis target" );
    check( data.maxVelocityForce.x > data.maxVelocityForce.y && data.maxVelocityForce.y > 0.0f, "Mover anisotropic preset demonstrates independent axis force budgets" );

    mover.setMoverJointSettings( { -1.25f, 0.75f }, { 12.0f, 4.0f } );
    data = mover.getWorld().getMoverJointData( mover.getMoverJoint() );
    check( near( data.linearVelocity.x, -1.25f ) && near( data.linearVelocity.y, 0.75f ), "Mover inspector applies target linear velocity" );
    check( near( data.maxVelocityForce.x, 12.0f ) && near( data.maxVelocityForce.y, 4.0f ), "Mover inspector applies per-axis force limits" );

    mover.applyMoverJointPreset( moverJointDemoPreset::horizontal );
    data = mover.getWorld().getMoverJointData( mover.getMoverJoint() );
    const bodyId driven = data.bodyB;
    mover.step( 1.0f / 60.0f, 1 );
    check( mover.getWorld().GetBodyLinearVelocity( driven ).x > 0.0f, "Mover demo visibly drives the target body" );
    check( near( mover.getWorld().GetBodyAngularVelocity( driven ), 0.0f ), "Mover demo leaves target rotation untouched" );

    return EXIT_SUCCESS;
}
