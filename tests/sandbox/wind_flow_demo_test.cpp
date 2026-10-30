#include "rigidBodyDemo.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

bool near( vec2 a, vec2 b, float tolerance = 1.0e-3f )
{
    return Length( a - b ) < tolerance;
}

void checkFluidCoupling()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    const fluidField2* field = scene.getWindField();
    check( field != nullptr && field->getWidth() == 24 && field->getHeight() == 14,
        "wind demo must own its separate MAC grid" );
    check( scene.getShapes().size() == 3, "wind demo must show three dynamic rigid bodies" );
    check( near( scene.getWorld().GetGravity(), {} ), "wind trial should initially be zero gravity" );

    const bodyId first = scene.getShapes()[0].bodyHandle;
    const vec2 initial = scene.getWorld().GetBodyTransform( first ).position;

    // 단방향 coupling이 없으면 유체만 움직이고 Body는 정지해야 함.
    scene.setWindObstacleCouplingEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    for( int i = 0; i < 120; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "zero drag rate must decouple the wind field from body motion" );

    const vec2 flow = field->sampleVelocity( { -3.0f, 0.0f } );
    check( IsFinite( flow ) && Length( flow ) > 1.0e-4f,
        "wind source must create an observable finite velocity field" );
    check( std::isfinite( field->getDivergenceRms() ) && field->getDivergenceRms() < 0.25f,
        "running wind demo must keep pressure projection residual bounded" );

    scene.resetWindFlow();
    check( field->getDivergenceRms() == 0.0f && near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "wind reset must clear velocity and restore body transforms" );

    // 소스가 꺼진 빈 유체는 강체를 가속하지 않아야 함.
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 3.0f );
    for( int i = 0; i < 60; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "empty field with source disabled must leave bodies at rest" );

    scene.setWindSourceEnabled( true );
    scene.setWindSourceStrength( 25.0f );
    for( int i = 0; i < 300; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( Length( scene.getWorld().GetBodyTransform( first ).position - initial ) > 0.01f,
        "fluid drag must move a body in the flow" );

    scene.resetWindFlow();
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ) &&
        near( scene.getWorld().GetBodyLinearVelocity( first ), {} ) &&
        scene.getWindSourceStrength() == 25.0f && scene.getWindDragCoefficient() == 3.0f,
        "wind reset must restore motion while preserving edited parameters" );
}

void checkMovingSolidReaction()
{
    rigidBodyDemo disabled{ demoKind::windFlow };
    rigidBodyDemo enabled{ demoKind::windFlow };
    const bodyId bodyDisabled = disabled.getShapes()[0].bodyHandle;
    const bodyId bodyEnabled = enabled.getShapes()[0].bodyHandle;

    for( rigidBodyDemo* scene : { &disabled, &enabled } )
    {
        scene->setWindSourceEnabled( false );
        scene->setWindDragCoefficient( 0.0f );
        scene->getWorld().SetBodyLinearVelocity( scene->getShapes()[0].bodyHandle, { 2.0f, 0.0f } );
    }
    disabled.setWindObstacleCouplingEnabled( false );
    enabled.setWindObstacleCouplingEnabled( true );

    disabled.step( 1.0f / 60.0f, 4 );
    enabled.step( 1.0f / 60.0f, 4 );

    check( disabled.getWindField()->getSolidCellCount() == 0 &&
        enabled.getWindField()->getSolidCellCount() > 0,
        "bidirectional toggle must control solid occupancy" );
    check( near( disabled.getWorld().GetBodyLinearVelocity( bodyDisabled ), { 2.0f, 0.0f } ),
        "disabled feedback must preserve momentum in an empty fluid" );
    check( enabled.getWindPressureForces()[0].force.x < -0.001f &&
        enabled.getWorld().GetBodyLinearVelocity( bodyEnabled ).x <
            disabled.getWorld().GetBodyLinearVelocity( bodyDisabled ).x,
        "a moving solid must stir fluid and receive opposite pressure force" );

    const vec2 wake = enabled.getWindField()->sampleVelocity( { -1.0f, 0.0f } );
    check( IsFinite( wake ) && Length( wake ) > 0.0001f,
        "moving solid must change the velocity field without a wind source" );

    enabled.resetWindFlow();
    check( enabled.getWindField()->getSolidCellCount() == 0 &&
        near( enabled.getWindPressureForces()[0].force, {} ) &&
        near( enabled.getWorld().GetBodyLinearVelocity( bodyEnabled ), {} ),
        "wind reset must clear pressure feedback and solid mask" );
    check( enabled.isWindObstacleCouplingEnabled(), "wind reset must preserve coupling setting" );
}

} // namespace

int main()
{
    checkFluidCoupling();
    checkMovingSolidReaction();
}
