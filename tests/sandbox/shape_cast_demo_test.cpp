#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "demo.h"
#include "shapeCastDemo.h"

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

void near( float value, float expected, const char* message )
{
    check( std::fabs( value - expected ) < 0.004f, message );
}

std::unique_ptr<demo> createVisualization( demoKind kind )
{
    if( kind == demoKind::shapeCastVisualization ) return std::make_unique<shapeCastDemo>();
    return std::make_unique<shapeCastDemo>();
}

} // namespace

int main()
{
    check( getDemoEntry( demoKind::shapeCastVisualization ).kind == demoKind::shapeCastVisualization,
        "ShapeCast visualization must be in the demo selector" );

    shapeCastDemo scene{};
    const auto& simulation = scene.getWorld();
    check( simulation.GetShapeCount() == 4 && simulation.GetBodyCount() == 4,
        "Visualization owns exactly four independent targets" );
    for( const auto& target : scene.getTargets() )
        check( simulation.IsValid( target.shape ) && simulation.IsValid( target.body ),
            "Every visual target must have valid body and shape handles" );

    const auto initial = scene.getHit();
    check( initial.hit && initial.shape == scene.getTargets()[0].shape,
        "Default sweep hits first target circle" );
    near( initial.fraction, ( 4.0f - 0.8f - 0.45f - 0.005f ) / 11.0f,
        "Initial hit fraction is computed by world ShapeCast" );
    near( scene.getPreviewFraction(), 0.0f, "Initial preview is paused at start" );

    scene.step( 0.5f, 4 );
    near( scene.getPreviewFraction(), 0.2f, "Preview advances independent of physics" );
    scene.step( 100.0f, 1 );
    near( scene.getPreviewFraction(), initial.fraction, "Preview stops at first hit" );

    scene.resetPreview();
    near( scene.getPreviewFraction(), 0.0f, "Reset preview retains cast path" );
    scene.setTarget( { -3.0f, 0.0f } );
    check( !scene.getHit().hit, "Shortened sweep should miss target" );

    scene.setTarget( { 6.0f, 0.0f } );
    check( scene.getHit().hit, "Restored sweep should report a hit" );

    scene.setProbe( shapeCastProbe::capsule );
    check( std::holds_alternative<capsule2>( scene.getInput().geometry ) && scene.getHit().hit,
        "Capsule preset must use capsule geometry and produce a hit" );

    scene.setProbe( shapeCastProbe::box );
    check( std::holds_alternative<polygon2>( scene.getInput().geometry ) && scene.getHit().hit,
        "Box preset must use a convex polygon" );

    scene.setProbe( shapeCastProbe::circle );
    scene.setStart( { -5.0f, 2.0f } );
    scene.setTarget( { 5.0f, 2.0f } );
    const shapeId sensor = scene.getTargets()[3].shape;
    check( scene.getHit().hit && scene.getHit().shape == sensor, "Sensor is hit with default query filter" );

    scene.setSensorIncluded( false );
    check( !scene.getHit().hit, "Disabling the sensor's layer must omit it from cast" );

    scene.setSensorIncluded( true );
    check( scene.getHit().shape == sensor, "Re-enabling sensor restores hit" );

    scene.setCanEncroach( true );
    check( scene.getInput().canEncroach && scene.getHit().hit, "Encroachment flag reaches World query" );

    const auto unchanged = scene.getInput().translation;
    scene.handleInput( { .mousePosition = { -8.0f, 2.0f }, .mousePressed = true } );
    check( scene.getInput().translation.x != unchanged.x && !scene.getHit().hit,
        "Click changes the sweep destination and refreshes the hit" );

    scene.cancelInput();
    scene.step( 0.0f, 1 );
    check( scene.getPreviewFraction() == 0.0f, "Paused preview remains stable after input cancellation" );

    demoSession session{ createVisualization };
    session.selectDemo( demoKind::shapeCastVisualization );
    check( !session.isPlaying(), "Switch starts paused" );
    session.stepOnce( 4 );
    check( static_cast<shapeCastDemo&>( session.getDemo() ).getPreviewFraction() > 0.0f,
        "Shared one-frame stepping advances ShapeCast preview" );
    session.reset();
    check( !session.isPlaying() && static_cast<shapeCastDemo&>( session.getDemo() ).getPreviewFraction() == 0.0f,
        "Shared reset recreates the default cast without retaining progress" );
}
