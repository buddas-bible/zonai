#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
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
    const auto firstRay = scene.getRayHit();
    check( firstRay.hit && firstRay.shape == initial.shape && firstRay.fraction > initial.fraction,
        "RayCast uses the same path but hits later than a finite-size probe" );
    near( firstRay.fraction, ( 4.0f - 0.8f ) / 11.0f, "Center ray hits the circle surface" );
    check( scene.getAABBCandidateCount() == 2, "Swept AABB includes circle and box broadphase candidates" );

    scene.setRayVisible( false );
    scene.setSweepAABBVisible( false );
    check( !scene.isRayVisible() && !scene.isSweepAABBVisible() && scene.getRayHit().hit,
        "Overlay visibility must not change the query results" );
    scene.setRayVisible( true );
    scene.setSweepAABBVisible( true );
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
    check( !scene.getHit().hit && !scene.getRayHit().hit && scene.getAABBCandidateCount() == 0,
        "Sensor filter must apply equally to ShapeCast, RayCast and AABB query" );

    scene.setSensorIncluded( true );
    check( scene.getHit().shape == sensor && scene.getRayHit().shape == sensor &&
        scene.getAABBCandidateCount() == 1, "Sensor is restored in all three spatial queries" );

    scene.setCanEncroach( true );
    check( scene.getInput().canEncroach && scene.getHit().hit, "Encroachment flag reaches World query" );

    const auto unchanged = scene.getInput().translation;
    scene.handleInput( { .mousePosition = { -8.0f, 2.0f }, .mousePressed = true } );
    check( scene.getInput().translation.x != unchanged.x && !scene.getHit().hit,
        "Click changes the sweep destination and refreshes the hit" );

    // RayCast는 선만 검사하므로 원 반지름 안쪽을 스치지 않는 경로도 ShapeCast에는 충돌함.
    scene.setStart( { -5.0f, 1.1f } );
    scene.setTarget( { 6.0f, 1.1f } );
    check( scene.getHit().hit && scene.getHit().shape == scene.getTargets()[0].shape &&
        !scene.getRayHit().hit, "ShapeCast hits a near-miss path where the center ray misses" );
    check( scene.getAABBCandidateCount() >= 1, "Swept AABB still returns broadphase candidates" );

    const vec2 beforeShiftEnd = scene.getInput().transform.position + scene.getInput().translation;
    scene.handleInput( { .mousePosition = { -6.0f, 1.1f }, .mousePressed = true, .selectTrialPosition = true } );
    check( LengthSquared( scene.getInput().transform.position - vec2{ -6.0f, 1.1f } ) == 0.0f &&
           LengthSquared( scene.getInput().transform.position + scene.getInput().translation - beforeShiftEnd ) < 1.0e-6f,
           "Shift+left click changes start without moving destination" );

    scene.cancelInput();
    scene.step( 0.0f, 1 );
    check( scene.getPreviewFraction() == 0.0f, "Paused preview remains stable after input cancellation" );

    // 회전 0에서는 기존 선형 World ShapeCast를 그대로 사용함.
    shapeCastDemo rotating{};
    rotating.setProbe( shapeCastProbe::box );
    rotating.setStart( { -1.0f, 1.25f } );
    rotating.setTarget( { -1.0f, 1.25f } );
    check( rotating.getRotationRadians() == 0.0f && !rotating.getHit().hit,
        "Default Sandbox query remains the original linear ShapeCast" );

    constexpr float HALF_PI = 1.5707963267948966f;
    rotating.setRotationRadians( HALF_PI );
    check( rotating.getRotationRadians() == HALF_PI,
        "Angle control must accept a quarter turn" );
    const transform2 rotatedEnd = rotating.getPoseAtFraction( 1.0f );
    near( rotatedEnd.rotation.c, 0.0f, "Rotating preview reaches final cosine" );
    near( rotatedEnd.rotation.s, 1.0f, "Rotating preview reaches final sine" );
    near( rotatedEnd.position.x, -1.0f, "Pure rotation preserves the origin X" );
    near( rotatedEnd.position.y, 1.25f, "Pure rotation preserves the origin Y" );

    const transform2 middlePose = rotating.getPoseAtFraction( 0.5f );
    check( middlePose.rotation.c > 0.5f && middlePose.rotation.s > 0.5f,
        "Preview must interpolate the TOI rotation through the middle pose" );

    const aabb2 sweepBounds = rotating.getSweepAABB();
    const aabb2 middleBounds = ComputeShapeAABB( rotating.getInput().geometry, middlePose );
    check( sweepBounds.min.x <= middleBounds.min.x && sweepBounds.max.x >= middleBounds.max.x &&
        sweepBounds.min.y <= middleBounds.min.y && sweepBounds.max.y >= middleBounds.max.y,
        "Rotational sweep bounds must contain an intermediate rotated shape" );
    const worldShapeCastResult2 rotatingHit = rotating.getHit();
    const worldShapeCastResult2 directHit = rotating.getWorld().castShapeSweepClosest(
        { rotating.getInput().geometry, rotating.getInput().transform,
          rotating.getInput().translation, rotating.getRotationRadians() } );
    check( rotatingHit.hit == directHit.hit && rotatingHit.shape == directHit.shape &&
        rotatingHit.fraction == directHit.fraction && rotatingHit.hit,
        "Sandbox rotation must present the actual World TOI hit" );
    check( rotating.getAABBCandidateCount() >= 1,
        "Rotating broadphase overlay must include the nearby target" );
    rotating.step( 0.25f, 1 );
    check( rotating.getPreviewFraction() > 0.0f &&
        rotating.getPreviewFraction() <= rotating.getHit().fraction,
        "Rotating preview advances only to the first TOI hit" );

    rotating.setRotationRadians( 2.0f );
    check( rotating.getRotationRadians() == HALF_PI,
        "Angles beyond World sweep limits must not change the demo" );
    rotating.setRotationRadians( std::numeric_limits<float>::quiet_NaN() );
    check( rotating.getRotationRadians() == HALF_PI,
        "NaN angle must not corrupt the sweep" );
    rotating.setRotationRadians( 0.0f );
    check( rotating.getRotationRadians() == 0.0f && !rotating.getHit().hit &&
        rotating.getPreviewFraction() == 0.0f,
        "Resetting rotation must restore the previous linear query" );

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
