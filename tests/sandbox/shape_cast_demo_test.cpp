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

    // 시작 회전과 상대 회전은 독립적이며, 상대 회전 0은 기울어진 선형 ShapeCast임.
    shapeCastDemo oriented{};
    oriented.setProbe( shapeCastProbe::box );
    constexpr float PI = 3.14159265358979323846f;
    constexpr float QUARTER_PI = 0.7853981633974483f;
    constexpr float SIXTH_PI = 0.5235987755982989f;
    check( oriented.getStartRotationRadians() == 0.0f &&
        oriented.getRotationRadians() == 0.0f,
        "Default start and relative rotations must be zero" );
    const worldShapeCastResult2 unrotatedBoxHit = oriented.getHit();
    check( unrotatedBoxHit.hit, "Unrotated reference Box cast must hit a target" );

    oriented.setStartRotationRadians( QUARTER_PI );
    near( oriented.getStartRotationRadians(), QUARTER_PI,
        "Start rotation is independently stored" );
    near( oriented.getInput().transform.rotation.c, std::cos( QUARTER_PI ),
        "Linear ShapeCast receives the rotated start cosine" );
    near( oriented.getInput().transform.rotation.s, std::sin( QUARTER_PI ),
        "Linear ShapeCast receives the rotated start sine" );
    check( oriented.getRotationRadians() == 0.0f && oriented.getSweepStats().toiCalls == 0,
        "An initial orientation alone must keep the existing linear GJK query" );

    const auto linearHit = oriented.getWorld().castShapeClosest( oriented.getInput() );
    const auto orientedHit = oriented.getHit();
    check( orientedHit.hit && orientedHit.hit == linearHit.hit &&
        orientedHit.shape == linearHit.shape && orientedHit.fraction == linearHit.fraction,
        "Initially tilted Sandbox box must match the direct World linear ShapeCast" );
    check( std::fabs( orientedHit.fraction - unrotatedBoxHit.fraction ) > 0.002f,
        "Tilting the initial Box must affect the first collision fraction" );

    const transform2 tiltedStart = oriented.getPoseAtFraction( 0.0f );
    const transform2 tiltedEnd = oriented.getPoseAtFraction( 1.0f );
    near( tiltedStart.rotation.c, tiltedEnd.rotation.c,
        "Linear sweep preserves the starting cosine" );
    near( tiltedStart.rotation.s, tiltedEnd.rotation.s,
        "Linear sweep preserves the starting sine" );
    near( tiltedEnd.position.x,
        oriented.getInput().transform.position.x + oriented.getInput().translation.x,
        "Initially tilted linear sweep reaches the requested X destination" );
    const aabb2 tiltedBounds = oriented.getSweepAABB();
    const aabb2 expectedStartBounds = ComputeShapeAABB(
        oriented.getInput().geometry, oriented.getInput().transform );
    check( tiltedBounds.min.x <= expectedStartBounds.min.x &&
        tiltedBounds.min.y <= expectedStartBounds.min.y &&
        tiltedBounds.max.x >= expectedStartBounds.max.x &&
        tiltedBounds.max.y >= expectedStartBounds.max.y,
        "Initially tilted linear swept AABB contains the rotated starting shape" );

    oriented.step( 0.1f, 1 );
    check( oriented.getPreviewFraction() > 0.0f, "Initial-angle preview can advance" );
    oriented.setRotationRadians( SIXTH_PI );
    near( oriented.getPreviewFraction(), 0.0f,
        "Adding relative rotation restarts the preview" );
    const transform2 combinedStart = oriented.getPoseAtFraction( 0.0f );
    const transform2 combinedEnd = oriented.getPoseAtFraction( 1.0f );
    near( combinedStart.rotation.c, std::cos( QUARTER_PI ),
        "Relative rotation does not modify the initial orientation" );
    near( combinedEnd.rotation.c, std::cos( QUARTER_PI + SIXTH_PI ),
        "Combined sweep ends at start angle plus relative angle cosine" );
    near( combinedEnd.rotation.s, std::sin( QUARTER_PI + SIXTH_PI ),
        "Combined sweep ends at start angle plus relative angle sine" );
    const auto expectedSweepHit = oriented.getWorld().castShapeSweepClosest(
        { oriented.getInput().geometry, oriented.getInput().transform,
          oriented.getInput().translation, oriented.getRotationRadians() } );
    check( oriented.getHit().hit == expectedSweepHit.hit &&
        oriented.getHit().shape == expectedSweepHit.shape &&
        oriented.getHit().fraction == expectedSweepHit.fraction &&
        oriented.getSweepStats().toiCalls > 0,
        "Initially rotated TOI Sandbox query matches the direct World sweep" );

    const aabb2 combinedBounds = oriented.getSweepAABB();
    for( int step = 0; step <= 4; ++step )
    {
        const transform2 pose = oriented.getPoseAtFraction( step * 0.25f );
        const aabb2 bounds = ComputeShapeAABB( oriented.getInput().geometry, pose );
        check( combinedBounds.min.x <= bounds.min.x &&
            combinedBounds.min.y <= bounds.min.y &&
            combinedBounds.max.x >= bounds.max.x &&
            combinedBounds.max.y >= bounds.max.y,
            "Combined rotating swept bounds contain each intermediate orientation" );
    }

    oriented.setStartRotationRadians( PI + 0.1f );
    check( oriented.getStartRotationRadians() == QUARTER_PI,
        "Start rotation beyond ±180 degrees is rejected" );
    oriented.setStartRotationRadians( std::numeric_limits<float>::quiet_NaN() );
    check( oriented.getStartRotationRadians() == QUARTER_PI,
        "Invalid start rotation must not change the Sandbox" );
    oriented.setStartRotationRadians( PI );
    near( oriented.getInput().transform.rotation.c, -1.0f,
        "Positive 180-degree start angle is accepted" );
    oriented.setStartRotationRadians( -PI );
    near( oriented.getInput().transform.rotation.c, -1.0f,
        "Negative 180-degree start angle is accepted" );

    oriented.setRotationRadians( 0.0f );
    oriented.setStartRotationRadians( 0.0f );
    check( oriented.getStartRotationRadians() == 0.0f &&
        oriented.getRotationRadians() == 0.0f &&
        oriented.getSweepStats().toiCalls == 0,
        "Both rotation controls reset to the original linear query" );

    // 로컬 형상 중심을 회전축에서 옮기면 제자리 회전만으로도 원호를 그려 충돌함.
    shapeCastDemo offset{};
    offset.setSensorIncluded( false );
    offset.setStart( { -1.0f, 1.8f } );
    offset.setTarget( { -1.0f, 1.8f } );
    check( !offset.getHit().hit && LengthSquared( offset.getLocalOffset() ) == 0.0f,
        "Centered probe at a stationary pivot must not hit" );

    offset.setLocalOffset( { 1.0f, 0.0f } );
    const circle2 shiftedCircle = std::get<circle2>( offset.getInput().geometry );
    near( shiftedCircle.center.x, 1.0f, "Circle local offset changes its center" );
    near( shiftedCircle.center.y, 0.0f, "Circle local offset preserves Y" );
    check( !offset.getHit().hit, "Local offset alone does not move a stationary cast" );

    offset.setRotationRadians( -HALF_PI );
    const auto arcHit = offset.getHit();
    const auto directArcHit = offset.getWorld().castShapeSweepClosest(
        { offset.getInput().geometry, offset.getInput().transform,
          offset.getInput().translation, offset.getRotationRadians() },
        { .maskBits = ~std::uint64_t{ 2 } } );
    check( arcHit.hit && arcHit.shape == offset.getTargets()[0].shape &&
        arcHit.fraction > 0.0f && arcHit.fraction < 1.0f &&
        arcHit.shape == directArcHit.shape && arcHit.fraction == directArcHit.fraction,
        "Off-center circle must hit by pure rotation and match direct World TOI" );
    near( offset.getPoseAtFraction( 1.0f ).position.y, 1.8f,
        "Pure pivot rotation must not translate the body origin" );
    const vec2 endCircleCenter = TransformPoint(
        offset.getPoseAtFraction( 1.0f ), shiftedCircle.center );
    near( endCircleCenter.x, -1.0f, "Offset circle traces the quarter arc to target X" );
    near( endCircleCenter.y, 0.8f, "Offset circle traces the quarter arc to target Y" );

    offset.setRotationRadians( 0.0f );
    offset.setProbe( shapeCastProbe::capsule );
    const capsule2 shiftedCapsule = std::get<capsule2>( offset.getInput().geometry );
    near( shiftedCapsule.center1.x, 1.0f, "Capsule first endpoint includes local offset" );
    near( shiftedCapsule.center2.x, 1.0f, "Capsule second endpoint includes local offset" );
    near( shiftedCapsule.center1.y, -0.45f, "Capsule preserves lower endpoint" );
    near( shiftedCapsule.center2.y, 0.45f, "Capsule preserves upper endpoint" );

    offset.setProbe( shapeCastProbe::box );
    const polygon2 shiftedBox = std::get<polygon2>( offset.getInput().geometry );
    near( shiftedBox.centroid.x, 1.0f, "Box polygon centroid includes local offset" );
    near( shiftedBox.vertices[0].x, 0.5f, "Box vertices shift with centroid" );
    const auto& shiftedInput = offset.getInput();
    const auto directLinearHit = offset.getWorld().castShapeClosest(
        shiftedInput, { .maskBits = ~std::uint64_t{ 2 } } );
    check( directLinearHit.hit == offset.getHit().hit &&
        directLinearHit.fraction == offset.getHit().fraction,
        "Offset polygon linear cast must match direct World query" );

    offset.step( 0.1f, 1 );
    check( offset.getPreviewFraction() > 0.0f, "Local-offset preview can advance" );
    offset.setLocalOffset( { 0.5f, -0.25f } );
    check( offset.getPreviewFraction() == 0.0f,
        "Changing local offset must restart the preview" );
    const vec2 validOffset = offset.getLocalOffset();
    offset.setLocalOffset( { 3.1f, 0.0f } );
    offset.setLocalOffset( { std::numeric_limits<float>::quiet_NaN(), 0.0f } );
    check( LengthSquared( offset.getLocalOffset() - validOffset ) == 0.0f,
        "Out-of-range and non-finite local offsets must be rejected" );
    offset.setLocalOffset( {} );
    check( LengthSquared( offset.getLocalOffset() ) == 0.0f &&
        std::get<polygon2>( offset.getInput().geometry ).centroid.x == 0.0f,
        "Resetting the offset restores a centered Box" );

    shapeCastDemo dense{};
    dense.setProbe( shapeCastProbe::box );
    dense.setRotationRadians( 0.6f );
    const worldShapeCastResult2 originalDenseHit = dense.getHit();
    dense.setDenseTargets( true );
    check( dense.hasDenseTargets() && dense.getDenseTargets().size() == 40 &&
        dense.getWorld().GetBodyCount() == 44 && dense.getWorld().GetShapeCount() == 44,
        "Dense visualization creates 40 additional mixed Body/Shape targets" );
    for( const shapeCastTarget& target : dense.getDenseTargets() )
        check( dense.getWorld().IsValid( target.shape ) &&
            dense.getWorld().IsValid( target.body ), "Dense target handle is valid" );

    const auto denseStats = dense.getSweepStats();
    check( denseStats.treeCandidates >= denseStats.narrowphaseCandidates &&
        denseStats.narrowphaseCandidates >= denseStats.toiCalls &&
        denseStats.toiCalls > 0 && denseStats.toi.distanceCalls >= denseStats.toiCalls,
        "Dense World query accounts for Tree candidates and TOI work" );
    worldShapeSweepStats2 directStats{};
    const auto measured = dense.getWorld().castShapeSweepClosest(
        { dense.getInput().geometry, dense.getInput().transform,
          dense.getInput().translation, dense.getRotationRadians() }, {}, &directStats );
    const auto unmeasured = dense.getWorld().castShapeSweepClosest(
        { dense.getInput().geometry, dense.getInput().transform,
          dense.getInput().translation, dense.getRotationRadians() } );
    check( measured.hit == unmeasured.hit && measured.fraction == unmeasured.fraction &&
        measured.shape == unmeasured.shape &&
        directStats.toiCalls == denseStats.toiCalls &&
        directStats.treeCandidates == denseStats.treeCandidates &&
        directStats.toi.distanceCalls == denseStats.toi.distanceCalls,
        "Optional World stage instrumentation must not change query results" );

    dense.setDenseTargets( true );
    check( dense.getWorld().GetShapeCount() == 44,
        "Repeated dense mode enable does not duplicate bodies" );
    dense.setDenseTargets( false );
    check( !dense.hasDenseTargets() && dense.getWorld().GetShapeCount() == 4 &&
        dense.getWorld().GetBodyCount() == 4 &&
        dense.getHit().hit == originalDenseHit.hit &&
        dense.getHit().fraction == originalDenseHit.fraction,
        "Dense mode teardown restores original four-target query" );
    dense.setRotationRadians( 0.0f );
    check( dense.getSweepStats().toiCalls == 0 && dense.getSweepStats().treeCandidates == 0,
        "Linear ShapeCast keeps rotation-only counters empty" );

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
