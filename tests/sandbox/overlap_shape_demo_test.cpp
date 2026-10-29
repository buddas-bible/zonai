#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>

#include "overlapShapeDemo.h"

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

void check( bool ok, const char* message )
{
    if( ok ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

std::unique_ptr<demo> createViewModel( demoKind )
{
    return std::make_unique<overlapShapeDemo>();
}

void checkFixturesAndQueries()
{
    overlapShapeDemo demo{};

    check( getDemoEntry( demoKind::overlapShapeVisualization ).kind == demoKind::overlapShapeVisualization,
        "New spatial overlap demo must be registered" );
    check( demo.getWorld().GetBodyCount() == 4 && demo.getWorld().GetShapeCount() == 4,
        "Overlap demo must keep four independent target handles" );

    for( const auto& target : demo.getTargets() )
        check( demo.getWorld().IsValid( target.body ) && demo.getWorld().IsValid( target.shape ),
            "Every target must be valid" );

    const shapeId box = demo.getTargets()[0].shape;
    check( demo.getCandidates().size() == 1 && demo.isCandidate( box ) &&
        demo.getExactOverlaps().empty(), "Initial placement demonstrates AABB-only false positive" );

    const aabb2 initialBounds = demo.getQueryAABB();
    check( initialBounds.min.x < 1.0f && initialBounds.min.y < 1.0f,
        "Circle query AABB must overlap the upper right box corner" );

    // 저장된 target AABB에는 speculative margin이 있으므로, tight AABB가 떨어져도 후보가 될 수 있음.
    demo.setQueryPosition( { 1.51f, 0.0f } );
    check( demo.getQueryAABB().min.x > 1.0f &&
        !Overlaps( demo.getQueryAABB(), ComputeShapeAABB( MakeBox( { 1.0f, 1.0f } ), {} ) ) &&
        Overlaps( demo.getQueryAABB(), demo.getWorld().GetShape( box ).aabb ) &&
        demo.isCandidate( box ) && !demo.isExactOverlap( box ),
        "Speculative target AABB is the actual World query candidate boundary" );

    demo.setQueryPosition( { 1.2f, 1.2f } );
    check( demo.getCandidates().size() == 1 && demo.getExactOverlaps().size() == 1 &&
        demo.isExactOverlap( box ), "Moving the circle toward the corner enables GJK overlap" );

    demo.setQueryPosition( { 1.5f, 0.45f } );
    check( demo.isExactOverlap( box ), "Tangent polygon-circle contact must count as overlap" );

    demo.setQueryPosition( { 1.5001f, 0.45f } );
    check( demo.isCandidate( box ) && !demo.isExactOverlap( box ),
        "GJK overlap rejects small genuine separation while AABBs still cross" );

    demo.setProbe( overlapProbe::capsule );
    demo.setQueryPosition( { 0.0f, 0.0f } );
    check( std::holds_alternative<capsule2>( demo.getQueryGeometry() ) && demo.isExactOverlap( box ),
        "Capsule query overlaps convex box" );

    demo.setProbe( overlapProbe::box );
    demo.setQueryRotation( 0.78539816339f );
    check( std::holds_alternative<polygon2>( demo.getQueryGeometry() ) &&
        demo.isExactOverlap( box ) && std::abs( demo.getQueryRotation() - 0.78539816339f ) < 1.0e-5f,
        "Rotated box query is evaluated using its transform" );

    demo.setQueryPosition( { 3.0f, 0.0f } );
    check( demo.isExactOverlap( demo.getTargets()[2].shape ),
        "Dynamic tree shape is included in exact queries" );

    demo.setProbe( overlapProbe::circle );
    demo.setQueryRotation( 0.0f );
    demo.setQueryPosition( { 0.0f, 2.8f } );
    const shapeId sensor = demo.getTargets()[3].shape;

    check( demo.isCandidate( sensor ) && demo.isExactOverlap( sensor ) &&
        demo.getCandidates().size() == 1 && demo.getExactOverlaps().size() == 1,
        "Kinematic sensor is included by default" );

    demo.setSensorIncluded( false );
    check( demo.getCandidates().empty() && demo.getExactOverlaps().empty(),
        "The same filter excludes the sensor from both AABB and exact queries" );

    demo.setSensorIncluded( true );
    check( demo.isCandidate( sensor ) && demo.isExactOverlap( sensor ),
        "Re-enabling sensor restores both query results" );

    demo.setCandidateAABBsVisible( false );
    check( !demo.areCandidateAABBsVisible() && demo.isExactOverlap( sensor ),
        "Visual toggle must not modify query results" );

    const vec2 previous = demo.getQueryTransform().position;
    demo.handleInput( { .mousePosition = { -3.0f, 0.0f }, .mousePressed = false } );
    check( LengthSquared( demo.getQueryTransform().position - previous ) == 0.0f,
        "Cursor movement without click must not change query position" );

    demo.handleInput( { .mousePosition = { -3.0f, 0.0f }, .mousePressed = true } );
    check( demo.isExactOverlap( demo.getTargets()[1].shape ),
        "Canvas left click moves query shape and refreshes overlap" );

    const vec2 beforeInvalid = demo.getQueryTransform().position;
    demo.setQueryPosition( { std::numeric_limits<float>::quiet_NaN(), 0.0f } );
    demo.setQueryRotation( std::numeric_limits<float>::infinity() );
    check( LengthSquared( demo.getQueryTransform().position - beforeInvalid ) == 0.0f &&
        std::isfinite( demo.getQueryRotation() ), "Non-finite UI settings must be ignored" );

    const auto previousCandidates = demo.getCandidates();
    demo.step( 1.0f / 60.0f, 4 );
    check( demo.getCandidates() == previousCandidates && demo.getWorld().GetBodyCount() == 4,
        "Query-only scene must not move physics bodies when playback steps" );
}

void checkPointQueryMode()
{
    overlapShapeDemo demo{};
    const shapeId box = demo.getTargets()[0].shape;
    const shapeId circle = demo.getTargets()[1].shape;
    const shapeId dynamic = demo.getTargets()[2].shape;
    const shapeId sensor = demo.getTargets()[3].shape;

    check( demo.getQueryMode() == overlapQueryMode::shape,
        "Existing overlap visualization must default to the shape query" );

    demo.setQueryMode( overlapQueryMode::point );
    check( demo.getCandidates().empty() && demo.getExactOverlaps().empty(),
        "Initial query point is outside the target box even though the initial circle AABB overlapped it" );

    demo.setQueryPosition( { 1.01f, 0.0f } );
    const aabb2 pointBounds = demo.getQueryAABB();
    check( pointBounds.min.x == 1.01f && pointBounds.max.x == 1.01f &&
        pointBounds.min.y == 0.0f && pointBounds.max.y == 0.0f,
        "Point query uses a zero-extent broadphase AABB" );
    check( demo.isCandidate( box ) && !demo.isExactOverlap( box ),
        "Speculative box AABB can contain a point that lies outside the actual box" );

    demo.setQueryPosition( { 1.0f, 0.0f } );
    check( demo.isCandidate( box ) && demo.isExactOverlap( box ),
        "A point on a polygon face must be included" );

    demo.setQueryPosition( { 1.3f, 0.0f } );
    check( !demo.isCandidate( box ) && !demo.isExactOverlap( box ),
        "Separated point must not reuse the circle probe's AABB" );
    demo.setQueryMode( overlapQueryMode::shape );
    check( demo.isCandidate( box ) && demo.isExactOverlap( box ),
        "Switching to shape mode restores GJK/radius overlap at the same query location" );

    demo.setQueryMode( overlapQueryMode::point );
    demo.setQueryPosition( { -3.0f, 0.0f } );
    check( demo.isExactOverlap( circle ) && demo.getExactOverlaps().size() == 1,
        "Point query must classify the static circle" );

    demo.setQueryPosition( { 3.0f, 0.0f } );
    check( demo.isExactOverlap( dynamic ) && demo.getExactOverlaps().size() == 1,
        "Point query must classify the Dynamic tree" );

    demo.setQueryPosition( { 0.0f, 2.8f } );
    check( demo.isExactOverlap( sensor ) && demo.getExactOverlaps().size() == 1,
        "Point query must include a Kinematic sensor by default" );

    demo.setSensorIncluded( false );
    check( demo.getCandidates().empty() && demo.getExactOverlaps().empty(),
        "Sensor mask must exclude point-query candidates and exact results together" );
    demo.setSensorIncluded( true );
    check( demo.isExactOverlap( sensor ), "Point query must restore sensor when re-enabled" );

    // 점 조회의 판정은 선택했던 도형 종류나 회전과 무관해야 함.
    demo.setQueryPosition( { 1.01f, 0.0f } );
    const auto previousCandidates = demo.getCandidates();
    const auto previousOverlaps = demo.getExactOverlaps();
    demo.setProbe( overlapProbe::capsule );
    demo.setQueryRotation( 0.7f );
    check( demo.getCandidates() == previousCandidates &&
        demo.getExactOverlaps() == previousOverlaps,
        "Changing hidden shape settings must not change the active point query" );

    demo.setQueryMode( overlapQueryMode::shape );
    check( demo.getProbe() == overlapProbe::capsule &&
        std::abs( demo.getQueryRotation() - 0.7f ) < 1.0e-5f,
        "Returning to shape mode must retain the previous geometry and rotation controls" );

    check( demo.getWorld().GetBodyCount() == 4 && demo.getWorld().GetShapeCount() == 4,
        "Query mode toggles must never create or move World bodies and shapes" );
}

void checkSharedSession()
{
    demoSession session{ createViewModel };
    session.selectDemo( demoKind::overlapShapeVisualization );
    session.setPlaying( true );
    session.advance( 1.0f / 60.0f, 4 );
    check( session.getStepCount() > 0 && static_cast<overlapShapeDemo&>( session.getDemo() ).getCandidates().size() == 1,
        "Shared playback must preserve spatial query scene without simulation" );

    session.handleInput( { .mousePosition = { 1.2f, 1.2f }, .mousePressed = true }, true );
    check( static_cast<overlapShapeDemo&>( session.getDemo() ).getExactOverlaps().size() == 1,
        "Shared canvas input routes clicks to query model" );
    session.reset();
    check( static_cast<overlapShapeDemo&>( session.getDemo() ).getCandidates().size() == 1 &&
        static_cast<overlapShapeDemo&>( session.getDemo() ).getExactOverlaps().empty(),
        "Reset restores the documented AABB-only initial state" );
}

} // namespace

int main()
{
    checkFixturesAndQueries();
    checkPointQueryMode();
    checkSharedSession();
}
