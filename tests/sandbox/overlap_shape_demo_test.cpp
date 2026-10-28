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
    checkSharedSession();
}
