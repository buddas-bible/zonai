#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "rigidBodyDemo.h"
#include "geometry/catmullRomSpline2.h"
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

bool near( vec2 a, vec2 b )
{
    return Length( a - b ) < 1.0e-5f;
}

}

int main()
{
    for( const auto kind : { demoKind::splinePath, demoKind::splineRail } )
    {
        rigidBodyDemo scene{ kind };
        auto& physics = scene.getWorld();
        check( scene.getSplineControlPoints().size() >= 2, "both spline path and rail expose editable control points" );
        const auto body = scene.getImpulseBody();
        const auto shape = scene.getShapes()[0].shapeHandle;
        if( kind == demoKind::splineRail ) check( physics.IsValid( scene.getSplineRailJoint() ) && physics.GetBody( body ).type == bodyType::Dynamic, "spline rail demo uses a dynamic slider and persistent joint" );
        scene.step( 0.5f, 4 );
        const auto pose = physics.GetBodyTransform( body );
        const auto velocity = physics.GetBodyLinearVelocity( body );
        const float spin = physics.GetBodyAngularVelocity( body );
        const double fraction = scene.getSplinePathLength() > 0.0 ? scene.getSplinePathDistance() / scene.getSplinePathLength() : 0.0;
        const auto original = scene.getSplineControlPoints()[1];
        demoInput input{};
        input.mousePressed = true;
        input.mouseHeld = true;
        input.mousePosition = original;
        scene.handleInput( input );
        input.mousePressed = false;
        input.mousePosition.y += 0.25f;
        scene.handleInput( input );
        check( near( scene.getSplineControlPoints()[1], input.mousePosition ) && scene.canUndoSplineEdit(), "both spline demos support a recorded control drag" );
        check( near( pose.position, physics.GetBodyTransform( body ).position ) && near( velocity, physics.GetBodyLinearVelocity( body ) ) && spin == physics.GetBodyAngularVelocity( body ) && physics.IsValid( body ) && physics.IsValid( shape ), "spline drag preserves physical state and handles" );
        check( !physics.IsValid( scene.getMouseJoint() ), "spline control editing does not create a mouse joint" );
        scene.cancelInput();
        if( kind == demoKind::splinePath ) check( std::abs( scene.getSplinePathDistance() / scene.getSplinePathLength() - fraction ) < 1.0e-10, "edited path preserves its progress ratio" );
        check( scene.undoSplineEdit() && near( scene.getSplineControlPoints()[1], original ) && scene.redoSplineEdit(), "both spline demos undo and redo control edits" );
        const auto count = scene.getSplineControlPoints().size();
        const vec2 midpoint = Evaluate( { scene.getSplineControlPoints() }, 1, 0.5f );
        input = {};
        input.mousePressed = true;
        input.mouseHeld = true;
        input.insertControlPoint = true;
        input.pixelsPerMeter = 100.0f;
        input.mousePosition = midpoint + vec2{ 0.0f, 0.03f };
        scene.handleInput( input );
        scene.cancelInput();
        check( scene.getSplineControlPoints().size() == count + 1 && near( scene.getSplineControlPoints()[2], input.mousePosition ), "both spline demos insert clicked positions near the actual curve" );
        check( scene.removeSplineControlPoint( 2 ) && scene.getSplineControlPoints().size() == count, "both spline demos delete a selected control" );
        input.mousePosition = { 0.0f, 100.0f };
        scene.handleInput( input );
        scene.cancelInput();
        check( scene.getSplineControlPoints().size() == count && !physics.IsValid( scene.getMouseJoint() ), "distant Ctrl clicks are consumed without insertion or body dragging" );
        if( kind == demoKind::splinePath )
        {
            scene.step( 0.0625f, 4 );
            const auto target = EvaluateByDistance( { scene.getSplinePathPoints() }, scene.getSplinePathDistance(), 1.0e-4 );
            check( target && Length( physics.GetBodyTransform( body ).position - target->point ) < 0.001f, "edited kinematic path continues following its current curve" );
            scene.resetSplinePath();
            check( near( physics.GetBodyTransform( body ).position, scene.getSplineControlPoints()[0] ), "path reset uses its edited current start" );
        }
        else
        {
            const auto data = physics.getSplineRailJointData( scene.getSplineRailJoint() );
            check( data.enableSpring && data.trackingValid && physics.getJointCount() == 1, "edited rail retains its spring and joint" );
            for( int i = 0; i < 120; ++i )
                scene.step( 1.0f / 60.0f, 4 );
            check( physics.getSplineRailJointData( scene.getSplineRailJoint() ).trackingValid, "edited dynamic rail keeps solving" );
        }
        while( scene.getSplineControlPoints().size() > 2 )
            check( scene.removeSplineControlPoint( 1 ), "both demos can delete down to two controls" );
        check( !scene.removeSplineControlPoint( 0 ), "two remaining spline controls cannot be deleted" );
        const vec2 first = scene.getSplineControlPoints()[0], last = scene.getSplineControlPoints()[1];
        input = {};
        input.mousePressed = input.mouseHeld = true;
        input.mousePosition = first;
        scene.handleInput( input );
        input.mousePressed = false;
        input.mousePosition = last;
        scene.handleInput( input );
        scene.cancelInput();
        check( near( scene.getSplineControlPoints()[0], first ) && scene.undoSplineEdit() && scene.getSplineControlPoints().size() == 3, "rejected duplicate-control drag preserves geometry and the previous undo record" );
    }
    return 0;
}
