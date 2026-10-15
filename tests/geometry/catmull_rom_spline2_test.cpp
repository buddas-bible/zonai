#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "geometry/catmullRomSpline2.h"

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

bool near( const vec2& a, const vec2& b, float epsilon = 1e-5f )
{
    return std::fabs( a.x - b.x ) <= epsilon && std::fabs( a.y - b.y ) <= epsilon;
}

} // namespace

int main()
{
    const vec2 line[] = { { 1.0f, 2.0f }, { 5.0f, 4.0f } };
    const catmullRomSpline2 spline{ line };

    check( IsValid( spline ), "two distinct finite points form a valid open spline" );
    check( GetSegmentCount( spline ) == 1, "two points form one segment" );

    const vec2 points[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f }, { 2.0f, 1.0f } };

    check( IsValid( { points } ), "multiple distinct points form a valid spline" );
    check( GetSegmentCount( { points } ) == 3, "open spline has one fewer segment than points" );
    check( !IsValid( {} ), "empty spline is invalid" );
    check( GetSegmentCount( {} ) == 0, "empty spline has no segments" );

    const vec2 single[] = { { 0.0f, 0.0f } };

    check( !IsValid( { single } ), "one point cannot form a spline segment" );
    check( GetSegmentCount( { single } ) == 0, "one point has no segments" );

    const vec2 repeated[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 0.0f } };
    const vec2 allRepeated[] = { { 1.0f, 1.0f }, { 1.0f, 1.0f } };

    check( !IsValid( { repeated } ), "adjacent duplicate points make centripetal knots degenerate" );
    check( !IsValid( { allRepeated } ), "all repeated points are invalid" );

    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const vec2 infinite[] = { { infinity, 0.0f }, { 1.0f, 1.0f } };
    const vec2 notANumber[] = { { 0.0f, 0.0f }, { 1.0f, nan } };

    check( !IsValid( { infinite } ), "infinite coordinates are invalid" );
    check( !IsValid( { notANumber } ), "NaN coordinates are invalid" );

    const vec2 tooLarge[] = { { -1.0e30f, 0.0f }, { 1.0e30f, 0.0f } };
    const vec2 tooSmall[] = { { 0.0f, 0.0f }, { 1.0e-30f, 0.0f } };

    check( !IsValid( { tooLarge } ), "overflowing squared edge length is invalid" );
    check( !IsValid( { tooSmall } ), "underflowing squared edge length is invalid" );

    check( near( Evaluate( spline, 0, 0.0f ), { 1.0f, 2.0f } ), "open spline starts at its first control point" );
    check( near( Evaluate( spline, 0, 1.0f ), { 5.0f, 4.0f } ), "open spline ends at its last control point" );
    check( near( Evaluate( spline, 0, 0.25f ), { 2.0f, 2.5f } ), "two-point spline follows the straight edge" );
    check( near( Evaluate( spline, 0, 0.5f ), { 3.0f, 3.0f } ), "two-point spline has a linear midpoint" );

    const catmullRomSpline2 curve{ points };

    for( std::size_t i = 0; i < GetSegmentCount( curve ); ++i )
    {
        check( near( Evaluate( curve, i, 0.0f ), points[i] ), "every segment starts at its control point" );
        check( near( Evaluate( curve, i, 1.0f ), points[i + 1] ), "every segment ends at its next control point" );
    }

    check( near( Evaluate( curve, 1, 0.25f ), { 1.046875f, 0.203125f } ), "equal edge lengths reproduce the cubic Catmull-Rom segment" );

    // edge 길이 4, 9, 16의 centripetal knot 간격은 2, 3, 4임.
    const vec2 uneven[] = { { 0.0f, 0.0f }, { 4.0f, 0.0f }, { 4.0f, 9.0f }, { 20.0f, 9.0f } };
    const catmullRomSpline2 unevenCurve{ uneven };

    check( near( Evaluate( unevenCurve, 1, 0.5f ), { 3.807142857f, 4.307142857f } ), "uneven edges use centripetal rather than uniform or chordal knots" );

    const vec2 unevenLine[] = { { 0.0f, 0.0f }, { 4.0f, 0.0f }, { 13.0f, 0.0f }, { 29.0f, 0.0f } };

    check( near( Evaluate( { unevenLine }, 1, 0.5f ), { 8.114285714f, 0.0f } ), "uneven collinear points stay on the line without forcing constant speed" );

    vec2 editable[] = { { 0.0f, 0.0f }, { 2.0f, 0.0f } };
    const catmullRomSpline2 editableCurve{ editable };
    editable[1] = { 4.0f, 2.0f };

    check( near( Evaluate( editableCurve, 0, 0.5f ), { 2.0f, 1.0f } ), "spline view observes edits to the control-point array" );

    check( near( Derivative( spline, 0, 0.0f ), { 4.0f, 2.0f } ), "two-point derivative equals the edge at the start" );
    check( near( Derivative( spline, 0, 0.5f ), { 4.0f, 2.0f } ), "two-point derivative is constant inside the segment" );
    check( near( Derivative( spline, 0, 1.0f ), { 4.0f, 2.0f } ), "two-point derivative equals the edge at the end" );

    check( near( Derivative( unevenCurve, 1, 0.0f ), { 3.6f, 3.6f } ), "uneven start derivative includes the local knot interval" );
    check( near( Derivative( unevenCurve, 1, 1.0f ), { 5.142857143f, 5.142857143f } ), "uneven end derivative includes the local knot interval" );
    check( near( Derivative( unevenCurve, 1, 0.5f ), { -2.185714286f, 11.314285714f } ), "interior derivative is the analytic cubic derivative" );

    // 구간별 t의 미분을 knot 간격으로 나누면 공유 제어점에서 같은 미분을 얻음.
    check( near( Derivative( unevenCurve, 0, 1.0f ) / 2.0f, Derivative( unevenCurve, 1, 0.0f ) / 3.0f ), "neighboring segments share the derivative in knot space" );
    check( near( Derivative( unevenCurve, 1, 1.0f ) / 3.0f, Derivative( unevenCurve, 2, 0.0f ) / 4.0f ), "last segment preserves the shared knot-space derivative" );

    constexpr float step = 1.0e-3f;
    const vec2 finiteDifference = ( Evaluate( unevenCurve, 1, 0.25f + step ) - Evaluate( unevenCurve, 1, 0.25f - step ) ) / ( 2.0f * step );

    check( near( Derivative( unevenCurve, 1, 0.25f ), finiteDifference, 1.0e-3f ), "analytic derivative agrees with a central finite difference" );

    const auto samples = Sample( spline, 4 );

    check( samples.size() == 5, "four subdivisions of one segment produce five samples" );
    check( near( samples.front(), { 1.0f, 2.0f } ) && near( samples.back(), { 5.0f, 4.0f } ), "sampling preserves both open endpoints" );
    check( near( samples[1], { 2.0f, 2.5f } ) && near( samples[2], { 3.0f, 3.0f } ), "sampling uses evenly spaced local parameters" );

    const auto curveSamples = Sample( curve, 4 );

    check( curveSamples.size() == 13, "segment joins appear only once in sampled output" );
    for( std::size_t i = 0; i < curve.points.size(); ++i )
        check( near( curveSamples[4 * i], points[i] ), "sampling preserves every control point" );

    check( near( curveSamples[5], { 1.046875f, 0.203125f } ), "sampling evaluates the curved segment rather than interpolating its chords" );

    const auto coarseSamples = Sample( curve, 1 );

    check( coarseSamples.size() == 4, "one subdivision returns only the control points" );
    check( Sample( spline, 0 ).empty(), "zero subdivisions are rejected" );
    check( Sample( {}, 4 ).empty(), "invalid spline sampling is rejected before evaluation" );
    check( Sample( { repeated }, 4 ).empty(), "degenerate spline sampling is rejected" );
    check( Sample( curve, std::numeric_limits<std::size_t>::max() ).empty(), "unrepresentable sample count is rejected before allocation" );

#ifdef NDEBUG
    check( near( Evaluate( {}, 0, 0.5f ), {} ), "invalid spline evaluation returns no point in Release" );
    check( near( Evaluate( spline, 1, 0.5f ), {} ), "out-of-range segment is rejected in Release" );
    check( near( Evaluate( spline, std::numeric_limits<std::size_t>::max(), 0.5f ), {} ), "maximum index is rejected before indexing in Release" );
    check( near( Evaluate( spline, 0, -0.1f ), {} ), "negative parameter is rejected in Release" );
    check( near( Evaluate( spline, 0, 1.1f ), {} ), "parameter above one is rejected in Release" );
    check( near( Evaluate( spline, 0, nan ), {} ), "NaN parameter is rejected in Release" );
    check( near( Evaluate( spline, 0, infinity ), {} ), "infinite parameter is rejected in Release" );

    check( near( Derivative( {}, 0, 0.5f ), {} ), "invalid spline derivative returns zero in Release" );
    check( near( Derivative( spline, 1, 0.5f ), {} ), "out-of-range derivative segment is rejected in Release" );
    check( near( Derivative( spline, 0, -0.1f ), {} ), "negative derivative parameter is rejected in Release" );
    check( near( Derivative( spline, 0, 1.1f ), {} ), "derivative parameter above one is rejected in Release" );
    check( near( Derivative( spline, 0, nan ), {} ), "NaN derivative parameter is rejected in Release" );
    check( near( Derivative( spline, 0, infinity ), {} ), "infinite derivative parameter is rejected in Release" );
#endif

    return 0;
}
