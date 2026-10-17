#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "geometry/catmullRomSpline2.h"
#include "geometry/segment2.h"

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

    const auto adaptiveLine = SampleAdaptive( spline, 0.01f );
    check( adaptiveLine.size() == 2 && near( adaptiveLine.front(), line[0] ) && near( adaptiveLine.back(), line[1] ), "adaptive straight segment needs only its endpoints" );
    const auto adaptive = SampleAdaptive( curve, 0.01f );
    const auto loose = SampleAdaptive( curve, 1.0f );
    const auto tight = SampleAdaptive( curve, 0.001f );
    check( loose.size() == 4 && adaptive.size() > loose.size() && tight.size() > adaptive.size(), "smaller adaptive error refines curved segments" );
    for( const vec2 point : points )
    {
        std::size_t occurrences = 0;
        for( const vec2 sample : adaptive ) occurrences += near( sample, point );
        check( occurrences == 1, "adaptive samples preserve each control point once" );
    }
    for( std::size_t segment = 0; segment < GetSegmentCount( curve ); ++segment )
        for( int i = 0; i <= 1000; ++i )
        {
            const vec2 point = Evaluate( curve, segment, static_cast<float>( i ) / 1000.0f );
            float distanceSquared = std::numeric_limits<float>::infinity();
            for( std::size_t j = 1; j < adaptive.size(); ++j )
                distanceSquared = std::min( distanceSquared, DistanceSquared( segment2{ adaptive[j - 1], adaptive[j] }, point ) );
            check( distanceSquared <= 0.01001f * 0.01001f, "adaptive polyline bounds dense curve samples including the S segment with a flat midpoint" );
        }
    check( SampleAdaptive( {}, 0.01f ).empty(), "adaptive sampling rejects invalid spline" );
    check( SampleAdaptive( curve, 0.0f ).empty() && SampleAdaptive( curve, -1.0f ).empty(), "adaptive sampling requires positive error" );
    check( SampleAdaptive( curve, nan ).empty() && SampleAdaptive( curve, infinity ).empty(), "adaptive sampling rejects nonfinite error" );
    check( SampleAdaptive( curve, 1.0e-30f ).empty(), "unattainable adaptive error returns no partial result at the depth limit" );

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

    const auto lineLength = GetLength( spline, 1.0e-6 );
    check( lineLength && std::abs( *lineLength - std::sqrt( 20.0 ) ) < 1.0e-6, "two-point spline length equals its edge length" );
    const auto unevenLength = GetLength( { unevenLine }, 1.0e-6 );
    check( unevenLength && std::abs( *unevenLength - 29.0 ) < 1.0e-6, "uneven straight segments preserve the complete path length" );
    const auto curveLength = GetLength( curve, 1.0e-6 );
    check( curveLength.has_value(), "curved spline length converges" );
    const auto dense = Sample( curve, 10000 );
    double referenceLength = 0.0;
    for( std::size_t i = 1; i < dense.size(); ++i )
        referenceLength += std::hypot( static_cast<double>( dense[i].x ) - dense[i - 1].x, static_cast<double>( dense[i].y ) - dense[i - 1].y );
    check( std::abs( *curveLength - referenceLength ) < 1.0e-5, "integrated length agrees with an independent dense polyline" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ) } )
        check( !GetLength( curve, tolerance ), "length rejects nonpositive and nonfinite tolerance" );
    check( !GetLength( {}, 1.0e-6 ) && !GetLength( { repeated }, 1.0e-6 ) && !GetLength( { notANumber }, 1.0e-6 ), "length rejects invalid splines without assertions or partial results" );
    check( !GetLength( curve, 1.0e-30 ), "unattainable length precision fails explicitly" );
    editable[1] = { 3.0f, 4.0f };
    const auto editedLength = GetLength( editableCurve, 1.0e-6 );
    check( editedLength && std::abs( *editedLength - 5.0 ) < 1.0e-6, "length observes current borrowed control points" );

    for( double fraction : { 0.0, 0.25, 0.5, 0.75, 1.0 } )
    {
        const auto sample = EvaluateByDistance( spline, fraction * *lineLength, 1.0e-6 );
        check( sample && near( sample->point, { 1.0f + 4.0f * static_cast<float>( fraction ), 2.0f + 2.0f * static_cast<float>( fraction ) } ), "two-point distance lookup returns points along the edge including both endpoints" );
        check( near( sample->tangent, Normalize( line[1] - line[0] ) ), "distance lookup returns a forward unit tangent" );
    }
    for( double distance : { 0.0, 2.0, 4.0, 8.5, 13.0, 21.0, 29.0 } )
    {
        const auto sample = EvaluateByDistance( { unevenLine }, distance, 1.0e-6 );
        check( sample && near( sample->point, { static_cast<float>( distance ), 0.0f } ) && near( sample->tangent, { 1.0f, 0.0f } ), "uneven collinear spline lookup inverts length instead of interpolating t" );
    }
    double prefixLength = 0.0;
    for( std::size_t i = 0; i < dense.size(); ++i )
    {
        if( i > 0 ) prefixLength += std::hypot( static_cast<double>( dense[i].x ) - dense[i - 1].x, static_cast<double>( dense[i].y ) - dense[i - 1].y );
        if( i % 2500 != 0 ) continue;
        const auto sample = EvaluateByDistance( curve, i + 1 == dense.size() ? *curveLength : prefixLength, 1.0e-6 );
        check( sample && near( sample->point, dense[i], 2.0e-5f ), "curve distance lookup agrees with independent dense arc lengths through segment boundaries" );
        const std::size_t segment = std::min( i / 10000, GetSegmentCount( curve ) - 1 );
        const float t = static_cast<float>( i - segment * 10000 ) / 10000.0f;
        check( near( sample->tangent, Normalize( Derivative( curve, segment, t ) ), 2.0e-5f ), "curve distance tangent follows the analytic derivative" );
    }
    for( double distance : { -1.0, std::nextafter( *lineLength, std::numeric_limits<double>::infinity() ), static_cast<double>( nan ), static_cast<double>( infinity ) } )
        check( !EvaluateByDistance( spline, distance, 1.0e-6 ), "distance lookup rejects nonfinite or out-of-range distance without clamping" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), 1.0e-30 } )
        check( !EvaluateByDistance( curve, 0.5, tolerance ), "distance lookup rejects invalid or unattainable precision" );
    check( !EvaluateByDistance( {}, 0.0, 1.0e-6 ) && !EvaluateByDistance( { repeated }, 0.0, 1.0e-6 ), "distance lookup rejects invalid splines in Debug and Release" );
    const vec2 unitLine[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };
    check( !EvaluateByDistance( { unitLine }, 1.0 / 3.0, 1.0e-12 ), "distance lookup fails when float t cannot meet the requested distance precision" );
    const vec2 translatedLine[] = { { 1.0e8f, 0.0f }, { 1.0e8f + 8.0f, 0.0f } };
    check( !EvaluateByDistance( { translatedLine }, 4.0, 1.0e-6 ), "distance lookup fails when float coordinates cannot represent the requested point" );
    const vec2 reversal[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 0.0f, 0.0f } };
    const auto reversalLength = GetLength( { reversal }, 1.0e-6 );
    check( reversalLength && std::abs( *reversalLength - 2.0 ) < 1.0e-6, "zero tangent does not invalidate path length" );
    check( !EvaluateByDistance( { reversal }, 1.0, 1.0e-6 ), "distance lookup fails at a zero tangent instead of inventing a direction" );
    const auto backward = EvaluateByDistance( { reversal }, *reversalLength, 1.0e-6 );
    check( backward && near( backward->point, {} ) && near( backward->tangent, { -1.0f, 0.0f } ), "last endpoint uses the last segment forward direction" );
    editable[1] = { 0.0f, 6.0f };
    const auto editedSample = EvaluateByDistance( editableCurve, 3.0, 1.0e-6 );
    check( editedSample && near( editedSample->point, { 0.0f, 3.0f } ) && near( editedSample->tangent, { 0.0f, 1.0f } ), "distance lookup observes current borrowed control points without a cache" );

    return 0;
}
