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

    const vec2 square[] = { { 0.0f, 0.0f }, { 2.0f, 0.0f }, { 2.0f, 2.0f }, { 0.0f, 2.0f } };
    const catmullRomSpline2 loop{ square, true };
    check( !spline.isLoop && GetSegmentCount( { square } ) == 3, "loop option defaults to the existing open path" );
    check( IsValid( loop ) && GetSegmentCount( loop ) == 4, "closed spline includes the last-to-first segment" );
    const vec2 triangle[] = { { 0.0f, 0.0f }, { 3.0f, 0.0f }, { 0.0f, 2.0f } };
    check( IsValid( { triangle, true } ) && GetSegmentCount( { triangle, true } ) == 3, "three controls form the minimum closed spline" );
    check( !IsValid( { {}, true } ) && !IsValid( { single, true } ) && !IsValid( { line, true } ) && GetSegmentCount( { line, true } ) == 0, "closed spline requires at least three controls" );
    const vec2 repeatedStart[] = { { 0.0f, 0.0f }, { 2.0f, 0.0f }, { 1.0f, 2.0f }, { 0.0f, 0.0f } };
    check( !IsValid( { repeatedStart, true } ) && IsValid( { repeatedStart } ), "closed controls must not repeat the first point at the end" );
    const vec2 closingOverflow[] = { { -1.0e19f, 0.0f }, { 0.0f, 0.0f }, { 1.0e19f, 0.0f } };
    const vec2 closingUnderflow[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0e-30f, 0.0f } };
    check( IsValid( { closingOverflow } ) && !IsValid( { closingOverflow, true } ) && IsValid( { closingUnderflow } ) && !IsValid( { closingUnderflow, true } ), "closed validation includes closing edge overflow and underflow" );
    check( !IsValid( { repeated, true } ) && !IsValid( { repeatedStart, true } ), "closed validation rejects adjacent duplicate controls" );
    for( std::size_t segment = 0; segment < 4; ++segment )
    {
        check( near( Evaluate( loop, segment, 0.0f ), square[segment] ) && near( Evaluate( loop, segment, 1.0f ), square[( segment + 1 ) % 4] ), "closed segments interpolate their controls through the seam" );
        check( near( Derivative( loop, segment, 1.0f ), Derivative( loop, ( segment + 1 ) % 4, 0.0f ) ), "equal-length closed segments share seam derivatives" );
    }
    check( near( Derivative( loop, 0, 0.0f ), { 1.0f, -1.0f } ) && near( Evaluate( loop, 0, 0.5f ), { 1.0f, -0.25f } ), "first closed segment uses the last control as its previous neighbor" );
    check( near( Evaluate( loop, 3, 0.5f ), { -0.25f, 1.0f } ), "closing segment wraps both endpoint and next neighbor" );
    const vec2 unevenLoopPoints[] = { { 0.0f, 0.0f }, { 4.0f, 0.0f }, { 4.0f, 9.0f }, { -1.0f, 3.0f } };
    const catmullRomSpline2 unevenLoop{ unevenLoopPoints, true };
    for( std::size_t segment = 0; segment < 4; ++segment )
    {
        const auto next = ( segment + 1 ) % 4;
        const float interval = std::sqrt( Length( unevenLoopPoints[next] - unevenLoopPoints[segment] ) );
        const float nextInterval = std::sqrt( Length( unevenLoopPoints[( next + 1 ) % 4] - unevenLoopPoints[next] ) );
        check( near( Derivative( unevenLoop, segment, 1.0f ) / interval, Derivative( unevenLoop, next, 0.0f ) / nextInterval ), "unequal closed segments preserve knot-space seam derivatives" );
    }
    const auto closedFixed = Sample( loop, 4 );
    const auto closedAdaptive = SampleAdaptive( loop, 0.01f );
    check( closedFixed.size() == 17 && near( closedFixed.front(), square[0] ) && near( closedFixed.back(), square[0] ), "fixed closed samples append one terminal first point" );
    check( closedAdaptive.size() > 4 && near( closedAdaptive.front(), square[0] ) && near( closedAdaptive.back(), square[0] ), "adaptive samples include the complete closing segment" );
    for( const auto* samples : { &closedFixed, &closedAdaptive } )
        for( std::size_t control = 0; control < 4; ++control )
        {
            std::size_t occurrences = 0;
            for( const vec2 point : *samples ) occurrences += near( point, square[control] );
            check( occurrences == ( control == 0 ? 2u : 1u ), "closed sampling preserves joins once except for the terminal closure" );
        }
    for( std::size_t segment = 0; segment < 4; ++segment )
        for( int i = 0; i <= 100; ++i )
        {
            const vec2 point = Evaluate( loop, segment, i / 100.0f );
            float distanceSquared = infinity;
            for( std::size_t j = 1; j < closedAdaptive.size(); ++j )
                distanceSquared = std::min( distanceSquared, DistanceSquared( segment2{ closedAdaptive[j - 1], closedAdaptive[j] }, point ) );
            check( distanceSquared <= 0.01001f * 0.01001f, "adaptive closed samples bound all segments including the seam" );
        }
    const auto closedLength = GetLength( loop, 1.0e-6 );
    const auto closedDense = Sample( loop, 10000 );
    check( closedLength.has_value(), "closed length converges across every segment" );
    double closedReferenceLength = 0.0;
    for( std::size_t i = 0; i < closedDense.size(); ++i )
    {
        if( i > 0 ) closedReferenceLength += std::hypot( static_cast<double>( closedDense[i].x ) - closedDense[i - 1].x, static_cast<double>( closedDense[i].y ) - closedDense[i - 1].y );
        if( i % 5000 != 0 ) continue;
        const auto sample = EvaluateByDistance( loop, i + 1 == closedDense.size() ? *closedLength : closedReferenceLength, 1.0e-6 );
        check( sample && near( sample->point, closedDense[i], 2.0e-5f ), "closed distance query follows all segments and returns to its start" );
    }
    check( std::abs( *closedLength - closedReferenceLength ) < 1.0e-5, "closed length includes the curved last-to-first segment" );
    const auto loopStart = EvaluateByDistance( loop, 0.0, 1.0e-6 ), loopEnd = EvaluateByDistance( loop, *closedLength, 1.0e-6 );
    check( loopStart && loopEnd && near( loopStart->point, loopEnd->point ) && near( loopStart->tangent, loopEnd->tangent ), "closed distance endpoints share position and forward unit tangent" );
    check( !EvaluateByDistance( loop, -0.1, 1.0e-6 ) && !EvaluateByDistance( loop, std::nextafter( *closedLength, std::numeric_limits<double>::infinity() ), 1.0e-6 ), "closed distance query preserves range rejection instead of wrapping" );
    check( Sample( { repeatedStart, true }, 4 ).empty() && SampleAdaptive( { repeatedStart, true }, 0.01f ).empty() && !GetLength( { repeatedStart, true }, 1.0e-6 ) && !EvaluateByDistance( { repeatedStart, true }, 0.0, 1.0e-6 ), "invalid closed controls produce no samples, length or distance result" );

    const vec2 invalidLoop[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { nan, 1.0f } };
    check( !IsValid( { invalidLoop, true } ) && !GetLength( { invalidLoop, true }, 1.0e-6 ), "closed validation rejects nonfinite controls including the closing endpoint" );
    const catmullRomSpline2 triangleLoop{ triangle, true };
    const auto triangleSamples = SampleAdaptive( triangleLoop, 0.01f );
    const auto triangleLength = GetLength( triangleLoop, 1.0e-5 );
    check( !triangleSamples.empty() && near( triangleSamples.front(), triangleSamples.back() ) && triangleLength, "minimum three-point loop supports adaptive sampling and length" );
    const auto triangleEnd = EvaluateByDistance( triangleLoop, *triangleLength, 1.0e-5 );
    check( triangleEnd && near( triangleEnd->point, triangle[0] ), "minimum loop distance lookup wraps its final segment" );
    const auto unevenClosedLength = GetLength( unevenLoop, 1.0e-4 );
    const auto unevenClosedDense = Sample( unevenLoop, 10000 );
    double unevenPrefix = 0.0;
    check( unevenClosedLength.has_value(), "unequal closed segments support arc length integration" );
    for( std::size_t i = 0; i < unevenClosedDense.size(); ++i )
    {
        if( i > 0 ) unevenPrefix += std::hypot( static_cast<double>( unevenClosedDense[i].x ) - unevenClosedDense[i - 1].x, static_cast<double>( unevenClosedDense[i].y ) - unevenClosedDense[i - 1].y );
        if( i % 5000 != 0 ) continue;
        const auto sample = EvaluateByDistance( unevenLoop, i + 1 == unevenClosedDense.size() ? *unevenClosedLength : unevenPrefix, 1.0e-4 );
        check( sample && near( sample->point, unevenClosedDense[i], 2.0e-4f ), "unequal closed distance lookup includes the nonuniform closing segment" );
    }
    check( std::abs( *unevenClosedLength - unevenPrefix ) < 1.0e-4, "unequal closed length agrees with an independent dense reference" );
#ifdef NDEBUG
    check( near( Evaluate( loop, 4, 0.5f ), {} ) && near( Derivative( loop, 4, 0.5f ), {} ), "closed segment indices remain bounded in Release" );
    check( near( Evaluate( { line, true }, 0, 0.5f ), {} ) && near( Derivative( { repeatedStart, true }, 0, 0.5f ), {} ), "invalid closed evaluation fails safely in Release" );
#endif

    const auto projected = ClosestPoint( { unitLine }, { 0.25f, 2.0f }, 1.0e-5 );
    check( projected && near( projected->point, { 0.25f, 0.0f } ) && projected->segmentIndex == 0 && projected->t == 0.25f && projected->distanceSquared == 4.0, "closest point projects onto a two-point spline" );
    const auto beforeStart = ClosestPoint( { unitLine }, { -2.0f, 1.0f }, 1.0e-5 );
    const auto afterEnd = ClosestPoint( { unitLine }, { 3.0f, 1.0f }, 1.0e-5 );
    check( beforeStart && afterEnd && beforeStart->t == 0.0f && afterEnd->t == 1.0f && near( beforeStart->point, unitLine[0] ) && near( afterEnd->point, unitLine[1] ) && beforeStart->distanceSquared == 5.0 && afterEnd->distanceSquared == 5.0, "closest point includes both open endpoints" );
    const auto unevenProjection = ClosestPoint( { unevenLine }, { 8.5f, 0.0f }, 1.0e-5 );
    check( unevenProjection && near( unevenProjection->point, { 8.5f, 0.0f }, 1.0e-4f ) && unevenProjection->segmentIndex == 1 && std::abs( unevenProjection->t - 0.5f ) > 0.01f, "closest point inverts nonuniform straight parameterization" );
    const vec2 wavePoints[] = { { -3.0f, 0.0f }, { -1.0f, 2.0f }, { 1.0f, -2.0f }, { 3.0f, 0.0f }, { 5.0f, 2.0f } };
    const catmullRomSpline2 wave{ wavePoints };
    const vec2 crossingPoints[] = { { -2.0f, -1.0f }, { 2.0f, 1.0f }, { -2.0f, 1.0f }, { 2.0f, -1.0f } };
    const catmullRomSpline2 crossing{ crossingPoints, true };
    for( const auto candidate : { curve, wave, loop, unevenLoop, crossing } )
    {
        const auto reference = Sample( candidate, 20000 );
        for( const vec2 query : { vec2{ 0.1f, 0.2f }, vec2{ -0.5f, -0.5f }, vec2{ 1.0f, 1.5f }, vec2{ 6.0f, -2.0f } } )
        {
            const auto closest = ClosestPoint( candidate, query, 1.0e-4 );
            check( closest && closest->segmentIndex < GetSegmentCount( candidate ) && closest->t >= 0.0f && closest->t <= 1.0f, "closest point searches all open or closed segments" );
            check( near( closest->point, Evaluate( candidate, closest->segmentIndex, closest->t ) ), "closest point metadata reconstructs the returned curve point" );
            const double measured = std::hypot( static_cast<double>( closest->point.x ) - query.x, static_cast<double>( closest->point.y ) - query.y );
            check( std::abs( closest->distanceSquared - measured * measured ) < 1.0e-10, "closest squared distance uses the returned point in double precision" );
            double bestReference = std::numeric_limits<double>::infinity();
            for( const vec2 point : reference ) bestReference = std::min( bestReference, std::hypot( static_cast<double>( point.x ) - query.x, static_cast<double>( point.y ) - query.y ) );
            check( measured <= bestReference + 1.0e-4 && measured >= bestReference - 1.0e-3, "closest distance agrees with dense global references for S curves and crossing branches" );
        }
    }
    const auto seamPoint = ClosestPoint( loop, square[0], 1.0e-5 );
    check( seamPoint && near( seamPoint->point, square[0] ) && seamPoint->distanceSquared == 0.0, "closest point includes the closed seam" );
    const auto stationary = ClosestPoint( { reversal }, { 1.2f, 0.0f }, 1.0e-5 );
    check( stationary && near( stationary->point, { 1.0f, 0.0f } ), "closest point succeeds at a zero tangent without requesting a direction" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), 1.0e-30 } )
        check( !ClosestPoint( curve, { 0.2f, 0.3f }, tolerance ), "closest point rejects invalid or unattainable tolerance" );
    check( !ClosestPoint( {}, {}, 1.0e-4 ) && !ClosestPoint( { repeatedStart, true }, {}, 1.0e-4 ) && !ClosestPoint( curve, { nan, 0.0f }, 1.0e-4 ) && !ClosestPoint( curve, { infinity, 0.0f }, 1.0e-4 ), "closest point rejects invalid curves and nonfinite query points without assertions" );
    const vec2 coarseDiagonal[] = { { 1.0e8f, 1.0e8f }, { 1.0e8f + 8.0f, 1.0e8f + 8.0f } };
    check( !ClosestPoint( { coarseDiagonal }, { 1.0e8f + 8.0f, 1.0e8f }, 1.0e-3 ), "closest point fails when float coordinates cannot represent a sufficiently close curve point" );
    editable[1] = { 4.0f, 0.0f };
    const auto editedClosest = ClosestPoint( editableCurve, { 2.0f, 1.0f }, 1.0e-5 );
    check( editedClosest && near( editedClosest->point, { 2.0f, 0.0f } ), "closest point observes edited borrowed controls" );

    check( near( SecondDerivative( curve, 1, 0.25f ), { -1.5f, 1.5f } ), "second derivative matches the analytic cubic" );
    check( near( SecondDerivative( curve, 1, 0.0f ), { -3.0f, 3.0f } ) && near( SecondDerivative( curve, 1, 1.0f ), { 3.0f, -3.0f } ), "second derivative includes one-sided segment endpoints" );
    const double analyticCurvature = 1.5 / std::pow( 1.1328125, 1.5 );
    const auto positiveCurvature = Curvature( curve, 1, 0.25f );
    const auto negativeCurvature = Curvature( curve, 1, 0.75f );
    const auto inflectionCurvature = Curvature( curve, 1, 0.5f );
    check( positiveCurvature && negativeCurvature && inflectionCurvature && std::abs( *positiveCurvature - analyticCurvature ) < 1.0e-12 && std::abs( *negativeCurvature + analyticCurvature ) < 1.0e-12 && *inflectionCurvature == 0.0, "signed curvature matches the analytic cubic and inflection" );
    for( float t : { 0.0f, 0.5f, 1.0f } )
    {
        const auto lineCurvature = Curvature( spline, 0, t );
        check( near( SecondDerivative( spline, 0, t ), {} ) && lineCurvature && *lineCurvature == 0.0, "two-point spline has zero second derivative and curvature" );
        const auto unevenLineCurvature = Curvature( { unevenLine }, 1, t );
        check( unevenLineCurvature && *unevenLineCurvature == 0.0, "nonuniform straight parameterization has zero curvature" );
    }
    for( const auto candidate : { curve, unevenCurve, loop, unevenLoop } )
    {
        for( std::size_t segment = 0; segment < GetSegmentCount( candidate ); ++segment )
        {
            for( float t : { 0.0f, 0.25f, 0.75f, 1.0f } )
            {
                constexpr float h = 0.001f;
                const float a = std::max( 0.0f, t - h ), b = std::min( 1.0f, t + h );
                const vec2 difference = ( Derivative( candidate, segment, b ) - Derivative( candidate, segment, a ) ) / ( b - a );
                check( near( SecondDerivative( candidate, segment, 0.5f * ( a + b ) ), difference, 0.01f ), "second derivative agrees with finite differences including open and closed boundaries" );
                check( Curvature( candidate, segment, t ).has_value(), "regular open and closed endpoints have curvature" );
            }
        }
    }
    check( !near( SecondDerivative( unevenLoop, 0, 0.0f ), SecondDerivative( unevenLoop, 3, 1.0f ) ), "closed seam preserves separate one-sided second derivatives" );
    const vec2 reversedPoints[] = { points[3], points[2], points[1], points[0] };
    const auto reversedCurvature = Curvature( { reversedPoints }, 1, 0.75f );
    check( near( SecondDerivative( { reversedPoints }, 1, 0.75f ), SecondDerivative( curve, 1, 0.25f ) ) && reversedCurvature && std::abs( *reversedCurvature + analyticCurvature ) < 1.0e-12, "reversing direction reverses curvature sign" );
    for( float scale : { 1.0e-10f, 2.0f, 1.0e10f } )
    {
        const vec2 scaledPoints[] = { scale * points[0], scale * points[1], scale * points[2], scale * points[3] };
        const auto scaledCurvature = Curvature( { scaledPoints }, 1, 0.25f );
        check( scaledCurvature && std::abs( *scaledCurvature * scale - analyticCurvature ) < 1.0e-5, "curvature scales inversely without float overflow or underflow" );
        check( near( SecondDerivative( { scaledPoints }, 1, 0.25f ) / scale, { -1.5f, 1.5f }, 1.0e-5f ), "second derivative scales with control coordinates" );
    }
    check( !Curvature( { reversal }, 0, 1.0f ), "curvature rejects a zero tangent" );
    for( const auto invalid : { catmullRomSpline2{}, catmullRomSpline2{ repeated }, catmullRomSpline2{ infinite }, catmullRomSpline2{ repeatedStart, true } } )
    {
        check( !Curvature( invalid, 0, 0.5f ), "curvature rejects invalid splines without assertions" );
#ifdef NDEBUG
        check( near( SecondDerivative( invalid, 0, 0.5f ), {} ), "invalid second derivative fails safely in Release" );
#endif
    }
    for( float t : { -0.1f, 1.1f, nan, infinity } )
    {
        check( !Curvature( curve, 1, t ), "curvature rejects invalid parameters without assertions" );
#ifdef NDEBUG
        check( near( SecondDerivative( curve, 1, t ), {} ), "second derivative rejects invalid parameters in Release" );
#endif
    }
    check( !Curvature( curve, GetSegmentCount( curve ), 0.5f ) && !Curvature( loop, std::numeric_limits<std::size_t>::max(), 0.5f ), "curvature rejects invalid open and closed segment indices" );
#ifdef NDEBUG
    check( near( SecondDerivative( loop, GetSegmentCount( loop ), 0.5f ), {} ), "second derivative rejects invalid closed segment indices in Release" );
#endif

    return 0;
}
