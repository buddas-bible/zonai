#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>
#include <type_traits>

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

bool equal( vec2 a, vec2 b )
{
    return a.x == b.x && a.y == b.y;
}

bool equal( const std::optional<splineParameter2>& a, const std::optional<splineParameter2>& b )
{
    return a.has_value() == b.has_value() && ( !a || ( a->segmentIndex == b->segmentIndex && a->t == b->t ) );
}

bool equal( const std::optional<splineSample2>& a, const std::optional<splineSample2>& b )
{
    return a.has_value() == b.has_value() && ( !a || ( equal( a->point, b->point ) && equal( a->tangent, b->tangent ) ) );
}

bool equal( const std::vector<splineParameter2>& a, const std::vector<splineParameter2>& b )
{
    if( a.size() != b.size() ) return false;
    for( std::size_t i = 0; i < a.size(); ++i )
        if( a[i].segmentIndex != b[i].segmentIndex || a[i].t != b[i].t ) return false;
    return true;
}

bool equal( const std::vector<vec2>& a, const std::vector<vec2>& b )
{
    if( a.size() != b.size() ) return false;
    for( std::size_t i = 0; i < a.size(); ++i )
        if( !equal( a[i], b[i] ) ) return false;
    return true;
}

void compare( const catmullRomSpline2& spline )
{
    const auto prepared = prepareSpline( spline );
    check( prepared.has_value(), "valid geometry creates a prepared snapshot" );
    check( prepared->getSegmentCount() == GetSegmentCount( spline ) && prepared->isLoop() == spline.isLoop, "prepared snapshot preserves topology" );
    static_assert( std::is_same_v<decltype( prepared->getSegments() ), std::span<const preparedSplineSegment2>> );
    check( prepared->getSegments().size() == prepared->getSegmentCount(), "prepared geometry exposes owned read-only segment coefficients" );
    for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
    {
        const auto& segment = prepared->getSegments()[i];
        const vec2 a = Evaluate( spline, i, 0.0f ), b = Evaluate( spline, i, 1.0f );
        const vec2 first = Derivative( spline, i, 0.0f ), last = Derivative( spline, i, 1.0f );
        check( equal( segment.start, a ) && equal( segment.end, b ) && equal( segment.tangent1, first ) && equal( segment.tangent2, last ) && equal( segment.quadratic, 3.0f * ( b - a ) - 2.0f * first - last ) && equal( segment.cubic, first + last - 2.0f * ( b - a ) ), "shared coefficients exactly match the previous rail preparation" );
    }

    for( double tolerance : { std::numeric_limits<double>::max(), 1.0e-2, 1.0e-6, 1.0e-10, 1.0e-15, std::numeric_limits<double>::min(), std::numeric_limits<double>::denorm_min() } )
    {
        const auto expected = GetLength( spline, tolerance );
        const auto actual = prepared->getLength( tolerance );
        check( actual == expected, "prepared total length preserves exact results and precision failures" );
        for( std::size_t subdivisions : { 1u, 3u, 8u } )
        {
            check( equal( prepared->sampleByDistanceParameters( { 0, 0.0f }, { prepared->getSegmentCount() - 1, 1.0f }, subdivisions, tolerance ), SampleByDistanceParameters( spline, { 0, 0.0f }, { prepared->getSegmentCount() - 1, 1.0f }, subdivisions, tolerance ) ), "prepared parameter samples preserve exact full-path results and failures" );
            check( equal( prepared->sampleByDistance( subdivisions, tolerance ), SampleByDistance( spline, subdivisions, tolerance ) ), "prepared position samples preserve exact full-path results and failures" );
        }
        check( equal( prepared->evaluateByDistance( 0.0, tolerance ), EvaluateByDistance( spline, 0.0, tolerance ) ), "prepared distance sample preserves start validation and failures" );
        check( equal( prepared->getParameterByDistance( 0.0, tolerance ), GetParameterByDistance( spline, 0.0, tolerance ) ), "prepared inverse validates total length before returning the start" );
        if( expected )
        {
            for( double distance : { std::nextafter( 0.0, 1.0 ), 0.1 * *expected, 0.37 * *expected, 0.5 * *expected, std::nextafter( *expected, 0.0 ), *expected, std::nextafter( *expected, std::numeric_limits<double>::infinity() ) } )
            {
                check( equal( prepared->getParameterByDistance( distance, tolerance ), GetParameterByDistance( spline, distance, tolerance ) ), "prepared inverse preserves exact parameters and failures through the final endpoint" );
                check( equal( prepared->evaluateByDistance( distance, tolerance ), EvaluateByDistance( spline, distance, tolerance ) ), "prepared distance samples preserve exact positions tangents and failures" );
            }
            for( std::size_t i = 0; i + 1 < prepared->getSegmentCount(); ++i )
            {
                const auto join = GetDistance( spline, i, 1.0f, tolerance );
                check( join.has_value(), "inverse join fixture has a computable prefix" );
                for( double distance : { std::nextafter( *join, 0.0 ), *join, std::nextafter( *join, std::numeric_limits<double>::infinity() ) } )
                {
                    check( equal( prepared->getParameterByDistance( distance, tolerance ), GetParameterByDistance( spline, distance, tolerance ) ), "prepared inverse preserves internal join selection and both neighbors" );
                    check( equal( prepared->evaluateByDistance( distance, tolerance ), EvaluateByDistance( spline, distance, tolerance ) ), "prepared distance samples preserve internal joins and both neighbors" );
                }
            }
        }
        for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            for( std::size_t j = i; j < prepared->getSegmentCount(); ++j )
            {
                const splineParameter2 from{ i, 0.37f }, to{ j, 0.75f };
                const auto rangeLength = GetLength( spline, from, to, tolerance );
                for( std::size_t subdivisions : { 1u, 3u, 8u } )
                {
                    check( equal( prepared->sampleByDistanceParameters( from, to, subdivisions, tolerance ), SampleByDistanceParameters( spline, from, to, subdivisions, tolerance ) ), "prepared parameter samples preserve exact partial-path results and failures" );
                    check( equal( prepared->sampleByDistance( from, to, subdivisions, tolerance ), SampleByDistance( spline, from, to, subdivisions, tolerance ) ), "prepared position samples preserve exact partial-path results and failures" );
                }
                check( equal( prepared->getParameterByDistance( from, to, 0.0, tolerance ), GetParameterByDistance( spline, from, to, 0.0, tolerance ) ), "prepared range inverse validates range length before returning the start" );
                check( equal( prepared->evaluateByDistance( from, to, 0.0, tolerance ), EvaluateByDistance( spline, from, to, 0.0, tolerance ) ), "prepared range sample preserves start validation and failures" );
                if( !rangeLength ) continue;
                for( double distance : { std::nextafter( 0.0, 1.0 ), 0.37 * *rangeLength, 0.5 * *rangeLength, std::nextafter( *rangeLength, 0.0 ), *rangeLength, std::nextafter( *rangeLength, std::numeric_limits<double>::infinity() ) } )
                {
                    check( equal( prepared->getParameterByDistance( from, to, distance, tolerance ), GetParameterByDistance( spline, from, to, distance, tolerance ) ), "prepared range inverse preserves exact parameters and precision failures" );
                    check( equal( prepared->evaluateByDistance( from, to, distance, tolerance ), EvaluateByDistance( spline, from, to, distance, tolerance ) ), "prepared range samples preserve exact positions tangents and failures" );
                }
                for( std::size_t k = i; k < j; ++k )
                {
                    const auto join = GetLength( spline, from, { k, 1.0f }, tolerance );
                    check( join.has_value(), "range inverse join fixture has finite length" );
                    for( double distance : { std::nextafter( *join, 0.0 ), *join, std::nextafter( *join, std::numeric_limits<double>::infinity() ) } )
                    {
                        check( equal( prepared->getParameterByDistance( from, to, distance, tolerance ), GetParameterByDistance( spline, from, to, distance, tolerance ) ), "range inverse preserves internal join selection and both neighbors" );
                        check( equal( prepared->evaluateByDistance( from, to, distance, tolerance ), EvaluateByDistance( spline, from, to, distance, tolerance ) ), "prepared range samples preserve internal joins and both neighbors" );
                    }
                }
            }
        for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            for( float t : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.125f, 0.37f, 0.5f, 0.875f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                check( prepared->getDistance( i, t, tolerance ) == GetDistance( spline, i, t, tolerance ), "prepared cumulative distance preserves exact results and precision failures" );
        for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            for( float t0 : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.125f, 0.37f, 0.5f, 0.875f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                for( float t1 : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.125f, 0.37f, 0.5f, 0.875f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                    if( t0 <= t1 ) check( prepared->getLength( i, t0, t1, tolerance ) == GetLength( spline, i, t0, t1, tolerance ), "prepared partial length preserves exact results and precision failures" );
        for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            for( std::size_t j = 0; j < prepared->getSegmentCount(); ++j )
                for( float t0 : { 0.0f, 0.37f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                    for( float t1 : { 0.0f, 0.37f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                        check( prepared->getLength( { i, t0 }, { j, t1 }, tolerance ) == GetLength( spline, { i, t0 }, { j, t1 }, tolerance ), "prepared range length preserves exact results and failures including reversed ranges" );
        check( prepared->getLength( { 0, 0.0f }, { prepared->getSegmentCount() - 1, 1.0f }, tolerance ) == actual, "full range length preserves the closed final endpoint" );
        for( std::size_t i = 1; i < prepared->getSegmentCount(); ++i )
            check( prepared->getLength( { i - 1, 1.0f }, { i, 0.0f }, tolerance ) == 0.0 && prepared->getLength( { i, 0.0f }, { i - 1, 1.0f }, tolerance ) == 0.0, "both join representations give zero length in either order" );
        check( prepared->getDistance( prepared->getSegmentCount() - 1, 1.0f, tolerance ) == actual, "final distance equals the whole length including the closing segment" );
        for( std::size_t i = 1; i < prepared->getSegmentCount(); ++i )
            check( prepared->getDistance( i - 1, 1.0f, tolerance ) == prepared->getDistance( i, 0.0f, tolerance ), "both representations of an internal join have the same distance" );
    }

    for( std::size_t i = 0; i < GetSegmentCount( spline ); ++i )
        for( int step = 0; step <= 128; ++step )
        {
            const float t = step / 128.0f;
            check( equal( prepared->evaluate( i, t ), Evaluate( spline, i, t ) ), "prepared positions exactly match existing evaluation including endpoints" );
            check( equal( prepared->derivative( i, t ), Derivative( spline, i, t ) ), "prepared first derivatives exactly match existing evaluation including endpoints" );
            check( equal( prepared->secondDerivative( i, t ), SecondDerivative( spline, i, t ) ), "prepared second derivatives exactly match existing evaluation" );
        }
    for( std::size_t i = 0; i < GetSegmentCount( spline ); ++i )
        for( float t : { std::nextafter( 0.0f, 1.0f ), 0.1f, 0.37f, 0.9f, std::nextafter( 1.0f, 0.0f ) } )
        {
            check( equal( prepared->evaluate( i, t ), Evaluate( spline, i, t ) ) && equal( prepared->derivative( i, t ), Derivative( spline, i, t ) ) && equal( prepared->secondDerivative( i, t ), SecondDerivative( spline, i, t ) ), "prepared queries preserve rounding near endpoints and at non-dyadic parameters" );
        }
}

}

int main()
{
    const vec2 uniformLine[] = { { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 } };
    const auto uniformSnapshot = prepareSpline( { uniformLine } );
    check( equal( uniformSnapshot->sampleByDistance( 6, 1.0e-6 ), std::vector<vec2>{ { 0, 0 }, { 0.5f, 0 }, { 1, 0 }, { 1.5f, 0 }, { 2, 0 }, { 2.5f, 0 }, { 3, 0 } } ), "prepared position samples match analytic straight-path locations" );
    check( equal( uniformSnapshot->sampleByDistance( { 0, 1.0f }, { 2, 0.0f }, 2, 1.0e-6 ), std::vector<vec2>{ { 1, 0 }, { 1.5f, 0 }, { 2, 0 } } ), "prepared partial position samples include both input endpoints" );
    check( equal( uniformSnapshot->sampleByDistance( { 1, 0.0f }, { 0, 1.0f }, 3, std::numeric_limits<double>::denorm_min() ), std::vector<vec2>( 4, { 1, 0 } ) ), "zero-length position samples repeat the input location" );
    check( uniformSnapshot->sampleByDistance( 10, 1.0e-10 ).empty(), "position sampling discards partial output on inverse precision failure" );
    const auto analyticSamples = uniformSnapshot->sampleByDistanceParameters( { 0, 0.25f }, { 2, 0.75f }, 5, 1.0e-6 );
    check( analyticSamples.size() == 6, "prepared uniform parameter sampling returns subdivisions plus one entries" );
    for( std::size_t i = 0; i < analyticSamples.size(); ++i )
        check( std::abs( ( static_cast<double>( analyticSamples[i].segmentIndex ) + analyticSamples[i].t ) - ( 0.25 + 0.5 * i ) ) <= 1.0e-6, "prepared uniform parameters satisfy analytic straight-path distance error" );
    check( equal( uniformSnapshot->sampleByDistanceParameters( { 0, 1.0f }, { 2, 0.0f }, 2, 1.0e-6 ), std::vector<splineParameter2>{ { 0, 1.0f }, { 1, 0.5f }, { 2, 0.0f } } ), "parameter samples preserve both input endpoint representations" );
    check( equal( uniformSnapshot->sampleByDistanceParameters( { 0, 0.0f }, { 2, 1.0f }, 3, 1.0e-6 ), std::vector<splineParameter2>{ { 0, 0.0f }, { 0, 1.0f }, { 1, 1.0f }, { 2, 1.0f } } ), "uniform samples select the preceding segment at internal joins" );
    check( equal( uniformSnapshot->sampleByDistanceParameters( { 1, 0.0f }, { 0, 1.0f }, 3, std::numeric_limits<double>::denorm_min() ), std::vector<splineParameter2>( 4, { 1, 0.0f } ) ), "zero-length parameter samples repeat from at the smallest positive tolerance" );
    check( uniformSnapshot->sampleByDistanceParameters( { 0, 0.0f }, { 2, 1.0f }, 10, 1.0e-10 ).empty(), "parameter sampling discards partial output on inverse precision failure" );
    check( equal( uniformSnapshot->evaluateByDistance( 1.5, 1.0e-6 ), splineSample2{ { 1.5f, 0 }, { 1, 0 } } ), "prepared distance sample returns analytic position and unit tangent" );
    check( equal( uniformSnapshot->evaluateByDistance( { 0, 0.25f }, { 2, 0.75f }, 1.25, 1.0e-6 ), splineSample2{ { 1.5f, 0 }, { 1, 0 } } ), "prepared range sample returns analytic position and unit tangent" );
    check( equal( uniformSnapshot->evaluateByDistance( { 1, 0.5f }, { 1, 0.5f }, 0.0, 1.0e-6 ), splineSample2{ { 1.5f, 0 }, { 1, 0 } } ) && !uniformSnapshot->evaluateByDistance( { 1, 0.5f }, { 1, 0.5f }, 0.1, 1.0e-6 ), "zero-length distance sample requires zero distance and a valid tangent" );
    check( equal( uniformSnapshot->getParameterByDistance( { 0, 0.25f }, { 2, 0.75f }, 1.25, 1.0e-6 ), splineParameter2{ 1, 0.5f } ), "prepared range inverse returns its analytic segment and parameter" );
    check( equal( uniformSnapshot->getParameterByDistance( { 0, 1.0f }, { 2, 0.0f }, 0.0, 1.0e-6 ), splineParameter2{ 0, 1.0f } ) && equal( uniformSnapshot->getParameterByDistance( { 0, 1.0f }, { 2, 0.0f }, 1.0, 1.0e-6 ), splineParameter2{ 2, 0.0f } ), "range inverse preserves both input endpoint representations" );
    check( equal( uniformSnapshot->getParameterByDistance( { 1, 0.0f }, { 0, 1.0f }, 0.0, std::numeric_limits<double>::denorm_min() ), splineParameter2{ 1, 0.0f } ) && !uniformSnapshot->getParameterByDistance( { 1, 0.0f }, { 0, 1.0f }, std::numeric_limits<double>::denorm_min(), 1.0e-6 ), "zero-length join range returns from only at zero distance" );
    check( equal( uniformSnapshot->getParameterByDistance( { 0, 0.25f }, { 2, 0.75f }, 0.75, 1.0e-6 ), splineParameter2{ 0, 1.0f } ), "range inverse chooses the preceding segment at an exact join" );
    check( !uniformSnapshot->getParameterByDistance( { 0, 0.25f }, { 2, 0.75f }, 0.1, 1.0e-10 ) && uniformSnapshot->getParameterByDistance( { 0, 0.25f }, { 2, 0.75f }, 0.1, 1.0e-6 ).has_value(), "range inverse rejects insufficient float parameter precision" );
    check( equal( uniformSnapshot->getParameterByDistance( 1.5, 1.0e-6 ), splineParameter2{ 1, 0.5f } ), "prepared inverse returns the analytic segment and parameter on a straight path" );
    check( equal( uniformSnapshot->getParameterByDistance( 0.0, 1.0e-6 ), splineParameter2{ 0, 0.0f } ) && equal( uniformSnapshot->getParameterByDistance( 3.0, 1.0e-6 ), splineParameter2{ 2, 1.0f } ), "inverse endpoints preserve the start and final segment representations" );
    check( equal( uniformSnapshot->getParameterByDistance( 1.0, 1.0e-6 ), splineParameter2{ 0, 1.0f } ), "an exact internal join belongs to the preceding segment" );
    check( !uniformSnapshot->getParameterByDistance( 0.1, 1.0e-10 ) && uniformSnapshot->getParameterByDistance( 0.1, 1.0e-6 ).has_value(), "inverse rejects insufficient float parameter precision" );
    check( !uniformSnapshot->getParameterByDistance( 0.0, std::numeric_limits<double>::denorm_min() ), "inverse start still requires a computable total length" );
    check( uniformSnapshot->getLength( { 0, 0.25f }, { 2, 0.75f }, 1.0e-6 ) == 2.5, "prepared range returns its analytic length across three segments" );
    check( uniformSnapshot->getLength( { 0, 1.0f }, { 2, 0.0f }, 1.0e-6 ) == 1.0, "join endpoints exclude empty intervals" );
    check( uniformSnapshot->getLength( { 1, 0.37f }, { 1, 0.37f }, std::numeric_limits<double>::denorm_min() ) == 0.0, "equal range parameters return zero at the smallest positive tolerance" );
    const double rangeBoundary = 384.0 * std::numeric_limits<double>::epsilon();
    check( !uniformSnapshot->getLength( { 0, 0.25f }, { 2, 0.75f }, std::nextafter( rangeBoundary, 0.0 ) ) && uniformSnapshot->getLength( { 0, 0.25f }, { 2, 0.75f }, rangeBoundary ) == 2.5, "range precision threshold uses only the included nonempty intervals" );
    const vec2 line[] = { { 1, 2 }, { 5, 4 } };
    const auto preparedLength = prepareSpline( { line } )->getLength( 1.0e-6 );
    check( preparedLength && std::abs( *preparedLength - std::sqrt( 20.0 ) ) < 1.0e-14, "prepared line returns its analytic total length" );
    const auto lineSnapshot = prepareSpline( { line } );
    const auto middleLength = lineSnapshot->getLength( 0, 0.25f, 0.75f, 1.0e-6 );
    check( middleLength && std::abs( *middleLength - 0.5 * std::sqrt( 20.0 ) ) < 1.0e-14, "prepared line returns its analytic middle interval length" );
    check( lineSnapshot->getLength( 0, 0.37f, 0.37f, std::numeric_limits<double>::denorm_min() ) == 0.0, "equal parameters return zero even at the smallest positive tolerance" );
    const auto halfDistance = lineSnapshot->getDistance( 0, 0.5f, 1.0e-6 );
    check( halfDistance && std::abs( *halfDistance - 0.5 * std::sqrt( 20.0 ) ) < 1.0e-14, "prepared line returns its analytic cumulative distance" );
    check( lineSnapshot->getDistance( 0, 0.0f, std::numeric_limits<double>::denorm_min() ) == 0.0, "start distance remains zero at any positive finite tolerance" );
    const vec2 curve[] = { { -3, 1 }, { -1.7f, -0.1f }, { 0.2f, 4.3f }, { 2.9f, -1.2f }, { 5, 2 } };
    const vec2 cusp[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
    compare( { line } );
    compare( { curve } );
    compare( { curve, true } );
    compare( { cusp } );
    const vec2 triangle[] = { { 0, 0 }, { 2, 0 }, { 0, 1 } };
    compare( { triangle, true } );
    check( prepareSpline( { cusp } )->getLength( 1.0e-6 ) == 2.0, "zero-tangent geometry retains its positive length" );
    check( prepareSpline( { cusp } )->getLength( 0, 0.5f, 1.0f, 1.0e-6 ) == 0.375, "partial length permits a zero tangent endpoint" );
    const vec2 horizontal[] = { { 0, 0 }, { 8, 0 } };
    const auto horizontalSnapshot = prepareSpline( { horizontal } );
    const float nearHalf = std::nextafter( 0.5f, 1.0f );
    check( horizontalSnapshot->getLength( 0, 0.5f, nearHalf, 1.0e-12 ) == 8.0 * ( static_cast<double>( nearHalf ) - 0.5 ), "a short interval retains its analytic length" );
    const double partialBoundary = 128.0 * std::numeric_limits<double>::epsilon() * 4.0;
    check( !horizontalSnapshot->getLength( 0, 0.25f, 0.75f, std::nextafter( partialBoundary, 0.0 ) ) && horizontalSnapshot->getLength( 0, 0.25f, 0.75f, partialBoundary ) == 4.0, "partial length preserves the precision threshold boundary" );
    check( prepareSpline( { cusp } )->getLength( { 0, 0.5f }, { 1, 0.5f }, 1.0e-6 ) == 0.75, "range length integrates across a zero tangent join" );
    check( equal( prepareSpline( { cusp } )->getParameterByDistance( 1.0, 1.0e-6 ), splineParameter2{ 0, 1.0f } ), "inverse permits a zero tangent at the exact join" );
    check( equal( prepareSpline( { cusp } )->getParameterByDistance( { 0, 0.5f }, { 1, 0.5f }, 0.375, 1.0e-6 ), splineParameter2{ 0, 1.0f } ), "range inverse permits a zero tangent at an exact join" );
    const auto cuspSnapshot = prepareSpline( { cusp } );
    check( !cuspSnapshot->evaluateByDistance( 1.0, 1.0e-6 ) && !cuspSnapshot->evaluateByDistance( { 0, 0.5f }, { 1, 0.5f }, 0.375, 1.0e-6 ) && !cuspSnapshot->evaluateByDistance( { 0, 1.0f }, { 1, 0.0f }, 0.0, 1.0e-6 ), "distance samples reject a zero tangent even when inverse queries succeed" );
    const vec2 offsetLine[] = { { 1.0e8f, 0 }, { 1.0e8f + 8.0f, 0 } };
    const auto offsetSnapshot = prepareSpline( { offsetLine } );
    check( offsetSnapshot->getParameterByDistance( 1.0, 1.0e-4 ).has_value() && !offsetSnapshot->evaluateByDistance( 1.0, 1.0e-4 ) && !offsetSnapshot->evaluateByDistance( { 0, 0.0f }, { 0, 1.0f }, 1.0, 1.0e-4 ), "distance samples reject unrepresentable positions after successful inversion" );
    check( equal( cuspSnapshot->sampleByDistanceParameters( { 0, 0.0f }, { 1, 1.0f }, 2, 1.0e-6 ), std::vector<splineParameter2>{ { 0, 0.0f }, { 0, 1.0f }, { 1, 1.0f } } ), "parameter samples permit a zero tangent at a sample" );
    check( offsetSnapshot->sampleByDistanceParameters( { 0, 0.0f }, { 0, 1.0f }, 8, 1.0e-4 ).size() == 9, "parameter sampling does not require representable rounded positions" );
    check( equal( cuspSnapshot->sampleByDistance( 2, 1.0e-6 ), std::vector<vec2>{ { 0, 0 }, { 1, 0 }, { 0, 0 } } ) && equal( cuspSnapshot->sampleByDistance( { 0, 1.0f }, { 1, 0.0f }, 2, 1.0e-6 ), std::vector<vec2>( 3, { 1, 0 } ) ), "position samples permit zero tangents including zero-length ranges" );
    check( offsetSnapshot->sampleByDistance( 8, 1.0e-4 ).empty() && offsetSnapshot->sampleByDistance( { 0, 0.0f }, { 0, 1.0f }, 8, 1.0e-4 ).empty(), "position precision failure discards all samples after a valid first endpoint" );
    check( equal( offsetSnapshot->sampleByDistance( 1, 1.0e-4 ), std::vector<vec2>{ offsetLine[0], offsetLine[1] } ), "exact endpoint positions bypass interior rounding checks" );
    const double precisionBoundary = 128.0 * std::numeric_limits<double>::epsilon() * *preparedLength;
    for( double tolerance : { std::nextafter( precisionBoundary, 0.0 ), precisionBoundary, std::nextafter( precisionBoundary, std::numeric_limits<double>::infinity() ) } )
        check( prepareSpline( { line } )->getLength( tolerance ) == GetLength( { line }, tolerance ), "prepared length preserves the representable-precision boundary policy" );
    for( float scale : { 1.0e-8f, 1.0f, 1.0e8f } )
    {
        std::vector<vec2> scaled;
        for( auto point : curve )
            scaled.push_back( scale * point );
        compare( { scaled } );
        compare( { scaled, true } );
    }

    for( bool loop : { false, true } )
    {
        const catmullRomSpline2 spline{ curve, loop };
        const auto prepared = prepareSpline( spline );
        for( double tolerance : { 1.0e-4, 1.0e-10 } )
        {
            const auto length = GetLength( spline, tolerance );
            check( length.has_value(), "distance equivalence fixture has finite length" );
            const splineParameter2 from{ 0, 0.0f }, to{ GetSegmentCount( spline ) - 1, 1.0f };
            for( int step = 0; step <= 16; ++step )
            {
                const double distance = *length * step / 16;
                const auto expected = EvaluateByDistance( spline, distance, tolerance );
                const auto parameter = GetParameterByDistance( spline, from, to, distance, tolerance );
                const auto actual = parameter ? prepared->evaluateSample( parameter->segmentIndex, parameter->t, tolerance ) : std::nullopt;
                check( expected.has_value() == actual.has_value() && ( !expected || ( equal( expected->point, actual->point ) && equal( expected->tangent, actual->tangent ) ) ), "cached final sampling preserves distance-query results and precision failures" );
            }
        }
    }
    const auto preparedLine = prepareSpline( { line } );
    check( preparedLine->evaluateSample( 0, 0.5f, 1.0e-4 ).has_value(), "prepared sample returns a finite point and unit tangent" );
    const auto preparedCusp = prepareSpline( { cusp } );
    check( !preparedCusp->evaluateSample( 0, 1.0f, 1.0e-4 ), "prepared sample rejects a zero tangent" );

    const vec2 unevenPrefix[] = { { 0, 0 }, { 1, 0 }, { 1.0e10f, 0 } };
    const auto prefixSnapshot = prepareSpline( { unevenPrefix } );
    const auto prefix = prefixSnapshot->getDistance( 0, 1.0f, 1.0e-10 );
    check( !prefixSnapshot->getLength( 1.0e-10 ) && prefix && prefix == GetDistance( { unevenPrefix }, 0, 1.0f, 1.0e-10 ), "a computable prefix does not require the full length to converge" );

    std::optional<preparedCatmullRomSpline2> owned;
    vec2 expectedPoint{}, expectedFirst{}, expectedSecond{};
    std::optional<splineParameter2> expectedParameter, expectedRangeParameter;
    std::optional<splineSample2> expectedSample, expectedRangeSample;
    std::vector<splineParameter2> expectedParameters;
    std::vector<vec2> expectedPoints, expectedRangePoints;
    std::optional<double> expectedLength, expectedDistance, expectedPartialLength, expectedRangeLength;
    {
        std::vector<vec2> source{ std::begin( curve ), std::end( curve ) };
        owned = prepareSpline( { source, true } );
        expectedPoints = SampleByDistance( { source, true }, 3, 1.0e-4 );
        expectedRangePoints = SampleByDistance( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 );
        expectedParameters = SampleByDistanceParameters( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 );
        expectedSample = EvaluateByDistance( { source, true }, 3.5, 1.0e-4 );
        expectedRangeSample = EvaluateByDistance( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 );
        expectedRangeParameter = GetParameterByDistance( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 );
        expectedParameter = GetParameterByDistance( { source, true }, 3.5, 1.0e-4 );
        expectedLength = GetLength( { source, true }, 1.0e-6 );
        expectedRangeLength = GetLength( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 1.0e-6 );
        expectedPartialLength = GetLength( { source, true }, 4, 0.25f, 0.75f, 1.0e-6 );
        expectedDistance = GetDistance( { source, true }, 4, 0.37f, 1.0e-6 );
        expectedPoint = Evaluate( { source, true }, 4, 0.37f );
        expectedFirst = Derivative( { source, true }, 4, 0.37f );
        expectedSecond = SecondDerivative( { source, true }, 4, 0.37f );
        source.assign( 1000, { 100, 200 } );
    }
    check( owned && equal( owned->evaluate( 4, 0.37f ), expectedPoint ) && equal( owned->derivative( 4, 0.37f ), expectedFirst ) && equal( owned->secondDerivative( 4, 0.37f ), expectedSecond ), "snapshot survives source mutation reallocation and destruction" );
    check( expectedLength && owned->getLength( 1.0e-6 ) == expectedLength, "prepared total length survives source mutation and destruction" );
    check( expectedDistance && owned->getDistance( 4, 0.37f, 1.0e-6 ) == expectedDistance, "prepared distance survives source mutation and destruction" );
    check( expectedPartialLength && owned->getLength( 4, 0.25f, 0.75f, 1.0e-6 ) == expectedPartialLength, "partial length survives source mutation and destruction" );
    check( expectedRangeLength && owned->getLength( { 1, 0.25f }, { 4, 0.75f }, 1.0e-6 ) == expectedRangeLength, "range length survives source mutation and destruction" );
    check( expectedParameter && equal( owned->getParameterByDistance( 3.5, 1.0e-4 ), expectedParameter ), "inverse survives source mutation and destruction" );
    check( expectedRangeParameter && equal( owned->getParameterByDistance( { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 ), expectedRangeParameter ), "range inverse survives source mutation and destruction" );
    check( expectedSample && expectedRangeSample && equal( owned->evaluateByDistance( 3.5, 1.0e-4 ), expectedSample ) && equal( owned->evaluateByDistance( { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 ), expectedRangeSample ), "distance samples survive source mutation and destruction" );
    check( !expectedParameters.empty() && equal( owned->sampleByDistanceParameters( { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 ), expectedParameters ), "parameter samples survive source mutation and destruction" );
    check( !expectedPoints.empty() && !expectedRangePoints.empty() && equal( owned->sampleByDistance( 3, 1.0e-4 ), expectedPoints ) && equal( owned->sampleByDistance( { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 ), expectedRangePoints ), "position samples survive source mutation and destruction" );
    auto copied = *owned;
    auto moved = std::move( copied );
    owned.reset();
    check( moved.isLoop() && moved.getSegmentCount() == 5 && equal( moved.evaluate( 4, 0.37f ), expectedPoint ), "copied and moved snapshots own their data" );

    check( moved.getLength( 1.0e-6 ) == expectedLength, "copied and moved snapshots preserve total length" );
    check( moved.getDistance( 4, 0.37f, 1.0e-6 ) == expectedDistance, "copied and moved snapshots preserve cumulative distance" );
    check( moved.getLength( 4, 0.25f, 0.75f, 1.0e-6 ) == expectedPartialLength, "copied and moved snapshots preserve partial length" );
    check( moved.getLength( { 1, 0.25f }, { 4, 0.75f }, 1.0e-6 ) == expectedRangeLength, "copied and moved snapshots preserve range length" );
    check( equal( moved.getParameterByDistance( 3.5, 1.0e-4 ), expectedParameter ), "copied and moved snapshots preserve inverse queries" );
    check( equal( moved.getParameterByDistance( { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 ), expectedRangeParameter ), "copied and moved snapshots preserve range inverse queries" );
    check( equal( moved.evaluateByDistance( 3.5, 1.0e-4 ), expectedSample ) && equal( moved.evaluateByDistance( { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 ), expectedRangeSample ), "copied and moved snapshots preserve distance samples" );
    check( equal( moved.sampleByDistanceParameters( { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 ), expectedParameters ), "copied and moved snapshots preserve parameter samples" );
    check( equal( moved.sampleByDistance( 3, 1.0e-4 ), expectedPoints ) && equal( moved.sampleByDistance( { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 ), expectedRangePoints ), "copied and moved snapshots preserve position samples" );
    const preparedCatmullRomSpline2 empty{};
    check( !empty.getLength( 1.0e-6 ), "empty prepared geometry has no total length" );
    check( !empty.getDistance( 0, 0.0f, 1.0e-6 ), "empty prepared geometry has no cumulative distance" );
    for( double tolerance : { 0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() } )
        check( !moved.getLength( tolerance ), "prepared total length rejects invalid tolerance without asserting" );
    const float nan = std::numeric_limits<float>::quiet_NaN(), infinity = std::numeric_limits<float>::infinity();
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ) } )
        check( !moved.getDistance( 0, 0.0f, tolerance ), "invalid tolerances are rejected even at the zero distance endpoint" );
    for( float t : { -0.01f, 1.01f, nan, infinity } )
        check( !moved.getDistance( 0, t, 1.0e-6 ), "prepared distance rejects invalid parameters without asserting" );
    for( std::size_t i : { moved.getSegmentCount(), std::numeric_limits<std::size_t>::max() } )
        check( !moved.getDistance( i, 0.0f, 1.0e-6 ), "invalid segments cannot masquerade as zero distance" );
    check( !empty.getLength( 0, 0.5f, 0.5f, 1.0e-6 ), "empty geometry cannot return an equal-parameter length" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( !moved.getLength( 0, 0.5f, 0.5f, tolerance ), "partial length validates tolerance before equal parameters" );
    for( float t : { -0.01f, 1.01f, nan, infinity, -infinity } )
        check( !moved.getLength( 0, t, 0.5f, 1.0e-6 ) && !moved.getLength( 0, 0.5f, t, 1.0e-6 ) && !moved.getLength( 0, t, t, 1.0e-6 ), "partial length rejects invalid parameters in either endpoint" );
    for( std::size_t i : { moved.getSegmentCount(), std::numeric_limits<std::size_t>::max() } )
        check( !moved.getLength( i, 0.5f, 0.5f, 1.0e-6 ), "partial length validates segment before equal parameters" );
    check( !moved.getLength( 0, 0.75f, 0.25f, 1.0e-6 ), "partial length rejects reversed intervals" );
    const vec2 longLine[] = { { 0, 0 }, { 1.0e10f, 0 } };
    const auto longSnapshot = prepareSpline( { longLine } );
    const auto shortLength = longSnapshot->getLength( 0, std::nextafter( 1.0f, 0.0f ), 1.0f, 1.0e-10 );
    check( !longSnapshot->getLength( 1.0e-10 ) && shortLength && *shortLength > 0.0 && shortLength == GetLength( { longLine }, 0, std::nextafter( 1.0f, 0.0f ), 1.0f, 1.0e-10 ), "a short interval does not require full or cumulative lengths to converge" );
    check( !empty.getLength( { 0, 0.0f }, { 0, 0.0f }, 1.0e-6 ), "empty geometry cannot return an equal-position range length" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( !moved.getLength( { 0, 1.0f }, { 1, 0.0f }, tolerance ), "range tolerance is validated before equal-position handling" );
    for( splineParameter2 invalid : { splineParameter2{ moved.getSegmentCount(), 0.0f }, { std::numeric_limits<std::size_t>::max(), 0.0f }, { 0, -0.01f }, { 0, 1.01f }, { 0, nan }, { 0, infinity }, { 0, -infinity } } )
        check( !moved.getLength( invalid, { 4, 1.0f }, 1.0e-6 ) && !moved.getLength( { 0, 0.0f }, invalid, 1.0e-6 ) && !moved.getLength( invalid, invalid, 1.0e-6 ), "range length rejects invalid parameters before normalization or equality" );
    check( !moved.getLength( { 4, 1.0f }, { 0, 0.0f }, 1.0e-6 ), "closed range does not wrap from the final endpoint to the start" );
    const vec2 longPrefix[] = { { -1.0e10f, 0 }, { -1, 0 }, { 0, 0 }, { 1, 0 }, { 2, 0 } };
    const auto tailSnapshot = prepareSpline( { longPrefix } );
    check( !tailSnapshot->getLength( 1.0e-10 ) && !tailSnapshot->getDistance( 3, 0.75f, 1.0e-10 ) && tailSnapshot->getLength( { 2, 0.25f }, { 3, 0.75f }, 1.0e-10 ) == 1.5, "range length does not require unrelated prefix or total lengths to converge" );
    check( !empty.getParameterByDistance( 0.0, 1.0e-6 ), "empty geometry has no inverse even at zero distance" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( !moved.getParameterByDistance( 0.0, tolerance ), "inverse rejects invalid tolerances even at the start" );
    for( double distance : { -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( !moved.getParameterByDistance( distance, 1.0e-6 ), "inverse rejects invalid distances without asserting" );
    check( !moved.getParameterByDistance( 2.0 * *expectedLength, 1.0e-6 ), "closed inverse rejects distance beyond one traversal instead of wrapping" );
    check( !empty.getParameterByDistance( { 0, 0.0f }, { 0, 0.0f }, 0.0, 1.0e-6 ), "empty geometry has no range inverse even at zero distance" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( !moved.getParameterByDistance( { 0, 1.0f }, { 1, 0.0f }, 0.0, tolerance ), "range inverse validates tolerance before returning an equal-position endpoint" );
    for( double distance : { -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( !moved.getParameterByDistance( { 0, 0.0f }, { 4, 1.0f }, distance, 1.0e-6 ), "range inverse rejects invalid distances" );
    for( splineParameter2 invalid : { splineParameter2{ moved.getSegmentCount(), 0.0f }, { std::numeric_limits<std::size_t>::max(), 0.0f }, { 0, -0.01f }, { 0, 1.01f }, { 0, nan }, { 0, infinity }, { 0, -infinity } } )
        check( !moved.getParameterByDistance( invalid, { 4, 1.0f }, 0.0, 1.0e-6 ) && !moved.getParameterByDistance( { 0, 0.0f }, invalid, 0.0, 1.0e-6 ) && !moved.getParameterByDistance( invalid, invalid, 0.0, 1.0e-6 ), "range inverse validates both endpoints before normalization or equality" );
    check( !moved.getParameterByDistance( { 4, 1.0f }, { 0, 0.0f }, 0.0, 1.0e-6 ) && !moved.getParameterByDistance( { 1, 0.75f }, { 1, 0.25f }, 0.0, 1.0e-6 ), "range inverse rejects reverse traversal without closed wrapping" );
    check( !moved.getParameterByDistance( { 1, 0.25f }, { 4, 0.75f }, 2.0 * *expectedRangeLength, 1.0e-6 ), "range inverse rejects distances beyond the specified range" );
    check( !tailSnapshot->getParameterByDistance( 0.5, 1.0e-10 ) && equal( tailSnapshot->getParameterByDistance( { 2, 0.25f }, { 3, 0.75f }, 0.5, 1.0e-10 ), splineParameter2{ 2, 0.75f } ), "range inverse does not require unrelated prefix or total lengths to converge" );
    check( !empty.evaluateByDistance( 0.0, 1.0e-6 ) && !empty.evaluateByDistance( { 0, 0.0f }, { 0, 0.0f }, 0.0, 1.0e-6 ), "empty geometry has no distance sample" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( !moved.evaluateByDistance( 0.0, tolerance ) && !moved.evaluateByDistance( { 0, 0.0f }, { 4, 1.0f }, 0.0, tolerance ), "distance samples reject invalid tolerances at the start" );
    for( double distance : { -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ), 2.0 * *expectedLength } )
        check( !moved.evaluateByDistance( distance, 1.0e-6 ) && !moved.evaluateByDistance( { 0, 0.0f }, { 4, 1.0f }, distance, 1.0e-6 ), "distance samples reject invalid distances without closed wrapping" );
    for( splineParameter2 invalid : { splineParameter2{ moved.getSegmentCount(), 0.0f }, { 0, -0.01f }, { 0, 1.01f }, { 0, nan }, { 0, infinity } } )
        check( !moved.evaluateByDistance( invalid, { 4, 1.0f }, 0.0, 1.0e-6 ) && !moved.evaluateByDistance( { 0, 0.0f }, invalid, 0.0, 1.0e-6 ), "distance samples reject invalid range endpoints" );
    check( !moved.evaluateByDistance( { 4, 1.0f }, { 0, 0.0f }, 0.0, 1.0e-6 ), "distance samples reject reverse ranges" );
    check( equal( tailSnapshot->evaluateByDistance( { 2, 0.25f }, { 3, 0.75f }, 0.5, 1.0e-10 ), splineSample2{ { 0.75f, 0 }, { 1, 0 } } ), "partial distance sample is independent of unrelated prefix failures" );
    check( empty.sampleByDistanceParameters( { 0, 0.0f }, { 0, 0.0f }, 1, 1.0e-6 ).empty(), "empty geometry has no parameter samples" );
    for( std::size_t subdivisions : { std::size_t{ 0 }, std::vector<splineParameter2>{}.max_size(), std::numeric_limits<std::size_t>::max() } )
        check( moved.sampleByDistanceParameters( { 0, 0.0f }, { 4, 1.0f }, subdivisions, 1.0e-6 ).empty(), "parameter sampling rejects zero and impossible subdivision counts" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( moved.sampleByDistanceParameters( { 0, 1.0f }, { 1, 0.0f }, 3, tolerance ).empty(), "parameter samples validate tolerance even for equal-position ranges" );
    for( splineParameter2 invalid : { splineParameter2{ moved.getSegmentCount(), 0.0f }, { std::numeric_limits<std::size_t>::max(), 0.0f }, { 0, -0.01f }, { 0, 1.01f }, { 0, nan }, { 0, infinity } } )
        check( moved.sampleByDistanceParameters( invalid, { 4, 1.0f }, 3, 1.0e-6 ).empty() && moved.sampleByDistanceParameters( { 0, 0.0f }, invalid, 3, 1.0e-6 ).empty(), "parameter samples reject invalid range endpoints" );
    check( moved.sampleByDistanceParameters( { 4, 1.0f }, { 0, 0.0f }, 3, 1.0e-6 ).empty() && moved.sampleByDistanceParameters( { 1, 0.75f }, { 1, 0.25f }, 3, 1.0e-6 ).empty(), "parameter sampling rejects reverse ranges without closed wrapping" );
    const auto tailParameters = tailSnapshot->sampleByDistanceParameters( { 2, 0.25f }, { 3, 0.75f }, 3, 1.0e-6 );
    check( tailParameters.size() == 4 && equal( tailParameters, SampleByDistanceParameters( { longPrefix }, { 2, 0.25f }, { 3, 0.75f }, 3, 1.0e-6 ) ), "parameter samples do not require unrelated prefix lengths to converge" );
    check( empty.sampleByDistance( 1, 1.0e-6 ).empty() && empty.sampleByDistance( { 0, 0.0f }, { 0, 0.0f }, 1, 1.0e-6 ).empty(), "empty geometry has no position samples" );
    for( std::size_t subdivisions : { std::size_t{ 0 }, std::vector<vec2>{}.max_size(), std::vector<splineParameter2>{}.max_size(), std::numeric_limits<std::size_t>::max() } )
        check( moved.sampleByDistance( subdivisions, 1.0e-6 ).empty() && moved.sampleByDistance( { 0, 0.0f }, { 4, 1.0f }, subdivisions, 1.0e-6 ).empty(), "position sampling rejects zero and impossible output sizes" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ) } )
        check( moved.sampleByDistance( 3, tolerance ).empty() && moved.sampleByDistance( { 0, 1.0f }, { 1, 0.0f }, 3, tolerance ).empty(), "position sampling validates tolerance even for zero-length ranges" );
    for( splineParameter2 invalid : { splineParameter2{ moved.getSegmentCount(), 0.0f }, { std::numeric_limits<std::size_t>::max(), 0.0f }, { 0, -0.01f }, { 0, 1.01f }, { 0, nan }, { 0, infinity } } )
        check( moved.sampleByDistance( invalid, { 4, 1.0f }, 3, 1.0e-6 ).empty() && moved.sampleByDistance( { 0, 0.0f }, invalid, 3, 1.0e-6 ).empty(), "position samples reject invalid range endpoints" );
    check( moved.sampleByDistance( { 4, 1.0f }, { 0, 0.0f }, 3, 1.0e-6 ).empty() && moved.sampleByDistance( { 1, 0.75f }, { 1, 0.25f }, 3, 1.0e-6 ).empty(), "position sampling rejects reverse ranges without closed wrapping" );
    const auto tailPoints = tailSnapshot->sampleByDistance( { 2, 0.25f }, { 3, 0.75f }, 3, 1.0e-6 );
    check( tailPoints.size() == 4 && equal( tailPoints, SampleByDistance( { longPrefix }, { 2, 0.25f }, { 3, 0.75f }, 3, 1.0e-6 ) ), "partial position sampling is independent of unrelated prefix failure" );
    const vec2 repeated[] = { { 0, 0 }, { 1, 0 }, { 1, 0 } };
    const vec2 nonfinite[] = { { 0, 0 }, { nan, 1 } };
    const vec2 infinite[] = { { 0, 0 }, { infinity, 1 } };
    const vec2 closedDuplicate[] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 0 } };
    check( !prepareSpline( {} ) && !prepareSpline( { std::span{ line, 1 } } ) && !prepareSpline( { line, true } ), "preparation rejects empty and undersized paths" );
    check( !prepareSpline( { repeated } ) && !prepareSpline( { nonfinite } ) && !prepareSpline( { infinite } ) && !prepareSpline( { closedDuplicate, true } ), "preparation rejects duplicate and nonfinite geometry" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ) } )
        check( !moved.evaluateSample( 0, 0.5f, tolerance ), "prepared sampling rejects invalid precision tolerances" );
    for( float t : { -0.01f, 1.01f, nan, infinity } )
        check( !moved.evaluateSample( 0, t, 1.0e-4 ), "prepared sampling rejects invalid parameters without asserting" );
    check( !moved.evaluateSample( moved.getSegmentCount(), 0.5f, 1.0e-4 ), "prepared sampling rejects an invalid segment" );
#ifdef NDEBUG
    for( float t : { -0.01f, 1.01f, nan, infinity } )
        check( equal( moved.evaluate( 0, t ), {} ) && equal( moved.derivative( 0, t ), {} ) && equal( moved.secondDerivative( 0, t ), {} ), "Release invalid parameters return zero vectors" );
    for( std::size_t i : { moved.getSegmentCount(), std::numeric_limits<std::size_t>::max() } )
        check( equal( moved.evaluate( i, 0.5f ), {} ) && equal( moved.derivative( i, 0.5f ), {} ) && equal( moved.secondDerivative( i, 0.5f ), {} ), "Release invalid segment indices return zero vectors" );
#endif
    return EXIT_SUCCESS;
}
