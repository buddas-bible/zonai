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
        for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            for( float t : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.125f, 0.37f, 0.5f, 0.875f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                check( prepared->getDistance( i, t, tolerance ) == GetDistance( spline, i, t, tolerance ), "prepared cumulative distance preserves exact results and precision failures" );
        for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            for( float t0 : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.125f, 0.37f, 0.5f, 0.875f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                for( float t1 : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.125f, 0.37f, 0.5f, 0.875f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                    if( t0 <= t1 ) check( prepared->getLength( i, t0, t1, tolerance ) == GetLength( spline, i, t0, t1, tolerance ), "prepared partial length preserves exact results and precision failures" );
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
    std::optional<double> expectedLength, expectedDistance, expectedPartialLength;
    {
        std::vector<vec2> source{ std::begin( curve ), std::end( curve ) };
        owned = prepareSpline( { source, true } );
        expectedLength = GetLength( { source, true }, 1.0e-6 );
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
    auto copied = *owned;
    auto moved = std::move( copied );
    owned.reset();
    check( moved.isLoop() && moved.getSegmentCount() == 5 && equal( moved.evaluate( 4, 0.37f ), expectedPoint ), "copied and moved snapshots own their data" );

    check( moved.getLength( 1.0e-6 ) == expectedLength, "copied and moved snapshots preserve total length" );
    check( moved.getDistance( 4, 0.37f, 1.0e-6 ) == expectedDistance, "copied and moved snapshots preserve cumulative distance" );
    check( moved.getLength( 4, 0.25f, 0.75f, 1.0e-6 ) == expectedPartialLength, "copied and moved snapshots preserve partial length" );
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
