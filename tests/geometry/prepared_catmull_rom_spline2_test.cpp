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
    const vec2 curve[] = { { -3, 1 }, { -1.7f, -0.1f }, { 0.2f, 4.3f }, { 2.9f, -1.2f }, { 5, 2 } };
    const vec2 cusp[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
    compare( { line } );
    compare( { curve } );
    compare( { curve, true } );
    compare( { cusp } );
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

    std::optional<preparedCatmullRomSpline2> owned;
    vec2 expectedPoint{}, expectedFirst{}, expectedSecond{};
    {
        std::vector<vec2> source{ std::begin( curve ), std::end( curve ) };
        owned = prepareSpline( { source, true } );
        expectedPoint = Evaluate( { source, true }, 4, 0.37f );
        expectedFirst = Derivative( { source, true }, 4, 0.37f );
        expectedSecond = SecondDerivative( { source, true }, 4, 0.37f );
        source.assign( 1000, { 100, 200 } );
    }
    check( owned && equal( owned->evaluate( 4, 0.37f ), expectedPoint ) && equal( owned->derivative( 4, 0.37f ), expectedFirst ) && equal( owned->secondDerivative( 4, 0.37f ), expectedSecond ), "snapshot survives source mutation reallocation and destruction" );
    auto copied = *owned;
    auto moved = std::move( copied );
    owned.reset();
    check( moved.isLoop() && moved.getSegmentCount() == 5 && equal( moved.evaluate( 4, 0.37f ), expectedPoint ), "copied and moved snapshots own their data" );

    const float nan = std::numeric_limits<float>::quiet_NaN(), infinity = std::numeric_limits<float>::infinity();
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
