#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>

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
#ifdef NDEBUG
    for( float t : { -0.01f, 1.01f, nan, infinity } )
        check( equal( moved.evaluate( 0, t ), {} ) && equal( moved.derivative( 0, t ), {} ) && equal( moved.secondDerivative( 0, t ), {} ), "Release invalid parameters return zero vectors" );
    for( std::size_t i : { moved.getSegmentCount(), std::numeric_limits<std::size_t>::max() } )
        check( equal( moved.evaluate( i, 0.5f ), {} ) && equal( moved.derivative( i, 0.5f ), {} ) && equal( moved.secondDerivative( i, 0.5f ), {} ), "Release invalid segment indices return zero vectors" );
#endif
    return EXIT_SUCCESS;
}
