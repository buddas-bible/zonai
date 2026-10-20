#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>
#include <type_traits>

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

bool equal( const std::optional<aabb2>& a, const std::optional<aabb2>& b )
{
    return a.has_value() == b.has_value() && ( !a || ( equal( a->min, b->min ) && equal( a->max, b->max ) ) );
}

bool equal( const std::optional<splineClosestPoint2>& a, const std::optional<splineClosestPoint2>& b )
{
    return a.has_value() == b.has_value() && ( !a || ( equal( a->point, b->point ) && a->segmentIndex == b->segmentIndex && a->t == b->t && a->distanceSquared == b->distanceSquared ) );
}

void adaptiveParameters()
{
    const vec2 line[] = { { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 } };
    const auto preparedLine = prepareSpline( { line } );
    check( equal( preparedLine->sampleAdaptiveParameters( { 0, 0.25f }, { 2, 0.75f }, 0.01f ), std::vector<splineParameter2>{ { 0, 0.25f }, { 0, 1.0f }, { 1, 1.0f }, { 2, 0.75f } } ), "prepared adaptive parameters preserve straight range endpoints and preceding join indices" );
    check( equal( preparedLine->sampleAdaptiveParameters( { 0, 1.0f }, { 2, 0.0f }, 0.01f ), std::vector<splineParameter2>{ { 0, 1.0f }, { 2, 0.0f } } ), "adaptive parameters retain aliased input endpoint representations" );
    const vec2 reversal[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
    const auto cusp = prepareSpline( { reversal } );
    for( const splineParameter2 from : { splineParameter2{ 0, 1.0f }, { 1, 0.0f } } )
        check( equal( cusp->sampleAdaptiveParameters( from, { 1 - from.segmentIndex, 1.0f - from.t }, 0.01f ), std::vector<splineParameter2>{ from } ), "equal-position adaptive ranges return from and allow zero tangents" );
    check( equal( cusp->sampleAdaptiveParameters( { 0, 0.5f }, { 1, 0.5f }, 0.01f ), std::vector<splineParameter2>{ { 0, 0.5f }, { 0, 1.0f }, { 1, 0.5f } } ), "adaptive parameters retain a returning path with coincident endpoint positions" );
    check( equal( preparedLine->sampleAdaptiveParameters( { 0, 0.37f }, { 0, 0.37f }, 0.01f ), std::vector<splineParameter2>{ { 0, 0.37f } } ), "same-parameter adaptive range has one entry" );
    const float neighbor = std::nextafter( 1.0f, 0.0f );
    check( equal( preparedLine->sampleAdaptiveParameters( { 0, neighbor }, { 0, 1.0f }, 0.01f ), std::vector<splineParameter2>{ { 0, neighbor }, { 0, 1.0f } } ), "adaptive parameters retain a one-ulp range" );
    const vec2 controls[] = { { -1, -4 }, { 0, 0 }, { 1, 0 }, { 2, 4 } };
    for( float scale : { 1.0e-10f, 1.0f, 1.0e10f } )
        for( bool loop : { false, true } )
        {
            std::vector<vec2> source;
            for( vec2 point : controls )
                source.push_back( scale * point );
            const catmullRomSpline2 spline{ source, loop };
            const auto prepared = prepareSpline( spline );
            const std::size_t count = prepared->getSegmentCount();
            for( float error : { 0.01f * scale, 0.1f * scale, std::numeric_limits<float>::denorm_min() } )
                for( std::size_t i = 0; i < count; ++i )
                    for( std::size_t j = 0; j < count; ++j )
                        for( float t : { 0.0f, 0.1234567f, 1.0f } )
                            check( equal( prepared->sampleAdaptiveParameters( { i, t }, { j, 0.8765432f }, error ), SampleAdaptiveParameters( spline, { i, t }, { j, 0.8765432f }, error ) ), "prepared adaptive parameters preserve exact legacy splits and failures across scales open and closed ranges" );
            const float error = 0.01f * scale;
            const auto parameters = prepared->sampleAdaptiveParameters( { 1, 0.0f }, { 1, 1.0f }, error );
            check( parameters.size() > 2, "adaptive subdivision detects an S curve whose midpoint lies on its chord" );
            for( int i = 0; i <= 400; ++i )
            {
                const vec2 point = prepared->evaluate( 1, static_cast<float>( i ) / 400.0f );
                double distanceSquared = std::numeric_limits<double>::infinity();
                for( std::size_t j = 1; j < parameters.size(); ++j )
                {
                    const vec2 a = prepared->evaluate( parameters[j - 1].segmentIndex, parameters[j - 1].t ), b = prepared->evaluate( parameters[j].segmentIndex, parameters[j].t );
                    const double dx = static_cast<double>( b.x ) - a.x, dy = static_cast<double>( b.y ) - a.y;
                    const double px = static_cast<double>( point.x ) - a.x, py = static_cast<double>( point.y ) - a.y;
                    const double lengthSquared = dx * dx + dy * dy;
                    const double t = lengthSquared > 0.0 ? std::clamp( ( px * dx + py * dy ) / lengthSquared, 0.0, 1.0 ) : 0.0;
                    distanceSquared = std::min( distanceSquared, ( px - t * dx ) * ( px - t * dx ) + ( py - t * dy ) * ( py - t * dy ) );
                }
                check( distanceSquared <= static_cast<double>( error ) * error, "prepared adaptive parameter polyline stays within spatial error on dense S curve samples" );
            }
            const splineParameter2 last{ count - 1, 1.0f };
            const auto expected = SampleAdaptiveParameters( spline, { 0, 0.0f }, last, error );
            check( !expected.empty() && expected.back().segmentIndex == last.segmentIndex && expected.back().t == 1.0f, "adaptive fixture retains the full path endpoint including the closed seam" );
            source.assign( 1000, { 100, 200 } );
            auto copied = *prepared;
            auto moved = std::move( copied );
            check( equal( moved.sampleAdaptiveParameters( { 0, 0.0f }, last, error ), expected ), "adaptive sampling survives source mutation reallocation and snapshot copy move" );
        }
    const float nan = std::numeric_limits<float>::quiet_NaN(), infinity = std::numeric_limits<float>::infinity();
    const preparedCatmullRomSpline2 empty;
    check( empty.sampleAdaptiveParameters( {}, {}, 0.01f ).empty(), "empty snapshot rejects adaptive sampling" );
    for( float error : { 0.0f, -1.0f, nan, infinity, -infinity } )
        check( preparedLine->sampleAdaptiveParameters( {}, { 2, 1.0f }, error ).empty() && preparedLine->sampleAdaptiveParameters( {}, {}, error ).empty(), "adaptive sampling validates errors even for equal ranges" );
    for( splineParameter2 invalid : { splineParameter2{ 3, 0.0f }, { std::numeric_limits<std::size_t>::max(), 1.0f }, { 0, -0.1f }, { 0, 1.1f }, { 0, nan }, { 0, infinity }, { 0, -infinity } } )
        check( preparedLine->sampleAdaptiveParameters( invalid, { 2, 1.0f }, 0.01f ).empty() && preparedLine->sampleAdaptiveParameters( {}, invalid, 0.01f ).empty() && preparedLine->sampleAdaptiveParameters( invalid, invalid, 0.01f ).empty(), "adaptive sampling validates both endpoints before normalization and equality" );
    check( prepareSpline( { controls, true } )->sampleAdaptiveParameters( { 3, 0.5f }, { 0, 0.5f }, 0.01f ).empty() && preparedLine->sampleAdaptiveParameters( { 1, 0.75f }, { 1, 0.25f }, 0.01f ).empty(), "adaptive ranges reject reverse order and implicit wrapping" );
    const vec2 coarse[] = { { 1.0e8f, 1.0e8f }, { 1.0e8f + 8.0f, 1.0e8f + 16.0f } };
    const auto coarseSnapshot = prepareSpline( { coarse } );
    check( coarseSnapshot->sampleAdaptiveParameters( {}, { 0, 1.0f }, 0.01f ).empty() && coarseSnapshot->sampleAdaptiveParameters( { 0, 0.5f }, { 0, 0.75f }, 0.001f ).empty(), "adaptive parameters reject float evaluation and endpoint precision failures without partial output" );
}

void adaptivePositions()
{
    const vec2 line[] = { { 0, 0 }, { 1, 0 }, { 2, 0 } };
    const auto preparedLine = prepareSpline( { line } );
    check( equal( preparedLine->sampleAdaptive( 0.01f ), std::vector<vec2>{ { 0, 0 }, { 1, 0 }, { 2, 0 } } ), "prepared adaptive positions preserve whole straight path endpoints and joins" );
    check( equal( preparedLine->sampleAdaptive( { 0, 0.25f }, { 1, 0.75f }, 0.01f ), std::vector<vec2>{ { 0.25f, 0 }, { 1, 0 }, { 1.75f, 0 } } ), "prepared adaptive positions preserve clipped straight endpoints" );
    check( equal( preparedLine->sampleAdaptive( { 0, 0.5f }, { 0, 0.5f }, 0.01f ), std::vector<vec2>{ { 0.5f, 0 } } ), "equal adaptive position parameters return one point" );
    check( equal( preparedLine->sampleAdaptive( { 0, std::nextafter( 1.0f, 0.0f ) }, { 0, 1.0f }, 1.0e-8f ), std::vector<vec2>{ { std::nextafter( 1.0f, 0.0f ), 0 }, { 1, 0 } } ), "adaptive position sampling retains one-ulp ranges" );
    const vec2 reversal[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
    const auto cusp = prepareSpline( { reversal } );
    for( splineParameter2 from : { splineParameter2{ 0, 1.0f }, { 1, 0.0f } } )
        check( equal( cusp->sampleAdaptive( from, { 1 - from.segmentIndex, 1.0f - from.t }, 0.01f ), std::vector<vec2>{ { 1, 0 } } ), "adaptive join aliases accept zero tangents and return one position" );
    check( equal( cusp->sampleAdaptive( { 0, 0.5f }, { 1, 0.5f }, 0.01f ), std::vector<vec2>{ { 0.625f, 0 }, { 1, 0 }, { 0.625f, 0 } } ), "coincident endpoints preserve a returning adaptive position path" );
    const vec2 controls[] = { { -1, -4 }, { 0, 0 }, { 1, 0 }, { 2, 4 } };
    for( float scale : { 1.0e-10f, 1.0f, 1.0e10f } )
        for( bool loop : { false, true } )
        {
            std::vector<vec2> source;
            for( vec2 point : controls )
                source.push_back( scale * point );
            const catmullRomSpline2 spline{ source, loop };
            const auto prepared = prepareSpline( spline );
            const std::size_t count = prepared->getSegmentCount();
            for( float error : { 0.01f * scale, 0.1f * scale, std::numeric_limits<float>::denorm_min() } )
            {
                check( equal( prepared->sampleAdaptive( error ), SampleAdaptive( spline, error ) ), "prepared whole adaptive positions exactly match legacy samples and failures" );
                for( std::size_t i = 0; i < count; ++i )
                    for( std::size_t j = 0; j < count; ++j )
                        for( float t : { 0.0f, 0.1234567f, 1.0f } )
                            check( equal( prepared->sampleAdaptive( { i, t }, { j, 0.8765432f }, error ), SampleAdaptive( spline, { i, t }, { j, 0.8765432f }, error ) ), "prepared adaptive range positions preserve legacy float results and failures across scales and topology" );
            }
            const float error = 0.01f * scale;
            const auto samples = prepared->sampleAdaptive( { 1, 0.0f }, { 1, 1.0f }, error );
            check( samples.size() > 2, "adaptive position sampling subdivides an S curve with a flat midpoint" );
            for( std::size_t j = 1; j < samples.size(); ++j )
                check( IsFinite( samples[j] ) && !equal( samples[j - 1], samples[j] ), "adaptive positions are finite without consecutive duplicates" );
            for( int i = 0; i <= 400; ++i )
            {
                const vec2 point = prepared->evaluate( 1, static_cast<float>( i ) / 400.0f );
                float distanceSquared = std::numeric_limits<float>::infinity();
                for( std::size_t j = 1; j < samples.size(); ++j )
                    distanceSquared = std::min( distanceSquared, DistanceSquared( segment2{ samples[j - 1], samples[j] }, point ) );
                check( distanceSquared <= ( 1.0001f * error ) * ( 1.0001f * error ), "prepared adaptive position polyline bounds dense S curve evaluations" );
            }
            const auto whole = prepared->sampleAdaptive( error );
            check( !whole.empty() && equal( whole, prepared->sampleAdaptive( { 0, 0.0f }, { count - 1, 1.0f }, error ) ), "whole adaptive sampling delegates to the full range" );
            check( equal( whole.front(), source.front() ) && equal( whole.back(), loop ? source.front() : source.back() ), "adaptive positions preserve whole path endpoints including closed closure" );
            const auto seam = prepared->sampleAdaptive( { count - 1, 0.5f }, { count - 1, 1.0f }, error );
            check( seam.size() >= 2 && equal( seam.back(), whole.back() ) && !equal( seam.front(), seam.back() ), "partial adaptive positions end at the requested endpoint without forced closure" );
        }
    const preparedCatmullRomSpline2 empty;
    check( empty.sampleAdaptive( 0.01f ).empty() && empty.sampleAdaptive( {}, {}, 0.01f ).empty(), "empty snapshots have no adaptive positions" );
    const float nan = std::numeric_limits<float>::quiet_NaN(), infinity = std::numeric_limits<float>::infinity();
    for( float error : { 0.0f, -1.0f, nan, infinity, -infinity } )
        check( preparedLine->sampleAdaptive( error ).empty() && preparedLine->sampleAdaptive( {}, { 1, 1.0f }, error ).empty() && preparedLine->sampleAdaptive( {}, {}, error ).empty(), "adaptive positions validate errors even for equal ranges" );
    for( splineParameter2 invalid : { splineParameter2{ 2, 0.0f }, { std::numeric_limits<std::size_t>::max(), 1.0f }, { 0, -0.1f }, { 0, 1.1f }, { 0, nan }, { 0, infinity }, { 0, -infinity } } )
        check( preparedLine->sampleAdaptive( invalid, { 1, 1.0f }, 0.01f ).empty() && preparedLine->sampleAdaptive( {}, invalid, 0.01f ).empty() && preparedLine->sampleAdaptive( invalid, invalid, 0.01f ).empty(), "adaptive positions validate both endpoints before normalization and equality" );
    const auto closed = prepareSpline( { controls, true } );
    check( closed->sampleAdaptive( { 3, 0.5f }, { 0, 0.5f }, 0.01f ).empty() && preparedLine->sampleAdaptive( { 1, 0.75f }, { 1, 0.25f }, 0.01f ).empty(), "adaptive positions reject reverse ranges without closed wrapping" );
    check( closed->sampleAdaptive( 1.0e-30f ).empty() && closed->sampleAdaptive( { 0, 0.25f }, { 2, 0.75f }, 1.0e-30f ).empty(), "unattainable adaptive position errors discard all partial output" );
    const vec2 coarse[] = { { 1.0e8f, 0 }, { 1.0e8f + 8.0f, 0 } };
    const auto coarseSnapshot = prepareSpline( { coarse } );
    check( equal( coarseSnapshot->sampleAdaptive( 0.01f ), std::vector<vec2>{ coarse[0], coarse[1] } ) && coarseSnapshot->sampleAdaptiveParameters( {}, { 0, 1.0f }, 0.01f ).empty(), "adaptive positions preserve Hermite hull behavior independently of parameter sampling precision" );
    check( equal( coarseSnapshot->sampleAdaptive( {}, { 0, 0.25f }, 10.0f ), std::vector<vec2>{ coarse[0] } ), "adaptive positions remove consecutive float-rounded duplicate endpoints" );
    check( coarseSnapshot->sampleAdaptive( { 0, 0.5f }, { 0, 0.75f }, 0.001f ).empty() && coarseSnapshot->sampleAdaptive( { 0, 0.5f }, { 0, 0.5f }, 0.001f ).empty(), "adaptive positions reject unrepresentable first endpoints including equal ranges" );
    check( coarseSnapshot->sampleAdaptive( {}, { 0, 0.5f }, 0.001f ).empty(), "adaptive positions discard an exact start when the final endpoint lacks precision" );
}

void fixedSamples()
{
    const vec2 line[] = { { 0, 0 }, { 2, 0 }, { 4, 0 } };
    const auto preparedLine = prepareSpline( { line } );
    check( equal( preparedLine->sample( 2 ), std::vector<vec2>{ { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 }, { 4, 0 } } ), "prepared fixed samples preserve straight positions and emit each internal join once" );
    check( equal( preparedLine->sample( 1 ), std::vector<vec2>{ { 0, 0 }, { 2, 0 }, { 4, 0 } } ), "one subdivision returns the original control points" );
    const vec2 controls[] = { { -3, 1 }, { -1.7f, -0.1f }, { 0.2f, 4.3f }, { 2.9f, -1.2f }, { 5, 2 } };
    for( float scale : { 1.0e-10f, 1.0f, 1.0e10f } )
        for( bool loop : { false, true } )
        {
            std::vector<vec2> source;
            for( vec2 point : controls )
                source.push_back( scale * point );
            const catmullRomSpline2 spline{ source, loop };
            const auto prepared = prepareSpline( spline );
            const std::size_t count = prepared->getSegmentCount();
            for( std::size_t subdivisions : { 1u, 2u, 3u, 7u, 16u } )
            {
                const auto samples = prepared->sample( subdivisions );
                check( equal( samples, Sample( spline, subdivisions ) ), "prepared fixed samples preserve exact legacy float values across topology scales and non-dyadic t" );
                check( samples.size() == count * subdivisions + 1 && equal( samples.front(), source.front() ) && equal( samples.back(), loop ? source.front() : source.back() ), "fixed samples preserve count and open or closed endpoint ordering" );
                for( std::size_t i = 0; i < count; ++i )
                    check( equal( samples[i * subdivisions], source[i] ), "fixed samples place each control point at its segment boundary" );
            }
            const std::size_t tooMany = std::vector<vec2>{}.max_size() / count + 1;
            check( prepared->sample( tooMany ).empty() && Sample( spline, tooMany ).empty(), "fixed sampling rejects output size overflow before allocating or multiplying" );
        }
    const vec2 reversal[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
    check( equal( prepareSpline( { reversal } )->sample( 2 ), std::vector<vec2>{ { 0, 0 }, { 0.625f, 0 }, { 1, 0 }, { 0.625f, 0 }, { 0, 0 } } ), "fixed samples preserve a returning path and allow zero tangent joins" );
    const vec2 coarse[] = { { 1.0e8f, 0 }, { 1.0e8f + 8.0f, 0 } };
    const auto coarseSamples = prepareSpline( { coarse } )->sample( 16 );
    check( coarseSamples.size() == 17 && equal( coarseSamples, Sample( { coarse }, 16 ) ) && equal( coarseSamples[0], coarseSamples[1] ), "fixed sampling retains rounded duplicate positions without extra precision rejection" );
    const preparedCatmullRomSpline2 empty;
    check( empty.sample( 0 ).empty() && empty.sample( 3 ).empty(), "empty snapshots have no fixed samples" );
    for( std::size_t subdivisions : { std::size_t{ 0 }, std::vector<vec2>{}.max_size(), std::numeric_limits<std::size_t>::max() } )
        check( preparedLine->sample( subdivisions ).empty() && prepareSpline( { coarse } )->sample( subdivisions ).empty(), "fixed samples reject zero and impossible subdivision counts for single and multiple segments" );
}

void fixedParameters()
{
    const vec2 line[] = { { 0, 0 }, { 2, 0 }, { 4, 0 } };
    const auto preparedLine = prepareSpline( { line } );
    check( equal( preparedLine->sampleParameters( 3 ), std::vector<splineParameter2>{ { 0, 0 }, { 0, 0.33333334f }, { 0, 0.6666667f }, { 0, 1 }, { 1, 0.33333334f }, { 1, 0.6666667f }, { 1, 1 } } ), "fixed parameter samples preserve non-dyadic t values and preceding join indices" );
    check( equal( preparedLine->sampleParameters( 1 ), std::vector<splineParameter2>{ { 0, 0 }, { 0, 1 }, { 1, 1 } } ), "one subdivision preserves the starting representation and segment endpoints" );
    const vec2 controls[] = { { -3, 1 }, { -1.7f, -0.1f }, { 0.2f, 4.3f }, { 2.9f, -1.2f }, { 5, 2 } };
    for( float scale : { 1.0e-10f, 1.0f, 1.0e10f } )
        for( bool loop : { false, true } )
        {
            std::vector<vec2> source;
            for( vec2 point : controls )
                source.push_back( scale * point );
            const auto prepared = prepareSpline( { source, loop } );
            const std::size_t count = prepared->getSegmentCount();
            for( std::size_t subdivisions : { 1u, 2u, 3u, 7u, 16u } )
            {
                const auto parameters = prepared->sampleParameters( subdivisions );
                const auto samples = prepared->sample( subdivisions );
                check( parameters.size() == count * subdivisions + 1 && parameters.size() == samples.size(), "fixed parameter samples match the position sample count" );
                check( parameters.front().segmentIndex == 0 && parameters.front().t == 0.0f && parameters.back().segmentIndex == count - 1 && parameters.back().t == 1.0f, "fixed parameter samples preserve open and closed endpoint representations" );
                for( std::size_t i = 0; i < parameters.size(); ++i )
                    check( equal( prepared->evaluate( parameters[i].segmentIndex, parameters[i].t ), samples[i] ), "evaluated fixed parameters exactly match sample position order and rounding" );
                for( std::size_t i = 1; i <= count; ++i )
                    check( parameters[i * subdivisions].segmentIndex == i - 1 && parameters[i * subdivisions].t == 1.0f, "fixed parameter joins retain the preceding segment without seam normalization" );
            }
            const std::size_t tooMany = std::vector<splineParameter2>{}.max_size() / count + 1;
            check( prepared->sampleParameters( tooMany ).empty(), "fixed parameters reject output size overflow before allocation or multiplication" );
        }
    const vec2 reversal[] = { { 0, 0 }, { 1, 0 }, { 0, 0 } };
    check( equal( prepareSpline( { reversal } )->sampleParameters( 2 ), std::vector<splineParameter2>{ { 0, 0 }, { 0, 0.5f }, { 0, 1 }, { 1, 0.5f }, { 1, 1 } } ), "fixed parameters retain returning paths and allow zero tangent joins" );
    const vec2 coarse[] = { { 1.0e8f, 0 }, { 1.0e8f + 8.0f, 0 } };
    const auto coarseSnapshot = prepareSpline( { coarse } );
    const auto coarseParameters = coarseSnapshot->sampleParameters( 16 );
    check( coarseParameters.size() == 17 && coarseParameters[1].t == 0.0625f && equal( coarseSnapshot->evaluate( 0, coarseParameters[0].t ), coarseSnapshot->evaluate( 0, coarseParameters[1].t ) ), "fixed parameters preserve distinct t values even when float positions coincide" );
    const preparedCatmullRomSpline2 empty;
    check( empty.sampleParameters( 0 ).empty() && empty.sampleParameters( 3 ).empty(), "empty snapshots have no fixed parameter samples" );
    for( std::size_t subdivisions : { std::size_t{ 0 }, std::vector<splineParameter2>{}.max_size(), std::numeric_limits<std::size_t>::max() } )
        check( preparedLine->sampleParameters( subdivisions ).empty() && coarseSnapshot->sampleParameters( subdivisions ).empty(), "fixed parameters reject zero and impossible subdivision counts for single and multiple segments" );
}

void compare( const catmullRomSpline2& spline )
{
    const auto prepared = prepareSpline( spline );
    check( prepared.has_value(), "valid geometry creates a prepared snapshot" );
    check( prepared->getSegmentCount() == GetSegmentCount( spline ) && prepared->isLoop() == spline.isLoop, "prepared snapshot preserves topology" );
    static_assert( std::is_same_v<decltype( prepared->getSegments() ), std::span<const preparedSplineSegment2>> );
    check( prepared->getSegments().size() == prepared->getSegmentCount(), "prepared geometry exposes owned read-only segment coefficients" );
    for( const vec2 query : { vec2{ 0, 0 }, { 0.2f, 0.3f }, { 3, -2 }, { 100, 100 }, spline.points[0] } )
        for( double tolerance : { 1.0e-2, 1.0e-4, 1.0e-10, std::numeric_limits<double>::denorm_min() } )
        {
            check( equal( prepared->closestPoint( query, tolerance ), ClosestPoint( spline, query, tolerance ) ), "prepared closest points preserve exact positions parameters squared distances and failures" );
            for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            {
                const splineParameter2 from{ i, 0.25f }, to{ prepared->getSegmentCount() - 1, 0.75f };
                check( equal( prepared->closestPoint( i, query, tolerance ), ClosestPoint( spline, i, query, tolerance ) ), "prepared segment closest points preserve all legacy fields and failures" );
                check( equal( prepared->closestPoint( from, to, query, tolerance ), ClosestPoint( spline, from, to, query, tolerance ) ), "prepared range closest points preserve all legacy fields and failures" );
                check( equal( prepared->closestPoint( { i, 0.37f }, { i, 0.37f }, query, tolerance ), ClosestPoint( spline, { i, 0.37f }, { i, 0.37f }, query, tolerance ) ), "equal-position closest queries preserve original from representation and precision failures" );
            }
        }
    const auto bounds = prepared->computeAABB();
    check( equal( bounds, ComputeAABB( spline ) ) && bounds && IsValidAABB( *bounds ), "prepared whole bounds exactly match legacy conservative bounds" );
    aabb2 combined{ spline.points[0], spline.points[0] };
    for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
    {
        const auto segmentBounds = prepared->computeAABB( i );
        check( equal( segmentBounds, ComputeAABB( spline, i ) ) && segmentBounds && IsValidAABB( *segmentBounds ), "prepared segment bounds exactly match legacy including the closing segment" );
        combined = Union( combined, *segmentBounds );
        for( int step = 0; step <= 1024; ++step )
        {
            const vec2 point = prepared->evaluate( i, step / 1024.0f );
            check( Contains( *bounds, point ) && Contains( *segmentBounds, point ), "prepared bounds contain dense float evaluations without extra epsilon" );
        }
        for( float t : { std::nextafter( 0.0f, 1.0f ), std::nextafter( 1.0f, 0.0f ) } )
            check( Contains( *segmentBounds, prepared->evaluate( i, t ) ), "prepared segment bounds contain endpoint-neighbor rounding" );
    }
    check( equal( bounds, combined ), "prepared full bounds are the union of all segment bounds" );
    check( equal( prepared->computeAABB( { 0, 0.0f }, { prepared->getSegmentCount() - 1, 1.0f } ), bounds ), "prepared full range includes the closing endpoint and all segment bounds" );
    for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
        for( std::size_t j = 0; j < prepared->getSegmentCount(); ++j )
            for( float t0 : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.37f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                for( float t1 : { 0.0f, std::nextafter( 0.0f, 1.0f ), 0.75f, std::nextafter( 1.0f, 0.0f ), 1.0f } )
                    check( equal( prepared->computeAABB( { i, t0 }, { j, t1 } ), ComputeAABB( spline, { i, t0 }, { j, t1 } ) ), "prepared range bounds preserve exact legacy values and failures including joins and reverse ranges" );
    for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
        for( std::size_t j = i; j < prepared->getSegmentCount(); ++j )
        {
            const auto rangeBounds = prepared->computeAABB( { i, 0.25f }, { j, 0.75f } );
            check( rangeBounds && IsValidAABB( *rangeBounds ), "prepared forward ranges have finite conservative bounds" );
            for( std::size_t segment = i; segment <= j; ++segment )
            {
                const float begin = segment == i ? 0.25f : 0.0f, end = segment == j ? 0.75f : 1.0f;
                for( int step = 0; step <= 256; ++step )
                    check( Contains( *rangeBounds, prepared->evaluate( segment, begin + ( end - begin ) * ( step / 256.0f ) ) ), "prepared range bounds contain dense float evaluations without extra epsilon" );
            }
        }
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
            check( prepared->curvature( i, t ) == Curvature( spline, i, t ), "prepared curvature preserves exact results and zero-tangent failures including endpoints" );
        }
    for( std::size_t i = 0; i < GetSegmentCount( spline ); ++i )
        for( float t : { std::nextafter( 0.0f, 1.0f ), 0.1f, 0.37f, 0.9f, std::nextafter( 1.0f, 0.0f ) } )
        {
            check( equal( prepared->evaluate( i, t ), Evaluate( spline, i, t ) ) && equal( prepared->derivative( i, t ), Derivative( spline, i, t ) ) && equal( prepared->secondDerivative( i, t ), SecondDerivative( spline, i, t ) ), "prepared queries preserve rounding near endpoints and at non-dyadic parameters" );
            check( prepared->curvature( i, t ) == Curvature( spline, i, t ), "prepared curvature preserves non-dyadic and neighboring-endpoint rounding" );
        }
}

}

int main()
{
    fixedParameters();
    fixedSamples();
    adaptivePositions();
    adaptiveParameters();

    const vec2 uniformLine[] = { { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 } };
    const auto uniformSnapshot = prepareSpline( { uniformLine } );
    check( equal( uniformSnapshot->closestPoint( 1, { 0, 1 }, 1.0e-6 ), splineClosestPoint2{ { 1, 0 }, 1, 0.0f, 2.0 } ), "prepared segment closest point excludes unrelated segments" );
    check( equal( uniformSnapshot->closestPoint( { 0, 0.25f }, { 2, 0.75f }, { -1, 0 }, 1.0e-6 ), splineClosestPoint2{ { 0.25f, 0 }, 0, 0.25f, 1.5625 } ) && equal( uniformSnapshot->closestPoint( { 0, 0.25f }, { 2, 0.75f }, { 4, 0 }, 1.0e-6 ), splineClosestPoint2{ { 2.75f, 0 }, 2, 0.75f, 1.5625 } ), "prepared range closest point clamps to both analytic range endpoints" );
    check( equal( uniformSnapshot->closestPoint( { 1, 0.0f }, { 0, 1.0f }, {}, 1.0e-6 ), splineClosestPoint2{ { 1, 0 }, 1, 0.0f, 1.0 } ), "equal join range returns the original from segment and parameter" );
    check( equal( uniformSnapshot->closestPoint( { 0, 0.25f }, { 2, 0.75f }, { 1, 0 }, 1.0e-6 ), splineClosestPoint2{ { 1, 0 }, 0, 1.0f, 0.0 } ), "partial closest point preserves preceding-segment tie selection at joins" );
    check( equal( uniformSnapshot->closestPoint( { 1.5f, 2 }, 1.0e-6 ), splineClosestPoint2{ { 1.5f, 0 }, 1, 0.5f, 4.0 } ), "prepared closest point matches an analytic straight-path projection" );
    check( equal( uniformSnapshot->closestPoint( { -1, 0 }, 1.0e-6 ), splineClosestPoint2{ { 0, 0 }, 0, 0.0f, 1.0 } ) && equal( uniformSnapshot->closestPoint( { 4, 0 }, 1.0e-6 ), splineClosestPoint2{ { 3, 0 }, 2, 1.0f, 1.0 } ), "prepared closest point includes both open endpoints" );
    check( equal( uniformSnapshot->closestPoint( { 1, 0 }, 1.0e-6 ), splineClosestPoint2{ { 1, 0 }, 0, 1.0f, 0.0 } ), "equal-distance join candidates choose the preceding segment" );
    const auto straightRange = uniformSnapshot->computeAABB( { 0, 0.25f }, { 2, 0.75f } );
    check( straightRange && Contains( *straightRange, { 0.25f, 0 } ) && Contains( *straightRange, { 2.75f, 0 } ) && straightRange->min.x > 0.2f && straightRange->max.x < 2.8f, "prepared range bounds include analytic endpoints and exclude unrelated path portions" );
    const auto singlePointBounds = uniformSnapshot->computeAABB( { 1, 0.5f }, { 1, 0.5f } );
    check( singlePointBounds && Contains( *singlePointBounds, { 1.5f, 0 } ) && singlePointBounds->min.x < 1.5f && singlePointBounds->max.x > 1.5f, "equal-position range bounds retain outward rounding allowance" );
    check( equal( uniformSnapshot->computeAABB( { 1, 0.0f }, { 0, 1.0f } ), ComputeAABB( { uniformLine }, { 1, 0.0f }, { 0, 1.0f } ) ), "equivalent join endpoints retain the original from-side rounding policy" );
    const auto straightBounds = uniformSnapshot->computeAABB();
    check( straightBounds && Contains( *straightBounds, { 0, 0 } ) && Contains( *straightBounds, { 3, 0 } ) && straightBounds->min.x < 0.0f && straightBounds->max.x > 3.0f, "prepared straight bounds include endpoints and outward rounding allowance" );
    const auto middleBounds = uniformSnapshot->computeAABB( 1 );
    check( middleBounds && Contains( *middleBounds, { 1, 0 } ) && Contains( *middleBounds, { 2, 0 } ) && middleBounds->min.x > 0.5f && middleBounds->max.x < 2.5f, "prepared segment bounds exclude unrelated straight segments" );
    check( uniformSnapshot->curvature( 0, 0.5f ) == 0.0, "prepared straight-path curvature is zero" );
    const vec2 analyticCurve[] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 2, 1 } };
    const auto curvatureSnapshot = prepareSpline( { analyticCurve } );
    const double analyticCurvature = 1.5 / std::pow( 1.1328125, 1.5 );
    const auto positiveCurvature = curvatureSnapshot->curvature( 1, 0.25f ), negativeCurvature = curvatureSnapshot->curvature( 1, 0.75f );
    check( positiveCurvature && negativeCurvature && std::abs( *positiveCurvature - analyticCurvature ) < 1.0e-12 && std::abs( *negativeCurvature + analyticCurvature ) < 1.0e-12 && curvatureSnapshot->curvature( 1, 0.5f ) == 0.0, "prepared signed curvature matches the analytic cubic and inflection" );
    const vec2 reversedCurve[] = { analyticCurve[3], analyticCurve[2], analyticCurve[1], analyticCurve[0] };
    const auto reversedCurvature = prepareSpline( { reversedCurve } )->curvature( 1, 0.75f );
    check( reversedCurvature && std::abs( *reversedCurvature + analyticCurvature ) < 1.0e-12, "reversing a prepared path reverses curvature sign" );
    for( float scale : { 1.0e-10f, 2.0f, 1.0e10f } )
    {
        const vec2 scaledCurve[] = { scale * analyticCurve[0], scale * analyticCurve[1], scale * analyticCurve[2], scale * analyticCurve[3] };
        const auto scaledCurvature = prepareSpline( { scaledCurve } )->curvature( 1, 0.25f );
        check( scaledCurvature && std::abs( *scaledCurvature * scale - analyticCurvature ) < 1.0e-5, "prepared curvature scales inversely without float overflow or underflow" );
    }
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
    check( !cuspSnapshot->curvature( 0, 1.0f ) && !cuspSnapshot->curvature( 1, 0.0f ), "prepared curvature rejects both sides of a zero tangent join" );
    const auto closedCurvature = prepareSpline( { curve, true } );
    const auto seamStart = closedCurvature->curvature( 0, 0.0f ), seamEnd = closedCurvature->curvature( 4, 1.0f );
    check( seamStart && seamEnd && *seamStart != *seamEnd && seamStart == Curvature( { curve, true }, 0, 0.0f ) && seamEnd == Curvature( { curve, true }, 4, 1.0f ), "closed seam curvature preserves the specified side" );
    const auto overshootBounds = curvatureSnapshot->computeAABB();
    check( overshootBounds && overshootBounds->min.y < 0.0f && overshootBounds->max.y > 1.0f, "prepared bounds include curve overshoot beyond its control polygon" );
    const auto stationaryClosest = cuspSnapshot->closestPoint( { 1.2f, 0 }, 1.0e-6 );
    check( stationaryClosest && equal( stationaryClosest->point, { 1, 0 } ) && stationaryClosest->segmentIndex == 0 && stationaryClosest->t == 1.0f, "prepared closest point allows a zero tangent without requesting a direction" );
    const auto seamClosest = closedCurvature->closestPoint( curve[0], 1.0e-6 );
    check( seamClosest && equal( seamClosest->point, curve[0] ) && seamClosest->segmentIndex == 0 && seamClosest->t == 0.0f && seamClosest->distanceSquared == 0.0, "closed closest point preserves deterministic seam selection" );
    const vec2 crossingCurve[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    compare( { crossingCurve } );
    check( equal( prepareSpline( { crossingCurve } )->closestPoint( {}, 1.0e-6 ), splineClosestPoint2{ {}, 0, 1.0f, 0.0 } ), "crossing branches preserve the first equal-distance candidate" );
    check( equal( closedCurvature->closestPoint( 4, curve[0], 1.0e-6 ), splineClosestPoint2{ curve[0], 4, 1.0f, 0.0 } ), "closed segment closest point preserves its closing endpoint representation" );
    check( equal( cuspSnapshot->closestPoint( { 1, 0.0f }, { 0, 1.0f }, { 1, 0 }, 1.0e-6 ), splineClosestPoint2{ { 1, 0 }, 1, 0.0f, 0.0 } ), "equal-position range closest point allows a zero tangent" );
    const auto crossingSnapshot = prepareSpline( { crossingCurve } );
    check( equal( crossingSnapshot->closestPoint( 4, {}, 1.0e-6 ), splineClosestPoint2{ {}, 4, 0.0f, 0.0 } ) && equal( crossingSnapshot->closestPoint( { 3, 0.5f }, { 4, 0.5f }, {}, 1.0e-6 ), splineClosestPoint2{ {}, 3, 1.0f, 0.0 } ), "crossing branch restrictions preserve candidate selection within the requested range" );
    const vec2 coarseDiagonal[] = { { 1.0e8f, 1.0e8f }, { 1.0e8f + 8.0f, 1.0e8f + 8.0f } };
    check( !prepareSpline( { coarseDiagonal } )->closestPoint( { 1.0e8f + 8.0f, 1.0e8f }, 1.0e-3 ), "prepared closest point rejects an unrepresentable sufficiently close position" );
    const auto coarseSnapshot = prepareSpline( { coarseDiagonal } );
    check( !coarseSnapshot->closestPoint( 0, { 1.0e8f + 8.0f, 1.0e8f }, 1.0e-3 ) && !coarseSnapshot->closestPoint( { 0, 0.0f }, { 0, 1.0f }, { 1.0e8f + 8.0f, 1.0e8f }, 1.0e-3 ), "restricted closest queries preserve position precision failure" );
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
    std::optional<double> expectedCurvature;
    std::optional<splineClosestPoint2> expectedClosest, expectedSegmentClosest, expectedRangeClosest;
    std::optional<aabb2> expectedBounds, expectedSegmentBounds, expectedRangeBounds;
    std::optional<splineParameter2> expectedParameter, expectedRangeParameter;
    std::optional<splineSample2> expectedSample, expectedRangeSample;
    std::vector<splineParameter2> expectedParameters, expectedAdaptiveParameters, expectedFixedParameters;
    std::vector<vec2> expectedPoints, expectedRangePoints, expectedAdaptivePoints, expectedAdaptiveRangePoints, expectedFixedPoints;
    std::optional<double> expectedLength, expectedDistance, expectedPartialLength, expectedRangeLength;
    {
        std::vector<vec2> source{ std::begin( curve ), std::end( curve ) };
        owned = prepareSpline( { source, true } );
        expectedPoints = SampleByDistance( { source, true }, 3, 1.0e-4 );
        expectedRangePoints = SampleByDistance( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 );
        expectedFixedParameters = owned->sampleParameters( 7 );
        expectedFixedPoints = Sample( { source, true }, 7 );
        expectedAdaptivePoints = SampleAdaptive( { source, true }, 0.01f );
        expectedAdaptiveRangePoints = SampleAdaptive( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 0.01f );
        expectedAdaptiveParameters = SampleAdaptiveParameters( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 0.01f );
        expectedParameters = SampleByDistanceParameters( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3, 1.0e-4 );
        expectedSample = EvaluateByDistance( { source, true }, 3.5, 1.0e-4 );
        expectedRangeSample = EvaluateByDistance( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 );
        expectedRangeParameter = GetParameterByDistance( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 3.5, 1.0e-4 );
        expectedParameter = GetParameterByDistance( { source, true }, 3.5, 1.0e-4 );
        expectedLength = GetLength( { source, true }, 1.0e-6 );
        expectedRangeLength = GetLength( { source, true }, { 1, 0.25f }, { 4, 0.75f }, 1.0e-6 );
        expectedPartialLength = GetLength( { source, true }, 4, 0.25f, 0.75f, 1.0e-6 );
        expectedDistance = GetDistance( { source, true }, 4, 0.37f, 1.0e-6 );
        expectedSegmentClosest = ClosestPoint( { source, true }, 4, { 3.5f, 0.2f }, 1.0e-4 );
        expectedRangeClosest = ClosestPoint( { source, true }, { 1, 0.25f }, { 4, 0.75f }, { 3.5f, 0.2f }, 1.0e-4 );
        expectedClosest = ClosestPoint( { source, true }, { 3.5f, 0.2f }, 1.0e-4 );
        expectedRangeBounds = ComputeAABB( { source, true }, { 1, 0.25f }, { 4, 0.75f } );
        expectedBounds = ComputeAABB( { source, true } );
        expectedSegmentBounds = ComputeAABB( { source, true }, 4 );
        expectedCurvature = Curvature( { source, true }, 4, 0.37f );
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
    check( expectedCurvature && owned->curvature( 4, 0.37f ) == expectedCurvature, "curvature survives source mutation and destruction" );
    check( expectedBounds && expectedSegmentBounds && equal( owned->computeAABB(), expectedBounds ) && equal( owned->computeAABB( 4 ), expectedSegmentBounds ), "prepared bounds survive source mutation and destruction" );
    check( expectedRangeBounds && equal( owned->computeAABB( { 1, 0.25f }, { 4, 0.75f } ), expectedRangeBounds ), "range bounds survive source mutation and destruction" );
    check( expectedClosest && equal( owned->closestPoint( { 3.5f, 0.2f }, 1.0e-4 ), expectedClosest ), "closest point survives source mutation and destruction" );
    check( expectedSegmentClosest && expectedRangeClosest && equal( owned->closestPoint( 4, { 3.5f, 0.2f }, 1.0e-4 ), expectedSegmentClosest ) && equal( owned->closestPoint( { 1, 0.25f }, { 4, 0.75f }, { 3.5f, 0.2f }, 1.0e-4 ), expectedRangeClosest ), "restricted closest queries survive source mutation and destruction" );
    check( !expectedAdaptiveParameters.empty() && equal( owned->sampleAdaptiveParameters( { 1, 0.25f }, { 4, 0.75f }, 0.01f ), expectedAdaptiveParameters ), "adaptive parameters survive original source destruction" );
    check( !expectedAdaptivePoints.empty() && !expectedAdaptiveRangePoints.empty() && equal( owned->sampleAdaptive( 0.01f ), expectedAdaptivePoints ) && equal( owned->sampleAdaptive( { 1, 0.25f }, { 4, 0.75f }, 0.01f ), expectedAdaptiveRangePoints ), "adaptive positions survive source mutation reallocation and destruction" );
    check( !expectedFixedPoints.empty() && equal( owned->sample( 7 ), expectedFixedPoints ), "fixed samples survive source mutation reallocation and destruction" );
    check( expectedFixedParameters.size() == 36 && equal( owned->sampleParameters( 7 ), expectedFixedParameters ), "fixed parameters survive source mutation reallocation and destruction" );
    auto copied = *owned;
    auto moved = std::move( copied );
    owned.reset();
    check( equal( moved.sampleParameters( 7 ), expectedFixedParameters ), "fixed parameters survive snapshot copy move and original destruction" );
    check( equal( moved.sample( 7 ), expectedFixedPoints ), "fixed samples survive snapshot copy move and original destruction" );
    check( equal( moved.sampleAdaptive( 0.01f ), expectedAdaptivePoints ) && equal( moved.sampleAdaptive( { 1, 0.25f }, { 4, 0.75f }, 0.01f ), expectedAdaptiveRangePoints ), "adaptive positions survive snapshot copy move and original destruction" );
    check( equal( moved.sampleAdaptiveParameters( { 1, 0.25f }, { 4, 0.75f }, 0.01f ), expectedAdaptiveParameters ), "adaptive parameters survive snapshot copy move and original destruction" );
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
    check( moved.curvature( 4, 0.37f ) == expectedCurvature, "copied and moved snapshots preserve curvature" );
    check( equal( moved.computeAABB(), expectedBounds ) && equal( moved.computeAABB( 4 ), expectedSegmentBounds ), "copied and moved snapshots preserve conservative bounds" );
    check( equal( moved.computeAABB( { 1, 0.25f }, { 4, 0.75f } ), expectedRangeBounds ), "copied and moved snapshots preserve range bounds" );
    check( equal( moved.closestPoint( { 3.5f, 0.2f }, 1.0e-4 ), expectedClosest ), "copied and moved snapshots preserve closest points" );
    check( equal( moved.closestPoint( 4, { 3.5f, 0.2f }, 1.0e-4 ), expectedSegmentClosest ) && equal( moved.closestPoint( { 1, 0.25f }, { 4, 0.75f }, { 3.5f, 0.2f }, 1.0e-4 ), expectedRangeClosest ), "copied and moved snapshots preserve restricted closest points" );
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
    check( !empty.curvature( 0, 0.5f ), "empty geometry has no curvature" );
    for( float t : { -0.01f, 1.01f, nan, infinity, -infinity } )
        check( !moved.curvature( 0, t ), "prepared curvature rejects invalid parameters without asserting" );
    for( std::size_t i : { moved.getSegmentCount(), std::numeric_limits<std::size_t>::max() } )
        check( !moved.curvature( i, 0.5f ), "prepared curvature rejects invalid segment indices without asserting" );
    check( !empty.computeAABB() && !empty.computeAABB( 0 ), "empty geometry has no conservative bounds" );
    for( std::size_t i : { moved.getSegmentCount(), std::numeric_limits<std::size_t>::max() } )
        check( !moved.computeAABB( i ), "prepared bounds reject invalid segment indices without asserting" );
    const vec2 unboundedAllowance[] = { { 0, std::numeric_limits<float>::max() }, { 1, std::numeric_limits<float>::max() } };
    const auto unboundedSnapshot = prepareSpline( { unboundedAllowance } );
    check( unboundedSnapshot && !unboundedSnapshot->computeAABB() && !unboundedSnapshot->computeAABB( 0 ) && !ComputeAABB( { unboundedAllowance } ), "prepared bounds preserve failure when outward allowance cannot remain finite" );
    const vec2 translatedControls[] = { { 1.0e8f, -1.0e8f }, { 1.0e8f + 8.0f, -1.0e8f }, { 1.0e8f + 8.0f, -1.0e8f + 8.0f }, { 1.0e8f + 16.0f, -1.0e8f + 8.0f } };
    const vec2 subnormalAxis[] = { { 0, std::numeric_limits<float>::denorm_min() }, { 1, 0 } };
    for( const auto candidate : { catmullRomSpline2{ translatedControls }, catmullRomSpline2{ subnormalAxis } } )
    {
        const auto prepared = prepareSpline( candidate );
        const auto bounds = prepared->computeAABB();
        check( bounds && equal( bounds, ComputeAABB( candidate ) ), "prepared bounds preserve large-translation and subnormal rounding" );
        for( std::size_t i = 0; i < prepared->getSegmentCount(); ++i )
            for( int step = 0; step <= 1024; ++step )
                check( Contains( *bounds, prepared->evaluate( i, step / 1024.0f ) ), "prepared bounds contain translated and subnormal float samples" );
    }
    check( !empty.computeAABB( { 0, 0.0f }, { 0, 0.0f } ), "empty geometry cannot return equal-position range bounds" );
    for( splineParameter2 invalid : { splineParameter2{ moved.getSegmentCount(), 0.0f }, { std::numeric_limits<std::size_t>::max(), 0.0f }, { 0, -0.01f }, { 0, 1.01f }, { 0, nan }, { 0, infinity }, { 0, -infinity } } )
        check( !moved.computeAABB( invalid, { 4, 1.0f } ) && !moved.computeAABB( { 0, 0.0f }, invalid ) && !moved.computeAABB( invalid, invalid ), "range bounds validate both inputs before normalization or equality" );
    check( !moved.computeAABB( { 4, 1.0f }, { 0, 0.0f } ) && !moved.computeAABB( { 1, 0.75f }, { 1, 0.25f } ), "prepared range bounds reject reverse traversal without closed wrapping" );
    check( !unboundedSnapshot->computeAABB( { 0, 0.25f }, { 0, 0.75f } ) && !unboundedSnapshot->computeAABB( { 0, 0.5f }, { 0, 0.5f } ), "range bounds preserve finite-boundary failure including equal-position ranges" );
    for( const auto candidate : { catmullRomSpline2{ translatedControls }, catmullRomSpline2{ subnormalAxis } } )
    {
        const auto prepared = prepareSpline( candidate );
        const splineParameter2 from{ 0, 0.25f }, to{ prepared->getSegmentCount() - 1, 0.75f };
        const auto bounds = prepared->computeAABB( from, to );
        check( bounds && equal( bounds, ComputeAABB( candidate, from, to ) ), "range bounds preserve translated and subnormal rounding" );
        for( std::size_t segment = from.segmentIndex; segment <= to.segmentIndex; ++segment )
        {
            const float begin = segment == from.segmentIndex ? from.t : 0.0f, end = segment == to.segmentIndex ? to.t : 1.0f;
            for( int step = 0; step <= 256; ++step )
                check( Contains( *bounds, prepared->evaluate( segment, begin + ( end - begin ) * ( step / 256.0f ) ) ), "range bounds contain translated and subnormal float evaluations" );
        }
    }
    check( !empty.closestPoint( {}, 1.0e-4 ), "empty prepared geometry has no closest point" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ), 1.0e-30 } )
        check( !moved.closestPoint( {}, tolerance ), "prepared closest point rejects invalid or unattainable tolerance" );
    for( const vec2 query : { vec2{ nan, 0 }, { 0, nan }, { infinity, 0 }, { 0, -infinity } } )
        check( !moved.closestPoint( query, 1.0e-4 ), "prepared closest point rejects nonfinite queries without asserting" );
    std::vector<vec2> nodeLimitPoints;
    for( int i = 0; i <= 4096; ++i )
        nodeLimitPoints.push_back( { static_cast<float>( i ), 0 } );
    check( prepareSpline( { nodeLimitPoints } )->closestPoint( {}, 1.0e-4 ).has_value(), "prepared closest point accepts the existing maximum interval count" );
    nodeLimitPoints.push_back( { 4097, 0 } );
    check( !prepareSpline( { nodeLimitPoints } )->closestPoint( {}, 1.0e-4 ) && !ClosestPoint( { nodeLimitPoints }, {}, 1.0e-4 ), "prepared closest point preserves the interval limit failure policy" );
    check( !empty.closestPoint( 0, {}, 1.0e-4 ) && !empty.closestPoint( { 0, 0.0f }, { 0, 0.0f }, {}, 1.0e-4 ), "empty geometry has no restricted closest point" );
    for( std::size_t i : { moved.getSegmentCount(), std::numeric_limits<std::size_t>::max() } )
        check( !moved.closestPoint( i, {}, 1.0e-4 ), "segment closest point rejects invalid indices without asserting" );
    for( splineParameter2 invalid : { splineParameter2{ moved.getSegmentCount(), 0.0f }, { std::numeric_limits<std::size_t>::max(), 0.0f }, { 0, -0.01f }, { 0, 1.01f }, { 0, nan }, { 0, infinity }, { 0, -infinity } } )
        check( !moved.closestPoint( invalid, { 4, 1.0f }, {}, 1.0e-4 ) && !moved.closestPoint( { 0, 0.0f }, invalid, {}, 1.0e-4 ) && !moved.closestPoint( invalid, invalid, {}, 1.0e-4 ), "closest point validates both range endpoints before normalization or equality" );
    for( double tolerance : { 0.0, -1.0, static_cast<double>( nan ), static_cast<double>( infinity ), -static_cast<double>( infinity ), 1.0e-30 } )
        check( !moved.closestPoint( 1, {}, tolerance ) && !moved.closestPoint( { 1, 0.25f }, { 4, 0.75f }, {}, tolerance ), "restricted closest queries reject invalid or unattainable tolerance" );
    for( const vec2 query : { vec2{ nan, 0 }, { 0, nan }, { infinity, 0 }, { 0, -infinity } } )
        check( !moved.closestPoint( 1, query, 1.0e-4 ) && !moved.closestPoint( { 1, 0.25f }, { 4, 0.75f }, query, 1.0e-4 ), "restricted closest queries reject nonfinite points" );
    check( !moved.closestPoint( { 4, 1.0f }, { 0, 0.0f }, {}, 1.0e-4 ) && !moved.closestPoint( { 1, 0.75f }, { 1, 0.25f }, {}, 1.0e-4 ), "range closest point rejects reverse traversal without implicit closed wrapping" );
    const auto manySegments = prepareSpline( { nodeLimitPoints } );
    check( equal( manySegments->closestPoint( 4096, { 4096.5f, 1 }, 1.0e-4 ), splineClosestPoint2{ { 4096.5f, 0 }, 4096, 0.5f, 1.0 } ) && manySegments->closestPoint( { 4095, 0.0f }, { 4096, 1.0f }, { 4096, 1 }, 1.0e-4 ).has_value(), "restricted queries count requested intervals instead of the entire path for search limits" );
    check( !manySegments->closestPoint( { 0, 0.0f }, { 4096, 1.0f }, {}, 1.0e-4 ), "range closest point preserves interval-count overflow failure" );
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
