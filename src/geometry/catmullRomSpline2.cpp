#include "geometry/catmullRomSpline2.h"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <array>
#include <limits>
#include <queue>

namespace zonai
{

namespace
{

struct bezierPoint2
{
    double x;
    double y;
};

vec2 GetPoint( const catmullRomSpline2& spline, std::size_t index )
{
    return spline.points[index % spline.points.size()];
}

void GetTangents( const catmullRomSpline2& spline, std::size_t segmentIndex, vec2& tangent1, vec2& tangent2 )
{
    const vec2 delta = GetPoint( spline, segmentIndex + 1 ) - spline.points[segmentIndex];
    const float interval = std::sqrt( Length( delta ) );

    // 열린 경로 양 끝의 가상 제어점은 인접 edge를 연장하므로 끝 접선이 delta와 같음.
    tangent1 = delta;
    tangent2 = delta;

    if( spline.isLoop || segmentIndex > 0 )
    {
        const vec2 previous = spline.points[segmentIndex] - spline.points[segmentIndex == 0 ? spline.points.size() - 1 : segmentIndex - 1];
        const float previousInterval = std::sqrt( Length( previous ) );

        tangent1 = ( previous * ( interval / previousInterval ) * interval + delta * previousInterval ) / ( previousInterval + interval );
    }

    if( spline.isLoop || segmentIndex + 2 < spline.points.size() )
    {
        const vec2 next = GetPoint( spline, segmentIndex + 2 ) - GetPoint( spline, segmentIndex + 1 );
        const float nextInterval = std::sqrt( Length( next ) );

        tangent2 = ( delta * nextInterval + next * ( interval / nextInterval ) * interval ) / ( interval + nextInterval );
    }
}

vec2 EvaluateSegment( const catmullRomSpline2& spline, std::size_t segmentIndex, float t )
{
    if( t == 0.0f ) return spline.points[segmentIndex];
    if( t == 1.0f ) return GetPoint( spline, segmentIndex + 1 );

    vec2 tangent1{};
    vec2 tangent2{};
    GetTangents( spline, segmentIndex, tangent1, tangent2 );

    const vec2 delta = GetPoint( spline, segmentIndex + 1 ) - spline.points[segmentIndex];
    const vec2 quadratic = 3.0f * delta - 2.0f * tangent1 - tangent2;
    const vec2 cubic = tangent1 + tangent2 - 2.0f * delta;

    return spline.points[segmentIndex] + t * ( tangent1 + t * ( quadratic + t * cubic ) );
}

std::optional<double> GetSegmentLength( const catmullRomSpline2& spline, std::size_t segmentIndex, double begin, double end, double tolerance )
{
    if( !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};
    if( begin == end ) return 0.0;
    vec2 tangent1{}, tangent2{};
    GetTangents( spline, segmentIndex, tangent1, tangent2 );
    if( !IsFinite( tangent1 ) || !IsFinite( tangent2 ) ) return {};
    const double dx = static_cast<double>( GetPoint( spline, segmentIndex + 1 ).x ) - spline.points[segmentIndex].x;
    const double dy = static_cast<double>( GetPoint( spline, segmentIndex + 1 ).y ) - spline.points[segmentIndex].y;
    const double qx = 3.0 * dx - 2.0 * tangent1.x - tangent2.x, qy = 3.0 * dy - 2.0 * tangent1.y - tangent2.y;
    const double cx = tangent1.x + static_cast<double>( tangent2.x ) - 2.0 * dx, cy = tangent1.y + static_cast<double>( tangent2.y ) - 2.0 * dy;
    // 미분 다항식을 double로 평가해 float t 반올림이 적분 수렴을 막지 않게 함.
    const auto speed = [&]( double t )
    {
        return std::hypot( tangent1.x + t * ( 2.0 * qx + t * 3.0 * cx ), tangent1.y + t * ( 2.0 * qy + t * 3.0 * cy ) );
    };
    const auto integrate = [&]( auto&& self, double a, double b, double fa, double fm, double fb, double estimate, double error, int depth ) -> std::optional<double>
    {
        if( error < 32.0 * std::numeric_limits<double>::epsilon() * estimate ) return {};
        const double middle = 0.5 * ( a + b );
        const double fl = speed( 0.5 * ( a + middle ) ), fr = speed( 0.5 * ( middle + b ) );
        const double left = ( middle - a ) * ( fa + 4.0 * fl + fm ) / 6.0;
        const double right = ( b - middle ) * ( fm + 4.0 * fr + fb ) / 6.0;
        const double correction = ( left + right - estimate ) / 15.0;
        if( std::abs( correction ) <= error ) return left + right + correction;
        constexpr int MAX_DEPTH = 20;
        if( depth == MAX_DEPTH ) return {};
        const auto l = self( self, a, middle, fa, fl, fm, left, error * 0.5, depth + 1 );
        if( !l ) return {};
        const auto r = self( self, middle, b, fm, fr, fb, right, error * 0.5, depth + 1 );
        if( !r ) return {};
        return *l + *r;
    };
    const double fa = speed( begin ), fm = speed( 0.5 * ( begin + end ) ), fb = speed( end );
    return integrate( integrate, begin, end, fa, fm, fb, ( end - begin ) * ( fa + 4.0 * fm + fb ) / 6.0, tolerance, 0 );
}

splineParameter2 NormalizeParameter( splineParameter2 parameter, std::size_t count )
{
    if( parameter.t == 1.0f && parameter.segmentIndex < count - 1 )
    {
        ++parameter.segmentIndex;
        parameter.t = 0.0f;
    }
    return parameter;
}

std::optional<splineParameter2> GetSegmentParameterByDistance( const catmullRomSpline2& spline, std::size_t segmentIndex, double begin, double end, double target, double error, double tolerance )
{
    double lower = begin, upper = end;
    constexpr int MAX_ITERATIONS = 64;
    for( int iteration = 0; iteration < MAX_ITERATIONS; ++iteration )
    {
        const float t = static_cast<float>( 0.5 * ( lower + upper ) );
        if( t <= lower || t >= upper ) return {};
        const auto partial = GetSegmentLength( spline, segmentIndex, begin, t, error );
        if( !partial ) return {};
        if( std::abs( *partial - target ) <= 0.5 * tolerance ) return splineParameter2{ segmentIndex, t };
        if( *partial < target ) lower = t;
        else upper = t;
    }
    return {};
}

double GetPositionError( vec2 point, vec2 a, vec2 b, vec2 tangent1, vec2 tangent2, float t )
{
    const double dx = static_cast<double>( b.x ) - a.x, dy = static_cast<double>( b.y ) - a.y;
    const double qx = 3.0 * dx - 2.0 * tangent1.x - tangent2.x, qy = 3.0 * dy - 2.0 * tangent1.y - tangent2.y;
    const double cx = tangent1.x + static_cast<double>( tangent2.x ) - 2.0 * dx, cy = tangent1.y + static_cast<double>( tangent2.y ) - 2.0 * dy;
    const double ex = ( static_cast<double>( point.x ) - a.x ) - t * ( tangent1.x + t * ( qx + t * cx ) );
    const double ey = ( static_cast<double>( point.y ) - a.y ) - t * ( tangent1.y + t * ( qy + t * cy ) );
    return std::hypot( ex, ey );
}

std::optional<vec2> GetPositionByParameter( const catmullRomSpline2& spline, std::size_t segment, float t, double tolerance )
{
    const vec2 point = Evaluate( spline, segment, t );
    if( !IsFinite( point ) ) return {};
    if( t > 0.0f && t < 1.0f )
    {
        vec2 tangent1{}, tangent2{};
        GetTangents( spline, segment, tangent1, tangent2 );
        const vec2 a = spline.points[segment], b = GetPoint( spline, segment + 1 );
        if( GetPositionError( point, a, b, tangent1, tangent2, t ) > 0.25 * tolerance ) return {};
    }
    return point;
}

std::optional<aabb2> GetSegmentAABB( const catmullRomSpline2& spline, std::size_t segmentIndex, double t0 = 0.0, double t1 = 1.0 )
{
    const auto bound = [&]( double start, double end, double tangent, double quadratic, double cubic, float& lower, float& upper )
    {
        std::array<double, 4> hull{ start, start + tangent / 3.0, start + ( 2.0 * tangent + quadratic ) / 3.0, start + tangent + quadratic + cubic };
        const auto trim = [&]( double t, bool keepLeft )
        {
            const auto mix = [&]( double a, double b ) { return ( 1.0 - t ) * a + t * b; };
            const double a = mix( hull[0], hull[1] ), b = mix( hull[1], hull[2] ), c = mix( hull[2], hull[3] );
            const double d = mix( a, b ), e = mix( b, c ), center = mix( d, e );
            hull = keepLeft ? std::array{ hull[0], a, d, center } : std::array{ center, e, c, hull[3] };
        };
        if( t1 < 1.0 ) trim( t1, true );
        if( t0 > 0.0 ) trim( t0 / t1, false );
        const double endpoint = t1 == 1.0 ? end : hull[3];
        // Horner 평가 6회 연산과 subnormal 반올림을 감싸고 double hull 구성 오차도 포함함.
        const double error = 8.0 * std::numeric_limits<float>::epsilon() * ( std::abs( start ) + std::abs( tangent ) + std::abs( quadratic ) + std::abs( cubic ) ) + 8.0 * std::numeric_limits<float>::denorm_min();
        const double minimum = std::min( *std::min_element( hull.begin(), hull.end() ), endpoint ) - error;
        const double maximum = std::max( *std::max_element( hull.begin(), hull.end() ), endpoint ) + error;
        if( !std::isfinite( minimum ) || !std::isfinite( maximum ) || minimum < -std::numeric_limits<float>::max() || maximum > std::numeric_limits<float>::max() ) return false;
        lower = std::nextafter( static_cast<float>( minimum ), -std::numeric_limits<float>::infinity() );
        upper = std::nextafter( static_cast<float>( maximum ), std::numeric_limits<float>::infinity() );
        return std::isfinite( lower ) && std::isfinite( upper );
    };

    vec2 tangent1{}, tangent2{};
    GetTangents( spline, segmentIndex, tangent1, tangent2 );
    const vec2 a = spline.points[segmentIndex], b = GetPoint( spline, segmentIndex + 1 );
    const vec2 delta = b - a;
    // Evaluate와 같은 float 계수를 Bézier로 바꾸며 별도 처리되는 끝점 b도 hull에 포함함.
    const vec2 quadratic = 3.0f * delta - 2.0f * tangent1 - tangent2;
    const vec2 cubic = tangent1 + tangent2 - 2.0f * delta;
    if( !IsFinite( tangent1 ) || !IsFinite( quadratic ) || !IsFinite( cubic ) ) return {};
    aabb2 segmentBounds{};
    if( !bound( a.x, b.x, tangent1.x, quadratic.x, cubic.x, segmentBounds.min.x, segmentBounds.max.x ) ||
        !bound( a.y, b.y, tangent1.y, quadratic.y, cubic.y, segmentBounds.min.y, segmentBounds.max.y ) ) return {};
    return segmentBounds;
}

} // namespace

std::optional<preparedCatmullRomSpline2> prepareSpline( const catmullRomSpline2& spline )
{
    if( !IsValid( spline ) ) return {};

    preparedCatmullRomSpline2 prepared{};
    prepared.isLoop_ = spline.isLoop;
    const std::size_t count = GetSegmentCount( spline );
    prepared.segments_.reserve( count );
    for( std::size_t i = 0; i < count; ++i )
    {
        preparedSplineSegment2 segment{};
        segment.start = spline.points[i];
        segment.end = GetPoint( spline, i + 1 );
        GetTangents( spline, i, segment.tangent1, segment.tangent2 );

        const vec2 delta = segment.end - segment.start;
        segment.quadratic = 3.0f * delta - 2.0f * segment.tangent1 - segment.tangent2;
        segment.cubic = segment.tangent1 + segment.tangent2 - 2.0f * delta;
        if( !IsFinite( segment.tangent1 ) || !IsFinite( segment.tangent2 ) || !IsFinite( segment.quadratic ) || !IsFinite( segment.cubic ) ) return {};

        prepared.segments_.push_back( segment );
    }
    return prepared;
}

vec2 preparedCatmullRomSpline2::evaluate( std::size_t segmentIndex, float t ) const
{
    const bool valid = segmentIndex < segments_.size() && std::isfinite( t ) && t >= 0.0f && t <= 1.0f;

    assert( valid );
    if( !valid ) return {};

    const auto& segment = segments_[segmentIndex];
    if( t == 0.0f ) return segment.start;
    if( t == 1.0f ) return segment.end;

    return segment.start + t * ( segment.tangent1 + t * ( segment.quadratic + t * segment.cubic ) );
}

vec2 preparedCatmullRomSpline2::derivative( std::size_t segmentIndex, float t ) const
{
    const bool valid = segmentIndex < segments_.size() && std::isfinite( t ) && t >= 0.0f && t <= 1.0f;

    assert( valid );
    if( !valid ) return {};

    const auto& segment = segments_[segmentIndex];
    if( t == 0.0f ) return segment.tangent1;
    if( t == 1.0f ) return segment.tangent2;

    return segment.tangent1 + t * ( 2.0f * segment.quadratic + 3.0f * t * segment.cubic );
}

vec2 preparedCatmullRomSpline2::secondDerivative( std::size_t segmentIndex, float t ) const
{
    const bool valid = segmentIndex < segments_.size() && std::isfinite( t ) && t >= 0.0f && t <= 1.0f;

    assert( valid );
    if( !valid ) return {};

    const auto& segment = segments_[segmentIndex];
    return 2.0f * segment.quadratic + 6.0f * t * segment.cubic;
}

std::optional<splineSample2> preparedCatmullRomSpline2::evaluateSample( std::size_t segmentIndex, float t, double tolerance ) const
{
    if( segmentIndex >= segments_.size() || !std::isfinite( t ) || t < 0.0f || t > 1.0f || !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};

    const vec2 point = evaluate( segmentIndex, t );
    if( !IsFinite( point ) ) return {};
    const auto& segment = segments_[segmentIndex];
    if( t > 0.0f && t < 1.0f && GetPositionError( point, segment.start, segment.end, segment.tangent1, segment.tangent2, t ) > 0.25 * tolerance ) return {};

    const vec2 first = derivative( segmentIndex, t );
    const double speed = std::hypot( static_cast<double>( first.x ), static_cast<double>( first.y ) );
    if( !std::isfinite( speed ) || speed <= 0.0 ) return {};
    return splineSample2{ point, { static_cast<float>( first.x / speed ), static_cast<float>( first.y / speed ) } };
}

bool IsValid( const catmullRomSpline2& spline )
{
    if( spline.points.size() < ( spline.isLoop ? 3u : 2u ) ) return false;

    for( std::size_t i = 0; i < spline.points.size(); ++i )
    {
        if( !IsFinite( spline.points[i] ) ) return false;

        if( i == 0 && !spline.isLoop ) continue;

        const float lengthSquared = LengthSquared( spline.points[i] - spline.points[i == 0 ? spline.points.size() - 1 : i - 1] );

        if( !std::isfinite( lengthSquared ) || lengthSquared <= 0.0f ) return false;
    }

    return true;
}

std::size_t GetSegmentCount( const catmullRomSpline2& spline )
{
    if( spline.points.size() < ( spline.isLoop ? 3u : 2u ) ) return 0;
    return spline.isLoop ? spline.points.size() : spline.points.size() - 1;
}

vec2 Evaluate( const catmullRomSpline2& spline, std::size_t segmentIndex, float t )
{
    const bool valid = IsValid( spline ) && segmentIndex < GetSegmentCount( spline ) && std::isfinite( t ) && t >= 0.0f && t <= 1.0f;

    assert( valid );
    if( !valid ) return {};

    return EvaluateSegment( spline, segmentIndex, t );
}

vec2 Derivative( const catmullRomSpline2& spline, std::size_t segmentIndex, float t )
{
    const bool valid = IsValid( spline ) && segmentIndex < GetSegmentCount( spline ) && std::isfinite( t ) && t >= 0.0f && t <= 1.0f;

    assert( valid );
    if( !valid ) return {};

    vec2 tangent1{};
    vec2 tangent2{};
    GetTangents( spline, segmentIndex, tangent1, tangent2 );

    if( t == 0.0f ) return tangent1;
    if( t == 1.0f ) return tangent2;

    const vec2 delta = GetPoint( spline, segmentIndex + 1 ) - spline.points[segmentIndex];
    const vec2 quadratic = 3.0f * delta - 2.0f * tangent1 - tangent2;
    const vec2 cubic = tangent1 + tangent2 - 2.0f * delta;

    return tangent1 + t * ( 2.0f * quadratic + 3.0f * t * cubic );
}

vec2 SecondDerivative( const catmullRomSpline2& spline, std::size_t segmentIndex, float t )
{
    const bool valid = IsValid( spline ) && segmentIndex < GetSegmentCount( spline ) && std::isfinite( t ) && t >= 0.0f && t <= 1.0f;

    assert( valid );
    if( !valid ) return {};

    vec2 tangent1{};
    vec2 tangent2{};
    GetTangents( spline, segmentIndex, tangent1, tangent2 );

    const vec2 delta = GetPoint( spline, segmentIndex + 1 ) - spline.points[segmentIndex];
    const vec2 quadratic = 3.0f * delta - 2.0f * tangent1 - tangent2;
    const vec2 cubic = tangent1 + tangent2 - 2.0f * delta;

    return 2.0f * quadratic + 6.0f * t * cubic;
}

std::optional<double> Curvature( const catmullRomSpline2& spline, std::size_t segmentIndex, float t )
{
    if( !IsValid( spline ) || segmentIndex >= GetSegmentCount( spline ) || !std::isfinite( t ) || t < 0.0f || t > 1.0f ) return {};

    const vec2 first = Derivative( spline, segmentIndex, t );
    const vec2 second = SecondDerivative( spline, segmentIndex, t );
    if( !IsFinite( first ) || !IsFinite( second ) ) return {};

    const double speed = std::hypot( static_cast<double>( first.x ), static_cast<double>( first.y ) );
    if( speed == 0.0 ) return {};

    const double cross = static_cast<double>( first.x ) * second.y - static_cast<double>( first.y ) * second.x;
    const double curvature = cross / ( speed * speed * speed );
    if( !std::isfinite( curvature ) ) return {};
    return curvature;
}

std::optional<aabb2> ComputeAABB( const catmullRomSpline2& spline )
{
    if( !IsValid( spline ) ) return {};
    aabb2 bounds{ spline.points[0], spline.points[0] };
    for( std::size_t i = 0; i < GetSegmentCount( spline ); ++i )
    {
        const auto segmentBounds = GetSegmentAABB( spline, i );
        if( !segmentBounds ) return {};
        bounds = Union( bounds, *segmentBounds );
    }
    return bounds;
}

std::optional<aabb2> ComputeAABB( const catmullRomSpline2& spline, std::size_t segmentIndex )
{
    if( !IsValid( spline ) || segmentIndex >= GetSegmentCount( spline ) ) return {};
    return GetSegmentAABB( spline, segmentIndex );
}

std::optional<aabb2> ComputeAABB( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to )
{
    if( !IsValid( spline ) ) return {};
    const std::size_t count = GetSegmentCount( spline );
    const auto valid = [&]( splineParameter2 parameter )
    {
        return parameter.segmentIndex < count && std::isfinite( parameter.t ) && parameter.t >= 0.0f && parameter.t <= 1.0f;
    };
    if( !valid( from ) || !valid( to ) ) return {};
    const auto beginParameter = NormalizeParameter( from, count ), endParameter = NormalizeParameter( to, count );
    if( beginParameter.segmentIndex > endParameter.segmentIndex || ( beginParameter.segmentIndex == endParameter.segmentIndex && beginParameter.t > endParameter.t ) ) return {};
    if( beginParameter.segmentIndex == endParameter.segmentIndex && beginParameter.t == endParameter.t )
        return GetSegmentAABB( spline, from.segmentIndex, from.t, from.t );
    const vec2 first = Evaluate( spline, from.segmentIndex, from.t );
    if( !IsFinite( first ) ) return {};
    aabb2 bounds{ first, first };
    for( std::size_t i = from.segmentIndex; i <= to.segmentIndex; ++i )
    {
        const double begin = i == from.segmentIndex ? from.t : 0.0, end = i == to.segmentIndex ? to.t : 1.0;
        if( begin == end ) continue;
        const auto segmentBounds = GetSegmentAABB( spline, i, begin, end );
        if( !segmentBounds ) return {};
        bounds = Union( bounds, *segmentBounds );
    }
    return bounds;
}

std::vector<vec2> Sample( const catmullRomSpline2& spline, std::size_t subdivisions )
{
    if( !IsValid( spline ) || subdivisions == 0 ) return {};

    const std::size_t segmentCount = GetSegmentCount( spline );
    std::vector<vec2> samples{};

    if( segmentCount > ( samples.max_size() - 1 ) / subdivisions ) return {};

    samples.reserve( segmentCount * subdivisions + 1 );
    samples.push_back( spline.points.front() );

    for( std::size_t i = 0; i < segmentCount; ++i )
        for( std::size_t j = 1; j <= subdivisions; ++j )
        {
            const float t = static_cast<float>( j ) / static_cast<float>( subdivisions );

            samples.push_back( Evaluate( spline, i, t ) );
        }

    return samples;
}

std::vector<vec2> SampleAdaptive( const catmullRomSpline2& spline, float maxError )
{
    const std::size_t count = GetSegmentCount( spline );
    if( count == 0 ) return {};
    return SampleAdaptive( spline, { 0, 0.0f }, { count - 1, 1.0f }, maxError );
}

namespace
{

std::vector<vec2> SampleAdaptiveInRange( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, float maxError, std::vector<splineParameter2>* parameters )
{
    if( !IsValid( spline ) || !std::isfinite( maxError ) || maxError <= 0.0f ) return {};
    const std::size_t count = GetSegmentCount( spline );
    const auto valid = [&]( splineParameter2 parameter )
    {
        return parameter.segmentIndex < count && std::isfinite( parameter.t ) && parameter.t >= 0.0f && parameter.t <= 1.0f;
    };
    if( !valid( from ) || !valid( to ) ) return {};
    const auto beginParameter = NormalizeParameter( from, count ), endParameter = NormalizeParameter( to, count );
    if( beginParameter.segmentIndex > endParameter.segmentIndex || ( beginParameter.segmentIndex == endParameter.segmentIndex && beginParameter.t > endParameter.t ) ) return {};
    const auto first = GetPositionByParameter( spline, from.segmentIndex, from.t, maxError );
    if( !first ) return {};
    std::vector<vec2> samples{ *first };
    if( parameters ) parameters->push_back( from );
    if( beginParameter.segmentIndex == endParameter.segmentIndex && beginParameter.t == endParameter.t ) return samples;
    double evaluationError = 0.0;
    const double errorSquared = static_cast<double>( maxError ) * maxError;
    const auto midpoint = []( bezierPoint2 a, bezierPoint2 b )
    {
        return bezierPoint2{ 0.5 * ( a.x + b.x ), 0.5 * ( a.y + b.y ) };
    };
    const auto subdivide = [&]( auto&& self, const std::array<bezierPoint2, 4>& curve, int depth, const vec2& end, std::size_t segment, float begin, float finish ) -> bool
    {
        if( !IsFinite( end ) ) return false;
        const vec2 start = samples.back();
        const double dx = static_cast<double>( end.x ) - start.x;
        const double dy = static_cast<double>( end.y ) - start.y;
        const double lengthSquared = dx * dx + dy * dy;
        const double allowedError = static_cast<double>( maxError ) - evaluationError;
        if( allowedError <= 0.0 ) return false;
        bool flat = true;
        // 실제 float 출력 chord에 hull 전체를 대조해 S자 구간과 끝점 반올림 오차도 포함함.
        for( const auto point : curve )
        {
            const double px = point.x - start.x, py = point.y - start.y;
            const double t = lengthSquared > 0.0 ? std::clamp( ( px * dx + py * dy ) / lengthSquared, 0.0, 1.0 ) : 0.0;
            const double ex = px - t * dx, ey = py - t * dy;
            flat &= ex * ex + ey * ey <= ( parameters ? allowedError * allowedError : errorSquared );
        }
        if( flat )
        {
            if( !parameters && start.x == end.x && start.y == end.y ) return true;
            if( samples.size() == samples.max_size() || ( parameters && parameters->size() == parameters->max_size() ) ) return false;
            samples.push_back( end );
            if( parameters ) parameters->push_back( { segment, finish } );
            return true;
        }
        constexpr int MAX_DEPTH = 12;
        if( depth == MAX_DEPTH ) return false;
        const float middle = static_cast<float>( 0.5 * ( static_cast<double>( begin ) + finish ) );
        if( parameters && ( middle <= begin || middle >= finish ) ) return false;
        const double fraction = parameters ? ( static_cast<double>( middle ) - begin ) / ( static_cast<double>( finish ) - begin ) : 0.5;
        const auto mix = [&]( bezierPoint2 a, bezierPoint2 b )
        {
            if( fraction == 0.5 ) return midpoint( a, b );
            return bezierPoint2{ ( 1.0 - fraction ) * a.x + fraction * b.x, ( 1.0 - fraction ) * a.y + fraction * b.y };
        };
        const auto a = mix( curve[0], curve[1] ), b = mix( curve[1], curve[2] ), c = mix( curve[2], curve[3] );
        const auto d = mix( a, b ), e = mix( b, c ), center = mix( d, e );
        const vec2 point = parameters ? EvaluateSegment( spline, segment, middle ) : vec2{ static_cast<float>( center.x ), static_cast<float>( center.y ) };
        return self( self, { curve[0], a, d, center }, depth + 1, point, segment, begin, middle ) &&
            self( self, { center, e, c, curve[3] }, depth + 1, end, segment, middle, finish );
    };
    const auto trim = []( std::array<bezierPoint2, 4>& curve, double t, bool keepLeft )
    {
        const auto mix = [&]( bezierPoint2 a, bezierPoint2 b )
        {
            return bezierPoint2{ ( 1.0 - t ) * a.x + t * b.x, ( 1.0 - t ) * a.y + t * b.y };
        };
        const auto a = mix( curve[0], curve[1] ), b = mix( curve[1], curve[2] ), c = mix( curve[2], curve[3] );
        const auto d = mix( a, b ), e = mix( b, c ), center = mix( d, e );
        curve = keepLeft ? std::array{ curve[0], a, d, center } : std::array{ center, e, c, curve[3] };
    };
    for( std::size_t i = from.segmentIndex; i <= to.segmentIndex; ++i )
    {
        const double begin = i == from.segmentIndex ? from.t : 0.0, end = i == to.segmentIndex ? to.t : 1.0;
        if( begin == end ) continue;
        const auto endpoint = end == 1.0 ? std::optional<vec2>{ GetPoint( spline, i + 1 ) } : GetPositionByParameter( spline, i, static_cast<float>( end ), maxError );
        if( !endpoint ) return {};
        vec2 tangent1{}, tangent2{};
        GetTangents( spline, i, tangent1, tangent2 );
        const vec2 a = spline.points[i], b = GetPoint( spline, i + 1 );
        std::array<bezierPoint2, 4> curve{
            bezierPoint2{ a.x, a.y },
            bezierPoint2{ a.x + static_cast<double>( tangent1.x ) / 3.0, a.y + static_cast<double>( tangent1.y ) / 3.0 },
            bezierPoint2{ b.x - static_cast<double>( tangent2.x ) / 3.0, b.y - static_cast<double>( tangent2.y ) / 3.0 },
            bezierPoint2{ b.x, b.y }
        };
        if( parameters )
        {
            const vec2 delta = b - a;
            const vec2 quadratic = 3.0f * delta - 2.0f * tangent1 - tangent2;
            const vec2 cubic = tangent1 + tangent2 - 2.0f * delta;
            if( !IsFinite( tangent1 ) || !IsFinite( quadratic ) || !IsFinite( cubic ) ) return {};
            curve = { bezierPoint2{ a.x, a.y },
                bezierPoint2{ a.x + static_cast<double>( tangent1.x ) / 3.0, a.y + static_cast<double>( tangent1.y ) / 3.0 },
                bezierPoint2{ a.x + ( 2.0 * tangent1.x + quadratic.x ) / 3.0, a.y + ( 2.0 * tangent1.y + quadratic.y ) / 3.0 },
                bezierPoint2{ a.x + static_cast<double>( tangent1.x ) + quadratic.x + cubic.x, a.y + static_cast<double>( tangent1.y ) + quadratic.y + cubic.y } };
            // Evaluate의 float 계수와 반올림 여유로 실제 평가점의 오차도 포함함.
            const double x = std::abs( static_cast<double>( a.x ) ) + std::abs( static_cast<double>( tangent1.x ) ) + std::abs( static_cast<double>( quadratic.x ) ) + std::abs( static_cast<double>( cubic.x ) );
            const double y = std::abs( static_cast<double>( a.y ) ) + std::abs( static_cast<double>( tangent1.y ) ) + std::abs( static_cast<double>( quadratic.y ) ) + std::abs( static_cast<double>( cubic.y ) );
            evaluationError = 8.0 * std::numeric_limits<float>::epsilon() * std::hypot( x, y ) + 16.0 * std::numeric_limits<float>::denorm_min();
        }
        if( end < 1.0 ) trim( curve, end, true );
        if( begin > 0.0 ) trim( curve, begin / end, false );
        if( !subdivide( subdivide, curve, 0, *endpoint, i, static_cast<float>( begin ), static_cast<float>( end ) ) ) return {};
    }
    if( parameters ) parameters->back() = to;
    return samples;
}

} // namespace

std::vector<vec2> SampleAdaptive( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, float maxError )
{
    return SampleAdaptiveInRange( spline, from, to, maxError, nullptr );
}

std::vector<splineParameter2> SampleAdaptiveParameters( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, float maxError )
{
    std::vector<splineParameter2> parameters;
    if( SampleAdaptiveInRange( spline, from, to, maxError, &parameters ).empty() ) return {};
    return parameters;
}

std::optional<double> GetLength( const catmullRomSpline2& spline, double tolerance )
{
    if( !IsValid( spline ) || !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};
    const std::size_t count = GetSegmentCount( spline );
    double length = 0.0;
    for( std::size_t i = 0; i < count; ++i )
    {
        const auto segmentLength = GetSegmentLength( spline, i, 0.0, 1.0, tolerance / ( 4.0 * count ) );
        if( !segmentLength || *segmentLength <= 0.0 ) return {};
        length += *segmentLength;
    }
    if( !std::isfinite( length ) || tolerance < 32.0 * std::numeric_limits<double>::epsilon() * length ) return {};
    return length;
}

std::optional<double> GetLength( const catmullRomSpline2& spline, std::size_t segmentIndex, float t0, float t1, double tolerance )
{
    if( !IsValid( spline ) || segmentIndex >= GetSegmentCount( spline ) || !std::isfinite( t0 ) || !std::isfinite( t1 ) || t0 < 0.0f || t1 > 1.0f || t0 > t1 || !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};
    if( t0 == t1 ) return 0.0;
    const auto length = GetSegmentLength( spline, segmentIndex, t0, t1, tolerance / 4.0 );
    if( !length || !std::isfinite( *length ) || *length <= 0.0 || tolerance < 32.0 * std::numeric_limits<double>::epsilon() * *length ) return {};
    return length;
}

std::optional<double> GetLength( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, double tolerance )
{
    if( !IsValid( spline ) || !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};
    const std::size_t count = GetSegmentCount( spline );
    const auto valid = [&]( splineParameter2 parameter )
    {
        return parameter.segmentIndex < count && std::isfinite( parameter.t ) && parameter.t >= 0.0f && parameter.t <= 1.0f;
    };
    if( !valid( from ) || !valid( to ) ) return {};
    from = NormalizeParameter( from, count );
    to = NormalizeParameter( to, count );
    if( from.segmentIndex > to.segmentIndex || ( from.segmentIndex == to.segmentIndex && from.t > to.t ) ) return {};
    if( from.segmentIndex == to.segmentIndex && from.t == to.t ) return 0.0;

    const std::size_t intervals = to.segmentIndex - from.segmentIndex + ( to.t > 0.0f ? 1u : 0u );
    const double error = tolerance / ( 4.0 * intervals );
    double length = 0.0;
    for( std::size_t i = from.segmentIndex; i <= to.segmentIndex; ++i )
    {
        const double begin = i == from.segmentIndex ? from.t : 0.0;
        const double end = i == to.segmentIndex ? to.t : 1.0;
        if( begin == end ) continue;
        const auto segmentLength = GetSegmentLength( spline, i, begin, end, error );
        if( !segmentLength || *segmentLength <= 0.0 ) return {};
        length += *segmentLength;
    }
    if( !std::isfinite( length ) || tolerance < 32.0 * std::numeric_limits<double>::epsilon() * length ) return {};
    return length;
}

std::optional<double> GetDistance( const catmullRomSpline2& spline, std::size_t segmentIndex, float t, double tolerance )
{
    if( !IsValid( spline ) || segmentIndex >= GetSegmentCount( spline ) || !std::isfinite( t ) || t < 0.0f || t > 1.0f || !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};

    const std::size_t count = GetSegmentCount( spline );
    double distance = 0.0;
    for( std::size_t i = 0; i <= segmentIndex; ++i )
    {
        const double end = i == segmentIndex ? t : 1.0;
        if( end == 0.0 ) break;
        const auto segmentLength = GetSegmentLength( spline, i, 0.0, end, tolerance / ( 4.0 * count ) );
        if( !segmentLength || *segmentLength <= 0.0 ) return {};
        distance += *segmentLength;
    }
    if( !std::isfinite( distance ) || tolerance < 32.0 * std::numeric_limits<double>::epsilon() * distance ) return {};
    return distance;
}

std::optional<splineParameter2> GetParameterByDistance( const catmullRomSpline2& spline, double distance, double tolerance )
{
    if( !std::isfinite( distance ) || distance < 0.0 ) return {};
    const auto length = GetLength( spline, tolerance );
    if( !length || distance > *length ) return {};
    const std::size_t count = GetSegmentCount( spline );
    if( distance == 0.0 ) return splineParameter2{ 0, 0.0f };
    if( distance == *length ) return splineParameter2{ count - 1, 1.0f };
    const double error = tolerance / ( 4.0 * count );
    double prefix = 0.0;
    for( std::size_t i = 0; i < count; ++i )
    {
        const auto segmentLength = GetSegmentLength( spline, i, 0.0, 1.0, error );
        if( !segmentLength ) return {};
        const double end = prefix + *segmentLength;
        if( distance > end )
        {
            prefix = end;
            continue;
        }
        if( distance == end ) return splineParameter2{ i, 1.0f };
        const double target = distance - prefix;
        return GetSegmentParameterByDistance( spline, i, 0.0, 1.0, target, error, tolerance );
    }
    return {};
}

std::optional<splineParameter2> GetParameterByDistance( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, double distance, double tolerance )
{
    if( !std::isfinite( distance ) || distance < 0.0 ) return {};
    const auto length = GetLength( spline, from, to, tolerance );
    if( !length || distance > *length ) return {};
    if( distance == 0.0 ) return from;
    if( distance == *length ) return to;
    const std::size_t count = GetSegmentCount( spline );
    from = NormalizeParameter( from, count );
    to = NormalizeParameter( to, count );
    const std::size_t intervals = to.segmentIndex - from.segmentIndex + ( to.t > 0.0f ? 1u : 0u );
    const double error = tolerance / ( 4.0 * intervals );
    double prefix = 0.0;
    for( std::size_t i = from.segmentIndex; i <= to.segmentIndex; ++i )
    {
        const double begin = i == from.segmentIndex ? from.t : 0.0;
        const double end = i == to.segmentIndex ? to.t : 1.0;
        if( begin == end ) continue;
        const auto segmentLength = GetSegmentLength( spline, i, begin, end, error );
        if( !segmentLength ) return {};
        const double next = prefix + *segmentLength;
        if( distance > next )
        {
            prefix = next;
            continue;
        }
        if( distance == next ) return splineParameter2{ i, static_cast<float>( end ) };
        return GetSegmentParameterByDistance( spline, i, begin, end, distance - prefix, error, tolerance );
    }
    return {};
}

std::vector<vec2> SampleByDistance( const catmullRomSpline2& spline, std::size_t subdivisions, double tolerance )
{
    const std::size_t count = GetSegmentCount( spline );
    if( count == 0 ) return {};
    return SampleByDistance( spline, { 0, 0.0f }, { count - 1, 1.0f }, subdivisions, tolerance );
}

std::vector<vec2> SampleByDistance( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, std::size_t subdivisions, double tolerance )
{
    std::vector<vec2> samples{};
    if( subdivisions == 0 || subdivisions >= samples.max_size() ) return {};
    const auto parameters = SampleByDistanceParameters( spline, from, to, subdivisions, tolerance );
    if( parameters.empty() ) return {};
    samples.reserve( parameters.size() );
    for( const auto parameter : parameters )
    {
        const auto point = GetPositionByParameter( spline, parameter.segmentIndex, parameter.t, tolerance );
        if( !point ) return {};
        samples.push_back( *point );
    }
    return samples;
}

std::vector<splineParameter2> SampleByDistanceParameters( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, std::size_t subdivisions, double tolerance )
{
    std::vector<splineParameter2> parameters{};
    if( subdivisions == 0 || subdivisions >= parameters.max_size() ) return {};
    const auto length = GetLength( spline, from, to, tolerance );
    if( !length ) return {};
    parameters.reserve( subdivisions + 1 );
    for( std::size_t i = 0; i <= subdivisions; ++i )
    {
        const double distance = i == subdivisions ? *length : *length * ( static_cast<double>( i ) / subdivisions );
        const auto parameter = GetParameterByDistance( spline, from, to, distance, tolerance );
        if( !parameter ) return {};
        parameters.push_back( *parameter );
    }
    return parameters;
}

std::optional<splineSample2> EvaluateByDistance( const catmullRomSpline2& spline, double distance, double tolerance )
{
    const std::size_t count = GetSegmentCount( spline );
    if( count == 0 ) return {};
    return EvaluateByDistance( spline, { 0, 0.0f }, { count - 1, 1.0f }, distance, tolerance );
}

std::optional<splineSample2> EvaluateByDistance( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, double distance, double tolerance )
{
    const auto parameter = GetParameterByDistance( spline, from, to, distance, tolerance );
    if( !parameter ) return {};
    const auto sample = [&]( std::size_t segment, float t ) -> std::optional<splineSample2>
    {
        const auto point = GetPositionByParameter( spline, segment, t, tolerance );
        if( !point ) return {};
        const vec2 derivative = Derivative( spline, segment, t );
        const double speed = std::hypot( static_cast<double>( derivative.x ), static_cast<double>( derivative.y ) );
        if( !std::isfinite( speed ) || speed <= 0.0 ) return {};
        return splineSample2{ *point, { static_cast<float>( derivative.x / speed ), static_cast<float>( derivative.y / speed ) } };
    };
    return sample( parameter->segmentIndex, parameter->t );
}

namespace
{

std::optional<splineClosestPoint2> ClosestPointInRange( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, const vec2& point, double tolerance )
{
    if( !IsValid( spline ) || !IsFinite( point ) || !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};
    const std::size_t count = GetSegmentCount( spline );
    const auto valid = [&]( splineParameter2 parameter )
    {
        return parameter.segmentIndex < count && std::isfinite( parameter.t ) && parameter.t >= 0.0f && parameter.t <= 1.0f;
    };
    if( !valid( from ) || !valid( to ) ) return {};
    const auto beginParameter = NormalizeParameter( from, count ), endParameter = NormalizeParameter( to, count );
    if( beginParameter.segmentIndex > endParameter.segmentIndex || ( beginParameter.segmentIndex == endParameter.segmentIndex && beginParameter.t > endParameter.t ) ) return {};
    constexpr std::size_t MAX_NODES = 4096;
    const std::size_t intervals = endParameter.segmentIndex - beginParameter.segmentIndex + ( endParameter.t > 0.0f ? 1u : 0u );
    if( intervals > MAX_NODES ) return {};
    double scale = std::max( std::abs( static_cast<double>( point.x ) ), std::abs( static_cast<double>( point.y ) ) );
    for( const vec2 control : spline.points ) scale = std::max( { scale, std::abs( static_cast<double>( control.x ) ), std::abs( static_cast<double>( control.y ) ) } );
    const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() * scale;
    if( tolerance < 8.0 * roundoff ) return {};
    struct node
    {
        std::array<bezierPoint2, 4> curve;
        std::size_t segment;
        double a, b, lower;
        int depth;
    };
    const auto farther = []( const node& a, const node& b )
    {
        if( a.lower != b.lower ) return a.lower > b.lower;
        if( a.segment != b.segment ) return a.segment > b.segment;
        return a.a > b.a;
    };
    std::priority_queue<node, std::vector<node>, decltype( farther )> pending{ farther };
    const auto lowerBound = [&]( const std::array<bezierPoint2, 4>& curve, int depth )
    {
        double minX = curve[0].x, maxX = minX, minY = curve[0].y, maxY = minY;
        for( const auto control : curve )
        {
            minX = std::min( minX, control.x ); maxX = std::max( maxX, control.x );
            minY = std::min( minY, control.y ); maxY = std::max( maxY, control.y );
        }
        const double dx = std::max( { minX - point.x, 0.0, point.x - maxX } );
        const double dy = std::max( { minY - point.y, 0.0, point.y - maxY } );
        // Hull 경계의 하한에 분할 반올림 여유를 남겨 조기 종료를 피함.
        return std::max( 0.0, std::hypot( dx, dy ) - roundoff * ( depth + 1 ) );
    };
    splineClosestPoint2 best{};
    double bestDistance = std::numeric_limits<double>::infinity();
    const auto consider = [&]( std::size_t segment, double parameter )
    {
        const float t = static_cast<float>( parameter );
        const vec2 candidate = Evaluate( spline, segment, t );
        if( !IsFinite( candidate ) ) return;
        if( t > 0.0f && t < 1.0f )
        {
            vec2 tangent1{}, tangent2{};
            GetTangents( spline, segment, tangent1, tangent2 );
            const vec2 a = spline.points[segment], b = GetPoint( spline, segment + 1 );
            const double dx = static_cast<double>( b.x ) - a.x, dy = static_cast<double>( b.y ) - a.y;
            const double qx = 3.0 * dx - 2.0 * tangent1.x - tangent2.x, qy = 3.0 * dy - 2.0 * tangent1.y - tangent2.y;
            const double cx = tangent1.x + static_cast<double>( tangent2.x ) - 2.0 * dx, cy = tangent1.y + static_cast<double>( tangent2.y ) - 2.0 * dy;
            const double ex = ( static_cast<double>( candidate.x ) - a.x ) - t * ( tangent1.x + t * ( qx + t * cx ) );
            const double ey = ( static_cast<double>( candidate.y ) - a.y ) - t * ( tangent1.y + t * ( qy + t * cy ) );
            if( std::hypot( ex, ey ) > 0.25 * tolerance ) return;
        }
        const double dx = static_cast<double>( candidate.x ) - point.x, dy = static_cast<double>( candidate.y ) - point.y;
        const double distance = std::hypot( dx, dy );
        if( distance < bestDistance || ( distance == bestDistance && ( segment < best.segmentIndex || ( segment == best.segmentIndex && t < best.t ) ) ) )
        {
            best = { candidate, segment, t, dx * dx + dy * dy };
            bestDistance = distance;
        }
    };
    if( beginParameter.segmentIndex == endParameter.segmentIndex && beginParameter.t == endParameter.t )
    {
        consider( from.segmentIndex, from.t );
        if( !std::isfinite( bestDistance ) ) return {};
        return best;
    }
    const auto trim = []( std::array<bezierPoint2, 4>& curve, double t, bool keepLeft )
    {
        const auto mix = [&]( bezierPoint2 a, bezierPoint2 b )
        {
            return bezierPoint2{ ( 1.0 - t ) * a.x + t * b.x, ( 1.0 - t ) * a.y + t * b.y };
        };
        const auto a = mix( curve[0], curve[1] ), b = mix( curve[1], curve[2] ), c = mix( curve[2], curve[3] );
        const auto d = mix( a, b ), e = mix( b, c ), center = mix( d, e );
        curve = keepLeft ? std::array{ curve[0], a, d, center } : std::array{ center, e, c, curve[3] };
    };
    for( std::size_t i = from.segmentIndex; i <= to.segmentIndex; ++i )
    {
        const double begin = i == from.segmentIndex ? from.t : 0.0, end = i == to.segmentIndex ? to.t : 1.0;
        consider( i, begin ); consider( i, end );
        if( begin == end ) continue;
        vec2 tangent1{}, tangent2{};
        GetTangents( spline, i, tangent1, tangent2 );
        if( !IsFinite( tangent1 ) || !IsFinite( tangent2 ) ) return {};
        const vec2 a = spline.points[i], b = GetPoint( spline, i + 1 );
        std::array<bezierPoint2, 4> curve{
            bezierPoint2{ a.x, a.y },
            bezierPoint2{ a.x + static_cast<double>( tangent1.x ) / 3.0, a.y + static_cast<double>( tangent1.y ) / 3.0 },
            bezierPoint2{ b.x - static_cast<double>( tangent2.x ) / 3.0, b.y - static_cast<double>( tangent2.y ) / 3.0 },
            bezierPoint2{ b.x, b.y }
        };
        int depth = 0;
        // 잘라낸 hull에도 분할 반올림 여유를 포함함.
        if( end < 1.0 ) { trim( curve, end, true ); ++depth; }
        if( begin > 0.0 ) { trim( curve, begin / end, false ); ++depth; }
        pending.push( { curve, i, begin, end, lowerBound( curve, depth ), depth } );
    }
    const auto midpoint = []( bezierPoint2 a, bezierPoint2 b )
    {
        return bezierPoint2{ 0.5 * ( a.x + b.x ), 0.5 * ( a.y + b.y ) };
    };
    for( std::size_t visited = 0; !pending.empty() && visited < MAX_NODES; ++visited )
    {
        const node current = pending.top();
        pending.pop();
        if( bestDistance - current.lower <= 0.75 * tolerance ) return best;
        const double middle = 0.5 * ( current.a + current.b );
        consider( current.segment, middle );
        const auto& curve = current.curve;
        const double dx = curve[3].x - curve[0].x, dy = curve[3].y - curve[0].y;
        const double lengthSquared = dx * dx + dy * dy;
        const double projection = lengthSquared > 0.0 ? std::clamp( ( ( point.x - curve[0].x ) * dx + ( point.y - curve[0].y ) * dy ) / lengthSquared, 0.0, 1.0 ) : 0.5;
        consider( current.segment, current.a + ( current.b - current.a ) * projection );
        if( bestDistance - current.lower <= 0.75 * tolerance ) return best;
        constexpr int MAX_DEPTH = 24;
        if( current.depth == MAX_DEPTH ) return {};
        const auto a = midpoint( curve[0], curve[1] ), b = midpoint( curve[1], curve[2] ), c = midpoint( curve[2], curve[3] );
        const auto d = midpoint( a, b ), e = midpoint( b, c ), center = midpoint( d, e );
        const std::array<bezierPoint2, 4> left{ curve[0], a, d, center }, right{ center, e, c, curve[3] };
        const double leftLower = lowerBound( left, current.depth + 1 ), rightLower = lowerBound( right, current.depth + 1 );
        if( leftLower <= bestDistance ) pending.push( { left, current.segment, current.a, middle, leftLower, current.depth + 1 } );
        if( rightLower <= bestDistance ) pending.push( { right, current.segment, middle, current.b, rightLower, current.depth + 1 } );
        if( pending.empty() ) return best;
    }
    return {};
}

} // namespace

std::optional<splineClosestPoint2> ClosestPoint( const catmullRomSpline2& spline, const vec2& point, double tolerance )
{
    const std::size_t count = GetSegmentCount( spline );
    if( count == 0 ) return {};
    return ClosestPointInRange( spline, { 0, 0.0f }, { count - 1, 1.0f }, point, tolerance );
}

std::optional<splineClosestPoint2> ClosestPoint( const catmullRomSpline2& spline, std::size_t segmentIndex, const vec2& point, double tolerance )
{
    if( segmentIndex >= GetSegmentCount( spline ) ) return {};
    return ClosestPointInRange( spline, { segmentIndex, 0.0f }, { segmentIndex, 1.0f }, point, tolerance );
}

std::optional<splineClosestPoint2> ClosestPoint( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, const vec2& point, double tolerance )
{
    return ClosestPointInRange( spline, from, to, point, tolerance );
}

} // namespace zonai
