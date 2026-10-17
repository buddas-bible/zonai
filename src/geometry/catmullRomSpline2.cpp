#include "geometry/catmullRomSpline2.h"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <array>
#include <limits>

namespace zonai
{

namespace
{

struct bezierPoint2
{
    double x;
    double y;
};

void GetTangents( const catmullRomSpline2& spline, std::size_t segmentIndex, vec2& tangent1, vec2& tangent2 )
{
    const vec2 delta = spline.points[segmentIndex + 1] - spline.points[segmentIndex];
    const float interval = std::sqrt( Length( delta ) );

    // 양 끝의 가상 제어점은 인접 edge를 연장하므로 끝 접선이 delta와 같음.
    tangent1 = delta;
    tangent2 = delta;

    if( segmentIndex > 0 )
    {
        const vec2 previous = spline.points[segmentIndex] - spline.points[segmentIndex - 1];
        const float previousInterval = std::sqrt( Length( previous ) );

        tangent1 = ( previous * ( interval / previousInterval ) * interval + delta * previousInterval ) / ( previousInterval + interval );
    }

    if( segmentIndex + 2 < spline.points.size() )
    {
        const vec2 next = spline.points[segmentIndex + 2] - spline.points[segmentIndex + 1];
        const float nextInterval = std::sqrt( Length( next ) );

        tangent2 = ( delta * nextInterval + next * ( interval / nextInterval ) * interval ) / ( interval + nextInterval );
    }
}

std::optional<double> GetSegmentLength( const catmullRomSpline2& spline, std::size_t segmentIndex, double end, double tolerance )
{
    if( !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};
    if( end == 0.0 ) return 0.0;
    vec2 tangent1{}, tangent2{};
    GetTangents( spline, segmentIndex, tangent1, tangent2 );
    if( !IsFinite( tangent1 ) || !IsFinite( tangent2 ) ) return {};
    const double dx = static_cast<double>( spline.points[segmentIndex + 1].x ) - spline.points[segmentIndex].x;
    const double dy = static_cast<double>( spline.points[segmentIndex + 1].y ) - spline.points[segmentIndex].y;
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
    const double fa = speed( 0.0 ), fm = speed( 0.5 * end ), fb = speed( end );
    return integrate( integrate, 0.0, end, fa, fm, fb, end * ( fa + 4.0 * fm + fb ) / 6.0, tolerance, 0 );
}

} // namespace

bool IsValid( const catmullRomSpline2& spline )
{
    if( spline.points.size() < 2 ) return false;

    for( std::size_t i = 0; i < spline.points.size(); ++i )
    {
        if( !IsFinite( spline.points[i] ) ) return false;

        if( i == 0 ) continue;

        const float lengthSquared = LengthSquared( spline.points[i] - spline.points[i - 1] );

        if( !std::isfinite( lengthSquared ) || lengthSquared <= 0.0f ) return false;
    }

    return true;
}

std::size_t GetSegmentCount( const catmullRomSpline2& spline )
{
    return spline.points.size() < 2 ? 0 : spline.points.size() - 1;
}

vec2 Evaluate( const catmullRomSpline2& spline, std::size_t segmentIndex, float t )
{
    const bool valid = IsValid( spline ) && segmentIndex < GetSegmentCount( spline ) && std::isfinite( t ) && t >= 0.0f && t <= 1.0f;

    assert( valid );
    if( !valid ) return {};

    if( t == 0.0f ) return spline.points[segmentIndex];
    if( t == 1.0f ) return spline.points[segmentIndex + 1];

    vec2 tangent1{};
    vec2 tangent2{};
    GetTangents( spline, segmentIndex, tangent1, tangent2 );

    const vec2 delta = spline.points[segmentIndex + 1] - spline.points[segmentIndex];
    const vec2 quadratic = 3.0f * delta - 2.0f * tangent1 - tangent2;
    const vec2 cubic = tangent1 + tangent2 - 2.0f * delta;

    return spline.points[segmentIndex] + t * ( tangent1 + t * ( quadratic + t * cubic ) );
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

    const vec2 delta = spline.points[segmentIndex + 1] - spline.points[segmentIndex];
    const vec2 quadratic = 3.0f * delta - 2.0f * tangent1 - tangent2;
    const vec2 cubic = tangent1 + tangent2 - 2.0f * delta;

    return tangent1 + t * ( 2.0f * quadratic + 3.0f * t * cubic );
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
    if( !IsValid( spline ) || !std::isfinite( maxError ) || maxError <= 0.0f ) return {};
    std::vector<vec2> samples{ spline.points.front() };
    const double errorSquared = static_cast<double>( maxError ) * maxError;
    const auto midpoint = []( bezierPoint2 a, bezierPoint2 b )
    {
        return bezierPoint2{ 0.5 * ( a.x + b.x ), 0.5 * ( a.y + b.y ) };
    };
    const auto subdivide = [&]( auto&& self, const std::array<bezierPoint2, 4>& curve, int depth ) -> bool
    {
        const vec2 end{ static_cast<float>( curve[3].x ), static_cast<float>( curve[3].y ) };
        if( !IsFinite( end ) ) return false;
        const vec2 start = samples.back();
        const double dx = static_cast<double>( end.x ) - start.x;
        const double dy = static_cast<double>( end.y ) - start.y;
        const double lengthSquared = dx * dx + dy * dy;
        bool flat = true;
        // 실제 float 출력 chord에 hull 전체를 대조해 S자 구간과 끝점 반올림 오차도 포함함.
        for( const auto point : curve )
        {
            const double px = point.x - start.x, py = point.y - start.y;
            const double t = lengthSquared > 0.0 ? std::clamp( ( px * dx + py * dy ) / lengthSquared, 0.0, 1.0 ) : 0.0;
            const double ex = px - t * dx, ey = py - t * dy;
            flat &= ex * ex + ey * ey <= errorSquared;
        }
        if( flat )
        {
            if( start.x == end.x && start.y == end.y ) return true;
            if( samples.size() == samples.max_size() ) return false;
            samples.push_back( end );
            return true;
        }
        constexpr int MAX_DEPTH = 12;
        if( depth == MAX_DEPTH ) return false;
        const auto a = midpoint( curve[0], curve[1] ), b = midpoint( curve[1], curve[2] ), c = midpoint( curve[2], curve[3] );
        const auto d = midpoint( a, b ), e = midpoint( b, c ), center = midpoint( d, e );
        return self( self, { curve[0], a, d, center }, depth + 1 ) && self( self, { center, e, c, curve[3] }, depth + 1 );
    };
    for( std::size_t i = 0; i < GetSegmentCount( spline ); ++i )
    {
        vec2 tangent1{}, tangent2{};
        GetTangents( spline, i, tangent1, tangent2 );
        const vec2 a = spline.points[i], b = spline.points[i + 1];
        const std::array<bezierPoint2, 4> curve{
            bezierPoint2{ a.x, a.y },
            bezierPoint2{ a.x + static_cast<double>( tangent1.x ) / 3.0, a.y + static_cast<double>( tangent1.y ) / 3.0 },
            bezierPoint2{ b.x - static_cast<double>( tangent2.x ) / 3.0, b.y - static_cast<double>( tangent2.y ) / 3.0 },
            bezierPoint2{ b.x, b.y }
        };
        if( !subdivide( subdivide, curve, 0 ) ) return {};
    }
    return samples;
}

std::optional<double> GetLength( const catmullRomSpline2& spline, double tolerance )
{
    if( !IsValid( spline ) || !std::isfinite( tolerance ) || tolerance <= 0.0 ) return {};
    const std::size_t count = GetSegmentCount( spline );
    double length = 0.0;
    for( std::size_t i = 0; i < count; ++i )
    {
        const auto segmentLength = GetSegmentLength( spline, i, 1.0, tolerance / ( 4.0 * count ) );
        if( !segmentLength || *segmentLength <= 0.0 ) return {};
        length += *segmentLength;
    }
    if( !std::isfinite( length ) || tolerance < 32.0 * std::numeric_limits<double>::epsilon() * length ) return {};
    return length;
}

std::optional<splineSample2> EvaluateByDistance( const catmullRomSpline2& spline, double distance, double tolerance )
{
    if( !std::isfinite( distance ) || distance < 0.0 ) return {};
    const auto length = GetLength( spline, tolerance );
    if( !length || distance > *length ) return {};
    const auto sample = [&]( std::size_t segment, float t ) -> std::optional<splineSample2>
    {
        const vec2 point = Evaluate( spline, segment, t );
        const vec2 derivative = Derivative( spline, segment, t );
        const double speed = std::hypot( static_cast<double>( derivative.x ), static_cast<double>( derivative.y ) );
        if( !IsFinite( point ) || !std::isfinite( speed ) || speed <= 0.0 ) return {};
        if( t > 0.0f && t < 1.0f )
        {
            vec2 tangent1{}, tangent2{};
            GetTangents( spline, segment, tangent1, tangent2 );
            const vec2 a = spline.points[segment], b = spline.points[segment + 1];
            const double dx = static_cast<double>( b.x ) - a.x, dy = static_cast<double>( b.y ) - a.y;
            const double qx = 3.0 * dx - 2.0 * tangent1.x - tangent2.x, qy = 3.0 * dy - 2.0 * tangent1.y - tangent2.y;
            const double cx = tangent1.x + static_cast<double>( tangent2.x ) - 2.0 * dx, cy = tangent1.y + static_cast<double>( tangent2.y ) - 2.0 * dy;
            const double ex = ( static_cast<double>( point.x ) - a.x ) - t * ( tangent1.x + t * ( qx + t * cx ) );
            const double ey = ( static_cast<double>( point.y ) - a.y ) - t * ( tangent1.y + t * ( qy + t * cy ) );
            if( std::hypot( ex, ey ) > 0.25 * tolerance ) return {};
        }
        return splineSample2{ point, { static_cast<float>( derivative.x / speed ), static_cast<float>( derivative.y / speed ) } };
    };
    const std::size_t count = GetSegmentCount( spline );
    if( distance == 0.0 ) return sample( 0, 0.0f );
    if( distance == *length ) return sample( count - 1, 1.0f );
    const double error = tolerance / ( 4.0 * count );
    double prefix = 0.0;
    for( std::size_t i = 0; i < count; ++i )
    {
        const auto segmentLength = GetSegmentLength( spline, i, 1.0, error );
        if( !segmentLength ) return {};
        const double end = prefix + *segmentLength;
        if( distance > end )
        {
            prefix = end;
            continue;
        }
        if( distance == end ) return sample( i, 1.0f );
        const double target = distance - prefix;
        double lower = 0.0, upper = 1.0;
        constexpr int MAX_ITERATIONS = 64;
        for( int iteration = 0; iteration < MAX_ITERATIONS; ++iteration )
        {
            const float t = static_cast<float>( 0.5 * ( lower + upper ) );
            if( t <= lower || t >= upper ) return {};
            const auto partial = GetSegmentLength( spline, i, t, error );
            if( !partial ) return {};
            if( std::abs( *partial - target ) <= 0.5 * tolerance ) return sample( i, t );
            if( *partial < target ) lower = t;
            else upper = t;
        }
        return {};
    }
    return {};
}

} // namespace zonai
