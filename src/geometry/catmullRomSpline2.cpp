#include "geometry/catmullRomSpline2.h"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <array>

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

} // namespace zonai
