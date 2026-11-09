#include "fluid/fluidField2.h"

#include "collision/distance2.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <numbers>
#include <type_traits>
#include <utility>
#include <variant>

namespace zonai
{
namespace
{

bool ContainsFluidObstacle( const fluidShapeObstacle2& obstacle, vec2 point )
{
    const vec2 local = InverseTransformPoint( obstacle.transform, point );
    if( const auto* circle = std::get_if<circle2>( &obstacle.geometry ) )
        return circle->radius > 0.0f && Contains( *circle, local );

    if( const auto* capsule = std::get_if<capsule2>( &obstacle.geometry ) )
        return capsule->radius > 0.0f && Contains( *capsule, local );

    if( const auto* polygon = std::get_if<polygon2>( &obstacle.geometry ) )
    {
        if( polygon->vertexCount < 2 || polygon->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) ||
            !std::isfinite( polygon->radius ) || polygon->radius < 0.0f ) return false;
        if( polygon->vertexCount == 2 )
            return polygon->radius > 0.0f &&
                DistanceSquared( segment2{ polygon->vertices[0], polygon->vertices[1] }, local ) <=
                    polygon->radius * polygon->radius;

        bool inside = true;
        float closestSquared = std::numeric_limits<float>::max();
        for( int i = 0; i < polygon->vertexCount; ++i )
        {
            if( Dot( polygon->normals[i], local - polygon->vertices[i] ) > 0.0f ) inside = false;
            const int next = ( i + 1 ) % polygon->vertexCount;
            closestSquared = std::min( closestSquared,
                DistanceSquared( segment2{ polygon->vertices[i], polygon->vertices[next] }, local ) );
        }
        return inside || ( polygon->radius > 0.0f && closestSquared <= polygon->radius * polygon->radius );
    }
    return false;
}

bool HasSameFluidGeometry( const shapeGeometry& a, const shapeGeometry& b )
{
    if( a.index() != b.index() ) return false;
    if( const auto* lhs = std::get_if<circle2>( &a ) )
    {
        const auto& rhs = std::get<circle2>( b );
        return lhs->radius == rhs.radius && lhs->center.x == rhs.center.x && lhs->center.y == rhs.center.y;
    }
    if( const auto* lhs = std::get_if<capsule2>( &a ) )
    {
        const auto& rhs = std::get<capsule2>( b );
        return lhs->radius == rhs.radius &&
            lhs->center1.x == rhs.center1.x && lhs->center1.y == rhs.center1.y &&
            lhs->center2.x == rhs.center2.x && lhs->center2.y == rhs.center2.y;
    }
    if( const auto* lhs = std::get_if<polygon2>( &a ) )
    {
        const auto& rhs = std::get<polygon2>( b );
        if( lhs->vertexCount != rhs.vertexCount || lhs->radius != rhs.radius ||
            lhs->vertexCount < 2 || lhs->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) ) return false;
        for( int i = 0; i < lhs->vertexCount; ++i )
            if( lhs->vertices[i].x != rhs.vertices[i].x || lhs->vertices[i].y != rhs.vertices[i].y ||
                lhs->normals[i].x != rhs.normals[i].x || lhs->normals[i].y != rhs.normals[i].y )
                return false;
        return true;
    }
    return false;
}

float FluidGeometryRadius( const shapeGeometry& geometry )
{
    if( const auto* circle = std::get_if<circle2>( &geometry ) )
        return Length( circle->center ) + circle->radius;
    if( const auto* capsule = std::get_if<capsule2>( &geometry ) )
        return std::max( Length( capsule->center1 ), Length( capsule->center2 ) ) + capsule->radius;
    if( const auto* polygon = std::get_if<polygon2>( &geometry ) )
    {
        if( polygon->vertexCount < 2 || polygon->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) )
            return std::numeric_limits<float>::quiet_NaN();
        float radius = polygon->radius;
        for( int i = 0; i < polygon->vertexCount; ++i )
            radius = std::max( radius, Length( polygon->vertices[i] ) + polygon->radius );
        return radius;
    }
    return 0.0f;
}

bool IsValidFluidObstacle( const fluidShapeObstacle2& obstacle )
{
    return IsFinite( obstacle.center ) && IsFinite( obstacle.transform.position ) &&
        std::isfinite( obstacle.transform.rotation.c ) && std::isfinite( obstacle.transform.rotation.s ) &&
        IsFinite( obstacle.linearVelocity ) && std::isfinite( obstacle.angularVelocity );
}

// 도형의 local 원점에서 모든 점을 감싸는 보수적인 world 반경.
// 회전이 비정규화된 입력이어도 false-negative culling을 피하도록 확대한다.
// 비정상 기하일 때는 성능 필터만 끄고 기존 Contains 판정을 유지한다.
double FluidObstacleBoundRadius( const fluidShapeObstacle2& obstacle )
{
    const double localRadius = FluidGeometryRadius( obstacle.geometry );
    const double scale = std::max( 1.0, std::hypot(
        static_cast<double>( obstacle.transform.rotation.c ),
        static_cast<double>( obstacle.transform.rotation.s ) ) );
    const double radius = localRadius * scale;
    return std::isfinite( radius ) && radius >= 0.0
        ? radius : std::numeric_limits<double>::infinity();
}

// 단일 원형 장애물에만 사용하는 분석적 Cut-cell 경계 클리핑.
// 다중 장애물의 겹침/우선순위는 기존 공통 공간 표본 경로로 유지한다.
struct analyticFluidCircle
{
    vec2 center{};
    double radius = 0.0;
};

bool TryGetAnalyticCircle( std::span<const fluidShapeObstacle2> obstacles, analyticFluidCircle& result )
{
    if( obstacles.size() != 1 || !IsValidFluidObstacle( obstacles.front() ) ) return false;
    const auto* circle = std::get_if<circle2>( &obstacles.front().geometry );
    if( !circle || !std::isfinite( circle->radius ) || circle->radius <= 0.0f ) return false;
    const vec2 center = TransformPoint( obstacles.front().transform, circle->center );
    if( !IsFinite( center ) ) return false;
    result = { center, circle->radius };
    return true;
}

struct analyticCircleInterval
{
    double length = 0.0;
    double midpoint = 0.0;
};

// 월드 축에 평행한 MAC face와 원의 교차 구간 [min,max]를 정확히 잘라냄.
analyticCircleInterval ClipCircleFace( const analyticFluidCircle& circle,
    bool vertical, double coordinate, double low, double high )
{
    const double normal = vertical ? circle.center.x : circle.center.y;
    const double tangent = vertical ? circle.center.y : circle.center.x;
    const double distance = coordinate - normal;
    if( std::abs( distance ) >= circle.radius ) return {};
    const double extent = std::sqrt( std::max( 0.0,
        circle.radius * circle.radius - distance * distance ) );
    const double left = std::max( low, tangent - extent );
    const double right = std::min( high, tangent + extent );
    if( right <= left ) return {};
    return { right - left, 0.5 * ( left + right ) };
}

// 원의 가로 단면 높이를 구간별로 적분한다. 셀의 상·하단이 원과 만나는
// x 위치에서 분할하면 단면은 상수 + k*sqrt(r²-x²), k=0,1,2가 된다.
double CircleRectangleArea( const analyticFluidCircle& circle,
    double x0, double y0, double x1, double y1 )
{
    const double r = circle.radius;
    const double left = std::max( x0, static_cast<double>( circle.center.x ) - r );
    const double right = std::min( x1, static_cast<double>( circle.center.x ) + r );
    if( right <= left ) return 0.0;

    std::array<double, 6> cuts{};
    int count = 0;
    cuts[count++] = left;
    cuts[count++] = right;
    for( const double y : { y0, y1 } )
    {
        const double dy = y - circle.center.y;
        if( std::abs( dy ) >= r ) continue;
        const double offset = std::sqrt( r * r - dy * dy );
        for( const double x : { static_cast<double>( circle.center.x ) - offset,
                                static_cast<double>( circle.center.x ) + offset } )
            if( x > left && x < right ) cuts[count++] = x;
    }
    std::sort( cuts.begin(), cuts.begin() + count );

    const auto primitive = [&]( double x )
    {
        const double t = std::clamp( x - circle.center.x, -r, r );
        return 0.5 * ( t * std::sqrt( std::max( 0.0, r * r - t * t ) ) +
            r * r * std::asin( std::clamp( t / r, -1.0, 1.0 ) ) );
    };
    double area = 0.0;
    for( int i = 1; i < count; ++i )
    {
        const double a = cuts[i - 1];
        const double b = cuts[i];
        if( b <= a ) continue;
        const double mid = 0.5 * ( a + b ) - circle.center.x;
        const double halfHeight = std::sqrt( std::max( 0.0, r * r - mid * mid ) );
        const double circleTop = circle.center.y + halfHeight;
        const double circleBottom = circle.center.y - halfHeight;
        if( circleTop <= y0 || circleBottom >= y1 ) continue;
        const bool freeTop = circleTop <= y1;
        const bool freeBottom = circleBottom >= y0;
        const double constant = ( freeTop ? circle.center.y : y1 ) -
            ( freeBottom ? circle.center.y : y0 );
        const double sqrtScale = static_cast<int>( freeTop ) + static_cast<int>( freeBottom );
        area += constant * ( b - a ) + sqrtScale * ( primitive( b ) - primitive( a ) );
    }
    return std::clamp( area, 0.0, ( x1 - x0 ) * ( y1 - y0 ) );
}

// 원과 달리 반지름이 0인 단일 convex polygon의 경계는 선분 조합이다.
// CCW polygon의 half-plane으로 격자 셀과 MAC 면을 정확히 clipping한다.
struct analyticFluidPolygon
{
    std::array<vec2, MAX_POLYGON_VERTICES> vertices{};
    int count = 0;
};

bool TryGetAnalyticPolygon( std::span<const fluidShapeObstacle2> obstacles, analyticFluidPolygon& result )
{
    if( obstacles.size() != 1 || !IsValidFluidObstacle( obstacles.front() ) ) return false;
    const auto* polygon = std::get_if<polygon2>( &obstacles.front().geometry );
    if( !polygon || polygon->radius != 0.0f || polygon->vertexCount < 3 ||
        polygon->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) ) return false;
    result = {};
    result.count = polygon->vertexCount;
    for( int i = 0; i < result.count; ++i )
    {
        result.vertices[i] = TransformPoint( obstacles.front().transform, polygon->vertices[i] );
        if( !IsFinite( result.vertices[i] ) ) return false;
    }
    // Box와 MakePolygon이 제공하는 CCW, strictly convex geometry만 처리한다.
    double area2 = 0.0;
    for( int i = 0; i < result.count; ++i )
    {
        const vec2& a = result.vertices[i];
        const vec2& b = result.vertices[( i + 1 ) % result.count];
        area2 += static_cast<double>( a.x ) * b.y - static_cast<double>( b.x ) * a.y;
    }
    if( area2 <= 1.0e-8 ) return false;
    for( int i = 0; i < result.count; ++i )
    {
        const vec2 a = result.vertices[i];
        const vec2 b = result.vertices[( i + 1 ) % result.count];
        const vec2 c = result.vertices[( i + 2 ) % result.count];
        if( Cross( b - a, c - b ) <= 0.0f ) return false;
    }
    return true;
}

// 공유 MAC face의 line segment를 polygon의 CCW half-planes에 clip.
// 결과 길이/중심점은 면적 개방율, 강체 표면 속도, Schur 힘과 swept flux가 공유한다.
analyticCircleInterval ClipPolygonFace( const analyticFluidPolygon& polygon,
    bool vertical, double coordinate, double low, double high )
{
    double enter = low;
    double exit = high;
    for( int i = 0; i < polygon.count; ++i )
    {
        const vec2 a = polygon.vertices[i];
        const vec2 b = polygon.vertices[( i + 1 ) % polygon.count];
        const double ex = static_cast<double>( b.x ) - a.x;
        const double ey = static_cast<double>( b.y ) - a.y;
        // cross(edge, P(t)-a) >= 0, t is tangent position.
        const double constant = vertical
            ? ex * ( -static_cast<double>( a.y ) ) - ey * ( coordinate - a.x )
            : ex * ( coordinate - a.y ) - ey * ( -static_cast<double>( a.x ) );
        const double slope = vertical ? ex : -ey;
        if( slope == 0.0 )
        {
            if( constant < 0.0 ) return {};
            continue;
        }
        const double boundary = -constant / slope;
        if( slope > 0.0 ) enter = std::max( enter, boundary );
        else exit = std::min( exit, boundary );
        if( exit <= enter ) return {};
    }
    return { exit - enter, 0.5 * ( enter + exit ) };
}

struct analyticPoint2
{
    double x = 0.0;
    double y = 0.0;
};

// Sutherland-Hodgman 4 half-plane rectangle clipping. Convex input has
// <=MAX_POLYGON_VERTICES+4 clipped vertices; no per-cell heap allocations.
double ConvexPolygonRectangleArea( const analyticFluidPolygon& polygon,
    double x0, double y0, double x1, double y1 )
{
    constexpr std::size_t capacity = MAX_POLYGON_VERTICES + 8;
    std::array<analyticPoint2, capacity> current{};
    std::array<analyticPoint2, capacity> next{};
    int count = polygon.count;
    for( int i = 0; i < count; ++i )
        current[i] = { polygon.vertices[i].x, polygon.vertices[i].y };

    for( int side = 0; side < 4 && count > 0; ++side )
    {
        const auto signedDistance = [&]( analyticPoint2 p )
        {
            if( side == 0 ) return p.x - x0;
            if( side == 1 ) return x1 - p.x;
            if( side == 2 ) return p.y - y0;
            return y1 - p.y;
        };
        int nextCount = 0;
        analyticPoint2 previous = current[count - 1];
        double before = signedDistance( previous );
        for( int i = 0; i < count; ++i )
        {
            const analyticPoint2 point = current[i];
            const double after = signedDistance( point );
            if( ( before >= 0.0 ) != ( after >= 0.0 ) )
            {
                const double t = before / ( before - after );
                next[nextCount++] = { previous.x + t * ( point.x - previous.x ),
                    previous.y + t * ( point.y - previous.y ) };
            }
            if( after >= 0.0 ) next[nextCount++] = point;
            previous = point;
            before = after;
        }
        current.swap( next );
        count = nextCount;
    }
    if( count < 3 ) return 0.0;
    double area2 = 0.0;
    const analyticPoint2 origin = current[0];
    for( int i = 1; i + 1 < count; ++i )
        area2 += ( current[i].x - origin.x ) * ( current[i + 1].y - origin.y ) -
            ( current[i].y - origin.y ) * ( current[i + 1].x - origin.x );
    return std::clamp( 0.5 * std::abs( area2 ), 0.0, ( x1 - x0 ) * ( y1 - y0 ) );
}

// Capsule는 선분과 원판의 Minkowski 합. 중앙 직사각형과 양 끝의
// 분리된 반원 영역으로 나눠 각 cell 면적을 정확히 적분한다.
struct analyticFluidCapsule
{
    analyticFluidCircle first{};
    analyticFluidCircle last{};
    analyticFluidPolygon core{};
    vec2 axis{};
};

bool TryGetAnalyticCapsule( std::span<const fluidShapeObstacle2> obstacles, analyticFluidCapsule& result )
{
    if( obstacles.size() != 1 || !IsValidFluidObstacle( obstacles.front() ) ) return false;
    const auto* shape = std::get_if<capsule2>( &obstacles.front().geometry );
    if( !shape || !std::isfinite( shape->radius ) || shape->radius <= 0.0f ||
        !IsFinite( shape->center1 ) || !IsFinite( shape->center2 ) ) return false;

    const vec2 a = TransformPoint( obstacles.front().transform, shape->center1 );
    const vec2 b = TransformPoint( obstacles.front().transform, shape->center2 );
    if( !IsFinite( a ) || !IsFinite( b ) ) return false;
    const double dx = static_cast<double>( b.x ) - a.x;
    const double dy = static_cast<double>( b.y ) - a.y;
    const double length = std::hypot( dx, dy );
    if( !std::isfinite( length ) || length <= 1.0e-5 ) return false;
    const vec2 axis{ static_cast<float>( dx / length ), static_cast<float>( dy / length ) };
    const vec2 perpendicular{ -axis.y, axis.x };
    const vec2 radiusVector = shape->radius * perpendicular;
    result = {};
    result.first = { a, shape->radius };
    result.last = { b, shape->radius };
    result.axis = axis;
    result.core.count = 4;
    result.core.vertices[0] = a - radiusVector;
    result.core.vertices[1] = b - radiusVector;
    result.core.vertices[2] = b + radiusVector;
    result.core.vertices[3] = a + radiusVector;
    return true;
}

analyticCircleInterval ClipCapsuleCapFace( const analyticFluidCapsule& capsule,
    bool firstCap, bool vertical, double coordinate, double low, double high )
{
    const analyticFluidCircle& circle = firstCap ? capsule.first : capsule.last;
    const auto whole = ClipCircleFace( circle, vertical, coordinate, low, high );
    if( whole.length <= 0.0 ) return {};
    double enter = whole.midpoint - 0.5 * whole.length;
    double exit = whole.midpoint + 0.5 * whole.length;
    // first cap: (p-a)·axis <= 0. last cap: (p-b)·axis >= 0.
    const double direction = firstCap ? -1.0 : 1.0;
    const double slope = direction * ( vertical ? capsule.axis.y : capsule.axis.x );
    const double intercept = direction * ( vertical
        ? static_cast<double>( capsule.axis.x ) * ( coordinate - circle.center.x ) -
            static_cast<double>( capsule.axis.y ) * circle.center.y
        : -static_cast<double>( capsule.axis.x ) * circle.center.x +
            static_cast<double>( capsule.axis.y ) * ( coordinate - circle.center.y ) );
    if( std::abs( slope ) <= 1.0e-14 )
    {
        if( intercept < 0.0 ) return {};
    }
    else
    {
        const double threshold = -intercept / slope;
        if( slope > 0.0 ) enter = std::max( enter, threshold );
        else exit = std::min( exit, threshold );
    }
    return exit > enter ? analyticCircleInterval{ exit - enter, 0.5 * ( enter + exit ) }
        : analyticCircleInterval{};
}

analyticCircleInterval ClipCapsuleFace( const analyticFluidCapsule& capsule,
    bool vertical, double coordinate, double low, double high )
{
    // 대부분의 MAC face는 Capsule AABB 외부다. 정확한 클리핑 이전에 제외.
    const double r = capsule.first.radius;
    const double minX = std::min( capsule.first.center.x, capsule.last.center.x ) - r;
    const double maxX = std::max( capsule.first.center.x, capsule.last.center.x ) + r;
    const double minY = std::min( capsule.first.center.y, capsule.last.center.y ) - r;
    const double maxY = std::max( capsule.first.center.y, capsule.last.center.y ) + r;
    if( vertical ? ( coordinate <= minX || coordinate >= maxX || high <= minY || low >= maxY )
                 : ( coordinate <= minY || coordinate >= maxY || high <= minX || low >= maxX ) )
        return {};
    const std::array<analyticCircleInterval, 3> parts{
        ClipPolygonFace( capsule.core, vertical, coordinate, low, high ),
        ClipCapsuleCapFace( capsule, true, vertical, coordinate, low, high ),
        ClipCapsuleCapFace( capsule, false, vertical, coordinate, low, high )
    };
    double enter = high;
    double exit = low;
    for( const auto& segment : parts )
    {
        if( segment.length <= 0.0 ) continue;
        enter = std::min( enter, segment.midpoint - 0.5 * segment.length );
        exit = std::max( exit, segment.midpoint + 0.5 * segment.length );
    }
    return exit > enter ? analyticCircleInterval{ exit - enter, 0.5 * ( enter + exit ) }
        : analyticCircleInterval{};
}

// CCW polygon과 원판의 정확한 교차 면적: 각 선분을 원 교차점에서
// 분할하고 내부는 signed triangle, 외부는 signed circular sector로 적분.
// 참조: KACTL CirclePolygonIntersection (CC0).
double CircleClippedPolygonArea( analyticFluidCircle circle,
    const std::array<analyticPoint2, 8>& vertices, int count )
{
    if( count < 3 ) return 0.0;
    const double r2 = circle.radius * circle.radius;
    double signedArea = 0.0;
    for( int i = 0; i < count; ++i )
    {
        const analyticPoint2& a = vertices[i];
        const analyticPoint2& b = vertices[( i + 1 ) % count];
        const double ax = a.x - circle.center.x;
        const double ay = a.y - circle.center.y;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double aa = dx * dx + dy * dy;
        if( aa <= 1.0e-28 ) continue;

        std::array<double, 4> times{ 0.0, 1.0, 1.0, 1.0 };
        int samples = 2;
        const double bb = 2.0 * ( ax * dx + ay * dy );
        const double cc = ax * ax + ay * ay - r2;
        const double discriminant = bb * bb - 4.0 * aa * cc;
        if( discriminant > 0.0 )
        {
            const double root = std::sqrt( discriminant );
            for( const double t : { ( -bb - root ) / ( 2.0 * aa ),
                                    ( -bb + root ) / ( 2.0 * aa ) } )
                if( t > 0.0 && t < 1.0 ) times[samples++] = t;
        }
        std::sort( times.begin(), times.begin() + samples );
        for( int j = 1; j < samples; ++j )
        {
            const double t0 = times[j - 1];
            const double t1 = times[j];
            if( t1 <= t0 ) continue;
            const double px = ax + t0 * dx, py = ay + t0 * dy;
            const double qx = ax + t1 * dx, qy = ay + t1 * dy;
            const double mx = 0.5 * ( px + qx ), my = 0.5 * ( py + qy );
            const double cross = px * qy - py * qx;
            if( mx * mx + my * my <= r2 )
                signedArea += 0.5 * cross;
            else
                signedArea += 0.5 * r2 * std::atan2( cross, px * qx + py * qy );
        }
    }
    return std::abs( signedArea );
}

// Cell을 Capsule 축의 양쪽 끝 반평면으로 잘라 독립적인 반원 영역을 얻는다.
double CapsuleCapRectangleArea( const analyticFluidCapsule& capsule, bool firstCap,
    double x0, double y0, double x1, double y1 )
{
    std::array<analyticPoint2, 8> current{{
        { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 }
    }};
    std::array<analyticPoint2, 8> next{};
    const analyticFluidCircle& circle = firstCap ? capsule.first : capsule.last;
    const double direction = firstCap ? -1.0 : 1.0;
    const auto signedDistance = [&]( analyticPoint2 p )
    {
        return direction * (
            static_cast<double>( capsule.axis.x ) * ( p.x - circle.center.x ) +
            static_cast<double>( capsule.axis.y ) * ( p.y - circle.center.y ) );
    };
    int count = 4;
    int nextCount = 0;
    analyticPoint2 previous = current[count - 1];
    double before = signedDistance( previous );
    for( int i = 0; i < count; ++i )
    {
        const analyticPoint2 point = current[i];
        const double after = signedDistance( point );
        if( ( before >= 0.0 ) != ( after >= 0.0 ) )
        {
            const double t = before / ( before - after );
            next[nextCount++] = { previous.x + t * ( point.x - previous.x ),
                previous.y + t * ( point.y - previous.y ) };
        }
        if( after >= 0.0 ) next[nextCount++] = point;
        previous = point;
        before = after;
    }
    current.swap( next );
    count = nextCount;
    if( count < 3 ) return 0.0;
    return CircleClippedPolygonArea( circle, current, count );
}

double CapsuleRectangleArea( const analyticFluidCapsule& capsule,
    double x0, double y0, double x1, double y1 )
{
    const double r = capsule.first.radius;
    const double minX = std::min( capsule.first.center.x, capsule.last.center.x ) - r;
    const double maxX = std::max( capsule.first.center.x, capsule.last.center.x ) + r;
    const double minY = std::min( capsule.first.center.y, capsule.last.center.y ) - r;
    const double maxY = std::max( capsule.first.center.y, capsule.last.center.y ) + r;
    if( x1 <= minX || x0 >= maxX || y1 <= minY || y0 >= maxY ) return 0.0;
    const double area = ConvexPolygonRectangleArea( capsule.core, x0, y0, x1, y1 ) +
        CapsuleCapRectangleArea( capsule, true, x0, y0, x1, y1 ) +
        CapsuleCapRectangleArea( capsule, false, x0, y0, x1, y1 );
    return std::clamp( area, 0.0, ( x1 - x0 ) * ( y1 - y0 ) );
}

// Rounded Polygon = convex core + 각 변 외부 strip + 각 꼭짓점의 바깥 원호 sector.
// 세 부분은 convex shape의 nearest-feature Voronoi 영역으로 면적이 겹치지 않는다.
struct analyticFluidRoundedPolygon
{
    analyticFluidPolygon core{};
    std::array<analyticFluidPolygon, MAX_POLYGON_VERTICES> strips{};
    std::array<vec2, MAX_POLYGON_VERTICES> outwardNormals{};
    double radius = 0.0;
    double minX = 0.0;
    double maxX = 0.0;
    double minY = 0.0;
    double maxY = 0.0;
};

bool TryGetAnalyticRoundedPolygon( std::span<const fluidShapeObstacle2> obstacles,
    analyticFluidRoundedPolygon& result )
{
    if( obstacles.size() != 1 || !IsValidFluidObstacle( obstacles.front() ) ) return false;
    fluidShapeObstacle2 sharp = obstacles.front();
    auto* polygon = std::get_if<polygon2>( &sharp.geometry );
    if( !polygon || !std::isfinite( polygon->radius ) || polygon->radius <= 0.0f ) return false;
    const double radius = polygon->radius;
    polygon->radius = 0.0f;
    analyticFluidRoundedPolygon rounded{};
    if( !TryGetAnalyticPolygon( std::span<const fluidShapeObstacle2>( &sharp, 1 ), rounded.core ) )
        return false;

    rounded.radius = radius;
    rounded.minX = rounded.maxX = rounded.core.vertices[0].x;
    rounded.minY = rounded.maxY = rounded.core.vertices[0].y;
    for( int i = 0; i < rounded.core.count; ++i )
    {
        const vec2 a = rounded.core.vertices[i];
        const vec2 b = rounded.core.vertices[( i + 1 ) % rounded.core.count];
        const vec2 edge = b - a;
        const float length = Length( edge );
        if( !std::isfinite( length ) || length <= 1.0e-6f ) return false;
        const vec2 outward{ edge.y / length, -edge.x / length };
        rounded.outwardNormals[i] = outward;
        const vec2 offset = static_cast<float>( radius ) * outward;
        analyticFluidPolygon& strip = rounded.strips[i];
        strip.count = 4;
        strip.vertices[0] = a;
        strip.vertices[1] = a + offset;
        strip.vertices[2] = b + offset;
        strip.vertices[3] = b;
        rounded.minX = std::min( rounded.minX, static_cast<double>( a.x ) );
        rounded.maxX = std::max( rounded.maxX, static_cast<double>( a.x ) );
        rounded.minY = std::min( rounded.minY, static_cast<double>( a.y ) );
        rounded.maxY = std::max( rounded.maxY, static_cast<double>( a.y ) );
    }
    rounded.minX -= radius;
    rounded.maxX += radius;
    rounded.minY -= radius;
    rounded.maxY += radius;
    result = rounded;
    return true;
}

// Circle chord를 vertex의 두 바깥 edge normal 사이 sector로 clip.
analyticCircleInterval ClipRoundedCornerFace( const analyticFluidRoundedPolygon& polygon,
    int vertexIndex, bool vertical, double coordinate, double low, double high )
{
    const int count = polygon.core.count;
    const vec2 vertex = polygon.core.vertices[vertexIndex];
    const vec2 previous = polygon.outwardNormals[( vertexIndex + count - 1 ) % count];
    const vec2 next = polygon.outwardNormals[vertexIndex];
    const auto chord = ClipCircleFace( { vertex, polygon.radius },
        vertical, coordinate, low, high );
    if( chord.length <= 0.0 ) return {};

    double enter = chord.midpoint - 0.5 * chord.length;
    double exit = chord.midpoint + 0.5 * chord.length;
    const double px = vertical ? coordinate : 0.0;
    const double py = vertical ? 0.0 : coordinate;
    const double dx = vertical ? 0.0 : 1.0;
    const double dy = vertical ? 1.0 : 0.0;
    const auto clipHalfPlane = [&]( double constant, double slope )
    {
        if( std::abs( slope ) <= 1.0e-14 ) return constant >= 0.0;
        const double crossing = -constant / slope;
        if( slope > 0.0 ) enter = std::max( enter, crossing );
        else exit = std::min( exit, crossing );
        return exit > enter;
    };

    // cross(previousNormal, P - vertex) >= 0
    if( !clipHalfPlane(
        static_cast<double>( previous.x ) * ( py - vertex.y ) -
            static_cast<double>( previous.y ) * ( px - vertex.x ),
        static_cast<double>( previous.x ) * dy - static_cast<double>( previous.y ) * dx ) )
        return {};
    // cross(P - vertex, nextNormal) >= 0
    if( !clipHalfPlane(
        ( px - vertex.x ) * static_cast<double>( next.y ) -
            ( py - vertex.y ) * static_cast<double>( next.x ),
        dx * static_cast<double>( next.y ) - dy * static_cast<double>( next.x ) ) )
        return {};
    return { exit - enter, 0.5 * ( enter + exit ) };
}

analyticCircleInterval ClipRoundedPolygonFace( const analyticFluidRoundedPolygon& polygon,
    bool vertical, double coordinate, double low, double high )
{
    if( vertical ? ( coordinate <= polygon.minX || coordinate >= polygon.maxX ||
                      high <= polygon.minY || low >= polygon.maxY )
                 : ( coordinate <= polygon.minY || coordinate >= polygon.maxY ||
                      high <= polygon.minX || low >= polygon.maxX ) )
        return {};

    // Rounded convex polygon과 직선의 교차는 연속 구간. 각 feature의 구간을 합산.
    double enter = high;
    double exit = low;
    const auto collect = [&]( analyticCircleInterval clipped )
    {
        if( clipped.length <= 0.0 ) return;
        enter = std::min( enter, clipped.midpoint - 0.5 * clipped.length );
        exit = std::max( exit, clipped.midpoint + 0.5 * clipped.length );
    };
    collect( ClipPolygonFace( polygon.core, vertical, coordinate, low, high ) );
    for( int i = 0; i < polygon.core.count; ++i )
    {
        collect( ClipPolygonFace( polygon.strips[i], vertical, coordinate, low, high ) );
        collect( ClipRoundedCornerFace( polygon, i, vertical, coordinate, low, high ) );
    }
    return exit > enter ? analyticCircleInterval{ exit - enter, 0.5 * ( enter + exit ) }
        : analyticCircleInterval{};
}

// 단일 rounded vertex Voronoi sector와 셀 사각형 교차. CircleClippedPolygonArea
// 는 polygon 영역 중 원판 내부 면적을 edge-triangle / sector로 정확히 적분한다.
double RoundedCornerRectangleArea( const analyticFluidRoundedPolygon& polygon, int vertexIndex,
    double x0, double y0, double x1, double y1 )
{
    const int count = polygon.core.count;
    const vec2 vertex = polygon.core.vertices[vertexIndex];
    const vec2 previous = polygon.outwardNormals[( vertexIndex + count - 1 ) % count];
    const vec2 next = polygon.outwardNormals[vertexIndex];
    std::array<analyticPoint2, 8> current{{
        { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 }
    }};
    std::array<analyticPoint2, 8> temporary{};
    int vertices = 4;
    for( int plane = 0; plane < 2 && vertices > 0; ++plane )
    {
        const auto signedDistance = [&]( analyticPoint2 p )
        {
            const double dx = p.x - vertex.x;
            const double dy = p.y - vertex.y;
            return plane == 0
                ? static_cast<double>( previous.x ) * dy - static_cast<double>( previous.y ) * dx
                : dx * static_cast<double>( next.y ) - dy * static_cast<double>( next.x );
        };
        int nextCount = 0;
        analyticPoint2 a = current[vertices - 1];
        double da = signedDistance( a );
        for( int i = 0; i < vertices; ++i )
        {
            const analyticPoint2 b = current[i];
            const double db = signedDistance( b );
            if( ( da >= 0.0 ) != ( db >= 0.0 ) )
            {
                const double t = da / ( da - db );
                temporary[nextCount++] = { a.x + t * ( b.x - a.x ), a.y + t * ( b.y - a.y ) };
            }
            if( db >= 0.0 ) temporary[nextCount++] = b;
            a = b;
            da = db;
        }
        current.swap( temporary );
        vertices = nextCount;
    }
    return vertices >= 3
        ? CircleClippedPolygonArea( { vertex, polygon.radius }, current, vertices ) : 0.0;
}

double RoundedPolygonRectangleArea( const analyticFluidRoundedPolygon& polygon,
    double x0, double y0, double x1, double y1 )
{
    if( x1 <= polygon.minX || x0 >= polygon.maxX || y1 <= polygon.minY || y0 >= polygon.maxY )
        return 0.0;
    double area = ConvexPolygonRectangleArea( polygon.core, x0, y0, x1, y1 );
    for( int i = 0; i < polygon.core.count; ++i )
    {
        area += ConvexPolygonRectangleArea( polygon.strips[i], x0, y0, x1, y1 );
        area += RoundedCornerRectangleArea( polygon, i, x0, y0, x1, y1 );
    }
    return std::clamp( area, 0.0, ( x1 - x0 ) * ( y1 - y0 ) );
}

// 두 부분 교차 원의 셀 내 교집합. X 방향 단면의 위/아래 경계는
// 수평 셀 경계 또는 두 원호 중 하나이므로 변화 지점에서 분할해 정확히 적분한다.
double CirclePairRectangleIntersectionArea( const analyticFluidCircle& a,
    const analyticFluidCircle& b, double x0, double y0, double x1, double y1 )
{
    const double left = std::max( { x0, static_cast<double>( a.center.x ) - a.radius,
        static_cast<double>( b.center.x ) - b.radius } );
    const double right = std::min( { x1, static_cast<double>( a.center.x ) + a.radius,
        static_cast<double>( b.center.x ) + b.radius } );
    if( right <= left ) return 0.0;

    std::array<double, 24> cuts{};
    int count = 0;
    cuts[count++] = left;
    cuts[count++] = right;
    const auto addCut = [&]( double x )
    {
        if( std::isfinite( x ) && x > left && x < right ) cuts[count++] = x;
    };
    for( const auto& circle : { a, b } )
    {
        addCut( circle.center.x );
        for( const double y : { y0, y1 } )
        {
            const double dy = y - circle.center.y;
            if( std::abs( dy ) >= circle.radius ) continue;
            const double dx = std::sqrt( circle.radius * circle.radius - dy * dy );
            addCut( static_cast<double>( circle.center.x ) - dx );
            addCut( static_cast<double>( circle.center.x ) + dx );
        }
    }
    const double dx = static_cast<double>( b.center.x ) - a.center.x;
    const double dy = static_cast<double>( b.center.y ) - a.center.y;
    const double distance = std::hypot( dx, dy );
    if( distance <= 0.0 ) return 0.0;
    const double offset = ( a.radius * a.radius - b.radius * b.radius + distance * distance ) /
        ( 2.0 * distance );
    const double height = std::sqrt( std::max( 0.0, a.radius * a.radius - offset * offset ) );
    addCut( a.center.x + offset * dx / distance - height * dy / distance );
    addCut( a.center.x + offset * dx / distance + height * dy / distance );
    std::sort( cuts.begin(), cuts.begin() + count );

    const auto primitive = []( const analyticFluidCircle& circle, double x, bool top )
    {
        const double r = circle.radius;
        const double t = std::clamp( x - circle.center.x, -r, r );
        const double arc = 0.5 * ( t * std::sqrt( std::max( 0.0, r * r - t * t ) ) +
            r * r * std::asin( std::clamp( t / r, -1.0, 1.0 ) ) );
        return static_cast<double>( circle.center.y ) * x + ( top ? arc : -arc );
    };
    double area = 0.0;
    const std::array<analyticFluidCircle, 2> circles{ a, b };
    for( int i = 1; i < count; ++i )
    {
        const double xStart = cuts[i - 1], xEnd = cuts[i];
        if( xEnd <= xStart ) continue;
        const double mid = 0.5 * ( xStart + xEnd );
        double bottom = y0, top = y1;
        int bottomOwner = -1, topOwner = -1;
        for( int owner = 0; owner < 2; ++owner )
        {
            const auto& circle = circles[owner];
            const double delta = mid - circle.center.x;
            const double half = std::sqrt( std::max( 0.0,
                circle.radius * circle.radius - delta * delta ) );
            if( circle.center.y - half > bottom )
            {
                bottom = circle.center.y - half;
                bottomOwner = owner;
            }
            if( circle.center.y + half < top )
            {
                top = circle.center.y + half;
                topOwner = owner;
            }
        }
        if( top <= bottom ) continue;
        const double integralTop = topOwner < 0 ? y1 * ( xEnd - xStart )
            : primitive( circles[topOwner], xEnd, true ) -
                primitive( circles[topOwner], xStart, true );
        const double integralBottom = bottomOwner < 0 ? y0 * ( xEnd - xStart )
            : primitive( circles[bottomOwner], xEnd, false ) -
                primitive( circles[bottomOwner], xStart, false );
        area += integralTop - integralBottom;
    }
    return std::clamp( area, 0.0, ( x1 - x0 ) * ( y1 - y0 ) );
}

bool HasUnitFluidRotation( const rot2& rotation )
{
    const float squared = rotation.c * rotation.c + rotation.s * rotation.s;
    return std::isfinite( squared ) && std::abs( squared - 1.0f ) <= 1.0e-4f;
}

// AABB가 겹치는 경우에도 실제 convex Shape가 충분히 떨어져 있으면
// 분석적 합집합을 유지한다. 접촉/근접/비정상 회전/거리 계산 실패는 표본 경로로 복귀.
bool AreDisjointFluidShapes( const fluidShapeObstacle2& first,
    const fluidShapeObstacle2& second, const aabb2& firstBounds, const aabb2& secondBounds )
{
    if( !Overlaps( firstBounds, secondBounds ) ) return true;

    if( !HasUnitFluidRotation( first.transform.rotation ) ||
        !HasUnitFluidRotation( second.transform.rotation ) ) return false;

    distanceInput2 input{};
    input.proxyA = MakeShapeProxy( first.geometry );
    input.proxyB = MakeShapeProxy( second.geometry );
    input.transform = InverseMul( first.transform, second.transform );
    input.useRadii = true;
    simplexCache2 cache{};
    const distanceOutput2 distance = ShapeDistance( input, cache );

    // Float GJK의 세계 좌표 오차를 감안해 작은 간격은 겹침으로 취급한다.
    const float scale = std::max( { 1.0f,
        std::abs( firstBounds.min.x ), std::abs( firstBounds.min.y ),
        std::abs( firstBounds.max.x ), std::abs( firstBounds.max.y ),
        std::abs( secondBounds.min.x ), std::abs( secondBounds.min.y ),
        std::abs( secondBounds.max.x ), std::abs( secondBounds.max.y ) } );
    const float tolerance = 32.0f * std::numeric_limits<float>::epsilon() * scale;
    return std::isfinite( distance.distance ) && distance.iterations < 20 &&
        distance.distance > tolerance;
}

// Convex Shape끼리 실제로 분리돼야 개별 분석적 면적/face 구간을 더할 수 있다.
struct analyticDisjointShape2
{
    std::variant<analyticFluidCircle, analyticFluidPolygon,
        analyticFluidCapsule, analyticFluidRoundedPolygon> geometry{};
    aabb2 bounds{};
    bool occluded = false;
    bool partialCircleOverlap = false;
};

// 두 원이 엄격히 포함 관계라면 작은 원은 외부 유체와 접하는 경계가 없다.
// 접선 접촉 또는 비정상 회전은 분석적 소유자 판정을 하지 않는다.
int FindOccludedCircle( const analyticDisjointShape2& first,
    const analyticDisjointShape2& second, const fluidShapeObstacle2& firstBody,
    const fluidShapeObstacle2& secondBody )
{
    const auto* a = std::get_if<analyticFluidCircle>( &first.geometry );
    const auto* b = std::get_if<analyticFluidCircle>( &second.geometry );
    if( !a || !b || !HasUnitFluidRotation( firstBody.transform.rotation ) ||
        !HasUnitFluidRotation( secondBody.transform.rotation ) ) return -1;

    const double centerDistance = std::hypot(
        static_cast<double>( a->center.x ) - b->center.x,
        static_cast<double>( a->center.y ) - b->center.y );
    const double scale = std::max( { 1.0, a->radius, b->radius,
        std::abs( static_cast<double>( a->center.x ) ),
        std::abs( static_cast<double>( a->center.y ) ),
        std::abs( static_cast<double>( b->center.x ) ),
        std::abs( static_cast<double>( b->center.y ) ) } );
    const double tolerance = 32.0 * std::numeric_limits<float>::epsilon() * scale;
    if( centerDistance + b->radius + tolerance < a->radius ) return 1;
    if( centerDistance + a->radius + tolerance < b->radius ) return 0;
    return -1;
}

bool HasPartialCircleOverlap( const analyticDisjointShape2& first,
    const analyticDisjointShape2& second, const fluidShapeObstacle2& firstBody,
    const fluidShapeObstacle2& secondBody )
{
    const auto* a = std::get_if<analyticFluidCircle>( &first.geometry );
    const auto* b = std::get_if<analyticFluidCircle>( &second.geometry );
    if( !a || !b || !HasUnitFluidRotation( firstBody.transform.rotation ) ||
        !HasUnitFluidRotation( secondBody.transform.rotation ) ) return false;

    const double distance = std::hypot( static_cast<double>( a->center.x ) - b->center.x,
        static_cast<double>( a->center.y ) - b->center.y );
    const double scale = std::max( { 1.0, a->radius, b->radius,
        std::abs( static_cast<double>( a->center.x ) ),
        std::abs( static_cast<double>( a->center.y ) ),
        std::abs( static_cast<double>( b->center.x ) ),
        std::abs( static_cast<double>( b->center.y ) ) } );
    const double tolerance = 32.0 * std::numeric_limits<float>::epsilon() * scale;
    return distance > std::abs( a->radius - b->radius ) + tolerance &&
        distance + tolerance < a->radius + b->radius;
}

std::vector<analyticDisjointShape2> BuildDisjointAnalyticShapes(
    std::span<const fluidShapeObstacle2> obstacles )
{
    if( obstacles.size() < 2 ) return {};
    std::vector<analyticDisjointShape2> shapes;
    shapes.reserve( obstacles.size() );
    for( const fluidShapeObstacle2& obstacle : obstacles )
    {
        if( !IsValidFluidObstacle( obstacle ) ) return {};
        const auto single = std::span<const fluidShapeObstacle2>( &obstacle, 1 );
        analyticDisjointShape2 analytic{};
        analyticFluidCircle circle{};
        analyticFluidCapsule capsule{};
        analyticFluidPolygon polygon{};
        analyticFluidRoundedPolygon rounded{};
        if( TryGetAnalyticCircle( single, circle ) ) analytic.geometry = circle;
        else if( TryGetAnalyticCapsule( single, capsule ) ) analytic.geometry = capsule;
        else if( TryGetAnalyticPolygon( single, polygon ) ) analytic.geometry = polygon;
        else if( TryGetAnalyticRoundedPolygon( single, rounded ) ) analytic.geometry = rounded;
        else return {};

        analytic.bounds = ComputeShapeAABB( obstacle.geometry, obstacle.transform );
        if( !IsValidAABB( analytic.bounds ) ) return {};
        // 실제 분리된 Shape는 원래대로 합산한다. 두 원의 엄격한 포함 관계에
        // 한해서만 내부 원의 모든 면적/face 반력을 무시한다.
        for( std::size_t i = 0; i < shapes.size(); ++i )
        {
            if( AreDisjointFluidShapes( obstacles[i], obstacle,
                shapes[i].bounds, analytic.bounds ) ) continue;
            if( obstacles.size() != 2 || shapes.size() != 1 ) return {};
            const int occluded = FindOccludedCircle(
                shapes[i], analytic, obstacles[i], obstacle );
            if( occluded >= 0 )
            {
                if( occluded == 0 ) shapes[i].occluded = true;
                else analytic.occluded = true;
            }
            else if( HasPartialCircleOverlap( shapes[i], analytic, obstacles[i], obstacle ) )
            {
                shapes[i].partialCircleOverlap = true;
                analytic.partialCircleOverlap = true;
            }
            else return {};
        }
        shapes.push_back( std::move( analytic ) );
    }
    return shapes;
}

double DisjointShapeRectangleArea( const analyticDisjointShape2& shape,
    double x0, double y0, double x1, double y1 )
{
    if( shape.occluded ) return 0.0;
    if( x1 <= shape.bounds.min.x || x0 >= shape.bounds.max.x ||
        y1 <= shape.bounds.min.y || y0 >= shape.bounds.max.y ) return 0.0;
    return std::visit( [&]( const auto& geometry ) -> double
    {
        using T = std::remove_cvref_t<decltype( geometry )>;
        if constexpr( std::is_same_v<T, analyticFluidCircle> )
            return CircleRectangleArea( geometry, x0, y0, x1, y1 );
        else if constexpr( std::is_same_v<T, analyticFluidPolygon> )
            return ConvexPolygonRectangleArea( geometry, x0, y0, x1, y1 );
        else if constexpr( std::is_same_v<T, analyticFluidCapsule> )
            return CapsuleRectangleArea( geometry, x0, y0, x1, y1 );
        else return RoundedPolygonRectangleArea( geometry, x0, y0, x1, y1 );
    }, shape.geometry );
}

analyticCircleInterval ClipDisjointShapeFace( const analyticDisjointShape2& shape,
    bool vertical, double coordinate, double low, double high )
{
    if( shape.occluded ) return {};
    if( vertical
        ? ( coordinate < shape.bounds.min.x || coordinate > shape.bounds.max.x ||
            high <= shape.bounds.min.y || low >= shape.bounds.max.y )
        : ( coordinate < shape.bounds.min.y || coordinate > shape.bounds.max.y ||
            high <= shape.bounds.min.x || low >= shape.bounds.max.x ) ) return {};
    return std::visit( [&]( const auto& geometry ) -> analyticCircleInterval
    {
        using T = std::remove_cvref_t<decltype( geometry )>;
        if constexpr( std::is_same_v<T, analyticFluidCircle> )
            return ClipCircleFace( geometry, vertical, coordinate, low, high );
        else if constexpr( std::is_same_v<T, analyticFluidPolygon> )
            return ClipPolygonFace( geometry, vertical, coordinate, low, high );
        else if constexpr( std::is_same_v<T, analyticFluidCapsule> )
            return ClipCapsuleFace( geometry, vertical, coordinate, low, high );
        else return ClipRoundedPolygonFace( geometry, vertical, coordinate, low, high );
    }, shape.geometry );
}

// 기존 두 Shape의 face 교차 구간은 따로 더해도 된다. 두 원이 부분 교차하면
// 첫 owner가 겹친 부분을 소유하고, 둘째 owner의 노출 구간만 남긴다.
// 셀 면적, Schur B/B^T, 강체 속도 예측, swept flux 모두 이 분할을 사용한다.
void ForEachAnalyticFacePortion( const std::vector<analyticDisjointShape2>& shapes,
    bool vertical, double coordinate, double low, double high, auto&& callback )
{
    if( shapes.size() != 2 || !shapes[0].partialCircleOverlap )
    {
        for( std::size_t i = 0; i < shapes.size(); ++i )
        {
            const auto interval = ClipDisjointShapeFace( shapes[i], vertical, coordinate, low, high );
            if( interval.length > 0.0 ) callback( i, interval );
        }
        return;
    }
    const auto first = ClipDisjointShapeFace( shapes[0], vertical, coordinate, low, high );
    const auto second = ClipDisjointShapeFace( shapes[1], vertical, coordinate, low, high );
    if( first.length > 0.0 ) callback( 0, first );
    if( second.length <= 0.0 ) return;
    if( first.length <= 0.0 )
    {
        callback( 1, second );
        return;
    }
    const double a = first.midpoint - 0.5 * first.length;
    const double b = first.midpoint + 0.5 * first.length;
    const double c = second.midpoint - 0.5 * second.length;
    const double d = second.midpoint + 0.5 * second.length;
    const auto emit = [&]( double start, double end )
    {
        if( end > start ) callback( 1, analyticCircleInterval{ end - start, 0.5 * ( start + end ) } );
    };
    emit( c, std::min( d, a ) );
    emit( std::max( c, b ), d );
}

} // namespace

// Current-step immutable analytic geometry is shared by geometry, Schur
// assembly and post-pressure boundary synchronization. Swept intermediate
// poses intentionally build their own independent shape snapshots.
struct analyticDisjointCache2
{
    std::vector<analyticDisjointShape2> shapes;
};

fluidField2::fluidField2( int width, int height, float cellSize, vec2 origin )
    : width_( width ), height_( height ), cellSize_( cellSize ), origin_( origin )
{
    if( width < 2 || width > 256 || height < 2 || height > 256 ||
        !std::isfinite( cellSize ) || cellSize <= 0.0f || !IsFinite( origin ) ||
        !std::isfinite( origin.x + width * cellSize ) || !std::isfinite( origin.y + height * cellSize ) )
        throw std::invalid_argument( "fluidField2 requires a finite 2..256 grid, positive cell size and finite bounds" );

    u_.resize( static_cast<std::size_t>( width + 1 ) * height );
    v_.resize( static_cast<std::size_t>( width ) * ( height + 1 ) );
    nextU_.resize( u_.size() );
    nextV_.resize( v_.size() );
    tracerMass_.assign( static_cast<std::size_t>( width ) * height, 0.0 );
    nextTracerMass_.resize( tracerMass_.size() );
    tracerOutgoing_.resize( tracerMass_.size() );
    tracerUFlux_.resize( u_.size() );
    tracerVFlux_.resize( v_.size() );
    pressure_.resize( static_cast<std::size_t>( width ) * height );
    nextPressure_.resize( pressure_.size() );
    divergence_.resize( pressure_.size() );
    solidOwner_.assign( pressure_.size(), -1 );
    previousSolidOwner_.assign( pressure_.size(), -1 );
    cellFluidFraction_.assign( pressure_.size(), 1.0f );
    previousCellFluidFraction_.assign( pressure_.size(), 1.0f );
    uOpenFraction_.assign( u_.size(), 1.0f );
    vOpenFraction_.assign( v_.size(), 1.0f );
    uSolidVelocity_.assign( u_.size(), 0.0f );
    vSolidVelocity_.assign( v_.size(), 0.0f );
    pressureUSolidFlux_.assign( u_.size(), 0.0f );
    pressureVSolidFlux_.assign( v_.size(), 0.0f );
    cutCellVolumeRate_.assign( pressure_.size(), 0.0f );
    sweptUVolume_.assign( u_.size(), 0.0f );
    sweptVVolume_.assign( v_.size(), 0.0f );
    cutCellComponent_.assign( pressure_.size(), -1 );
    cutCellNearestFluid_.assign( pressure_.size(), -1 );
    cutCellQueue_.reserve( pressure_.size() );
    for( int y = 0; y < height_; ++y )
    {
        uOpenFraction_[uIndex( 0, y )] = 0.0f;
        uOpenFraction_[uIndex( width_, y )] = 0.0f;
    }
    for( int x = 0; x < width_; ++x )
    {
        vOpenFraction_[vIndex( x, 0 )] = 0.0f;
        vOpenFraction_[vIndex( x, height_ )] = 0.0f;
    }
}

void fluidField2::setCutCellSampleCount( int count ) noexcept
{
    const int samples = count >= 8 ? 8 : 4;
    if( cutCellSampleCount_ == samples ) return;
    cutCellSampleCount_ = samples;

    // 서로 다른 이산 면적의 연속 snapshot을 차분하면 가짜 체적 source가 발생한다.
    invalidateObstacleHistory();
}

void fluidField2::invalidateObstacleHistory() noexcept
{
    hasCutCellHistory_ = false;
    volumeSourceValid_ = false;
    previousDisjointAnalyticGeometry_ = false;
    previousContainedCircleIndex_ = -1;
    previousPartialCircleOverlap_ = false;
    analyticDisjointCache_.reset();
    previousObstacles_.clear();
    sweptFaceFlux_ = {};
    cutCellPressure_ = {};
}

void fluidField2::setAnalyticCircleCutCellEnabled( bool enabled ) noexcept
{
    if( analyticCircleCutCellEnabled_ == enabled ) return;
    analyticCircleCutCellEnabled_ = enabled;
    invalidateObstacleHistory();
}

void fluidField2::setAnalyticPolygonCutCellEnabled( bool enabled ) noexcept
{
    if( analyticPolygonCutCellEnabled_ == enabled ) return;
    analyticPolygonCutCellEnabled_ = enabled;
    invalidateObstacleHistory();
}

void fluidField2::setAnalyticCapsuleCutCellEnabled( bool enabled ) noexcept
{
    if( analyticCapsuleCutCellEnabled_ == enabled ) return;
    analyticCapsuleCutCellEnabled_ = enabled;
    invalidateObstacleHistory();
}

void fluidField2::setAnalyticRoundedPolygonCutCellEnabled( bool enabled ) noexcept
{
    if( analyticRoundedPolygonCutCellEnabled_ == enabled ) return;
    analyticRoundedPolygonCutCellEnabled_ = enabled;
    invalidateObstacleHistory();
}

void fluidField2::setDisjointAnalyticCutCellEnabled( bool enabled ) noexcept
{
    if( disjointAnalyticCutCellEnabled_ == enabled ) return;
    disjointAnalyticCutCellEnabled_ = enabled;
    invalidateObstacleHistory();
}

void fluidField2::clear() noexcept
{
    std::fill( u_.begin(), u_.end(), 0.0f );
    std::fill( v_.begin(), v_.end(), 0.0f );
    std::fill( nextU_.begin(), nextU_.end(), 0.0f );
    std::fill( nextV_.begin(), nextV_.end(), 0.0f );
    std::fill( tracerMass_.begin(), tracerMass_.end(), 0.0 );
    std::fill( nextTracerMass_.begin(), nextTracerMass_.end(), 0.0 );
    std::fill( tracerOutgoing_.begin(), tracerOutgoing_.end(), 0.0 );
    std::fill( tracerUFlux_.begin(), tracerUFlux_.end(), 0.0 );
    std::fill( tracerVFlux_.begin(), tracerVFlux_.end(), 0.0 );
    tracerEnabled_ = false;
    tracerStats_ = {};
    std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    std::fill( nextPressure_.begin(), nextPressure_.end(), 0.0f );
    std::fill( divergence_.begin(), divergence_.end(), 0.0f );
    std::fill( solidOwner_.begin(), solidOwner_.end(), -1 );
    std::fill( previousSolidOwner_.begin(), previousSolidOwner_.end(), -1 );
    std::fill( cellFluidFraction_.begin(), cellFluidFraction_.end(), 1.0f );
    std::fill( previousCellFluidFraction_.begin(), previousCellFluidFraction_.end(), 1.0f );
    analyticCoveredAreaScratch_.clear();
    std::fill( uSolidVelocity_.begin(), uSolidVelocity_.end(), 0.0f );
    std::fill( vSolidVelocity_.begin(), vSolidVelocity_.end(), 0.0f );
    std::fill( pressureUSolidFlux_.begin(), pressureUSolidFlux_.end(), 0.0f );
    std::fill( pressureVSolidFlux_.begin(), pressureVSolidFlux_.end(), 0.0f );
    std::fill( cutCellVolumeRate_.begin(), cutCellVolumeRate_.end(), 0.0f );
    std::fill( sweptUVolume_.begin(), sweptUVolume_.end(), 0.0f );
    std::fill( sweptVVolume_.begin(), sweptVVolume_.end(), 0.0f );
    analyticUBlockedFaces_.clear();
    analyticVBlockedFaces_.clear();
    analyticDisjointCache_.reset();
    analyticUFaceScratch_.clear();
    analyticVFaceScratch_.clear();
    previousObstacles_.clear();
    sweptFaceFlux_ = {};
    std::fill( cutCellComponent_.begin(), cutCellComponent_.end(), -1 );
    std::fill( cutCellNearestFluid_.begin(), cutCellNearestFluid_.end(), -1 );
    cutCellQueue_.clear();
    hasCutCellHistory_ = false;
    volumeSourceValid_ = false;
    previousDisjointAnalyticGeometry_ = false;
    previousContainedCircleIndex_ = -1;
    previousPartialCircleOverlap_ = false;
    std::fill( uOpenFraction_.begin(), uOpenFraction_.end(), 1.0f );
    std::fill( vOpenFraction_.begin(), vOpenFraction_.end(), 1.0f );
    for( int y = 0; y < height_; ++y )
    {
        uOpenFraction_[uIndex( 0, y )] = 0.0f;
        uOpenFraction_[uIndex( width_, y )] = 0.0f;
    }
    for( int x = 0; x < width_; ++x )
    {
        vOpenFraction_[vIndex( x, 0 )] = 0.0f;
        vOpenFraction_[vIndex( x, height_ )] = 0.0f;
    }
    solidCellCount_ = 0;
    solidTransition_ = {};
    cutCellTransition_ = {};
    cutCellPressure_ = {};
    advectionStats_ = {};
    pressureMomentum_ = {};
    stepMomentum_ = {};
    stepTiming_ = {};
    frozenPressureReplayReady_ = false;
    frozenPressureObstacles_.clear();
}

float fluidField2::tracerCellFraction( int x, int y ) const noexcept
{
    if( x < 0 || x >= width_ || y < 0 || y >= height_ ) return 0.0f;
    return cutCellProjectionEnabled_ ? cellFluidFraction_[cellIndex( x, y )] :
        ( isSolidCell( x, y ) ? 0.0f : 1.0f );
}

void fluidField2::setTracerConcentration( int x, int y, float concentration ) noexcept
{
    if( !std::isfinite( concentration ) || concentration < 0.0f ||
        x < 0 || x >= width_ || y < 0 || y >= height_ ) return;
    const float fraction = tracerCellFraction( x, y );
    if( fraction <= 0.0f && concentration > 0.0f ) return;
    const double mass = static_cast<double>( concentration ) * cellSize_ * cellSize_ * fraction;
    if( !std::isfinite( mass ) ) return;
    tracerMass_[cellIndex( x, y )] = mass;
    tracerEnabled_ = true;
}

float fluidField2::getTracerConcentration( int x, int y ) const noexcept
{
    const float fraction = tracerCellFraction( x, y );
    if( fraction <= 0.0f ) return 0.0f;
    return static_cast<float>( tracerMass_[cellIndex( x, y )] /
        ( static_cast<double>( cellSize_ ) * cellSize_ * fraction ) );
}

float fluidField2::getTracerMass() const noexcept
{
    double mass = 0.0;
    for( const double value : tracerMass_ ) mass += value;
    return static_cast<float>( mass );
}

vec2 fluidField2::getCellCenter( int x, int y ) const noexcept
{
    return origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) + 0.5f };
}

float fluidField2::sampleComponent( const std::vector<float>& values, int width, int height, float x, float y ) noexcept
{
    x = std::clamp( x, 0.0f, static_cast<float>( width - 1 ) );
    y = std::clamp( y, 0.0f, static_cast<float>( height - 1 ) );
    const int i = static_cast<int>( x );
    const int j = static_cast<int>( y );
    const int i1 = std::min( i + 1, width - 1 );
    const int j1 = std::min( j + 1, height - 1 );
    const float tx = x - i;
    const float ty = y - j;
    const float bottom = std::lerp( values[j * width + i], values[j * width + i1], tx );
    const float top = std::lerp( values[j1 * width + i], values[j1 * width + i1], tx );
    return std::lerp( bottom, top, ty );
}

vec2 fluidField2::sampleFrom( const std::vector<float>& u, const std::vector<float>& v, vec2 position ) const noexcept
{
    if( !IsFinite( position ) ) return {};
    const vec2 grid = ( position - origin_ ) / cellSize_;
    return {
        sampleComponent( u, width_ + 1, height_, grid.x, grid.y - 0.5f ),
        sampleComponent( v, width_, height_ + 1, grid.x - 0.5f, grid.y )
    };
}

vec2 fluidField2::sampleVelocity( vec2 position ) const noexcept
{
    // 바깥 영역에 유체가 있다고 가정하지 않음.
    if( !IsFinite( position ) || position.x < origin_.x || position.y < origin_.y ||
        position.x > origin_.x + width_ * cellSize_ || position.y > origin_.y + height_ * cellSize_ ) return {};
    return sampleFrom( u_, v_, position );
}

void fluidField2::addVelocity( vec2 position, vec2 velocityDelta, float radius )
{
    if( !IsFinite( position ) || !IsFinite( velocityDelta ) || !std::isfinite( radius ) || radius <= 0.0f ) return;

    // 중심에서 선형 감쇠하는 국소 속도 source. 경계 face는 no-through이므로 건드리지 않음.
    const float radiusSquared = radius * radius;
    if( !std::isfinite( radiusSquared ) ) return;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            const vec2 offset = face - position;
            const float distanceSquared = Dot( offset, offset );
            if( distanceSquared < radiusSquared )
                u_[uIndex( x, y )] += velocityDelta.x * ( 1.0f - distanceSquared / radiusSquared );
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            const vec2 offset = face - position;
            const float distanceSquared = Dot( offset, offset );
            if( distanceSquared < radiusSquared )
                v_[vIndex( x, y )] += velocityDelta.y * ( 1.0f - distanceSquared / radiusSquared );
        }
}

vec2 fluidField2::getFluidMomentum() const noexcept
{
    // 각 내부 fluid-fluid face의 dual volume은 h² * unitDepth.
    // Solid 경계 속도는 지정된 운동학 경계값이므로 유체 운동량 적분에서 제외.
    const double faceMass = static_cast<double>( cellSize_ ) * cellSize_;
    double x = 0.0;
    double y = 0.0;
    for( int j = 0; j < height_; ++j )
        for( int i = 1; i < width_; ++i )
            if( !isSolidCell( i - 1, j ) && !isSolidCell( i, j ) )
                x += u_[uIndex( i, j )];

    for( int j = 1; j < height_; ++j )
        for( int i = 0; i < width_; ++i )
            if( !isSolidCell( i, j - 1 ) && !isSolidCell( i, j ) )
                y += v_[vIndex( i, j )];

    return { static_cast<float>( x * faceMass ), static_cast<float>( y * faceMass ) };
}

double fluidField2::getFluidKineticEnergy() const noexcept
{
    // fluid-fluid face에만 정의한 MAC dual kinetic energy. 전체 유체 에너지는 아님.
    double squared = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( !isSolidCell( x - 1, y ) && !isSolidCell( x, y ) )
            {
                const double speed = u_[uIndex( x, y )];
                squared += speed * speed;
            }
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( !isSolidCell( x, y - 1 ) && !isSolidCell( x, y ) )
            {
                const double speed = v_[vIndex( x, y )];
                squared += speed * speed;
            }
    return 0.5 * static_cast<double>( cellSize_ ) * cellSize_ * squared;
}

vec2 fluidField2::applyMomentumImpulse( vec2 position, vec2 impulse, float radius )
{
    if( !IsFinite( position ) || !IsFinite( impulse ) || !std::isfinite( radius ) || radius <= 0.0f )
        return {};

    const float radiusSquared = radius * radius;
    const float faceMass = cellSize_ * cellSize_; // 1 kg/m³ * h² * 1 m
    if( !std::isfinite( radiusSquared ) || !std::isfinite( faceMass ) || faceMass <= 0.0f )
        return {};

    // X/Y 성분은 서로 다른 위치의 MAC face에 존재하므로 독립 정규화한다.
    // 압력 보정/벽 제약은 이 교환 다음 Step에 따로 작용한다.
    const auto weight = [&]( vec2 face )
    {
        const vec2 delta = face - position;
        return std::max( 0.0f, 1.0f - Dot( delta, delta ) / radiusSquared );
    };

    double sumU = 0.0;
    double sumV = 0.0;
    for( int j = 0; j < height_; ++j )
        for( int i = 1; i < width_; ++i )
        {
            if( isSolidCell( i - 1, j ) || isSolidCell( i, j ) ) continue;
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ), static_cast<float>( j ) + 0.5f };
            sumU += weight( face );
        }

    for( int j = 1; j < height_; ++j )
        for( int i = 0; i < width_; ++i )
        {
            if( isSolidCell( i, j - 1 ) || isSolidCell( i, j ) ) continue;
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ) + 0.5f, static_cast<float>( j ) };
            sumV += weight( face );
        }

    // 너무 큰 impulse/너무 작은 cell을 다룰 때 float face 속도 overflow 방지.
    const double xScale = sumU > 0.0 ? static_cast<double>( impulse.x ) / ( faceMass * sumU ) : 0.0;
    const double yScale = sumV > 0.0 ? static_cast<double>( impulse.y ) / ( faceMass * sumV ) : 0.0;
    if( !std::isfinite( xScale ) || !std::isfinite( yScale ) ||
        std::abs( xScale ) > std::numeric_limits<float>::max() ||
        std::abs( yScale ) > std::numeric_limits<float>::max() ) return {};

    double actualX = 0.0;
    double actualY = 0.0;
    if( impulse.x != 0.0f && sumU > 0.0 )
    {
        for( int j = 0; j < height_; ++j )
            for( int i = 1; i < width_; ++i )
            {
                if( isSolidCell( i - 1, j ) || isSolidCell( i, j ) ) continue;
                const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ), static_cast<float>( j ) + 0.5f };
                const float w = weight( face );
                if( w <= 0.0f ) continue;
                const int index = uIndex( i, j );
                const float old = u_[index];
                const float next = old + static_cast<float>( w * xScale );
                if( !std::isfinite( next ) ) continue;
                u_[index] = next;
                actualX += static_cast<double>( next - old ) * faceMass;
            }
    }

    if( impulse.y != 0.0f && sumV > 0.0 )
    {
        for( int j = 1; j < height_; ++j )
            for( int i = 0; i < width_; ++i )
            {
                if( isSolidCell( i, j - 1 ) || isSolidCell( i, j ) ) continue;
                const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ) + 0.5f, static_cast<float>( j ) };
                const float w = weight( face );
                if( w <= 0.0f ) continue;
                const int index = vIndex( i, j );
                const float old = v_[index];
                const float next = old + static_cast<float>( w * yScale );
                if( !std::isfinite( next ) ) continue;
                v_[index] = next;
                actualY += static_cast<double>( next - old ) * faceMass;
            }
    }
    return { static_cast<float>( actualX ), static_cast<float>( actualY ) };
}

void fluidField2::step( float timeStep, int pressureIterations )
{
    step( timeStep, std::span<const fluidShapeObstacle2>{}, std::span<fluidObstacleForce2>{}, pressureIterations );
}

void fluidField2::updateObstacles( std::span<const fluidShapeObstacle2> obstacles )
{
    std::copy( solidOwner_.begin(), solidOwner_.end(), previousSolidOwner_.begin() );
    solidTransition_ = {};
    std::fill( solidOwner_.begin(), solidOwner_.end(), -1 );
    solidCellCount_ = 0;
    for( std::size_t index = 0; index < obstacles.size(); ++index )
    {
        const fluidShapeObstacle2& obstacle = obstacles[index];
        if( !IsValidFluidObstacle( obstacle ) ) continue;

        // Cell-center occupancy는 도형의 보수적 AABB 범위에서만 검사한다.
        // 멀리 떨어진 Static 장애물이 늘어도 격자 전체를 도형마다 순회하지 않는다.
        const double radius = FluidObstacleBoundRadius( obstacle );
        const auto boundIndex = [&]( double coordinate, float origin, int limit, bool end )
        {
            const double cell = ( coordinate - origin ) / cellSize_ - 0.5;
            const double inclusive = end ? std::ceil( cell ) + 2.0 : std::floor( cell ) - 1.0;
            return static_cast<int>( std::clamp( inclusive, 0.0, static_cast<double>( limit ) ) );
        };
        const double x = obstacle.transform.position.x;
        const double y = obstacle.transform.position.y;
        const int minX = boundIndex( x - radius, origin_.x, width_, false );
        const int maxX = boundIndex( x + radius, origin_.x, width_, true );
        const int minY = boundIndex( y - radius, origin_.y, height_, false );
        const int maxY = boundIndex( y + radius, origin_.y, height_, true );

        // 최초의 원이 cell을 소유함. 겹치는 도형의 우선순위는 유지한다.
        for( int cellY = minY; cellY < maxY; ++cellY )
            for( int cellX = minX; cellX < maxX; ++cellX )
            {
                const int cell = cellIndex( cellX, cellY );
                if( solidOwner_[cell] >= 0 ) continue;
                if( ContainsFluidObstacle( obstacle, getCellCenter( cellX, cellY ) ) )
                {
                    solidOwner_[cell] = static_cast<int>( index );
                    ++solidCellCount_;
                }
            }
    }

    for( std::size_t cell = 0; cell < solidOwner_.size(); ++cell )
    {
        if( previousSolidOwner_[cell] >= 0 && solidOwner_[cell] < 0 ) ++solidTransition_.exposedCells;
        if( previousSolidOwner_[cell] < 0 && solidOwner_[cell] >= 0 ) ++solidTransition_.coveredCells;
    }
    solidTransition_.fluidVolumeDelta =
        static_cast<float>( solidTransition_.exposedCells - solidTransition_.coveredCells ) * cellSize_ * cellSize_;
}

float fluidField2::getCellFluidFraction( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y >= 0 && y < height_ ? cellFluidFraction_[cellIndex( x, y )] : 0.0f;
}

float fluidField2::getUFaceOpenFraction( int x, int y ) const noexcept
{
    return x > 0 && x < width_ && y >= 0 && y < height_ ? uOpenFraction_[uIndex( x, y )] : 0.0f;
}

float fluidField2::getVFaceOpenFraction( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y > 0 && y < height_ ? vOpenFraction_[vIndex( x, y )] : 0.0f;
}

float fluidField2::getUFaceSolidVelocity( int x, int y ) const noexcept
{
    return x > 0 && x < width_ && y >= 0 && y < height_
        ? uSolidVelocity_[uIndex( x, y )] : 0.0f;
}

float fluidField2::getVFaceSolidVelocity( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y > 0 && y < height_
        ? vSolidVelocity_[vIndex( x, y )] : 0.0f;
}

void fluidField2::updateCutCellGeometry( std::span<const fluidShapeObstacle2> obstacles )
{
    // 이진 cell-center mask와 별도로 NxN sub-cell / face당 N점 표본을 계산.
    // 선택적 가중 압력 경로에서 면 개방율과 강체 속도를 직접 사용한다.
    // Disjoint analytic and sampled union use different area discretizations.
    // A transition between them is a new geometry snapshot, not a swept ΔV.
    // Rebuild for this World snapshot. Swept intermediate snapshots differ.
    analyticDisjointCache_.reset();
    auto disjointShapes = disjointAnalyticCutCellEnabled_
        ? BuildDisjointAnalyticShapes( obstacles ) : std::vector<analyticDisjointShape2>{};
    const bool disjointGeometry = !disjointShapes.empty();
    analyticUBlockedFaces_.clear();
    analyticVBlockedFaces_.clear();
    int containedCircle = -1;
    for( std::size_t i = 0; i < disjointShapes.size(); ++i )
        if( disjointShapes[i].occluded ) containedCircle = static_cast<int>( i );
    const bool partialCircleOverlap = disjointShapes.size() == 2 &&
        disjointShapes[0].partialCircleOverlap;
    if( hasCutCellHistory_ && ( disjointGeometry != previousDisjointAnalyticGeometry_ ||
        containedCircle != previousContainedCircleIndex_ ||
        partialCircleOverlap != previousPartialCircleOverlap_ ) )
        invalidateObstacleHistory();
    previousDisjointAnalyticGeometry_ = disjointGeometry;
    previousContainedCircleIndex_ = containedCircle;
    previousPartialCircleOverlap_ = partialCircleOverlap;
    const int sampleCount = cutCellSampleCount_;
    const float inverseCount = 1.0f / sampleCount;
    const float inverseCellSamples = inverseCount * inverseCount;
    std::copy( cellFluidFraction_.begin(), cellFluidFraction_.end(), previousCellFluidFraction_.begin() );
    volumeSourceValid_ = hasCutCellHistory_;
    hasCutCellHistory_ = true;
    cutCellTransition_ = {};
    cutCellTransition_.usedDisjointAnalyticGeometry = disjointGeometry;
    cutCellTransition_.disjointAnalyticShapes =
        disjointGeometry ? static_cast<int>( disjointShapes.size() ) : 0;
    cutCellTransition_.containedAnalyticShapes = containedCircle >= 0 ? 1 : 0;
    cutCellTransition_.usedPartialCircleAnalyticUnion = partialCircleOverlap;
    std::fill( uSolidVelocity_.begin(), uSolidVelocity_.end(), 0.0f );
    std::fill( vSolidVelocity_.begin(), vSolidVelocity_.end(), 0.0f );

    double previousVolume = 0.0;
    for( const float fraction : previousCellFluidFraction_ ) previousVolume += fraction;
    cutCellTransition_.previousFluidVolume = static_cast<float>(
        previousVolume * static_cast<double>( cellSize_ ) * cellSize_ );

    if( obstacles.empty() )
    {
        std::fill( cellFluidFraction_.begin(), cellFluidFraction_.end(), 1.0f );
        std::fill( uOpenFraction_.begin(), uOpenFraction_.end(), 1.0f );
        std::fill( vOpenFraction_.begin(), vOpenFraction_.end(), 1.0f );
    }
    else
    {
        if( disjointGeometry )
        {
            const double h = cellSize_;
            const double inverseArea = 1.0 / ( h * h );
            if( analyticCutCellBoundedCellAreasEnabled_ )
            {
                // Accumulate each shape into only the cells that overlap its
                // world-space AABB. Shape-major order preserves each cell's
                // original addition order and double-precision area.
                analyticCoveredAreaScratch_.assign( pressure_.size(), 0.0 );
                const auto clampIndex = []( double index, int maximum )
                {
                    return static_cast<int>( std::clamp( index, 0.0, static_cast<double>( maximum ) ) );
                };
                for( const auto& shape : disjointShapes )
                {
                    if( shape.occluded ) continue;
                    // Expand by one cell to conservatively include grid-edge
                    // rounding; the exact rectangle/shape intersection still
                    // decides whether a covered area is nonzero.
                    const int minX = clampIndex( std::floor(
                        ( static_cast<double>( shape.bounds.min.x ) - origin_.x ) / h ) - 1.0, width_ - 1 );
                    const int maxX = clampIndex( std::ceil(
                        ( static_cast<double>( shape.bounds.max.x ) - origin_.x ) / h ) + 1.0, width_ - 1 );
                    const int minY = clampIndex( std::floor(
                        ( static_cast<double>( shape.bounds.min.y ) - origin_.y ) / h ) - 1.0, height_ - 1 );
                    const int maxY = clampIndex( std::ceil(
                        ( static_cast<double>( shape.bounds.max.y ) - origin_.y ) / h ) + 1.0, height_ - 1 );
                    for( int y = minY; y <= maxY; ++y )
                        for( int x = minX; x <= maxX; ++x )
                        {
                            const double left = origin_.x + x * h;
                            const double bottom = origin_.y + y * h;
                            analyticCoveredAreaScratch_[cellIndex( x, y )] +=
                                DisjointShapeRectangleArea( shape, left, bottom, left + h, bottom + h );
                        }
                }
            }
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int cell = cellIndex( x, y );
                    const double left = origin_.x + x * h;
                    const double bottom = origin_.y + y * h;
                    double covered = 0.0;
                    if( analyticCutCellBoundedCellAreasEnabled_ )
                        covered = analyticCoveredAreaScratch_[cell];
                    else
                        for( const auto& shape : disjointShapes )
                            covered += DisjointShapeRectangleArea( shape, left, bottom, left + h, bottom + h );
                    if( partialCircleOverlap )
                        covered -= CirclePairRectangleIntersectionArea(
                            std::get<analyticFluidCircle>( disjointShapes[0].geometry ),
                            std::get<analyticFluidCircle>( disjointShapes[1].geometry ),
                            left, bottom, left + h, bottom + h );
                    cellFluidFraction_[cell] = static_cast<float>(
                        std::clamp( 1.0 - covered * inverseArea, 0.0, 1.0 ) );
                }
            // For three or more disjoint shapes, the blocked intervals are
            // independent. Accumulate them shape-major in their AABB face
            // ranges, retaining the original per-face shape addition order.
            // Two-circle overlap/containment continues through the reference
            // clipping path and its explicit ownership rules.
            const bool boundedFaceClipping = analyticCutCellBoundedFaceClippingEnabled_ &&
                disjointShapes.size() > 2;
            if( boundedFaceClipping )
            {
                analyticUFaceScratch_.assign( u_.size(), {} );
                analyticVFaceScratch_.assign( v_.size(), {} );
                const auto clampFace = []( double index, int minimum, int maximum )
                {
                    return static_cast<int>( std::clamp( index,
                        static_cast<double>( minimum ), static_cast<double>( maximum ) ) );
                };
                for( std::size_t i = 0; i < disjointShapes.size(); ++i )
                {
                    const auto& shape = disjointShapes[i];
                    if( shape.occluded ) continue;
                    const auto accumulateFaces = [&]( bool vertical )
                    {
                        const int minX = clampFace( std::floor(
                            ( static_cast<double>( shape.bounds.min.x ) - origin_.x ) / h ) - 1.0,
                            vertical ? 1 : 0, width_ - 1 );
                        const int maxX = clampFace( std::ceil(
                            ( static_cast<double>( shape.bounds.max.x ) - origin_.x ) / h ) + 1.0,
                            vertical ? 1 : 0, width_ - 1 );
                        const int minY = clampFace( std::floor(
                            ( static_cast<double>( shape.bounds.min.y ) - origin_.y ) / h ) - 1.0,
                            vertical ? 0 : 1, height_ - 1 );
                        const int maxY = clampFace( std::ceil(
                            ( static_cast<double>( shape.bounds.max.y ) - origin_.y ) / h ) + 1.0,
                            vertical ? 0 : 1, height_ - 1 );
                        for( int y = minY; y <= maxY; ++y )
                            for( int x = minX; x <= maxX; ++x )
                            {
                                const double normal = vertical ? origin_.x + x * h : origin_.y + y * h;
                                const double low = vertical ? origin_.y + y * h : origin_.x + x * h;
                                const auto clipped = ClipDisjointShapeFace(
                                    shape, vertical, normal, low, low + h );
                                if( clipped.length <= 0.0 ) continue;
                                const vec2 point = vertical
                                    ? vec2{ static_cast<float>( normal ), static_cast<float>( clipped.midpoint ) }
                                    : vec2{ static_cast<float>( clipped.midpoint ), static_cast<float>( normal ) };
                                const auto& body = obstacles[i];
                                const vec2 velocity = body.linearVelocity +
                                    Cross( body.angularVelocity, point - body.center );
                                auto& face = vertical ? analyticUFaceScratch_[uIndex( x, y )] :
                                    analyticVFaceScratch_[vIndex( x, y )];
                                face.blockedLength += clipped.length;
                                face.velocitySum += clipped.length * ( vertical ? velocity.x : velocity.y );
                            }
                    };
                    accumulateFaces( true );
                    accumulateFaces( false );
                }
            }
            const auto updateFace = [&]( bool vertical, int x, int y )
            {
                double blockedLength = 0.0;
                double velocitySum = 0.0;
                if( boundedFaceClipping )
                {
                    const auto& face = vertical ? analyticUFaceScratch_[uIndex( x, y )] :
                        analyticVFaceScratch_[vIndex( x, y )];
                    blockedLength = face.blockedLength;
                    velocitySum = face.velocitySum;
                }
                else
                {
                    const double normal = vertical ? origin_.x + x * h : origin_.y + y * h;
                    const double low = vertical ? origin_.y + y * h : origin_.x + x * h;
                    ForEachAnalyticFacePortion( disjointShapes, vertical, normal, low, low + h,
                        [&]( std::size_t i, analyticCircleInterval interval )
                    {
                        const vec2 point = vertical
                            ? vec2{ static_cast<float>( normal ), static_cast<float>( interval.midpoint ) }
                            : vec2{ static_cast<float>( interval.midpoint ), static_cast<float>( normal ) };
                        const auto& body = obstacles[i];
                        const vec2 velocity = body.linearVelocity +
                            Cross( body.angularVelocity, point - body.center );
                        blockedLength += interval.length;
                        velocitySum += interval.length * ( vertical ? velocity.x : velocity.y );
                    } );
                }
                const int face = vertical ? uIndex( x, y ) : vIndex( x, y );
                // Retain every positive analytic intersection, including those
                // too narrow to change the stored float aperture from 1.0.
                if( blockedLength > 0.0 )
                {
                    if( vertical ) analyticUBlockedFaces_.push_back( face );
                    else analyticVBlockedFaces_.push_back( face );
                }
                const float open = static_cast<float>(
                    std::clamp( 1.0 - blockedLength / h, 0.0, 1.0 ) );
                const float speed = blockedLength > 0.0
                    ? static_cast<float>( velocitySum / blockedLength ) : 0.0f;
                if( vertical )
                {
                    uOpenFraction_[face] = open;
                    uSolidVelocity_[face] = speed;
                }
                else
                {
                    vOpenFraction_[face] = open;
                    vSolidVelocity_[face] = speed;
                }
            };
            for( int y = 0; y < height_; ++y )
                for( int x = 1; x < width_; ++x ) updateFace( true, x, y );
            for( int y = 1; y < height_; ++y )
                for( int x = 0; x < width_; ++x ) updateFace( false, x, y );
        }
        else
        {
        analyticFluidCircle circle{};
        analyticFluidPolygon polygon{};
        analyticFluidCapsule capsule{};
        analyticFluidRoundedPolygon rounded{};
        const bool circleGeometry = analyticCircleCutCellEnabled_ &&
            TryGetAnalyticCircle( obstacles, circle );
        const bool polygonGeometry = !circleGeometry && analyticPolygonCutCellEnabled_ &&
            TryGetAnalyticPolygon( obstacles, polygon );
        const bool capsuleGeometry = !circleGeometry && !polygonGeometry &&
            analyticCapsuleCutCellEnabled_ && TryGetAnalyticCapsule( obstacles, capsule );
        const bool roundedGeometry = !circleGeometry && !polygonGeometry && !capsuleGeometry &&
            analyticRoundedPolygonCutCellEnabled_ && TryGetAnalyticRoundedPolygon( obstacles, rounded );
        if( circleGeometry || polygonGeometry || capsuleGeometry || roundedGeometry )
        {
            const double h = cellSize_;
            const double inverseArea = 1.0 / ( h * h );
            const fluidShapeObstacle2& shape = obstacles.front();
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const double left = origin_.x + x * h;
                    const double bottom = origin_.y + y * h;
                    const double covered = circleGeometry
                        ? CircleRectangleArea( circle, left, bottom, left + h, bottom + h )
                        : polygonGeometry
                            ? ConvexPolygonRectangleArea( polygon, left, bottom, left + h, bottom + h )
                            : capsuleGeometry
                                ? CapsuleRectangleArea( capsule, left, bottom, left + h, bottom + h )
                                : RoundedPolygonRectangleArea( rounded, left, bottom, left + h, bottom + h );
                    cellFluidFraction_[cellIndex( x, y )] =
                        static_cast<float>( std::clamp( 1.0 - covered * inverseArea, 0.0, 1.0 ) );
                }
            for( int y = 0; y < height_; ++y )
                for( int x = 1; x < width_; ++x )
                {
                    const double normal = origin_.x + x * h;
                    const double low = origin_.y + y * h;
                    const auto interval = circleGeometry
                        ? ClipCircleFace( circle, true, normal, low, low + h )
                        : polygonGeometry
                            ? ClipPolygonFace( polygon, true, normal, low, low + h )
                            : capsuleGeometry
                                ? ClipCapsuleFace( capsule, true, normal, low, low + h )
                                : ClipRoundedPolygonFace( rounded, true, normal, low, low + h );
                    const int face = uIndex( x, y );
                    uOpenFraction_[face] = static_cast<float>(
                        std::clamp( 1.0 - interval.length / h, 0.0, 1.0 ) );
                    if( interval.length > 0.0 )
                    {
                        const vec2 point{ static_cast<float>( normal ),
                            static_cast<float>( interval.midpoint ) };
                        uSolidVelocity_[face] = ( shape.linearVelocity +
                            Cross( shape.angularVelocity, point - shape.center ) ).x;
                    }
                }
            for( int y = 1; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const double normal = origin_.y + y * h;
                    const double low = origin_.x + x * h;
                    const auto interval = circleGeometry
                        ? ClipCircleFace( circle, false, normal, low, low + h )
                        : polygonGeometry
                            ? ClipPolygonFace( polygon, false, normal, low, low + h )
                            : capsuleGeometry
                                ? ClipCapsuleFace( capsule, false, normal, low, low + h )
                                : ClipRoundedPolygonFace( rounded, false, normal, low, low + h );
                    const int face = vIndex( x, y );
                    vOpenFraction_[face] = static_cast<float>(
                        std::clamp( 1.0 - interval.length / h, 0.0, 1.0 ) );
                    if( interval.length > 0.0 )
                    {
                        const vec2 point{ static_cast<float>( interval.midpoint ),
                            static_cast<float>( normal ) };
                        vSolidVelocity_[face] = ( shape.linearVelocity +
                            Cross( shape.angularVelocity, point - shape.center ) ).y;
                    }
                }
        }
        else
        {
        // 서브셀 16표본과 MAC face 4표본 각각에 대해 Shape Contains를
        // 호출하기 전에 도형의 보수적 world 반경으로 대부분을 걸러낸다.
        std::vector<double> bounds( obstacles.size(), -1.0 );
        for( std::size_t i = 0; i < obstacles.size(); ++i )
            if( IsValidFluidObstacle( obstacles[i] ) )
                bounds[i] = FluidObstacleBoundRadius( obstacles[i] );
        const auto firstOwner = [&]( vec2 point )
        {
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                const double radius = bounds[i];
                if( radius < 0.0 ) continue;
                const vec2 center = obstacles[i].transform.position;
                if( std::abs( static_cast<double>( point.x ) - center.x ) > radius ||
                    std::abs( static_cast<double>( point.y ) - center.y ) > radius ) continue;
                if( ContainsFluidObstacle( obstacles[i], point ) ) return static_cast<int>( i );
            }
            return -1;
        };
        const auto isOccupied = [&]( vec2 point ) { return firstOwner( point ) >= 0; };

        // 대부분의 셀/face는 장애물 바깥이다. 보수적 원형 경계의 AABB와
        // 대상 셀/면 구간이 만나지 않으면 공간 N×N 표본 검사를 생략한다.
        // 반경 여유는 경계 부동소수점 오차로 인한 누락을 막는다.
        const auto mayOverlap = [&]( vec2 center, vec2 halfSize )
        {
            constexpr double tolerance = 1.0e-5;
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                if( bounds[i] < 0.0 ) continue;
                const vec2 body = obstacles[i].transform.position;
                if( std::abs( static_cast<double>( center.x ) - body.x ) >
                        bounds[i] + halfSize.x + tolerance ||
                    std::abs( static_cast<double>( center.y ) - body.y ) >
                        bounds[i] + halfSize.y + tolerance ) continue;
                return true;
            }
            return false;
        };

        for( int y = 0; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const vec2 center = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ) + 0.5f, static_cast<float>( y ) + 0.5f };
                if( !mayOverlap( center, { 0.5f * cellSize_, 0.5f * cellSize_ } ) )
                {
                    cellFluidFraction_[cellIndex( x, y )] = 1.0f;
                    continue;
                }
                int covered = 0;
                for( int sy = 0; sy < sampleCount; ++sy )
                    for( int sx = 0; sx < sampleCount; ++sx )
                    {
                        const vec2 point = origin_ + cellSize_ * vec2{
                            static_cast<float>( x ) + ( sx + 0.5f ) * inverseCount,
                            static_cast<float>( y ) + ( sy + 0.5f ) * inverseCount };
                        if( isOccupied( point ) ) ++covered;
                    }
                cellFluidFraction_[cellIndex( x, y )] = 1.0f - covered * inverseCellSamples;
            }

        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const vec2 center = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
                if( !mayOverlap( center, { 0.0f, 0.5f * cellSize_ } ) )
                {
                    uOpenFraction_[uIndex( x, y )] = 1.0f;
                    continue;
                }
                int blocked = 0;
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) * inverseCount };
                    const int owner = firstOwner( point );
                    if( owner >= 0 )
                    {
                        ++blocked;
                        const fluidShapeObstacle2& shape = obstacles[static_cast<std::size_t>( owner )];
                        uSolidVelocity_[uIndex( x, y )] +=
                            ( shape.linearVelocity + Cross( shape.angularVelocity, point - shape.center ) ).x;
                    }
                }
                uOpenFraction_[uIndex( x, y )] = 1.0f - blocked * inverseCount;
                if( blocked > 0 ) uSolidVelocity_[uIndex( x, y )] /= blocked;
            }

        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const vec2 center = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
                if( !mayOverlap( center, { 0.5f * cellSize_, 0.0f } ) )
                {
                    vOpenFraction_[vIndex( x, y )] = 1.0f;
                    continue;
                }
                int blocked = 0;
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ) + ( sample + 0.5f ) * inverseCount, static_cast<float>( y ) };
                    const int owner = firstOwner( point );
                    if( owner >= 0 )
                    {
                        ++blocked;
                        const fluidShapeObstacle2& shape = obstacles[static_cast<std::size_t>( owner )];
                        vSolidVelocity_[vIndex( x, y )] +=
                            ( shape.linearVelocity + Cross( shape.angularVelocity, point - shape.center ) ).y;
                    }
                }
                vOpenFraction_[vIndex( x, y )] = 1.0f - blocked * inverseCount;
                if( blocked > 0 ) vSolidVelocity_[vIndex( x, y )] /= blocked;
            }
        }
        }
    }

    // Exterior faces always remain closed even in a geometrically empty domain.
    for( int y = 0; y < height_; ++y )
    {
        uOpenFraction_[uIndex( 0, y )] = 0.0f;
        uOpenFraction_[uIndex( width_, y )] = 0.0f;
    }
    for( int x = 0; x < width_; ++x )
    {
        vOpenFraction_[vIndex( x, 0 )] = 0.0f;
        vOpenFraction_[vIndex( x, height_ )] = 0.0f;
    }

    double current = 0.0;
    double covered = 0.0;
    double released = 0.0;
    for( std::size_t cell = 0; cell < cellFluidFraction_.size(); ++cell )
    {
        const float old = previousCellFluidFraction_[cell];
        const float now = cellFluidFraction_[cell];
        current += now;
        covered += std::max( 0.0f, old - now );
        released += std::max( 0.0f, now - old );
        if( now > 0.0f && now < 1.0f ) ++cutCellTransition_.partialCells;
    }
    for( const float open : uOpenFraction_ )
        if( open > 0.0f && open < 1.0f ) ++cutCellTransition_.partialUFaces;
    for( const float open : vOpenFraction_ )
        if( open > 0.0f && open < 1.0f ) ++cutCellTransition_.partialVFaces;

    const double volumeScale = static_cast<double>( cellSize_ ) * cellSize_; // unit depth = 1m
    cutCellTransition_.fluidVolume = static_cast<float>( current * volumeScale );
    cutCellTransition_.coveredVolume = static_cast<float>( covered * volumeScale );
    cutCellTransition_.releasedVolume = static_cast<float>( released * volumeScale );
    // The cached list is consumed only by coupled Cut-cell pressure. Do not
    // allocate it for one-way fields or scenes with immovable obstacles only.
    const bool needsCoupledGeometry = cutCellProjectionEnabled_ &&
        cutCellCoupledProjectionEnabled_ &&
        std::any_of( obstacles.begin(), obstacles.end(), []( const fluidShapeObstacle2& body )
        {
            return body.inverseMass > 0.0f || body.inverseInertia > 0.0f;
        } );
    if( analyticCutCellCurrentGeometryReuseEnabled_ && disjointGeometry && needsCoupledGeometry )
        analyticDisjointCache_ = std::make_shared<const analyticDisjointCache2>(
            analyticDisjointCache2{ std::move( disjointShapes ) } );
}

void fluidField2::remapExposedFaces()
{
    // 새로 열린 fluid-fluid face가 이전 rigid boundary의 속도를 그대로 상속하지 않도록
    // 직전 Step에서도 fluid이고 이번 Step에도 fluid인 face만 source로 사용한다.
    // nextU_/nextV_는 이류 전에 비어있는 scratch이므로 원래 속도를 복사해둔다.
    if( solidTransition_.exposedCells > 0 )
    {
        std::copy( u_.begin(), u_.end(), nextU_.begin() );
        std::copy( v_.begin(), v_.end(), nextV_.begin() );
    }

    const auto oldFluid = [&]( int x, int y ) { return previousSolidOwner_[cellIndex( x, y )] < 0; };
    const auto newFluid = [&]( int x, int y ) { return solidOwner_[cellIndex( x, y )] < 0; };
    const auto validU = [&]( int x, int y )
    {
        return x > 0 && x < width_ && y >= 0 && y < height_ &&
            oldFluid( x - 1, y ) && oldFluid( x, y ) &&
            newFluid( x - 1, y ) && newFluid( x, y );
    };
    const auto validV = [&]( int x, int y )
    {
        return x >= 0 && x < width_ && y > 0 && y < height_ &&
            oldFluid( x, y - 1 ) && oldFluid( x, y ) &&
            newFluid( x, y - 1 ) && newFluid( x, y );
    };

    // 가까운 유효 유체 면의 직전 속도를 복사한다. 이전 solid 경계 속도나
    // 이번 frame에 함께 새로 열린 면은 참조하지 않으므로 순회 순서와 무관하다.
    const auto extrapolate = [&]( int x, int y, bool horizontal )
    {
        for( int distance = 1; distance <= 4; ++distance )
        {
            double sum = 0.0;
            int count = 0;
            for( int dx = -distance; dx <= distance; ++dx )
            {
                const int dy = distance - std::abs( dx );
                const auto sample = [&]( int sx, int sy )
                {
                    if( horizontal ? validU( sx, sy ) : validV( sx, sy ) )
                    {
                        sum += horizontal ? nextU_[uIndex( sx, sy )] : nextV_[vIndex( sx, sy )];
                        ++count;
                    }
                };
                sample( x + dx, y + dy );
                if( dy != 0 ) sample( x + dx, y - dy );
            }
            if( count > 0 )
            {
                ++solidTransition_.extrapolatedFaces;
                return static_cast<float>( sum / count );
            }
        }
        ++solidTransition_.fallbackFaces;
        return 0.0f;
    };

    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const bool wasFluid = oldFluid( x - 1, y ) && oldFluid( x, y );
            const bool nowFluid = newFluid( x - 1, y ) && newFluid( x, y );
            if( wasFluid && !nowFluid ) ++solidTransition_.closedUFaces;
            if( !wasFluid && nowFluid )
            {
                ++solidTransition_.openedUFaces;
                u_[uIndex( x, y )] = extrapolate( x, y, true );
            }
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const bool wasFluid = oldFluid( x, y - 1 ) && oldFluid( x, y );
            const bool nowFluid = newFluid( x, y - 1 ) && newFluid( x, y );
            if( wasFluid && !nowFluid ) ++solidTransition_.closedVFaces;
            if( !wasFluid && nowFluid )
            {
                ++solidTransition_.openedVFaces;
                v_[vIndex( x, y )] = extrapolate( x, y, false );
            }
        }
}

bool fluidField2::isSolidCell( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y >= 0 && y < height_ && solidOwner_[cellIndex( x, y )] >= 0;
}

void fluidField2::applySolidBoundaries( std::span<const fluidShapeObstacle2> obstacles,
    std::span<const fluidObstacleForce2> pressureFeedback, float timeStep )
{
    // 압력 투영 전에는 전달된 Body 속도로 no-through 경계를 설정한다.
    // 수렴한 Schur 압력 이후에는 같은 pressure traction으로 계산한
    // Δv=dt*M^-1*F, Δomega=dt*I^-1*tau를 같은 MAC face에도 반영한다.
    const bool applyPressure = timeStep > 0.0f && pressureFeedback.size() == obstacles.size();
    double changedSquared = 0.0;
    int boundaryFaces = 0;
    const auto predictedVelocity = [&]( int owner, vec2 point )
    {
        const fluidShapeObstacle2& solid = obstacles[static_cast<std::size_t>( owner )];
        vec2 linear = solid.linearVelocity;
        float angular = solid.angularVelocity;
        if( applyPressure )
        {
            const fluidObstacleForce2& pressure = pressureFeedback[static_cast<std::size_t>( owner )];
            linear += ( timeStep * solid.inverseMass ) * pressure.force;
            angular += timeStep * solid.inverseInertia * pressure.torque;
        }
        return linear + Cross( angular, point - solid.center );
    };

    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const int left = solidOwner_[cellIndex( x - 1, y )];
            const int right = solidOwner_[cellIndex( x, y )];
            const int owner = left >= 0 ? left : right;
            if( owner < 0 ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            const int face = uIndex( x, y );
            const float next = predictedVelocity( owner, point ).x;
            if( applyPressure && ( left < 0 || right < 0 ) )
            {
                const double difference = static_cast<double>( next ) - u_[face];
                changedSquared += difference * difference;
                ++boundaryFaces;
            }
            u_[face] = next;
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int bottom = solidOwner_[cellIndex( x, y - 1 )];
            const int top = solidOwner_[cellIndex( x, y )];
            const int owner = bottom >= 0 ? bottom : top;
            if( owner < 0 ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            const int face = vIndex( x, y );
            const float next = predictedVelocity( owner, point ).y;
            if( applyPressure && ( bottom < 0 || top < 0 ) )
            {
                const double difference = static_cast<double>( next ) - v_[face];
                changedSquared += difference * difference;
                ++boundaryFaces;
            }
            v_[face] = next;
        }

    if( applyPressure )
    {
        pressureMomentum_.synchronizedBoundaryFaces = boundaryFaces;
        pressureMomentum_.boundaryNormalChangeRms = boundaryFaces > 0
            ? static_cast<float>( std::sqrt( changedSquared / boundaryFaces ) ) : 0.0f;
    }
}

float fluidField2::advectionUOpen( int x, int y ) const noexcept
{
    if( x <= 0 || x >= width_ || y < 0 || y >= height_ ) return 0.0f;
    if( cutCellProjectionEnabled_ ) return cutCellUOpen( x, y );
    return isSolidCell( x - 1, y ) || isSolidCell( x, y ) ? 0.0f : 1.0f;
}

float fluidField2::advectionVOpen( int x, int y ) const noexcept
{
    if( x < 0 || x >= width_ || y <= 0 || y >= height_ ) return 0.0f;
    if( cutCellProjectionEnabled_ ) return cutCellVOpen( x, y );
    return isSolidCell( x, y - 1 ) || isSolidCell( x, y ) ? 0.0f : 1.0f;
}

vec2 fluidField2::getAdvectionFaceMomentum() const noexcept
{
    double sumU = 0.0;
    double sumV = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            sumU += static_cast<double>( advectionUOpen( x, y ) ) * u_[uIndex( x, y )];
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            sumV += static_cast<double>( advectionVOpen( x, y ) ) * v_[vIndex( x, y )];
    const double mass = static_cast<double>( cellSize_ ) * cellSize_; // rho=1, depth=1m
    return { static_cast<float>( sumU * mass ), static_cast<float>( sumV * mass ) };
}

void fluidField2::advectConservative( float timeStep )
{
    // 고정된 현재 face 개방율에 대해 U/V dual control volume의 flux-form upwind 이류.
    // 공유 dual edge는 양쪽 셀에서 동일한 flux를 사용하며, dual 외곽은 닫힘.
    float speedMax = 0.0f;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( advectionUOpen( x, y ) > 0.0f )
                speedMax = std::max( speedMax, std::abs( u_[uIndex( x, y )] ) );
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( advectionVOpen( x, y ) > 0.0f )
                speedMax = std::max( speedMax, std::abs( v_[vIndex( x, y )] ) );

    const double cfl = static_cast<double>( speedMax ) * timeStep / cellSize_;
    constexpr int maxSubsteps = 64;
    const int substeps = static_cast<int>( std::clamp( std::ceil( cfl / 0.45 ), 1.0,
        static_cast<double>( maxSubsteps ) ) );
    advectionStats_.maxCfl = static_cast<float>( cfl );
    advectionStats_.substeps = substeps;
    advectionStats_.cflLimited = cfl > maxSubsteps * 0.45;
    const float scale = ( timeStep / substeps ) / cellSize_;
    const auto flux = []( float speed, float lower, float upper, float open )
    {
        return open * speed * ( speed >= 0.0f ? lower : upper );
    };

    for( int step = 0; step < substeps; ++step )
    {
        std::copy( u_.begin(), u_.end(), nextU_.begin() );
        std::copy( v_.begin(), v_.end(), nextV_.begin() );
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const float open = advectionUOpen( x, y );
                if( open == 0.0f ) continue;
                const int index = uIndex( x, y );
                const float u = u_[index];
                float east = 0.0f, west = 0.0f, north = 0.0f, south = 0.0f;
                if( x + 1 < width_ )
                {
                    const float next = u_[uIndex( x + 1, y )];
                    east = flux( 0.5f * ( u + next ), u, next,
                        std::min( open, advectionUOpen( x + 1, y ) ) );
                }
                if( x > 1 )
                {
                    const float prev = u_[uIndex( x - 1, y )];
                    west = flux( 0.5f * ( prev + u ), prev, u,
                        std::min( open, advectionUOpen( x - 1, y ) ) );
                }
                if( y + 1 < height_ )
                {
                    const float next = u_[uIndex( x, y + 1 )];
                    north = flux( 0.5f * ( v_[vIndex( x - 1, y + 1 )] + v_[vIndex( x, y + 1 )] ),
                        u, next, std::min( open, advectionUOpen( x, y + 1 ) ) );
                }
                if( y > 0 )
                {
                    const float prev = u_[uIndex( x, y - 1 )];
                    south = flux( 0.5f * ( v_[vIndex( x - 1, y )] + v_[vIndex( x, y )] ),
                        prev, u, std::min( open, advectionUOpen( x, y - 1 ) ) );
                }
                nextU_[index] = u - scale * ( east - west + north - south ) / open;
            }

        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const float open = advectionVOpen( x, y );
                if( open == 0.0f ) continue;
                const int index = vIndex( x, y );
                const float v = v_[index];
                float east = 0.0f, west = 0.0f, north = 0.0f, south = 0.0f;
                if( x + 1 < width_ )
                {
                    const float next = v_[vIndex( x + 1, y )];
                    east = flux( 0.5f * ( u_[uIndex( x + 1, y - 1 )] + u_[uIndex( x + 1, y )] ),
                        v, next, std::min( open, advectionVOpen( x + 1, y ) ) );
                }
                if( x > 0 )
                {
                    const float prev = v_[vIndex( x - 1, y )];
                    west = flux( 0.5f * ( u_[uIndex( x, y - 1 )] + u_[uIndex( x, y )] ),
                        prev, v, std::min( open, advectionVOpen( x - 1, y ) ) );
                }
                if( y + 1 < height_ )
                {
                    const float next = v_[vIndex( x, y + 1 )];
                    north = flux( 0.5f * ( v + next ), v, next,
                        std::min( open, advectionVOpen( x, y + 1 ) ) );
                }
                if( y > 1 )
                {
                    const float prev = v_[vIndex( x, y - 1 )];
                    south = flux( 0.5f * ( prev + v ), prev, v,
                        std::min( open, advectionVOpen( x, y - 1 ) ) );
                }
                nextV_[index] = v - scale * ( east - west + north - south ) / open;
            }
        u_.swap( nextU_ );
        v_.swap( nextV_ );
    }
}


void fluidField2::remapTracerSwept()
{
    // 끝점의 cut-cell 체적 감소를 이송 상한으로, 실제 sampled swept face
    // 통과 체적에 출발 셀의 이전 농도를 곱한 값을 면별 이송 제안량으로 사용.
    // 정확한 공간-시간 계면 재구성은 아니므로 Part 12a 방식은 기본값으로 유지.
    if( !sweptTracerRemapEnabled_ || !cutCellProjectionEnabled_ ||
        !volumeSourceValid_ || !sweptFaceFlux_.usedSweptFlux ) return;

    tracerStats_.usedSweptRemap = true;
    std::copy( tracerMass_.begin(), tracerMass_.end(), nextTracerMass_.begin() );
    const double cellVolume = static_cast<double>( cellSize_ ) * cellSize_; // unit depth, 1m

    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int source = cellIndex( x, y );
            const double previous = previousCellFluidFraction_[source];
            const double current = cellFluidFraction_[source];
            const double availableMass = tracerMass_[source];
            if( previous <= current || previous <= 0.0 || availableMass <= 0.0 ) continue;

            // 사라진 유체 부피 안에 있던 수동 tracer 질량의 snapshot 추정.
            const double concentration = availableMass / ( previous * cellVolume );
            const double displaced = std::min( availableMass,
                concentration * ( previous - current ) * cellVolume );
            if( displaced <= 0.0 ) continue;

            const std::array<int, 4> neighbors{{
                x > 0 ? cellIndex( x - 1, y ) : -1,
                x + 1 < width_ ? cellIndex( x + 1, y ) : -1,
                y > 0 ? cellIndex( x, y - 1 ) : -1,
                y + 1 < height_ ? cellIndex( x, y + 1 ) : -1
            }};
            std::array<double, 4> outward{{
                -static_cast<double>( sweptUVolume_[uIndex( x, y )] ),
                static_cast<double>( sweptUVolume_[uIndex( x + 1, y )] ),
                -static_cast<double>( sweptVVolume_[vIndex( x, y )] ),
                static_cast<double>( sweptVVolume_[vIndex( x, y + 1 )] )
            }};

            // 각 U/V face의 부호를 셀 바깥쪽으로 통일. 목적지가 현재
            // solid라면 현재 유체의 shared-face 수송 경로로 인정하지 않는다.
            double totalOutwardVolume = 0.0;
            for( int direction = 0; direction < 4; ++direction )
            {
                const int neighbor = neighbors[direction];
                if( neighbor < 0 || cellFluidFraction_[neighbor] <= 0.0f ||
                    !std::isfinite( outward[direction] ) )
                    outward[direction] = 0.0;
                else
                    outward[direction] = std::max( 0.0, outward[direction] );
                totalOutwardVolume += outward[direction];
            }

            const double requested = concentration * totalOutwardVolume;
            const double sweptTransfer = std::min( displaced, requested );
            tracerStats_.sweptRequestedMass += static_cast<float>( requested );
            tracerStats_.sweptLimitedMass += static_cast<float>( std::max( 0.0, requested - displaced ) );
            if( sweptTransfer > 0.0 )
            {
                // 공유 swept face가 기록한 체적 비율대로 각 recipient에게
                // 양을 분배한다. 요청량이 축소 체적을 초과하면 함께 축소한다.
                // 모든 원천 계산은 이 Step 시작 질량만 참조하므로 순서 독립.
                double remaining = sweptTransfer;
                int lastDestination = -1;
                for( int direction = 0; direction < 4; ++direction )
                {
                    if( outward[direction] <= 0.0 ) continue;
                    const double proposed = sweptTransfer * outward[direction] / totalOutwardVolume;
                    const double transfer = std::min( remaining, proposed );
                    nextTracerMass_[source] -= transfer;
                    nextTracerMass_[neighbors[direction]] += transfer;
                    tracerStats_.sweptRemappedMass += static_cast<float>( transfer );
                    remaining -= transfer;
                    lastDestination = neighbors[direction];
                }
                if( remaining > 0.0 && lastDestination >= 0 )
                {
                    nextTracerMass_[source] -= remaining;
                    nextTracerMass_[lastDestination] += remaining;
                    tracerStats_.sweptRemappedMass += static_cast<float>( remaining );
                }
            }

            const double unmatched = displaced - sweptTransfer;
            if( unmatched <= 1.0e-12 * std::max( 1.0, displaced ) ) continue;

            // 적분된 swept 체적이 감소한 cut-cell 체적보다 작을 수 있다.
            // 완전 차폐 셀에서는 기존의 가장 가까운 활성 유체로 이동하고,
            // 부분 개방 셀에서는 새로 열린 이웃이 있을 때에만 이동한다.
            // 그 밖에는 질량을 그대로 두고 미해결량을 공개한다.
            int destination = -1;
            if( current <= 0.0 )
                destination = cutCellNearestFluid_[source];
            else
            {
                float largestGain = 0.0f;
                for( const int neighbor : neighbors )
                {
                    if( neighbor < 0 || cellFluidFraction_[neighbor] <= 0.0f ) continue;
                    const float gain = cellFluidFraction_[neighbor] - previousCellFluidFraction_[neighbor];
                    if( gain > largestGain )
                    {
                        largestGain = gain;
                        destination = neighbor;
                    }
                }
            }

            if( destination >= 0 && destination != source )
            {
                nextTracerMass_[source] -= unmatched;
                nextTracerMass_[destination] += unmatched;
                tracerStats_.fallbackRemappedMass += static_cast<float>( unmatched );
            }
            else
                tracerStats_.unresolvedDisplacementMass += static_cast<float>( unmatched );
        }

    tracerMass_.swap( nextTracerMass_ );
}

void fluidField2::transportTracer( float timeStep )
{
    tracerStats_ = {};
    if( !tracerEnabled_ ) return;

    tracerStats_.beforeMass = getTracerMass();
    remapTracerSwept();
    const double area = static_cast<double>( cellSize_ ) * cellSize_;
    double maxSpeed = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( advectionUOpen( x, y ) > 0.0f )
                maxSpeed = std::max( maxSpeed, static_cast<double>( std::abs( u_[uIndex( x, y )] ) ) );
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( advectionVOpen( x, y ) > 0.0f )
                maxSpeed = std::max( maxSpeed, static_cast<double>( std::abs( v_[vIndex( x, y )] ) ) );

    const double cfl = maxSpeed * timeStep / cellSize_;
    constexpr int maxSubsteps = 64;
    const int substeps = static_cast<int>( std::clamp( std::ceil( cfl / 0.45 ), 1.0,
        static_cast<double>( maxSubsteps ) ) );
    tracerStats_.maxCfl = static_cast<float>( cfl );
    tracerStats_.substeps = substeps;
    tracerStats_.cflLimited = cfl > maxSubsteps * 0.45;
    const double lengthTime = static_cast<double>( timeStep ) * cellSize_ / substeps;

    // 셀 tracer 질량 = 농도 * 현재 유체 비율 * h². 각 공유 MAC 면의
    // signed upwind 유량은 딱 한 번 계산하고, donor에서만 질량을 뺀다.
    // 출구가 여럿일 때 동시 유출량을 donor 질량으로 제한해 음수를 방지한다.
    for( int substep = 0; substep < substeps; ++substep )
    {
        std::fill( tracerOutgoing_.begin(), tracerOutgoing_.end(), 0.0 );
        std::fill( tracerUFlux_.begin(), tracerUFlux_.end(), 0.0 );
        std::fill( tracerVFlux_.begin(), tracerVFlux_.end(), 0.0 );
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const float open = advectionUOpen( x, y );
                if( open <= 0.0f ) continue;
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                const double velocity = u_[uIndex( x, y )];
                const int donor = velocity >= 0.0 ? left : right;
                const float fraction = tracerCellFraction( velocity >= 0.0 ? x - 1 : x, y );
                if( fraction <= 0.0f ) continue;
                const double concentration = tracerMass_[donor] / ( area * fraction );
                const double flux = velocity * open * lengthTime * concentration;
                tracerUFlux_[uIndex( x, y )] = flux;
                tracerOutgoing_[donor] += std::abs( flux );
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const float open = advectionVOpen( x, y );
                if( open <= 0.0f ) continue;
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                const double velocity = v_[vIndex( x, y )];
                const int donor = velocity >= 0.0 ? bottom : top;
                const float fraction = tracerCellFraction( x, velocity >= 0.0 ? y - 1 : y );
                if( fraction <= 0.0f ) continue;
                const double concentration = tracerMass_[donor] / ( area * fraction );
                const double flux = velocity * open * lengthTime * concentration;
                tracerVFlux_[vIndex( x, y )] = flux;
                tracerOutgoing_[donor] += std::abs( flux );
            }

        std::copy( tracerMass_.begin(), tracerMass_.end(), nextTracerMass_.begin() );
        const auto limitedFlux = [&]( double flux, int donor )
        {
            const double outgoing = tracerOutgoing_[donor];
            return outgoing > tracerMass_[donor] && outgoing > 0.0
                ? flux * ( tracerMass_[donor] / outgoing ) : flux;
        };
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                const double flux = tracerUFlux_[uIndex( x, y )];
                const double transfer = limitedFlux( flux, flux >= 0.0 ? left : right );
                nextTracerMass_[left] -= transfer;
                nextTracerMass_[right] += transfer;
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                const double flux = tracerVFlux_[vIndex( x, y )];
                const double transfer = limitedFlux( flux, flux >= 0.0 ? bottom : top );
                nextTracerMass_[bottom] -= transfer;
                nextTracerMass_[top] += transfer;
            }
        tracerMass_.swap( nextTracerMass_ );
    }

    double trapped = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( tracerCellFraction( x, y ) <= 0.0f )
                trapped += tracerMass_[cellIndex( x, y )];
    tracerStats_.trappedMass = static_cast<float>( trapped );
    tracerStats_.afterMass = getTracerMass();
}

void fluidField2::step( float timeStep, std::span<const fluidCircleObstacle2> obstacles,
    std::span<fluidObstacleForce2> feedback, int pressureIterations )
{
    if( feedback.size() != obstacles.size() )
        throw std::invalid_argument( "fluidField2: obstacle and feedback sizes must match" );

    // 기존 원 전용 API는 호환 유지. 단일 solver는 공통 Shape API만 수행.
    std::vector<fluidShapeObstacle2> shapes;
    shapes.reserve( obstacles.size() );
    for( const fluidCircleObstacle2& obstacle : obstacles )
        shapes.push_back( { circle2{ {}, obstacle.radius }, { obstacle.center, {} }, obstacle.center,
            obstacle.linearVelocity, obstacle.angularVelocity } );
    step( timeStep, std::span<const fluidShapeObstacle2>( shapes ), feedback, pressureIterations );
}

void fluidField2::step( float timeStep, std::span<const fluidShapeObstacle2> obstacles,
    std::span<fluidObstacleForce2> feedback, int pressureIterations )
{
    if( feedback.size() != obstacles.size() )
        throw std::invalid_argument( "fluidField2: obstacle and feedback sizes must match" );
    std::fill( feedback.begin(), feedback.end(), fluidObstacleForce2{} );
    if( !std::isfinite( timeStep ) || timeStep <= 0.0f ) return;

    // No clock reads in normal simulation: this is an explicit profiling mode.
    using clock = std::chrono::steady_clock;
    const bool profile = stepTimingEnabled_;
    if( profile ) stepTiming_ = {};
    const auto started = profile ? clock::now() : clock::time_point{};
    auto sectionStarted = started;
    const auto record = [&]( double& elapsed )
    {
        if( !profile ) return;
        const auto now = clock::now();
        elapsed += std::chrono::duration<double, std::milli>( now - sectionStarted ).count();
        sectionStarted = now;
    };

    faceStageAudit_ = {};
    const bool auditFace = faceStageAuditEnabled_ && cutCellProjectionEnabled_;
    const auto readAuditFace = [&]() noexcept
    {
        return faceStageAuditIsU_
            ? u_[uIndex( faceStageAuditX_, faceStageAuditY_ )]
            : v_[vIndex( faceStageAuditX_, faceStageAuditY_ )];
    };
    if( auditFace )
    {
        faceStageAudit_.valid = true;
        faceStageAudit_.isU = faceStageAuditIsU_;
        faceStageAudit_.x = faceStageAuditX_;
        faceStageAudit_.y = faceStageAuditY_;
        faceStageAudit_.beforeGeometry = readAuditFace();
    }

    stepMomentum_.before = getFluidMomentum();
    record( stepTiming_.other );
    updateObstacles( obstacles );
    record( stepTiming_.occupancy );
    updateCutCellGeometry( obstacles );
    if( auditFace )
        faceStageAudit_.openFraction = faceStageAuditIsU_
            ? cutCellUOpen( faceStageAuditX_, faceStageAuditY_ )
            : cutCellVOpen( faceStageAuditX_, faceStageAuditY_ );
    record( stepTiming_.geometry );
    if( cutCellProjectionEnabled_ )
    {
        updateSweptSolidFlux( timeStep, obstacles );
        reconcileSweptSolidFlux();
    }
    else sweptFaceFlux_ = {};
    record( stepTiming_.swept );
    remapExposedFaces();
    if( auditFace ) faceStageAudit_.afterRemap = readAuditFace();
    record( stepTiming_.remap );
    stepMomentum_.afterObstacles = getFluidMomentum();
    if( cutCellProjectionEnabled_ ) applyCutCellBoundaries();
    else applySolidBoundaries( obstacles );
    if( auditFace ) faceStageAudit_.afterBoundary = readAuditFace();

    advectionStats_ = {};
    advectionStats_.conservative = conservativeAdvectionEnabled_;
    advectionStats_.before = getAdvectionFaceMomentum();
    record( stepTiming_.boundaries );
    if( conservativeAdvectionEnabled_ ) advectConservative( timeStep );
    else
    {
        // 기존 반 라그랑주 이류: 읽기 전용 source로부터 face별 역추적.
        std::fill( nextU_.begin(), nextU_.end(), 0.0f );
    std::fill( nextV_.begin(), nextV_.end(), 0.0f );
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            const vec2 previous = point - timeStep * sampleFrom( u_, v_, point );
            nextU_[uIndex( x, y )] = sampleFrom( u_, v_, previous ).x;
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            const vec2 previous = point - timeStep * sampleFrom( u_, v_, point );
            nextV_[vIndex( x, y )] = sampleFrom( u_, v_, previous ).y;
        }

        u_.swap( nextU_ );
        v_.swap( nextV_ );
        advectionStats_.substeps = 1;
    }
    record( stepTiming_.advection );
    if( cutCellProjectionEnabled_ ) applyCutCellBoundaries();
    else applySolidBoundaries( obstacles );
    if( auditFace ) faceStageAudit_.afterAdvection = readAuditFace();
    advectionStats_.after = getAdvectionFaceMomentum();
    stepMomentum_.afterAdvection = getFluidMomentum();
    record( stepTiming_.boundaries );
    if( cutCellProjectionEnabled_ && prePressureSnapshotObserver_ )
    {
        // Only an explicit S0 observer pays for this obstacle copy. The
        // observer may deep-copy *this; no shared mutable replay workspace.
        frozenPressureObstacles_.assign( obstacles.begin(), obstacles.end() );
        frozenPressureTimeStep_ = timeStep;
        frozenPressureIterations_ = std::clamp( pressureIterations, 1, 200 );
        frozenPressureReplayReady_ = true;
        try { prePressureSnapshotObserver_( *this ); }
        catch( ... )
        {
            frozenPressureReplayReady_ = false;
            frozenPressureObstacles_.clear();
            throw;
        }
        frozenPressureReplayReady_ = false;
        frozenPressureObstacles_.clear();
    }
    if( cutCellProjectionEnabled_ )
    {
        pressureMomentum_ = {}; // 이전 이진 MAC face impulse ledger와 단위가 다르므로 혼합하지 않음.
        projectCutCell( timeStep, std::clamp( pressureIterations, 1, 200 ), obstacles, feedback );
    }
    else
    {
        cutCellPressure_ = {};
        project( timeStep, std::clamp( pressureIterations, 1, 200 ), obstacles, feedback );
    }
    stepMomentum_.afterPressure = getFluidMomentum();
    if( auditFace )
    {
        faceStageAudit_.afterPressure = readAuditFace();
        const int x = faceStageAuditX_;
        const int y = faceStageAuditY_;
        const int lower = faceStageAuditIsU_ ? cellIndex( x - 1, y ) : cellIndex( x, y - 1 );
        const int upper = cellIndex( x, y );
        faceStageAudit_.lowerCellFraction = cellFluidFraction_[lower];
        faceStageAudit_.upperCellFraction = cellFluidFraction_[upper];
        faceStageAudit_.lowerVolumeRate = cutCellVolumeRate_[lower];
        faceStageAudit_.upperVolumeRate = cutCellVolumeRate_[upper];
        faceStageAudit_.lowerPressure = pressure_[lower];
        faceStageAudit_.upperPressure = pressure_[upper];
        // The open-face correction in projectCutCell is -dt/h * (pUpper-pLower).
        // Closed faces have boundary velocity and are not pressure-corrected.
        if( faceStageAudit_.openFraction > 0.0f )
            faceStageAudit_.predictedPressureDelta = timeStep * ( 1.0f / cellSize_ ) *
                ( pressure_[lower] - pressure_[upper] );

        // Mirror the final pressure operator row without writing to scratch,
        // pressure, velocities or feedback. Only active for a selected face.
        std::vector<coupledBodyResponse> responses;
        if( cutCellPressure_.usedCoupledProjection )
        {
            responses.resize( obstacles.size() );
            for( const coupledPressureFace& face : coupledPressureScratch_.faces )
            {
                coupledBodyResponse& response = responses[face.owner];
                const double p = pressure_[face.cell];
                response.fx += p * face.fx;
                response.fy += p * face.fy;
                response.torque += p * face.torque;
            }
        }
        const auto inspect = [&]( int cell ) -> fluidFaceCellPressureAudit2
        {
            fluidFaceCellPressureAudit2 row{};
            const int cx = cell % width_;
            const int cy = cell / width_;
            row.westOpen = cutCellUOpen( cx, cy );
            row.eastOpen = cutCellUOpen( cx + 1, cy );
            row.southOpen = cutCellVOpen( cx, cy );
            row.northOpen = cutCellVOpen( cx, cy + 1 );
            const double p = pressure_[cell];
            if( row.westOpen > 0.0f )
                row.laplacian += row.westOpen * ( p - pressure_[cellIndex( cx - 1, cy )] );
            if( row.eastOpen > 0.0f )
                row.laplacian += row.eastOpen * ( p - pressure_[cellIndex( cx + 1, cy )] );
            if( row.southOpen > 0.0f )
                row.laplacian += row.southOpen * ( p - pressure_[cellIndex( cx, cy - 1 )] );
            if( row.northOpen > 0.0f )
                row.laplacian += row.northOpen * ( p - pressure_[cellIndex( cx, cy + 1 )] );
            if( cutCellPressure_.usedCoupledProjection )
                for( const coupledPressureFace& face : coupledPressureScratch_.faces )
                {
                    if( face.cell != cell ) continue;
                    const fluidShapeObstacle2& obstacle = obstacles[face.owner];
                    const coupledBodyResponse& response = responses[face.owner];
                    row.schur += static_cast<double>( obstacle.inverseMass ) *
                        ( face.fx * response.fx + face.fy * response.fy ) +
                        static_cast<double>( obstacle.inverseInertia ) * face.torque * response.torque;
                }
            row.rhs = -static_cast<double>( divergence_[cell] ) *
                ( static_cast<double>( cellSize_ ) * cellSize_ / timeStep );
            row.rowResidual = row.rhs - row.laplacian - row.schur;
            row.finalOpenFlux = static_cast<double>( row.eastOpen ) * u_[uIndex( cx + 1, cy )] -
                static_cast<double>( row.westOpen ) * u_[uIndex( cx, cy )] +
                static_cast<double>( row.northOpen ) * v_[vIndex( cx, cy + 1 )] -
                static_cast<double>( row.southOpen ) * v_[vIndex( cx, cy )];
            row.rawContinuity = row.finalOpenFlux / cellSize_ +
                static_cast<double>( cutCellVolumeRate_[cell] ) / timeStep;
            return row;
        };
        faceStageAudit_.lowerRow = inspect( lower );
        faceStageAudit_.upperRow = inspect( upper );
    }
    record( stepTiming_.pressure );
    transportTracer( timeStep );
    record( stepTiming_.tracer );
    previousObstacles_.assign( obstacles.begin(), obstacles.end() );
    record( stepTiming_.other );
    if( profile )
        stepTiming_.total = std::chrono::duration<double, std::milli>(
            clock::now() - started ).count();
}

float fluidField2::cutCellUOpen( int x, int y ) const noexcept
{
    if( x <= 0 || x >= width_ || y < 0 || y >= height_ ) return 0.0f;
    if( cellFluidFraction_[cellIndex( x - 1, y )] == 0.0f ||
        cellFluidFraction_[cellIndex( x, y )] == 0.0f ) return 0.0f;
    return uOpenFraction_[uIndex( x, y )];
}

float fluidField2::cutCellVOpen( int x, int y ) const noexcept
{
    if( x < 0 || x >= width_ || y <= 0 || y >= height_ ) return 0.0f;
    if( cellFluidFraction_[cellIndex( x, y - 1 )] == 0.0f ||
        cellFluidFraction_[cellIndex( x, y )] == 0.0f ) return 0.0f;
    return vOpenFraction_[vIndex( x, y )];
}

void fluidField2::applyCutCellBoundaries() noexcept
{
    // 완전히 막힌 face에는 강체 법선 속도를 둔다. 일부 열린 face의 u/v는
    // 유체 성분 속도이며, 막힌 구간의 강체 속도는 별도 flux 항에만 사용한다.
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( cutCellUOpen( x, y ) == 0.0f ) u_[uIndex( x, y )] = uSolidVelocity_[uIndex( x, y )];
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( cutCellVOpen( x, y ) == 0.0f ) v_[vIndex( x, y )] = vSolidVelocity_[vIndex( x, y )];
}

void fluidField2::synchronizeCutCellBoundaries( float timeStep,
    std::span<const fluidShapeObstacle2> obstacles, std::span<const fluidObstacleForce2> feedback )
{
    // updateCutCellGeometry와 동일한 sub-face 표본 / 첫 점유 Shape 규칙으로 재계산한다.
    // fluid 면의 속도 u_/v_는 보존하고 막힌 비율의 solid 속도만 예측값으로 갱신.
    const int sampleCount = cutCellSampleCount_;
    const float inverseCount = 1.0f / sampleCount;
    struct predictedBody
    {
        vec2 linear{};
        float angular = 0.0f;
    };
    std::fill( pressureUSolidFlux_.begin(), pressureUSolidFlux_.end(), 0.0f );
    std::fill( pressureVSolidFlux_.begin(), pressureVSolidFlux_.end(), 0.0f );
    std::vector<predictedBody> predicted( obstacles.size() );
    std::vector<double> bounds( obstacles.size(), -1.0 );
    for( std::size_t i = 0; i < obstacles.size(); ++i )
    {
        const fluidShapeObstacle2& body = obstacles[i];
        predicted[i].linear = body.linearVelocity + ( timeStep * body.inverseMass ) * feedback[i].force;
        predicted[i].angular = body.angularVelocity + timeStep * body.inverseInertia * feedback[i].torque;
        if( IsValidFluidObstacle( body ) ) bounds[i] = FluidObstacleBoundRadius( body );
    }
    const auto firstOwner = [&]( vec2 point )
    {
        for( std::size_t i = 0; i < obstacles.size(); ++i )
        {
            if( bounds[i] < 0.0 ) continue;
            const vec2 center = obstacles[i].transform.position;
            if( std::abs( static_cast<double>( point.x ) - center.x ) > bounds[i] ||
                std::abs( static_cast<double>( point.y ) - center.y ) > bounds[i] ) continue;
            if( ContainsFluidObstacle( obstacles[i], point ) ) return static_cast<int>( i );
        }
        return -1;
    };

    double changedSquared = 0.0;
    int boundaryFaces = 0;
    int partialFaces = 0;
    const auto referenceShapes = analyticDisjointCache_
        ? std::vector<analyticDisjointShape2>{} : disjointAnalyticCutCellEnabled_
            ? BuildDisjointAnalyticShapes( obstacles ) : std::vector<analyticDisjointShape2>{};
    const auto& disjointShapes = analyticDisjointCache_
        ? analyticDisjointCache_->shapes : referenceShapes;
    if( !disjointShapes.empty() )
    {
        const double h = cellSize_;
        const auto updateFace = [&]( bool vertical, int x, int y )
        {
            const int face = vertical ? uIndex( x, y ) : vIndex( x, y );
            const float open = vertical ? uOpenFraction_[face] : vOpenFraction_[face];
            if( open >= 1.0f ) return;
            const double normal = vertical ? origin_.x + x * h : origin_.y + y * h;
            const double low = vertical ? origin_.y + y * h : origin_.x + x * h;
            double length = 0.0;
            double sum = 0.0;
            ForEachAnalyticFacePortion( disjointShapes, vertical, normal, low, low + h,
                [&]( std::size_t i, analyticCircleInterval clipped )
            {
                const vec2 point = vertical
                    ? vec2{ static_cast<float>( normal ), static_cast<float>( clipped.midpoint ) }
                    : vec2{ static_cast<float>( clipped.midpoint ), static_cast<float>( normal ) };
                const vec2 velocity = predicted[i].linear +
                    Cross( predicted[i].angular, point - obstacles[i].center );
                length += clipped.length;
                sum += clipped.length * ( vertical ? velocity.x : velocity.y );
            } );
            if( length <= 0.0 ) return;
            float& previous = vertical ? uSolidVelocity_[face] : vSolidVelocity_[face];
            const float next = static_cast<float>( sum / length );
            const double change = static_cast<double>( next ) - previous;
            changedSquared += change * change;
            // Weighted mean is exactly equivalent to the sum of owner-specific Bᵀ fluxes.
            const float flux = ( 1.0f - open ) * static_cast<float>( change );
            if( vertical ) pressureUSolidFlux_[face] = flux;
            else pressureVSolidFlux_[face] = flux;
            previous = next;
            ++boundaryFaces;
            if( open > 0.0f ) ++partialFaces;
        };
        if( analyticCutCellIndexedBoundarySyncEnabled_ )
        {
            // The geometry pass stored every positive blocked interval, even
            // when the float aperture rounds to fully open. Since updateFace
            // already skips open >= 1, these are an exact superset of faces
            // that can receive a predicted solid boundary velocity change.
            // Preserve U-then-V row-major traversal and per-face reductions.
            for( int face : analyticUBlockedFaces_ )
                updateFace( true, face % ( width_ + 1 ), face / ( width_ + 1 ) );
            for( int face : analyticVBlockedFaces_ )
                updateFace( false, face % width_, face / width_ );
        }
        else
        {
            for( int y = 0; y < height_; ++y )
                for( int x = 1; x < width_; ++x ) updateFace( true, x, y );
            for( int y = 1; y < height_; ++y )
                for( int x = 0; x < width_; ++x ) updateFace( false, x, y );
        }
    }
    else
    {
    analyticFluidCircle circle{};
    analyticFluidPolygon polygon{};
    analyticFluidCapsule capsule{};
    analyticFluidRoundedPolygon rounded{};
    const bool circleGeometry = analyticCircleCutCellEnabled_ &&
        TryGetAnalyticCircle( obstacles, circle );
    const bool polygonGeometry = !circleGeometry && analyticPolygonCutCellEnabled_ &&
        TryGetAnalyticPolygon( obstacles, polygon );
    const bool capsuleGeometry = !circleGeometry && !polygonGeometry &&
        analyticCapsuleCutCellEnabled_ && TryGetAnalyticCapsule( obstacles, capsule );
    const bool roundedGeometry = !circleGeometry && !polygonGeometry && !capsuleGeometry &&
        analyticRoundedPolygonCutCellEnabled_ && TryGetAnalyticRoundedPolygon( obstacles, rounded );
    if( circleGeometry || polygonGeometry || capsuleGeometry || roundedGeometry )
    {
        const double h = cellSize_;
        const auto& body = predicted.front();
        const auto updateFace = [&]( bool vertical, int x, int y )
        {
            const int face = vertical ? uIndex( x, y ) : vIndex( x, y );
            const float open = vertical ? uOpenFraction_[face] : vOpenFraction_[face];
            if( open >= 1.0f ) return;
            const double normal = vertical ? origin_.x + x * h : origin_.y + y * h;
            const double low = vertical ? origin_.y + y * h : origin_.x + x * h;
            const auto clipped = circleGeometry
                ? ClipCircleFace( circle, vertical, normal, low, low + h )
                : polygonGeometry
                    ? ClipPolygonFace( polygon, vertical, normal, low, low + h )
                    : capsuleGeometry
                        ? ClipCapsuleFace( capsule, vertical, normal, low, low + h )
                        : ClipRoundedPolygonFace( rounded, vertical, normal, low, low + h );
            if( clipped.length <= 0.0 ) return;
            const vec2 point = vertical
                ? vec2{ static_cast<float>( normal ), static_cast<float>( clipped.midpoint ) }
                : vec2{ static_cast<float>( clipped.midpoint ), static_cast<float>( normal ) };
            const vec2 velocity = body.linear + Cross( body.angular, point - obstacles.front().center );
            const float next = vertical ? velocity.x : velocity.y;
            float& previous = vertical ? uSolidVelocity_[face] : vSolidVelocity_[face];
            const double change = static_cast<double>( next ) - previous;
            changedSquared += change * change;
            const float solidFraction = static_cast<float>( clipped.length / h );
            if( vertical ) pressureUSolidFlux_[face] = solidFraction * static_cast<float>( change );
            else pressureVSolidFlux_[face] = solidFraction * static_cast<float>( change );
            previous = next;
            ++boundaryFaces;
            if( open > 0.0f ) ++partialFaces;
        };
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x ) updateFace( true, x, y );
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x ) updateFace( false, x, y );
    }
    else
    {
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const int face = uIndex( x, y );
            if( uOpenFraction_[face] >= 1.0f ) continue;
            float sum = 0.0f;
            int blocked = 0;
            for( int sample = 0; sample < sampleCount; ++sample )
            {
                const vec2 point = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) * inverseCount };
                const int owner = firstOwner( point );
                if( owner < 0 ) continue;
                const predictedBody& body = predicted[static_cast<std::size_t>( owner )];
                sum += ( body.linear + Cross( body.angular, point - obstacles[owner].center ) ).x;
                ++blocked;
            }
            if( blocked == 0 ) continue;
            const float next = sum / blocked;
            const double change = static_cast<double>( next ) - uSolidVelocity_[face];
            changedSquared += change * change;
            pressureUSolidFlux_[face] = ( 1.0f - uOpenFraction_[face] ) * static_cast<float>( change );
            uSolidVelocity_[face] = next;
            ++boundaryFaces;
            if( uOpenFraction_[face] > 0.0f ) ++partialFaces;
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int face = vIndex( x, y );
            if( vOpenFraction_[face] >= 1.0f ) continue;
            float sum = 0.0f;
            int blocked = 0;
            for( int sample = 0; sample < sampleCount; ++sample )
            {
                const vec2 point = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ) + ( sample + 0.5f ) * inverseCount, static_cast<float>( y ) };
                const int owner = firstOwner( point );
                if( owner < 0 ) continue;
                const predictedBody& body = predicted[static_cast<std::size_t>( owner )];
                sum += ( body.linear + Cross( body.angular, point - obstacles[owner].center ) ).y;
                ++blocked;
            }
            if( blocked == 0 ) continue;
            const float next = sum / blocked;
            const double change = static_cast<double>( next ) - vSolidVelocity_[face];
            changedSquared += change * change;
            pressureVSolidFlux_[face] = ( 1.0f - vOpenFraction_[face] ) * static_cast<float>( change );
            vSolidVelocity_[face] = next;
            ++boundaryFaces;
            if( vOpenFraction_[face] > 0.0f ) ++partialFaces;
        }
    }
    }

    // 완전히 막힌 MAC 면만 u_/v_로 복사한다. 부분 개방 면의 유체 성분은 그대로 둔다.
    applyCutCellBoundaries();
    cutCellPressure_.synchronizedBoundaryFaces = boundaryFaces;
    cutCellPressure_.synchronizedPartialFaces = partialFaces;
    cutCellPressure_.boundaryNormalChangeRms = boundaryFaces > 0
        ? static_cast<float>( std::sqrt( changedSquared / boundaryFaces ) ) : 0.0f;
}

float fluidField2::cutCellFluxDivergenceRms( float inverseTimeStep ) const noexcept
{
    double sum = 0.0;
    int active = 0;
    const float inverseCell = 1.0f / cellSize_;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] <= 0.0f ) continue;
            const float flux = (
                cutCellUOpen( x + 1, y ) * u_[uIndex( x + 1, y )] -
                cutCellUOpen( x, y ) * u_[uIndex( x, y )] +
                cutCellVOpen( x, y + 1 ) * v_[vIndex( x, y + 1 )] -
                cutCellVOpen( x, y ) * v_[vIndex( x, y )] ) * inverseCell;
            const double residual = static_cast<double>( flux ) +
                static_cast<double>( cutCellVolumeRate_[cell] ) * inverseTimeStep;
            sum += residual * residual;
            ++active;
        }
    return active > 0 ? static_cast<float>( std::sqrt( sum / active ) ) : 0.0f;
}

void fluidField2::updateSweptSolidFlux( float timeStep, std::span<const fluidShapeObstacle2> obstacles )
{
    sweptFaceFlux_ = {};
    std::fill( sweptUVolume_.begin(), sweptUVolume_.end(), 0.0f );
    std::fill( sweptVVolume_.begin(), sweptVVolume_.end(), 0.0f );

    // 인덱스가 유지되고 기하가 같으며 도메인 바깥을 가로지르지 않은 shape만 추적.
    if( !volumeSourceValid_ || obstacles.empty() || previousObstacles_.size() != obstacles.size() ) return;

    const float limitX = origin_.x + width_ * cellSize_;
    const float limitY = origin_.y + height_ * cellSize_;
    std::vector<vec2> linearSpeeds( obstacles.size() );
    std::vector<float> startAngles( obstacles.size() );
    std::vector<float> angularSpeeds( obstacles.size() );
    std::vector<double> bounds( obstacles.size() );
    float maximumTravel = 0.0f;
    for( std::size_t i = 0; i < obstacles.size(); ++i )
    {
        const auto& start = previousObstacles_[i];
        const auto& finish = obstacles[i];
        if( !IsValidFluidObstacle( start ) || !IsValidFluidObstacle( finish ) ||
            !HasSameFluidGeometry( start.geometry, finish.geometry ) ) return;
        const float radius = FluidGeometryRadius( finish.geometry );
        if( !std::isfinite( radius ) || radius < 0.0f ) return;
        for( const fluidShapeObstacle2* shape : { &start, &finish } )
        {
            const vec2 p = shape->transform.position;
            if( p.x - radius <= origin_.x || p.x + radius >= limitX ||
                p.y - radius <= origin_.y || p.y + radius >= limitY ) return;
        }

        const vec2 displacement = finish.transform.position - start.transform.position;
        linearSpeeds[i] = displacement / timeStep;
        const float startAngle = std::atan2( start.transform.rotation.s, start.transform.rotation.c );
        const float endAngle = std::atan2( finish.transform.rotation.s, finish.transform.rotation.c );
        const float deltaAngle = std::remainder( endAngle - startAngle, 2.0f * std::numbers::pi_v<float> );
        startAngles[i] = startAngle;
        angularSpeeds[i] = deltaAngle / timeStep;
        bounds[i] = FluidObstacleBoundRadius( finish );
        maximumTravel = std::max( maximumTravel, Length( displacement ) + std::abs( deltaAngle ) * radius );
    }

    sweptFaceFlux_.usedSweptFlux = true;
    // 정적 장애물이 많은 장면에서 매 step 전 격자·4 sub-face를 검사할 필요가 없다.
    // 유효한 동일 geometry가 정지했다면 swept flux는 정확히 0이다.
    if( maximumTravel == 0.0f )
    {
        sweptFaceFlux_.timeSamples = 1;
        return;
    }

    // Swept velocity is derived from the change in transform, not the
    // obstacle's instantaneous velocity or inverse mass. Kinematic moving
    // walls are included; physically stationary bodies contribute zero flux.
    std::vector<int> movingOwners;
    if( analyticCutCellMovingSweptFacesEnabled_ )
    {
        movingOwners.reserve( obstacles.size() );
        for( std::size_t i = 0; i < obstacles.size(); ++i )
            if( linearSpeeds[i].x != 0.0f || linearSpeeds[i].y != 0.0f ||
                angularSpeeds[i] != 0.0f )
                movingOwners.push_back( static_cast<int>( i ) );
    }

    const int faceSamples = cutCellSampleCount_;
    constexpr int maxTimeSamples = 64;
    int timeSamples = std::clamp(
        static_cast<int>( std::ceil( maximumTravel / ( 0.25f * cellSize_ ) ) ), 1, 32 );
    std::vector<fluidShapeObstacle2> intermediate( obstacles.begin(), obstacles.end() );
    // Reused only by the bounded multi-shape analytic swept path; allocation
    // and clearing are skipped for other swept geometry modes.
    std::vector<double> boundedUSweptScratch, boundedVSweptScratch;

    const auto integrate = [&]( int samples, std::vector<float>& sweptU, std::vector<float>& sweptV )
    {
        std::fill( sweptU.begin(), sweptU.end(), 0.0f );
        std::fill( sweptV.begin(), sweptV.end(), 0.0f );
        const float weight = timeStep * cellSize_ / ( samples * faceSamples );
        for( int step = 0; step < samples; ++step )
        {
            const float alpha = ( step + 0.5f ) / samples;
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                const auto& start = previousObstacles_[i];
                const auto& finish = obstacles[i];
                intermediate[i].transform.position =
                    start.transform.position + alpha * ( finish.transform.position - start.transform.position );
                intermediate[i].transform.rotation =
                    rot2::FromRadians( startAngles[i] + angularSpeeds[i] * timeStep * alpha );
            }

            const auto disjointShapes = disjointAnalyticCutCellEnabled_
                ? BuildDisjointAnalyticShapes( intermediate ) : std::vector<analyticDisjointShape2>{};
            if( !disjointShapes.empty() )
            {
                const double h = cellSize_;
                const float intervalDt = timeStep / samples;
                if( analyticCutCellBoundedSweptFacesEnabled_ &&
                    analyticCutCellMovingSweptFacesEnabled_ && disjointShapes.size() > 2 )
                {
                    // Unlike the current-step geometry cache, these analytic
                    // shapes belong to the *intermediate* time snapshot.
                    // Only moving owners have swept normal velocity. Evaluate
                    // their clipped intervals inside conservatively padded
                    // AABB face ranges, keeping each face's owner sum order.
                    boundedUSweptScratch.assign( u_.size(), 0.0 );
                    boundedVSweptScratch.assign( v_.size(), 0.0 );
                    const auto clampFace = []( double index, int lower, int upper )
                    {
                        return static_cast<int>( std::clamp( index,
                            static_cast<double>( lower ), static_cast<double>( upper ) ) );
                    };
                    for( int owner : movingOwners )
                    {
                        const auto& shape = disjointShapes[owner];
                        const auto accumulateFaces = [&]( bool vertical )
                        {
                            const int minX = clampFace( std::floor(
                                ( static_cast<double>( shape.bounds.min.x ) - origin_.x ) / h ) - 1.0,
                                vertical ? 1 : 0, width_ - 1 );
                            const int maxX = clampFace( std::ceil(
                                ( static_cast<double>( shape.bounds.max.x ) - origin_.x ) / h ) + 1.0,
                                vertical ? 1 : 0, width_ - 1 );
                            const int minY = clampFace( std::floor(
                                ( static_cast<double>( shape.bounds.min.y ) - origin_.y ) / h ) - 1.0,
                                vertical ? 0 : 1, height_ - 1 );
                            const int maxY = clampFace( std::ceil(
                                ( static_cast<double>( shape.bounds.max.y ) - origin_.y ) / h ) + 1.0,
                                vertical ? 0 : 1, height_ - 1 );
                            for( int y = minY; y <= maxY; ++y )
                                for( int x = minX; x <= maxX; ++x )
                                {
                                    const double normal = vertical ? origin_.x + x * h : origin_.y + y * h;
                                    const double low = vertical ? origin_.y + y * h : origin_.x + x * h;
                                    const auto clipped = ClipDisjointShapeFace(
                                        shape, vertical, normal, low, low + h );
                                    if( clipped.length <= 0.0 ) continue;
                                    const vec2 point = vertical
                                        ? vec2{ static_cast<float>( normal ), static_cast<float>( clipped.midpoint ) }
                                        : vec2{ static_cast<float>( clipped.midpoint ), static_cast<float>( normal ) };
                                    const vec2 velocity = linearSpeeds[owner] +
                                        Cross( angularSpeeds[owner], point - intermediate[owner].transform.position );
                                    double& flux = vertical ? boundedUSweptScratch[uIndex( x, y )] :
                                        boundedVSweptScratch[vIndex( x, y )];
                                    flux += clipped.length * ( vertical ? velocity.x : velocity.y );
                                }
                        };
                        accumulateFaces( true );
                        accumulateFaces( false );
                    }

                    // Convert the per-face double sum to float exactly once,
                    // in the same U-then-V order as the original reference.
                    for( int y = 0; y < height_; ++y )
                        for( int x = 1; x < width_; ++x )
                        {
                            const int face = uIndex( x, y );
                            sweptU[face] += intervalDt * static_cast<float>( boundedUSweptScratch[face] );
                        }
                    for( int y = 1; y < height_; ++y )
                        for( int x = 0; x < width_; ++x )
                        {
                            const int face = vIndex( x, y );
                            sweptV[face] += intervalDt * static_cast<float>( boundedVSweptScratch[face] );
                        }
                }
                else
                {
                    const auto accumulateFace = [&]( bool vertical, int x, int y )
                    {
                        const double normal = vertical ? origin_.x + x * h : origin_.y + y * h;
                        const double low = vertical ? origin_.y + y * h : origin_.x + x * h;
                        double flux = 0.0;
                        const auto addPortion = [&]( std::size_t i, analyticCircleInterval clipped )
                        {
                            const vec2 point = vertical
                                ? vec2{ static_cast<float>( normal ), static_cast<float>( clipped.midpoint ) }
                                : vec2{ static_cast<float>( clipped.midpoint ), static_cast<float>( normal ) };
                            const vec2 velocity = linearSpeeds[i] +
                                Cross( angularSpeeds[i], point - intermediate[i].transform.position );
                            flux += clipped.length * ( vertical ? velocity.x : velocity.y );
                        };
                        if( analyticCutCellMovingSweptFacesEnabled_ && disjointShapes.size() > 2 )
                        {
                            // For 3+ disjoint analytic shapes, zero-transform-motion
                            // bodies cannot contribute to face-swept volume. Keep
                            // input order and reclip intermediate poses each time;
                            // two-circle overlap/containment still uses ownership.
                            for( int owner : movingOwners )
                            {
                                const auto clipped = ClipDisjointShapeFace(
                                    disjointShapes[owner], vertical, normal, low, low + h );
                                if( clipped.length > 0.0 ) addPortion( owner, clipped );
                            }
                        }
                        else
                            ForEachAnalyticFacePortion( disjointShapes, vertical, normal, low, low + h,
                                addPortion );
                        const float swept = intervalDt * static_cast<float>( flux );
                        if( vertical ) sweptU[uIndex( x, y )] += swept;
                        else sweptV[vIndex( x, y )] += swept;
                    };
                    for( int y = 0; y < height_; ++y )
                        for( int x = 1; x < width_; ++x ) accumulateFace( true, x, y );
                    for( int y = 1; y < height_; ++y )
                        for( int x = 0; x < width_; ++x ) accumulateFace( false, x, y );
                }
                continue;
            }

            analyticFluidCircle circle{};
            analyticFluidPolygon polygon{};
            analyticFluidCapsule capsule{};
            analyticFluidRoundedPolygon rounded{};
            const bool circleGeometry = analyticCircleCutCellEnabled_ &&
                TryGetAnalyticCircle( intermediate, circle );
            const bool polygonGeometry = !circleGeometry && analyticPolygonCutCellEnabled_ &&
                TryGetAnalyticPolygon( intermediate, polygon );
            const bool capsuleGeometry = !circleGeometry && !polygonGeometry &&
                analyticCapsuleCutCellEnabled_ && TryGetAnalyticCapsule( intermediate, capsule );
            const bool roundedGeometry = !circleGeometry && !polygonGeometry && !capsuleGeometry &&
                analyticRoundedPolygonCutCellEnabled_ && TryGetAnalyticRoundedPolygon( intermediate, rounded );
            if( circleGeometry || polygonGeometry || capsuleGeometry || roundedGeometry )
            {
                const double h = cellSize_;
                const float intervalDt = timeStep / samples;
                const fluidShapeObstacle2& shape = intermediate.front();
                const auto sweptVelocity = [&]( vec2 point, bool vertical )
                {
                    const vec2 velocity = linearSpeeds.front() +
                        Cross( angularSpeeds.front(), point - shape.transform.position );
                    return vertical ? velocity.x : velocity.y;
                };
                for( int y = 0; y < height_; ++y )
                    for( int x = 1; x < width_; ++x )
                    {
                        const double normal = origin_.x + x * h;
                        const double low = origin_.y + y * h;
                        const auto clipped = circleGeometry
                            ? ClipCircleFace( circle, true, normal, low, low + h )
                            : polygonGeometry
                                ? ClipPolygonFace( polygon, true, normal, low, low + h )
                                : capsuleGeometry
                                    ? ClipCapsuleFace( capsule, true, normal, low, low + h )
                                    : ClipRoundedPolygonFace( rounded, true, normal, low, low + h );
                        if( clipped.length <= 0.0 ) continue;
                        const vec2 point{ static_cast<float>( normal ),
                            static_cast<float>( clipped.midpoint ) };
                        sweptU[uIndex( x, y )] += intervalDt *
                            static_cast<float>( clipped.length ) * sweptVelocity( point, true );
                    }
                for( int y = 1; y < height_; ++y )
                    for( int x = 0; x < width_; ++x )
                    {
                        const double normal = origin_.y + y * h;
                        const double low = origin_.x + x * h;
                        const auto clipped = circleGeometry
                            ? ClipCircleFace( circle, false, normal, low, low + h )
                            : polygonGeometry
                                ? ClipPolygonFace( polygon, false, normal, low, low + h )
                                : capsuleGeometry
                                    ? ClipCapsuleFace( capsule, false, normal, low, low + h )
                                    : ClipRoundedPolygonFace( rounded, false, normal, low, low + h );
                        if( clipped.length <= 0.0 ) continue;
                        const vec2 point{ static_cast<float>( clipped.midpoint ),
                            static_cast<float>( normal ) };
                        sweptV[vIndex( x, y )] += intervalDt *
                            static_cast<float>( clipped.length ) * sweptVelocity( point, false );
                    }
                continue;
            }

            const auto normalSpeed = [&]( vec2 point, bool horizontal )
            {
                for( std::size_t i = 0; i < intermediate.size(); ++i )
                {
                    const vec2 center = intermediate[i].transform.position;
                    // 미리 계산한 보수적 geometry radius: 원거리 static obstacle Contains 검사 생략.
                    if( std::abs( static_cast<double>( point.x ) - center.x ) > bounds[i] ||
                        std::abs( static_cast<double>( point.y ) - center.y ) > bounds[i] ) continue;
                    if( !ContainsFluidObstacle( intermediate[i], point ) ) continue;
                    const vec2 velocity = linearSpeeds[i] +
                        Cross( angularSpeeds[i], point - center );
                    return horizontal ? velocity.x : velocity.y;
                }
                return 0.0f;
            };
            for( int y = 0; y < height_; ++y )
                for( int x = 1; x < width_; ++x )
                    for( int sample = 0; sample < faceSamples; ++sample )
                    {
                        const vec2 point = origin_ + cellSize_ * vec2{
                            static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) / faceSamples };
                        sweptU[uIndex( x, y )] += weight * normalSpeed( point, true );
                    }
            for( int y = 1; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                    for( int sample = 0; sample < faceSamples; ++sample )
                    {
                        const vec2 point = origin_ + cellSize_ * vec2{
                            static_cast<float>( x ) + ( sample + 0.5f ) / faceSamples, static_cast<float>( y ) };
                        sweptV[vIndex( x, y )] += weight * normalSpeed( point, false );
                    }
        }
    };

    integrate( timeSamples, sweptUVolume_, sweptVVolume_ );
    if( adaptiveSweptSamplingEnabled_ )
    {
        // 스냅샷의 sub-cell ΔV와 swept face divergence가 최대한 일치하는
        // 표본만 채택한다. face마다 임의 보정하지 않고 공유 face 유량을 유지.
        const double area = static_cast<double>( cellSize_ ) * cellSize_;
        const auto mismatch = [&]( const std::vector<float>& sweptU, const std::vector<float>& sweptV )
        {
            double sum = 0.0;
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int cell = cellIndex( x, y );
                    const double snapshot =
                        static_cast<double>( cellFluidFraction_[cell] - previousCellFluidFraction_[cell] ) * area;
                    const double flux =
                        static_cast<double>( sweptU[uIndex( x + 1, y )] ) - sweptU[uIndex( x, y )] +
                        static_cast<double>( sweptV[vIndex( x, y + 1 )] ) - sweptV[vIndex( x, y )];
                    sum += std::abs( snapshot - flux );
                }
            return sum;
        };

        double bestMismatch = mismatch( sweptUVolume_, sweptVVolume_ );
        sweptFaceFlux_.initialTimeSamples = timeSamples;
        sweptFaceFlux_.initialGeometryMismatch = static_cast<float>( bestMismatch );
        if( bestMismatch > 1.0e-7 )
        {
            std::vector<float> trialU( sweptUVolume_.size() );
            std::vector<float> trialV( sweptVVolume_.size() );
            // 적응 검사는 opt-in에서만 수행. 개선이 멈추면 더 비싼 샘플을 실행하지 않는다.
            for( int refine = 0; refine < 3 && timeSamples < maxTimeSamples; ++refine )
            {
                const int candidateSamples = std::min( 2 * timeSamples, maxTimeSamples );
                integrate( candidateSamples, trialU, trialV );
                const double candidateMismatch = mismatch( trialU, trialV );
                const double minimumImprovement = std::max( 1.0e-7, bestMismatch * 1.0e-4 );
                if( candidateMismatch + minimumImprovement >= bestMismatch ) break;
                sweptUVolume_.swap( trialU );
                sweptVVolume_.swap( trialV );
                bestMismatch = candidateMismatch;
                timeSamples = candidateSamples;
                ++sweptFaceFlux_.refinements;
            }
        }
    }

    double absoluteFlux = 0.0;
    for( const float flux : sweptUVolume_ ) absoluteFlux += std::abs( flux );
    for( const float flux : sweptVVolume_ ) absoluteFlux += std::abs( flux );
    sweptFaceFlux_.timeSamples = timeSamples;
    sweptFaceFlux_.totalAbsoluteFlux = static_cast<float>( absoluteFlux );
}

void fluidField2::reconcileSweptSolidFlux()
{
    // 기본 Cut-cell 경로에 잔차 vector 할당/추가 순회를 발생시키지 않는다.
    // 보정 미사용 시 raw 값은 prepareCutCellVolumeChange의 최종 관측값과 같다.
    if( !sweptFaceFlux_.usedSweptFlux || !sweptGeometryCorrectionEnabled_ ) return;

    // 순수 기하 discrepancy: 시작/종료 점유율 변화와 경로 적분된 공유 면
    // 체적의 divergence 간 차이. 외벽의 swept flux는 항상 0.
    const double area = static_cast<double>( cellSize_ ) * cellSize_;
    std::vector<double> residual( cellFluidFraction_.size() );
    double mismatch = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            const double snapshot = static_cast<double>(
                cellFluidFraction_[cell] - previousCellFluidFraction_[cell] ) * area;
            const double swept =
                static_cast<double>( sweptUVolume_[uIndex( x + 1, y )] ) -
                static_cast<double>( sweptUVolume_[uIndex( x, y )] ) +
                static_cast<double>( sweptVVolume_[vIndex( x, y + 1 )] ) -
                static_cast<double>( sweptVVolume_[vIndex( x, y )] );
            residual[cell] = snapshot - swept;
            mismatch += std::abs( residual[cell] );
        }
    sweptFaceFlux_.rawGeometryMismatch = static_cast<float>( mismatch );
    // 서로 부호가 반대인 이웃의 discrepancy만 공유 edge에서 국소 상쇄.
    // 각 update는 두 셀의 잔차를 같은 크기·반대 부호로 변경해
    // 전체 체적 부호합 및 닫힌 외벽을 보존하고 L1 오차를 증가시키지 않는다.
    // 더 먼 이웃까지 강제로 전달하거나 total ΔV의 비영(非零) 합을 지우지 않는다.
    const auto reconcile = [&]( int left, int right, float& face )
    {
        const double a = residual[left];
        const double b = residual[right];
        double delta = 0.0;
        if( a > 0.0 && b < 0.0 ) delta = std::min( a, -b );
        else if( a < 0.0 && b > 0.0 ) delta = -std::min( -a, b );
        if( delta == 0.0 ) return;

        const double before = face;
        double next = before + delta;
        // 원래 측정된 면 속도를 정반대로 뒤집지 않음.
        if( before != 0.0 && ( before > 0.0 ) != ( next > 0.0 ) ) next = 0.0;
        if( !std::isfinite( next ) || std::abs( next ) > std::numeric_limits<float>::max() ) return;
        face = static_cast<float>( next );
        const double applied = static_cast<double>( face ) - before;
        if( applied == 0.0 ) return;
        residual[left] -= applied;
        residual[right] += applied;
        sweptFaceFlux_.correctionAbsoluteFlux += static_cast<float>( std::abs( applied ) );
        ++sweptFaceFlux_.correctedFaces;
    };

    // 변한 셀에 인접하거나 원래 swept flux가 있는 면만 보정한다.
    // 먼 정적 영역을 통과하는 전역적 보정 흐름은 생성하지 않는다.
    constexpr int passes = 4;
    for( int pass = 0; pass < passes; ++pass )
    {
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                float& face = sweptUVolume_[uIndex( x, y )];
                if( face == 0.0f &&
                    cellFluidFraction_[left] == previousCellFluidFraction_[left] &&
                    cellFluidFraction_[right] == previousCellFluidFraction_[right] ) continue;
                reconcile( left, right, face );
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                float& face = sweptVVolume_[vIndex( x, y )];
                if( face == 0.0f &&
                    cellFluidFraction_[bottom] == previousCellFluidFraction_[bottom] &&
                    cellFluidFraction_[top] == previousCellFluidFraction_[top] ) continue;
                reconcile( bottom, top, face );
            }
    }

    // final geometryMismatch는 기존 prepareCutCellVolumeChange가 새 face를
    // 이용해 계산한다. rawGeometryMismatch와 직접 비교할 수 있다.
}

void fluidField2::prepareCutCellVolumeChange( float timeStep )
{
    std::fill( cutCellVolumeRate_.begin(), cutCellVolumeRate_.end(), 0.0f );
    cutCellPressure_.hasVolumeHistory = volumeSourceValid_;
    if( !volumeSourceValid_ ) return;

    // Δfraction = (new fluid area - old fluid area) / h². 최초 배치에는 적용하지 않는다.
    // 현재 완전히 solid가 된 cell에는 압력 미지수가 없으므로 가장 가까운 활성
    // fluid cell로 해당 Δfraction을 이송한다. 이는 swept 경로가 아닌 공간 근사다.
    cutCellQueue_.clear();
    std::fill( cutCellNearestFluid_.begin(), cutCellNearestFluid_.end(), -1 );
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int index = cellIndex( x, y );
            if( cellFluidFraction_[index] <= 0.0f ) continue;
            cutCellNearestFluid_[index] = index;
            cutCellQueue_.push_back( index );
        }

    for( std::size_t read = 0; read < cutCellQueue_.size(); ++read )
    {
        const int index = cutCellQueue_[read];
        const int x = index % width_;
        const int y = index / width_;
        const auto visit = [&]( int neighbor )
        {
            if( cutCellNearestFluid_[neighbor] >= 0 ) return;
            cutCellNearestFluid_[neighbor] = cutCellNearestFluid_[index];
            cutCellQueue_.push_back( neighbor );
        };
        if( x > 0 ) visit( index - 1 );
        if( x + 1 < width_ ) visit( index + 1 );
        if( y > 0 ) visit( index - width_ );
        if( y + 1 < height_ ) visit( index + width_ );
    }

    double volumeRate = 0.0;
    double relocatedRate = 0.0;
    double unresolvedRate = 0.0;
    const double scale = static_cast<double>( cellSize_ ) * cellSize_ / timeStep;
    for( std::size_t i = 0; i < cellFluidFraction_.size(); ++i )
    {
        const float snapshotChange = cellFluidFraction_[i] - previousCellFluidFraction_[i];
        volumeRate += static_cast<double>( snapshotChange ) * scale;
        const int x = static_cast<int>( i ) % width_;
        const int y = static_cast<int>( i ) / width_;
        const float change = sweptFaceFlux_.usedSweptFlux
            ? ( sweptUVolume_[uIndex( x + 1, y )] - sweptUVolume_[uIndex( x, y )] +
                sweptVVolume_[vIndex( x, y + 1 )] - sweptVVolume_[vIndex( x, y )] ) /
                ( cellSize_ * cellSize_ )
            : snapshotChange;
        if( sweptFaceFlux_.usedSweptFlux )
            sweptFaceFlux_.geometryMismatch += std::abs( change - snapshotChange ) * cellSize_ * cellSize_;
        if( cellFluidFraction_[i] > 0.0f )
            cutCellVolumeRate_[i] += change;
        else if( change != 0.0f )
        {
            const int destination = cutCellNearestFluid_[i];
            if( destination >= 0 )
            {
                cutCellVolumeRate_[destination] += change;
                relocatedRate += static_cast<double>( change ) * scale;
            }
            else unresolvedRate += static_cast<double>( change ) * scale;
        }
    }
    if( sweptFaceFlux_.usedSweptFlux && !sweptGeometryCorrectionEnabled_ )
        sweptFaceFlux_.rawGeometryMismatch = sweptFaceFlux_.geometryMismatch;
    cutCellPressure_.volumeRate = static_cast<float>( volumeRate );
    cutCellPressure_.redistributedVolumeRate = static_cast<float>( relocatedRate );
    cutCellPressure_.unresolvedVolumeRate = static_cast<float>( unresolvedRate );
}

std::optional<std::vector<fluidObstacleForce2>> fluidField2::replayFrozenCutCellPressure()
{
    if( !frozenPressureReplayReady_ || !cutCellProjectionEnabled_ ||
        !std::isfinite( frozenPressureTimeStep_ ) || frozenPressureTimeStep_ <= 0.0f )
        return std::nullopt;
    frozenPressureReplayReady_ = false;
    std::vector<fluidObstacleForce2> feedback( frozenPressureObstacles_.size() );
    pressureMomentum_ = {};
    projectCutCell( frozenPressureTimeStep_, frozenPressureIterations_,
        frozenPressureObstacles_, feedback );
    stepMomentum_.afterPressure = getFluidMomentum();
    return feedback;
}

void fluidField2::projectCutCell( float timeStep, int pressureIterations,
    std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback )
{
    // 유체 점유 면적의 시간 변화는 다음 보존형 연속 방정식에 직접 들어간다:
    // div(a*u_fluid) + ΔfluidFraction / dt = 0.
    // 차단된 face의 solid 속도를 여기에 다시 더하면 경계 이동을 이중 계상하므로
    // face의 유체 개방율 a만 압력 gradient/flux에 적용한다.
    const float inverseCell = 1.0f / cellSize_;
    const float pressureScale = cellSize_ * cellSize_ / timeStep;
    const float gradientScale = timeStep * inverseCell;
    cutCellPressure_ = {};
    prepareCutCellVolumeChange( timeStep );
    cutCellPressure_.divergenceBefore = cutCellFluxDivergenceRms( 1.0f / timeStep );
    double sumDivergence = 0.0;

    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] <= 0.0f )
            {
                divergence_[cell] = 0.0f;
                continue;
            }
            const auto fluxU = [&]( int i ) { return cutCellUOpen( i, y ) * u_[uIndex( i, y )]; };
            const auto fluxV = [&]( int j ) { return cutCellVOpen( x, j ) * v_[vIndex( x, j )]; };
            divergence_[cell] = ( fluxU( x + 1 ) - fluxU( x ) +
                fluxV( y + 1 ) - fluxV( y ) ) * inverseCell +
                cutCellVolumeRate_[cell] / timeStep;
            sumDivergence += divergence_[cell];
            ++cutCellPressure_.activeCells;
        }

    // 닫힌 외벽과 고체 사이의 압력 Poisson은 각 연결 영역마다 Neumann 문제다.
    // 전체 평균만 빼면 서로 분리된 유체 포켓의 독립적인 질량 불일치를 풀 수 없다.
    // 개방 face로 연결된 영역별 source 평균을 빼고 그 크기를 진단값으로 남긴다.
    cutCellPressure_.sourceImbalance =
        static_cast<float>( sumDivergence * cellSize_ * cellSize_ ) + cutCellPressure_.unresolvedVolumeRate;
    std::fill( cutCellComponent_.begin(), cutCellComponent_.end(), -1 );
    std::vector<double> regionSums;
    std::vector<int> regionSizes;
    cutCellQueue_.clear();
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int start = cellIndex( x, y );
            if( cellFluidFraction_[start] <= 0.0f || cutCellComponent_[start] >= 0 ) continue;
            const int region = static_cast<int>( regionSums.size() );
            regionSums.push_back( 0.0 );
            regionSizes.push_back( 0 );
            cutCellQueue_.clear();
            cutCellQueue_.push_back( start );
            cutCellComponent_[start] = region;

            for( std::size_t read = 0; read < cutCellQueue_.size(); ++read )
            {
                const int index = cutCellQueue_[read];
                const int cx = index % width_;
                const int cy = index / width_;
                regionSums[region] += divergence_[index];
                ++regionSizes[region];
                const auto visit = [&]( int neighbor, float opening )
                {
                    if( opening <= 0.0f || cutCellComponent_[neighbor] >= 0 ) return;
                    cutCellComponent_[neighbor] = region;
                    cutCellQueue_.push_back( neighbor );
                };
                if( cx > 0 ) visit( index - 1, cutCellUOpen( cx, cy ) );
                if( cx + 1 < width_ ) visit( index + 1, cutCellUOpen( cx + 1, cy ) );
                if( cy > 0 ) visit( index - width_, cutCellVOpen( cx, cy ) );
                if( cy + 1 < height_ ) visit( index + width_, cutCellVOpen( cx, cy + 1 ) );
            }
        }
    cutCellPressure_.connectedRegions = static_cast<int>( regionSums.size() );
    double incompatible = std::abs( static_cast<double>( cutCellPressure_.unresolvedVolumeRate ) );
    for( const double sum : regionSums ) incompatible += std::abs( sum ) * cellSize_ * cellSize_;
    cutCellPressure_.compatibilityCorrection = static_cast<float>( incompatible );

    for( std::size_t i = 0; i < divergence_.size(); ++i )
    {
        const int region = cutCellComponent_[i];
        if( region >= 0 )
            divergence_[i] -= static_cast<float>( regionSums[region] / regionSizes[region] );
    }

    // Cut-cell + 질량 결합은 별도 선택 경로. PCG가 실패하면 명시적 Jacobi
    // 압력 반력을 Dynamic Body에 적용하지 않고 해당 Step을 격리한다.
    const bool coupled = cutCellCoupledProjectionEnabled_ &&
        std::any_of( obstacles.begin(), obstacles.end(), []( const fluidShapeObstacle2& obstacle )
        {
            return obstacle.inverseMass > 0.0f || obstacle.inverseInertia > 0.0f;
        } );
    const auto solveStart = cutCellPressureTimingEnabled_ && coupled
        ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    const bool solvedCoupled = coupled && solveCoupledPressure( timeStep, pressureIterations, obstacles, true );
    if( cutCellPressureTimingEnabled_ && coupled )
        cutCellPressure_.coupledSolveMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - solveStart ).count();
    cutCellPressure_.usedCoupledProjection = solvedCoupled;
    cutCellPressure_.coupledStatus = coupled ? pressureMomentum_.coupledStatus :
        fluidCoupledPressureStatus2::notRequested;
    cutCellPressure_.coupledIterations = coupled ? pressureMomentum_.coupledIterations : 0;
    cutCellPressure_.coupledRelativeResidual = coupled ? pressureMomentum_.coupledRelativeResidual : 0.0f;
    cutCellPressure_.usedIncompleteCholesky = coupled && pressureMomentum_.usedIncompleteCholesky;
    cutCellPressure_.coupledTiming = coupled ? pressureMomentum_.coupledTiming : fluidCoupledPressureTiming2{};
    cutCellPressure_.coupledSolveFailed = coupled && !solvedCoupled &&
        cutCellPressure_.coupledStatus != fluidCoupledPressureStatus2::noBoundary;
    const float coupledPressureResidual = pressureMomentum_.coupledFinalResidualRms;
    pressureMomentum_ = {}; // 이진 MAC 압력 운동량 장부를 Cut-cell의 반력과 혼합하지 않는다.
    // 분석적 multi-shape Schur 경로에서만 압력 impulse 장부를 기록한다.
    // 면 개방율과 실제 MAC Δu/Δv를 곱하고, pressure feedback/외벽과 대조.
    const bool auditImpulse = solvedCoupled && cutCellTransition_.usedDisjointAnalyticGeometry;
    const double dualFaceMass = static_cast<double>( cellSize_ ) * cellSize_;
    double fluidImpulseX = 0.0;
    double fluidImpulseY = 0.0;

    if( cutCellPressure_.coupledSolveFailed )
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    else if( !solvedCoupled )
    {
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
        for( int iteration = 0; iteration < pressureIterations; ++iteration )
        {
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int cell = cellIndex( x, y );
                    if( cellFluidFraction_[cell] <= 0.0f ) { nextPressure_[cell] = 0.0f; continue; }
                    const float left = cutCellUOpen( x, y );
                    const float right = cutCellUOpen( x + 1, y );
                    const float bottom = cutCellVOpen( x, y );
                    const float top = cutCellVOpen( x, y + 1 );
                    const float diagonal = left + right + bottom + top;
                    const float sum =
                        ( left > 0.0f ? left * pressure_[cellIndex( x - 1, y )] : 0.0f ) +
                        ( right > 0.0f ? right * pressure_[cellIndex( x + 1, y )] : 0.0f ) +
                        ( bottom > 0.0f ? bottom * pressure_[cellIndex( x, y - 1 )] : 0.0f ) +
                        ( top > 0.0f ? top * pressure_[cellIndex( x, y + 1 )] : 0.0f );
                    nextPressure_[cell] = diagonal > 0.0f
                        ? ( sum - divergence_[cell] * pressureScale ) / diagonal : 0.0f;
                }
            pressure_.swap( nextPressure_ );
        }
    }

    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const float open = cutCellUOpen( x, y );
            if( open <= 0.0f ) continue;
            float& velocity = u_[uIndex( x, y )];
            const float before = velocity;
            velocity -= gradientScale *
                ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x - 1, y )] );
            if( auditImpulse )
                fluidImpulseX += dualFaceMass * open *
                    ( static_cast<double>( velocity ) - before );
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const float open = cutCellVOpen( x, y );
            if( open <= 0.0f ) continue;
            float& velocity = v_[vIndex( x, y )];
            const float before = velocity;
            velocity -= gradientScale *
                ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x, y - 1 )] );
            if( auditImpulse )
                fluidImpulseY += dualFaceMass * open *
                    ( static_cast<double>( velocity ) - before );
        }

    cutCellPressure_.divergenceAfter = cutCellFluxDivergenceRms( 1.0f / timeStep );
    double squaredResidual = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] == 0.0f ) continue;
            const float p = pressure_[cell];
            const float sum = cutCellUOpen( x, y ) *
                    ( x > 0 ? pressure_[cellIndex( x - 1, y )] - p : 0.0f ) +
                cutCellUOpen( x + 1, y ) *
                    ( x + 1 < width_ ? pressure_[cellIndex( x + 1, y )] - p : 0.0f ) +
                cutCellVOpen( x, y ) *
                    ( y > 0 ? pressure_[cellIndex( x, y - 1 )] - p : 0.0f ) +
                cutCellVOpen( x, y + 1 ) *
                    ( y + 1 < height_ ? pressure_[cellIndex( x, y + 1 )] - p : 0.0f );
            const double residual = static_cast<double>(
                sum - divergence_[cell] * pressureScale ) * ( timeStep / cellSize_ );
            squaredResidual += residual * residual;
        }
    // 결합 PCG의 압력 방정식 잔차는 dt/h²를 곱하면 연속성 잔차 (1/s)다.
    // 기존 Jacobi의 legacy pressureResidual 표시 단위는 이 단계에서 변경하지 않는다.
    cutCellPressure_.pressureResidual = solvedCoupled
        ? coupledPressureResidual * ( timeStep / ( cellSize_ * cellSize_ ) ) :
        ( cutCellPressure_.activeCells > 0
            ? static_cast<float>( std::sqrt( squaredResidual / cutCellPressure_.activeCells ) ) : 0.0f );

    // 수렴한 Schur 해에서는 행렬에 들어간 B 자체를 pressure traction으로
    // 사용한다. 별도의 COM 근사 압력 적분은 중복하면 안 된다.
    if( solvedCoupled )
    {
        for( const coupledPressureFace& face : coupledPressureScratch_.faces )
        {
            const double p = pressure_[face.cell];
            fluidObstacleForce2& force = feedback[face.owner];
            force.force += static_cast<float>( p ) *
                vec2{ static_cast<float>( face.fx ), static_cast<float>( face.fy ) };
            force.torque += static_cast<float>( p * face.torque );
        }
        if( auditImpulse )
        {
            double forceX = 0.0;
            double forceY = 0.0;
            for( const fluidObstacleForce2& body : feedback )
            {
                forceX += body.force.x;
                forceY += body.force.y;
            }
            // 외벽은 닫힌 격자 경계의 압력 traction. Pressure 미지수가 없는
            // 완전 고체 셀은 압력 0이며, 모든 내부 open/blocked face는 상쇄된다.
            double wallX = 0.0;
            double wallY = 0.0;
            for( int y = 0; y < height_; ++y )
            {
                wallX -= static_cast<double>( pressure_[cellIndex( 0, y )] ) * cellSize_;
                wallX += static_cast<double>( pressure_[cellIndex( width_ - 1, y )] ) * cellSize_;
            }
            for( int x = 0; x < width_; ++x )
            {
                wallY -= static_cast<double>( pressure_[cellIndex( x, 0 )] ) * cellSize_;
                wallY += static_cast<double>( pressure_[cellIndex( x, height_ - 1 )] ) * cellSize_;
            }
            cutCellPressure_.hasAnalyticImpulseBudget = true;
            cutCellPressure_.impulse.fluid = {
                static_cast<float>( fluidImpulseX ), static_cast<float>( fluidImpulseY ) };
            cutCellPressure_.impulse.solid = timeStep * vec2{
                static_cast<float>( forceX ), static_cast<float>( forceY ) };
            cutCellPressure_.impulse.wall = timeStep * vec2{
                static_cast<float>( wallX ), static_cast<float>( wallY ) };
        }
        synchronizeCutCellBoundaries( timeStep, obstacles, feedback );

        // 동일 sub-face 교환의 선형화된 volume source Δ(f)/dt를 추가한다.
        // 마지막 geometry snapshot 자체를 변경하지 않으므로 다음 Step의
        // swept volume·tracer mass와 이 예측 증분을 이중 적용하지 않는다.
        double squaredContinuity = 0.0;
        double volumeL1 = 0.0;
        // P^T continuity must be checked in the SAME pooled groups as
        // P^T A P. A tiny child cell need not independently satisfy div=0.
        const bool groupedPressure = cutCellPressure_.mergedPressureCells > 0;
        auto& groupError = coupledPressureScratch_.smallCellReducedPreconditioner;
        if( groupedPressure ) groupError.assign( pressure_.size(), 0.0 );
        for( int y = 0; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int cell = cellIndex( x, y );
                if( cellFluidFraction_[cell] <= 0.0f ) continue;
                const float fluidFlux =
                    cutCellUOpen( x + 1, y ) * u_[uIndex( x + 1, y )] -
                    cutCellUOpen( x, y ) * u_[uIndex( x, y )] +
                    cutCellVOpen( x, y + 1 ) * v_[vIndex( x, y + 1 )] -
                    cutCellVOpen( x, y ) * v_[vIndex( x, y )];
                const float solidFlux =
                    pressureUSolidFlux_[uIndex( x + 1, y )] - pressureUSolidFlux_[uIndex( x, y )] +
                    pressureVSolidFlux_[vIndex( x, y + 1 )] - pressureVSolidFlux_[vIndex( x, y )];
                const int region = cutCellComponent_[cell];
                const double mean = region >= 0 ? regionSums[region] / regionSizes[region] : 0.0;
                const double error = static_cast<double>( fluidFlux + solidFlux ) / cellSize_ +
                    static_cast<double>( cutCellVolumeRate_[cell] ) / timeStep - mean;
                squaredContinuity += error * error;
                if( groupedPressure )
                {
                    const int root = coupledPressureScratch_.smallCellPressureRoots[cell];
                    groupError[root] += error;
                    if( root != cell )
                        cutCellPressure_.virtualInternalExchangeVolume +=
                            static_cast<float>( std::abs( error ) * timeStep * cellSize_ * cellSize_ );
                }
                volumeL1 += std::abs( static_cast<double>( solidFlux ) ) * cellSize_ * timeStep;
            }
        if( groupedPressure )
        {
            double totalSquared = 0.0;
            const auto& roots = coupledPressureScratch_.smallCellPressureRoots;
            for( std::size_t i = 0; i < roots.size(); ++i )
                if( cellFluidFraction_[i] > 0.0f &&
                    roots[i] == static_cast<int>( i ) ) totalSquared += groupError[i] * groupError[i];
            const int groups = cutCellPressure_.activeCells - cutCellPressure_.mergedPressureCells;
            if( groups > 0 )
                cutCellPressure_.groupedPredictedContinuityRms =
                    static_cast<float>( std::sqrt( totalSquared / groups ) );
        }
        cutCellPressure_.predictedContinuityRms = cutCellPressure_.activeCells > 0
            ? static_cast<float>( std::sqrt( squaredContinuity / cutCellPressure_.activeCells ) ) : 0.0f;
        cutCellPressure_.predictedSolidVolumeL1 = static_cast<float>( volumeL1 );
        return;
    }

    // 비결합 Cut-cell의 기존 셀 aperture/nearest COM 표면 반력 경로.
    if( obstacles.empty() ) return;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] == 0.0f ) continue;
            const float l = x == 0 ? cellFluidFraction_[cell] : cutCellUOpen( x, y );
            const float r = x == width_ - 1 ? cellFluidFraction_[cell] : cutCellUOpen( x + 1, y );
            const float b = y == 0 ? cellFluidFraction_[cell] : cutCellVOpen( x, y );
            const float t = y == height_ - 1 ? cellFluidFraction_[cell] : cutCellVOpen( x, y + 1 );
            const vec2 force = -pressure_[cell] * cellSize_ * vec2{ r - l, t - b };
            if( LengthSquared( force ) == 0.0f ) continue;
            const vec2 center = getCellCenter( x, y );
            std::size_t owner = 0;
            float nearest = std::numeric_limits<float>::max();
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                if( !IsValidFluidObstacle( obstacles[i] ) ) continue;
                const float distance = LengthSquared( center - obstacles[i].center );
                if( distance < nearest ) { nearest = distance; owner = i; }
            }
            if( nearest == std::numeric_limits<float>::max() ) continue;
            feedback[owner].force += force;
            feedback[owner].torque += Cross( center - obstacles[owner].center, force );
        }
}

bool fluidField2::solveCoupledPressure( float timeStep, int iterations,
    std::span<const fluidShapeObstacle2> obstacles, bool cutCell )
{
    // 두 모드 모두 대칭인 A = L + B M_body^-1 B^T를 PCG로 푼다.
    // Cut-cell L은 개방 면적 가중치, B는 반환하는 표면 압력 반력과 동일한 계수다.
    // 정확한 cut interface 적분이 아닌 기존 cell aperture traction 근사만 결합한다.
    pressureMomentum_.coupledIterationLimit = iterations;
    // One opt-in clock for the whole matrix-free Schur solve. PCG operation
    // counters and their timings are recorded only during explicit profiling.
    using clock = std::chrono::steady_clock;
    const bool profile = cutCellPressureTimingEnabled_;
    auto& timing = pressureMomentum_.coupledTiming;
    timing = {};
    const auto started = profile ? clock::now() : clock::time_point{};
    auto phaseStarted = started;
    enum class phase { assembly, factorization, pcg, validation, output, done };
    phase current = phase::assembly;
    const auto nextPhase = [&]( phase next )
    {
        if( !profile ) return;
        const auto now = clock::now();
        const double elapsed = std::chrono::duration<double, std::milli>( now - phaseStarted ).count();
        switch( current )
        {
        case phase::assembly: timing.assembly += elapsed; break;
        case phase::factorization: timing.factorization += elapsed; break;
        case phase::pcg:
            timing.vector += std::max( 0.0, elapsed - timing.multiply - timing.preconditioner );
            break;
        case phase::validation: timing.validation += elapsed; break;
        case phase::output: timing.output += elapsed; break;
        case phase::done: break;
        }
        if( next == phase::done )
            timing.total = std::chrono::duration<double, std::milli>( now - started ).count();
        current = next;
        phaseStarted = now;
    };
    const auto fail = [&]( fluidCoupledPressureStatus2 status )
    {
        pressureMomentum_.coupledStatus = status;
        nextPhase( phase::done );
        return false;
    };
    auto& scratch = coupledPressureScratch_;
    auto& faces = scratch.faces;
    faces.clear();
    auto& diagonal = scratch.diagonal;
    diagonal.assign( pressure_.size(), 0.0 );
    int active = 0;
    double rhsSum = 0.0;
    const double scale = static_cast<double>( cellSize_ ) * cellSize_ / timeStep;
    auto& rhs = scratch.rhs;
    rhs.assign( pressure_.size(), 0.0 );
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cutCell ? cellFluidFraction_[cell] <= 0.0f : solidOwner_[cell] >= 0 ) continue;
            const double count = cutCell
                ? cutCellUOpen( x, y ) + cutCellUOpen( x + 1, y ) +
                    cutCellVOpen( x, y ) + cutCellVOpen( x, y + 1 )
                : ( x > 0 && !isSolidCell( x - 1, y ) ? 1.0 : 0.0 ) +
                    ( x + 1 < width_ && !isSolidCell( x + 1, y ) ? 1.0 : 0.0 ) +
                    ( y > 0 && !isSolidCell( x, y - 1 ) ? 1.0 : 0.0 ) +
                    ( y + 1 < height_ && !isSolidCell( x, y + 1 ) ? 1.0 : 0.0 );
            diagonal[cell] = count;
            rhs[cell] = -static_cast<double>( divergence_[cell] ) * scale;
            rhsSum += rhs[cell];
            ++active;
        }
    if( active == 0 )
    {
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
        pressureMomentum_.coupledStatus = fluidCoupledPressureStatus2::converged;
        nextPhase( phase::done );
        return true;
    }

    // Experimental virtual pressure-cell merging: each tiny active cell
    // pools its pressure with the most-open directly connected neighbor that
    // is at least the chosen fluid-fraction threshold. Never cross a closed
    // face or join two slivers into a cycle. All original face geometry and
    // swept source remain untouched.
    auto& pressureRoot = scratch.smallCellPressureRoots;
    int mergedCells = 0;
    if( cutCell && smallCellPressureMergeFraction_ > 0.0f )
    {
        pressureRoot.resize( rhs.size() );
        for( std::size_t i = 0; i < rhs.size(); ++i )
            pressureRoot[i] = static_cast<int>( i );
        for( int y = 0; y < height_; ++y )
            for( int xCell = 0; xCell < width_; ++xCell )
            {
                const int cell = cellIndex( xCell, y );
                const float fraction = cellFluidFraction_[cell];
                if( fraction <= 0.0f || fraction >= smallCellPressureMergeFraction_ ||
                    !volumeSourceValid_ || previousCellFluidFraction_[cell] <= fraction ) continue;
                int neighbor = -1;
                float bestOpening = 0.0f;
                const auto visit = [&]( int other, float opening )
                {
                    if( opening <= bestOpening ||
                        cellFluidFraction_[other] < smallCellPressureMergeFraction_ ) return;
                    neighbor = other;
                    bestOpening = opening;
                };
                if( xCell > 0 ) visit( cell - 1, cutCellUOpen( xCell, y ) );
                if( xCell + 1 < width_ ) visit( cell + 1, cutCellUOpen( xCell + 1, y ) );
                if( y > 0 ) visit( cell - width_, cutCellVOpen( xCell, y ) );
                if( y + 1 < height_ ) visit( cell + width_, cutCellVOpen( xCell, y + 1 ) );
                if( neighbor >= 0 )
                {
                    pressureRoot[cell] = neighbor;
                    ++mergedCells;
                    cutCellPressure_.mergedFluidFraction += fraction;
                }
                else ++cutCellPressure_.unmergedSmallPressureCells;
            }
        // P^T*b: accumulate every cell's original conservation RHS, including
        // tiny swept-volume sources, in the retained neighbor's pressure row.
        for( std::size_t i = 0; i < rhs.size(); ++i )
            if( pressureRoot[i] != static_cast<int>( i ) )
            {
                rhs[pressureRoot[i]] += rhs[i];
                rhs[i] = 0.0;
            }
    }
    cutCellPressure_.mergedPressureCells = mergedCells;
    const bool merging = mergedCells > 0;
    const auto pressureCell = [&]( int cell )
    {
        return merging ? pressureRoot[cell] : cell;
    };

    auto& dynamicResponseOwners = scratch.dynamicResponseOwners;
    dynamicResponseOwners.clear();
    for( std::size_t i = 0; i < obstacles.size(); ++i )
        if( obstacles[i].inverseMass > 0.0f || obstacles[i].inverseInertia > 0.0f )
            dynamicResponseOwners.push_back( static_cast<int>( i ) );

    int dynamicBoundaryFaces = 0;
    const auto addBoundary = [&]( int cell, int owner, vec2 force, vec2 point )
    {
        if( owner < 0 || cell < 0 ) return;
        const fluidShapeObstacle2& obstacle = obstacles[static_cast<std::size_t>( owner )];
        const bool dynamic = obstacle.inverseMass > 0.0f || obstacle.inverseInertia > 0.0f;
        // 이진 MAC은 종전과 같이 dynamic interface만 기록한다.
        // Cut-cell은 static/kinematic pressure traction도 feedback에 남긴다.
        if( !cutCell && !dynamic ) return;
        const double torque = Cross( point - obstacle.center, force );
        // Static/kinematic boundaries have zero inverse mass and inertia,
        // so they contribute nothing to B M_body^-1 B^T. Keep their
        // pressure traction in the separate feedback path, but do not
        // revisit them in every PCG matrix product.
        if( !dynamic ) return;
        faces.push_back( { cell, owner, force.x, force.y, torque } );
        ++dynamicBoundaryFaces;
        // Diagonal-only (Jacobi) preconditioner. Same-body cross terms are
        // ignored in the preconditioner, not in the Schur matrix product.
        diagonal[cell] += static_cast<double>( obstacle.inverseMass ) *
            ( static_cast<double>( force.x ) * force.x + static_cast<double>( force.y ) * force.y ) +
            static_cast<double>( obstacle.inverseInertia ) * torque * torque;
    };
    if( cutCell )
    {
        const auto referenceShapes = analyticDisjointCache_
            ? std::vector<analyticDisjointShape2>{} : disjointAnalyticCutCellEnabled_
                ? BuildDisjointAnalyticShapes( obstacles ) : std::vector<analyticDisjointShape2>{};
        const auto& disjointShapes = analyticDisjointCache_
            ? analyticDisjointCache_->shapes : referenceShapes;
        if( !disjointShapes.empty() )
        {
            const double h = cellSize_;
            const auto addClippedFace = [&]( bool vertical, int x, int y )
            {
                const int a = vertical ? cellIndex( x - 1, y ) : cellIndex( x, y - 1 );
                const int b = cellIndex( x, y );
                if( cellFluidFraction_[a] <= 0.0f && cellFluidFraction_[b] <= 0.0f ) return;
                const double normal = vertical ? origin_.x + x * h : origin_.y + y * h;
                const double low = vertical ? origin_.y + y * h : origin_.x + x * h;
                const auto addPortion = [&]( std::size_t owner, analyticCircleInterval clipped )
                {
                    const vec2 point = vertical
                        ? vec2{ static_cast<float>( normal ), static_cast<float>( clipped.midpoint ) }
                        : vec2{ static_cast<float>( clipped.midpoint ), static_cast<float>( normal ) };
                    const vec2 force = vertical
                        ? vec2{ static_cast<float>( clipped.length ), 0.0f }
                        : vec2{ 0.0f, static_cast<float>( clipped.length ) };
                    if( cellFluidFraction_[a] > 0.0f )
                        addBoundary( a, static_cast<int>( owner ), force, point );
                    if( cellFluidFraction_[b] > 0.0f )
                        addBoundary( b, static_cast<int>( owner ), -force, point );
                };
                if( analyticCutCellMovableSchurFacesEnabled_ && disjointShapes.size() > 2 )
                {
                    // For 3+ analytically disjoint shapes, the face contributions
                    // are independent. Static/kinematic bodies have zero Schur
                    // mass response, so their clipped intervals cannot enter B.
                    // Keep original owner order and the U/V face order.
                    for( int owner : dynamicResponseOwners )
                    {
                        const auto clipped = ClipDisjointShapeFace(
                            disjointShapes[owner], vertical, normal, low, low + h );
                        if( clipped.length > 0.0 ) addPortion( owner, clipped );
                    }
                }
                else
                    ForEachAnalyticFacePortion( disjointShapes, vertical, normal, low, low + h,
                        addPortion );
            };
            if( analyticCutCellIndexedBoundaryFacesEnabled_ )
            {
                // Same U-then-V row-major face order as the reference path,
                // without scanning fully open faces or changing Schur physics.
                for( int face : analyticUBlockedFaces_ )
                    addClippedFace( true, face % ( width_ + 1 ), face / ( width_ + 1 ) );
                for( int face : analyticVBlockedFaces_ )
                    addClippedFace( false, face % width_, face / width_ );
            }
            else
            {
                for( int y = 0; y < height_; ++y )
                    for( int x = 1; x < width_; ++x ) addClippedFace( true, x, y );
                for( int y = 1; y < height_; ++y )
                    for( int x = 0; x < width_; ++x ) addClippedFace( false, x, y );
            }
        }
        else
        {
        analyticFluidCircle circle{};
        analyticFluidPolygon polygon{};
        analyticFluidCapsule capsule{};
        analyticFluidRoundedPolygon rounded{};
        const bool circleGeometry = analyticCircleCutCellEnabled_ &&
            TryGetAnalyticCircle( obstacles, circle );
        const bool polygonGeometry = !circleGeometry && analyticPolygonCutCellEnabled_ &&
            TryGetAnalyticPolygon( obstacles, polygon );
        const bool capsuleGeometry = !circleGeometry && !polygonGeometry &&
            analyticCapsuleCutCellEnabled_ && TryGetAnalyticCapsule( obstacles, capsule );
        const bool roundedGeometry = !circleGeometry && !polygonGeometry && !capsuleGeometry &&
            analyticRoundedPolygonCutCellEnabled_ && TryGetAnalyticRoundedPolygon( obstacles, rounded );
        if( circleGeometry || polygonGeometry || capsuleGeometry || roundedGeometry )
        {
            const double h = cellSize_;
            for( int y = 0; y < height_; ++y )
                for( int x = 1; x < width_; ++x )
                {
                    const int left = cellIndex( x - 1, y );
                    const int right = cellIndex( x, y );
                    if( cellFluidFraction_[left] <= 0.0f && cellFluidFraction_[right] <= 0.0f ) continue;
                    const double normal = origin_.x + x * h;
                    const double low = origin_.y + y * h;
                    const auto clipped = circleGeometry
                        ? ClipCircleFace( circle, true, normal, low, low + h )
                        : polygonGeometry
                            ? ClipPolygonFace( polygon, true, normal, low, low + h )
                            : capsuleGeometry
                                ? ClipCapsuleFace( capsule, true, normal, low, low + h )
                                : ClipRoundedPolygonFace( rounded, true, normal, low, low + h );
                    if( clipped.length <= 0.0 ) continue;
                    const vec2 point{ static_cast<float>( normal ), static_cast<float>( clipped.midpoint ) };
                    const vec2 force{ static_cast<float>( clipped.length ), 0.0f };
                    if( cellFluidFraction_[left] > 0.0f ) addBoundary( left, 0, force, point );
                    if( cellFluidFraction_[right] > 0.0f ) addBoundary( right, 0, -force, point );
                }
            for( int y = 1; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int bottom = cellIndex( x, y - 1 );
                    const int top = cellIndex( x, y );
                    if( cellFluidFraction_[bottom] <= 0.0f && cellFluidFraction_[top] <= 0.0f ) continue;
                    const double normal = origin_.y + y * h;
                    const double low = origin_.x + x * h;
                    const auto clipped = circleGeometry
                        ? ClipCircleFace( circle, false, normal, low, low + h )
                        : polygonGeometry
                            ? ClipPolygonFace( polygon, false, normal, low, low + h )
                            : capsuleGeometry
                                ? ClipCapsuleFace( capsule, false, normal, low, low + h )
                                : ClipRoundedPolygonFace( rounded, false, normal, low, low + h );
                    if( clipped.length <= 0.0 ) continue;
                    const vec2 point{ static_cast<float>( clipped.midpoint ), static_cast<float>( normal ) };
                    const vec2 force{ 0.0f, static_cast<float>( clipped.length ) };
                    if( cellFluidFraction_[bottom] > 0.0f ) addBoundary( bottom, 0, force, point );
                    if( cellFluidFraction_[top] > 0.0f ) addBoundary( top, 0, -force, point );
                }
        }
        else
        {
        // 막힌 공유 sub-face를 힘/토크 적분점으로 사용한다. 점유율 및
        // 경계 속도와 같은 표본 수를 사용해 B와 B^T의 adjoint를 유지한다.
        const int sampleCount = cutCellSampleCount_;
        const float sampleScale = 1.0f / sampleCount;
        std::vector<double> bounds( obstacles.size(), -1.0 );
        for( std::size_t i = 0; i < obstacles.size(); ++i )
            if( IsValidFluidObstacle( obstacles[i] ) )
                bounds[i] = FluidObstacleBoundRadius( obstacles[i] );
        const auto firstOwner = [&]( vec2 point )
        {
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                if( bounds[i] < 0.0 ) continue;
                const vec2 center = obstacles[i].transform.position;
                if( std::abs( static_cast<double>( point.x ) - center.x ) > bounds[i] ||
                    std::abs( static_cast<double>( point.y ) - center.y ) > bounds[i] ) continue;
                if( ContainsFluidObstacle( obstacles[i], point ) ) return static_cast<int>( i );
            }
            return -1;
        };

        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                if( cellFluidFraction_[left] <= 0.0f && cellFluidFraction_[right] <= 0.0f ) continue;
                const vec2 force{ cellSize_ * sampleScale, 0.0f };
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) * sampleScale };
                    const int owner = firstOwner( point );
                    if( owner < 0 ) continue;
                    if( cellFluidFraction_[left] > 0.0f ) addBoundary( left, owner, force, point );
                    if( cellFluidFraction_[right] > 0.0f ) addBoundary( right, owner, -force, point );
                }
            }

        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                if( cellFluidFraction_[bottom] <= 0.0f && cellFluidFraction_[top] <= 0.0f ) continue;
                const vec2 force{ 0.0f, cellSize_ * sampleScale };
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ) + ( sample + 0.5f ) * sampleScale, static_cast<float>( y ) };
                    const int owner = firstOwner( point );
                    if( owner < 0 ) continue;
                    if( cellFluidFraction_[bottom] > 0.0f ) addBoundary( bottom, owner, force, point );
                    if( cellFluidFraction_[top] > 0.0f ) addBoundary( top, owner, -force, point );
                }
            }
        }
        }
    }
    else
    {
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                const int leftOwner = solidOwner_[left];
                const int rightOwner = solidOwner_[right];
                const vec2 point = origin_ + cellSize_ *
                    vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
                if( leftOwner < 0 && rightOwner >= 0 ) addBoundary( left, rightOwner, { cellSize_, 0.0f }, point );
                if( rightOwner < 0 && leftOwner >= 0 ) addBoundary( right, leftOwner, { -cellSize_, 0.0f }, point );
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                const int bottomOwner = solidOwner_[bottom];
                const int topOwner = solidOwner_[top];
                const vec2 point = origin_ + cellSize_ *
                    vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
                if( bottomOwner < 0 && topOwner >= 0 ) addBoundary( bottom, topOwner, { 0.0f, cellSize_ }, point );
                if( topOwner < 0 && bottomOwner >= 0 ) addBoundary( top, bottomOwner, { 0.0f, -cellSize_ }, point );
            }
    }
    if( dynamicBoundaryFaces == 0 ) return fail( fluidCoupledPressureStatus2::noBoundary );

    // Closed-domain Neumann pressure has an arbitrary gauge. Remove the global
    // incompatible source; disconnected cavities need a future component treatment.
    if( !cutCell )
    {
        const double mean = rhsSum / active;
        for( std::size_t i = 0; i < rhs.size(); ++i )
            if( solidOwner_[i] < 0 ) rhs[i] -= mean;
    }
    // Cut-cell의 region별 호환성 보정은 projectCutCell()에서 이미 완료했다.

    // Keep the original horizontal-then-vertical face order. The Laplacian
    // remains matrix-free; only its fixed nonzero edge weights are cached for
    // this solve. Dynamic body Schur response stays in the original loops.
    const bool cachedSchurEdges = cutCell && cutCellTransition_.usedDisjointAnalyticGeometry &&
        analyticCutCellCachedSchurEdgesEnabled_;
    auto& laplacianEdges = scratch.laplacianEdges;
    if( cachedSchurEdges )
    {
        laplacianEdges.clear();
        laplacianEdges.reserve( static_cast<std::size_t>( 2 * width_ * height_ ) );
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const double weight = cutCellUOpen( x, y );
                if( weight == 0.0 ) continue;
                laplacianEdges.push_back( { cellIndex( x - 1, y ), cellIndex( x, y ), weight } );
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const double weight = cutCellVOpen( x, y );
                if( weight == 0.0 ) continue;
                laplacianEdges.push_back( { cellIndex( x, y - 1 ), cellIndex( x, y ), weight } );
            }
        if( profile )
            timing.cachedLaplacianEdges = static_cast<int>( laplacianEdges.size() );
    }

    auto& response = scratch.response;
    response.resize( obstacles.size() );
    const auto multiply = [&]( const std::vector<double>& values, std::vector<double>& result )
    {
        std::fill( result.begin(), result.end(), 0.0 );
        // Only movable bodies have Schur faces. Static response slots are
        // never read, so clearing every obstacle on each PCG iteration wastes
        // work in scenes with many immovable walls.
        for( int owner : dynamicResponseOwners )
            response[owner] = {};
        if( cachedSchurEdges )
        {
            for( const coupledPressureEdge& edge : laplacianEdges )
            {
                const int first = pressureCell( edge.first );
                const int second = pressureCell( edge.second );
                const double d = edge.weight * ( values[first] - values[second] );
                result[first] += d;
                result[second] -= d;
            }
        }
        else
        {
            // Reference path retained for deterministic pressure/impulse
            // comparison. Iterate horizontal faces before vertical faces.
            for( int y = 0; y < height_; ++y )
                for( int x = 1; x < width_; ++x )
                {
                    const int left = cellIndex( x - 1, y ), right = cellIndex( x, y );
                    if( !cutCell && ( solidOwner_[left] >= 0 || solidOwner_[right] >= 0 ) ) continue;
                    const double weight = cutCell ? cutCellUOpen( x, y ) : 1.0;
                    const int a = pressureCell( left ), b = pressureCell( right );
                    const double d = weight * ( values[a] - values[b] );
                    result[a] += d;
                    result[b] -= d;
                }
            for( int y = 1; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int bottom = cellIndex( x, y - 1 ), top = cellIndex( x, y );
                    if( !cutCell && ( solidOwner_[bottom] >= 0 || solidOwner_[top] >= 0 ) ) continue;
                    const double weight = cutCell ? cutCellVOpen( x, y ) : 1.0;
                    const int a = pressureCell( bottom ), b = pressureCell( top );
                    const double d = weight * ( values[a] - values[b] );
                    result[a] += d;
                    result[b] -= d;
                }
        }
        for( const coupledPressureFace& face : faces )
        {
            coupledBodyResponse& body = response[face.owner];
            const double p = values[pressureCell( face.cell )];
            body.fx += p * face.fx;
            body.fy += p * face.fy;
            body.torque += p * face.torque;
        }
        for( const coupledPressureFace& face : faces )
        {
            const fluidShapeObstacle2& obstacle = obstacles[static_cast<std::size_t>( face.owner )];
            const coupledBodyResponse& body = response[face.owner];
            result[pressureCell( face.cell )] += static_cast<double>( obstacle.inverseMass ) *
                ( face.fx * body.fx + face.fy * body.fy ) +
                static_cast<double>( obstacle.inverseInertia ) * face.torque * body.torque;
        }
    };

    auto& x = scratch.x;
    auto& residual = scratch.residual;
    auto& z = scratch.z;
    auto& direction = scratch.direction;
    auto& product = scratch.product;
    x.assign( rhs.size(), 0.0 );
    residual = rhs;
    z.assign( rhs.size(), 0.0 );
    direction.assign( rhs.size(), 0.0 );
    product.assign( rhs.size(), 0.0 );

    // Opt-in warm start or diagnostic: apply the *new* Schur operator to
    // last step's float pressure. Never reuse the old RHS or preconditioner.
    // Reject stale guesses unless they halve the residual norm. Keep the
    // original ||b|| baseline for relative tolerance and true-residual tests.
    bool warmStartAccepted = false;
    double coldRhsSquared = 0.0;
    if( cutCell && ( coupledPressureWarmStartEnabled_ ||
        ( profile && coupledPressureWarmStartProbeEnabled_ ) ) )
    {
        bool validGuess = true;
        for( std::size_t i = 0; i < rhs.size(); ++i )
        {
            if( cellFluidFraction_[i] <= 0.0f ) continue;
            if( !std::isfinite( pressure_[i] ) )
            {
                validGuess = false;
                break;
            }
            if( merging && pressureRoot[i] != static_cast<int>( i ) ) continue;
            x[i] = static_cast<double>( pressure_[i] );
        }
        if( validGuess )
        {
            multiply( x, product );
            double guessSquared = 0.0;
            for( std::size_t i = 0; i < rhs.size(); ++i )
            {
                if( cellFluidFraction_[i] <= 0.0f ) continue;
                const double error = rhs[i] - product[i];
                coldRhsSquared += rhs[i] * rhs[i];
                guessSquared += error * error;
            }
            if( std::isfinite( coldRhsSquared ) && std::isfinite( guessSquared ) )
            {
                if( profile && coupledPressureWarmStartProbeEnabled_ )
                {
                    timing.warmStartRhsSquared += coldRhsSquared;
                    timing.warmStartGuessSquared += guessSquared;
                    ++timing.warmStartProbes;
                }
                // squared residual < 0.25 * ||b||² => at least 50% reduction.
                warmStartAccepted = coupledPressureWarmStartEnabled_ &&
                    coldRhsSquared > 0.0 && guessSquared < 0.25 * coldRhsSquared;
                if( warmStartAccepted )
                    for( std::size_t i = 0; i < rhs.size(); ++i )
                        if( cellFluidFraction_[i] > 0.0f )
                            residual[i] = rhs[i] - product[i];
            }
        }
        if( profile && coupledPressureWarmStartEnabled_ )
        {
            ++timing.warmStartAttempts;
            if( warmStartAccepted ) ++timing.warmStartAccepted;
        }
        if( !warmStartAccepted ) std::fill( x.begin(), x.end(), 0.0 );
        std::fill( product.begin(), product.end(), 0.0 );
    }

    // Zero-fill incomplete LDL^T on the 5-point cut-cell Laplacian plus the
    // existing diagonal body Schur approximation. The full A = L + B M^-1 B^T
    // stays unchanged in multiply(), so this cannot hide the true residual.
    // The small positive shift protects isolated Neumann pockets and tiny pivots.
    const bool useIncompleteCholesky = cutCell && cutCellTransition_.usedDisjointAnalyticGeometry &&
        analyticCutCellIncompleteCholeskyEnabled_;
    pressureMomentum_.usedIncompleteCholesky = useIncompleteCholesky;
    const bool cachedIc0 = useIncompleteCholesky && analyticCutCellCachedIc0Enabled_;
    const bool activeSweep = cachedIc0 && analyticCutCellActiveIc0SweepEnabled_;
    const bool splitIc0 = activeSweep && analyticCutCellSplitIc0CoefficientsEnabled_;
    auto& incompletePivot = scratch.incompletePivot;
    auto& incompleteForward = scratch.incompleteForward;
    auto& incompleteCached = scratch.incompleteCached;
    auto& splitForward = scratch.incompleteSplitForward;
    auto& splitBackward = scratch.incompleteSplitBackward;
    auto& incompleteActiveCells = scratch.incompleteActiveCells;
    nextPhase( phase::factorization );
    if( useIncompleteCholesky )
    {
        incompletePivot.assign( rhs.size(), 0.0 );
        incompleteForward.assign( rhs.size(), 0.0 );
        if( cachedIc0 && !splitIc0 ) incompleteCached.assign( rhs.size(), {} );
        if( splitIc0 )
        {
            splitForward.assign( rhs.size(), {} );
            splitBackward.assign( rhs.size(), {} );
        }
        if( activeSweep ) incompleteActiveCells.clear();
        for( int y = 0; y < height_; ++y )
            for( int xCell = 0; xCell < width_; ++xCell )
            {
                const int cell = cellIndex( xCell, y );
                if( cellFluidFraction_[cell] <= 0.0f ) continue;
                if( activeSweep ) incompleteActiveCells.push_back( cell );
                const double regularization = std::max( 1.0e-5, diagonal[cell] * 1.0e-3 );
                double pivot = diagonal[cell] + regularization;
                if( xCell > 0 )
                {
                    const double west = cutCellUOpen( xCell, y );
                    if( west > 0.0 )
                    {
                        pivot -= west * west / incompletePivot[cell - 1];
                        if( cachedIc0 )
                        {
                            if( splitIc0 )
                            {
                                splitForward[cell].westOverPivot = west / incompletePivot[cell - 1];
                                splitBackward[cell - 1].east = west;
                            }
                            else
                            {
                                auto& coefficients = incompleteCached[cell];
                                coefficients.west = west;
                                coefficients.westOverPivot = west / incompletePivot[cell - 1];
                                if( activeSweep ) incompleteCached[cell - 1].east = west;
                            }
                        }
                    }
                }
                if( y > 0 )
                {
                    const double south = cutCellVOpen( xCell, y );
                    if( south > 0.0 )
                    {
                        pivot -= south * south / incompletePivot[cell - width_];
                        if( cachedIc0 )
                        {
                            if( splitIc0 )
                            {
                                splitForward[cell].southOverPivot = south / incompletePivot[cell - width_];
                                splitBackward[cell - width_].north = south;
                            }
                            else
                            {
                                auto& coefficients = incompleteCached[cell];
                                coefficients.south = south;
                                coefficients.southOverPivot = south / incompletePivot[cell - width_];
                                if( activeSweep ) incompleteCached[cell - width_].north = south;
                            }
                        }
                    }
                }
                incompletePivot[cell] = std::max( regularization, pivot );
                if( cachedIc0 )
                {
                    if( splitIc0 ) splitBackward[cell].inversePivot = 1.0 / incompletePivot[cell];
                    else incompleteCached[cell].inversePivot = 1.0 / incompletePivot[cell];
                }
            }
    }
    nextPhase( phase::pcg );
    const auto applyPreconditioner = [&]( const std::vector<double>& source )
    {
        if( !useIncompleteCholesky )
        {
            for( std::size_t i = 0; i < source.size(); ++i )
                z[i] = source[i] / std::max( 1.0e-8, diagonal[i] );
            return;
        }
        if( splitIc0 )
        {
            // The forward pass only touches two cached multipliers. The
            // reverse pass uses east/north face weights and the inverse pivot.
            // The active-cell traversal and arithmetic order are identical
            // to the existing AoS IC(0) path.
            const auto forwardBegin = profile ? clock::now() : clock::time_point{};
            for( int cell : incompleteActiveCells )
            {
                const auto& coefficients = splitForward[cell];
                double value = source[cell];
                if( coefficients.westOverPivot != 0.0 )
                    value += coefficients.westOverPivot * incompleteForward[cell - 1];
                if( coefficients.southOverPivot != 0.0 )
                    value += coefficients.southOverPivot * incompleteForward[cell - width_];
                incompleteForward[cell] = value;
            }
            const auto backwardBegin = profile ? clock::now() : clock::time_point{};
            if( profile )
                timing.ic0Forward += std::chrono::duration<double, std::milli>(
                    backwardBegin - forwardBegin ).count();
            for( auto it = incompleteActiveCells.rbegin(); it != incompleteActiveCells.rend(); ++it )
            {
                const int cell = *it;
                const auto& coefficients = splitBackward[cell];
                double value = incompleteForward[cell];
                if( coefficients.east != 0.0 )
                    value += coefficients.east * z[cell + 1];
                if( coefficients.north != 0.0 )
                    value += coefficients.north * z[cell + width_];
                z[cell] = value * coefficients.inversePivot;
            }
            if( profile )
                timing.ic0Backward += std::chrono::duration<double, std::milli>(
                    clock::now() - backwardBegin ).count();
            return;
        }
        if( activeSweep )
        {
            // Each active cell is visited once in row-major order. The face
            // coefficients were stored when the pivot was assembled, including
            // east/north weights of the next neighbor. No per-iteration fluid
            // occupancy test, x/y decoding, or cut-cell lookup is necessary.
            for( int cell : incompleteActiveCells )
            {
                const auto& coefficients = incompleteCached[cell];
                double value = source[cell];
                if( coefficients.west != 0.0 )
                    value += coefficients.westOverPivot * incompleteForward[cell - 1];
                if( coefficients.south != 0.0 )
                    value += coefficients.southOverPivot * incompleteForward[cell - width_];
                incompleteForward[cell] = value;
            }
            for( auto it = incompleteActiveCells.rbegin(); it != incompleteActiveCells.rend(); ++it )
            {
                const int cell = *it;
                const auto& coefficients = incompleteCached[cell];
                double value = incompleteForward[cell];
                if( coefficients.east != 0.0 )
                    value += coefficients.east * z[cell + 1];
                if( coefficients.north != 0.0 )
                    value += coefficients.north * z[cell + width_];
                z[cell] = value * coefficients.inversePivot;
            }
            return;
        }
        if( cachedIc0 )
        {
            // Precomputed face coefficients and reciprocal pivots are valid for
            // this solve only. No cut-cell lookup or division in either sweep.
            for( int y = 0; y < height_; ++y )
                for( int xCell = 0; xCell < width_; ++xCell )
                {
                    const int cell = cellIndex( xCell, y );
                    if( cellFluidFraction_[cell] <= 0.0f ) continue;
                    const auto& coefficients = incompleteCached[cell];
                    double value = source[cell];
                    if( xCell > 0 )
                        value += coefficients.westOverPivot * incompleteForward[cell - 1];
                    if( y > 0 )
                        value += coefficients.southOverPivot * incompleteForward[cell - width_];
                    incompleteForward[cell] = value;
                }
            for( int y = height_ - 1; y >= 0; --y )
                for( int xCell = width_ - 1; xCell >= 0; --xCell )
                {
                    const int cell = cellIndex( xCell, y );
                    if( cellFluidFraction_[cell] <= 0.0f ) continue;
                    double value = incompleteForward[cell];
                    if( xCell + 1 < width_ )
                        value += incompleteCached[cell + 1].west * z[cell + 1];
                    if( y + 1 < height_ )
                        value += incompleteCached[cell + width_].south * z[cell + width_];
                    z[cell] = value * incompleteCached[cell].inversePivot;
                }
            return;
        }
        // Reference path: fetch face apertures and divide by pivots on every
        // application. Keep it for deterministic differential verification.
        // Forward solve L*y=r (unit lower triangular, west/south neighbors).
        for( int y = 0; y < height_; ++y )
            for( int xCell = 0; xCell < width_; ++xCell )
            {
                const int cell = cellIndex( xCell, y );
                if( cellFluidFraction_[cell] <= 0.0f ) continue;
                double value = source[cell];
                if( xCell > 0 )
                {
                    const double west = cutCellUOpen( xCell, y );
                    if( west > 0.0 )
                        value += west * incompleteForward[cell - 1] / incompletePivot[cell - 1];
                }
                if( y > 0 )
                {
                    const double south = cutCellVOpen( xCell, y );
                    if( south > 0.0 )
                        value += south * incompleteForward[cell - width_] / incompletePivot[cell - width_];
                }
                incompleteForward[cell] = value;
            }
        // Backward solve D*L^T*z=y (east/north neighbors).
        for( int y = height_ - 1; y >= 0; --y )
            for( int xCell = width_ - 1; xCell >= 0; --xCell )
            {
                const int cell = cellIndex( xCell, y );
                if( cellFluidFraction_[cell] <= 0.0f ) continue;
                double value = incompleteForward[cell];
                if( xCell + 1 < width_ )
                    value += cutCellUOpen( xCell + 1, y ) * z[cell + 1];
                if( y + 1 < height_ )
                    value += cutCellVOpen( xCell, y + 1 ) * z[cell + width_];
                z[cell] = value / incompletePivot[cell];
            }
    };

    // P^T M_full^-1 P is symmetric positive definite for the pooled solve.
    // Keep the original IC(0) on the unmodified five-point grid; lift the
    // reduced residual to every member, then restrict the preconditioned
    // vector by group sum. No IC(0) on a non-Cartesian merged graph required.
    const auto timedPreconditioner = [&]( const std::vector<double>& source )
    {
        const auto begin = profile ? clock::now() : clock::time_point{};
        if( merging )
        {
            auto& lifted = scratch.smallCellLiftedResidual;
            lifted.resize( source.size() );
            for( std::size_t i = 0; i < source.size(); ++i )
                lifted[i] = cellFluidFraction_[i] > 0.0f
                    ? source[pressureRoot[i]] : 0.0;
            applyPreconditioner( lifted );
            auto& reduced = scratch.smallCellReducedPreconditioner;
            reduced.assign( source.size(), 0.0 );
            for( std::size_t i = 0; i < source.size(); ++i )
                if( cellFluidFraction_[i] > 0.0f )
                    reduced[pressureRoot[i]] += z[i];
            z.swap( reduced );
        }
        else applyPreconditioner( source );
        if( profile )
        {
            timing.preconditioner += std::chrono::duration<double, std::milli>(
                clock::now() - begin ).count();
            ++timing.preconditionerApplications;
        }
    };
    const auto timedMultiply = [&]( const std::vector<double>& source, std::vector<double>& target )
    {
        if( !profile ) { multiply( source, target ); return; }
        const auto begin = clock::now();
        multiply( source, target );
        timing.multiply += std::chrono::duration<double, std::milli>( clock::now() - begin ).count();
        ++timing.matrixProducts;
    };

    // IC(0) constructed this sorted active-cell list during factorization.
    // Only the PCG vector operations change: matrix products and true-residual
    // validation keep their full-grid reference paths. When inactive cells are
    // skipped, their x/r/z/direction values remain zero as initialized above.
    const bool activePcgVectors = activeSweep && analyticCutCellActivePcgVectorsEnabled_;
    const auto forEachPcgCell = [&]( const auto& visit )
    {
        if( activePcgVectors )
        {
            for( int cell : incompleteActiveCells )
                if( !merging || pressureRoot[cell] == cell )
                    visit( static_cast<std::size_t>( cell ) );
        }
        else if( merging )
        {
            for( std::size_t cell = 0; cell < rhs.size(); ++cell )
                if( cellFluidFraction_[cell] > 0.0f &&
                    pressureRoot[cell] == static_cast<int>( cell ) ) visit( cell );
        }
        else
            for( std::size_t cell = 0; cell < rhs.size(); ++cell ) visit( cell );
    };

    timedPreconditioner( residual );
    double rz = 0.0, squared = 0.0;
    forEachPcgCell( [&]( std::size_t i )
    {
        if( !activePcgVectors &&
            ( cutCell ? cellFluidFraction_[i] <= 0.0f : solidOwner_[i] >= 0 ) ) return;
        direction[i] = z[i];
        rz += residual[i] * z[i];
        squared += residual[i] * residual[i];
    } );
    if( !std::isfinite( rz ) || !std::isfinite( squared ) )
        return fail( fluidCoupledPressureStatus2::numericalBreakdown );
    // Target the same residual relative to the *cold* RHS, even when
    // the recursive PCG residual started from the accepted previous pressure.
    const double initialSquared = warmStartAccepted ? coldRhsSquared : squared;
    const double tolerance = std::max( 1.0e-18, initialSquared * 1.0e-10 );
    pressureMomentum_.coupledInitialResidualRms = static_cast<float>( std::sqrt( initialSquared / active ) );
    for( int iteration = 0; iteration < iterations && squared > tolerance; ++iteration )
    {
        timedMultiply( direction, product );
        double denom = 0.0;
        forEachPcgCell( [&]( std::size_t i )
        {
            denom += direction[i] * product[i];
        } );
        if( !std::isfinite( denom ) || denom <= 0.0 )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        const double alpha = rz / denom;
        if( !std::isfinite( alpha ) )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        squared = 0.0;
        forEachPcgCell( [&]( std::size_t i )
        {
            x[i] += alpha * direction[i];
            residual[i] -= alpha * product[i];
            squared += residual[i] * residual[i];
        } );
        pressureMomentum_.coupledIterations = iteration + 1;
        if( !std::isfinite( squared ) )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        if( squared <= tolerance ) break;
        timedPreconditioner( residual );
        double nextRz = 0.0;
        forEachPcgCell( [&]( std::size_t i )
        {
            nextRz += residual[i] * z[i];
        } );
        if( !std::isfinite( nextRz ) || rz <= 0.0 )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        const double beta = nextRz / rz;
        if( !std::isfinite( beta ) )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        forEachPcgCell( [&]( std::size_t i )
        {
            direction[i] = z[i] + beta * direction[i];
        } );
        rz = nextRz;
    }

    nextPhase( phase::validation );
    // Recursive CG 잔차는 반올림으로 실제 b-A*x와 달라질 수 있으므로
    // 종료 시 항상 원래 연산자로 직접 재계산한다. 반복 한도는 성공 조건이 아님.
    multiply( x, product );
    double trueSquared = 0.0;
    for( std::size_t i = 0; i < rhs.size(); ++i )
    {
        if( cutCell ? cellFluidFraction_[i] <= 0.0f : solidOwner_[i] >= 0 ) continue;
        const double error = rhs[i] - product[i];
        trueSquared += error * error;
    }
    if( !std::isfinite( trueSquared ) )
        return fail( fluidCoupledPressureStatus2::numericalBreakdown );
    pressureMomentum_.coupledFinalResidualRms = static_cast<float>( std::sqrt( trueSquared / active ) );
    pressureMomentum_.coupledRelativeResidual = initialSquared > 0.0
        ? static_cast<float>( std::sqrt( trueSquared / initialSquared ) ) : 0.0f;
    if( trueSquared > tolerance )
        return fail( fluidCoupledPressureStatus2::iterationLimit );
    nextPhase( phase::output );

    double pressureSum = 0.0;
    for( std::size_t i = 0; i < rhs.size(); ++i )
        if( cutCell ? cellFluidFraction_[i] > 0.0f : solidOwner_[i] < 0 ) pressureSum += x[i];
    // B*invM*B^T가 있는 Cut-cell에서는 불완전 점유/외벽 근사로
    // 상수 압력 방향이 정확한 nullspace가 아닐 수 있다. 수렴한 x에서
    // 임의의 gauge를 빼면 A*x=b를 다시 위반하므로 그대로 사용한다.
    const double gauge = cutCell ? 0.0 : pressureSum / active;
    for( std::size_t i = 0; i < rhs.size(); ++i )
    {
        if( cutCell ? cellFluidFraction_[i] <= 0.0f : solidOwner_[i] >= 0 ) { pressure_[i] = 0.0f; continue; }
        const double value = x[pressureCell( static_cast<int>( i ) )] - gauge;
        if( !std::isfinite( value ) || std::abs( value ) > std::numeric_limits<float>::max() )
            return fail( fluidCoupledPressureStatus2::invalidPressure );
        pressure_[i] = static_cast<float>( value );
    }
    pressureMomentum_.coupledStatus = fluidCoupledPressureStatus2::converged;
    nextPhase( phase::done );
    return true;
}

void fluidField2::project( float timeStep, int pressureIterations,
    std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback )
{
    // div(v) / dt = Laplace(p). 벽/solid no-through, Neumann 압력 경계.
    // 압력 경계가 fluid-fluid face 운동량을 얼마나 변경하는지 독립 측정한다.
    pressureMomentum_ = {};
    pressureMomentum_.energy.fluidBefore = getFluidKineticEnergy();
    const auto rigidKineticEnergy = []( std::span<const fluidShapeObstacle2> shapes,
        std::span<const fluidObstacleForce2> impulses, float dt )
    {
        double energy = 0.0;
        for( std::size_t i = 0; i < shapes.size(); ++i )
        {
            const fluidShapeObstacle2& shape = shapes[i];
            if( shape.inverseMass > 0.0f )
            {
                const vec2 velocity = shape.linearVelocity + ( impulses.empty() ? vec2{} :
                    ( dt * shape.inverseMass ) * impulses[i].force );
                energy += 0.5 * static_cast<double>( Dot( velocity, velocity ) ) / shape.inverseMass;
            }
            if( shape.inverseInertia > 0.0f )
            {
                const double omega = static_cast<double>( shape.angularVelocity ) + ( impulses.empty() ? 0.0 :
                    static_cast<double>( dt ) * shape.inverseInertia * impulses[i].torque );
                energy += 0.5 * omega * omega / shape.inverseInertia;
            }
        }
        return energy;
    };
    pressureMomentum_.energy.rigidBefore = rigidKineticEnergy( obstacles, {}, 0.0f );
    const vec2 beforePressure = getFluidMomentum();
    const float inverseCell = 1.0f / cellSize_;
    const float scale = cellSize_ * cellSize_ / timeStep;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            divergence_[cell] = solidOwner_[cell] < 0
                ? ( u_[uIndex( x + 1, y )] - u_[uIndex( x, y )] +
                    v_[vIndex( x, y + 1 )] - v_[vIndex( x, y )] ) * inverseCell
                : 0.0f;
        }

    const bool coupled = std::any_of( obstacles.begin(), obstacles.end(),
        []( const fluidShapeObstacle2& obstacle )
        {
            return obstacle.inverseMass > 0.0f || obstacle.inverseInertia > 0.0f;
        } );
    const bool solvedCoupled = coupled && solveCoupledPressure( timeStep, pressureIterations, obstacles );
    pressureMomentum_.usedCoupledProjection = solvedCoupled;
    pressureMomentum_.coupledSolveFailed = coupled && !solvedCoupled &&
        pressureMomentum_.coupledStatus != fluidCoupledPressureStatus2::noBoundary;
    // 연립 압력 계산에 실패했을 때 질량을 무시하는 Jacobi 반력을 강체에
    // 주입하면 added-mass 폭주가 재발할 수 있다. 실패 Step은 압력 반력을
    // 건너뛰고 진단을 남긴다. 이 경우 비압축성 제약은 만족하지 않을 수 있다.
    if( pressureMomentum_.coupledSolveFailed )
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    else if( !solvedCoupled )
    {
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
        for( int iteration = 0; iteration < pressureIterations; ++iteration )
        {
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int cell = cellIndex( x, y );
                    if( solidOwner_[cell] >= 0 ) { nextPressure_[cell] = 0.0f; continue; }
                float sum = 0.0f;
                int neighbors = 0;
                if( x > 0 && !isSolidCell( x - 1, y ) ) { sum += pressure_[cellIndex( x - 1, y )]; ++neighbors; }
                if( x + 1 < width_ && !isSolidCell( x + 1, y ) ) { sum += pressure_[cellIndex( x + 1, y )]; ++neighbors; }
                if( y > 0 && !isSolidCell( x, y - 1 ) ) { sum += pressure_[cellIndex( x, y - 1 )]; ++neighbors; }
                if( y + 1 < height_ && !isSolidCell( x, y + 1 ) ) { sum += pressure_[cellIndex( x, y + 1 )]; ++neighbors; }
                nextPressure_[cell] = neighbors > 0 ? ( sum - divergence_[cell] * scale ) / neighbors : 0.0f;
            }
            pressure_.swap( nextPressure_ );
        }
    }

    const float gradientScale = timeStep * inverseCell;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( !isSolidCell( x - 1, y ) && !isSolidCell( x, y ) )
                u_[uIndex( x, y )] -= gradientScale * ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x - 1, y )] );

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( !isSolidCell( x, y - 1 ) && !isSolidCell( x, y ) )
                v_[vIndex( x, y )] -= gradientScale * ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x, y - 1 )] );

    // Pressure traction: solid-fluid 경계의 fluid cell 압력을 solid 외향 법선에 적분.
    // 2D unit-depth, rho=1 근사이므로 반력은 정밀한 cut-cell 표면 적분이 아님.
    const auto accumulate = [&]( int owner, vec2 normal, vec2 point, float pressure )
    {
        fluidObstacleForce2& result = feedback[static_cast<std::size_t>( owner )];
        const vec2 force = pressure * cellSize_ * normal;
        result.force += force;
        result.torque += Cross( point - obstacles[static_cast<std::size_t>( owner )].center, force );
    };
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const int left = solidOwner_[cellIndex( x - 1, y )];
            const int right = solidOwner_[cellIndex( x, y )];
            if( left == right ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            if( left < 0 && right >= 0 ) accumulate( right, { 1.0f, 0.0f }, point, pressure_[cellIndex( x - 1, y )] );
            else if( right < 0 && left >= 0 ) accumulate( left, { -1.0f, 0.0f }, point, pressure_[cellIndex( x, y )] );
        }
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int bottom = solidOwner_[cellIndex( x, y - 1 )];
            const int top = solidOwner_[cellIndex( x, y )];
            if( bottom == top ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            if( bottom < 0 && top >= 0 ) accumulate( top, { 0.0f, 1.0f }, point, pressure_[cellIndex( x, y - 1 )] );
            else if( top < 0 && bottom >= 0 ) accumulate( bottom, { 0.0f, -1.0f }, point, pressure_[cellIndex( x, y )] );
        }

    // 내부 face와 강체 사이의 같은 압력해를 적용한다. pressure traction으로
    // 예측되는 Body 속도 증분을 solid-normal MAC face에도 갱신한다.
    // World의 Drag/Contact/Joint로 뒤이어 변하는 속도까지 미리 반영하지는 않는다.
    if( solvedCoupled ) applySolidBoundaries( obstacles, feedback, timeStep );

    // 닫힌 격자의 외벽도 유체 압력으로 반력을 받는다.
    // Solid가 외곽에 걸쳐 있는 셀은 외벽에 유체 압력이 없으므로 제외한다.
    // fluid+solid+wall = 0: 동일 pressure 미지수의 내부 face gradient가
    // 전부 telescope된 결과 (단, float 반올림에 따른 잔차는 남음).
    double wallX = 0.0;
    double wallY = 0.0;
    const double faceArea = static_cast<double>( cellSize_ ); // unit depth
    for( int y = 0; y < height_; ++y )
    {
        if( !isSolidCell( 0, y ) ) wallX -= pressure_[cellIndex( 0, y )] * faceArea;
        if( !isSolidCell( width_ - 1, y ) ) wallX += pressure_[cellIndex( width_ - 1, y )] * faceArea;
    }
    for( int x = 0; x < width_; ++x )
    {
        if( !isSolidCell( x, 0 ) ) wallY -= pressure_[cellIndex( x, 0 )] * faceArea;
        if( !isSolidCell( x, height_ - 1 ) ) wallY += pressure_[cellIndex( x, height_ - 1 )] * faceArea;
    }

    pressureMomentum_.energy.fluidAfter = getFluidKineticEnergy();
    pressureMomentum_.energy.rigidAfter = rigidKineticEnergy( obstacles, feedback, timeStep );

    vec2 solidForce{};
    for( const fluidObstacleForce2& force : feedback ) solidForce += force.force;
    pressureMomentum_.fluid = getFluidMomentum() - beforePressure;
    pressureMomentum_.solid = timeStep * solidForce;
    pressureMomentum_.wall = timeStep * vec2{ static_cast<float>( wallX ), static_cast<float>( wallY ) };
}

fluidFlowAudit2 fluidField2::getFlowAudit( float timeStep ) const noexcept
{
    fluidFlowAudit2 result{};
    if( !std::isfinite( timeStep ) || timeStep <= 0.0f )
    {
        result.continuityRms = std::numeric_limits<float>::quiet_NaN();
        return result;
    }

    result.continuityRms = cutCellProjectionEnabled_
        ? cutCellFluxDivergenceRms( 1.0f / timeStep ) : getDivergenceRms();
    // A pressure solve can converge while a nearly closed face carries an
    // unexpectedly large velocity. Preserve the exact face and open fraction.
    const auto observe = [&]( float speed, float open, int x, int y, bool isU )
    {
        if( open <= 0.0f ) return;
        if( !std::isfinite( speed ) )
        {
            ++result.nonFiniteFluidFaces;
            return;
        }
        const float magnitude = std::abs( speed );
        result.maxOpenWeightedFaceSpeed = std::max( result.maxOpenWeightedFaceSpeed, magnitude * open );
        if( magnitude > result.maxFluidFaceSpeed )
        {
            result.maxFluidFaceSpeed = magnitude;
            result.maxFaceVelocity = speed;
            result.maxFaceOpenFraction = open;
            result.maxFaceX = x;
            result.maxFaceY = y;
            result.maxFaceIsU = isU;
        }
    };
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            observe( u_[uIndex( x, y )], advectionUOpen( x, y ), x, y, true );
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            observe( v_[vIndex( x, y )], advectionVOpen( x, y ), x, y, false );
    return result;
}

float fluidField2::getDivergenceRms() const noexcept
{
    double sum = 0.0;
    int fluidCells = 0;
    const float inverseCell = 1.0f / cellSize_;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            if( isSolidCell( x, y ) ) continue;
            const float divergence =
                ( u_[uIndex( x + 1, y )] - u_[uIndex( x, y )] +
                  v_[vIndex( x, y + 1 )] - v_[vIndex( x, y )] ) * inverseCell;
            sum += static_cast<double>( divergence ) * divergence;
            ++fluidCells;
        }
    return fluidCells > 0 ? static_cast<float>( std::sqrt( sum / fluidCells ) ) : 0.0f;
}

} // namespace zonai
