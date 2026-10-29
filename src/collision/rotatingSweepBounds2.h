#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "collision/aabb2.h"
#include "collision/shapeProxy2.h"
#include "math/transform2.h"

namespace zonai
{

// 회전하는 각 support point의 짧은 원호를 축별로 감싸는 보수적 AABB.
// 병진 위치는 시작/끝의 구간으로 더하고, 도형 radius와 float 오차 여유를 확장함.
// 시작/종료 회전 사이가 90도 이하이며 GetSweepTransform의 nlerp 경로라는 전제임.
// 비유한 수치에서는 false를 반환하므로 호출자는 기존 외접 원 AABB를 사용함.
inline bool TryComputeRotatingSweepBounds( const shapeProxy2& proxy, const transform2& start,
    vec2 endPosition, rot2 endRotation, float rotationRadians, aabb2& result )
{
    if( proxy.count < 1 || proxy.count > static_cast<int>( proxy.points.size() ) ||
        !IsValidAABB( { start.position, start.position } ) || !IsFinite( endPosition ) ||
        !std::isfinite( proxy.radius ) || proxy.radius < 0.0f ||
        !std::isfinite( rotationRadians ) || std::fabs( rotationRadians ) > 1.5707963267948966f ) return false;

    const double startLength = std::hypot( static_cast<double>( start.rotation.c ),
        static_cast<double>( start.rotation.s ) );
    const double endLength = std::hypot( static_cast<double>( endRotation.c ),
        static_cast<double>( endRotation.s ) );
    if( !std::isfinite( startLength ) || !std::isfinite( endLength ) ||
        startLength == 0.0 || endLength == 0.0 ) return false;

    // 실제 sweep은 회전 성분을 보간한 뒤 Normalize하므로 단위 회전으로 원호를 계산함.
    const double startC = static_cast<double>( start.rotation.c ) / startLength;
    const double startS = static_cast<double>( start.rotation.s ) / startLength;
    const double endC = static_cast<double>( endRotation.c ) / endLength;
    const double endS = static_cast<double>( endRotation.s ) / endLength;

    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double maxX = -minX;
    double maxY = -minX;
    double maxLocalRadius = 0.0;

    for( int i = 0; i < proxy.count; ++i )
    {
        const vec2 point = proxy.points[i];
        if( !IsFinite( point ) ) return false;

        const double x = point.x;
        const double y = point.y;
        const double radius = std::hypot( x, y );
        if( !std::isfinite( radius ) ) return false;
        maxLocalRadius = std::max( maxLocalRadius, radius );

        const double x0 = startC * x - startS * y;
        const double y0 = startS * x + startC * y;
        const double x1 = endC * x - endS * y;
        const double y1 = endS * x + endC * y;
        minX = std::min( { minX, x0, x1 } );
        minY = std::min( { minY, y0, y1 } );
        maxX = std::max( { maxX, x0, x1 } );
        maxY = std::max( { maxY, y0, y1 } );

        if( radius == 0.0 ) continue;

        // 짧은 원호가 ±X/±Y 축을 지나면 해당 축의 extrema는 ±radius.
        // 각도 변환(atan2) 없이 cross 부호로 두 ray 사이에 있는지 판정함.
        const double ax = rotationRadians >= 0.0f ? x0 : x1;
        const double ay = rotationRadians >= 0.0f ? y0 : y1;
        const double bx = rotationRadians >= 0.0f ? x1 : x0;
        const double by = rotationRadians >= 0.0f ? y1 : y0;
        const double tolerance = 32.0 * std::numeric_limits<float>::epsilon() * radius;

        if( -ay >= -tolerance && by >= -tolerance ) maxX = std::max( maxX, radius );
        if( ay >= -tolerance && -by >= -tolerance ) minX = std::min( minX, -radius );
        if( ax >= -tolerance && -bx >= -tolerance ) maxY = std::max( maxY, radius );
        if( -ax >= -tolerance && bx >= -tolerance ) minY = std::min( minY, -radius );
    }

    // GetSweepTransform의 float normalize/위치 보간 오차와 radius 반올림을 포함함.
    constexpr double EPSILON = std::numeric_limits<float>::epsilon();
    const double padX = 64.0 * EPSILON * ( 1.0 + std::fabs( start.position.x ) +
        std::fabs( endPosition.x ) + maxLocalRadius + proxy.radius );
    const double padY = 64.0 * EPSILON * ( 1.0 + std::fabs( start.position.y ) +
        std::fabs( endPosition.y ) + maxLocalRadius + proxy.radius );

    const double lowX = std::min( static_cast<double>( start.position.x ),
        static_cast<double>( endPosition.x ) ) + minX - proxy.radius - padX;
    const double lowY = std::min( static_cast<double>( start.position.y ),
        static_cast<double>( endPosition.y ) ) + minY - proxy.radius - padY;
    const double highX = std::max( static_cast<double>( start.position.x ),
        static_cast<double>( endPosition.x ) ) + maxX + proxy.radius + padX;
    const double highY = std::max( static_cast<double>( start.position.y ),
        static_cast<double>( endPosition.y ) ) + maxY + proxy.radius + padY;

    const float infinity = std::numeric_limits<float>::infinity();
    const aabb2 bounds{
        { std::nextafter( static_cast<float>( lowX ), -infinity ),
          std::nextafter( static_cast<float>( lowY ), -infinity ) },
        { std::nextafter( static_cast<float>( highX ), infinity ),
          std::nextafter( static_cast<float>( highY ), infinity ) }
    };

    if( !IsValidAABB( bounds ) ) return false;
    result = bounds;
    return true;
}

} // namespace zonai
