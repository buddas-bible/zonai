#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "math/transform2.h"
#include "math/vec2.h"

namespace zonai
{

constexpr std::size_t MAX_MANIFOLD_POINTS = 2;

// A/B feature index를 각각 8bit로 묶어 contact point id로 사용함.
// 같은 feature pair가 유지되면 이전 solver impulse를 warm start에 재사용할 수 있음.
constexpr std::uint16_t MakeContactPointId( std::size_t indexA, std::size_t indexB ) noexcept
{
    const std::uint16_t featureA = static_cast<std::uint8_t>( indexA );

    const std::uint16_t featureB = static_cast<std::uint8_t>( indexB );

    return static_cast<std::uint16_t>( ( featureA << 8u ) | featureB );
}

struct localManifoldPoint2
{
    vec2 point{};
    float separation{};
    std::uint16_t id{};
};

struct localManifold2
{
    vec2 normal{};
    std::array<localManifoldPoint2, MAX_MANIFOLD_POINTS> points{};
    int pointCount{};
};

struct manifoldPoint2
{
    vec2 point{};
    float separation{};

    // 마지막 solver step에서 이 접점에 남은 누적 impulse.
    // debug draw / contact query에서 실제 solver 반응을 확인할 수 있게 공개함.
    float normalImpulse{};
    float tangentImpulse{};

    std::uint16_t id{};
};

struct manifold2
{
    vec2 normal{};
    std::array<manifoldPoint2, MAX_MANIFOLD_POINTS> points{};
    int pointCount{};
};

inline bool IsTouchingManifold( const localManifold2& manifold )
{
    for( int i = 0; i < manifold.pointCount; ++i )
    {
        if( manifold.points[i].separation <= 0.0f ) return true;
    }

    return false;
}

// Shape A local-space manifold을 world-space public manifold로 변환함.
inline manifold2 ToWorldManifold( const localManifold2& localManifold, const transform2& transformA )
{
    manifold2 manifold{};
    manifold.normal = TransformVector( transformA, localManifold.normal );
    manifold.pointCount = localManifold.pointCount;

    for( int i = 0; i < localManifold.pointCount; ++i )
    {
        manifold.points[i].point = TransformPoint( transformA, localManifold.points[i].point );

        manifold.points[i].separation = localManifold.points[i].separation;

        manifold.points[i].id = localManifold.points[i].id;
    }

    return manifold;
}

} // namespace zonai
