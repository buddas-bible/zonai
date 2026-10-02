#pragma once

#include <cstdint>
#include <variant>

#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"

#include "collision/aabb2.h"
#include "collision/filter.h"
#include "collision/massData2.h"
#include "math/transform2.h"

namespace zonai
{

// Box2D의 shape type + union을 C++20에서 안전하게 표현함.
// monostate는 아직 geometry가 연결되지 않은 shape 상태임.
using shapeGeometry =
    std::variant<
        std::monostate,
        circle2,
        capsule2,
        polygon2,
        segment2
    >;

struct shapeExtent2
{
    float minExtent = 0.0f;
    float maxExtent = 0.0f;
};

struct shape
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // 이 shape를 소유하는 body index. 아직 연결되지 않았으면 NULL_INDEX임.
    std::int32_t bodyId = NULL_INDEX;

    // 같은 body에 연결된 이전 / 다음 shape index.
    // body의 head부터 index 기반 doubly linked list로 순회함.
    std::int32_t prevShapeId = NULL_INDEX;
    std::int32_t nextShapeId = NULL_INDEX;

    // slot이 재사용될 때 증가해 오래된 shapeId를 검출함.
    std::uint16_t generation = 0;

    // sensor overlap 저장소 index. NULL_INDEX면 일반 collision shape임.
    std::int32_t sensorIndex = NULL_INDEX;

    // free slot일 때 다음 재사용 가능한 shape index.
    std::int32_t nextFreeId = NULL_INDEX;

    // broadPhase에 등록된 proxy key. 아직 proxy가 없으면 NULL_INDEX임.
    std::int32_t proxyKey = NULL_INDEX;

    // 현재 transform에서 geometry bounds에 speculative distance를 더한 AABB.
    // narrowphase 후보 유지에 쓰이며 Dynamic Tree의 fat AABB와는 구분됨.
    aabb2 aabb{};

    // Dynamic Tree fat AABB에 추가하는 shape 크기 기반 margin.
    float aabbMargin = 0.0f;

    // body local space에 저장되는 실제 collision geometry.
    // variant가 Box2D의 type + union 역할을 대신함.
    shapeGeometry geometry{};

    // 면적당 질량. Box2D 기본값과 같이 1로 시작함.
    float density = 1.0f;

    // 접촉면의 Coulomb friction coefficient.
    float friction = 0.6f;

    // 충돌 전 normal 상대속도 중 얼마를 반대 방향으로 되돌릴지 결정함.
    float restitution = 0.0f;

    // category / mask / group 기반 collision filter.
    collisionFilter filter{};
};

// body local geometry에 transform을 적용해 world-space AABB를 계산함.
aabb2 ComputeShapeAABB(
    const shapeGeometry& geometry,
    const transform2& transform );

// Dynamic Tree fat AABB에 사용할 shape 크기 기반 margin을 계산함.
float ComputeShapeAABBMargin(
    const shapeGeometry& geometry );

// localCenter 기준으로 CCD에 사용할 최소 / 최대 shape extent를 계산함.
shapeExtent2 ComputeShapeExtent(
    const shapeGeometry& geometry,
    const vec2& localCenter );

// shape geometry와 density로 local-space 질량 특성을 계산함.
massData2 ComputeShapeMass( const shape& shape );

} // namespace zonai
