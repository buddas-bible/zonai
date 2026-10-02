#pragma once

#include <cstdint>

#include "dynamics/bodyType.h"

namespace zonai
{

struct body
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // World 내부 stable slot id. NULL_INDEX면 현재 free slot임.
    std::int32_t bodyId = NULL_INDEX;

    // slot이 재사용될 때 증가해 오래된 bodyId를 검출함.
    std::uint16_t generation = 0;

    // free slot일 때 다음 재사용 가능한 body index.
    std::int32_t nextFreeId = NULL_INDEX;

    // body의 물리 동작 종류.
    bodyType type = bodyType::Static;

    // Dynamic body에 연결된 shape 질량의 합.
    float mass = 0.0f;

    // Dynamic body의 center of mass 기준 회전 관성.
    float inertia = 0.0f;

    // Static이 아닌 body가 현재 solver에 참여하는지 나타냄.
    bool awake = true;

    // false면 자동 sleep 대상에서 제외됨.
    bool enableSleep = true;

    // sleep threshold 아래에서 연속으로 머문 시간.
    float sleepTime = 0.0f;

    // 가장 빠른 body point의 속도가 이 값 이하일 때 sleep timer가 증가함.
    float sleepThreshold = 0.05f;

    // CCD fast-body 판정 기준. 작을수록 더 이른 속도에서 continuous 대상으로 분류됨.
    float safetyFactor = 0.5f;

    // [contactId : edgeIndex] key로 연결된 첫 Contact.
    // 하위 1bit는 Contact의 어느 edge가 이 body에 연결됐는지 나타냄.
    std::int32_t headContactKey = NULL_INDEX;

    // 이 body에 연결된 Contact 개수.
    std::int32_t contactCount = 0;

    // 이 body에 연결된 첫 shape index.
    std::int32_t headShapeId = NULL_INDEX;

    // 이 body에 연결된 shape 개수.
    std::int32_t shapeCount = 0;
};

} // namespace zonai
