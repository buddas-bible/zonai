#pragma once

#include <array>
#include <cstdint>

namespace zonai
{

// Body 하나의 Joint 연결 목록에서 사용하는 edge.
// key는 (jointId << 1) | edgeIndex 형태이며 prev/next는 같은 Body에 연결된 다른 Joint edge를 가리킴.
struct jointEdge2
{
    std::int32_t bodyId = -1; // 이 edge가 속한 Body의 simulation index.
    std::int32_t prevKey = -1;
    std::int32_t nextKey = -1;
};

// 모든 Joint 종류가 공통으로 사용하는 stable slot과 Body graph 연결 정보.
// 실제 Joint별 설정/solver cache는 같은 slot의 *JointSim2에 따로 저장함.
struct joint2
{
    std::int32_t jointId = -1; // joints_ / jointSims_의 stable slot index.
    std::uint16_t generation = 0; // slot 재사용 후 오래된 jointId handle을 거부하기 위한 세대값.
    std::int32_t nextFree = -1; // slot이 비어 있을 때 free-list의 다음 slot을 가리킴.
    std::array<jointEdge2, 2> edges{}; // [0]은 Body A, [1]은 Body B 쪽 연결 edge.
    bool collideConnected = false; // false이면 연결된 두 Body 사이의 Contact 생성을 막음.
};

} // namespace zonai
