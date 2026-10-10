#pragma once

#include "dynamics/id.h"

namespace zonai
{

// 두 Body 사이의 collision만 제어하는 solver-less Joint.
// 별도 위치/속도 제약은 만들지 않지만 공용 Joint edge를 사용하므로 island / sleep은 연결됨.
// Filter를 켜고 끄는 수명 자체가 collision 상태 전환이므로 별도 runtime setter는 두지 않음.
struct filterJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // false이면 두 Body 사이 Contact를 제거하고 이후 collision 후보도 차단함.
    // true이면 collision은 유지하고 Joint graph 연결만 사용함.
    bool collideConnected = false;
};

struct filterJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    bool collideConnected = false;
};

} // namespace zonai
