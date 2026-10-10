#include "dynamics/joints/filterJointConstraint2.h"

namespace zonai
{

filterJointConstraint2 prepareFilterJointConstraint( const filterJointSim2& joint )
{
    return { joint.jointId, joint.bodyIdA, joint.bodyIdB };
}

void warmStartFilterJointConstraint( const filterJointConstraint2& )
{
    // Filter Joint는 solver impulse가 없으므로 warm start할 상태도 없음.
}

void solveFilterJointConstraint( filterJointConstraint2& )
{
    // Collision 차단은 broad-phase/contact 생성 경로에서 처리하고 속도/위치는 건드리지 않음.
}

} // namespace zonai
