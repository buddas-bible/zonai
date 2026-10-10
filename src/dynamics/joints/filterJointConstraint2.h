#pragma once

#include "dynamics/joints/filterJointSim2.h"

namespace zonai
{

// Filter Joint는 solver impulse를 만들지 않음.
// island의 joint index와 solver constraint 배열을 1:1로 유지하기 위한 빈 packet임.
struct filterJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
};

[[nodiscard]] filterJointConstraint2 prepareFilterJointConstraint( const filterJointSim2& joint );
void warmStartFilterJointConstraint( const filterJointConstraint2& constraint );
void solveFilterJointConstraint( filterJointConstraint2& constraint );

} // namespace zonai
