#pragma once

#include "rigidBodyDemoUi.h"

namespace zonai::sandbox
{

void drawRagdollJointLimits( debugDraw& draw, const rigidBodyDemo& model );
[[nodiscard]] std::unique_ptr<demo> createJointDemoView( demoKind kind );

} // namespace zonai::sandbox
