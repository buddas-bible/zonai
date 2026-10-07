#pragma once

#include "rigidBodyDemoUi.h"

namespace zonai::sandbox
{

[[nodiscard]] std::unique_ptr<demo> createJointDemoView( demoKind kind );

} // namespace zonai::sandbox
