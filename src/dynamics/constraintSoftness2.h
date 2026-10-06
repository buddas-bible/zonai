#pragma once

namespace zonai
{

// Box2D의 soft constraint 계수. massScale + impulseScale = 1로 correction을 감쇠함.
struct constraintSoftness2
{
    float biasRate = 0.0f;
    float massScale = 1.0f;
    float impulseScale = 0.0f;
};

[[nodiscard]] constraintSoftness2 makeConstraintSoftness( float hertz, float dampingRatio, float timeStep );

} // namespace zonai
