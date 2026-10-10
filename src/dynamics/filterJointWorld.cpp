#include "dynamics/world.h"

#include <cassert>

namespace zonai
{

#pragma region FilterJointLifecycle

jointId world::createFilterJoint( const filterJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );

    filterJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    jointSims_[index] = sim;

    return makeJointId( index );
}

filterJointData world::getFilterJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const filterJointSim2& sim = std::get<filterJointSim2>( jointSims_[index] );

    filterJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.collideConnected = joints_[index].collideConnected;
    return data;
}

#pragma endregion FilterJointLifecycle

} // namespace zonai
