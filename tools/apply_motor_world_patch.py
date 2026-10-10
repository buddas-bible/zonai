from pathlib import Path

path = Path("src/dynamics/world.cpp")
text = path.read_text(encoding="utf-8")


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one match, found {count}")
    text = text.replace(old, new, 1)


motor_lifecycle = r'''jointId world::createMotorJoint( const motorJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );
    assert( bodies_[bodyIndexA].type == bodyType::Dynamic || bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.localAnchorA ) && IsFinite( definition.localAnchorB ) );
    assert( IsFinite( definition.linearVelocity ) );
    assert( std::isfinite( definition.maxVelocityForce ) && definition.maxVelocityForce >= 0.0f );
    assert( std::isfinite( definition.angularVelocity ) );
    assert( std::isfinite( definition.maxVelocityTorque ) && definition.maxVelocityTorque >= 0.0f );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );
    motorJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.localAnchorA = definition.localAnchorA;
    sim.localAnchorB = definition.localAnchorB;
    sim.linearVelocity = definition.linearVelocity;
    sim.maxVelocityForce = definition.maxVelocityForce;
    sim.angularVelocity = definition.angularVelocity;
    sim.maxVelocityTorque = definition.maxVelocityTorque;
    jointSims_[index] = sim;

    return makeJointId( index );
}

void world::setMotorJointLinearVelocity( jointId id, vec2 linearVelocity, float maxVelocityForce )
{
    assert( IsFinite( linearVelocity ) );
    assert( std::isfinite( maxVelocityForce ) && maxVelocityForce >= 0.0f );

    auto& joint = std::get<motorJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.linearVelocity.x == linearVelocity.x && joint.linearVelocity.y == linearVelocity.y && joint.maxVelocityForce == maxVelocityForce ) return;

    joint.linearVelocity = linearVelocity;
    joint.maxVelocityForce = maxVelocityForce;

    // 선형/회전 motor는 cache가 독립적이므로 선형 설정 변경은 선형 impulse만 버림.
    joint.linearVelocityImpulse = {};

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

void world::setMotorJointAngularVelocity( jointId id, float angularVelocity, float maxVelocityTorque )
{
    assert( std::isfinite( angularVelocity ) );
    assert( std::isfinite( maxVelocityTorque ) && maxVelocityTorque >= 0.0f );

    auto& joint = std::get<motorJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.angularVelocity == angularVelocity && joint.maxVelocityTorque == maxVelocityTorque ) return;

    joint.angularVelocity = angularVelocity;
    joint.maxVelocityTorque = maxVelocityTorque;

    // 선형/회전 motor는 cache가 독립적이므로 회전 설정 변경은 회전 impulse만 버림.
    joint.angularVelocityImpulse = 0.0f;

    if( bodies_[joint.bodyIdA].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdA );
    }
    if( bodies_[joint.bodyIdB].type != bodyType::Static )
    {
        WakeBodyByIndex( joint.bodyIdB );
    }
}

motorJointData world::getMotorJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const motorJointSim2& sim = std::get<motorJointSim2>( jointSims_[index] );

    motorJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.anchorA = TransformPoint( bodySims_[sim.bodyIdA].transform, sim.localAnchorA );
    data.anchorB = TransformPoint( bodySims_[sim.bodyIdB].transform, sim.localAnchorB );
    data.linearVelocity = sim.linearVelocity;
    data.maxVelocityForce = sim.maxVelocityForce;
    data.angularVelocity = sim.angularVelocity;
    data.maxVelocityTorque = sim.maxVelocityTorque;
    data.force = sim.subStepTime > 0.0f ? sim.linearVelocityImpulse / sim.subStepTime : vec2{};
    data.torque = sim.subStepTime > 0.0f ? sim.angularVelocityImpulse / sim.subStepTime : 0.0f;
    data.collideConnected = joints_[index].collideConnected;

    return data;
}

'''

replace_once(
    "jointId world::createRevoluteJoint( const revoluteJointDef& definition )\n",
    motor_lifecycle + "jointId world::createRevoluteJoint( const revoluteJointDef& definition )\n",
    "Motor lifecycle insertion",
)

replace_once(
    r'''                    if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, distanceJointSim2> )
                    {
                        return prepareDistanceJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, prismaticJointSim2> )
''',
    r'''                    if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, distanceJointSim2> )
                    {
                        return prepareDistanceJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, motorJointSim2> )
                    {
                        return prepareMotorJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, prismaticJointSim2> )
''',
    "Motor prepare dispatch",
)

old_store = r'''                    using constraintType = std::remove_cvref_t<decltype( constraint )>;
                    using simType = std::conditional_t<std::is_same_v<constraintType, distanceJointConstraint2>, distanceJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2, std::conditional_t<std::is_same_v<constraintType, revoluteJointConstraint2>, revoluteJointSim2, std::conditional_t<std::is_same_v<constraintType, weldJointConstraint2>, weldJointSim2, std::conditional_t<std::is_same_v<constraintType, wheelJointConstraint2>, wheelJointSim2, mouseJointSim2>>>>>;
                    auto& joint = std::get<simType>( jointSims_[constraint.jointId] );
                    joint.impulse = constraint.impulse;
                    joint.subStepTime = subStepTime;
                    if constexpr( std::is_same_v<simType, distanceJointSim2> || std::is_same_v<simType, revoluteJointSim2> )
                    {
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
                    if constexpr( std::is_same_v<simType, prismaticJointSim2> )
                    {
                        joint.springImpulse = constraint.springImpulse;
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
                    if constexpr( std::is_same_v<simType, weldJointSim2> )
                    {
                        joint.angularImpulse = constraint.angularImpulse;
                    }
                    if constexpr( std::is_same_v<simType, wheelJointSim2> )
                    {
                        joint.springImpulse = constraint.springImpulse;
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
'''

new_store = r'''                    using constraintType = std::remove_cvref_t<decltype( constraint )>;
                    using simType = std::conditional_t<std::is_same_v<constraintType, distanceJointConstraint2>, distanceJointSim2, std::conditional_t<std::is_same_v<constraintType, motorJointConstraint2>, motorJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2, std::conditional_t<std::is_same_v<constraintType, revoluteJointConstraint2>, revoluteJointSim2, std::conditional_t<std::is_same_v<constraintType, weldJointConstraint2>, weldJointSim2, std::conditional_t<std::is_same_v<constraintType, wheelJointConstraint2>, wheelJointSim2, mouseJointSim2>>>>>>;
                    auto& joint = std::get<simType>( jointSims_[constraint.jointId] );
                    joint.subStepTime = subStepTime;

                    if constexpr( std::is_same_v<simType, motorJointSim2> )
                    {
                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;
                        joint.angularVelocityImpulse = constraint.angularVelocityImpulse;
                    }
                    else
                    {
                        joint.impulse = constraint.impulse;
                    }

                    if constexpr( std::is_same_v<simType, distanceJointSim2> || std::is_same_v<simType, revoluteJointSim2> )
                    {
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
                    if constexpr( std::is_same_v<simType, prismaticJointSim2> )
                    {
                        joint.springImpulse = constraint.springImpulse;
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
                    if constexpr( std::is_same_v<simType, weldJointSim2> )
                    {
                        joint.angularImpulse = constraint.angularImpulse;
                    }
                    if constexpr( std::is_same_v<simType, wheelJointSim2> )
                    {
                        joint.springImpulse = constraint.springImpulse;
                        joint.lowerImpulse = constraint.lowerImpulse;
                        joint.upperImpulse = constraint.upperImpulse;
                        joint.motorImpulse = constraint.motorImpulse;
                    }
'''
replace_once(old_store, new_store, "Motor impulse store")

old_reset = r'''void world::resetJointImpulses( std::int32_t bodyIndex )
{
    for( std::int32_t key = bodies_[bodyIndex].headJointKey; key != -1; key = joints_[key >> 1].edges[key & 1].nextKey )
    {
        std::visit(
            []( auto& joint )
            {
                joint.impulse = {};
                using simType = std::remove_cvref_t<decltype( joint )>;
                if constexpr( std::is_same_v<simType, distanceJointSim2> || std::is_same_v<simType, revoluteJointSim2> )
                {
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, weldJointSim2> )
                {
                    joint.angularImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, prismaticJointSim2> )
                {
                    joint.springImpulse = 0.0f;
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, wheelJointSim2> )
                {
                    joint.springImpulse = 0.0f;
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
            },
            jointSims_[key >> 1] );
    }
}
'''

new_reset = r'''void world::resetJointImpulses( std::int32_t bodyIndex )
{
    for( std::int32_t key = bodies_[bodyIndex].headJointKey; key != -1; key = joints_[key >> 1].edges[key & 1].nextKey )
    {
        std::visit(
            []( auto& joint )
            {
                using simType = std::remove_cvref_t<decltype( joint )>;

                if constexpr( std::is_same_v<simType, motorJointSim2> )
                {
                    joint.linearVelocityImpulse = {};
                    joint.angularVelocityImpulse = 0.0f;
                }
                else
                {
                    joint.impulse = {};
                }

                if constexpr( std::is_same_v<simType, distanceJointSim2> || std::is_same_v<simType, revoluteJointSim2> )
                {
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, weldJointSim2> )
                {
                    joint.angularImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, prismaticJointSim2> )
                {
                    joint.springImpulse = 0.0f;
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
                if constexpr( std::is_same_v<simType, wheelJointSim2> )
                {
                    joint.springImpulse = 0.0f;
                    joint.lowerImpulse = 0.0f;
                    joint.upperImpulse = 0.0f;
                    joint.motorImpulse = 0.0f;
                }
            },
            jointSims_[key >> 1] );
    }
}
'''
replace_once(old_reset, new_reset, "Motor reset cache")

replace_once(
    r'''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )
                {
                    warmStartDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    r'''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )
                {
                    warmStartDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )
                {
                    warmStartMotorJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    "Motor warm-start dispatch",
)

replace_once(
    r'''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )
                {
                    solveDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    r'''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )
                {
                    solveDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )
                {
                    // Motor target은 실제로 남겨야 하는 물리 속도라 hard position bias처럼 relaxation에서 제거하지 않음.
                    solveMotorJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    "Motor solve dispatch",
)

path.write_text(text, encoding="utf-8")
