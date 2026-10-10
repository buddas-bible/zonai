from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, got {count}")
    return text.replace(old, new, 1)


world_h_path = Path("src/dynamics/world.h")
world_h = world_h_path.read_text(encoding="utf-8")
world_h = replace_once(
    world_h,
    """    // 회전 목표속도 / 토크 한도 변경은 회전 cache만 비우고 연결된 non-static component를 깨움.\n    void setMotorJointAngularVelocity( jointId id, float angularVelocity, float maxVelocityTorque );\n    [[nodiscard]] motorJointData getMotorJointData( jointId id ) const;\n""",
    """    // 회전 목표속도 / 토크 한도 변경은 회전 cache만 비우고 연결된 non-static component를 깨움.\n    void setMotorJointAngularVelocity( jointId id, float angularVelocity, float maxVelocityTorque );\n    // 두 anchor를 복원하는 선형 spring 설정. 변경 시 선형 spring cache만 비우고 component를 깨움.\n    void setMotorJointLinearSpring( jointId id, float hertz, float dampingRatio, float maxSpringForce );\n    // 기준 상대각도와 회전 spring 설정. 변경 시 회전 spring cache만 비우고 component를 깨움.\n    void setMotorJointAngularSpring( jointId id, float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque );\n    [[nodiscard]] motorJointData getMotorJointData( jointId id ) const;\n""",
    "world.h Motor API",
)
world_h_path.write_text(world_h, encoding="utf-8")

world_cpp_path = Path("src/dynamics/world.cpp")
world_cpp = world_cpp_path.read_text(encoding="utf-8")
world_cpp = replace_once(
    world_cpp,
    """    assert( std::isfinite( definition.angularVelocity ) );\n    assert( std::isfinite( definition.maxVelocityTorque ) && definition.maxVelocityTorque >= 0.0f );\n\n    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );\n""",
    """    assert( std::isfinite( definition.angularVelocity ) );\n    assert( std::isfinite( definition.maxVelocityTorque ) && definition.maxVelocityTorque >= 0.0f );\n    assert( std::isfinite( definition.referenceAngle ) );\n    assert( std::isfinite( definition.linearHertz ) && definition.linearHertz >= 0.0f );\n    assert( std::isfinite( definition.linearDampingRatio ) && definition.linearDampingRatio >= 0.0f );\n    assert( std::isfinite( definition.maxSpringForce ) && definition.maxSpringForce >= 0.0f );\n    assert( std::isfinite( definition.angularHertz ) && definition.angularHertz >= 0.0f );\n    assert( std::isfinite( definition.angularDampingRatio ) && definition.angularDampingRatio >= 0.0f );\n    assert( std::isfinite( definition.maxSpringTorque ) && definition.maxSpringTorque >= 0.0f );\n\n    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );\n""",
    "Motor create validation",
)
world_cpp = replace_once(
    world_cpp,
    """    sim.angularVelocity = definition.angularVelocity;\n    sim.maxVelocityTorque = definition.maxVelocityTorque;\n    jointSims_[index] = sim;\n""",
    """    sim.angularVelocity = definition.angularVelocity;\n    sim.maxVelocityTorque = definition.maxVelocityTorque;\n    sim.referenceAngle = definition.referenceAngle;\n    sim.linearHertz = definition.linearHertz;\n    sim.linearDampingRatio = definition.linearDampingRatio;\n    sim.maxSpringForce = definition.maxSpringForce;\n    sim.angularHertz = definition.angularHertz;\n    sim.angularDampingRatio = definition.angularDampingRatio;\n    sim.maxSpringTorque = definition.maxSpringTorque;\n    jointSims_[index] = sim;\n""",
    "Motor create storage",
)
world_cpp = replace_once(
    world_cpp,
    """}\n\nmotorJointData world::getMotorJointData( jointId id ) const\n""",
    """}\n\nvoid world::setMotorJointLinearSpring( jointId id, float hertz, float dampingRatio, float maxSpringForce )\n{\n    assert( std::isfinite( hertz ) && hertz >= 0.0f );\n    assert( std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );\n    assert( std::isfinite( maxSpringForce ) && maxSpringForce >= 0.0f );\n\n    auto& joint = std::get<motorJointSim2>( jointSims_[getJointIndex( id )] );\n    if( joint.linearHertz == hertz && joint.linearDampingRatio == dampingRatio && joint.maxSpringForce == maxSpringForce ) return;\n\n    joint.linearHertz = hertz;\n    joint.linearDampingRatio = dampingRatio;\n    joint.maxSpringForce = maxSpringForce;\n    // Velocity Motor와 spring은 별도 actuator이므로 spring 설정 변경은 spring cache만 버림.\n    joint.linearSpringImpulse = {};\n\n    if( bodies_[joint.bodyIdA].type != bodyType::Static )\n    {\n        WakeBodyByIndex( joint.bodyIdA );\n    }\n    if( bodies_[joint.bodyIdB].type != bodyType::Static )\n    {\n        WakeBodyByIndex( joint.bodyIdB );\n    }\n}\n\nvoid world::setMotorJointAngularSpring( jointId id, float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque )\n{\n    assert( std::isfinite( referenceAngle ) );\n    assert( std::isfinite( hertz ) && hertz >= 0.0f );\n    assert( std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );\n    assert( std::isfinite( maxSpringTorque ) && maxSpringTorque >= 0.0f );\n\n    auto& joint = std::get<motorJointSim2>( jointSims_[getJointIndex( id )] );\n    if( joint.referenceAngle == referenceAngle && joint.angularHertz == hertz && joint.angularDampingRatio == dampingRatio && joint.maxSpringTorque == maxSpringTorque ) return;\n\n    joint.referenceAngle = referenceAngle;\n    joint.angularHertz = hertz;\n    joint.angularDampingRatio = dampingRatio;\n    joint.maxSpringTorque = maxSpringTorque;\n    // 기준 상대각도나 회전 spring 계수가 바뀌면 이전 spring 해는 더 이상 같은 제약의 해가 아님.\n    joint.angularSpringImpulse = 0.0f;\n\n    if( bodies_[joint.bodyIdA].type != bodyType::Static )\n    {\n        WakeBodyByIndex( joint.bodyIdA );\n    }\n    if( bodies_[joint.bodyIdB].type != bodyType::Static )\n    {\n        WakeBodyByIndex( joint.bodyIdB );\n    }\n}\n\nmotorJointData world::getMotorJointData( jointId id ) const\n""",
    "Motor spring setters",
)
world_cpp = replace_once(
    world_cpp,
    """    data.angularVelocity = sim.angularVelocity;\n    data.maxVelocityTorque = sim.maxVelocityTorque;\n    data.force = sim.subStepTime > 0.0f ? sim.linearVelocityImpulse / sim.subStepTime : vec2{};\n    data.torque = sim.subStepTime > 0.0f ? sim.angularVelocityImpulse / sim.subStepTime : 0.0f;\n    data.collideConnected = joints_[index].collideConnected;\n""",
    """    data.angularVelocity = sim.angularVelocity;\n    data.maxVelocityTorque = sim.maxVelocityTorque;\n    data.referenceAngle = sim.referenceAngle;\n    data.linearHertz = sim.linearHertz;\n    data.linearDampingRatio = sim.linearDampingRatio;\n    data.maxSpringForce = sim.maxSpringForce;\n    data.angularHertz = sim.angularHertz;\n    data.angularDampingRatio = sim.angularDampingRatio;\n    data.maxSpringTorque = sim.maxSpringTorque;\n    // Velocity Motor와 transform spring은 별도 cache지만 둘 다 실제 물리 반력에 기여함.\n    data.force = sim.subStepTime > 0.0f ? ( sim.linearVelocityImpulse + sim.linearSpringImpulse ) / sim.subStepTime : vec2{};\n    data.torque = sim.subStepTime > 0.0f ? ( sim.angularVelocityImpulse + sim.angularSpringImpulse ) / sim.subStepTime : 0.0f;\n    data.collideConnected = joints_[index].collideConnected;\n""",
    "Motor data query",
)
world_cpp = replace_once(
    world_cpp,
    """                    if constexpr( std::is_same_v<simType, motorJointSim2> )\n                    {\n                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;\n                        joint.angularVelocityImpulse = constraint.angularVelocityImpulse;\n                    }\n""",
    """                    if constexpr( std::is_same_v<simType, motorJointSim2> )\n                    {\n                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;\n                        joint.linearSpringImpulse = constraint.linearSpringImpulse;\n                        joint.angularVelocityImpulse = constraint.angularVelocityImpulse;\n                        joint.angularSpringImpulse = constraint.angularSpringImpulse;\n                    }\n""",
    "Motor impulse store",
)
world_cpp = replace_once(
    world_cpp,
    """                if constexpr( std::is_same_v<simType, motorJointSim2> )\n                {\n                    joint.linearVelocityImpulse = {};\n                    joint.angularVelocityImpulse = 0.0f;\n                }\n""",
    """                if constexpr( std::is_same_v<simType, motorJointSim2> )\n                {\n                    joint.linearVelocityImpulse = {};\n                    joint.linearSpringImpulse = {};\n                    joint.angularVelocityImpulse = 0.0f;\n                    joint.angularSpringImpulse = 0.0f;\n                }\n""",
    "Motor impulse reset",
)
world_cpp_path.write_text(world_cpp, encoding="utf-8")
