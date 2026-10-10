from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"{label} marker not found")
    return text.replace(old, new, 1)


Path("src/dynamics/joints/moverJoint2.h").write_text(r'''#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// Mover Joint 생성 시 World에 전달하는 입력 설정.
// 두 Body의 COM 상대 선속도만 제어하며 회전에는 어떤 제약이나 torque도 만들지 않음.
struct moverJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // World 좌표계에서 B가 A에 대해 가져야 하는 목표 상대 선속도.
    vec2 linearVelocity{};

    // x/y 방향별 최대 구동 힘. 서로 독립적으로 제한해서 축마다 다른 가속 능력을 줄 수 있음.
    vec2 maxVelocityForce{};

    bool collideConnected = false;
};

// getMoverJointData()가 반환하는 현재 Mover Joint 상태 snapshot.
struct moverJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 linearVelocity{};
    vec2 maxVelocityForce{};

    // 마지막 substep의 누적 impulse / h. Body B에 작용한 Mover의 실제 force임.
    vec2 force{};

    bool collideConnected = false;
};

} // namespace zonai
''', encoding="utf-8")

world_h_path = Path("src/dynamics/world.h")
world_h = world_h_path.read_text(encoding="utf-8")
world_h = replace_once(
    world_h,
    '#include "dynamics/joints/motorJointConstraint2.h"\n',
    '#include "dynamics/joints/motorJointConstraint2.h"\n#include "dynamics/joints/moverJoint2.h"\n#include "dynamics/joints/moverJointConstraint2.h"\n',
    "world.h Mover includes",
)
world_h = replace_once(
    world_h,
    '    [[nodiscard]] motorJointData getMotorJointData( jointId id ) const;\n\n    // 두 작용점을 일치시키며 상대 회전은 허용함. 서로 다른 Body 중 하나 이상은 Dynamic이어야 함.\n',
    '    [[nodiscard]] motorJointData getMotorJointData( jointId id ) const;\n\n'
    '    // 두 Body의 COM 상대 선속도만 제어하며 회전은 건드리지 않음. 하나 이상은 Dynamic이어야 함.\n'
    '    [[nodiscard]] jointId createMoverJoint( const moverJointDef& definition );\n'
    '    // 목표 상대 선속도 변경은 누적 impulse를 비우고 연결된 non-static component를 깨움.\n'
    '    void setMoverJointLinearVelocity( jointId id, vec2 linearVelocity );\n'
    '    // x/y 방향별 최대 힘 변경은 누적 impulse를 비우고 연결된 non-static component를 깨움.\n'
    '    void setMoverJointMaxVelocityForce( jointId id, vec2 maxVelocityForce );\n'
    '    [[nodiscard]] moverJointData getMoverJointData( jointId id ) const;\n\n'
    '    // 두 작용점을 일치시키며 상대 회전은 허용함. 서로 다른 Body 중 하나 이상은 Dynamic이어야 함.\n',
    "world.h Mover API",
)
world_h = replace_once(
    world_h,
    '    using jointConstraint = std::variant<distanceJointConstraint2, motorJointConstraint2, mouseJointConstraint2, prismaticJointConstraint2, revoluteJointConstraint2, weldJointConstraint2, wheelJointConstraint2>;\n',
    '    using jointConstraint = std::variant<distanceJointConstraint2, motorJointConstraint2, moverJointConstraint2, mouseJointConstraint2, prismaticJointConstraint2, revoluteJointConstraint2, weldJointConstraint2, wheelJointConstraint2>;\n',
    "world.h constraint variant",
)
world_h = replace_once(
    world_h,
    '    std::vector<std::variant<distanceJointSim2, motorJointSim2, mouseJointSim2, prismaticJointSim2, revoluteJointSim2, weldJointSim2, wheelJointSim2>> jointSims_;\n',
    '    std::vector<std::variant<distanceJointSim2, motorJointSim2, moverJointSim2, mouseJointSim2, prismaticJointSim2, revoluteJointSim2, weldJointSim2, wheelJointSim2>> jointSims_;\n',
    "world.h sim variant",
)
world_h_path.write_text(world_h, encoding="utf-8")

world_cpp_path = Path("src/dynamics/world.cpp")
world_cpp = world_cpp_path.read_text(encoding="utf-8")

mover_lifecycle = r'''jointId world::createMoverJoint( const moverJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );
    assert( bodies_[bodyIndexA].type == bodyType::Dynamic || bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.linearVelocity ) );
    assert( IsFinite( definition.maxVelocityForce ) );
    assert( definition.maxVelocityForce.x >= 0.0f && definition.maxVelocityForce.y >= 0.0f );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );
    moverJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.linearVelocity = definition.linearVelocity;
    sim.maxVelocityForce = definition.maxVelocityForce;
    jointSims_[index] = sim;

    return makeJointId( index );
}

void world::setMoverJointLinearVelocity( jointId id, vec2 linearVelocity )
{
    assert( IsFinite( linearVelocity ) );

    auto& joint = std::get<moverJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.linearVelocity.x == linearVelocity.x && joint.linearVelocity.y == linearVelocity.y ) return;

    joint.linearVelocity = linearVelocity;
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

void world::setMoverJointMaxVelocityForce( jointId id, vec2 maxVelocityForce )
{
    assert( IsFinite( maxVelocityForce ) );
    assert( maxVelocityForce.x >= 0.0f && maxVelocityForce.y >= 0.0f );

    auto& joint = std::get<moverJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.maxVelocityForce.x == maxVelocityForce.x && joint.maxVelocityForce.y == maxVelocityForce.y ) return;

    joint.maxVelocityForce = maxVelocityForce;
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

moverJointData world::getMoverJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const moverJointSim2& sim = std::get<moverJointSim2>( jointSims_[index] );

    moverJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.linearVelocity = sim.linearVelocity;
    data.maxVelocityForce = sim.maxVelocityForce;
    data.force = sim.subStepTime > 0.0f ? sim.linearVelocityImpulse / sim.subStepTime : vec2{};
    data.collideConnected = joints_[index].collideConnected;
    return data;
}

'''
world_cpp = replace_once(
    world_cpp,
    'jointId world::createRevoluteJoint( const revoluteJointDef& definition )\n',
    mover_lifecycle + 'jointId world::createRevoluteJoint( const revoluteJointDef& definition )\n',
    "world.cpp Mover lifecycle",
)

world_cpp = replace_once(
    world_cpp,
    '''                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, motorJointSim2> )
                    {
                        return prepareMotorJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, prismaticJointSim2> )
''',
    '''                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, motorJointSim2> )
                    {
                        return prepareMotorJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, moverJointSim2> )
                    {
                        return prepareMoverJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );
                    }
                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, prismaticJointSim2> )
''',
    "world.cpp prepare Mover",
)
world_cpp = replace_once(
    world_cpp,
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )
                {
                    warmStartMotorJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )
                {
                    warmStartMotorJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, moverJointConstraint2> )
                {
                    warmStartMoverJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    "world.cpp warm Mover",
)
world_cpp = replace_once(
    world_cpp,
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )
                {
                    // Motor target은 실제로 남겨야 하는 물리 속도라 hard position bias처럼 relaxation에서 제거하지 않음.
                    solveMotorJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )
                {
                    // Motor target은 실제로 남겨야 하는 물리 속도라 hard position bias처럼 relaxation에서 제거하지 않음.
                    solveMotorJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, moverJointConstraint2> )
                {
                    // Mover도 실제 상대 선속도를 만드는 actuator라 두 solve pass 모두에서 같은 목표를 유지함.
                    solveMoverJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );
                }
                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )
''',
    "world.cpp solve Mover",
)

old_mapping = '                    using simType = std::conditional_t<std::is_same_v<constraintType, distanceJointConstraint2>, distanceJointSim2, std::conditional_t<std::is_same_v<constraintType, motorJointConstraint2>, motorJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2, std::conditional_t<std::is_same_v<constraintType, revoluteJointConstraint2>, revoluteJointSim2, std::conditional_t<std::is_same_v<constraintType, weldJointConstraint2>, weldJointSim2, std::conditional_t<std::is_same_v<constraintType, wheelJointConstraint2>, wheelJointSim2, mouseJointSim2>>>>>>;\n'
new_mapping = '                    using simType = std::conditional_t<std::is_same_v<constraintType, distanceJointConstraint2>, distanceJointSim2, std::conditional_t<std::is_same_v<constraintType, motorJointConstraint2>, motorJointSim2, std::conditional_t<std::is_same_v<constraintType, moverJointConstraint2>, moverJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2, std::conditional_t<std::is_same_v<constraintType, revoluteJointConstraint2>, revoluteJointSim2, std::conditional_t<std::is_same_v<constraintType, weldJointConstraint2>, weldJointSim2, std::conditional_t<std::is_same_v<constraintType, wheelJointConstraint2>, wheelJointSim2, mouseJointSim2>>>>>>>;\n'
world_cpp = replace_once( world_cpp, old_mapping, new_mapping, "world.cpp store type mapping" )
world_cpp = replace_once(
    world_cpp,
    '''                    if constexpr( std::is_same_v<simType, motorJointSim2> )
                    {
                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;
                        joint.linearSpringImpulse = constraint.linearSpringImpulse;
                        joint.angularVelocityImpulse = constraint.angularVelocityImpulse;
                        joint.angularSpringImpulse = constraint.angularSpringImpulse;
                    }
                    else
''',
    '''                    if constexpr( std::is_same_v<simType, motorJointSim2> )
                    {
                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;
                        joint.linearSpringImpulse = constraint.linearSpringImpulse;
                        joint.angularVelocityImpulse = constraint.angularVelocityImpulse;
                        joint.angularSpringImpulse = constraint.angularSpringImpulse;
                    }
                    else if constexpr( std::is_same_v<simType, moverJointSim2> )
                    {
                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;
                    }
                    else
''',
    "world.cpp store Mover impulse",
)
world_cpp = replace_once(
    world_cpp,
    '''                if constexpr( std::is_same_v<simType, motorJointSim2> )
                {
                    joint.linearVelocityImpulse = {};
                    joint.linearSpringImpulse = {};
                    joint.angularVelocityImpulse = 0.0f;
                    joint.angularSpringImpulse = 0.0f;
                }
                else
''',
    '''                if constexpr( std::is_same_v<simType, motorJointSim2> )
                {
                    joint.linearVelocityImpulse = {};
                    joint.linearSpringImpulse = {};
                    joint.angularVelocityImpulse = 0.0f;
                    joint.angularSpringImpulse = 0.0f;
                }
                else if constexpr( std::is_same_v<simType, moverJointSim2> )
                {
                    joint.linearVelocityImpulse = {};
                }
                else
''',
    "world.cpp reset Mover impulse",
)
world_cpp_path.write_text(world_cpp, encoding="utf-8")
