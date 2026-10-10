from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected 1 match, got {count}")
    return text.replace(old, new, 1)

world_h = Path("src/dynamics/world.h")
h = world_h.read_text(encoding="utf-8")

h = replace_once(
    h,
    '#include "dynamics/joints/distanceJointConstraint2.h"\n',
    '#include "dynamics/joints/distanceJointConstraint2.h"\n#include "dynamics/joints/filterJoint2.h"\n#include "dynamics/joints/filterJointConstraint2.h"\n',
    "world.h Filter includes",
)

h = replace_once(
    h,
    '    // 유한한 목표 축속도와 비음수 최대 힘. 변경 시 cache를 비우고 연결된 component를 깨움.\n    void setDistanceJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce );\n\n',
    '    // 유한한 목표 축속도와 비음수 최대 힘. 변경 시 cache를 비우고 연결된 component를 깨움.\n    void setDistanceJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce );\n\n    // 두 Body 사이 collision만 선택적으로 차단하는 solver-less Joint. Joint graph / sleep 연결은 유지함.\n    [[nodiscard]] jointId createFilterJoint( const filterJointDef& definition );\n    [[nodiscard]] filterJointData getFilterJointData( jointId id ) const;\n\n',
    "world.h Filter API",
)

h = replace_once(
    h,
    'std::vector<std::variant<distanceJointSim2, motorJointSim2, moverJointSim2, pogoJointSim2, mouseJointSim2, prismaticJointSim2, revoluteJointSim2, weldJointSim2, wheelJointSim2>> jointSims_;',
    'std::vector<std::variant<distanceJointSim2, filterJointSim2, motorJointSim2, moverJointSim2, pogoJointSim2, mouseJointSim2, prismaticJointSim2, revoluteJointSim2, weldJointSim2, wheelJointSim2>> jointSims_;',
    "world.h joint sim variant",
)

h = replace_once(
    h,
    'using jointConstraint = std::variant<distanceJointConstraint2, motorJointConstraint2, moverJointConstraint2, pogoJointConstraint2, mouseJointConstraint2, prismaticJointConstraint2, revoluteJointConstraint2, weldJointConstraint2, wheelJointConstraint2>;',
    'using jointConstraint = std::variant<distanceJointConstraint2, filterJointConstraint2, motorJointConstraint2, moverJointConstraint2, pogoJointConstraint2, mouseJointConstraint2, prismaticJointConstraint2, revoluteJointConstraint2, weldJointConstraint2, wheelJointConstraint2>;',
    "world.h joint constraint variant",
)

world_h.write_text(h, encoding="utf-8")

world_cpp = Path("src/dynamics/world.cpp")
c = world_cpp.read_text(encoding="utf-8")

c = replace_once(
    c,
    '''                    if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, distanceJointSim2> )\n                    {\n                        return prepareDistanceJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );\n                    }\n                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, motorJointSim2> )''',
    '''                    if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, distanceJointSim2> )\n                    {\n                        return prepareDistanceJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );\n                    }\n                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, filterJointSim2> )\n                    {\n                        return prepareFilterJointConstraint( joint );\n                    }\n                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, motorJointSim2> )''',
    "world.cpp Filter prepare dispatch",
)

old_mapping = 'using simType = std::conditional_t<std::is_same_v<constraintType, distanceJointConstraint2>, distanceJointSim2, std::conditional_t<std::is_same_v<constraintType, motorJointConstraint2>, motorJointSim2, std::conditional_t<std::is_same_v<constraintType, moverJointConstraint2>, moverJointSim2, std::conditional_t<std::is_same_v<constraintType, pogoJointConstraint2>, pogoJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2, std::conditional_t<std::is_same_v<constraintType, revoluteJointConstraint2>, revoluteJointSim2, std::conditional_t<std::is_same_v<constraintType, weldJointConstraint2>, weldJointSim2, std::conditional_t<std::is_same_v<constraintType, wheelJointConstraint2>, wheelJointSim2, mouseJointSim2>>>>>>>>;'
new_mapping = 'using simType = std::conditional_t<std::is_same_v<constraintType, distanceJointConstraint2>, distanceJointSim2, std::conditional_t<std::is_same_v<constraintType, filterJointConstraint2>, filterJointSim2, std::conditional_t<std::is_same_v<constraintType, motorJointConstraint2>, motorJointSim2, std::conditional_t<std::is_same_v<constraintType, moverJointConstraint2>, moverJointSim2, std::conditional_t<std::is_same_v<constraintType, pogoJointConstraint2>, pogoJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2, std::conditional_t<std::is_same_v<constraintType, revoluteJointConstraint2>, revoluteJointSim2, std::conditional_t<std::is_same_v<constraintType, weldJointConstraint2>, weldJointSim2, std::conditional_t<std::is_same_v<constraintType, wheelJointConstraint2>, wheelJointSim2, mouseJointSim2>>>>>>>>>;'
c = replace_once(c, old_mapping, new_mapping, "world.cpp Filter store mapping")

c = replace_once(
    c,
    '                    joint.subStepTime = subStepTime;\n\n                    if constexpr( std::is_same_v<simType, motorJointSim2> )',
    '                    if constexpr( !std::is_same_v<simType, filterJointSim2> ) joint.subStepTime = subStepTime;\n\n                    if constexpr( std::is_same_v<simType, motorJointSim2> )',
    "world.cpp Filter store timestep",
)

c = replace_once(
    c,
    '''                    else if constexpr( std::is_same_v<simType, pogoJointSim2> )\n                    {\n                        joint.impulse = constraint.impulse;\n                        joint.velocity = constraint.velocity;\n                    }\n                    else\n                    {\n                        joint.impulse = constraint.impulse;\n                    }''',
    '''                    else if constexpr( std::is_same_v<simType, pogoJointSim2> )\n                    {\n                        joint.impulse = constraint.impulse;\n                        joint.velocity = constraint.velocity;\n                    }\n                    else if constexpr( std::is_same_v<simType, filterJointSim2> )\n                    {\n                        // Filter Joint는 solver state를 저장하지 않음.\n                    }\n                    else\n                    {\n                        joint.impulse = constraint.impulse;\n                    }''',
    "world.cpp Filter store no-op",
)

c = replace_once(
    c,
    '''                if constexpr( std::is_same_v<simType, motorJointSim2> )\n                {\n                    joint.linearVelocityImpulse = {};''',
    '''                if constexpr( std::is_same_v<simType, filterJointSim2> )\n                {\n                    // Filter Joint는 reset할 solver cache가 없음.\n                }\n                else if constexpr( std::is_same_v<simType, motorJointSim2> )\n                {\n                    joint.linearVelocityImpulse = {};''',
    "world.cpp Filter reset no-op",
)

c = replace_once(
    c,
    '''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )\n                {\n                    warmStartDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )''',
    '''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )\n                {\n                    warmStartDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, filterJointConstraint2> )\n                {\n                    warmStartFilterJointConstraint( constraint );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )''',
    "world.cpp Filter warm-start dispatch",
)

c = replace_once(
    c,
    '''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )\n                {\n                    solveDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )''',
    '''                if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, distanceJointConstraint2> )\n                {\n                    solveDistanceJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, filterJointConstraint2> )\n                {\n                    solveFilterJointConstraint( constraint );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, motorJointConstraint2> )''',
    "world.cpp Filter solve dispatch",
)

world_cpp.write_text(c, encoding="utf-8")
print("Applied Filter Joint core integration")
