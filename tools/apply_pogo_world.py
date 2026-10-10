from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    file = Path(path)
    text = file.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one marker, found {count}: {old[:80]!r}")
    file.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_once(
    "src/dynamics/world.h",
    '#include "dynamics/joints/moverJointConstraint2.h"\n#include "dynamics/joints/mouseJoint2.h"',
    '#include "dynamics/joints/moverJointConstraint2.h"\n#include "dynamics/joints/pogoJoint2.h"\n#include "dynamics/joints/pogoJointConstraint2.h"\n#include "dynamics/joints/mouseJoint2.h"',
)

replace_once(
    "src/dynamics/world.h",
    '    [[nodiscard]] moverJointData getMoverJointData( jointId id ) const;\n\n    // 두 작용점을 일치시키며 상대 회전은 허용함.',
    '    [[nodiscard]] moverJointData getMoverJointData( jointId id ) const;\n\n'
    '    // Pogo 축으로 spring 길이를 측정하고 contact normal 방향으로 반력을 가하는 character support Joint.\n'
    '    [[nodiscard]] jointId createPogoJoint( const pogoJointDef& definition );\n'
    '    // rest length / 주파수 / 감쇠 변경. 같은 값이면 깨우지 않음.\n'
    '    void setPogoJointSpring( jointId id, float restLength, float hertz, float dampingRatio );\n'
    '    // 인장/압축 최대 힘을 각각 설정함. 다음 Prepare에서 cached impulse도 새 한도로 제한함.\n'
    '    void setPogoJointForceLimits( jointId id, float maxTensionForce, float maxCompressionForce );\n'
    '    [[nodiscard]] pogoJointData getPogoJointData( jointId id ) const;\n\n'
    '    // 두 작용점을 일치시키며 상대 회전은 허용함.',
)

replace_once(
    "src/dynamics/world.h",
    'std::vector<std::variant<distanceJointSim2, motorJointSim2, moverJointSim2, mouseJointSim2, prismaticJointSim2, revoluteJointSim2, weldJointSim2, wheelJointSim2>> jointSims_;',
    'std::vector<std::variant<distanceJointSim2, motorJointSim2, moverJointSim2, pogoJointSim2, mouseJointSim2, prismaticJointSim2, revoluteJointSim2, weldJointSim2, wheelJointSim2>> jointSims_;',
)

replace_once(
    "src/dynamics/world.h",
    'using jointConstraint = std::variant<distanceJointConstraint2, motorJointConstraint2, moverJointConstraint2, mouseJointConstraint2, prismaticJointConstraint2, revoluteJointConstraint2, weldJointConstraint2, wheelJointConstraint2>;',
    'using jointConstraint = std::variant<distanceJointConstraint2, motorJointConstraint2, moverJointConstraint2, pogoJointConstraint2, mouseJointConstraint2, prismaticJointConstraint2, revoluteJointConstraint2, weldJointConstraint2, wheelJointConstraint2>;',
)

replace_once(
    "src/dynamics/world.cpp",
    '''                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, moverJointSim2> )\n                    {\n                        return prepareMoverJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );\n                    }\n                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, prismaticJointSim2> )''',
    '''                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, moverJointSim2> )\n                    {\n                        return prepareMoverJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );\n                    }\n                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, pogoJointSim2> )\n                    {\n                        return preparePogoJointConstraint( joint, bodySims_[joint.bodyIdA], bodySims_[joint.bodyIdB], subStepTime );\n                    }\n                    else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( joint )>, prismaticJointSim2> )''',
)

replace_once(
    "src/dynamics/world.cpp",
    'std::conditional_t<std::is_same_v<constraintType, moverJointConstraint2>, moverJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2',
    'std::conditional_t<std::is_same_v<constraintType, moverJointConstraint2>, moverJointSim2, std::conditional_t<std::is_same_v<constraintType, pogoJointConstraint2>, pogoJointSim2, std::conditional_t<std::is_same_v<constraintType, prismaticJointConstraint2>, prismaticJointSim2',
)

replace_once(
    "src/dynamics/world.cpp",
    '''                    else if constexpr( std::is_same_v<simType, moverJointSim2> )\n                    {\n                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;\n                    }\n                    else\n                    {\n                        joint.impulse = constraint.impulse;\n                    }''',
    '''                    else if constexpr( std::is_same_v<simType, moverJointSim2> )\n                    {\n                        joint.linearVelocityImpulse = constraint.linearVelocityImpulse;\n                    }\n                    else if constexpr( std::is_same_v<simType, pogoJointSim2> )\n                    {\n                        joint.impulse = constraint.impulse;\n                        joint.velocity = constraint.velocity;\n                    }\n                    else\n                    {\n                        joint.impulse = constraint.impulse;\n                    }''',
)

replace_once(
    "src/dynamics/world.cpp",
    '''                else if constexpr( std::is_same_v<simType, moverJointSim2> )\n                {\n                    joint.linearVelocityImpulse = {};\n                }\n                else\n                {\n                    joint.impulse = {};\n                }''',
    '''                else if constexpr( std::is_same_v<simType, moverJointSim2> )\n                {\n                    joint.linearVelocityImpulse = {};\n                }\n                else if constexpr( std::is_same_v<simType, pogoJointSim2> )\n                {\n                    joint.impulse = 0.0f;\n                    joint.velocity = 0.0f;\n                }\n                else\n                {\n                    joint.impulse = {};\n                }''',
)

replace_once(
    "src/dynamics/world.cpp",
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, moverJointConstraint2> )\n                {\n                    warmStartMoverJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )''',
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, moverJointConstraint2> )\n                {\n                    warmStartMoverJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, pogoJointConstraint2> )\n                {\n                    warmStartPogoJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )''',
)

replace_once(
    "src/dynamics/world.cpp",
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, moverJointConstraint2> )\n                {\n                    // Mover도 실제 상대 선속도를 만드는 actuator라 두 solve pass 모두에서 같은 목표를 유지함.\n                    solveMoverJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )''',
    '''                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, moverJointConstraint2> )\n                {\n                    // Mover도 실제 상대 선속도를 만드는 actuator라 두 solve pass 모두에서 같은 목표를 유지함.\n                    solveMoverJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB] );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, pogoJointConstraint2> )\n                {\n                    // Pogo의 spring 보정 속도는 bias pass에서만 만들고 relaxation에서 제거함.\n                    solvePogoJointConstraint( constraint, bodyStates_[constraint.bodyIdA], bodyStates_[constraint.bodyIdB], useBias );\n                }\n                else if constexpr( std::is_same_v<std::remove_cvref_t<decltype( constraint )>, prismaticJointConstraint2> )''',
)

print("Applied Pogo World integration")
