from pathlib import Path

path = Path("tests/dynamics/pogo_joint_test.cpp")
text = path.read_text(encoding="utf-8")
marker = "    return EXIT_SUCCESS;\n}"
block = '''    // Box2D Pogo는 한 substep 동안 Prepare 시점의 pogo 축을 고정함.\n    // Body B의 deltaRotation은 anchor lever arm에는 반영하지만 길이 측정축에는 다시 곱하지 않음.\n    {\n        pogoJointSim2 joint = makePogo();\n        joint.localAnchorB = {};\n        joint.restLength = 0.0f;\n        auto constraint = preparePogoJointConstraint( joint, bodyA, bodyB, h );\n        bodyState stateA{};\n        bodyState stateB{};\n        stateB.deltaPosition = { 0.0f, 1.0f };\n        stateB.deltaRotation = rot2::FromRadians( 0.5f * 3.14159265358979323846f );\n\n        solvePogoJointConstraint( constraint, stateA, stateB, true );\n        check( stateB.linearVelocity.y < 0.0f, "Pogo keeps the prepared measurement axis fixed within the substep" );\n    }\n\n'''
if text.count(marker) != 1:
    raise RuntimeError("Pogo test insertion marker missing")
path.write_text(text.replace(marker, block + marker, 1), encoding="utf-8")
print("Added Pogo prepared-axis regression")
