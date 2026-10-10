from pathlib import Path

path = Path("tests/dynamics/motor_joint_test.cpp")
text = path.read_text(encoding="utf-8")
marker = "    // World가 Motor를 공용 Joint graph / solver 경로에 넣고 query까지 되돌려주는지 검증함.\n"
if text.count(marker) != 1:
    raise RuntimeError("expected one Motor World marker")

regression = r'''    // Linear / angular Motor를 동시에 켜고 anchor가 COM 밖에 있으면 두 제약은 서로 결합됨.
    // 순차 angular -> linear solve만 두 번 호출해서는 linear impulse가 다시 만든 angular error가 남음.
    bodySim coupledBodyA{};
    coupledBodyA.bodyId = 2;
    bodySim coupledBodyB{};
    coupledBodyB.bodyId = 3;
    coupledBodyB.invMass = 1.0f;
    coupledBodyB.invInertia = 1.0f;

    motorJointSim2 coupledJoint{};
    coupledJoint.jointId = 1;
    coupledJoint.bodyIdA = 2;
    coupledJoint.bodyIdB = 3;
    coupledJoint.localAnchorB = { 0.0f, 1.0f };
    coupledJoint.linearVelocity = { 1.0f, 0.0f };
    coupledJoint.maxVelocityForce = 1000.0f;
    coupledJoint.angularVelocity = 0.0f;
    coupledJoint.maxVelocityTorque = 1000.0f;

    auto coupledConstraint = prepareMotorJointConstraint( coupledJoint, coupledBodyA, coupledBodyB, h );
    bodyState coupledStateA{};
    bodyState coupledStateB{};
    // World의 현재 Joint solve와 같은 두 번의 pass를 재현함.
    solveMotorJointConstraint( coupledConstraint, coupledStateA, coupledStateB );
    solveMotorJointConstraint( coupledConstraint, coupledStateA, coupledStateB );
    const vec2 coupledPointVelocity = coupledStateB.linearVelocity + Cross( coupledStateB.angularVelocity, coupledConstraint.anchorB );
    check( near( coupledPointVelocity.x, coupledJoint.linearVelocity.x ) && near( coupledPointVelocity.y, coupledJoint.linearVelocity.y ), "off-center combined Motor preserves linear target" );
    check( near( coupledStateB.angularVelocity - coupledStateA.angularVelocity, coupledJoint.angularVelocity ), "off-center combined Motor preserves angular target" );

'''

path.write_text(text.replace(marker, regression + marker, 1), encoding="utf-8")
