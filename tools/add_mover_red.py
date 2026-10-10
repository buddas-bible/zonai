from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"{label} marker not found")
    return text.replace(old, new, 1)


# Demo registry
demo_h_path = Path("sandbox/demo.h")
demo_h = demo_h_path.read_text(encoding="utf-8")
demo_h = replace_once(
    demo_h,
    "    motorJointPlayground,\n    motorCar\n",
    "    motorJointPlayground,\n    moverJointPlayground,\n    motorCar\n",
    "demoKind Mover entry",
)
demo_h_path.write_text(demo_h, encoding="utf-8")

demo_cpp_path = Path("sandbox/demo.cpp")
demo_cpp = demo_cpp_path.read_text(encoding="utf-8")
motor_entry = '    demoEntry{ demoKind::motorJointPlayground, "조인트", "모터 조인트", "Motor Joint의 상대속도 actuator와 목표 transform spring을 비교합니다. 제동·선형·회전·스프링·결합 프리셋으로 속도 목표와 위치/각도 복원의 차이를 관찰합니다.", { 0.0f, 0.0f }, 95.0f },\n'
mover_entry = motor_entry + '    demoEntry{ demoKind::moverJointPlayground, "조인트", "무버 조인트", "Mover Joint는 두 Body의 COM 상대 선속도만 제어하고 회전은 건드리지 않습니다. 수평·수직·대각선·축별 힘 한도 프리셋으로 character mover용 velocity actuator를 비교합니다.", { 0.0f, 0.0f }, 95.0f },\n'
demo_cpp = replace_once( demo_cpp, motor_entry, mover_entry, "Mover demo registry" )
demo_cpp_path.write_text(demo_cpp, encoding="utf-8")

# Headless model public surface
header_path = Path("sandbox/rigidBodyDemo.h")
header = header_path.read_text(encoding="utf-8")
header = replace_once(
    header,
    '''enum class motorJointDemoPreset
{
    brake,
    linear,
    angular,
    combined,
    linearSpring,
    angularSpring,
    springBoth,
    velocityAndSpring
};
''',
    '''enum class motorJointDemoPreset
{
    brake,
    linear,
    angular,
    combined,
    linearSpring,
    angularSpring,
    springBoth,
    velocityAndSpring
};

// Mover Joint의 x/y 목표속도와 축별 힘 한도를 빠르게 비교하는 설정.
enum class moverJointDemoPreset
{
    horizontal,
    vertical,
    diagonal,
    anisotropic
};
''',
    "Mover preset enum",
)
header = replace_once(
    header,
    '    void setMotorJointAngularSpringSettings( float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque );\n',
    '    void setMotorJointAngularSpringSettings( float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque );\n    void setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce );\n',
    "Mover settings API",
)
header = replace_once(
    header,
    '    void applyMotorJointPreset( motorJointDemoPreset preset );\n',
    '    void applyMotorJointPreset( motorJointDemoPreset preset );\n    void applyMoverJointPreset( moverJointDemoPreset preset );\n',
    "Mover preset API",
)
header = replace_once(
    header,
    '    [[nodiscard]] jointId getMotorJoint() const noexcept { return motorJoint_; }\n',
    '    [[nodiscard]] jointId getMotorJoint() const noexcept { return motorJoint_; }\n\n    [[nodiscard]] jointId getMoverJoint() const noexcept { return moverJoint_; }\n',
    "Mover getter",
)
header = replace_once(
    header,
    '    void createMotorJointPlayground();\n',
    '    void createMotorJointPlayground();\n    void createMoverJointPlayground();\n',
    "Mover scene declaration",
)
header = replace_once(
    header,
    '    jointId motorJoint_{}; // Motor Joint 전용 학습 데모에서 사용하는 persistent handle.\n',
    '    jointId motorJoint_{}; // Motor Joint 전용 학습 데모에서 사용하는 persistent handle.\n    jointId moverJoint_{}; // Mover Joint 전용 학습 데모의 persistent handle.\n',
    "Mover member",
)
header_path.write_text(header, encoding="utf-8")

# Scene selection
model_path = Path("sandbox/rigidBodyDemo.cpp")
model = model_path.read_text(encoding="utf-8")
model = replace_once(
    model,
    '''    else if( kind == demoKind::motorJointPlayground )
    {
        createMotorJointPlayground();
    }
    else
''',
    '''    else if( kind == demoKind::motorJointPlayground )
    {
        createMotorJointPlayground();
    }
    else if( kind == demoKind::moverJointPlayground )
    {
        createMoverJointPlayground();
    }
    else
''',
    "Mover scene selection",
)
model_path.write_text(model, encoding="utf-8")

joint_demo_path = Path("sandbox/rigidBodyJointDemo.cpp")
joint_demo = joint_demo_path.read_text(encoding="utf-8")

settings_block = r'''
void rigidBodyDemo::setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce )
{
    if( kind_ != demoKind::moverJointPlayground || !world_.IsValid( moverJoint_ ) ) return;
    world_.setMoverJointLinearVelocity( moverJoint_, linearVelocity );
    world_.setMoverJointMaxVelocityForce( moverJoint_, maxVelocityForce );
}

'''
joint_demo = replace_once(
    joint_demo,
    '#pragma endregion Settings\n',
    settings_block + '#pragma endregion Settings\n',
    "Mover settings implementation",
)

preset_block = r'''
void rigidBodyDemo::applyMoverJointPreset( moverJointDemoPreset preset )
{
    if( kind_ != demoKind::moverJointPlayground || !world_.IsValid( moverJoint_ ) ) return;

    vec2 targetVelocity{};
    vec2 maxForce{ 20.0f, 20.0f };

    switch( preset )
    {
    case moverJointDemoPreset::horizontal:
        targetVelocity = { 2.0f, 0.0f };
        break;

    case moverJointDemoPreset::vertical:
        targetVelocity = { 0.0f, 2.0f };
        break;

    case moverJointDemoPreset::diagonal:
        targetVelocity = { 1.5f, 1.5f };
        break;

    case moverJointDemoPreset::anisotropic:
        targetVelocity = { 2.0f, 2.0f };
        // 같은 속도 목표라도 x/y actuator 예산을 다르게 줘 축별 clamp를 눈으로 비교함.
        maxForce = { 20.0f, 5.0f };
        break;
    }

    setMoverJointSettings( targetVelocity, maxForce );
}

'''
joint_demo = replace_once(
    joint_demo,
    '#pragma endregion Presets\n',
    preset_block + '#pragma endregion Presets\n',
    "Mover preset implementation",
)

scene_block = r'''void rigidBodyDemo::createMoverJointPlayground()
{
    world_.SetGravity( {} );

    // Mover는 Body 위치나 anchor를 제약하지 않음. A는 상대 선속도의 기준 Body로만 사용함.
    const bodyId reference = world_.CreateBody( bodyType::Static, { { -2.0f, 0.0f }, {} } );
    const shapeId referenceShape = world_.CreateShape( reference, MakeBox( { 0.25f, 0.25f } ) );
    shapes_.push_back( { reference, referenceShape, "기준 Body A [Mover]" } );

    const bodyId driven = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 0.0f }, {} } );
    const shapeId drivenShape = world_.CreateShape( driven, MakeBox( { 0.55f, 0.35f } ) );
    shapes_.push_back( { driven, drivenShape, "구동 Body B [Mover]" } );

    moverJointDef joint{};
    joint.bodyA = reference;
    joint.bodyB = driven;
    joint.linearVelocity = { 2.0f, 0.0f };
    joint.maxVelocityForce = { 20.0f, 20.0f };
    moverJoint_ = world_.createMoverJoint( joint );

    impulseBody_ = driven;
    torqueBody_ = driven;
}

'''
joint_demo = replace_once(
    joint_demo,
    'void rigidBodyDemo::createMotorJointPlayground()\n',
    scene_block + 'void rigidBodyDemo::createMotorJointPlayground()\n',
    "Mover scene implementation",
)
joint_demo_path.write_text(joint_demo, encoding="utf-8")

# A new independently selectable scene increases the registry count from 9 to 10.
for path_text in [
    "tests/sandbox/demo_test.cpp",
    "tests/sandbox/joint_demo_test.cpp",
    "tests/sandbox/motor_joint_demo_test.cpp",
]:
    path = Path(path_text)
    text = path.read_text(encoding="utf-8")
    text = text.replace("getDemoEntries().size() == 9", "getDemoEntries().size() == 10")
    text = text.replace("nine independently selectable demo entries", "ten independently selectable demo entries")
    path.write_text(text, encoding="utf-8")
