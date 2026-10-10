from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    file = Path(path)
    text = file.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one marker, found {count}: {old[:100]!r}")
    file.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_once(
    "sandbox/demo.h",
    "    moverJointPlayground,\n    motorCar",
    "    moverJointPlayground,\n    pogoJointPlayground,\n    motorCar",
)

replace_once(
    "sandbox/demo.cpp",
    '    demoEntry{ demoKind::moverJointPlayground, "조인트", "무버 조인트", "Mover Joint는 두 Body의 COM 상대 선속도만 제어하고 회전은 건드리지 않습니다. 수평·수직·대각선·축별 힘 한도 프리셋으로 character mover용 velocity actuator를 비교합니다.", { 0.0f, 0.0f }, 95.0f },\n    demoEntry{ demoKind::motorCar,',
    '    demoEntry{ demoKind::moverJointPlayground, "조인트", "무버 조인트", "Mover Joint는 두 Body의 COM 상대 선속도만 제어하고 회전은 건드리지 않습니다. 수평·수직·대각선·축별 힘 한도 프리셋으로 character mover용 velocity actuator를 비교합니다.", { 0.0f, 0.0f }, 95.0f },\n    demoEntry{ demoKind::pogoJointPlayground, "조인트", "포고 조인트", "Pogo Joint는 길이 오차를 pogo 축으로 측정하지만 지면 contact normal 방향으로 반력을 가합니다. Soft·Stiff·압축 전용·비대칭 힘 프리셋으로 character support 동작을 비교합니다.", { 0.0f, 0.7f }, 110.0f },\n    demoEntry{ demoKind::motorCar,',
)

replace_once(
    "sandbox/rigidBodyDemo.h",
    "enum class moverJointDemoPreset\n{\n    horizontal,\n    vertical,\n    diagonal,\n    anisotropic\n};\n",
    "enum class moverJointDemoPreset\n{\n    horizontal,\n    vertical,\n    diagonal,\n    anisotropic\n};\n\n// Pogo의 spring 응답과 인장/압축 force budget 차이를 빠르게 비교하는 설정.\nenum class pogoJointDemoPreset\n{\n    soft,\n    stiff,\n    compressionOnly,\n    asymmetric\n};\n",
)

replace_once(
    "sandbox/rigidBodyDemo.h",
    "    void setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce );\n",
    "    void setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce );\n    void setPogoJointSettings( float restLength, float hertz, float dampingRatio, float maxTensionForce, float maxCompressionForce );\n",
)

replace_once(
    "sandbox/rigidBodyDemo.h",
    "    void applyMoverJointPreset( moverJointDemoPreset preset );\n",
    "    void applyMoverJointPreset( moverJointDemoPreset preset );\n    void applyPogoJointPreset( pogoJointDemoPreset preset );\n",
)

replace_once(
    "sandbox/rigidBodyDemo.h",
    "    [[nodiscard]] jointId getMoverJoint() const noexcept { return moverJoint_; }\n",
    "    [[nodiscard]] jointId getMoverJoint() const noexcept { return moverJoint_; }\n\n    [[nodiscard]] jointId getPogoJoint() const noexcept { return pogoJoint_; }\n",
)

replace_once(
    "sandbox/rigidBodyDemo.h",
    "    void createMoverJointPlayground();\n    void createMotorCar();",
    "    void createMoverJointPlayground();\n    void createPogoJointPlayground();\n    void createMotorCar();",
)

replace_once(
    "sandbox/rigidBodyDemo.h",
    "    jointId moverJoint_{}; // Mover Joint 전용 학습 데모의 persistent handle.\n",
    "    jointId moverJoint_{}; // Mover Joint 전용 학습 데모의 persistent handle.\n    jointId pogoJoint_{}; // Pogo Joint 전용 학습 데모의 persistent handle.\n",
)

replace_once(
    "sandbox/rigidBodyDemo.cpp",
    "    else if( kind == demoKind::moverJointPlayground )\n    {\n        createMoverJointPlayground();\n    }\n    else\n    {\n        assert( kind == demoKind::motorCar );",
    "    else if( kind == demoKind::moverJointPlayground )\n    {\n        createMoverJointPlayground();\n    }\n    else if( kind == demoKind::pogoJointPlayground )\n    {\n        createPogoJointPlayground();\n    }\n    else\n    {\n        assert( kind == demoKind::motorCar );",
)

replace_once(
    "sandbox/rigidBodyJointDemo.cpp",
    "void rigidBodyDemo::setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce )\n{\n    if( kind_ != demoKind::moverJointPlayground || !world_.IsValid( moverJoint_ ) ) return;\n    world_.setMoverJointLinearVelocity( moverJoint_, linearVelocity );\n    world_.setMoverJointMaxVelocityForce( moverJoint_, maxVelocityForce );\n}\n\n#pragma endregion Settings",
    "void rigidBodyDemo::setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce )\n{\n    if( kind_ != demoKind::moverJointPlayground || !world_.IsValid( moverJoint_ ) ) return;\n    world_.setMoverJointLinearVelocity( moverJoint_, linearVelocity );\n    world_.setMoverJointMaxVelocityForce( moverJoint_, maxVelocityForce );\n}\n\nvoid rigidBodyDemo::setPogoJointSettings( float restLength, float hertz, float dampingRatio, float maxTensionForce, float maxCompressionForce )\n{\n    if( kind_ != demoKind::pogoJointPlayground || !world_.IsValid( pogoJoint_ ) ) return;\n    world_.setPogoJointSpring( pogoJoint_, restLength, hertz, dampingRatio );\n    world_.setPogoJointForceLimits( pogoJoint_, maxTensionForce, maxCompressionForce );\n}\n\n#pragma endregion Settings",
)

replace_once(
    "sandbox/rigidBodyJointDemo.cpp",
    "    setMoverJointSettings( targetVelocity, maxForce );\n}\n\n#pragma endregion Presets",
    "    setMoverJointSettings( targetVelocity, maxForce );\n}\n\nvoid rigidBodyDemo::applyPogoJointPreset( pogoJointDemoPreset preset )\n{\n    if( kind_ != demoKind::pogoJointPlayground || !world_.IsValid( pogoJoint_ ) ) return;\n\n    constexpr float restLength = 0.8f;\n    constexpr float dampingRatio = 0.7f;\n    float hertz = 2.0f;\n    float maxTensionForce = 50.0f;\n    float maxCompressionForce = 200.0f;\n\n    switch( preset )\n    {\n    case pogoJointDemoPreset::soft:\n        break;\n\n    case pogoJointDemoPreset::stiff:\n        hertz = 8.0f;\n        maxTensionForce = 100.0f;\n        maxCompressionForce = 400.0f;\n        break;\n\n    case pogoJointDemoPreset::compressionOnly:\n        hertz = 4.0f;\n        maxTensionForce = 0.0f;\n        maxCompressionForce = 300.0f;\n        break;\n\n    case pogoJointDemoPreset::asymmetric:\n        hertz = 4.0f;\n        maxTensionForce = 40.0f;\n        maxCompressionForce = 350.0f;\n        break;\n    }\n\n    setPogoJointSettings( restLength, hertz, dampingRatio, maxTensionForce, maxCompressionForce );\n}\n\n#pragma endregion Presets",
)

replace_once(
    "sandbox/rigidBodyJointDemo.cpp",
    "void rigidBodyDemo::createMotorJointPlayground()\n{",
    "void rigidBodyDemo::createPogoJointPlayground()\n{\n    world_.SetGravity( { 0.0f, -9.8f } );\n\n    const bodyId ground = world_.CreateBody( bodyType::Static, { { 0.0f, -0.25f }, {} } );\n    const shapeId groundShape = world_.CreateShape( ground, MakeBox( { 3.0f, 0.25f } ) );\n    shapes_.push_back( { ground, groundShape, \"지면 Body A [Pogo]\" } );\n\n    const bodyId character = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 0.75f }, {} } );\n    const shapeId characterShape = world_.CreateShape( character, MakeBox( { 0.4f, 0.35f } ) );\n    shapes_.push_back( { character, characterShape, \"캐릭터 Body B [Pogo]\" } );\n\n    pogoJointDef joint{};\n    joint.bodyA = ground;\n    joint.bodyB = character;\n    joint.localAnchorA = { 0.0f, 0.25f };\n    joint.localAnchorB = { 0.0f, -0.35f };\n    joint.localPogoAxisB = { 0.0f, 1.0f };\n    joint.normal = { 0.0f, 1.0f };\n    joint.restLength = 0.8f;\n    joint.hertz = 2.0f;\n    joint.dampingRatio = 0.7f;\n    joint.maxTensionForce = 50.0f;\n    joint.maxCompressionForce = 200.0f;\n    // 실제 character mover처럼 Pogo와 별개로 ground contact도 유지할 수 있게 함.\n    joint.collideConnected = true;\n    pogoJoint_ = world_.createPogoJoint( joint );\n\n    impulseBody_ = character;\n    torqueBody_ = character;\n}\n\nvoid rigidBodyDemo::createMotorJointPlayground()\n{",
)

replace_once(
    "sandbox/jointDemoView.cpp",
    "kind != demoKind::motorJointPlayground && kind != demoKind::moverJointPlayground ) return;",
    "kind != demoKind::motorJointPlayground && kind != demoKind::moverJointPlayground && kind != demoKind::pogoJointPlayground ) return;",
)

replace_once(
    "sandbox/jointDemoView.cpp",
    "            else if( kind == demoKind::weldPair )\n            {",
    "            else if( kind == demoKind::pogoJointPlayground )\n            {\n                if( ImGui::Button( \"Soft###PogoPresetSoft\" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::soft );\n                ImGui::SameLine();\n                if( ImGui::Button( \"Stiff###PogoPresetStiff\" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::stiff );\n                ImGui::SameLine();\n                if( ImGui::Button( \"압축 전용###PogoPresetCompressionOnly\" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::compressionOnly );\n                ImGui::SameLine();\n                if( ImGui::Button( \"비대칭###PogoPresetAsymmetric\" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::asymmetric );\n                ImGui::TextWrapped( \"Pogo는 길이를 재는 축과 실제 반력 normal을 분리합니다. 인장/압축 힘 한도도 독립적으로 조절할 수 있습니다.\" );\n            }\n            else if( kind == demoKind::weldPair )\n            {",
)

replace_once(
    "sandbox/jointDemoView.cpp",
    "        else if( kind == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n        {",
    "        else if( kind == demoKind::pogoJointPlayground && view_.getWorld().IsValid( view_.getPogoJoint() ) )\n        {\n            drawPogoJointInspector();\n        }\n        else if( kind == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n        {",
)

replace_once(
    "sandbox/jointDemoView.cpp",
    "        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n        {",
    "        if( view_.getKind() == demoKind::pogoJointPlayground && view_.getWorld().IsValid( view_.getPogoJoint() ) )\n        {\n            const pogoJointData joint = view_.getWorld().getPogoJointData( view_.getPogoJoint() );\n            constexpr ImU32 ANCHOR_A_COLOR = IM_COL32( 100, 235, 220, 255 );\n            constexpr ImU32 ANCHOR_B_COLOR = IM_COL32( 255, 220, 90, 255 );\n            constexpr ImU32 AXIS_COLOR = IM_COL32( 110, 180, 255, 255 );\n            constexpr ImU32 NORMAL_COLOR = IM_COL32( 90, 220, 110, 255 );\n            constexpr ImU32 FORCE_COLOR = IM_COL32( 230, 120, 255, 255 );\n\n            draw.DrawPoint( joint.anchorA, ANCHOR_A_COLOR, 7.0f );\n            draw.DrawPoint( joint.anchorB, ANCHOR_B_COLOR, 7.0f );\n            // Pogo는 이 두 점 사이의 길이를 실제로 복원하므로 Mover와 달리 연결선을 표시함.\n            draw.DrawSegment( { joint.anchorA, joint.anchorB }, IM_COL32( 180, 185, 200, 255 ), 2.0f );\n            draw.DrawArrow( joint.anchorB, joint.pogoAxis, AXIS_COLOR, 0.7f );\n            draw.DrawArrow( joint.anchorA, joint.normal, NORMAL_COLOR, 0.7f );\n            if( LengthSquared( joint.force ) > 0.0f )\n            {\n                draw.DrawArrow( joint.anchorB, Normalize( joint.force ), FORCE_COLOR, std::min( 1.25f, Length( joint.force ) * 0.01f ) );\n            }\n            return;\n        }\n\n        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n        {",
)

replace_once(
    "sandbox/jointDemoView.cpp",
    "private:\n    void drawMoverJointInspector()",
    "private:\n    void drawPogoJointInspector()\n    {\n        if( !ImGui::CollapsingHeader( \"포고 조인트 인스펙터###PogoJointInspector\", ImGuiTreeNodeFlags_DefaultOpen ) ) return;\n\n        pogoJointData joint = view_.getWorld().getPogoJointData( view_.getPogoJoint() );\n        ImGui::TextWrapped( \"길이 오차는 pogo axis로 측정하고 실제 impulse는 contact normal 방향으로 작용합니다. 위치 복원 속도는 integration 뒤 relaxation에서 제거됩니다.\" );\n\n        float restLength = joint.restLength;\n        float hertz = joint.hertz;\n        float dampingRatio = joint.dampingRatio;\n        float maxTensionForce = joint.maxTensionForce;\n        float maxCompressionForce = joint.maxCompressionForce;\n        bool changed = ImGui::DragFloat( \"Rest Length###PogoRestLength\", &restLength, 0.02f, 0.0f, 5.0f, \"%.2f m\" );\n        changed |= ImGui::DragFloat( \"주파수###PogoHertz\", &hertz, 0.1f, 0.0f, 30.0f, \"%.2f Hz\" );\n        changed |= ImGui::DragFloat( \"감쇠비###PogoDamping\", &dampingRatio, 0.02f, 0.0f, 2.0f, \"%.2f\" );\n        changed |= ImGui::DragFloat( \"최대 인장 힘###PogoTensionForce\", &maxTensionForce, 1.0f, 0.0f, 2000.0f, \"%.1f N\" );\n        changed |= ImGui::DragFloat( \"최대 압축 힘###PogoCompressionForce\", &maxCompressionForce, 1.0f, 0.0f, 2000.0f, \"%.1f N\" );\n        restLength = std::max( 0.0f, restLength );\n        hertz = std::max( 0.0f, hertz );\n        dampingRatio = std::max( 0.0f, dampingRatio );\n        maxTensionForce = std::max( 0.0f, maxTensionForce );\n        maxCompressionForce = std::max( 0.0f, maxCompressionForce );\n        if( changed ) view_.setPogoJointSettings( restLength, hertz, dampingRatio, maxTensionForce, maxCompressionForce );\n\n        joint = view_.getWorld().getPogoJointData( view_.getPogoJoint() );\n        ImGui::Text( \"Current Length: %.3f m\", joint.length );\n        ImGui::Text( \"Spring Velocity: %.3f m/s\", joint.velocity );\n        ImGui::Text( \"Reaction Force: (%.2f, %.2f) N\", joint.force.x, joint.force.y );\n    }\n\n    void drawMoverJointInspector()",
)

for path in [
    "tests/sandbox/demo_test.cpp",
    "tests/sandbox/joint_demo_test.cpp",
    "tests/sandbox/motor_joint_demo_test.cpp",
    "tests/sandbox/mover_joint_demo_test.cpp",
]:
    file = Path(path)
    text = file.read_text(encoding="utf-8")
    if "getDemoEntries().size() == 10" not in text:
        raise RuntimeError(f"{path}: demo count marker missing")
    file.write_text(text.replace("getDemoEntries().size() == 10", "getDemoEntries().size() == 11", 1), encoding="utf-8")

print("Applied Pogo Sandbox implementation")
