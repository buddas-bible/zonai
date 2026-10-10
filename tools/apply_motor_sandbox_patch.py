from pathlib import Path


def patch(path_name: str, replacements: list[tuple[str, str, str]]) -> None:
    path = Path(path_name)
    text = path.read_text(encoding="utf-8")
    for old, new, label in replacements:
        count = text.count(old)
        if count != 1:
            raise RuntimeError(f"{path_name}: {label}: expected one match, found {count}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")


patch("sandbox/demo.h", [
    (
        "    weldPair,\n    mouseJointPlayground,\n    motorCar\n",
        "    weldPair,\n    mouseJointPlayground,\n    motorJointPlayground,\n    motorCar\n",
        "demo kind",
    ),
])

patch("sandbox/rigidBodyDemo.h", [
    (
        "enum class weldDemoPreset\n{\n    rigid,\n    softLinear,\n    softAngular,\n    softBoth\n};\n",
        "enum class weldDemoPreset\n{\n    rigid,\n    softLinear,\n    softAngular,\n    softBoth\n};\n\n// Motor Joint 자체의 선형/회전 velocity 채널을 독립적으로 비교하는 빠른 설정.\nenum class motorJointDemoPreset\n{\n    brake,\n    linear,\n    angular,\n    combined\n};\n",
        "Motor preset enum",
    ),
    (
        "    void setWeldAngularSettings( float hertz, float dampingRatio );\n    void applyDistancePreset( distanceDemoPreset preset );\n",
        "    void setWeldAngularSettings( float hertz, float dampingRatio );\n    void setMotorJointLinearSettings( vec2 linearVelocity, float maxVelocityForce );\n    void setMotorJointAngularSettings( float angularVelocity, float maxVelocityTorque );\n    void applyDistancePreset( distanceDemoPreset preset );\n",
        "Motor settings declarations",
    ),
    (
        "    void applyWeldPreset( weldDemoPreset preset );\n",
        "    void applyWeldPreset( weldDemoPreset preset );\n    void applyMotorJointPreset( motorJointDemoPreset preset );\n",
        "Motor preset declaration",
    ),
    (
        "    [[nodiscard]] jointId getWeldJoint() const noexcept { return weldJoint_; }\n\n    [[nodiscard]] float getPrismaticCurrentSpeed() const;\n",
        "    [[nodiscard]] jointId getWeldJoint() const noexcept { return weldJoint_; }\n\n    [[nodiscard]] jointId getMotorJoint() const noexcept { return motorJoint_; }\n\n    [[nodiscard]] float getPrismaticCurrentSpeed() const;\n",
        "Motor getter",
    ),
    (
        "    void createWeldPair();\n    void createMouseJointPlayground();\n",
        "    void createWeldPair();\n    void createMouseJointPlayground();\n    void createMotorJointPlayground();\n",
        "Motor scene declaration",
    ),
    (
        "    jointId weldJoint_{};\n    std::array<jointId, 2> carJoints_{};\n",
        "    jointId weldJoint_{};\n    jointId motorJoint_{}; // Motor Joint 전용 학습 데모에서 사용하는 persistent handle.\n    std::array<jointId, 2> carJoints_{};\n",
        "Motor stored handle",
    ),
])

patch("sandbox/rigidBodyDemo.cpp", [
    (
        "    else if( kind == demoKind::mouseJointPlayground )\n    {\n        createMouseJointPlayground();\n    }\n    else\n",
        "    else if( kind == demoKind::mouseJointPlayground )\n    {\n        createMouseJointPlayground();\n    }\n    else if( kind == demoKind::motorJointPlayground )\n    {\n        createMotorJointPlayground();\n    }\n    else\n",
        "Motor scene dispatch",
    ),
])

patch("sandbox/demo.cpp", [
    (
        "    demoEntry{ demoKind::weldPair, \"조인트\", \"웰드 조인트\", \"캔버스: A/D 유지로 오른쪽 상자를 밀기, 스페이스로 위쪽 충격량, S로 회전 충격량을 가합니다. 한쪽에만 힘을 줘도 두 상자의 연결점과 상대 각도가 유지되어 하나의 강체처럼 움직이는지 관찰합니다.\", { 0.0f, 0.0f }, 100.0f },\n    demoEntry{ demoKind::motorCar,",
        "    demoEntry{ demoKind::weldPair, \"조인트\", \"웰드 조인트\", \"캔버스: A/D 유지로 오른쪽 상자를 밀기, 스페이스로 위쪽 충격량, S로 회전 충격량을 가합니다. 한쪽에만 힘을 줘도 두 상자의 연결점과 상대 각도가 유지되어 하나의 강체처럼 움직이는지 관찰합니다.\", { 0.0f, 0.0f }, 100.0f },\n    demoEntry{ demoKind::motorJointPlayground, \"조인트\", \"모터 조인트\", \"Motor Joint는 위치를 고정하지 않고 두 Body의 상대 선속도와 상대 각속도를 목표값으로 만듭니다. 제동·선형·회전·결합 프리셋과 힘/토크 한도를 비교합니다.\", { 0.0f, 0.0f }, 95.0f },\n    demoEntry{ demoKind::motorCar,",
        "Motor demo entry",
    ),
])

settings = r'''
void rigidBodyDemo::setMotorJointLinearSettings( vec2 linearVelocity, float maxVelocityForce )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointLinearVelocity( motorJoint_, linearVelocity, maxVelocityForce );
}

void rigidBodyDemo::setMotorJointAngularSettings( float angularVelocity, float maxVelocityTorque )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointAngularVelocity( motorJoint_, angularVelocity, maxVelocityTorque );
}

'''

preset = r'''
void rigidBodyDemo::applyMotorJointPreset( motorJointDemoPreset preset )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;

    constexpr vec2 linearVelocity{ 2.0f, 0.5f };
    constexpr float maxVelocityForce = 20.0f;
    constexpr float angularVelocity = 2.0f;
    constexpr float maxVelocityTorque = 10.0f;

    vec2 targetLinearVelocity{};
    float linearForce = 0.0f;
    float targetAngularVelocity = 0.0f;
    float angularTorque = 0.0f;

    switch( preset )
    {
    case motorJointDemoPreset::brake:
        // 목표속도 0에 유한한 힘/토크를 주면 transform lock이 아니라 속도를 죽이는 brake가 됨.
        linearForce = maxVelocityForce;
        angularTorque = maxVelocityTorque;
        break;

    case motorJointDemoPreset::linear:
        targetLinearVelocity = linearVelocity;
        linearForce = maxVelocityForce;
        break;

    case motorJointDemoPreset::angular:
        targetAngularVelocity = angularVelocity;
        angularTorque = maxVelocityTorque;
        break;

    case motorJointDemoPreset::combined:
        targetLinearVelocity = linearVelocity;
        linearForce = maxVelocityForce;
        targetAngularVelocity = angularVelocity;
        angularTorque = maxVelocityTorque;
        break;
    }

    world_.setMotorJointLinearVelocity( motorJoint_, targetLinearVelocity, linearForce );
    world_.setMotorJointAngularVelocity( motorJoint_, targetAngularVelocity, angularTorque );
}

'''

scene = r'''
void rigidBodyDemo::createMotorJointPlayground()
{
    world_.SetGravity( {} );

    // A는 움직이지 않는 기준 frame, B는 Motor의 목표 상대속도를 따라가는 Dynamic body임.
    const bodyId reference = world_.CreateBody( bodyType::Static, { { -2.0f, 0.0f }, {} } );
    const shapeId referenceShape = world_.CreateShape( reference, MakeBox( { 0.25f, 0.25f } ) );
    shapes_.push_back( { reference, referenceShape, "기준 Body A [정적]" } );

    const bodyId driven = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 0.0f }, {} } );
    const shapeId drivenShape = world_.CreateShape( driven, MakeBox( { 0.55f, 0.35f } ) );
    shapes_.push_back( { driven, drivenShape, "구동 Body B [Motor Joint]" } );

    motorJointDef joint{};
    joint.bodyA = reference;
    joint.bodyB = driven;
    joint.maxVelocityForce = 20.0f;
    joint.maxVelocityTorque = 10.0f;
    motorJoint_ = world_.createMotorJoint( joint );

    impulseBody_ = driven;
    torqueBody_ = driven;
}

'''

patch("sandbox/rigidBodyJointDemo.cpp", [
    (
        "void rigidBodyDemo::setWeldAngularSettings( float hertz, float dampingRatio )\n{\n    if( kind_ != demoKind::weldPair || !world_.IsValid( weldJoint_ ) ) return;\n    world_.setWeldJointAngularTuning( weldJoint_, hertz, dampingRatio );\n}\n\n#pragma endregion Settings\n",
        "void rigidBodyDemo::setWeldAngularSettings( float hertz, float dampingRatio )\n{\n    if( kind_ != demoKind::weldPair || !world_.IsValid( weldJoint_ ) ) return;\n    world_.setWeldJointAngularTuning( weldJoint_, hertz, dampingRatio );\n}\n\n" + settings + "#pragma endregion Settings\n",
        "Motor settings definitions",
    ),
    (
        "\n#pragma endregion Presets\n",
        "\n" + preset + "#pragma endregion Presets\n",
        "Motor preset definition",
    ),
    (
        "void rigidBodyDemo::createMouseJointPlayground()\n",
        scene + "void rigidBodyDemo::createMouseJointPlayground()\n",
        "Motor scene definition",
    ),
])

quick = r'''            else if( kind == demoKind::motorJointPlayground )
            {
                if( ImGui::Button( "제동###MotorJointPresetBrake" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::brake );
                ImGui::SameLine();
                if( ImGui::Button( "선형###MotorJointPresetLinear" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::linear );
                ImGui::SameLine();
                if( ImGui::Button( "회전###MotorJointPresetAngular" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::angular );
                ImGui::SameLine();
                if( ImGui::Button( "둘 다###MotorJointPresetCombined" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::combined );
                ImGui::TextWrapped( "Motor Joint는 두 Body의 상대 transform을 고정하지 않습니다. 목표 상대속도 0은 위치 고정이 아니라 힘/토크 한도 안에서 제동하는 상태입니다." );
            }
'''

inspector = r'''    void drawMotorJointInspector()
    {
        if( !ImGui::CollapsingHeader( "모터 조인트 인스펙터###MotorJointInspector", ImGuiTreeNodeFlags_DefaultOpen ) ) return;

        motorJointData joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
        ImGui::TextWrapped( "선형과 회전 채널은 위치 오차가 아니라 Cdot = 상대속도 - 목표속도를 0으로 만듭니다. 최대 힘/토크는 한 substep의 누적 impulse 한도를 결정합니다." );

        if( ImGui::TreeNodeEx( "선형 Velocity Motor###MotorJointLinearSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            vec2 velocity = joint.linearVelocity;
            float maxForce = joint.maxVelocityForce;
            bool changed = ImGui::DragFloat( "목표 X###MotorJointLinearX", &velocity.x, 0.05f, -10.0f, 10.0f, "%.2f m/s" );
            changed |= ImGui::DragFloat( "목표 Y###MotorJointLinearY", &velocity.y, 0.05f, -10.0f, 10.0f, "%.2f m/s" );
            changed |= ImGui::DragFloat( "최대 힘###MotorJointMaxForce", &maxForce, 0.25f, 0.0f, 100.0f, "%.2f N" );
            maxForce = std::max( 0.0f, maxForce );
            if( changed ) view_.setMotorJointLinearSettings( velocity, maxForce );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
        if( ImGui::TreeNodeEx( "회전 Velocity Motor###MotorJointAngularSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            float velocity = joint.angularVelocity;
            float maxTorque = joint.maxVelocityTorque;
            bool changed = ImGui::DragFloat( "목표 각속도###MotorJointAngularVelocity", &velocity, 0.05f, -10.0f, 10.0f, "%.2f rad/s" );
            changed |= ImGui::DragFloat( "최대 토크###MotorJointMaxTorque", &maxTorque, 0.25f, 0.0f, 100.0f, "%.2f N*m" );
            maxTorque = std::max( 0.0f, maxTorque );
            if( changed ) view_.setMotorJointAngularSettings( velocity, maxTorque );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
        ImGui::SeparatorText( "현재 상태###MotorJointState" );
        ImGui::Text( "Target linear: (%.2f, %.2f) m/s", joint.linearVelocity.x, joint.linearVelocity.y );
        ImGui::Text( "Target angular: %.2f rad/s", joint.angularVelocity );
        ImGui::Text( "Reaction force: (%.2f, %.2f) N", joint.force.x, joint.force.y );
        ImGui::Text( "Reaction torque: %.2f N*m", joint.torque );
    }

'''

motor_draw = r'''        if( view_.getKind() == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
        {
            const motorJointData joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
            constexpr ImU32 ANCHOR_COLOR = IM_COL32( 100, 235, 220, 255 );
            constexpr ImU32 TARGET_COLOR = IM_COL32( 255, 220, 90, 255 );

            draw.DrawPoint( joint.anchorA, ANCHOR_COLOR, 6.0f );
            draw.DrawPoint( joint.anchorB, TARGET_COLOR, 7.0f );
            if( LengthSquared( joint.linearVelocity ) > 0.0f )
            {
                // 두 anchor를 선으로 묶지 않음: Motor는 두 점의 위치를 일치시키는 제약이 아니기 때문임.
                draw.DrawArrow( joint.anchorB, Normalize( joint.linearVelocity ), TARGET_COLOR, std::min( 1.5f, Length( joint.linearVelocity ) * 0.5f ) );
            }
            return;
        }

'''

patch("sandbox/jointDemoView.cpp", [
    (
        "        if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::weldPair && kind != demoKind::mouseJointPlayground ) return;\n",
        "        if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::weldPair && kind != demoKind::mouseJointPlayground && kind != demoKind::motorJointPlayground ) return;\n",
        "Motor control eligibility",
    ),
    (
        "            else if( kind == demoKind::weldPair )\n            {\n",
        quick + "            else if( kind == demoKind::weldPair )\n            {\n",
        "Motor quick settings",
    ),
    (
        "        if( kind == demoKind::prismaticRail && view_.getWorld().IsValid( view_.getPrismaticJoint() ) )\n        {\n            drawPrismaticInspector();\n        }\n        else if( kind == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n",
        "        if( kind == demoKind::prismaticRail && view_.getWorld().IsValid( view_.getPrismaticJoint() ) )\n        {\n            drawPrismaticInspector();\n        }\n        else if( kind == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )\n        {\n            drawMotorJointInspector();\n        }\n        else if( kind == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n",
        "Motor inspector dispatch",
    ),
    (
        "    void draw( debugDraw& draw ) const override\n    {\n        view_.draw( draw );\n        if( view_.getKind() == demoKind::weldPair",
        "    void draw( debugDraw& draw ) const override\n    {\n        view_.draw( draw );\n" + motor_draw + "        if( view_.getKind() == demoKind::weldPair",
        "Motor debug draw",
    ),
    (
        "private:\n    void drawWeldInspector()\n",
        "private:\n" + inspector + "    void drawWeldInspector()\n",
        "Motor inspector implementation",
    ),
])
