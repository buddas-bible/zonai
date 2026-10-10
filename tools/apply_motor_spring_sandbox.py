from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, got {count}")
    return text.replace(old, new, 1)


def replace_between(text: str, start: str, end: str, replacement: str, label: str) -> str:
    first = text.find(start)
    if first < 0:
        raise SystemExit(f"{label}: start marker not found")
    last = text.find(end, first)
    if last < 0:
        raise SystemExit(f"{label}: end marker not found")
    return text[:first] + replacement + text[last:]


joint_demo_path = Path("sandbox/rigidBodyJointDemo.cpp")
joint_demo = joint_demo_path.read_text(encoding="utf-8")
settings_marker = """void rigidBodyDemo::setMotorJointAngularSettings( float angularVelocity, float maxVelocityTorque )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointAngularVelocity( motorJoint_, angularVelocity, maxVelocityTorque );
}
"""
settings_replacement = settings_marker + """

void rigidBodyDemo::setMotorJointLinearSpringSettings( float hertz, float dampingRatio, float maxSpringForce )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointLinearSpring( motorJoint_, hertz, dampingRatio, maxSpringForce );
}

void rigidBodyDemo::setMotorJointAngularSpringSettings( float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointAngularSpring( motorJoint_, referenceAngle, hertz, dampingRatio, maxSpringTorque );
}
"""
joint_demo = replace_once(joint_demo, settings_marker, settings_replacement, "Motor Sandbox settings")

preset_start = "void rigidBodyDemo::applyMotorJointPreset( motorJointDemoPreset preset )\n"
preset_end = "#pragma endregion Presets\n"
preset = """void rigidBodyDemo::applyMotorJointPreset( motorJointDemoPreset preset )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;

    constexpr vec2 linearVelocity{ 2.0f, 0.5f };
    constexpr float maxVelocityForce = 20.0f;
    constexpr float angularVelocity = 2.0f;
    constexpr float maxVelocityTorque = 10.0f;
    constexpr float referenceAngle = 0.75f;
    constexpr float springHertz = 3.0f;
    constexpr float springDampingRatio = 0.7f;
    constexpr float maxSpringForce = 20.0f;
    constexpr float maxSpringTorque = 10.0f;

    vec2 targetLinearVelocity{};
    float linearForce = 0.0f;
    float targetAngularVelocity = 0.0f;
    float angularTorque = 0.0f;
    float linearSpringHertz = 0.0f;
    float linearSpringForce = 0.0f;
    float angularSpringTarget = 0.0f;
    float angularSpringHertz = 0.0f;
    float angularSpringTorque = 0.0f;

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

    case motorJointDemoPreset::linearSpring:
        linearSpringHertz = springHertz;
        linearSpringForce = maxSpringForce;
        break;

    case motorJointDemoPreset::angularSpring:
        angularSpringTarget = referenceAngle;
        angularSpringHertz = springHertz;
        angularSpringTorque = maxSpringTorque;
        break;

    case motorJointDemoPreset::springBoth:
        linearSpringHertz = springHertz;
        linearSpringForce = maxSpringForce;
        angularSpringTarget = referenceAngle;
        angularSpringHertz = springHertz;
        angularSpringTorque = maxSpringTorque;
        break;

    case motorJointDemoPreset::velocityAndSpring:
        targetLinearVelocity = linearVelocity;
        linearForce = maxVelocityForce;
        targetAngularVelocity = angularVelocity;
        angularTorque = maxVelocityTorque;
        linearSpringHertz = springHertz;
        linearSpringForce = maxSpringForce;
        angularSpringTarget = referenceAngle;
        angularSpringHertz = springHertz;
        angularSpringTorque = maxSpringTorque;
        break;
    }

    // 모든 actuator를 항상 명시적으로 설정해 이전 preset의 velocity / spring 상태가 남지 않게 함.
    world_.setMotorJointLinearVelocity( motorJoint_, targetLinearVelocity, linearForce );
    world_.setMotorJointAngularVelocity( motorJoint_, targetAngularVelocity, angularTorque );
    world_.setMotorJointLinearSpring( motorJoint_, linearSpringHertz, springDampingRatio, linearSpringForce );
    world_.setMotorJointAngularSpring( motorJoint_, angularSpringTarget, angularSpringHertz, springDampingRatio, angularSpringTorque );
}

"""
joint_demo = replace_between(joint_demo, preset_start, preset_end, preset + preset_end, "Motor Sandbox preset")
joint_demo_path.write_text(joint_demo, encoding="utf-8")


demo_path = Path("sandbox/demo.cpp")
demo = demo_path.read_text(encoding="utf-8")
demo = replace_once(
    demo,
    "Motor Joint는 위치를 고정하지 않고 두 Body의 상대 선속도와 상대 각속도를 목표값으로 만듭니다. 제동·선형·회전·결합 프리셋과 힘/토크 한도를 비교합니다.",
    "Motor Joint의 상대속도 actuator와 목표 transform spring을 비교합니다. 제동·선형·회전·스프링·결합 프리셋으로 속도 목표와 위치/각도 복원의 차이를 관찰합니다.",
    "Motor demo description",
)
demo_path.write_text(demo, encoding="utf-8")


view_path = Path("sandbox/jointDemoView.cpp")
view = view_path.read_text(encoding="utf-8")
quick_start = "            else if( kind == demoKind::motorJointPlayground )\n"
quick_end = "            else if( kind == demoKind::weldPair )\n"
quick = """            else if( kind == demoKind::motorJointPlayground )
            {
                if( ImGui::Button( "제동###MotorJointPresetBrake" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::brake );
                ImGui::SameLine();
                if( ImGui::Button( "선형 속도###MotorJointPresetLinear" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::linear );
                ImGui::SameLine();
                if( ImGui::Button( "회전 속도###MotorJointPresetAngular" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::angular );
                ImGui::SameLine();
                if( ImGui::Button( "속도 둘 다###MotorJointPresetCombined" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::combined );
                if( ImGui::Button( "선형 Spring###MotorJointPresetLinearSpring" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::linearSpring );
                ImGui::SameLine();
                if( ImGui::Button( "회전 Spring###MotorJointPresetAngularSpring" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::angularSpring );
                ImGui::SameLine();
                if( ImGui::Button( "Spring 둘 다###MotorJointPresetSpringBoth" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::springBoth );
                ImGui::SameLine();
                if( ImGui::Button( "Velocity + Spring###MotorJointPresetVelocitySpring" ) ) view_.applyMotorJointPreset( motorJointDemoPreset::velocityAndSpring );
                ImGui::TextWrapped( "Velocity Motor는 상대속도를 목표로 하고 transform spring은 두 anchor와 기준 상대각도를 복원합니다. 둘은 독립 actuator라 동시에 켤 수 있습니다." );
            }
"""
view = replace_between(view, quick_start, quick_end, quick + quick_end, "Motor quick settings")

draw_start = "        if( view_.getKind() == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )\n"
draw_end = "        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n"
draw = """        if( view_.getKind() == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
        {
            const motorJointData joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
            constexpr ImU32 ANCHOR_COLOR = IM_COL32( 100, 235, 220, 255 );
            constexpr ImU32 TARGET_COLOR = IM_COL32( 255, 220, 90, 255 );
            constexpr ImU32 SPRING_COLOR = IM_COL32( 230, 120, 255, 255 );

            draw.DrawPoint( joint.anchorA, ANCHOR_COLOR, 6.0f );
            draw.DrawPoint( joint.anchorB, TARGET_COLOR, 7.0f );
            if( LengthSquared( joint.linearVelocity ) > 0.0f )
            {
                // Velocity Motor만 켜진 상태에서는 두 anchor를 선으로 묶지 않아 위치 제약으로 보이지 않게 함.
                draw.DrawArrow( joint.anchorB, Normalize( joint.linearVelocity ), TARGET_COLOR, std::min( 1.5f, Length( joint.linearVelocity ) * 0.5f ) );
            }
            if( joint.linearHertz > 0.0f && joint.maxSpringForce > 0.0f )
            {
                // Linear transform spring이 실제로 두 anchor의 위치 오차를 복원할 때만 목표 연결을 그림.
                draw.DrawSegment( { joint.anchorA, joint.anchorB }, SPRING_COLOR, 2.0f );
            }
            if( joint.angularHertz > 0.0f && joint.maxSpringTorque > 0.0f )
            {
                const rot2 targetRotation = view_.getWorld().GetBodyTransform( joint.bodyA ).rotation * rot2::FromRadians( joint.referenceAngle );
                draw.DrawArrow( joint.anchorA, Rotate( targetRotation, { 1.0f, 0.0f } ), SPRING_COLOR, 0.65f );
            }
            return;
        }

"""
view = replace_between(view, draw_start, draw_end, draw + draw_end, "Motor draw")

inspector_start = "    void drawMotorJointInspector()\n"
inspector_end = "    void drawWeldInspector()\n"
inspector = """    void drawMotorJointInspector()
    {
        if( !ImGui::CollapsingHeader( "모터 조인트 인스펙터###MotorJointInspector", ImGuiTreeNodeFlags_DefaultOpen ) ) return;

        motorJointData joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
        ImGui::TextWrapped( "Velocity 채널은 상대속도를 직접 목표로 하고, Transform Spring은 anchor 위치와 기준 상대각도 오차를 실제 spring-damper 힘으로 복원합니다." );

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
        if( ImGui::TreeNodeEx( "선형 Transform Spring###MotorJointLinearSpring", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            float hertz = joint.linearHertz;
            float dampingRatio = joint.linearDampingRatio;
            float maxForce = joint.maxSpringForce;
            bool changed = ImGui::DragFloat( "주파수###MotorJointLinearSpringHertz", &hertz, 0.1f, 0.0f, 30.0f, "%.2f Hz" );
            changed |= ImGui::DragFloat( "감쇠비###MotorJointLinearSpringDamping", &dampingRatio, 0.02f, 0.0f, 2.0f, "%.2f" );
            changed |= ImGui::DragFloat( "최대 Spring 힘###MotorJointMaxSpringForce", &maxForce, 0.25f, 0.0f, 100.0f, "%.2f N" );
            hertz = std::max( 0.0f, hertz );
            dampingRatio = std::max( 0.0f, dampingRatio );
            maxForce = std::max( 0.0f, maxForce );
            if( changed ) view_.setMotorJointLinearSpringSettings( hertz, dampingRatio, maxForce );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
        if( ImGui::TreeNodeEx( "회전 Transform Spring###MotorJointAngularSpring", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            float referenceAngle = joint.referenceAngle;
            float hertz = joint.angularHertz;
            float dampingRatio = joint.angularDampingRatio;
            float maxTorque = joint.maxSpringTorque;
            bool changed = ImGui::DragFloat( "기준 상대각도###MotorJointReferenceAngle", &referenceAngle, 0.02f, -3.14f, 3.14f, "%.2f rad" );
            changed |= ImGui::DragFloat( "주파수###MotorJointAngularSpringHertz", &hertz, 0.1f, 0.0f, 30.0f, "%.2f Hz" );
            changed |= ImGui::DragFloat( "감쇠비###MotorJointAngularSpringDamping", &dampingRatio, 0.02f, 0.0f, 2.0f, "%.2f" );
            changed |= ImGui::DragFloat( "최대 Spring 토크###MotorJointMaxSpringTorque", &maxTorque, 0.25f, 0.0f, 100.0f, "%.2f N*m" );
            hertz = std::max( 0.0f, hertz );
            dampingRatio = std::max( 0.0f, dampingRatio );
            maxTorque = std::max( 0.0f, maxTorque );
            if( changed ) view_.setMotorJointAngularSpringSettings( referenceAngle, hertz, dampingRatio, maxTorque );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getMotorJointData( view_.getMotorJoint() );
        ImGui::SeparatorText( "현재 상태###MotorJointState" );
        ImGui::Text( "Target linear: (%.2f, %.2f) m/s", joint.linearVelocity.x, joint.linearVelocity.y );
        ImGui::Text( "Target angular: %.2f rad/s", joint.angularVelocity );
        ImGui::Text( "Linear spring: %.2f Hz / damping %.2f / %.2f N", joint.linearHertz, joint.linearDampingRatio, joint.maxSpringForce );
        ImGui::Text( "Angular spring: %.2f rad / %.2f Hz / damping %.2f / %.2f N*m", joint.referenceAngle, joint.angularHertz, joint.angularDampingRatio, joint.maxSpringTorque );
        ImGui::Text( "Reaction force: (%.2f, %.2f) N", joint.force.x, joint.force.y );
        ImGui::Text( "Reaction torque: %.2f N*m", joint.torque );
    }

"""
view = replace_between(view, inspector_start, inspector_end, inspector + inspector_end, "Motor inspector")
view_path.write_text(view, encoding="utf-8")
