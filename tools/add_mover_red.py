from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"{label} marker not found")
    return text.replace(old, new, 1)


path = Path("sandbox/jointDemoView.cpp")
text = path.read_text(encoding="utf-8")

text = replace_once(
    text,
    '        if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::weldPair && kind != demoKind::mouseJointPlayground && kind != demoKind::motorJointPlayground ) return;\n',
    '        if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::weldPair && kind != demoKind::mouseJointPlayground && kind != demoKind::motorJointPlayground && kind != demoKind::moverJointPlayground ) return;\n',
    "Mover controls gate",
)

motor_quick = '''            else if( kind == demoKind::motorJointPlayground )
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
'''
mover_quick = motor_quick + '''            else if( kind == demoKind::moverJointPlayground )
            {
                if( ImGui::Button( "수평###MoverPresetHorizontal" ) ) view_.applyMoverJointPreset( moverJointDemoPreset::horizontal );
                ImGui::SameLine();
                if( ImGui::Button( "수직###MoverPresetVertical" ) ) view_.applyMoverJointPreset( moverJointDemoPreset::vertical );
                ImGui::SameLine();
                if( ImGui::Button( "대각선###MoverPresetDiagonal" ) ) view_.applyMoverJointPreset( moverJointDemoPreset::diagonal );
                ImGui::SameLine();
                if( ImGui::Button( "축별 힘###MoverPresetAnisotropic" ) ) view_.applyMoverJointPreset( moverJointDemoPreset::anisotropic );
                ImGui::TextWrapped( "Mover는 COM 상대 선속도만 제어합니다. x/y 최대 힘을 따로 제한할 수 있고 회전은 전혀 건드리지 않습니다." );
            }
'''
text = replace_once( text, motor_quick, mover_quick, "Mover quick presets" )

text = replace_once(
    text,
    '''        else if( kind == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
        {
            drawMotorJointInspector();
        }
        else if( kind == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )
''',
    '''        else if( kind == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
        {
            drawMotorJointInspector();
        }
        else if( kind == demoKind::moverJointPlayground && view_.getWorld().IsValid( view_.getMoverJoint() ) )
        {
            drawMoverJointInspector();
        }
        else if( kind == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )
''',
    "Mover inspector dispatch",
)

motor_draw_end = '''            return;
        }

        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )
'''
mover_draw = '''            return;
        }

        if( view_.getKind() == demoKind::moverJointPlayground && view_.getWorld().IsValid( view_.getMoverJoint() ) )
        {
            const moverJointData joint = view_.getWorld().getMoverJointData( view_.getMoverJoint() );
            const transform2 bodyTransform = view_.getWorld().GetBodyTransform( joint.bodyB );
            const vec2 bodyCenter = TransformPoint( bodyTransform, view_.getWorld().GetBodyLocalCenter( joint.bodyB ) );
            constexpr ImU32 BODY_COLOR = IM_COL32( 100, 235, 220, 255 );
            constexpr ImU32 TARGET_COLOR = IM_COL32( 255, 220, 90, 255 );
            constexpr ImU32 FORCE_COLOR = IM_COL32( 230, 120, 255, 255 );

            // Mover는 anchor 위치를 잠그지 않으므로 연결선 없이 COM에서 velocity/force만 시각화함.
            draw.DrawPoint( bodyCenter, BODY_COLOR, 6.0f );
            if( LengthSquared( joint.linearVelocity ) > 0.0f )
            {
                draw.DrawArrow( bodyCenter, Normalize( joint.linearVelocity ), TARGET_COLOR, std::min( 1.5f, Length( joint.linearVelocity ) * 0.5f ) );
            }
            if( LengthSquared( joint.force ) > 0.0f )
            {
                draw.DrawArrow( bodyCenter, Normalize( joint.force ), FORCE_COLOR, std::min( 1.25f, Length( joint.force ) * 0.04f ) );
            }
            return;
        }

        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )
'''
text = replace_once( text, motor_draw_end, mover_draw, "Mover debug draw" )

inspector = '''    void drawMoverJointInspector()
    {
        if( !ImGui::CollapsingHeader( "무버 조인트 인스펙터###MoverJointInspector", ImGuiTreeNodeFlags_DefaultOpen ) ) return;

        moverJointData joint = view_.getWorld().getMoverJointData( view_.getMoverJoint() );
        ImGui::TextWrapped( "Mover는 두 Body의 COM 상대 선속도만 목표로 합니다. 회전 제약이 없고 x/y actuator 힘을 독립적으로 제한합니다." );

        float targetVelocity[2] = { joint.linearVelocity.x, joint.linearVelocity.y };
        if( ImGui::DragFloat2( "목표 속도 (m/s)###MoverTargetVelocity", targetVelocity, 0.05f ) )
        {
            view_.setMoverJointSettings( { targetVelocity[0], targetVelocity[1] }, joint.maxVelocityForce );
            joint.linearVelocity = { targetVelocity[0], targetVelocity[1] };
        }

        float maxForce[2] = { joint.maxVelocityForce.x, joint.maxVelocityForce.y };
        if( ImGui::DragFloat2( "최대 힘 (N)###MoverMaxForce", maxForce, 0.25f, 0.0f, 1000.0f ) )
        {
            maxForce[0] = std::max( maxForce[0], 0.0f );
            maxForce[1] = std::max( maxForce[1], 0.0f );
            view_.setMoverJointSettings( joint.linearVelocity, { maxForce[0], maxForce[1] } );
        }

        ImGui::Text( "Reaction Force: (%.2f, %.2f) N", joint.force.x, joint.force.y );
        ImGui::TextUnformatted( "Rotation: unaffected" );
    }

'''
text = replace_once(
    text,
    '''private:
    void drawMotorJointInspector()
''',
    '''private:
''' + inspector + '''    void drawMotorJointInspector()
''',
    "Mover inspector implementation",
)

path.write_text(text, encoding="utf-8")
