#include "jointDemoView.h"

#include <algorithm>

#include <imgui.h>

namespace zonai::sandbox
{
namespace
{

class jointDemoView final : public demo
{
public:
    explicit jointDemoView( demoKind kind ) : view_( kind ) {}

    void step( float timeStep, int subStepCount ) override { view_.step( timeStep, subStepCount ); }

    void handleInput( const demoInput& input ) override { view_.handleInput( input ); }

    void cancelInput() override { view_.cancelInput(); }

    void setCollisionMatrix( const collisionMatrix& matrix ) override { view_.setCollisionMatrix( matrix ); }

    void drawControls() override
    {
        view_.drawControls();

        const demoKind kind = view_.getKind();
        if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::mouseJointPlayground ) return;

        if( ImGui::CollapsingHeader( "조인트 빠른 설정###JointQuickSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            if( kind == demoKind::distancePendulum )
            {
                if( ImGui::Button( "고정###DistancePresetRigid" ) ) view_.applyDistancePreset( distanceDemoPreset::rigid );
                ImGui::SameLine();
                if( ImGui::Button( "스프링###DistancePresetSpring" ) ) view_.applyDistancePreset( distanceDemoPreset::spring );
                ImGui::SameLine();
                if( ImGui::Button( "제한###DistancePresetLimit" ) ) view_.applyDistancePreset( distanceDemoPreset::limit );
                ImGui::SameLine();
                if( ImGui::Button( "모터###DistancePresetMotor" ) ) view_.applyDistancePreset( distanceDemoPreset::motor );
                ImGui::TextWrapped( "대표 상태를 바로 적용한 뒤 위의 세부 설정에서 값을 조절할 수 있습니다." );
            }
            else if( kind == demoKind::revoluteHinge )
            {
                if( ImGui::Button( "자유###RevolutePresetFree" ) ) view_.applyRevolutePreset( revoluteDemoPreset::free );
                ImGui::SameLine();
                if( ImGui::Button( "제한###RevolutePresetLimit" ) ) view_.applyRevolutePreset( revoluteDemoPreset::limit );
                ImGui::SameLine();
                if( ImGui::Button( "모터###RevolutePresetMotor" ) ) view_.applyRevolutePreset( revoluteDemoPreset::motor );
                ImGui::SameLine();
                if( ImGui::Button( "모터+제한###RevolutePresetCombined" ) ) view_.applyRevolutePreset( revoluteDemoPreset::motorLimit );
                ImGui::TextWrapped( "자유 회전, 각도 제한, 모터, 두 기능의 결합을 같은 막대에서 빠르게 비교합니다." );
            }
            else if( kind == demoKind::wheelSuspension )
            {
                if( ImGui::Button( "스프링###WheelPresetSpring" ) ) view_.applyWheelPreset( wheelDemoPreset::spring );
                ImGui::SameLine();
                if( ImGui::Button( "제한###WheelPresetLimit" ) ) view_.applyWheelPreset( wheelDemoPreset::limit );
                ImGui::SameLine();
                if( ImGui::Button( "모터###WheelPresetMotor" ) ) view_.applyWheelPreset( wheelDemoPreset::motor );
                ImGui::SameLine();
                if( ImGui::Button( "전체###WheelPresetCombined" ) ) view_.applyWheelPreset( wheelDemoPreset::combined );
                ImGui::TextWrapped( "서스펜션 스프링, 이동 제한, 회전 모터와 결합 상태를 빠르게 전환합니다." );
            }
            else if( kind == demoKind::prismaticRail )
            {
                if( ImGui::Button( "자유###PrismaticPresetFree" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::free );
                ImGui::SameLine();
                if( ImGui::Button( "제한###PrismaticPresetLimited" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::limited );
                ImGui::SameLine();
                if( ImGui::Button( "위치 고정###PrismaticPresetLocked" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::locked );
                if( ImGui::Button( "모터 +###PrismaticPresetMotorForward" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::motorForward );
                ImGui::SameLine();
                if( ImGui::Button( "모터 -###PrismaticPresetMotorReverse" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::motorReverse );
                ImGui::SameLine();
                if( ImGui::Button( "제동###PrismaticPresetBrake" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::brake );
                ImGui::SameLine();
                if( ImGui::Button( "모터+제한###PrismaticPresetMotorLimit" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::motorLimit );
                if( ImGui::Button( "스프링###PrismaticPresetSpring" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::spring );
                ImGui::SameLine();
                if( ImGui::Button( "스프링+제한###PrismaticPresetSpringLimit" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::springLimit );
                ImGui::SameLine();
                if( ImGui::Button( "전체###PrismaticPresetCombined" ) ) view_.applyPrismaticPreset( prismaticDemoPreset::combined );
                ImGui::TextWrapped( "프리셋은 대표 상태를 빠르게 만드는 용도입니다. 아래 인스펙터에서 각 제약을 직접 켜고 수치를 실시간으로 조절할 수 있습니다." );
            }
            else
            {
                ImGui::TextWrapped( "질량이 다른 세 상자를 같은 Mouse Joint 설정으로 드래그해 추종 차이를 비교합니다." );
                if( view_.getWorld().IsValid( view_.getMouseJoint() ) )
                {
                    const auto joint = view_.getWorld().getMouseJointData( view_.getMouseJoint() );
                    ImGui::Text( "목표 오차: %.3f m / 힘: %.2f N", Length( joint.target - joint.anchorB ), Length( joint.force ) );
                }
                else
                {
                    ImGui::TextUnformatted( "상자를 왼쪽 드래그하면 Mouse Joint가 생성됩니다." );
                }
            }
        }

        if( kind == demoKind::prismaticRail && view_.getWorld().IsValid( view_.getPrismaticJoint() ) )
        {
            drawPrismaticInspector();
        }
    }

    void draw( debugDraw& draw ) const override
    {
        view_.draw( draw );
        if( view_.getKind() != demoKind::prismaticRail || !view_.getWorld().IsValid( view_.getPrismaticJoint() ) ) return;

        const prismaticJointData joint = view_.getWorld().getPrismaticJointData( view_.getPrismaticJoint() );
        const vec2 tick = 0.12f * Cross( 1.0f, joint.axis );
        constexpr ImU32 AXIS_COLOR = IM_COL32( 100, 235, 220, 255 );
        constexpr ImU32 CURRENT_COLOR = IM_COL32( 255, 220, 90, 255 );
        constexpr ImU32 TARGET_COLOR = IM_COL32( 230, 120, 255, 255 );
        constexpr ImU32 LOWER_COLOR = IM_COL32( 90, 220, 110, 255 );
        constexpr ImU32 UPPER_COLOR = IM_COL32( 245, 100, 100, 255 );

        draw.DrawArrow( joint.anchorA, joint.axis, AXIS_COLOR, 0.65f );
        draw.DrawPoint( joint.anchorA, AXIS_COLOR, 6.0f );
        draw.DrawPoint( joint.anchorB, CURRENT_COLOR, 6.0f );

        const vec2 current = joint.anchorA + joint.currentTranslation * joint.axis;
        draw.DrawSegment( { current - tick, current + tick }, CURRENT_COLOR, 2.0f );

        if( joint.enableSpring )
        {
            const vec2 target = joint.anchorA + joint.targetTranslation * joint.axis;
            draw.DrawSegment( { target - 1.35f * tick, target + 1.35f * tick }, TARGET_COLOR, 2.0f );
            draw.DrawPoint( target, TARGET_COLOR, 5.0f );
            draw.DrawSegment( { current, target }, TARGET_COLOR, 1.0f );
        }

        if( joint.enableLimit )
        {
            const vec2 lower = joint.anchorA + joint.lowerTranslation * joint.axis;
            const vec2 upper = joint.anchorA + joint.upperTranslation * joint.axis;
            draw.DrawSegment( { lower, upper }, IM_COL32( 180, 185, 200, 255 ) );
            draw.DrawSegment( { lower - tick, lower + tick }, LOWER_COLOR, 2.0f );
            draw.DrawSegment( { upper - tick, upper + tick }, UPPER_COLOR, 2.0f );
        }
    }

private:
    void drawPrismaticInspector()
    {
        if( !ImGui::CollapsingHeader( "프리즈매틱 조인트 인스펙터###PrismaticJointInspector", ImGuiTreeNodeFlags_DefaultOpen ) ) return;

        prismaticJointData joint = view_.getWorld().getPrismaticJointData( view_.getPrismaticJoint() );

        ImGui::TextWrapped( "Spring은 목표 위치, Motor는 목표 속도, Limit은 허용 범위를 제어합니다. 세 제약은 같은 축을 사용하지만 서로 독립적으로 켜고 조절할 수 있습니다." );

        if( ImGui::TreeNodeEx( "스프링###PrismaticSpringSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            bool enableSpring = joint.enableSpring;
            float hertz = joint.hertz;
            float dampingRatio = joint.dampingRatio;
            float targetTranslation = joint.targetTranslation;
            bool changed = ImGui::Checkbox( "사용###PrismaticSpringEnabled", &enableSpring );
            changed |= ImGui::DragFloat( "주파수###PrismaticHertz", &hertz, 0.1f, 0.0f, 30.0f, "%.2f Hz" );
            changed |= ImGui::DragFloat( "감쇠비###PrismaticDamping", &dampingRatio, 0.02f, 0.0f, 2.0f, "%.2f" );
            changed |= ImGui::DragFloat( "목표 이동###PrismaticTarget", &targetTranslation, 0.05f, -10.0f, 10.0f, "%.2f m" );
            hertz = std::clamp( hertz, 0.0f, 30.0f );
            dampingRatio = std::clamp( dampingRatio, 0.0f, 2.0f );
            targetTranslation = std::clamp( targetTranslation, -10.0f, 10.0f );
            if( changed ) view_.setPrismaticSpringSettings( enableSpring, hertz, dampingRatio, targetTranslation );
            if( ImGui::Button( "현재 위치를 목표로###PrismaticTargetCurrent" ) ) view_.setPrismaticSpringTargetToCurrent();
            ImGui::TreePop();
        }

        joint = view_.getWorld().getPrismaticJointData( view_.getPrismaticJoint() );
        if( ImGui::TreeNodeEx( "이동 제한###PrismaticLimitSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            bool enableLimit = joint.enableLimit;
            float lowerTranslation = joint.lowerTranslation;
            float upperTranslation = joint.upperTranslation;
            bool changed = ImGui::Checkbox( "사용###PrismaticLimitEnabled", &enableLimit );
            if( ImGui::DragFloat( "하한###PrismaticLower", &lowerTranslation, 0.05f, -10.0f, 10.0f, "%.2f m" ) )
            {
                lowerTranslation = std::min( lowerTranslation, upperTranslation );
                changed = true;
            }
            if( ImGui::DragFloat( "상한###PrismaticUpper", &upperTranslation, 0.05f, -10.0f, 10.0f, "%.2f m" ) )
            {
                upperTranslation = std::max( upperTranslation, lowerTranslation );
                changed = true;
            }
            lowerTranslation = std::clamp( lowerTranslation, -10.0f, 10.0f );
            upperTranslation = std::clamp( upperTranslation, -10.0f, 10.0f );
            if( changed ) view_.setPrismaticLimitSettings( enableLimit, lowerTranslation, upperTranslation );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getPrismaticJointData( view_.getPrismaticJoint() );
        if( ImGui::TreeNodeEx( "모터###PrismaticMotorSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            bool enableMotor = joint.enableMotor;
            float motorSpeed = joint.motorSpeed;
            float maxMotorForce = joint.maxMotorForce;
            bool changed = ImGui::Checkbox( "사용###PrismaticMotorEnabled", &enableMotor );
            changed |= ImGui::DragFloat( "목표 속도###PrismaticMotorSpeed", &motorSpeed, 0.1f, -20.0f, 20.0f, "%.2f m/s" );
            changed |= ImGui::DragFloat( "최대 힘###PrismaticMotorForce", &maxMotorForce, 1.0f, 0.0f, 500.0f, "%.1f N" );
            motorSpeed = std::clamp( motorSpeed, -20.0f, 20.0f );
            maxMotorForce = std::clamp( maxMotorForce, 0.0f, 500.0f );
            if( changed ) view_.setPrismaticMotorSettings( enableMotor, motorSpeed, maxMotorForce );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getPrismaticJointData( view_.getPrismaticJoint() );
        const float targetError = joint.currentTranslation - joint.targetTranslation;
        const float axialForce = Dot( joint.force, joint.axis );
        ImGui::SeparatorText( "현재 상태###PrismaticState" );
        ImGui::Text( "이동: %.3f m / 축 상대속도: %.3f m/s", joint.currentTranslation, view_.getPrismaticCurrentSpeed() );
        ImGui::Text( "Spring 목표 오차: %.3f m / 힘: %.2f N", targetError, joint.springForce );
        ImGui::Text( "Motor 목표: %.2f m/s / 힘: %.2f N", joint.motorSpeed, joint.motorForce );
        if( joint.enableLimit ) ImGui::Text( "Limit: %.2f ~ %.2f m", joint.lowerTranslation, joint.upperTranslation );
        else ImGui::TextUnformatted( "Limit: 꺼짐" );
        ImGui::Text( "전체 축 반력: %.2f N / 수직 오차: %.4f m", axialForce, joint.lateralError );
        ImGui::Text( "상대 각도: %.4f rad / 회전 반력: %.2f N*m", joint.currentAngle, joint.torque );
    }

    rigidBodyDemoUi view_;
};

} // namespace

std::unique_ptr<demo> createJointDemoView( demoKind kind )
{
    return std::make_unique<jointDemoView>( kind );
}

} // namespace zonai::sandbox
