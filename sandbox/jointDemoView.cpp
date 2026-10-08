#include "jointDemoView.h"

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

                const auto joint = view_.getWorld().getPrismaticJointData( view_.getPrismaticJoint() );
                const float axialForce = Dot( joint.force, joint.axis );
                ImGui::TextWrapped( "4단계 Prismatic은 축 위치를 목표로 하는 물리 스프링을 추가합니다. Spring은 위치 오차를 줄이고 Motor는 목표 속도를 만들며 Limit은 허용 범위를 넘는 이동을 막습니다." );
                ImGui::Text( "축 이동: %.3f m", joint.currentTranslation );
                if( joint.enableSpring )
                {
                    ImGui::Text( "스프링 목표: %.2f m / %.2f Hz / 감쇠 %.2f", joint.targetTranslation, joint.hertz, joint.dampingRatio );
                    ImGui::Text( "스프링 힘: %.2f N", joint.springForce );
                }
                else
                {
                    ImGui::TextUnformatted( "스프링: 꺼짐" );
                }
                if( joint.enableLimit )
                {
                    ImGui::Text( "이동 범위: %.2f ~ %.2f m", joint.lowerTranslation, joint.upperTranslation );
                }
                else
                {
                    ImGui::TextUnformatted( "이동 범위: 자유" );
                }
                if( joint.enableMotor )
                {
                    ImGui::Text( "모터 목표 속도: %.2f m/s / 모터 힘: %.2f N", joint.motorSpeed, joint.motorForce );
                }
                else
                {
                    ImGui::TextUnformatted( "모터: 꺼짐" );
                }
                ImGui::Text( "총 축 반력: %.2f N / 수직 오차: %.4f m", axialForce, joint.lateralError );
                ImGui::Text( "상대 각도: %.4f rad / 회전 반력: %.2f N*m", joint.currentAngle, joint.torque );
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
    }

    void draw( debugDraw& draw ) const override { view_.draw( draw ); }

private:
    rigidBodyDemoUi view_;
};

} // namespace

std::unique_ptr<demo> createJointDemoView( demoKind kind )
{
    return std::make_unique<jointDemoView>( kind );
}

} // namespace zonai::sandbox
