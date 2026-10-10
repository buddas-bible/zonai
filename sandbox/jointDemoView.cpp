#include "jointDemoView.h"

#include <algorithm>

#include <imgui.h>

#include "debug/debugDraw.h"

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
        if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::weldPair && kind != demoKind::mouseJointPlayground && kind != demoKind::motorJointPlayground && kind != demoKind::moverJointPlayground && kind != demoKind::pogoJointPlayground && kind != demoKind::filterJointPlayground ) return;

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
            else if( kind == demoKind::motorJointPlayground )
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
            else if( kind == demoKind::moverJointPlayground )
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
            else if( kind == demoKind::filterJointPlayground )
            {
                bool enabled = view_.getWorld().IsValid( view_.getFilterJoint() );
                if( ImGui::Checkbox( "Filter Enabled###FilterJointEnabled", &enabled ) ) view_.setFilterJointEnabled( enabled );
                ImGui::TextWrapped( "Filter는 solver 힘을 만들지 않고 이 두 Body 사이 Contact만 차단합니다. Off는 Joint를 파괴해 같은 pair의 collision을 다시 허용합니다." );
                ImGui::Text( "Contacts: %zu", view_.getWorld().GetContactCount() );
            }
            else if( kind == demoKind::pogoJointPlayground )
            {
                if( ImGui::Button( "Soft###PogoPresetSoft" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::soft );
                ImGui::SameLine();
                if( ImGui::Button( "Stiff###PogoPresetStiff" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::stiff );
                ImGui::SameLine();
                if( ImGui::Button( "압축 전용###PogoPresetCompressionOnly" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::compressionOnly );
                ImGui::SameLine();
                if( ImGui::Button( "비대칭###PogoPresetAsymmetric" ) ) view_.applyPogoJointPreset( pogoJointDemoPreset::asymmetric );
                ImGui::TextWrapped( "Pogo는 길이를 재는 축과 실제 반력 normal을 분리합니다. 인장/압축 힘 한도도 독립적으로 조절할 수 있습니다." );
            }
            else if( kind == demoKind::weldPair )
            {
                if( ImGui::Button( "고정###WeldPresetRigid" ) ) view_.applyWeldPreset( weldDemoPreset::rigid );
                ImGui::SameLine();
                if( ImGui::Button( "선형 Soft###WeldPresetSoftLinear" ) ) view_.applyWeldPreset( weldDemoPreset::softLinear );
                ImGui::SameLine();
                if( ImGui::Button( "회전 Soft###WeldPresetSoftAngular" ) ) view_.applyWeldPreset( weldDemoPreset::softAngular );
                ImGui::SameLine();
                if( ImGui::Button( "둘 다 Soft###WeldPresetSoftBoth" ) ) view_.applyWeldPreset( weldDemoPreset::softBoth );
                ImGui::TextWrapped( "0 Hz는 기존 hard Weld입니다. 선형과 회전 softness를 독립적으로 바꿔 두 anchor와 상대 각도의 복원 차이를 비교합니다." );
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
        else if( kind == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
        {
            drawMotorJointInspector();
        }
        else if( kind == demoKind::moverJointPlayground && view_.getWorld().IsValid( view_.getMoverJoint() ) )
        {
            drawMoverJointInspector();
        }
        else if( kind == demoKind::pogoJointPlayground && view_.getWorld().IsValid( view_.getPogoJoint() ) )
        {
            drawPogoJointInspector();
        }
        else if( kind == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )
        {
            drawWeldInspector();
        }
    }

    void draw( debugDraw& draw ) const override
    {
        view_.draw( draw );
        if( view_.getKind() == demoKind::ragdoll )
        {
            drawRagdollJointLimits( draw, view_ );
            return;
        }

        if( view_.getKind() == demoKind::filterJointPlayground )
        {
            if( view_.getWorld().IsValid( view_.getFilterJoint() ) )
            {
                const filterJointData joint = view_.getWorld().getFilterJointData( view_.getFilterJoint() );
                const vec2 centerA = TransformPoint( view_.getWorld().GetBodyTransform( joint.bodyA ), view_.getWorld().GetBodyLocalCenter( joint.bodyA ) );
                const vec2 centerB = TransformPoint( view_.getWorld().GetBodyTransform( joint.bodyB ), view_.getWorld().GetBodyLocalCenter( joint.bodyB ) );
                constexpr ImU32 BODY_COLOR = IM_COL32( 100, 235, 220, 255 );
                constexpr ImU32 FILTER_COLOR = IM_COL32( 255, 190, 70, 255 );
                draw.DrawPoint( centerA, BODY_COLOR, 6.0f );
                draw.DrawPoint( centerB, BODY_COLOR, 6.0f );
                draw.DrawSegment( { centerA, centerB }, FILTER_COLOR, 2.0f );
            }
            return;
        }

        if( view_.getKind() == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
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

        if( view_.getKind() == demoKind::pogoJointPlayground && view_.getWorld().IsValid( view_.getPogoJoint() ) )
        {
            const pogoJointData joint = view_.getWorld().getPogoJointData( view_.getPogoJoint() );
            constexpr ImU32 ANCHOR_A_COLOR = IM_COL32( 100, 235, 220, 255 );
            constexpr ImU32 ANCHOR_B_COLOR = IM_COL32( 255, 220, 90, 255 );
            constexpr ImU32 AXIS_COLOR = IM_COL32( 110, 180, 255, 255 );
            constexpr ImU32 NORMAL_COLOR = IM_COL32( 90, 220, 110, 255 );
            constexpr ImU32 FORCE_COLOR = IM_COL32( 230, 120, 255, 255 );

            draw.DrawPoint( joint.anchorA, ANCHOR_A_COLOR, 7.0f );
            draw.DrawPoint( joint.anchorB, ANCHOR_B_COLOR, 7.0f );
            // Pogo는 이 두 점 사이의 길이를 실제로 복원하므로 Mover와 달리 연결선을 표시함.
            draw.DrawSegment( { joint.anchorA, joint.anchorB }, IM_COL32( 180, 185, 200, 255 ), 2.0f );
            draw.DrawArrow( joint.anchorB, joint.pogoAxis, AXIS_COLOR, 0.7f );
            draw.DrawArrow( joint.anchorA, joint.normal, NORMAL_COLOR, 0.7f );
            if( LengthSquared( joint.force ) > 0.0f )
            {
                draw.DrawArrow( joint.anchorB, Normalize( joint.force ), FORCE_COLOR, std::min( 1.25f, Length( joint.force ) * 0.01f ) );
            }
            return;
        }

        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )
        {
            const weldJointData joint = view_.getWorld().getWeldJointData( view_.getWeldJoint() );
            constexpr ImU32 ANCHOR_A_COLOR = IM_COL32( 100, 235, 220, 255 );
            constexpr ImU32 ANCHOR_B_COLOR = IM_COL32( 255, 220, 90, 255 );
            constexpr ImU32 ERROR_COLOR = IM_COL32( 245, 100, 100, 255 );
            constexpr ImU32 FRAME_A_COLOR = IM_COL32( 110, 180, 255, 255 );
            constexpr ImU32 FRAME_B_COLOR = IM_COL32( 230, 120, 255, 255 );

            draw.DrawPoint( joint.anchorA, ANCHOR_A_COLOR, 7.0f );
            draw.DrawPoint( joint.anchorB, ANCHOR_B_COLOR, 7.0f );
            draw.DrawSegment( { joint.anchorA, joint.anchorB }, ERROR_COLOR, 2.0f );

            const vec2 axisA = Rotate( view_.getWorld().GetBodyTransform( joint.bodyA ).rotation, { 1.0f, 0.0f } );
            const vec2 axisB = Rotate( view_.getWorld().GetBodyTransform( joint.bodyB ).rotation, { 1.0f, 0.0f } );
            draw.DrawArrow( joint.anchorA, axisA, FRAME_A_COLOR, 0.55f );
            draw.DrawArrow( joint.anchorB, axisB, FRAME_B_COLOR, 0.55f );
            return;
        }

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
    void drawPogoJointInspector()
    {
        if( !ImGui::CollapsingHeader( "포고 조인트 인스펙터###PogoJointInspector", ImGuiTreeNodeFlags_DefaultOpen ) ) return;

        pogoJointData joint = view_.getWorld().getPogoJointData( view_.getPogoJoint() );
        ImGui::TextWrapped( "길이 오차는 pogo axis로 측정하고 실제 impulse는 contact normal 방향으로 작용합니다. 위치 복원 속도는 integration 뒤 relaxation에서 제거됩니다." );

        float restLength = joint.restLength;
        float hertz = joint.hertz;
        float dampingRatio = joint.dampingRatio;
        float maxTensionForce = joint.maxTensionForce;
        float maxCompressionForce = joint.maxCompressionForce;
        bool changed = ImGui::DragFloat( "Rest Length###PogoRestLength", &restLength, 0.02f, 0.0f, 5.0f, "%.2f m" );
        changed |= ImGui::DragFloat( "주파수###PogoHertz", &hertz, 0.1f, 0.0f, 30.0f, "%.2f Hz" );
        changed |= ImGui::DragFloat( "감쇠비###PogoDamping", &dampingRatio, 0.02f, 0.0f, 2.0f, "%.2f" );
        changed |= ImGui::DragFloat( "최대 인장 힘###PogoTensionForce", &maxTensionForce, 1.0f, 0.0f, 2000.0f, "%.1f N" );
        changed |= ImGui::DragFloat( "최대 압축 힘###PogoCompressionForce", &maxCompressionForce, 1.0f, 0.0f, 2000.0f, "%.1f N" );
        restLength = std::max( 0.0f, restLength );
        hertz = std::max( 0.0f, hertz );
        dampingRatio = std::max( 0.0f, dampingRatio );
        maxTensionForce = std::max( 0.0f, maxTensionForce );
        maxCompressionForce = std::max( 0.0f, maxCompressionForce );
        if( changed ) view_.setPogoJointSettings( restLength, hertz, dampingRatio, maxTensionForce, maxCompressionForce );

        joint = view_.getWorld().getPogoJointData( view_.getPogoJoint() );
        ImGui::Text( "Current Length: %.3f m", joint.length );
        ImGui::Text( "Spring Velocity: %.3f m/s", joint.velocity );
        ImGui::Text( "Reaction Force: (%.2f, %.2f) N", joint.force.x, joint.force.y );
    }

    void drawMoverJointInspector()
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

    void drawMotorJointInspector()
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

    void drawWeldInspector()
    {
        if( !ImGui::CollapsingHeader( "웰드 조인트 인스펙터###WeldJointInspector", ImGuiTreeNodeFlags_DefaultOpen ) ) return;

        weldJointData joint = view_.getWorld().getWeldJointData( view_.getWeldJoint() );
        ImGui::TextWrapped( "선형 2x2 제약과 각도 scalar 제약은 같은 Weld를 구성하지만 softness는 독립적입니다. 각 주파수가 0 Hz면 해당 채널은 hard constraint로 동작합니다." );

        if( ImGui::TreeNodeEx( "선형 Softness###WeldLinearSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            float hertz = joint.linearHertz;
            float dampingRatio = joint.linearDampingRatio;
            bool changed = ImGui::DragFloat( "주파수###WeldLinearHertz", &hertz, 0.1f, 0.0f, 30.0f, "%.2f Hz" );
            changed |= ImGui::DragFloat( "감쇠비###WeldLinearDamping", &dampingRatio, 0.02f, 0.0f, 2.0f, "%.2f" );
            hertz = std::clamp( hertz, 0.0f, 30.0f );
            dampingRatio = std::clamp( dampingRatio, 0.0f, 2.0f );
            if( changed ) view_.setWeldLinearSettings( hertz, dampingRatio );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getWeldJointData( view_.getWeldJoint() );
        if( ImGui::TreeNodeEx( "회전 Softness###WeldAngularSettings", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            float hertz = joint.angularHertz;
            float dampingRatio = joint.angularDampingRatio;
            bool changed = ImGui::DragFloat( "주파수###WeldAngularHertz", &hertz, 0.1f, 0.0f, 30.0f, "%.2f Hz" );
            changed |= ImGui::DragFloat( "감쇠비###WeldAngularDamping", &dampingRatio, 0.02f, 0.0f, 2.0f, "%.2f" );
            hertz = std::clamp( hertz, 0.0f, 30.0f );
            dampingRatio = std::clamp( dampingRatio, 0.0f, 2.0f );
            if( changed ) view_.setWeldAngularSettings( hertz, dampingRatio );
            ImGui::TreePop();
        }

        joint = view_.getWorld().getWeldJointData( view_.getWeldJoint() );
        const vec2 anchorError = joint.anchorB - joint.anchorA;
        ImGui::SeparatorText( "현재 상태###WeldState" );
        ImGui::Text( "Linear: %.2f Hz / damping %.2f", joint.linearHertz, joint.linearDampingRatio );
        ImGui::Text( "Angular: %.2f Hz / damping %.2f", joint.angularHertz, joint.angularDampingRatio );
        ImGui::Text( "Anchor A: (%.3f, %.3f)", joint.anchorA.x, joint.anchorA.y );
        ImGui::Text( "Anchor B: (%.3f, %.3f)", joint.anchorB.x, joint.anchorB.y );
        ImGui::Text( "Anchor 오차: (%.4f, %.4f) / %.4f m", anchorError.x, anchorError.y, Length( anchorError ) );
        ImGui::Text( "기준 상대각: %.4f rad", joint.referenceAngle );
        ImGui::Text( "상대 각도 오차: %.4f rad", joint.currentAngle );
        ImGui::Text( "반력: (%.2f, %.2f) N / 크기 %.2f N", joint.force.x, joint.force.y, Length( joint.force ) );
        ImGui::Text( "반토크: %.2f N*m", joint.torque );
    }

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

void drawRagdollJointLimits( debugDraw& draw, const rigidBodyDemo& model )
{
    constexpr ImU32 anchorColor = IM_COL32( 100, 235, 220, 255 );
    constexpr ImU32 bodyColor = IM_COL32( 230, 170, 255, 255 );
    constexpr ImU32 lowerColor = IM_COL32( 100, 255, 140, 255 );
    constexpr ImU32 upperColor = IM_COL32( 255, 105, 90, 255 );
    constexpr ImU32 arcColor = IM_COL32( 180, 185, 200, 255 );

    for( const jointId id : model.getRagdollJoints() )
    {
        if( id == model.getRevoluteJoint() || !model.getWorld().IsValid( id ) ) continue;

        const revoluteJointData joint = model.getWorld().getRevoluteJointData( id );
        draw.DrawPoint( joint.anchorA, anchorColor, 6.0f );
        draw.DrawPoint( joint.anchorB, bodyColor, 5.0f );
        draw.DrawSegment( { joint.anchorA, joint.anchorB }, bodyColor );
        if( !joint.enableLimit ) continue;

        const rot2 reference = model.getWorld().GetBodyTransform( joint.bodyA ).rotation * rot2::FromRadians( joint.referenceAngle );
        const vec2 lower = joint.anchorA + 0.5f * Rotate( reference * rot2::FromRadians( joint.lowerAngle ), { 0.0f, -1.0f } );
        const vec2 upper = joint.anchorA + 0.5f * Rotate( reference * rot2::FromRadians( joint.upperAngle ), { 0.0f, -1.0f } );
        draw.DrawSegment( { joint.anchorA, lower }, lowerColor );
        draw.DrawSegment( { joint.anchorA, upper }, upperColor );

        vec2 previous = joint.anchorA + 0.38f * Rotate( reference * rot2::FromRadians( joint.lowerAngle ), { 0.0f, -1.0f } );
        for( int i = 1; i <= 12; ++i )
        {
            const float angle = joint.lowerAngle + ( joint.upperAngle - joint.lowerAngle ) * static_cast<float>( i ) / 12.0f;
            const vec2 next = joint.anchorA + 0.38f * Rotate( reference * rot2::FromRadians( angle ), { 0.0f, -1.0f } );
            draw.DrawSegment( { previous, next }, arcColor, 1.0f );
            previous = next;
        }
    }
}

std::unique_ptr<demo> createJointDemoView( demoKind kind )
{
    return std::make_unique<jointDemoView>( kind );
}

} // namespace zonai::sandbox