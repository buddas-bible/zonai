#include "rigidBodyDemoUi.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <numbers>
#include <imgui.h>
#include <imgui_internal.h>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include "debug/debugDraw.h"
#include "dynamics/constants.h"
#include "geometry/catmullRomSpline2.h"

namespace zonai::sandbox
{
namespace
{

struct shapeColors
{
    ImU32 outline = 0;
    ImU32 fill = 0;
};

const char* getBodyTypeName( bodyType type )
{
    switch( type )
    {
    case bodyType::Static:
        return "정적";

    case bodyType::Kinematic:
        return "키네마틱";

    case bodyType::Dynamic:
        return "동적";

    default:
        return "알 수 없음";
    }
}

shapeColors getShapeColors( bodyType type, bool awake )
{
    if( type == bodyType::Dynamic && !awake ) return { IM_COL32( 100, 125, 145, 220 ), IM_COL32( 100, 125, 145, 45 ) };

    switch( type )
    {
    case bodyType::Static:
        return { IM_COL32( 120, 220, 140, 255 ), IM_COL32( 120, 220, 140, 55 ) };

    case bodyType::Kinematic:
        return { IM_COL32( 245, 205, 90, 255 ), IM_COL32( 245, 205, 90, 60 ) };

    case bodyType::Dynamic:
        return { IM_COL32( 90, 190, 255, 255 ), IM_COL32( 90, 190, 255, 70 ) };

    default:
        return { IM_COL32( 230, 230, 230, 255 ), IM_COL32( 230, 230, 230, 50 ) };
    }
}

vec2 getWorldCenter( const world& world, bodyId bodyId )
{
    return TransformPoint( world.GetBodyTransform( bodyId ), world.GetBodyLocalCenter( bodyId ) );
}

bool drawCollisionBits( const char* label, const char* id, std::uint64_t& bits, int selectionLimit = 64 )
{
    char preview[96];
    if( bits == 0 )
    {
        std::snprintf( preview, sizeof( preview ), "선택 없음" );
    }
    else if( bits == ~std::uint64_t{ 0 } )
    {
        std::snprintf( preview, sizeof( preview ), "전체 레이어" );
    }
    else if( std::popcount( bits ) == 1 )
    {
        std::snprintf( preview, sizeof( preview ), "레이어 %02d", std::countr_zero( bits ) + 1 );
    }
    else
    {
        std::snprintf( preview, sizeof( preview ), "%d개 레이어 선택", std::popcount( bits ) );
    }
    bool changed = false;
    if( ImGui::BeginCombo( label, preview, ImGuiComboFlags_HeightLarge ) )
    {
        ImGui::PushID( id );
        if( selectionLimit == 64 )
        {
            if( ImGui::Button( "전체 선택###All" ) )
            {
                bits = ~std::uint64_t{ 0 };
                changed = true;
            }
            ImGui::SameLine();
        }
        if( ImGui::Button( "전체 해제###None" ) )
        {
            bits = 0;
            changed = true;
        }
        ImGui::PopID();
        for( int bit = 0; bit < 64; ++bit )
        {
            const auto flag = std::uint64_t{ 1 } << bit;
            bool enabled = ( bits & flag ) != 0;
            ImGui::BeginDisabled( !enabled && std::popcount( bits ) >= selectionLimit );
            char name[48];
            std::snprintf( name, sizeof( name ), "레이어 %02d##%s", bit + 1, id );
            if( ImGui::Checkbox( name, &enabled ) )
            {
                bits = enabled ? bits | flag : bits & ~flag;
                changed = true;
            }
            ImGui::EndDisabled();
        }
        ImGui::EndCombo();
    }
    return changed;
}

} // namespace

#pragma region DemoView

rigidBodyDemoUi::rigidBodyDemoUi( demoKind kind ) : rigidBodyDemo( kind )
{
    if( kind == demoKind::collisionComparison || kind == demoKind::frictionComparison || kind == demoKind::restitutionComparison || kind == demoKind::boxStack || kind == demoKind::ccdComparison || kind == demoKind::collisionFilterComparison || kind == demoKind::sensorWallComparison || kind == demoKind::sleepWakeComparison || kind == demoKind::dampingComparison || kind == demoKind::gravityScaleComparison || kind == demoKind::massImpulseComparison || kind == demoKind::impulsePointComparison || kind == demoKind::rotationalInertiaComparison || kind == demoKind::forceImpulseComparison || kind == demoKind::torqueImpulseComparison || kind == demoKind::dynamicKinematicComparison || kind == demoKind::compoundCenterComparison || kind == demoKind::fastRotationComparison || kind == demoKind::maximumLinearSpeedComparison || kind == demoKind::frictionMixingComparison || kind == demoKind::restitutionMixingComparison || kind == demoKind::lowSpeedRestitutionComparison || kind == demoKind::collisionGroupComparison || kind == demoKind::movingPlatformFrictionComparison || kind == demoKind::rollingSlidingFrictionComparison || kind == demoKind::frictionForceComparison || kind == demoKind::obliqueImpactFrictionComparison || kind == demoKind::collisionMassRatioComparison || kind == demoKind::centralOffsetImpactComparison || kind == demoKind::springDampingComparison || kind == demoKind::distanceMotorForceComparison || kind == demoKind::distanceLimitMotorComparison || kind == demoKind::revoluteMotorTorqueComparison || kind == demoKind::revoluteLimitMotorComparison || kind == demoKind::suspensionLoadComparison || kind == demoKind::suspensionLimitComparison || kind == demoKind::suspensionMotorComparison || kind == demoKind::weldElasticityComparison ) showLabels_ = true;
    selectedShapeIndex_ = kind == demoKind::playground || kind == demoKind::motorCar ? 2 : 1;
    if( getShapes().size() > static_cast<std::size_t>( selectedShapeIndex_ ) ) selectedShape_ = getShapes()[selectedShapeIndex_].shapeHandle;
}

void rigidBodyDemoUi::handleInput( const demoInput& input )
{
    if( getKind() == demoKind::splineRail )
    {
        if( input.mousePressed ) splineRailSelecting_ = input.mouseHeld && input.selectReprojectionSegment && !input.insertControlPoint;
        if( !input.mouseHeld ) splineRailSelecting_ = false;
        if( splineRailSelecting_ )
        {
            if( input.mousePressed && getWorld().IsValid( getSplineRailJoint() ) && std::isfinite( input.pixelsPerMeter ) && input.pixelsPerMeter > 0.0f )
            {
                const float radius = 10.0f / input.pixelsPerMeter;
                if( std::isfinite( radius ) && radius > 0.0f )
                {
                    const auto rail = getWorld().getSplineRailJointData( getSplineRailJoint() );
                    const vec2 point = InverseTransformPoint( getWorld().GetBodyTransform( rail.bodyA ), input.mousePosition );
                    // Ctrl 삽입과 같은 0.01px 거리 오차를 사용함.
                    const auto closest = ClosestPoint( { getWorld().getSplineRailJointPoints( getSplineRailJoint() ), rail.isLoop }, point, static_cast<double>( radius ) * 0.001 );
                    if( closest && closest->distanceSquared <= static_cast<double>( radius ) * radius )
                    {
                        splineRailReprojectSelected_ = true;
                        splineRailReprojectSegment_ = closest->segmentIndex;
                    }
                }
            }
            auto forwarded = input;
            forwarded.mousePressed = false;
            forwarded.mouseHeld = false;
            rigidBodyDemo::handleInput( forwarded );
            return;
        }
    }
    rigidBodyDemo::handleInput( input );
}

void rigidBodyDemoUi::cancelInput()
{
    splineRailSelecting_ = false;
    rigidBodyDemo::cancelInput();
}

void rigidBodyDemoUi::drawControls()
{
    // 한글 이름이 고정 폭 패널에서 잘리지 않도록 입력 영역과 이름의 폭을 나눔.
    ImGui::PushItemWidth( ImGui::GetContentRegionAvail().x * 0.45f );
    if( ImGui::CollapsingHeader( "월드 설정###WorldSettings" ) )
    {
        drawWorldSettings();
    }
    drawInspector();
    drawExperimentControls();
    if( ImGui::CollapsingHeader( "마우스 조인트 설정###MouseControls" ) )
    {
        drawMouseControls();
    }
    if( ImGui::CollapsingHeader( "물리 정보 표시 설정###DebugDrawSettings" ) )
    {
        drawDebugSettings();
    }
    ImGui::PopItemWidth();
}

void rigidBodyDemoUi::draw( debugDraw& draw ) const
{
    const broadPhase& phase = getWorld().GetBroadPhase();
    const dynamicTree& dynamicTreeRef = phase.GetTree( bodyType::Dynamic );
    const dynamicTree& kinematicTree = phase.GetTree( bodyType::Kinematic );
    const dynamicTree& staticTree = phase.GetTree( bodyType::Static );
    if( showGrid_ )
    {
        draw.DrawGrid();
    }

    if( getKind() == demoKind::windFlow )
    {
        if( const fluidField2* field = getWindField() )
        {
            constexpr ImU32 WIND_COLOR = IM_COL32( 85, 205, 255, 225 );
            for( int y = 0; y < field->getHeight(); y += 2 )
                for( int x = 0; x < field->getWidth(); x += 2 )
                {
                    const vec2 point = field->getCellCenter( x, y );
                    if( field->isSolidCell( x, y ) )
                    {
                        draw.DrawPoint( point, IM_COL32( 255, 155, 85, 150 ), 3.0f );
                        continue;
                    }
                    const vec2 velocity = field->sampleVelocity( point );
                    const float speed = Length( velocity );
                    if( speed > 0.025f )
                        draw.DrawArrow( point, velocity, WIND_COLOR, std::min( 0.48f, 0.12f + 0.12f * speed ) );
                }
            draw.DrawPoint( { -3.5f, 0.0f }, IM_COL32( 250, 200, 90, 245 ), 6.0f );
            draw.DrawLabel( { -3.5f, -0.45f }, "바람 소스", IM_COL32( 250, 200, 90, 245 ) );
        }
    }

    if( getKind() == demoKind::splinePath || getKind() == demoKind::splineRail )
    {
        const auto points = SampleAdaptive( { getSplineControlPoints() }, 0.01f );
        const auto pose = getSplineControlTransform();
        for( std::size_t i = 1; i < points.size(); ++i )
            draw.DrawSegment( { TransformPoint( pose, points[i - 1] ), TransformPoint( pose, points[i] ) }, IM_COL32( 180, 150, 255, 255 ), 1.5f );
    }

    constexpr ImU32 DYNAMIC_TREE_LEAF = IM_COL32( 70, 200, 255, 220 );
    constexpr ImU32 DYNAMIC_TREE_INTERNAL = IM_COL32( 80, 120, 255, 140 );

    constexpr ImU32 KINEMATIC_TREE_LEAF = IM_COL32( 245, 205, 90, 220 );
    constexpr ImU32 KINEMATIC_TREE_INTERNAL = IM_COL32( 210, 165, 70, 130 );

    constexpr ImU32 STATIC_TREE_LEAF = IM_COL32( 110, 220, 130, 220 );
    constexpr ImU32 STATIC_TREE_INTERNAL = IM_COL32( 80, 160, 100, 130 );

    if( showStaticTree_ )
    {
        draw.DrawTree( staticTree, "정", showTreeLeaves_, showTreeInternal_, showTreeLabels_, STATIC_TREE_LEAF, STATIC_TREE_INTERNAL );
    }

    if( showKinematicTree_ )
    {
        draw.DrawTree( kinematicTree, "운", showTreeLeaves_, showTreeInternal_, showTreeLabels_, KINEMATIC_TREE_LEAF, KINEMATIC_TREE_INTERNAL );
    }

    if( showDynamicTree_ )
    {
        draw.DrawTree( dynamicTreeRef, "동", showTreeLeaves_, showTreeInternal_, showTreeLabels_, DYNAMIC_TREE_LEAF, DYNAMIC_TREE_INTERNAL );
    }

    constexpr ImU32 AABB_COLOR = IM_COL32( 210, 100, 230, 210 );

    constexpr ImU32 FAT_AABB_COLOR = IM_COL32( 255, 185, 80, 190 );

    constexpr ImU32 LABEL_COLOR = IM_COL32( 230, 232, 238, 255 );

    constexpr ImU32 COM_COLOR = IM_COL32( 255, 90, 180, 255 );

    constexpr ImU32 VELOCITY_COLOR = IM_COL32( 80, 220, 255, 245 );

    for( const visualShape& visual : getShapes() )
    {
        if( !getWorld().IsValid( visual.bodyHandle ) || !getWorld().IsValid( visual.shapeHandle ) ) continue;

        if( !getWorld().IsValid( visual.bodyHandle ) || !getWorld().IsValid( visual.shapeHandle ) )
        {
            ImGui::TextUnformatted( "선택한 오브젝트가 삭제되었습니다." );
            return;
        }
        const body& bodyRef = getWorld().GetBody( visual.bodyHandle );

        const shape& shapeRef = getWorld().GetShape( visual.shapeHandle );

        const transform2 transform = getWorld().GetBodyTransform( visual.bodyHandle );

        const shapeColors colors = getWorld().IsShapeSensor( visual.shapeHandle ) ? shapeColors{ IM_COL32( 230, 170, 255, 255 ), IM_COL32( 230, 170, 255, 25 ) } : getShapeColors( bodyRef.type, bodyRef.awake );

        draw.DrawShape( shapeRef.geometry, transform, colors.outline, colors.fill );
        if( getKind() == demoKind::terrainChain || getKind() == demoKind::splineTerrain )
            if( const auto* edge = std::get_if<chainSegment2>( &shapeRef.geometry ) )
            {
                const vec2 a = TransformPoint( transform, edge->segment.a );
                const vec2 b = TransformPoint( transform, edge->segment.b );
                draw.DrawArrow( 0.5f * ( a + b ), Cross( Normalize( b - a ), 1.0f ), IM_COL32( 100, 255, 140, 255 ), 0.6f );
            }

        if( showShapeAABBs_ )
        {
            draw.DrawAABB( getWorld().GetShapeAABB( visual.shapeHandle ), AABB_COLOR );
        }

        if( showFatAABBs_ )
        {
            draw.DrawAABB( getWorld().GetShapeFatAABB( visual.shapeHandle ), FAT_AABB_COLOR );
        }

        const vec2 worldCenter = getWorldCenter( getWorld(), visual.bodyHandle );

        if( showCOM_ && bodyRef.type != bodyType::Static )
        {
            draw.DrawPoint( worldCenter, COM_COLOR, 4.5f );
        }

        if( showVelocities_ && bodyRef.type != bodyType::Static )
        {
            const vec2 velocity = getWorld().GetBodyLinearVelocity( visual.bodyHandle );

            if( LengthSquared( velocity ) > 1e-6f )
            {
                const float arrowLength = std::clamp( Length( velocity ) * 0.18f, 0.2f, 2.0f );

                draw.DrawArrow( worldCenter, velocity, VELOCITY_COLOR, arrowLength );
            }
        }

        if( showLabels_ )
        {
            vec2 labelPosition = transform.position;
            if( getKind() == demoKind::compoundCenterComparison ) labelPosition = TransformPoint( transform, std::get<circle2>( shapeRef.geometry ).center );
            draw.DrawLabel( labelPosition, visual.label, LABEL_COLOR );
        }
    }

    if( getKind() == demoKind::compoundCenterComparison )
    {
        for( std::size_t i : { 0u, 2u } )
        {
            const auto pose = getWorld().GetBodyTransform( getShapes()[i].bodyHandle );
            const auto& left = std::get<circle2>( getWorld().GetShape( getShapes()[i].shapeHandle ).geometry );
            const auto& right = std::get<circle2>( getWorld().GetShape( getShapes()[i + 1].shapeHandle ).geometry );
            draw.DrawSegment( { TransformPoint( pose, left.center ), TransformPoint( pose, right.center ) }, IM_COL32( 180, 185, 200, 255 ), 1.5f );
        }
    }
    if( getKind() == demoKind::impulsePointComparison || getKind() == demoKind::compoundCenterComparison )
    {
        constexpr ImU32 POINT_COLOR = IM_COL32( 255, 185, 80, 255 );
        for( std::size_t i = 0; i < getShapes().size(); ++i )
            if( const auto point = getImpulseTrialPoint( i ) )
            {
                draw.DrawPoint( *point, POINT_COLOR, 5.0f );
                draw.DrawArrow( *point, { 0, 1 }, POINT_COLOR, 0.75f );
            }
    }

    if( hasEditableSpline() )
    {
        const auto pose = getSplineControlTransform();
        const auto controls = getSplineControlPoints();
        for( std::size_t i = 0; i < controls.size(); ++i )
        {
            const ImU32 color = getTerrainPointDragIndex() == i ? IM_COL32( 255, 170, 70, 255 ) : IM_COL32( 230, 170, 255, 255 );
            draw.DrawPoint( TransformPoint( pose, controls[i] ), color, 6.0f );
        }
        const auto target = getTerrainTrialTarget( terrainTrial::front );
        if( target )
        {
            constexpr ImU32 TRIAL_COLOR = IM_COL32( 80, 220, 255, 255 );
            draw.DrawPoint( target->point, TRIAL_COLOR, 6.0f );
            draw.DrawArrow( target->point, Cross( target->tangent, 1.0f ), TRIAL_COLOR, 0.8f );
        }
    }

    if( getWorld().IsValid( getSplineRailJoint() ) )
    {
        const auto rail = getWorld().getSplineRailJointData( getSplineRailJoint() );
        draw.DrawPoint( rail.anchorA, IM_COL32( 255, 170, 70, 255 ) );
        draw.DrawArrow( rail.anchorA, rail.tangent, IM_COL32( 80, 220, 255, 255 ), 0.7f );
        draw.DrawArrow( rail.anchorA, rail.normal, IM_COL32( 100, 255, 140, 255 ), 0.5f );
        if( splineRailReprojectSelected_ )
        {
            const catmullRomSpline2 spline{ getWorld().getSplineRailJointPoints( getSplineRailJoint() ), rail.isLoop };
            const std::size_t count = spline.isLoop ? spline.points.size() : spline.points.size() - 1;
            if( splineRailReprojectSegment_ < count )
            {
                const auto points = SampleAdaptive( spline, { splineRailReprojectSegment_, 0.0f }, { splineRailReprojectSegment_, 1.0f }, 0.01f );
                if( points.size() >= 2 )
                {
                    constexpr ImU32 COLOR = IM_COL32( 255, 220, 80, 255 );
                    const auto pose = getWorld().GetBodyTransform( rail.bodyA );
                    for( std::size_t i = 1; i < points.size(); ++i )
                        draw.DrawSegment( { TransformPoint( pose, points[i - 1] ), TransformPoint( pose, points[i] ) }, COLOR, 3.0f );
                    draw.DrawPoint( TransformPoint( pose, points.front() ), COLOR, 6.0f );
                    draw.DrawPoint( TransformPoint( pose, points.back() ), COLOR, 6.0f );
                    char label[32];
                    std::snprintf( label, sizeof( label ), "구간 %zu", splineRailReprojectSegment_ + 1 );
                    draw.DrawLabel( TransformPoint( pose, Evaluate( spline, splineRailReprojectSegment_, 0.5f ) ), label, COLOR );
                }
            }
        }
    }

    if( getWorld().IsValid( getMouseJoint() ) )
    {
        const auto joint = getWorld().getMouseJointData( getMouseJoint() );
        constexpr ImU32 MOUSE_COLOR = IM_COL32( 255, 170, 70, 255 );
        draw.DrawSegment( { joint.anchorB, joint.target }, MOUSE_COLOR );
        draw.DrawPoint( joint.anchorB, MOUSE_COLOR );
        draw.DrawPoint( joint.target, MOUSE_COLOR );
    }
    for( const auto id : getWeldComparisonJoints() )
        if( getWorld().IsValid( id ) )
        {
            const auto joint = getWorld().getWeldJointData( id );
            draw.DrawSegment( { joint.anchorA, joint.anchorB }, IM_COL32( 230, 170, 255, 255 ) );
            draw.DrawPoint( joint.anchorA, IM_COL32( 100, 235, 220, 255 ), 7.0f );
            draw.DrawPoint( joint.anchorB, IM_COL32( 230, 170, 255, 255 ) );
        }
    for( const auto id : getAngleLimitComparisonJoints() )
        if( getWorld().IsValid( id ) )
        {
            const auto joint = getWorld().getRevoluteJointData( id );
            draw.DrawPoint( joint.anchorA, IM_COL32( 100, 200, 255, 255 ) );
            if( joint.enableLimit )
            {
                const vec2 lower{ std::cos( joint.lowerAngle ), std::sin( joint.lowerAngle ) };
                const vec2 upper{ std::cos( joint.upperAngle ), std::sin( joint.upperAngle ) };
                draw.DrawSegment( { joint.anchorA, joint.anchorA + 0.65f * lower }, IM_COL32( 100, 230, 140, 255 ) );
                draw.DrawSegment( { joint.anchorA, joint.anchorA + 0.65f * upper }, IM_COL32( 255, 100, 100, 255 ) );
            }
        }
    for( const auto id : getTorqueComparisonJoints() )
        if( getWorld().IsValid( id ) )
        {
            const auto joint = getWorld().getRevoluteJointData( id );
            draw.DrawPoint( joint.anchorA, IM_COL32( 100, 200, 255, 255 ) );
        }
    for( const auto id : getMotorComparisonJoints() )
        if( getWorld().IsValid( id ) )
        {
            const auto joint = getWorld().getDistanceJointData( id );
            constexpr ImU32 MOTOR_COLOR = IM_COL32( 100, 200, 255, 255 );
            draw.DrawSegment( { joint.anchorA, joint.anchorB }, MOTOR_COLOR );
            draw.DrawPoint( joint.anchorA, MOTOR_COLOR );
            draw.DrawPoint( joint.anchorB, MOTOR_COLOR );
        }
    for( const auto id : getLimitComparisonJoints() )
        if( getWorld().IsValid( id ) )
        {
            const auto joint = getWorld().getDistanceJointData( id );
            constexpr ImU32 MOTOR_COLOR = IM_COL32( 100, 200, 255, 255 );
            draw.DrawSegment( { joint.anchorA, joint.anchorB }, MOTOR_COLOR );
            draw.DrawPoint( joint.anchorA, MOTOR_COLOR );
            draw.DrawPoint( joint.anchorB, MOTOR_COLOR );
            if( joint.enableLimit )
            {
                draw.DrawCircle( { joint.anchorA, joint.minLength }, IM_COL32( 100, 230, 140, 100 ), 0 );
                draw.DrawCircle( { joint.anchorA, joint.maxLength }, IM_COL32( 255, 100, 100, 100 ), 0 );
            }
        }
    for( const auto id : getSpringComparisonJoints() )
        if( getWorld().IsValid( id ) )
        {
            const auto joint = getWorld().getDistanceJointData( id );
            constexpr ImU32 SPRING_COLOR = IM_COL32( 230, 170, 255, 255 );
            draw.DrawSegment( { joint.anchorA, joint.anchorB }, SPRING_COLOR );
            draw.DrawPoint( joint.anchorA, SPRING_COLOR );
            draw.DrawPoint( joint.anchorB, SPRING_COLOR );
            draw.DrawCircle( { joint.anchorA, joint.length }, IM_COL32( 230, 170, 255, 100 ), 0 );
        }
    if( getWorld().IsValid( getPendulumJoint() ) )
    {
        constexpr ImU32 JOINT_COLOR = IM_COL32( 230, 170, 255, 255 );
        const distanceJointData joint = getWorld().getDistanceJointData( getPendulumJoint() );
        draw.DrawSegment( { joint.anchorA, joint.anchorB }, JOINT_COLOR );
        draw.DrawPoint( joint.anchorA, JOINT_COLOR );
        draw.DrawPoint( joint.anchorB, JOINT_COLOR );
        draw.DrawCircle( { joint.anchorA, joint.length }, IM_COL32( 230, 170, 255, 100 ), 0 );
        if( joint.enableSpring && joint.enableLimit && joint.minLength < joint.maxLength )
        {
            const vec2 axis = Normalize( joint.anchorB - joint.anchorA );
            const vec2 lower = joint.anchorA + joint.minLength * axis, upper = joint.anchorA + joint.maxLength * axis;
            draw.DrawSegment( { lower, upper }, IM_COL32( 160, 160, 160, 255 ) );
            draw.DrawPoint( lower, IM_COL32( 100, 255, 140, 255 ) );
            draw.DrawPoint( upper, IM_COL32( 255, 105, 90, 255 ) );
        }
    }

    if( getWorld().IsValid( getRevoluteJoint() ) )
    {
        const revoluteJointData joint = getWorld().getRevoluteJointData( getRevoluteJoint() );
        constexpr ImU32 ANCHOR_COLOR = IM_COL32( 100, 235, 220, 255 );
        constexpr ImU32 ROD_COLOR = IM_COL32( 230, 170, 255, 255 );
        draw.DrawPoint( joint.anchorA, ANCHOR_COLOR, 7.0f );
        draw.DrawPoint( joint.anchorB, ROD_COLOR );
        draw.DrawSegment( { joint.anchorA, joint.anchorB }, ROD_COLOR );
        draw.DrawArrow( joint.anchorB, Rotate( getWorld().GetBodyTransform( joint.bodyB ).rotation, { 0.0f, -1.0f } ), ROD_COLOR, 0.5f );
        if( joint.enableLimit )
        {
            // 막대의 초기 아래쪽 방향을 기준으로 허용 각도를 표시함. A가 회전하면 경계도 함께 회전함.
            const rot2 reference = getWorld().GetBodyTransform( joint.bodyA ).rotation * rot2::FromRadians( joint.referenceAngle );
            const vec2 lower = joint.anchorA + 0.85f * Rotate( reference * rot2::FromRadians( joint.lowerAngle ), { 0.0f, -1.0f } );
            const vec2 upper = joint.anchorA + 0.85f * Rotate( reference * rot2::FromRadians( joint.upperAngle ), { 0.0f, -1.0f } );
            draw.DrawSegment( { joint.anchorA, lower }, IM_COL32( 100, 255, 140, 255 ) );
            draw.DrawSegment( { joint.anchorA, upper }, IM_COL32( 255, 105, 90, 255 ) );
            vec2 previous = joint.anchorA + 0.65f * Rotate( reference * rot2::FromRadians( joint.lowerAngle ), { 0.0f, -1.0f } );
            for( int i = 1; i <= 16; ++i )
            {
                const float angle = joint.lowerAngle + ( joint.upperAngle - joint.lowerAngle ) * static_cast<float>( i ) / 16.0f;
                const vec2 next = joint.anchorA + 0.65f * Rotate( reference * rot2::FromRadians( angle ), { 0.0f, -1.0f } );
                draw.DrawSegment( { previous, next }, IM_COL32( 180, 185, 200, 255 ), 1.0f );
                previous = next;
            }
        }
    }

    for( const jointId id : { getWheelJoint(), getCarJoints()[0], getCarJoints()[1], getSuspensionComparisonJoints()[0], getSuspensionComparisonJoints()[1], getSuspensionComparisonJoints()[2], getSuspensionLimitJoints()[0], getSuspensionLimitJoints()[1], getSuspensionMotorJoints()[0], getSuspensionMotorJoints()[1] } )
    {
        if( !getWorld().IsValid( id ) ) continue;

        const wheelJointData joint = getWorld().getWheelJointData( id );
        constexpr ImU32 AXIS_COLOR = IM_COL32( 100, 235, 220, 255 );
        constexpr ImU32 WHEEL_COLOR = IM_COL32( 230, 170, 255, 255 );
        draw.DrawSegment( { joint.anchorA - joint.axis, joint.anchorA + joint.axis }, IM_COL32( 160, 160, 160, 255 ) );
        draw.DrawArrow( joint.anchorA, joint.axis, AXIS_COLOR, 0.5f );
        draw.DrawPoint( joint.anchorA, AXIS_COLOR, 7.0f );
        draw.DrawPoint( joint.anchorB, WHEEL_COLOR );
        if( joint.enableLimit )
        {
            const vec2 lower = joint.anchorA + joint.lowerTranslation * joint.axis;
            const vec2 upper = joint.anchorA + joint.upperTranslation * joint.axis;
            const vec2 tick = 0.09f * Cross( 1.0f, joint.axis );
            draw.DrawSegment( { lower, upper }, IM_COL32( 190, 190, 190, 255 ) );
            draw.DrawSegment( { lower - tick, lower + tick }, IM_COL32( 90, 220, 110, 255 ) );
            draw.DrawSegment( { upper - tick, upper + tick }, IM_COL32( 245, 100, 100, 255 ) );
        }
        // 지지대와 바퀴 사이 연결선. 물리 스프링의 변위 0은 별도의 청록색 기준점임.
        draw.DrawSegment( { ( getKind() == demoKind::suspensionLoadComparison || getKind() == demoKind::suspensionLimitComparison || getKind() == demoKind::suspensionMotorComparison ) ? joint.anchorA : getWorld().GetBodyTransform( joint.bodyA ).position, joint.anchorB }, WHEEL_COLOR );
        draw.DrawArrow( joint.anchorB, Rotate( getWorld().GetBodyTransform( joint.bodyB ).rotation, { 1.0f, 0.0f } ), WHEEL_COLOR, 0.3f );
    }

    if( showContacts_ )
    {
        constexpr ImU32 CONTACT_COLOR = IM_COL32( 255, 220, 70, 255 );

        constexpr ImU32 PENETRATION_COLOR = IM_COL32( 255, 105, 90, 255 );

        constexpr ImU32 NORMAL_COLOR = IM_COL32( 100, 255, 140, 255 );

        constexpr ImU32 CONTACT_TEXT_COLOR = IM_COL32( 245, 245, 245, 255 );

        for( const contactData& contact : getContacts() )
        {
            for( int i = 0; i < contact.manifold.pointCount; ++i )
            {
                const manifoldPoint2& point = contact.manifold.points[i];

                const ImU32 pointColor = point.separation < -0.01f ? PENETRATION_COLOR : CONTACT_COLOR;

                draw.DrawPoint( point.point, pointColor, 5.0f );

                draw.DrawArrow( point.point, contact.manifold.normal, NORMAL_COLOR, 0.7f );

                if( showContactDetails_ )
                {
                    char label[96]{};

                    std::snprintf( label, sizeof( label ), "간격 %.3f  수직 충격량 %.2f  마찰 충격량 %.2f", point.separation, point.normalImpulse, point.tangentImpulse );

                    draw.DrawLabel( point.point, label, CONTACT_TEXT_COLOR );
                }
            }
        }
    }

    if( isContactEventVisualizationEnabled() )
    {
        for( const contactEventTrace2& trace : getContactEventTraces() )
        {
            const int alpha = static_cast<int>( 255.0f * std::clamp( trace.remainingTime / trace.duration, 0.0f, 1.0f ) );
            if( trace.type == contactEventTraceType::begin )
            {
                const ImU32 color = IM_COL32( 110, 240, 140, alpha );
                draw.DrawPoint( trace.point, color, 6.0f );
                draw.DrawLabel( trace.point, "시작", color );
            }
            else if( trace.type == contactEventTraceType::end )
            {
                const ImU32 color = IM_COL32( 255, 170, 90, alpha );
                draw.DrawPoint( trace.point, color, 6.0f );
                draw.DrawLabel( trace.point, "종료", color );
            }
            else
            {
                const ImU32 color = IM_COL32( 255, 90, 105, alpha );
                draw.DrawPoint( trace.point, color, 7.0f );
                draw.DrawArrow( trace.point, trace.normal, color, 0.75f );

                char label[96]{};
                std::snprintf( label, sizeof( label ), "충격 %.2f m/s  %.2f N·s", trace.approachSpeed, trace.normalImpulse );
                draw.DrawLabel( trace.point, label, color );
            }
        }
    }
}

#pragma endregion DemoView

#pragma region Controls

void rigidBodyDemoUi::drawWorldSettings()
{
    vec2 gravity = getWorld().GetGravity();

    float gravityValues[2]{ gravity.x, gravity.y };

    if( ImGui::DragFloat2( "중력###Gravity", gravityValues, 0.1f, -30.0f, 30.0f, "%.2f" ) )
    {
        getWorld().SetGravity( { gravityValues[0], gravityValues[1] } );
    }

    float maximumLinearSpeed = getWorld().GetMaximumLinearSpeed();

    if( ImGui::DragFloat( "최대 선속도###Max linear speed", &maximumLinearSpeed, 1.0f, 1.0f, 1000.0f, "%.1f m/s" ) )
    {
        getWorld().SetMaximumLinearSpeed( maximumLinearSpeed );
    }

    bool sleepingEnabled = getWorld().IsSleepingEnabled();

    if( ImGui::Checkbox( "수면 허용###Enable sleeping", &sleepingEnabled ) )
    {
        getWorld().SetSleepingEnabled( sleepingEnabled );
    }

    bool continuousEnabled = getWorld().IsContinuousEnabled();

    if( ImGui::Checkbox( "CCD 검사###Continuous collision", &continuousEnabled ) )
    {
        getWorld().SetContinuousEnabled( continuousEnabled );
    }

    float contactRecycleDistance = getWorld().GetContactRecycleDistance();

    if( ImGui::DragFloat( "접촉 재사용 거리###Contact recycle dist", &contactRecycleDistance, 0.001f, 0.0f, 0.2f, "%.3f m" ) )
    {
        getWorld().SetContactRecycleDistance( contactRecycleDistance );
    }
}

void rigidBodyDemoUi::drawInspector()
{
    ImGui::Separator();
    ImGui::TextUnformatted( "오브젝트 인스펙터" );

    if( !getShapes().empty() )
    {
        const auto selectedVisualIt = std::find_if( getShapes().begin(), getShapes().end(),
            [this]( const visualShape& visual ) { return visual.shapeHandle == selectedShape_; } );
        selectedShapeIndex_ = selectedVisualIt != getShapes().end() ? static_cast<int>( selectedVisualIt - getShapes().begin() ) : 0;

        const visualShape& selectedVisual = getShapes()[selectedShapeIndex_];

        if( ImGui::BeginCombo( "오브젝트###Object", selectedVisual.label ) )
        {
            for( int i = 0; i < static_cast<int>( getShapes().size() ); ++i )
            {
                const bool selected = i == selectedShapeIndex_;

                if( ImGui::Selectable( getShapes()[i].label, selected ) )
                {
                    selectedShapeIndex_ = i;
                }

                if( selected )
                {
                    ImGui::SetItemDefaultFocus();
                }
            }

            ImGui::EndCombo();
        }

        const visualShape& visual = getShapes()[selectedShapeIndex_];
        selectedShape_ = visual.shapeHandle;

        if( !getWorld().IsValid( visual.bodyHandle ) || !getWorld().IsValid( visual.shapeHandle ) )
        {
            ImGui::TextUnformatted( "선택한 오브젝트가 삭제되었습니다." );
            return;
        }
        const body& bodyRef = getWorld().GetBody( visual.bodyHandle );

        ImGui::Text( "오브젝트 유형: %s", getBodyTypeName( bodyRef.type ) );

#pragma region Transform
        if( ImGui::CollapsingHeader( "위치와 회전###Transform", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            transform2 transform = getWorld().GetBodyTransform( visual.bodyHandle );
            float position[2]{ transform.position.x, transform.position.y };
            bool changed = ImGui::DragFloat2( "위치###Position", position, 0.05f, -100.0f, 100.0f, "%.2f" );
            if( changed )
            {
                transform.position = { position[0], position[1] };
            }
            float rotation = std::atan2( transform.rotation.s, transform.rotation.c );
            if( ImGui::DragFloat( "회전###Rotation", &rotation, 0.01f, -3.14159265f, 3.14159265f, "%.3f rad" ) )
            {
                transform.rotation = rot2::FromRadians( rotation );
                changed = true;
            }
            if( changed )
            {
                getWorld().SetBodyTransform( visual.bodyHandle, transform );
                refreshContacts();
            }
        }
#pragma endregion Transform

#pragma region PhysicsQuantities
        if( ImGui::CollapsingHeader( "물리량###Physics quantities", ImGuiTreeNodeFlags_DefaultOpen ) )
        {
            if( bodyRef.type != bodyType::Static )
            {
                const vec2 linearVelocity = getWorld().GetBodyLinearVelocity( visual.bodyHandle );
                float velocity[2]{ linearVelocity.x, linearVelocity.y };
                if( ImGui::DragFloat2( "선속도###Linear velocity", velocity, 0.05f, -100.0f, 100.0f, "%.2f" ) )
                {
                    getWorld().SetBodyLinearVelocity( visual.bodyHandle, { velocity[0], velocity[1] } );
                }
                float angularVelocity = getWorld().GetBodyAngularVelocity( visual.bodyHandle );
                if( ImGui::DragFloat( "각속도###Angular velocity", &angularVelocity, 0.05f, -100.0f, 100.0f, "%.2f rad/s" ) )
                {
                    getWorld().SetBodyAngularVelocity( visual.bodyHandle, angularVelocity );
                }
                // 지정한 경로의 목표 transform을 감쇠 없이 따라가야 함.
                if( getKind() != demoKind::splinePath )
                {
                    float linearDamping = getWorld().GetBodyLinearDamping( visual.bodyHandle );
                    if( ImGui::DragFloat( "선형 감쇠###Linear damping", &linearDamping, 0.05f, 0.0f, 20.0f, "%.2f" ) )
                    {
                        getWorld().SetBodyLinearDamping( visual.bodyHandle, linearDamping );
                    }
                    float angularDamping = getWorld().GetBodyAngularDamping( visual.bodyHandle );
                    if( ImGui::DragFloat( "회전 감쇠###Angular damping", &angularDamping, 0.05f, 0.0f, 20.0f, "%.2f" ) )
                    {
                        getWorld().SetBodyAngularDamping( visual.bodyHandle, angularDamping );
                    }
                }
                float gravityScale = getWorld().GetBodyGravityScale( visual.bodyHandle );
                if( ImGui::DragFloat( "중력 배율###Gravity scale", &gravityScale, 0.05f, -10.0f, 10.0f, "%.2f" ) )
                {
                    getWorld().SetBodyGravityScale( visual.bodyHandle, gravityScale );
                }
            }
            float density = getWorld().GetShapeDensity( visual.shapeHandle );
            if( ImGui::DragFloat( "밀도###Density", &density, 0.05f, 0.0f, 100.0f, "%.2f" ) )
            {
                getWorld().SetShapeDensity( visual.shapeHandle, density );
            }
            ImGui::Text( "질량: %.3f", getWorld().GetBodyMass( visual.bodyHandle ) );
            ImGui::Text( "회전 관성: %.3f", getWorld().GetBodyRotationalInertia( visual.bodyHandle ) );
        }
#pragma endregion PhysicsQuantities

#pragma region CollisionMaterial
        if( ImGui::CollapsingHeader( "충돌 재질###Collision material" ) )
        {
            float friction = getWorld().GetShapeFriction( visual.shapeHandle );
            if( ImGui::DragFloat( "마찰 계수###Friction", &friction, 0.02f, 0.0f, 5.0f, "%.2f" ) )
            {
                getWorld().SetShapeFriction( visual.shapeHandle, friction );
            }
            float restitution = getWorld().GetShapeRestitution( visual.shapeHandle );
            if( ImGui::DragFloat( "반발 계수###Restitution", &restitution, 0.02f, 0.0f, 2.0f, "%.2f" ) )
            {
                getWorld().SetShapeRestitution( visual.shapeHandle, restitution );
            }
        }
#pragma endregion CollisionMaterial

#pragma region CollisionMask
        if( ImGui::CollapsingHeader( "충돌 마스크###Collision mask" ) )
        {
            collisionFilter filter = getWorld().GetShapeFilter( visual.shapeHandle );
            bool changed = drawCollisionBits( "소속 레이어###Category membership", "Category", filter.categoryBits );
            ImGui::TextWrapped( "레이어 간 충돌 관계는 공통 충돌 설정에서 편집합니다." );
            if( ImGui::CollapsingHeader( "개별 추가 제한 (고급)###ObjectCollisionOverrides" ) )
            {
                changed |= drawCollisionBits( "충돌 대상 레이어###Collision partners", "Mask", filter.maskBits );
                changed |= ImGui::InputInt( "충돌 그룹###Group index", &filter.groupIndex );
                ImGui::TextWrapped( "마스크는 개별 충돌을 추가 제한합니다. 양쪽 도형이 서로를 허용해야 합니다. 같은 0이 아닌 그룹은 개별 마스크보다 우선하며 양수는 허용, 음수는 제외합니다." );
            }
            if( changed )
            {
                getWorld().SetShapeFilter( visual.shapeHandle, filter );
                refreshContacts();
            }
        }
#pragma endregion CollisionMask

#pragma region SleepAndCcd
        if( bodyRef.type != bodyType::Static && ImGui::CollapsingHeader( "Sleep과 CCD###Sleep and CCD" ) )
        {
            bool awake = getWorld().IsBodyAwake( visual.bodyHandle );
            if( ImGui::Checkbox( "Awake###Awake", &awake ) )
            {
                getWorld().SetBodyAwake( visual.bodyHandle, awake );
            }
            bool sleepEnabled = getWorld().IsBodySleepEnabled( visual.bodyHandle );
            if( ImGui::Checkbox( "오브젝트 수면 허용###Body sleep", &sleepEnabled ) )
            {
                getWorld().SetBodySleepEnabled( visual.bodyHandle, sleepEnabled );
            }
            float sleepThreshold = getWorld().GetBodySleepThreshold( visual.bodyHandle );
            if( ImGui::DragFloat( "수면 속도 기준###Sleep threshold", &sleepThreshold, 0.005f, 0.0f, 5.0f, "%.3f m/s" ) )
            {
                getWorld().SetBodySleepThreshold( visual.bodyHandle, sleepThreshold );
            }
            float safetyFactor = getWorld().GetBodySafetyFactor( visual.bodyHandle );
            if( ImGui::DragFloat( "CCD 안전 계수###CCD safety factor", &safetyFactor, 0.01f, 0.01f, 2.0f, "%.2f" ) )
            {
                getWorld().SetBodySafetyFactor( visual.bodyHandle, safetyFactor );
            }
            bool contactRecycling = getWorld().IsBodyContactRecyclingEnabled( visual.bodyHandle );
            if( ImGui::Checkbox( "접촉 재사용###Contact recycling", &contactRecycling ) )
            {
                getWorld().SetBodyContactRecyclingEnabled( visual.bodyHandle, contactRecycling );
            }
            bool bullet = getWorld().IsBodyBullet( visual.bodyHandle );
            if( ImGui::Checkbox( "고속 충돌 검사###Bullet", &bullet ) )
            {
                getWorld().SetBodyBullet( visual.bodyHandle, bullet );
            }
            bool fastRotation = getWorld().IsBodyFastRotationAllowed( visual.bodyHandle );
            if( ImGui::Checkbox( "빠른 회전 허용###Allow fast rotation", &fastRotation ) )
            {
                getWorld().SetBodyFastRotationAllowed( visual.bodyHandle, fastRotation );
            }
            ImGui::Text( "고속 오브젝트: %s", getWorld().IsBodyFast( visual.bodyHandle ) ? "예" : "아니요" );
            ImGui::Text( "이번 단계의 충돌 시점 감지: %s", getWorld().HadBodyTimeOfImpact( visual.bodyHandle ) ? "예" : "아니요" );
        }
#pragma endregion SleepAndCcd

#pragma region Sensors
        if( ImGui::CollapsingHeader( "센서###Sensors" ) )
        {
            const bool isSensor = getWorld().IsShapeSensor( visual.shapeHandle );
            ImGui::Text( "센서: %s", isSensor ? "예" : "아니요" );
            bool enabled = getWorld().AreShapeSensorEventsEnabled( visual.shapeHandle );
            if( ImGui::Checkbox( "센서 이벤트###Sensor events", &enabled ) )
            {
                getWorld().SetShapeSensorEventsEnabled( visual.shapeHandle, enabled );
            }
            if( isSensor )
            {
                ImGui::Text( "센서 겹침 수: %zu", getWorld().GetShapeSensorCapacity( visual.shapeHandle ) );
            }
        }
#pragma endregion Sensors
    }
    ImGui::Spacing();
    ImGui::Separator();
}

void rigidBodyDemoUi::drawSplineEditorControls()
{
    ImGui::BeginDisabled( !canUndoSplineEdit() );
    if( ImGui::Button( "되돌리기###SplineUndo" ) ) undoSplineEdit();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled( !canRedoSplineEdit() );
    if( ImGui::Button( "다시하기###SplineRedo" ) ) redoSplineEdit();
    ImGui::EndDisabled();
    const auto controls = getSplineControlPoints();
    if( controls.empty() ) return;
    if( getTerrainPointDragIndex() ) splineControlIndex_ = *getTerrainPointDragIndex();
    splineControlIndex_ = std::min( splineControlIndex_, controls.size() - 1 );
    char label[32];
    std::snprintf( label, sizeof( label ), "제어점 %zu", splineControlIndex_ + 1 );
    if( ImGui::BeginCombo( "제어점###SplineControl", label ) )
    {
        for( std::size_t i = 0; i < controls.size(); ++i )
        {
            std::snprintf( label, sizeof( label ), "제어점 %zu", i + 1 );
            if( ImGui::Selectable( label, i == splineControlIndex_ ) ) splineControlIndex_ = i;
        }
        ImGui::EndCombo();
    }
    ImGui::TextWrapped( "곡선에서 10px 이내를 Ctrl+왼쪽 클릭하면 클릭 위치에 제어점을 삽입합니다." );
    ImGui::BeginDisabled( getSplineControlPoints().size() <= 2 );
    if( ImGui::Button( "제어점 삭제###SplineRemove" ) && removeSplineControlPoint( splineControlIndex_ ) ) splineControlIndex_ = std::min( splineControlIndex_, getSplineControlPoints().size() - 1 );
    ImGui::EndDisabled();
}

void rigidBodyDemoUi::drawExperimentControls()
{
    if( getKind() == demoKind::windFlow )
    {
        ImGui::Separator();
        ImGui::TextUnformatted( "2D 유체 속도장" );
        bool enabled = isWindSourceEnabled();
        if( ImGui::Checkbox( "바람 소스 활성###WindSourceEnabled", &enabled ) ) setWindSourceEnabled( enabled );

        float strength = getWindSourceStrength();
        if( ImGui::SliderFloat( "바람 세기###WindSourceStrength", &strength, 0.0f, 25.0f, "%.1f m/s²" ) )
            setWindSourceStrength( strength );

        bool couple = isWindObstacleCouplingEnabled();
        if( ImGui::Checkbox( "강체·유체 양방향###WindObstacleCoupling", &couple ) )
            setWindObstacleCouplingEnabled( couple );

        bool conservative = isWindConservativeAdvectionEnabled();
        if( ImGui::Checkbox( "보존형 MAC 이류 (실험)###WindConservativeAdvection", &conservative ) )
            setWindConservativeAdvectionEnabled( conservative );
        if( ImGui::Button( "이류 비교 프리셋 (와류 + 외력/강체 OFF)###WindAdvectionTrial" ) )
            beginWindAdvectionTrial();
        if( ImGui::Button( "현재 흐름에 와류 추가###WindVortex" ) ) addWindVortex();
        if( ImGui::Button( "수동 추적 물질 주입###WindTracerSeed" ) ) seedWindTracer();
        bool sweptTracer = isWindSweptTracerRemapEnabled();
        if( ImGui::Checkbox( "이동 강체의 tracer 재배치 (실험)###WindSweptTracerRemap", &sweptTracer ) )
            setWindSweptTracerRemapEnabled( sweptTracer );

        bool cutCellProjection = isWindCutCellProjectionEnabled();
        if( ImGui::Checkbox( "부분 개방 압력 투영 (실험)###WindCutCellPressure", &cutCellProjection ) )
            setWindCutCellProjectionEnabled( cutCellProjection );
        bool correction = isWindSweptGeometryCorrectionEnabled();
        if( ImGui::Checkbox( "Swept 면 기하 오차 국소 보정 (실험)###WindSweptGeometryCorrection", &correction ) )
            setWindSweptGeometryCorrectionEnabled( correction );

        float drag = getWindDragCoefficient();
        if( ImGui::SliderFloat( "강체 바람 저항 계수###WindBodyDrag", &drag, 0.0f, 15.0f, "%.1f N·s/m" ) )
            setWindDragCoefficient( drag );

        if( const fluidField2* field = getWindField() )
        {
            ImGui::Text( "MAC 격자: %d x %d / 셀 %.2f m", field->getWidth(), field->getHeight(), field->getCellSize() );
            const fluidAdvectionStats2& advection = field->getAdvectionStats();
            ImGui::SeparatorText( "유체 이류 비교 (마지막 Step, 압력 전)" );
            ImGui::Text( "이류: %s / substep %d / 최대 CFL %.3f",
                advection.conservative ? "보존형 Upwind" : "Semi-Lagrangian",
                advection.substeps, advection.maxCfl );
            ImGui::Text( "유효 면 운동량 전: (%.5f, %.5f) N·s",
                advection.before.x, advection.before.y );
            ImGui::Text( "유효 면 운동량 후: (%.5f, %.5f) N·s",
                advection.after.x, advection.after.y );
            ImGui::Text( "이류만의 운동량 변화: (%.7f, %.7f) N·s",
                advection.delta().x, advection.delta().y );
            const fluidTracerStats2& tracer = field->getTracerStats();
            ImGui::SeparatorText( "수동 추적 물질 (Part 12a)" );
            ImGui::Text( "총 tracer 질량: %.6f kg / 막힌 셀 보관: %.6f kg",
                field->getTracerMass(), tracer.trappedMass );
            ImGui::Text( "마지막 수송 전/후: %.6f / %.6f kg (변화 %+.8f kg)",
                tracer.beforeMass, tracer.afterMass, tracer.delta() );
            ImGui::Text( "tracer CFL: %.3f / substep: %d%s",
                tracer.maxCfl, tracer.substeps, tracer.cflLimited ? " (상한 도달)" : "" );
            ImGui::Text( "이동 강체 remap: %s / 직접 %.6f kg / 최근접 %.6f kg",
                tracer.usedSweptRemap ? "적용" : "미적용",
                tracer.sweptRemappedMass, tracer.fallbackRemappedMass );
            ImGui::Text( "Swept 통과 체적 기반 요청/제한: %.6f / %.6f kg",
                tracer.sweptRequestedMass, tracer.sweptLimitedMass );
            ImGui::Text( "재배치하지 못한 감소 체적의 tracer: %.6f kg",
                tracer.unresolvedDisplacementMass );
            ImGui::TextDisabled( "Swept 체적 × 이전 농도 (표본 근사), 정확한 표면 수송은 아님" );
            if( advection.cflLimited )
                ImGui::TextColored( ImVec4{ 1.0f, 0.4f, 0.2f, 1.0f },
                    "CFL substep 상한 도달: 정확도/안정성 저하 가능" );
            if( !field->isCutCellProjectionEnabled() )
                ImGui::Text( "이진 셀 발산 RMS: %.5f /s", field->getDivergenceRms() );
            else
            {
                const fluidCutCellPressure2& projection = field->getCutCellPressure();
                ImGui::Text( "체적 포함 연속 방정식 잔차 RMS: %.5f -> %.5f /s",
                    projection.divergenceBefore, projection.divergenceAfter );
                ImGui::Text( "가중 압력 잔차: %.6f / 순 체적 불일치: %+.6f m³/s",
                    projection.pressureResidual, projection.sourceImbalance );
                ImGui::Text( "체적 변화율: %+.5f m³/s (이력 %s)",
                    projection.volumeRate, projection.hasVolumeHistory ? "있음" : "초기 snapshot" );
                ImGui::Text( "완전 점유 셀의 전달량/미해결량: %+.5f / %+.5f m³/s",
                    projection.redistributedVolumeRate, projection.unresolvedVolumeRate );
                ImGui::Text( "영역별 정합 보정: %.6f m³/s / 유체 영역: %d",
                    projection.compatibilityCorrection, projection.connectedRegions );
                const fluidSweptFaceFlux2& swept = field->getSweptFaceFlux();
                ImGui::Text( "Solid 시간 적분: %s / 구간 %d",
                    swept.usedSweptFlux ? "Swept face flux" : "Snapshot 변화", swept.timeSamples );
                ImGui::Text( "통과한 MAC 면 체적 절댓값 합: %.5f m³", swept.totalAbsoluteFlux );
                ImGui::Text( "Swept 체적 오차 L1 보정 전/후: %.5f / %.5f m³",
                    swept.rawGeometryMismatch, swept.geometryMismatch );
                ImGui::Text( "국소 면 보정 누적량: %.5f m³ / 갱신 횟수: %d",
                    swept.correctionAbsoluteFlux, swept.correctedFaces );
                ImGui::Text( "부분 유체 셀: %d", projection.activeCells );
            }
            ImGui::Text( "강체 점유 셀: %d", field->getSolidCellCount() );
            const fluidSolidTransition2& transition = field->getSolidTransition();
            ImGui::Text( "새 유체 셀/막힌 셀: %d / %d", transition.exposedCells, transition.coveredCells );
            ImGui::Text( "재노출 U/V face: %d / %d", transition.openedUFaces, transition.openedVFaces );
            ImGui::Text( "이웃 복원/0 대체: %d / %d", transition.extrapolatedFaces, transition.fallbackFaces );
            ImGui::Text( "이산 유체 체적 변화: %+.4f m³", transition.fluidVolumeDelta );
            const fluidCutCellTransition2& cut = field->getCutCellTransition();
            ImGui::SeparatorText( "부분 점유 기하 (4x4 표본)" );
            ImGui::Text( "유체 체적: %.4f m³ / 변화 %+.4f m³", cut.fluidVolume, cut.volumeDelta() );
            ImGui::Text( "가린/다시 열린 체적: %.4f / %.4f m³", cut.coveredVolume, cut.releasedVolume );
            ImGui::Text( "부분 셀/개방 U/V face: %d / %d / %d", cut.partialCells, cut.partialUFaces, cut.partialVFaces );
            const vec2 dragBody = getWindDragBodyImpulse();
            const vec2 dragFluid = getWindDragFluidImpulse();
            ImGui::Text( "Drag Δp 강체: (%.5f, %.5f) N·s", dragBody.x, dragBody.y );
            ImGui::Text( "Drag Δp 유체: (%.5f, %.5f) N·s", dragFluid.x, dragFluid.y );
            ImGui::Text( "Drag 교환 잔차: %.7f N·s", Length( getWindDragExchangeResidual() ) );
            const vec2 momentum = field->getFluidMomentum();
            ImGui::Text( "유체 내부 face 운동량: (%.4f, %.4f) N·s", momentum.x, momentum.y );
            const fluidPressureMomentum2& pressure = field->getPressureMomentum();
            const fluidStepMomentum2& stages = field->getStepMomentum();
            if( !field->isCutCellProjectionEnabled() )
            {
                ImGui::SeparatorText( "압력 투영 운동량 수지 (마지막 Step)" );
                ImGui::Text( "유체 Δp: (%.5f, %.5f) N·s", pressure.fluid.x, pressure.fluid.y );
                ImGui::Text( "강체 Δp: (%.5f, %.5f) N·s", pressure.solid.x, pressure.solid.y );
                ImGui::Text( "외벽 Δp: (%.5f, %.5f) N·s", pressure.wall.x, pressure.wall.y );
                ImGui::Text( "압력 수지 잔차: %.7f N·s", Length( pressure.residual() ) );
            }
            else ImGui::TextWrapped( "실험 모드는 이진 MAC 압력 운동량 장부를 지원하지 않습니다." );
            ImGui::Text( "점유 변경 Δp: (%.5f, %.5f) N·s", stages.occupancyDelta().x, stages.occupancyDelta().y );
            ImGui::Text( "이류 Δp: (%.5f, %.5f) N·s", stages.advectionDelta().x, stages.advectionDelta().y );
            for( std::size_t i = 0; i < getWindPressureForces().size(); ++i )
                ImGui::Text( "압력 반력 %zu: (%.2f, %.2f) N", i + 1,
                    getWindPressureForces()[i].force.x, getWindPressureForces()[i].force.y );
        }
        if( ImGui::Button( "유체·물체 초기화###WindReset" ) ) resetWindFlow();
        ImGui::TextWrapped( "파란 화살표는 유체 속도, 주황색 점은 강체가 차지한 셀입니다. 원·캡슐·볼록 다각형은 실제 회전한 도형으로 셀을 점유합니다. 양방향 모드에서는 해당 경계의 수직 속도를 유체에 적용하고 압력 반력을 강체에 전달합니다. Drag는 유체 내부 face의 유효 질량 기준으로 실제 전달한 충격량만 강체에 반대로 적용합니다. Drag 교환 잔차와 압력(유체·강체·외벽) 수지 잔차를 별도로 표시합니다. 점유 변경 때 새로 열린 유체 면은 이전의 순수 유체 이웃 속도로 복원합니다. 셀 중심 마스크 외에도 4x4 표본으로 부분 점유와 면 개방율을 측정합니다. 부분 점유 체적 변동은 진단값입니다. 실험 모드에서는 개방면 유체 유량과 이전/현재 강체 이동·회전 경로의 MAC 면 통과 체적을 압력 방정식에 사용합니다. 경로가 없거나 도메인 벽을 통과하면 점유율 차이로 돌아갑니다. 첫 snapshot은 이동으로 취급하지 않고, 완전히 가려진 셀의 체적 소스는 근접 유체 셀로 이동합니다. 영역별로 풀 수 없는 체적 변동도 표시합니다. 보존형 MAC 이류 실험 모드에서는 고정된 면 개방율의 dual volume upwind flux를 사용하고 이류 단계 자체의 운동량을 비교할 수 있습니다. 정확한 moving interface·보존형 전체 유체 질량·표면 압력 적분·전체 운동량 보존은 아직 지원하지 않습니다. 점유 변경과 이류 단계의 운동량 변화도 따로 기록합니다. 이 압력 수지의 작은 잔차는 이산화된 face 회계이며 전체 유체·강체 운동량 보존을 뜻하지 않습니다." );
        return;
    }
    if( getKind() == demoKind::revoluteLimitMotorComparison )
    {
        ImGui::TextUnformatted( "회전 각도 제한·모터 비교" );
        ImGui::TextWrapped( "같은 막대의 초기 각도 0°·목표 각속도 2rad/s·토크 한도 0.2N·m는 같고 각도 제한만 A 꺼짐, B -45°~45° 켜짐입니다. 중력·감쇠는 끕니다. 양수 목표는 상한, 음수 목표는 하한에서 멈춥니다. 경계에서 목표 부호를 바꾸면 반대 방향으로 출발합니다. 초록 선은 하한, 빨강 선은 상한입니다. 정지 → 시험 복귀 → 재생으로 처음 1초를 비교하세요." );
        const auto joints = getAngleLimitComparisonJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getRevoluteJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            bool enableLimit = joint.enableLimit;
            float lower = joint.lowerAngle * 180.0f / std::numbers::pi_v<float>, upper = joint.upperAngle * 180.0f / std::numbers::pi_v<float>;
            bool limitChanged = ImGui::Checkbox( "각도 제한###Angle limit enabled", &enableLimit );
            limitChanged |= ImGui::SliderFloat( "하한###Angle lower limit", &lower, -170.0f, upper, "%.1f deg", ImGuiSliderFlags_AlwaysClamp );
            limitChanged |= ImGui::SliderFloat( "상한###Angle upper limit", &upper, lower, 170.0f, "%.1f deg", ImGuiSliderFlags_AlwaysClamp );
            if( limitChanged ) getWorld().setRevoluteJointLimit( joints[i], enableLimit, lower * std::numbers::pi_v<float> / 180.0f, upper * std::numbers::pi_v<float> / 180.0f );
            float speed = joint.motorSpeed;
            bool changed = ImGui::SliderFloat( "목표 각속도###Motor target angular speed", &speed, -4.0f, 4.0f, "%.2f rad/s", ImGuiSliderFlags_AlwaysClamp );
            if( changed ) getWorld().setRevoluteJointMotor( joints[i], true, speed, joint.maxMotorTorque );
            ImGui::TextWrapped( "현재 각도 %.2f° / 각속도 %.3f rad/s", joint.currentAngle * 180.0f / std::numbers::pi_v<float>, getWorld().GetBodyAngularVelocity( joint.bodyB ) );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "현재 각도는 기준각 0°에 대한 -180°~180° 값이며 회전 횟수를 누적하지 않습니다. 하한과 상한이 같으면 해당 각도로 고정합니다. 시험 복귀는 초기 위치·회전·정지 속도를 복원하고 각도 제한·목표 각속도·물성·선택·재생 상태를 유지합니다. 상단 초기화는 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::suspensionLimitComparison )
    {
        ImGui::TextUnformatted( "서스펜션·이동 제한 비교" );
        ImGui::TextWrapped( "같은 바퀴에 0.5Hz·감쇠비 0.7을 적용하고 이동 제한만 A 꺼짐, B -0.4~0.4m 켜짐으로 비교합니다. 기본 중력에서는 하한에서 멈춥니다. 월드 중력을 (0, 9.8)로 바꾸면 상한 정지를 확인할 수 있습니다. 모터·물체 감쇠는 끄고 초기 각속도 2rad/s로 자유 회전합니다. 초록 표시는 하한, 빨강 표시는 상한입니다. 편집 후 정지 → 시험 복귀 → 재생으로 같은 조건을 비교하세요." );
        const auto joints = getSuspensionLimitJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getWheelJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            bool enableLimit = joint.enableLimit;
            float lower = joint.lowerTranslation, upper = joint.upperTranslation;
            bool limitChanged = ImGui::Checkbox( "이동 제한###Suspension limit enabled", &enableLimit );
            limitChanged |= ImGui::SliderFloat( "하한###Suspension lower limit", &lower, -2.0f, upper, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
            limitChanged |= ImGui::SliderFloat( "상한###Suspension upper limit", &upper, lower, 2.0f, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
            if( limitChanged ) getWorld().setWheelJointLimit( joints[i], enableLimit, lower, upper );
            ImGui::TextWrapped( "축 변위 %.3f m / 스프링 반력 %.3f N", joint.currentTranslation, joint.springForce );
            ImGui::TextWrapped( "제한 반력 %.3f N / 각속도 %.3f rad/s", joint.limitForce, getWorld().GetBodyAngularVelocity( joint.bodyB ) );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "음수 변위는 아래 방향입니다. 하한과 상한이 같으면 해당 변위를 유지하며 회전은 자유롭습니다. 시험 복귀는 초기 위치·회전·선속도 0·각속도 2rad/s를 복원하고 이동 제한·물성·중력·선택·재생 상태를 유지합니다. 상단 초기화는 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::distanceLimitMotorComparison )
    {
        ImGui::TextUnformatted( "거리 제한·모터 비교" );
        ImGui::TextWrapped( "초기 길이 1m·목표 속도 1m/s·힘 한도 2N은 같고 거리 제한만 A 꺼짐, B 켜짐입니다. 중력·물체 감쇠·스프링 힘은 끕니다. 양수 속도는 상한, 음수 속도는 하한에서 멈춥니다. 목표 속도 부호를 바꾸면 경계에서 반대 방향으로 출발합니다. 초록 원은 하한, 빨강 원은 상한입니다. 정지 → 시험 복귀 → 재생으로 처음 1초를 비교하세요." );
        const auto joints = getLimitComparisonJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getDistanceJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            bool enableLimit = joint.enableLimit;
            float lower = joint.minLength, upper = joint.maxLength;
            bool limitChanged = ImGui::Checkbox( "거리 제한###Distance limit enabled", &enableLimit );
            limitChanged |= ImGui::SliderFloat( "하한###Distance lower limit", &lower, 0.1f, upper, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
            limitChanged |= ImGui::SliderFloat( "상한###Distance upper limit", &upper, lower, 3.0f, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
            if( limitChanged ) getWorld().setDistanceJointLimit( joints[i], enableLimit, lower, upper );
            float speed = joint.motorSpeed;
            bool changed = ImGui::SliderFloat( "목표 속도###Motor target speed", &speed, -3.0f, 3.0f, "%.2f m/s", ImGuiSliderFlags_AlwaysClamp );
            if( changed ) getWorld().setDistanceJointMotor( joints[i], true, speed, joint.maxMotorForce );
            const float axialSpeed = joint.currentLength > 0.0f ? Dot( getWorld().GetBodyLinearVelocity( joint.bodyB ), ( joint.anchorB - joint.anchorA ) / joint.currentLength ) : 0.0f;
            ImGui::TextWrapped( "현재 길이 %.3f m / 축 방향 속도 %.3f m/s", joint.currentLength, axialSpeed );
            ImGui::TextWrapped( "실제 모터 힘 %.3f N", joint.motorForce );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "하한과 상한이 같으면 기존 정책에 따라 목표 길이 1m의 고정 거리 제약으로 돌아갑니다. 초기 위치·회전·정지 속도를 복원합니다. 거리 제한·목표 속도·물성·선택·재생 상태는 유지합니다. 상단 초기화는 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::weldElasticityComparison )
    {
        ImGui::TextUnformatted( "웰드·선형/회전 탄성 비교" );
        ImGui::TextWrapped( "같은 막대를 한쪽 끝에서 지지하고 중력을 적용합니다. A는 강체 연결, B는 선형만 1Hz, C는 회전만 1Hz이며 감쇠비는 모두 0.7입니다. 0Hz는 해당 방향의 강한 제약이고 양수는 탄성입니다. 선형 탄성은 연결점이 처지고 회전 탄성은 막대가 기울어집니다. 주파수 편집 후 정지 → 시험 복귀 → 재생으로 같은 조건을 비교하세요." );
        const auto joints = getWeldComparisonJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getWeldJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            float linearHertz = joint.linearHertz, angularHertz = joint.angularHertz;
            if( ImGui::SliderFloat( "선형 주파수###Weld linear frequency", &linearHertz, 0.0f, 10.0f, "%.2f Hz", ImGuiSliderFlags_AlwaysClamp ) ) getWorld().setWeldJointLinearTuning( joints[i], linearHertz, joint.linearDampingRatio );
            if( ImGui::SliderFloat( "회전 주파수###Weld angular frequency", &angularHertz, 0.0f, 10.0f, "%.2f Hz", ImGuiSliderFlags_AlwaysClamp ) ) getWorld().setWeldJointAngularTuning( joints[i], angularHertz, joint.angularDampingRatio );
            ImGui::TextWrapped( "연결점 오차 %.3f m / 상대 각도 %.2f°", Length( joint.anchorB - joint.anchorA ), joint.currentAngle * 180.0f / std::numbers::pi_v<float> );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "초기 위치·회전·정지 속도를 복원합니다. 선형·회전 주파수·물성·선택·재생 상태는 유지합니다. 청록 점은 고정점, 보라색 점은 막대의 연결점입니다. 상단 초기화는 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::suspensionMotorComparison )
    {
        ImGui::TextUnformatted( "서스펜션·회전 모터 비교" );
        ImGui::TextWrapped( "같은 바퀴와 1Hz·감쇠비 0.7의 스프링에서 모터만 A 꺼짐, B 켜짐으로 비교합니다. 초기 각속도는 0이며 목표 4rad/s·토크 한도 0.05N·m로 시작합니다. 중력은 적용하고 이동 제한·물체 감쇠는 끕니다. 회전은 달라도 기본 스프링의 처짐은 같습니다. 음수 목표로 방향 반전, 0으로 제동을 확인하세요. 모터 해제나 토크 0은 이미 회전 중인 바퀴를 제동하지 않습니다." );
        const auto joints = getSuspensionMotorJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getWheelJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            float speed = joint.motorSpeed, maxTorque = joint.maxMotorTorque;
            bool enableMotor = joint.enableMotor;
            bool changed = ImGui::Checkbox( "회전 모터###Suspension motor enabled", &enableMotor );
            changed |= ImGui::SliderFloat( "목표 각속도###Suspension motor speed", &speed, -4.0f, 4.0f, "%.2f rad/s", ImGuiSliderFlags_AlwaysClamp );
            changed |= ImGui::SliderFloat( "토크 한도###Suspension motor torque", &maxTorque, 0.0f, 0.2f, "%.3f N*m", ImGuiSliderFlags_AlwaysClamp );
            if( changed ) getWorld().setWheelJointMotor( joints[i], enableMotor, speed, maxTorque );
            ImGui::TextWrapped( "축 변위 %.3f m", joint.currentTranslation );
            ImGui::TextWrapped( "현재 각속도 %.3f rad/s / 실제 모터 토크 %.3f N·m", getWorld().GetBodyAngularVelocity( joint.bodyB ), joint.motorTorque );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "초기 위치·회전·정지 속도를 복원합니다. 모터 활성화·목표 각속도·토크 한도·물성·선택·재생 상태는 유지합니다. 상단 초기화는 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::revoluteMotorTorqueComparison )
    {
        ImGui::TextUnformatted( "회전 모터·토크 한도 비교" );
        ImGui::TextWrapped( "같은 막대를 중심 회전축에 연결하고 목표 각속도 2rad/s에서 토크 한도 A 0, B 0.01, C 0.2N·m를 비교합니다. 중력·감쇠·각도 제한은 끕니다. 토크 한도는 가속·제동 능력을 제한합니다. 음수 목표는 방향 반전, 0은 제동이며 토크 한도가 0이면 제동하지 않습니다. 정지 → 시험 복귀 → 재생으로 처음 1초를 비교하세요." );
        const auto joints = getTorqueComparisonJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getRevoluteJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            float speed = joint.motorSpeed, maxTorque = joint.maxMotorTorque;
            bool changed = ImGui::SliderFloat( "목표 각속도###Motor target angular speed", &speed, -4.0f, 4.0f, "%.2f rad/s", ImGuiSliderFlags_AlwaysClamp );
            changed |= ImGui::SliderFloat( "토크 한도###Motor torque limit", &maxTorque, 0.0f, 1.0f, "%.3f N*m", ImGuiSliderFlags_AlwaysClamp );
            if( changed ) getWorld().setRevoluteJointMotor( joints[i], true, speed, maxTorque );
            ImGui::TextWrapped( "현재 각속도 %.3f rad/s / 실제 모터 토크 %.3f N·m", getWorld().GetBodyAngularVelocity( joint.bodyB ), joint.motorTorque );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "초기 위치·회전·정지 속도를 복원합니다. 목표 각속도·토크 한도·물성·선택·재생 상태는 유지합니다. 상단 초기화는 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::distanceMotorForceComparison )
    {
        ImGui::TextUnformatted( "거리 모터·힘 한도 비교" );
        ImGui::TextWrapped( "초기 목표 속도는 모두 거리 증가 방향 1m/s이며 힘 한도만 A 0, B 0.05, C 2N입니다. 중력·물체 감쇠·스프링 힘·거리 제한은 끕니다. 힘 한도는 가속 능력을 제한하며 충분한 힘이 있으면 목표 속도를 따라갑니다. 설정을 편집하고 정지 → 시험 복귀 → 재생으로 처음 1초를 비교하세요. 음수 속도는 거리를 줄입니다." );
        const auto joints = getMotorComparisonJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getDistanceJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            float speed = joint.motorSpeed, maxForce = joint.maxMotorForce;
            bool changed = ImGui::SliderFloat( "목표 속도###Motor target speed", &speed, -3.0f, 3.0f, "%.2f m/s", ImGuiSliderFlags_AlwaysClamp );
            changed |= ImGui::SliderFloat( "힘 한도###Motor force limit", &maxForce, 0.0f, 5.0f, "%.2f N", ImGuiSliderFlags_AlwaysClamp );
            if( changed ) getWorld().setDistanceJointMotor( joints[i], true, speed, maxForce );
            const float axialSpeed = joint.currentLength > 0.0f ? Dot( getWorld().GetBodyLinearVelocity( joint.bodyB ), ( joint.anchorB - joint.anchorA ) / joint.currentLength ) : 0.0f;
            ImGui::TextWrapped( "현재 길이 %.3f m / 축 방향 속도 %.3f m/s", joint.currentLength, axialSpeed );
            ImGui::TextWrapped( "실제 모터 힘 %.3f N", joint.motorForce );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "초기 위치·회전·정지 속도를 복원합니다. 목표 속도·힘 한도·물성·선택·재생 상태는 유지합니다. 상단 초기화는 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::suspensionLoadComparison )
    {
        ImGui::TextUnformatted( "서스펜션·하중 비교" );
        ImGui::TextWrapped( "같은 바퀴에 주파수 A 0.5, B 1, C 2Hz·감쇠비 0.7을 적용합니다. 중력으로 처진 뒤 수렴하는 차이를 비교하세요. 모터·이동 제한·물체 감쇠는 끄고 초기 각속도 2rad/s로 자유 회전합니다. 청록 화살표는 이동축, 점은 변위 0 기준입니다. 주파수·감쇠비 편집 후 정지 → 시험 복귀 → 재생으로 같은 출발 조건을 비교하세요. 0Hz는 스프링 힘을 꺼 중력 아래에서 낙하합니다." );
        const auto joints = getSuspensionComparisonJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getWheelJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            float hertz = joint.hertz, dampingRatio = joint.dampingRatio;
            bool changed = ImGui::SliderFloat( "스프링 주파수###Spring Hertz", &hertz, 0.0f, 30.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
            changed |= ImGui::SliderFloat( "감쇠비###Spring damping", &dampingRatio, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
            if( changed ) getWorld().setWheelJointSpring( joints[i], true, hertz, dampingRatio );
            ImGui::TextWrapped( "축 변위 %.3f m / 스프링 반력 %.3f N", joint.currentTranslation, joint.springForce );
            ImGui::TextWrapped( "세로 속도 %.3f m/s / 각속도 %.3f rad/s", getWorld().GetBodyLinearVelocity( joint.bodyB ).y, getWorld().GetBodyAngularVelocity( joint.bodyB ) );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "초기 위치·회전·선속도 0·각속도 2rad/s를 복원합니다. 주파수·감쇠비·물성·선택·재생 상태는 유지합니다. 상단 초기화는 장면의 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::springDampingComparison )
    {
        ImGui::TextUnformatted( "거리 스프링·감쇠 비교" );
        ImGui::TextWrapped( "초기 목표 길이 1m·초기 길이 1.5m·주파수 2Hz는 같고 감쇠비만 A 0, B 0.7, C 2입니다. 중력과 물체 감쇠는 끕니다. 주파수·감쇠비를 편집하고 정지 → 시험 복귀 → 재생으로 진동과 수렴을 비교하세요. 0Hz는 스프링 힘을 끕니다. 보라색 선은 현재 연결, 원은 목표 거리입니다." );
        const auto joints = getSpringComparisonJoints();
        for( std::size_t i = 0; i < joints.size(); ++i )
        {
            if( !getWorld().IsValid( joints[i] ) ) continue;
            const auto joint = getWorld().getDistanceJointData( joints[i] );
            ImGui::PushID( static_cast<int>( i ) );
            ImGui::TextUnformatted( getShapes()[i].label );
            float hertz = joint.hertz, dampingRatio = joint.dampingRatio;
            bool changed = ImGui::SliderFloat( "스프링 주파수###Spring Hertz", &hertz, 0.0f, 30.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
            changed |= ImGui::SliderFloat( "감쇠비###Spring damping", &dampingRatio, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
            if( changed ) getWorld().setDistanceJointSpring( joints[i], true, hertz, dampingRatio );
            ImGui::TextWrapped( "현재 길이 %.3f m / 세로 속도 %.3f m/s", joint.currentLength, getWorld().GetBodyLinearVelocity( joint.bodyB ).y );
            ImGui::PopID();
        }
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "초기 위치·회전·정지 속도를 복원합니다. 주파수·감쇠비·물성·선택·재생 상태는 유지합니다. 상단 초기화는 장면의 기본 설정으로 돌아갑니다." );
        return;
    }
    if( getKind() == demoKind::collisionComparison || getKind() == demoKind::frictionComparison || getKind() == demoKind::restitutionComparison || getKind() == demoKind::boxStack || getKind() == demoKind::ccdComparison || getKind() == demoKind::collisionFilterComparison || getKind() == demoKind::sensorWallComparison || getKind() == demoKind::sleepWakeComparison || getKind() == demoKind::dampingComparison || getKind() == demoKind::gravityScaleComparison || getKind() == demoKind::massImpulseComparison || getKind() == demoKind::impulsePointComparison || getKind() == demoKind::rotationalInertiaComparison || getKind() == demoKind::forceImpulseComparison || getKind() == demoKind::torqueImpulseComparison || getKind() == demoKind::dynamicKinematicComparison || getKind() == demoKind::compoundCenterComparison || getKind() == demoKind::fastRotationComparison || getKind() == demoKind::maximumLinearSpeedComparison || getKind() == demoKind::frictionMixingComparison ||
        getKind() == demoKind::restitutionMixingComparison || getKind() == demoKind::lowSpeedRestitutionComparison || getKind() == demoKind::collisionGroupComparison || getKind() == demoKind::movingPlatformFrictionComparison || getKind() == demoKind::rollingSlidingFrictionComparison || getKind() == demoKind::frictionForceComparison || getKind() == demoKind::obliqueImpactFrictionComparison || getKind() == demoKind::collisionMassRatioComparison || getKind() == demoKind::centralOffsetImpactComparison )
    {
        ImGui::TextUnformatted( "반복 시험" );
        if( getKind() == demoKind::ccdComparison )
        {
            ImGui::Text( "현재 월드 CCD 검사: %s", getWorld().IsContinuousEnabled() ? "켜짐" : "꺼짐" );
            ImGui::TextWrapped( "정지 → 시험 복귀 → 1프레임 순서로 비교하세요. 월드 설정의 CCD 검사를 바꿉니다. 시작 속도는 100m/s이며 복귀하면 같은 조건으로 돌아갑니다. Bullet 해제만으로 정적 벽 CCD가 꺼지지는 않습니다." );
        }
        if( getKind() == demoKind::collisionFilterComparison ) ImGui::TextWrapped( "처음에는 위쪽이 충돌하고 아래쪽은 통과합니다. 아래 A의 충돌 마스크 → 개별 추가 제한에서 충돌 대상 레이어 02를 켜고 시험 복귀로 비교하세요. 양쪽 마스크가 서로를 허용해야 하며 공통 충돌 설정도 함께 적용됩니다. 이름의 레이어는 초기값입니다." );
        if( getKind() == demoKind::rollingSlidingFrictionComparison )
        {
            ImGui::TextWrapped( "두 원은 처음에 오른쪽 4m/s·각속도 0으로 출발합니다. A는 마찰로 감속하면서 시계 방향 회전을 얻고, B는 원 마찰 0이라 회전 없이 미끄러집니다. 정지 → 시험 복귀 → 재생으로 처음 1초를 비교하세요. 충돌 재질에서 바닥·원 마찰을 편집할 수 있습니다. 수평 바닥에서 미끄럼 속도가 0에 가까우면 미끄럼 없는 구름입니다. 감쇠가 없으므로 구름 자체를 임의로 멈추지는 않습니다." );
            for( std::size_t i = 0; i < 2; ++i )
            {
                const auto& floor = getShapes()[2 * i];
                const auto& circle = getShapes()[2 * i + 1];
                const float speed = getWorld().GetBodyLinearVelocity( circle.bodyHandle ).x;
                const float angularVelocity = getWorld().GetBodyAngularVelocity( circle.bodyHandle );
                ImGui::TextWrapped( "%s / %s: 마찰 %.2f / %.2f", floor.label, circle.label, getWorld().GetShapeFriction( floor.shapeHandle ), getWorld().GetShapeFriction( circle.shapeHandle ) );
                ImGui::TextWrapped( "가로 속도 %.2f m/s / 각속도 %.2f rad/s", speed, angularVelocity );
                ImGui::TextWrapped( "수평 바닥 기준 미끄럼 속도: %.2f m/s", speed + 0.4f * angularVelocity );
            }
        }
        if( getKind() == demoKind::movingPlatformFrictionComparison )
        {
            ImGui::TextWrapped( "두 발판은 처음에 오른쪽 2m/s로 움직이고 상자는 정지해 있습니다. A는 마찰 0.8로 상자를 운반하고 B는 상자 마찰 0이라 아래로 미끄러져 지나갑니다. 정지 → 시험 복귀 → 재생으로 처음 1초를 비교하세요. 오브젝트의 충돌 재질에서 발판·상자 마찰을 편집할 수 있습니다. 키네마틱 발판은 접촉 힘으로 속도가 바뀌지 않습니다. 발판은 유한하므로 오래 진행하면 상자가 끝에서 떨어질 수 있습니다." );
            for( std::size_t i = 0; i < 2; ++i )
            {
                const auto& platform = getShapes()[2 * i];
                const auto& box = getShapes()[2 * i + 1];
                const float platformSpeed = getWorld().GetBodyLinearVelocity( platform.bodyHandle ).x;
                const float boxSpeed = getWorld().GetBodyLinearVelocity( box.bodyHandle ).x;
                ImGui::TextWrapped( "%s / %s: 마찰 %.2f / %.2f", platform.label, box.label, getWorld().GetShapeFriction( platform.shapeHandle ), getWorld().GetShapeFriction( box.shapeHandle ) );
                ImGui::TextWrapped( "가로 속도: 발판 %.2f / 상자 %.2f m/s", platformSpeed, boxSpeed );
                ImGui::TextWrapped( "상자 - 발판 상대속도: %.2f m/s", boxSpeed - platformSpeed );
            }
        }
        if( getKind() == demoKind::collisionGroupComparison )
        {
            ImGui::TextWrapped( "초기 A는 그룹 0·마스크 차단으로 통과, B는 같은 양수 그룹으로 마스크를 무시하고 충돌, C는 같은 음수 그룹으로 마스크가 허용해도 통과합니다. 공통 충돌 설정의 차단은 양수 그룹보다 우선합니다. 각 원의 충돌 마스크 → 개별 추가 제한에서 그룹·마스크를 편집하고 정지 → 시험 복귀 → 재생으로 비교하세요. 같은 그룹이 아니거나 그룹이 0이면 양쪽 마스크가 서로를 허용해야 합니다." );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const auto& left = getShapes()[2 * i];
                const auto& right = getShapes()[2 * i + 1];
                const auto a = getWorld().GetShapeFilter( left.shapeHandle );
                const auto b = getWorld().GetShapeFilter( right.shapeHandle );
                ImGui::TextWrapped( "%s / %s: 그룹 %d / %d", left.label, right.label, a.groupIndex, b.groupIndex );
                ImGui::TextWrapped( "개별 필터: %s / 공통 설정: %s", ShouldShapesCollide( a, b ) ? "허용" : "차단", getWorld().getCollisionMatrix().allows( a.categoryBits, b.categoryBits ) ? "허용" : "차단" );
                ImGui::TextWrapped( "가로 속도: %.2f / %.2f m/s", getWorld().GetBodyLinearVelocity( left.bodyHandle ).x, getWorld().GetBodyLinearVelocity( right.bodyHandle ).x );
            }
        }
        if( getKind() == demoKind::sensorWallComparison )
        {
            ImGui::TextWrapped( "위쪽 일반 벽은 원을 막고 보라색 센서 영역은 통과시킵니다. 센서와 아래 원의 센서 이벤트가 모두 켜져야 감지합니다. 이벤트·마스크 변경은 다음 진행 단계부터 반영됩니다. 시험 복귀는 편집값을 유지하고 이전 겹침을 정리합니다." );
            for( const auto& visual : getShapes() )
                if( getWorld().IsShapeSensor( visual.shapeHandle ) ) ImGui::Text( "센서 겹침 수: %zu", getWorld().GetShapeSensorCapacity( visual.shapeHandle ) );
        }
        if( getKind() == demoKind::sleepWakeComparison )
        {
            ImGui::TextWrapped( "처음에는 왼쪽만 자동 수면을 허용합니다. 안정된 뒤 색상과 현재 상태를 비교하세요. 인스펙터의 Sleep과 CCD에서 수면 허용·속도 기준을 바꾸고 시험 복귀로 반복합니다. 왼쪽 드래그나 Awake로 깨울 수 있습니다. 월드 수면 설정도 함께 적용됩니다." );
            for( const auto& visual : getShapes() )
                if( getWorld().GetBody( visual.bodyHandle ).type == bodyType::Dynamic ) ImGui::TextWrapped( "%s: %s", visual.label, getWorld().IsBodyAwake( visual.bodyHandle ) ? "깨어 있음" : "수면 중" );
        }
        if( getKind() == demoKind::dampingComparison )
        {
            ImGui::TextWrapped( "같은 2m/s·2rad/s로 출발하며 초기 선형·회전 감쇠는 0·0.5·2입니다. 물리량에서 두 감쇠를 각각 편집하고 시험 복귀로 비교하세요. 처음에는 중력과 자동 수면을 끕니다. 이름의 감쇠는 초기값이며 편집값은 복귀해도 유지됩니다." );
            for( const auto& visual : getShapes() )
                ImGui::TextWrapped( "%s: 속력 %.2f m/s / 각속도 %.2f rad/s", visual.label, Length( getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ), getWorld().GetBodyAngularVelocity( visual.bodyHandle ) );
        }
        if( getKind() == demoKind::gravityScaleComparison )
        {
            ImGui::TextWrapped( "같은 높이에서 정지 상태로 출발하며 초기 중력 배율은 0·0.5·1입니다. 물리량의 중력 배율이나 월드 설정의 중력을 편집하고 시험 복귀로 비교하세요. 음수 배율은 월드 중력과 반대로 움직입니다. 처음에는 감쇠와 자동 수면을 끕니다. 이름의 배율은 초기값입니다." );
            const vec2 gravity = getWorld().GetGravity();
            ImGui::Text( "현재 월드 중력: (%.2f, %.2f) m/s²", gravity.x, gravity.y );
            for( const auto& visual : getShapes() )
                ImGui::TextWrapped( "%s: 높이 %.2f m / 세로 속도 %.2f m/s", visual.label, getWorld().GetBodyTransform( visual.bodyHandle ).position.y, getWorld().GetBodyLinearVelocity( visual.bodyHandle ).y );
        }
        if( getKind() == demoKind::massImpulseComparison )
        {
            ImGui::TextWrapped( "같은 크기의 상자에 중심 충격량 2N·s를 위쪽으로 추가합니다. 밀도를 편집하고 시험 복귀 → 같은 충격량 적용으로 재비교하세요. 다시 누르면 누적됩니다. 정지 중에도 속도는 즉시 바뀌며 이동은 재생·1프레임부터 진행합니다. 처음에는 중력·감쇠·자동 수면을 끕니다. 이름의 밀도는 초기값입니다." );
            if( ImGui::Button( "같은 충격량 적용###MassTrialImpulse" ) ) applyMassTrialImpulse();
            for( const auto& visual : getShapes() )
                ImGui::TextWrapped( "%s: 질량 %.2f kg / 속력 %.2f m/s", visual.label, getWorld().GetBodyMass( visual.bodyHandle ), Length( getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ) );
        }
        if( getKind() == demoKind::impulsePointComparison )
        {
            ImGui::TextWrapped( "주황색 점에 월드 위쪽 2N·s를 추가합니다. 기본 조건에서는 선속도 증가가 같고 좌우 작용점은 반대로 회전합니다. 점은 물체와 함께 회전합니다. 회전을 편집한 뒤 그대로 적용해 비교하세요. 반복 적용은 누적되며 시험 복귀는 초기 회전·정지 속도로 돌아가고 밀도 등 설정은 유지합니다. 처음에는 중력·감쇠·자동 수면을 끕니다." );
            if( ImGui::Button( "같은 충격량 적용###PointTrialImpulse" ) ) applyPointTrialImpulse();
            for( const auto& visual : getShapes() )
                ImGui::TextWrapped( "%s: 속력 %.2f m/s / 각속도 %.2f rad/s", visual.label, Length( getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ), getWorld().GetBodyAngularVelocity( visual.bodyHandle ) );
        }
        if( getKind() == demoKind::rotationalInertiaComparison )
        {
            ImGui::TextWrapped( "초기 질량은 모두 1kg이며 모양에 따라 회전 관성이 다릅니다. 같은 회전 충격량 1N·m·s를 추가하고 각속도를 비교하세요. 선속도는 바뀌지 않습니다. 밀도를 편집하고 시험 복귀 후 다시 적용할 수 있습니다. 처음에는 중력·감쇠·자동 수면을 끕니다." );
            if( ImGui::Button( "같은 회전 충격량 적용###InertiaTrialImpulse" ) ) applyInertiaTrialImpulse();
            for( const auto& visual : getShapes() )
            {
                ImGui::TextWrapped( "%s: 질량 %.2f kg / 회전 관성 %.3f kg·m²", visual.label, getWorld().GetBodyMass( visual.bodyHandle ), getWorld().GetBodyRotationalInertia( visual.bodyHandle ) );
                ImGui::Text( "각속도 %.2f rad/s", getWorld().GetBodyAngularVelocity( visual.bodyHandle ) );
            }
        }
        if( getKind() == demoKind::centralOffsetImpactComparison )
        {
            ImGui::TextWrapped( "같은 원이 같은 상자를 오른쪽 4m/s로 칩니다. A는 중심, B는 중심 위 0.25m, C는 중심 아래 0.25m를 향합니다. 마찰이 없어도 편심 충돌은 상자에 회전을 전달하며 B는 시계 방향, C는 반시계 방향으로 회전합니다. 물리량에서 상자 밀도를 편집하고 정지 → 시험 복귀 → 재생으로 처음 0.75초를 비교하세요. 초기 반발은 1이고 중력·마찰·감쇠·자동 수면은 끕니다." );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const auto& box = getShapes()[2 * i + 1];
                const vec2 velocity = getWorld().GetBodyLinearVelocity( box.bodyHandle );
                ImGui::TextWrapped( "%s: 질량 %.2f kg / 가로 %.2f · 세로 %.2f m/s", box.label, getWorld().GetBodyMass( box.bodyHandle ), velocity.x, velocity.y );
                ImGui::TextWrapped( "각속도 %.2f rad/s", getWorld().GetBodyAngularVelocity( box.bodyHandle ) );
            }
        }
        if( getKind() == demoKind::collisionMassRatioComparison )
        {
            ImGui::TextWrapped( "초기 이동 원:표적의 질량비는 A 1:1, B 1:4, C 4:1입니다. 같은 크기의 원이 4m/s로 정지한 표적을 칩니다. 반발 1·중력·마찰·감쇠 0에서 A는 속도를 전달하고 B는 되튕기며 C는 계속 전진합니다. 물리량에서 이동 원이나 표적의 밀도를 편집하고 정지 → 시험 복귀 → 재생으로 처음 0.75초를 비교하세요. 이름의 A·B·C는 행 구분이며 현재 질량은 아래에 표시합니다." );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const auto& moving = getShapes()[2 * i];
                const auto& target = getShapes()[2 * i + 1];
                ImGui::TextWrapped( "%s / %s: 질량 %.3f / %.3f kg", moving.label, target.label, getWorld().GetBodyMass( moving.bodyHandle ), getWorld().GetBodyMass( target.bodyHandle ) );
                ImGui::TextWrapped( "가로 속도: %.2f / %.2f m/s", getWorld().GetBodyLinearVelocity( moving.bodyHandle ).x, getWorld().GetBodyLinearVelocity( target.bodyHandle ).x );
            }
        }
        if( getKind() == demoKind::obliqueImpactFrictionComparison )
        {
            ImGui::TextWrapped( "두 원은 중력 없이 가로 3m/s·세로 -4m/s·각속도 0으로 출발합니다. 초기 반발은 모두 1이며 A는 마찰 0.8, B는 원 마찰 0입니다. 두 원의 위쪽 반발은 같고 A만 가로 감속과 시계 방향 회전이 생깁니다. 바닥·원의 마찰을 편집하고 정지 → 시험 복귀 → 재생으로 처음 0.6초를 비교하세요. 표시 속도는 수평 바닥의 접선·수직 방향 기준입니다." );
            for( std::size_t i = 0; i < 2; ++i )
            {
                const auto& floor = getShapes()[2 * i];
                const auto& circle = getShapes()[2 * i + 1];
                const vec2 velocity = getWorld().GetBodyLinearVelocity( circle.bodyHandle );
                ImGui::TextWrapped( "%s / %s: 마찰 %.2f / %.2f", floor.label, circle.label, getWorld().GetShapeFriction( floor.shapeHandle ), getWorld().GetShapeFriction( circle.shapeHandle ) );
                ImGui::TextWrapped( "가로 속도 %.2f / 세로 속도 %.2f m/s", velocity.x, velocity.y );
                ImGui::TextWrapped( "각속도 %.2f rad/s", getWorld().GetBodyAngularVelocity( circle.bodyHandle ) );
            }
        }
        if( getKind() == demoKind::frictionForceComparison )
        {
            ImGui::TextWrapped( "초기 질량은 각각 1kg, 바닥·상자 마찰은 모두 0.5입니다. 같은 조건 시험 시작으로 A에 오른쪽 2N, B에 8N을 중심에 1초간 적용합니다. A는 정지를 유지하고 B는 미끄러집니다. 마찰을 편집하고 재시험하세요. 정지 중에는 출발 상태만 복원하며 재생·1프레임 진행부터 힘과 시간이 누적됩니다. 1초 뒤에는 힘이 끝나고 기존 마찰로 움직임이 줄어듭니다." );
            if( ImGui::Button( "같은 조건 시험 시작###FrictionForceStart" ) ) startFrictionForceTrial();
            ImGui::Text( "시험 경과: %.2f s / 힘 적용: %s", getForceTrialTime(), isForceTrialActive() ? "진행 중" : "없음" );
            for( std::size_t i = 0; i < 2; ++i )
            {
                const auto& floor = getShapes()[2 * i];
                const auto& box = getShapes()[2 * i + 1];
                const float force = isForceTrialActive() ? ( i == 0 ? 2.0f : 8.0f ) : 0.0f;
                ImGui::TextWrapped( "%s / %s: 마찰 %.2f / %.2f", floor.label, box.label, getWorld().GetShapeFriction( floor.shapeHandle ), getWorld().GetShapeFriction( box.shapeHandle ) );
                ImGui::TextWrapped( "적용 중인 힘 %.1f N / 질량 %.2f kg / 가로 속도 %.2f m/s", force, getWorld().GetBodyMass( box.bodyHandle ), getWorld().GetBodyLinearVelocity( box.bodyHandle ).x );
            }
        }
        if( getKind() == demoKind::forceImpulseComparison )
        {
            ImGui::TextWrapped( "같은 조건 시험 시작은 편집값을 유지하며 위치·회전·속도를 복원합니다. A에는 위쪽 2N을 1초간, B에는 위쪽 2N·s를 한 번 적용합니다. 정지 중에는 충격량만 즉시 반영되며 재생·1프레임 진행 때 힘과 시간이 누적됩니다. 같은 질량에서 중력·감쇠가 없으면 1초 후 속도가 같습니다." );
            if( ImGui::Button( "같은 조건 시험 시작###ForceTrialStart" ) ) startForceImpulseTrial();
            ImGui::Text( "시험 경과: %.2f s / 힘 적용: %.2f / 1.00 s", getForceTrialTime(), std::min( getForceTrialTime(), 1.0 ) );
            for( std::size_t i = 0; i < getShapes().size(); ++i )
            {
                const auto& visual = getShapes()[i];
                const vec2 start{ i == 0 ? -2.5f : 2.5f, 0 };
                ImGui::TextWrapped( "%s: 질량 %.2f kg / 속력 %.2f m/s", visual.label, getWorld().GetBodyMass( visual.bodyHandle ), Length( getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ) );
                ImGui::Text( "출발점에서 거리 %.2f m", Length( getWorld().GetBodyTransform( visual.bodyHandle ).position - start ) );
            }
        }
        if( getKind() == demoKind::torqueImpulseComparison )
        {
            ImGui::TextWrapped( "같은 조건 시험 시작은 편집값을 유지하며 위치·회전·속도를 복원합니다. A에는 1N·m 토크를 1초간, B에는 1N·m·s 회전 충격량을 한 번 적용합니다. 정지 중에는 회전 충격량만 즉시 반영됩니다. 같은 회전 관성에서 회전 감쇠가 없으면 1초 후 각속도가 같습니다. 밀도·회전 감쇠를 편집하고 재시험하세요." );
            if( ImGui::Button( "같은 조건 시험 시작###TorqueTrialStart" ) ) startTorqueImpulseTrial();
            ImGui::Text( "시험 경과: %.2f s / 토크 적용: %.2f / 1.00 s", getTorqueTrialTime(), std::min( getTorqueTrialTime(), 1.0 ) );
            for( const auto& visual : getShapes() )
            {
                ImGui::TextWrapped( "%s: 질량 %.2f kg / 회전 관성 %.3f kg·m²", visual.label, getWorld().GetBodyMass( visual.bodyHandle ), getWorld().GetBodyRotationalInertia( visual.bodyHandle ) );
                ImGui::TextWrapped( "각속도 %.2f rad/s / 속력 %.2f m/s", getWorld().GetBodyAngularVelocity( visual.bodyHandle ), Length( getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ) );
            }
        }
        if( getKind() == demoKind::dynamicKinematicComparison )
        {
            ImGui::TextWrapped( "위쪽 동적 상자와 아래쪽 키네마틱 상자가 처음에 오른쪽 2m/s로 정지한 동적 표적을 밉니다. 동적 상자는 반작용으로 감속하지만 키네마틱 상자는 지정 속도를 유지합니다. 밀도·반발을 편집하고 정지 → 시험 복귀 → 재생·1프레임으로 비교하세요. 키네마틱의 밀도는 충돌 반작용 질량을 만들지 않습니다." );
            for( std::size_t i = 0; i < getShapes().size(); ++i )
            {
                const auto& visual = getShapes()[i];
                ImGui::TextWrapped( "%s [%s]: 가로 속도 %.2f m/s", visual.label, getBodyTypeName( getWorld().GetBody( visual.bodyHandle ).type ), getWorld().GetBodyLinearVelocity( visual.bodyHandle ).x );
                if( i % 2 == 1 ) ImGui::Text( "표적 출발점에서 거리 %.2f m", Length( getWorld().GetBodyTransform( visual.bodyHandle ).position - vec2{ 0, i < 2 ? 1.5f : -1.5f } ) );
            }
        }
        if( getKind() == demoKind::compoundCenterComparison )
        {
            ImGui::TextWrapped( "각 강체는 원 두 개를 공유하며 초기 밀도는 대칭 2·2, 비대칭 1·3입니다. 전체 질량은 같지만 질량중심은 다릅니다. 주황색 원점에서 위쪽 2N·s를 한 번씩 추가합니다. 분홍색은 질량중심이며 표시 설정에서 켜고 끌 수 있습니다. 오브젝트에서 각 원의 밀도를 편집하고 시험 복귀 후 다시 비교하세요." );
            if( ImGui::Button( "같은 원점 충격량 적용###CompoundTrialImpulse" ) ) applyPointTrialImpulse();
            for( std::size_t i : { 0u, 2u } )
            {
                const auto body = getShapes()[i].bodyHandle;
                const vec2 center = getWorld().GetBodyLocalCenter( body );
                ImGui::TextWrapped( "%s: 질량 %.3f kg / 회전 관성 %.3f kg·m²", i == 0 ? "대칭 강체" : "비대칭 강체", getWorld().GetBodyMass( body ), getWorld().GetBodyRotationalInertia( body ) );
                ImGui::Text( "물체 기준 질량중심: (%.3f, %.3f) m", center.x, center.y );
                ImGui::TextWrapped( "속력 %.2f m/s / 각속도 %.2f rad/s", Length( getWorld().GetBodyLinearVelocity( body ) ), getWorld().GetBodyAngularVelocity( body ) );
            }
        }
        if( getKind() == demoKind::fastRotationComparison )
        {
            ImGui::TextWrapped( "같은 막대가 120rad/s로 출발하며 초기 빠른 회전 허용은 A 꺼짐·B 켜짐입니다. Sleep과 CCD의 빠른 회전 허용을 바꾸고 시험 복귀 → 1프레임으로 비교하세요. 허용을 켜도 이미 줄어든 각속도가 자동 복원되지는 않습니다. 이름은 초기 설정 기준입니다." );
            const float timeStep = getRotationTrialTimeStep();
            if( timeStep > 0.0f )
            {
                ImGui::Text( "마지막 진행 간격: %.6f s", timeStep );
                ImGui::Text( "빠른 회전 비허용 시 제한: %.2f rad/s", MAX_ROTATION / timeStep );
            }
            else
                ImGui::TextUnformatted( "진행 간격·회전 제한: 아직 진행하지 않음" );
            ImGui::TextWrapped( "제한은 전체 진행 간격당 최대 45도이며 내부 분할 수와 무관합니다. 기본 1/60초에서는 약 47.12rad/s입니다. 처음에는 중력·감쇠·자동 수면을 끕니다." );
            for( const auto& visual : getShapes() )
            {
                ImGui::TextWrapped( "%s: 빠른 회전 %s", visual.label, getWorld().IsBodyFastRotationAllowed( visual.bodyHandle ) ? "허용" : "비허용" );
                ImGui::TextWrapped( "각속도 %.2f rad/s / 속력 %.2f m/s", getWorld().GetBodyAngularVelocity( visual.bodyHandle ), Length( getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ) );
            }
        }
        if( getKind() == demoKind::maximumLinearSpeedComparison )
        {
            ImGui::TextWrapped( "같은 대각선 방향의 상자가 2·8m/s로 출발하며 초기 월드 최대 선속도는 4m/s입니다. 제한은 진행 단계에서 속력만 줄이고 방향과 각속도는 유지합니다. 월드 설정의 최대 선속도를 편집하고 시험 복귀 → 1프레임으로 비교하세요. 제한을 높여도 이미 줄어든 속도가 자동 복원되지는 않습니다." );
            ImGui::Text( "현재 월드 최대 선속도: %.2f m/s", getWorld().GetMaximumLinearSpeed() );
            for( const auto& visual : getShapes() )
            {
                const vec2 velocity = getWorld().GetBodyLinearVelocity( visual.bodyHandle );
                ImGui::TextWrapped( "%s: 선속도 (%.2f, %.2f) m/s", visual.label, velocity.x, velocity.y );
                ImGui::TextWrapped( "속력 %.2f m/s / 각속도 %.2f rad/s", Length( velocity ), getWorld().GetBodyAngularVelocity( visual.bodyHandle ) );
            }
        }
        if( getKind() == demoKind::frictionMixingComparison )
        {
            ImGui::TextWrapped( "바닥·상자 마찰의 초기 조합은 A 0.25·1, B 1·0.25, C 1·0입니다. 혼합 계수는 두 마찰 곱의 제곱근이므로 A·B는 같은 감속을 보이고 C는 마찰 없이 미끄러집니다. 오브젝트에서 바닥이나 상자의 마찰을 편집하고 정지 → 시험 복귀 → 재생·1프레임으로 비교하세요. 처음에는 반발·감쇠·자동 수면을 끕니다." );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const auto& floor = getShapes()[2 * i];
                const auto& box = getShapes()[2 * i + 1];
                const float floorFriction = getWorld().GetShapeFriction( floor.shapeHandle );
                const float boxFriction = getWorld().GetShapeFriction( box.shapeHandle );
                ImGui::TextWrapped( "%s / %s: 바닥 %.2f · 상자 %.2f", floor.label, box.label, floorFriction, boxFriction );
                ImGui::TextWrapped( "혼합 계수 %.3f / 속력 %.2f m/s", std::sqrt( floorFriction * boxFriction ), Length( getWorld().GetBodyLinearVelocity( box.bodyHandle ) ) );
            }
        }
        if( getKind() == demoKind::restitutionMixingComparison )
        {
            ImGui::TextWrapped( "바닥·원 반발의 초기 조합은 A 0·0.8, B 0.8·0, C 0·0입니다. 혼합 계수는 두 반발 중 큰 값이므로 A·B는 같은 반발을 보이고 C는 튀지 않습니다. 오브젝트에서 바닥이나 원의 반발을 편집하고 정지 → 시험 복귀 → 재생·1프레임으로 비교하세요. 처음에는 마찰·감쇠·자동 수면을 끕니다." );
            for( std::size_t i = 0; i < 3; ++i )
            {
                const auto& floor = getShapes()[2 * i];
                const auto& circle = getShapes()[2 * i + 1];
                const float floorRestitution = getWorld().GetShapeRestitution( floor.shapeHandle );
                const float circleRestitution = getWorld().GetShapeRestitution( circle.shapeHandle );
                ImGui::TextWrapped( "%s / %s: 바닥 %.2f · 원 %.2f", floor.label, circle.label, floorRestitution, circleRestitution );
                ImGui::TextWrapped( "혼합 계수 %.2f / 중심 높이 %.2f m / 세로 속도 %.2f m/s", std::max( floorRestitution, circleRestitution ), getWorld().GetBodyTransform( circle.bodyHandle ).position.y, getWorld().GetBodyLinearVelocity( circle.bodyHandle ).y );
            }
        }
        if( getKind() == demoKind::lowSpeedRestitutionComparison )
        {
            ImGui::TextWrapped( "초기 속도는 A 0.5m/s, B 2m/s로 바닥을 향합니다. 기존 엔진은 충돌 접근 속도가 1m/s를 넘을 때 반발을 적용하므로 A는 멈추고 B는 튑니다. 처음에는 중력·마찰·감쇠·자동 수면을 끕니다. 반발계수를 편집하고 정지 → 시험 복귀 → 재생·1프레임으로 비교하세요. 다른 설정을 편집하면 기본 비교 결과가 달라질 수 있습니다." );
            for( std::size_t i = 0; i < 2; ++i )
            {
                const auto& floor = getShapes()[2 * i];
                const auto& circle = getShapes()[2 * i + 1];
                const float floorRestitution = getWorld().GetShapeRestitution( floor.shapeHandle );
                const float circleRestitution = getWorld().GetShapeRestitution( circle.shapeHandle );
                ImGui::TextWrapped( "%s / %s: 바닥 %.2f · 원 %.2f", floor.label, circle.label, floorRestitution, circleRestitution );
                ImGui::TextWrapped( "혼합 계수 %.2f / 세로 속도 %.2f m/s", std::max( floorRestitution, circleRestitution ), getWorld().GetBodyLinearVelocity( circle.bodyHandle ).y );
            }
        }
        if( getKind() == demoKind::frictionComparison ) ImGui::TextWrapped( "마찰은 물체와 경사면 계수의 기하평균으로 정해집니다. 현재 계수는 인스펙터에서 확인·편집하세요." );
        if( getKind() == demoKind::restitutionComparison ) ImGui::TextWrapped( "반발은 두 물체 중 큰 계수를 사용합니다. 바닥의 기본 반발은 0입니다. 인스펙터에서 원과 바닥을 비교하세요." );
        if( ImGui::Button( "시험 복귀 (편집값 유지)###CollisionTrialReset" ) ) resetCollisionTrial();
        ImGui::TextWrapped( "위치·회전·출발 속도를 복원합니다. 물성·중력·선택·재생 상태는 유지합니다. 정지 후 복귀하면 1프레임씩 비교할 수 있습니다. 상단 초기화는 장면의 편집값을 기본 상태로 되돌립니다." );
        return;
    }
    if( getKind() == demoKind::splinePath )
    {
        ImGui::TextUnformatted( "스플라인 경로 이동" );
        ImGui::TextWrapped( "1m/s로 곡선을 따라 이동하고 접선 방향으로 회전합니다. 끝점에서 정지합니다." );
        if( ImGui::Button( "처음부터###SplinePathReset" ) ) resetSplinePath();
        drawSplineEditorControls();
        return;
    }

    if( getKind() == demoKind::splineRail )
    {
        if( !getWorld().IsValid( getSplineRailJoint() ) ) return;
        const auto id = getSplineRailJoint();
        auto rail = getWorld().getSplineRailJointData( id );
        ImGui::TextUnformatted( "스플라인 레일" );
        ImGui::TextWrapped( "Alt+왼쪽 클릭으로 곡선에서 10px 이내의 구간을 선택합니다." );
        if( ImGui::BeginCombo( "재투영 대상###SplineRailReprojectTarget", splineRailReprojectSelected_ ? "지정 구간" : "전체 곡선" ) )
        {
            if( ImGui::Selectable( "전체 곡선", !splineRailReprojectSelected_ ) ) splineRailReprojectSelected_ = false;
            if( ImGui::Selectable( "지정 구간", splineRailReprojectSelected_ ) ) splineRailReprojectSelected_ = true;
            ImGui::EndCombo();
        }
        const auto points = getWorld().getSplineRailJointPoints( id );
        const std::size_t segmentCount = rail.isLoop ? points.size() : points.size() - 1;
        const bool segmentValid = splineRailReprojectSegment_ < segmentCount;
        if( splineRailReprojectSelected_ )
        {
            char label[32];
            if( segmentValid ) std::snprintf( label, sizeof( label ), "구간 %zu", splineRailReprojectSegment_ + 1 );
            else std::snprintf( label, sizeof( label ), "구간 재선택" );
            if( ImGui::BeginCombo( "재투영 구간###SplineRailReprojectSegment", label ) )
            {
                for( std::size_t i = 0; i < segmentCount; ++i )
                {
                    std::snprintf( label, sizeof( label ), "구간 %zu", i + 1 );
                    if( ImGui::Selectable( label, i == splineRailReprojectSegment_ ) ) splineRailReprojectSegment_ = i;
                }
                ImGui::EndCombo();
            }
        }
        ImGui::DragFloat( "재투영 허용 거리###SplineRailReprojectDistance", &splineRailReprojectDistance_, 0.01f, 0.0f, 0.0f, "%.3f m" );
        ImGui::BeginDisabled( splineRailReprojectSelected_ && splineRailReprojectSegment_ >= segmentCount );
        if( ImGui::Button( "레일에 재투영###SplineRailReproject" ) ) splineRailReprojectResult_ = splineRailReprojectSelected_ ? getWorld().reprojectSplineRailJointWithinDistance( id, splineRailReprojectSegment_, splineRailReprojectDistance_ ) : getWorld().reprojectSplineRailJointWithinDistance( id, splineRailReprojectDistance_ );
        ImGui::EndDisabled();
        if( splineRailReprojectResult_ ) ImGui::TextUnformatted( *splineRailReprojectResult_ ? "최근 재투영: 성공" : "최근 재투영: 거절" );
        rail = getWorld().getSplineRailJointData( id );
        ImGui::BeginDisabled( !rail.trackingValid );
        if( ImGui::Button( "현재 위치를 0점으로###SplineRailRebase" ) ) getWorld().rebaseSplineRailJoint( id );
        ImGui::EndDisabled();
        rail = getWorld().getSplineRailJointData( id );
        ImGui::Text( "이동 거리: %.2f / %.2f m", rail.currentTranslation, rail.length );
        ImGui::Text( "실제 이동 속도: %.3f m/s", rail.currentSpeed );
        ImGui::TextWrapped( "동적 물체를 열린 곡선에 구속합니다. 양 끝점에서는 항상 정지합니다." );
        if( ImGui::Checkbox( "회전 구속###SplineRailRotationEnabled", &rail.enableRotation ) ) getWorld().setSplineRailJointRotationEnabled( id, rail.enableRotation );
        if( !rail.enableRotation ) ImGui::TextWrapped( "회전은 자유롭습니다. 아래 회전 설정은 다시 켤 때 적용됩니다." );
        bool follow = rail.rotationMode == splineRailRotationMode::followTangent;
        bool rotationChanged = ImGui::Checkbox( "접선 회전###SplineRailFollow", &follow );
        ImGui::BeginDisabled( !follow );
        ImGui::TextUnformatted( "접선 회전 제약" );
        if( ImGui::RadioButton( "강한 제약###SplineRailAngularHard", !rail.enableAngularSpring ) )
        {
            rail.enableAngularSpring = false;
            rotationChanged = true;
        }
        ImGui::SameLine();
        if( ImGui::RadioButton( "스프링###SplineRailAngularSpring", rail.enableAngularSpring ) )
        {
            rail.enableAngularSpring = true;
            rotationChanged = true;
        }
        ImGui::BeginDisabled( !rail.enableAngularSpring );
        rotationChanged |= ImGui::SliderFloat( "회전 스프링###SplineRailAngularHertz", &rail.angularHertz, 0.0f, 10.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
        rotationChanged |= ImGui::SliderFloat( "회전 감쇠###SplineRailAngularDamping", &rail.angularDampingRatio, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        if( rotationChanged ) getWorld().setSplineRailJointRotation( id, follow ? splineRailRotationMode::followTangent : splineRailRotationMode::fixed, rail.angularHertz, rail.angularDampingRatio, rail.enableAngularSpring );
        rail = getWorld().getSplineRailJointData( id );
        float reference = rail.rotationMode == splineRailRotationMode::fixed ? rail.referenceAngle : rail.tangentOffset;
        const char* referenceLabel = rail.rotationMode == splineRailRotationMode::fixed ? "기준 상대각도###SplineRailRotationReference" : "접선 기준각도###SplineRailRotationReference";
        if( ImGui::DragFloat( referenceLabel, &reference, 0.01f, 0.0f, 0.0f, "%.3f rad" ) ) getWorld().setSplineRailJointRotationReference( id, reference );
        ImGui::BeginDisabled( !rail.trackingValid );
        if( ImGui::Button( "현재 자세를 기준으로###SplineRailRotationRebase" ) ) getWorld().rebaseSplineRailJointRotation( id );
        ImGui::EndDisabled();
        rail = getWorld().getSplineRailJointData( id );
        ImGui::Text( "회전 오차: %.3f rad", rail.currentAngle );
        ImGui::Text( "오차 변화 속도: %.3f rad/s", rail.currentAngularSpeed );
        bool springChanged = ImGui::Checkbox( "스프링###SplineRailSpring", &rail.enableSpring );
        springChanged |= ImGui::SliderFloat( "진동수###SplineRailHertz", &rail.hertz, 0.0f, 10.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
        springChanged |= ImGui::SliderFloat( "감쇠비###SplineRailDamping", &rail.dampingRatio, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
        springChanged |= ImGui::SliderFloat( "목표 위치###SplineRailTarget", &rail.targetTranslation, static_cast<float>( -rail.referenceDistance ), static_cast<float>( rail.length - rail.referenceDistance ), "%.2f m", ImGuiSliderFlags_AlwaysClamp );
        if( springChanged ) getWorld().setSplineRailJointSpring( id, rail.enableSpring, rail.hertz, rail.dampingRatio, rail.targetTranslation );
        bool motorChanged = ImGui::Checkbox( "모터###SplineRailMotor", &rail.enableMotor );
        motorChanged |= ImGui::SliderFloat( "모터 속도###SplineRailSpeed", &rail.motorSpeed, -10.0f, 10.0f, "%.1f m/s", ImGuiSliderFlags_AlwaysClamp );
        motorChanged |= ImGui::SliderFloat( "최대 힘###SplineRailForce", &rail.maxMotorForce, 0.0f, 100.0f, "%.1f N", ImGuiSliderFlags_AlwaysClamp );
        if( motorChanged ) getWorld().setSplineRailJointMotor( id, rail.enableMotor, rail.motorSpeed, rail.maxMotorForce );
        bool limitChanged = ImGui::Checkbox( "이동 제한###SplineRailLimit", &rail.enableLimit );
        const float start = static_cast<float>( -rail.referenceDistance ), end = static_cast<float>( rail.length - rail.referenceDistance );
        limitChanged |= ImGui::SliderFloat( "최소 위치###SplineRailLower", &rail.lowerTranslation, start, end, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
        limitChanged |= ImGui::SliderFloat( "최대 위치###SplineRailUpper", &rail.upperTranslation, start, end, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
        if( limitChanged ) getWorld().setSplineRailJointLimit( id, rail.enableLimit, rail.lowerTranslation, rail.upperTranslation );
        drawSplineEditorControls();
        return;
    }

    if( getKind() == demoKind::distancePendulum && !getWorld().IsValid( getPendulumJoint() ) ) return;

    if( getKind() == demoKind::revoluteHinge && ( !getWorld().IsValid( getRevoluteJoint() ) || !getWorld().IsValid( getImpulseBody() ) ) ) return;

    if( getKind() == demoKind::wheelSuspension && ( !getWorld().IsValid( getWheelJoint() ) || !getWorld().IsValid( getImpulseBody() ) ) ) return;

    if( getKind() == demoKind::motorCar && ( !getWorld().IsValid( getImpulseBody() ) || !getWorld().IsValid( getCarJoints()[0] ) ) ) return;

    if( getKind() == demoKind::playground && ( !getWorld().IsValid( getImpulseBody() ) || !getWorld().IsValid( getTorqueBody() ) ) ) return;

    if( getKind() == demoKind::terrainChain || getKind() == demoKind::splineTerrain )
    {
        const char* probes[] = { "원", "캡슐", "상자" };
        if( getKind() == demoKind::splineTerrain )
        {
            drawSplineEditorControls();
            if( ImGui::Button( "지형 원형 복원###SplineTerrainReset" ) ) resetSplineTerrain();
            float error = getSplineSamplingError() * 100.0f;
            const bool errorChanged = ImGui::SliderFloat( "근사 오차###SplineSamplingError", &error, 0.5f, 5.0f, "%.2f cm", ImGuiSliderFlags_AlwaysClamp );
            if( ImGui::IsItemActivated() ) beginSplineSamplingEdit();
            if( errorChanged ) setSplineSamplingError( error / 100.0f );
            if( ImGui::IsItemDeactivated() ) endSplineSamplingEdit();
            ImGui::Text( "선분 수: %zu", getTerrainPointCount() - 1 );
            ImGui::TextWrapped( "충돌 선분이 너무 짧아지는 오차 변경이나 제어점 이동은 적용하지 않습니다." );
            float fraction = getSplineTrialFraction() * 100.0f;
            if( ImGui::SliderFloat( "앞·뒷면 시험 위치###SplineTrialFraction", &fraction, 0.0f, 100.0f, "%.0f %%", ImGuiSliderFlags_AlwaysClamp ) ) setSplineTrialFraction( fraction / 100.0f );
            float speed = getSplineTrialSpeed();
            if( ImGui::SliderFloat( "앞·뒷면 시험 속도###SplineTrialSpeed", &speed, 1.0f, 20.0f, "%.1f m/s", ImGuiSliderFlags_AlwaysClamp ) ) setSplineTrialSpeed( speed );
            ImGui::TextUnformatted( "속도는 다음 앞면·뒷면 발사부터 적용합니다." );
            ImGui::TextWrapped( "지형 시작부터의 길이 비율입니다. Shift+왼쪽 드래그로 가까운 지형 위 위치를 고를 수 있습니다. 하늘색 점과 화살표는 시험 위치와 충돌 앞면을 표시합니다. 이음새 통과 시험은 시작 위치를 유지합니다." );
            ImGui::BeginDisabled( !getTerrainTrialReference() );
            if( ImGui::Button( "기준 조건으로 재발사###SplineTrialRelaunch" ) && relaunchTerrainTrialReference() ) terrainProbeIndex_ = static_cast<int>( getTerrainTrialObservation().probeIndex );
            ImGui::EndDisabled();
            const auto observation = getTerrainTrialObservation();
            ImGui::BeginDisabled( !observation.active || observation.elapsed <= 0.0f );
            if( ImGui::Button( "관측 기준 고정###SplineTrialPin" ) ) pinTerrainTrialReference();
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled( !getTerrainTrialReference() );
            if( ImGui::Button( "기준 해제###SplineTrialReferenceClear" ) ) clearTerrainTrialReference();
            ImGui::EndDisabled();
            const auto drawObservation = [&]( const terrainTrialObservation& value, const char* title )
            {
                if( !value.active )
                {
                    ImGui::Text( "%s: 없음", title );
                    return;
                }
                const char* trial = value.trial == terrainTrial::front ? "앞면" : value.trial == terrainTrial::back ? "뒷면" : "이음새";
                ImGui::Text( "%s: %s / %s", title, probes[value.probeIndex], trial );
                if( value.launchFraction ) ImGui::Text( "발사 조건: 위치 %.1f %% / 초기 속도 %.1f m/s", *value.launchFraction * 100.0f, value.launchSpeed );
                else ImGui::Text( "발사 조건: 첫 선분 중앙 / 초기 속도 %.1f m/s", value.launchSpeed );
                ImGui::Text( "발사 중력: (%.3f, %.3f) m/s²", value.launchGravity.x, value.launchGravity.y );
                ImGui::Text( "발사 물체 중력 배율: %.3f", value.launchGravityScale );
                ImGui::Text( "발사 물체 물성: 마찰 %.3f / 반발 %.3f", value.launchFriction, value.launchRestitution );
                ImGui::Text( "발사 물체 질량: %.3f kg", value.launchMass );
                ImGui::Text( "발사 물체 회전 관성: %.3f kg·m²", value.launchInertia );
                ImGui::Text( "경과: %.3f s / 지형 접촉: %s", value.elapsed, value.touching ? "있음" : "없음" );
                if( value.firstContactTime )
                {
                    ImGui::Text( "첫 접촉 관측: %.3f s", *value.firstContactTime );
                    ImGui::Text( "관측 단계: %.3f ms / 내부 분할 %d", value.firstContactTimeStep * 1000.0f, value.firstContactSubStepCount );
                    ImGui::Text( "첫 접촉까지 진행 설정 변경: %s", value.preContactSettingsChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 중력 변경: %s", value.preContactGravityChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 중력 배율 변경: %s", value.preContactGravityScaleChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 물체 물성 변경: %s", value.preContactMaterialChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 물체 질량 변경: %s", value.preContactMassChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 물체 관성 변경: %s", value.preContactInertiaChanged ? "있음" : "없음" );
                }
                else ImGui::TextUnformatted( "첫 접촉 관측: 아직 없음" );
            };
            drawObservation( observation, "최근 발사" );
            const auto reference = getTerrainTrialReference();
            if( reference ) drawObservation( *reference, "비교 기준" );
            else ImGui::TextUnformatted( "비교 기준: 없음" );
            const auto comparison = getTerrainTrialComparison();
            switch( comparison.status )
            {
                case terrainTrialComparisonStatus::noReference: ImGui::TextUnformatted( "비교: 기준 없음" ); break;
                case terrainTrialComparisonStatus::noObservation: ImGui::TextUnformatted( "비교: 현재 관측 없음" ); break;
                case terrainTrialComparisonStatus::differentConditions: ImGui::TextUnformatted( "비교: 발사 조건 다름" ); break;
                case terrainTrialComparisonStatus::waitingForContact: ImGui::TextUnformatted( "비교: 발사 조건 같음 / 접촉 관측 부족" ); break;
                case terrainTrialComparisonStatus::ready:
                    ImGui::TextUnformatted( "비교: 발사 조건 같음" );
                    ImGui::Text( "첫 접촉 관측 단계 설정: %s", comparison.contactStepSettingsMatch ? "같음" : "다름" );
                    ImGui::Text( "첫 접촉까지 진행 설정 비교: %s", comparison.stepSettings == terrainTrialStepComparison::changed ? "변경 포함" : comparison.stepSettings == terrainTrialStepComparison::constantMatch ? "일정·같음" : "일정·다름" );
                    ImGui::Text( "비교 중력 변경 관측: %s", comparison.gravityChangesObserved ? "포함" : "없음" );
                    ImGui::Text( "비교 물체 물성 변경 관측: %s", comparison.materialChangesObserved ? "포함" : "없음" );
                    ImGui::Text( "비교 물체 질량 변경 관측: %s", comparison.massChangesObserved ? "포함" : "없음" );
                    ImGui::Text( "비교 물체 관성 변경 관측: %s", comparison.inertiaChangesObserved ? "포함" : "없음" );
                    ImGui::Text( "비교 중력 배율 변경 관측: %s", comparison.gravityScaleChangesObserved ? "포함" : "없음" );
                    ImGui::Text( "첫 접촉 관측 차이 (현재 - 기준): %+.3f s", comparison.firstContactTimeDifference );
                    break;
            }
            ImGui::TextWrapped( "비교는 기록된 발사 조건·중력·중력 배율·물체 물성·질량·관성과 첫 접촉 관측 단계 설정만 확인합니다. 전체 진행 이력이나 지형 물성·발사 이후 변경까지 같다는 뜻은 아닙니다." );
            ImGui::TextWrapped( "접촉 시간은 물리 진행이 끝난 뒤 처음 확인한 시각입니다. 물리를 진행한 관측을 기준으로 고정할 수 있으며, 지형 편집 시 기준은 지워집니다. 기준 조건 재발사는 현재 중력·물성을 사용합니다." );
        }
        ImGui::Combo( "시험 도형###TerrainProbe", &terrainProbeIndex_, probes, 3 );
        if( ImGui::Button( "이음새 통과###TerrainSeams" ) ) launchTerrainProbe( terrainProbeIndex_, terrainTrial::seams );
        if( ImGui::Button( "앞면 접근###TerrainFront" ) ) launchTerrainProbe( terrainProbeIndex_, terrainTrial::front );
        ImGui::SameLine();
        if( ImGui::Button( "뒷면 접근###TerrainBack" ) ) launchTerrainProbe( terrainProbeIndex_, terrainTrial::back );
        if( ImGui::Button( "시험 물체 복귀###TerrainReset" ) ) resetTerrainProbes();
        ImGui::TextWrapped( "초록 화살표 쪽이 충돌하는 앞면입니다. 뒷면에서는 통과할 수 있습니다. 이웃 점 정보는 이음새의 불필요한 걸림을 줄이며 급경사·마찰까지 없애지는 않습니다. 같은 경로를 도형별로 비교하세요. 발사·복귀는 편집 지형과 물성·중력을 보존합니다. 지형을 접거나 교차시키면 단순한 위·아래 대신 각 선분의 화살표를 기준으로 보세요." );
    }
    else if( getKind() == demoKind::distancePendulum )
    {
        ImGui::TextUnformatted( "거리 조인트" );
        distanceJointData pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
        float length = pendulum.length;
        if( ImGui::SliderFloat( "목표 길이###DistanceLength", &length, 0.5f, 3.0f, "%.2f m", ImGuiSliderFlags_AlwaysClamp ) )
        {
            getWorld().setDistanceJointLength( getPendulumJoint(), length );
            pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
        }
        bool enableSpring = pendulum.enableSpring;
        float hertz = pendulum.hertz, dampingRatio = pendulum.dampingRatio;
        bool changed = ImGui::Checkbox( "거리 스프링###Distance spring", &enableSpring );
        changed |= ImGui::SliderFloat( "거리 스프링 주파수###Distance Hertz", &hertz, 0.0f, 30.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
        changed |= ImGui::SliderFloat( "거리 스프링 감쇠 비율###Distance damping", &dampingRatio, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
        if( changed )
        {
            getWorld().setDistanceJointSpring( getPendulumJoint(), enableSpring, hertz, dampingRatio );
            pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
        }
        bool enableLimit = pendulum.enableLimit;
        float minLength = pendulum.minLength, maxLength = pendulum.maxLength;
        bool limitChanged = ImGui::Checkbox( "거리 제한###Distance limit", &enableLimit );
        limitChanged |= ImGui::SliderFloat( "최소 거리###Distance min", &minLength, LINEAR_SLOP, maxLength, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
        limitChanged |= ImGui::SliderFloat( "최대 거리###Distance max", &maxLength, minLength, 4.0f, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
        if( limitChanged )
        {
            getWorld().setDistanceJointLimit( getPendulumJoint(), enableLimit, minLength, maxLength );
            pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
        }
        const bool rigid = !enableSpring || ( enableLimit && minLength == maxLength );
        if( ImGui::CollapsingHeader( "거리 모터###DistanceMotorSettings" ) )
        {
            bool enableMotor = pendulum.enableMotor;
            float motorSpeed = pendulum.motorSpeed;
            float maxMotorForce = pendulum.maxMotorForce;
            bool motorChanged = ImGui::Checkbox( "모터 사용###Distance motor", &enableMotor );
            if( ImGui::Button( "늘리기###DistanceExtend" ) ) { motorSpeed = 1.0f; enableMotor = true; motorChanged = true; }
            ImGui::SameLine();
            if( ImGui::Button( "감기###DistanceRetract" ) ) { motorSpeed = -1.0f; enableMotor = true; motorChanged = true; }
            ImGui::SameLine();
            if( ImGui::Button( "제동###DistanceBrake" ) ) { motorSpeed = 0.0f; enableMotor = true; motorChanged = true; }
            motorChanged |= ImGui::SliderFloat( "목표 축속도###Distance motor speed", &motorSpeed, -5.0f, 5.0f, "%.2f m/s", ImGuiSliderFlags_AlwaysClamp );
            motorChanged |= ImGui::SliderFloat( "최대 모터 힘###Distance motor force", &maxMotorForce, 0.0f, 50.0f, "%.1f N", ImGuiSliderFlags_AlwaysClamp );
            if( motorChanged )
            {
                getWorld().setDistanceJointMotor( getPendulumJoint(), enableMotor, motorSpeed, maxMotorForce );
                pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
            }

            ImGui::Text( "실제 모터 힘: %.2f N", pendulum.motorForce );
            ImGui::TextWrapped( "양의 속도는 거리를 늘이고 음의 속도는 줄입니다. 속도 0은 제동, 힘 0은 모터 힘을 끕니다. 스프링 모드에서 작동하며 거리 제한도 함께 적용됩니다." );
            if( rigid )
            {
                ImGui::TextWrapped( "현재 고정 거리 모드에서는 모터가 작동하지 않습니다. 스프링을 켜고 최소·최대 거리를 다르게 설정하세요. 0 Hz로 스프링 힘만 끌 수 있습니다." );
            }
        }

        ImGui::TextUnformatted( rigid ? "목표 거리에 고정" : hertz > 0.0f ? "거리 스프링" : pendulum.enableMotor && pendulum.maxMotorForce > 0.0f ? "거리 모터 (0 Hz)" : enableLimit ? "거리 제한만 적용 (0 Hz)" : "거리 축 자유 이동 (0 Hz)" );
        ImGui::Text( "목표 거리: %.3f m / 현재 거리: %.3f m", pendulum.length, pendulum.currentLength );
        ImGui::Text( "늘어난 길이: %.3f m / 축 방향 힘: %.2f N", pendulum.currentLength - pendulum.length, pendulum.axialForce );
        ImGui::TextWrapped( "주파수는 스프링 강성, 감쇠 비율은 진동을 조절합니다. 음의 힘은 당기는 힘입니다. 거리 제한은 스프링을 켜야 적용되며 0 Hz에서도 작동합니다. 최소·최대 거리가 같으면 목표 거리에 고정됩니다." );
        if( enableLimit && !rigid )
        {
            ImGui::TextWrapped( "초록색은 최소, 빨간색은 최대 거리입니다. 제한은 부드럽게 보정하므로 하중이 있으면 작은 오차가 남을 수 있습니다." );
        }
        if( ImGui::Button( "진자 옆으로 밀기###Kick pendulum", ImVec2( -1.0f, 0.0f ) ) )
        {
            getWorld().ApplyLinearImpulseToCenter( getPendulumBody(), { getWorld().GetBodyMass( getPendulumBody() ) * 2.0f, 0.0f } );
        }
        ImGui::TextWrapped( "보라색 원은 목표 길이입니다. 스프링 프리셋을 선택하고 아래 버튼으로 늘려 놓아 탄성과 감쇠를 비교하세요." );
        if( ImGui::Button( "스프링 늘였다 놓기 (축 방향 충격)###Radial kick", ImVec2( -1.0f, 0.0f ) ) )
        {
            const vec2 axis = Normalize( pendulum.anchorB - pendulum.anchorA );
            getWorld().ApplyLinearImpulseToCenter( getPendulumBody(), getWorld().GetBodyMass( getPendulumBody() ) * 2.0f * axis );
        }
        ImGui::Spacing();
    }
    else if( getKind() == demoKind::revoluteHinge )
    {
        ImGui::TextUnformatted( "회전축 관찰" );
        revoluteJointData joint = getWorld().getRevoluteJointData( getRevoluteJoint() );
        ImGui::Text( "기준 대비 각도: %.1f 도", joint.currentAngle * 180.0f / std::numbers::pi_v<float> );
        ImGui::Text( "연결점 오차: %.4f m", Length( joint.anchorB - joint.anchorA ) );
        ImGui::Text( "반력: (%.2f, %.2f) N", joint.force.x, joint.force.y );
        ImGui::TextWrapped( "청록색은 고정점, 보라색은 막대의 연결점과 방향입니다. 막대를 잡거나 충격량을 주면서 연결점이 유지되는지 관찰하세요." );

        if( ImGui::TreeNode( "각도 제한###RevoluteLimitSettings" ) )
        {
            constexpr float TO_DEGREES = 180.0f / std::numbers::pi_v<float>;
            constexpr float TO_RADIANS = std::numbers::pi_v<float> / 180.0f;
            constexpr float MAX_ANGLE = 178.2f;
            bool enableLimit = joint.enableLimit;
            float lower = joint.lowerAngle * TO_DEGREES;
            float upper = joint.upperAngle * TO_DEGREES;
            bool changed = ImGui::Checkbox( "각도 제한 사용###Revolute limit", &enableLimit );
            ImGui::Text( "기준 각도: %.1f 도", joint.referenceAngle * TO_DEGREES );
            if( ImGui::SliderFloat( "최소 각도###Revolute lower", &lower, -MAX_ANGLE, upper, "%.1f 도", ImGuiSliderFlags_AlwaysClamp ) )
            {
                lower = std::clamp( lower, -MAX_ANGLE, upper );
                changed = true;
            }
            if( ImGui::SliderFloat( "최대 각도###Revolute upper", &upper, lower, MAX_ANGLE, "%.1f 도", ImGuiSliderFlags_AlwaysClamp ) )
            {
                upper = std::clamp( upper, lower, MAX_ANGLE );
                changed = true;
            }
            if( changed )
            {
                getWorld().setRevoluteJointLimit( getRevoluteJoint(), enableLimit, lower * TO_RADIANS, upper * TO_RADIANS );
                joint = getWorld().getRevoluteJointData( getRevoluteJoint() );
            }
            ImGui::Text( "제한 토크: %.2f N·m", joint.torque );
            ImGui::TextWrapped( "초록은 최소, 빨강은 최대 각도이며 회색 호는 허용 범위입니다. 경계에서는 바깥 회전을 막고 안쪽 복귀는 허용합니다. 두 각도가 같으면 그 각도를 유지합니다." );
            ImGui::TreePop();
        }

        if( ImGui::TreeNode( "회전 모터###RevoluteMotorSettings" ) )
        {
            bool enableMotor = joint.enableMotor;
            float motorSpeed = joint.motorSpeed;
            float maxMotorTorque = joint.maxMotorTorque;
            bool changed = ImGui::Checkbox( "모터 사용###Revolute motor", &enableMotor );
            if( ImGui::SliderFloat( "목표 각속도###Revolute motor speed", &motorSpeed, -5.0f, 5.0f, "%.2f rad/s", ImGuiSliderFlags_AlwaysClamp ) )
            {
                motorSpeed = std::clamp( motorSpeed, -5.0f, 5.0f );
                changed = true;
            }
            if( ImGui::SliderFloat( "최대 모터 토크###Revolute motor torque", &maxMotorTorque, 0.0f, 50.0f, "%.1f N·m", ImGuiSliderFlags_AlwaysClamp ) )
            {
                maxMotorTorque = std::clamp( maxMotorTorque, 0.0f, 50.0f );
                changed = true;
            }
            if( ImGui::Button( "회전 방향 바꾸기###Reverse revolute motor", ImVec2( -1.0f, 0.0f ) ) )
            {
                motorSpeed = -motorSpeed;
                changed = true;
            }
            if( ImGui::Button( "제동하기###Brake revolute motor", ImVec2( -1.0f, 0.0f ) ) )
            {
                enableMotor = true;
                motorSpeed = 0.0f;
                changed = true;
            }
            if( changed )
            {
                getWorld().setRevoluteJointMotor( getRevoluteJoint(), enableMotor, motorSpeed, maxMotorTorque );
                joint = getWorld().getRevoluteJointData( getRevoluteJoint() );
            }

            const float angularVelocity = getWorld().GetBodyAngularVelocity( joint.bodyB ) - getWorld().GetBodyAngularVelocity( joint.bodyA );
            ImGui::Text( "현재 상대 각속도: %.2f rad/s", angularVelocity );
            ImGui::Text( "실제 모터 토크: %.2f N·m", joint.motorTorque );
            ImGui::TextWrapped( "양의 속도는 반시계, 음의 속도는 시계 방향입니다. 속도 0은 제동, 토크 0은 모터 힘을 끕니다. 토크가 중력 하중보다 작으면 목표 속도에 도달하지 못할 수 있습니다. 각도 제한을 켜면 경계에서 멈춥니다." );
            ImGui::TreePop();
        }

        if( ImGui::Button( "막대 회전시키기###Spin hinge", ImVec2( -1.0f, 0.0f ) ) )
        {
            getWorld().ApplyAngularImpulse( getImpulseBody(), getWorld().GetBodyRotationalInertia( getImpulseBody() ) * 3.0f );
        }
        if( ImGui::Button( "각도 제한 시험 (±45도)###Test hinge limit", ImVec2( -1.0f, 0.0f ) ) )
        {
            testRevoluteLimit();
        }
        ImGui::TextWrapped( "제한 시험은 모터를 끄고 ±45도 제한을 적용한 뒤 막대를 강하게 밉니다. 경계에서 멈춘 뒤 안쪽으로 돌아오는지 관찰하세요." );
        if( ImGui::Button( "막대 옆으로 밀기###Kick hinge", ImVec2( -1.0f, 0.0f ) ) )
        {
            getWorld().ApplyLinearImpulseToCenter( getImpulseBody(), { getWorld().GetBodyMass( getImpulseBody() ) * 2.0f, 0.0f } );
        }
        ImGui::Spacing();
    }
    else if( getKind() == demoKind::wheelSuspension )
    {
        ImGui::TextUnformatted( "서스펜션 관찰" );
        wheelJointData joint = getWorld().getWheelJointData( getWheelJoint() );
        ImGui::Text( "축 방향 변위: %.3f m", joint.currentTranslation );
        ImGui::Text( "축 옆 오차: %.4f m", joint.lateralError );
        const float angularVelocity = getWorld().GetBodyAngularVelocity( joint.bodyB ) - getWorld().GetBodyAngularVelocity( joint.bodyA );
        ImGui::Text( "바퀴 상대 각속도: %.2f rad/s", angularVelocity );
        ImGui::Text( "스프링 힘: %.2f N / 제한 힘: %.2f N", joint.springForce, joint.limitForce );
        ImGui::TextWrapped( "청록색 점은 스프링 변위 0, 화살표는 이동 축의 양의 방향입니다. 보라색은 바퀴와 회전 방향입니다. 스프링을 꺼도 축 옆으로는 벗어나지 않으며 모터가 꺼져 있으면 바퀴 회전은 자유롭습니다." );

        if( ImGui::TreeNode( "서스펜션 스프링###WheelSpringSettings" ) )
        {
            bool enableSpring = joint.enableSpring;
            float hertz = joint.hertz;
            float dampingRatio = joint.dampingRatio;
            bool changed = ImGui::Checkbox( "스프링 사용###Wheel spring", &enableSpring );
            if( ImGui::SliderFloat( "스프링 주파수###Wheel hertz", &hertz, 0.0f, 10.0f, "%.2f Hz", ImGuiSliderFlags_AlwaysClamp ) )
            {
                hertz = std::clamp( hertz, 0.0f, 10.0f );
                changed = true;
            }
            if( ImGui::SliderFloat( "감쇠 비율###Wheel damping", &dampingRatio, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp ) )
            {
                dampingRatio = std::clamp( dampingRatio, 0.0f, 2.0f );
                changed = true;
            }
            if( changed )
            {
                getWorld().setWheelJointSpring( getWheelJoint(), enableSpring, hertz, dampingRatio );
                joint = getWorld().getWheelJointData( getWheelJoint() );
            }
            ImGui::TextWrapped( "주파수는 스프링 강성, 감쇠 비율은 진동을 조절합니다. 스프링을 끄거나 0 Hz로 설정하면 축 방향 스프링 힘만 사라집니다. 이동 범위 제한도 꺼져 있으면 중력으로 떨어집니다." );
            ImGui::TreePop();
        }
        if( ImGui::TreeNode( "이동 범위 제한###WheelLimitSettings" ) )
        {
            constexpr float MAX_TRANSLATION = 1.0f;
            bool enableLimit = joint.enableLimit;
            float lower = joint.lowerTranslation;
            float upper = joint.upperTranslation;
            bool changed = ImGui::Checkbox( "이동 제한 사용###Wheel limit", &enableLimit );
            if( ImGui::SliderFloat( "최소 변위###Wheel lower", &lower, -MAX_TRANSLATION, upper, "%.2f m", ImGuiSliderFlags_AlwaysClamp ) )
            {
                lower = std::clamp( lower, -MAX_TRANSLATION, upper );
                changed = true;
            }
            if( ImGui::SliderFloat( "최대 변위###Wheel upper", &upper, lower, MAX_TRANSLATION, "%.2f m", ImGuiSliderFlags_AlwaysClamp ) )
            {
                upper = std::clamp( upper, lower, MAX_TRANSLATION );
                changed = true;
            }
            if( changed )
            {
                getWorld().setWheelJointLimit( getWheelJoint(), enableLimit, lower, upper );
                joint = getWorld().getWheelJointData( getWheelJoint() );
            }
            ImGui::Text( "제한 힘: %.2f N", joint.limitForce );
            ImGui::TextWrapped( "청록색 기준점에서 축 방향의 부호 있는 변위입니다. 초록은 최소, 빨강은 최대 변위이며 회색은 허용 범위입니다. 스프링을 꺼도 제한은 유지되고 회전은 별도 모터로 조절합니다. 두 변위가 같으면 해당 위치를 유지합니다. 제한은 부드럽게 보정하므로 하중에서 작은 오차가 남을 수 있습니다." );
            ImGui::TreePop();
        }
        if( ImGui::TreeNode( "회전 모터###WheelMotorSettings" ) )
        {
            bool enableMotor = joint.enableMotor;
            float motorSpeed = joint.motorSpeed;
            float maxMotorTorque = joint.maxMotorTorque;
            bool changed = ImGui::Checkbox( "모터 사용###Wheel motor", &enableMotor );
            if( ImGui::SliderFloat( "목표 각속도###Wheel motor speed", &motorSpeed, -10.0f, 10.0f, "%.2f rad/s", ImGuiSliderFlags_AlwaysClamp ) )
            {
                motorSpeed = std::clamp( motorSpeed, -10.0f, 10.0f );
                changed = true;
            }
            if( ImGui::SliderFloat( "최대 모터 토크###Wheel motor torque", &maxMotorTorque, 0.0f, 5.0f, "%.2f N·m", ImGuiSliderFlags_AlwaysClamp ) )
            {
                maxMotorTorque = std::clamp( maxMotorTorque, 0.0f, 5.0f );
                changed = true;
            }
            if( ImGui::Button( "회전 방향 바꾸기###Reverse wheel motor", ImVec2( -1.0f, 0.0f ) ) )
            {
                motorSpeed = -motorSpeed;
                changed = true;
            }
            if( ImGui::Button( "제동하기###Brake wheel motor", ImVec2( -1.0f, 0.0f ) ) )
            {
                enableMotor = true;
                motorSpeed = 0.0f;
                changed = true;
            }
            if( changed )
            {
                getWorld().setWheelJointMotor( getWheelJoint(), enableMotor, motorSpeed, maxMotorTorque );
                joint = getWorld().getWheelJointData( getWheelJoint() );
            }

            ImGui::Text( "실제 모터 토크: %.2f N·m", joint.motorTorque );
            ImGui::TextWrapped( "양의 속도는 반시계, 음의 속도는 시계 방향입니다. 속도 0은 제동, 토크 0은 모터 힘을 끕니다. 토크가 작거나 연결점이 중심 밖에 있으면 목표 속도에 도달하지 못할 수 있습니다. 모터는 각도를 고정하지 않으며 이동 제한과 함께 사용할 수 있습니다." );
            ImGui::TreePop();
        }

        if( ImGui::Button( "바퀴 축 방향으로 밀기###Kick wheel", ImVec2( -1.0f, 0.0f ) ) )
        {
            getWorld().ApplyLinearImpulseToCenter( getImpulseBody(), getWorld().GetBodyMass( getImpulseBody() ) * 2.0f * joint.axis );
        }
        if( ImGui::Button( "상한 시험###WheelUpperTrial" ) ) testWheelLimit( true );
        ImGui::SameLine();
        if( ImGui::Button( "하한 시험###WheelLowerTrial" ) ) testWheelLimit( false );
        ImGui::TextWrapped( "시험은 결합 프리셋(스프링·±0.5m 제한·모터)을 적용하고 선택한 경계로 강하게 밉니다. 경계에서 멈춘 뒤 스프링으로 복귀하며 회전은 계속되는지 관찰하세요." );
        if( ImGui::Button( "바퀴 회전시키기###Spin wheel", ImVec2( -1.0f, 0.0f ) ) )
        {
            getWorld().ApplyAngularImpulse( getImpulseBody(), getWorld().GetBodyRotationalInertia( getImpulseBody() ) * 3.0f );
        }
        ImGui::Spacing();
    }
    else if( getKind() == demoKind::motorCar )
    {
        ImGui::TextUnformatted( "자동차 주행 관찰" );
        bool tracking = getCarCameraTracking();
        if( ImGui::Checkbox( "카메라 수평 추적###CarCameraTracking", &tracking ) ) setCarCameraTracking( tracking );
        if( ImGui::Button( "차량 시작 위치 복귀###CarReset" ) ) resetCar();
        if( ImGui::Button( "부드러운###CarSoft" ) ) applyCarSuspensionPreset( carSuspensionPreset::soft );
        ImGui::SameLine();
        if( ImGui::Button( "기본###CarStandard" ) ) applyCarSuspensionPreset( carSuspensionPreset::standard );
        ImGui::SameLine();
        if( ImGui::Button( "단단한###CarStiff" ) ) applyCarSuspensionPreset( carSuspensionPreset::stiff );
        if( ImGui::Button( "스프링 없음###CarNoSpring" ) ) applyCarSuspensionPreset( carSuspensionPreset::noSpring );
        if( ImGui::Button( "기본 마찰 (0.9)###CarGrip" ) ) setCarLowFriction( false );
        ImGui::SameLine();
        if( ImGui::Button( "저마찰 (0.02)###CarLowGrip" ) ) setCarLowFriction( true );
        ImGui::TextWrapped( "서스펜션 프리셋은 ±0.25m 이동 제한을 켭니다. 설정을 바꾼 뒤 차량 복귀로 같은 경사를 비교하세요. 복귀는 마찰·서스펜션·구동 설정을 보존합니다. 추적을 끄면 카메라를 수동 이동할 수 있습니다. 바닥 끝에서 떨어지거나 뒤집히면 복귀하세요." );
        ImGui::Text( "차체 수평 속도: %.2f m/s", getWorld().GetBodyLinearVelocity( getImpulseBody() ).x );
        ImGui::TextWrapped( "캔버스 위 A/D로 주행하고 스페이스를 유지하면 제동합니다. 키를 놓으면 모터를 끄고 관성으로 구릅니다. 양쪽 키를 함께 누르면 자유 주행하며 제동이 주행보다 우선합니다." );
        if( ImGui::TreeNode( "구동 모터 설정###CarMotorSettings" ) )
        {
            float speed = getCarMotorSpeed(), torque = getCarMaxMotorTorque();
            bool changed = false;
            if( ImGui::SliderFloat( "목표 속도 크기###Car motor speed", &speed, 0.0f, 20.0f, "%.2f rad/s", ImGuiSliderFlags_AlwaysClamp ) )
            {
                speed = std::clamp( speed, 0.0f, 20.0f );
                changed = true;
            }
            if( ImGui::SliderFloat( "최대 구동·제동 토크###Car motor torque", &torque, 0.0f, 20.0f, "%.2f N·m", ImGuiSliderFlags_AlwaysClamp ) )
            {
                torque = std::clamp( torque, 0.0f, 20.0f );
                changed = true;
            }
            if( changed ) setCarMotorSettings( speed, torque );

            ImGui::TextWrapped( "값은 양쪽 바퀴에 같은 한도로 적용합니다. 오른쪽 주행은 시계 방향 회전입니다. 토크 0은 구동과 제동을 끕니다. 바퀴가 미끄러지면 회전 속도만으로 차체 속도를 예측할 수 없습니다. 인스펙터에서 타이어·바닥의 마찰을 비교해 보세요." );
            ImGui::TreePop();
        }
        if( ImGui::TreeNode( "서스펜션 설정###CarSuspensionSettings" ) )
        {
            const auto joint = getWorld().getWheelJointData( getCarJoints()[0] );
            bool spring = joint.enableSpring, limit = joint.enableLimit;
            float hertz = joint.hertz, damping = joint.dampingRatio;
            float lower = joint.lowerTranslation, upper = joint.upperTranslation;
            bool springChanged = ImGui::Checkbox( "스프링 사용###Car spring", &spring );
            if( ImGui::SliderFloat( "스프링 주파수###Car hertz", &hertz, 0.0f, 10.0f, "%.2f Hz", ImGuiSliderFlags_AlwaysClamp ) )
            {
                hertz = std::clamp( hertz, 0.0f, 10.0f );
                springChanged = true;
            }
            if( ImGui::SliderFloat( "감쇠 비율###Car damping", &damping, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp ) )
            {
                damping = std::clamp( damping, 0.0f, 2.0f );
                springChanged = true;
            }
            bool limitChanged = ImGui::Checkbox( "이동 제한 사용###Car limit", &limit );
            if( ImGui::SliderFloat( "최소 변위###Car lower", &lower, -0.5f, upper, "%.2f m", ImGuiSliderFlags_AlwaysClamp ) )
            {
                lower = std::clamp( lower, -0.5f, upper );
                limitChanged = true;
            }
            if( ImGui::SliderFloat( "최대 변위###Car upper", &upper, lower, 0.5f, "%.2f m", ImGuiSliderFlags_AlwaysClamp ) )
            {
                upper = std::clamp( upper, lower, 0.5f );
                limitChanged = true;
            }
            for( const jointId id : getCarJoints() )
            {
                if( !getWorld().IsValid( id ) ) continue;

                if( springChanged ) getWorld().setWheelJointSpring( id, spring, hertz, damping );

                if( limitChanged ) getWorld().setWheelJointLimit( id, limit, lower, upper );
            }
            ImGui::TextWrapped( "양쪽 바퀴를 함께 조절합니다. 축과 이동 경계는 차체와 함께 회전합니다. 스프링과 제한을 모두 끄면 차체 하중을 지지하지 못합니다. 낮은 경사면에서 변위와 반력이 어떻게 달라지는지 비교하세요." );
            ImGui::TreePop();
        }
        if( ImGui::TreeNode( "바퀴 반력 관찰###CarWheelObservations" ) )
        {
            for( std::size_t wheel = 0; wheel < getCarJoints().size(); ++wheel )
            {
                const jointId id = getCarJoints()[wheel];
                if( !getWorld().IsValid( id ) ) continue;

                const auto joint = getWorld().getWheelJointData( id );
                const float speed = getWorld().GetBodyAngularVelocity( joint.bodyB ) - getWorld().GetBodyAngularVelocity( joint.bodyA );
                ImGui::Text( "%s 바퀴: %.2f rad/s / %.2f N·m", wheel == 0 ? "왼쪽" : "오른쪽", speed, joint.motorTorque );
                ImGui::Text( "변위 %.3f m / 스프링 %.2f N / 제한 %.2f N", joint.currentTranslation, joint.springForce, joint.limitForce );
                const auto observation = getCarWheelObservation( wheel );
                if( observation.grounded ) ImGui::Text( "접지 / 접선 미끄러짐 %.3f m/s", observation.slipSpeed );
                else ImGui::TextUnformatted( "공중 / 미끄러짐 측정 없음" );
            }
            ImGui::TreePop();
        }
        ImGui::Spacing();
    }
    else if( getKind() == demoKind::ragdoll )
    {
        ImGui::TextUnformatted( "렉돌 낙하·관절 비교" );
        if( ImGui::Button( "처음 자세로 복귀###RagdollReset" ) ) resetRagdoll();
        if( ImGui::Button( "자유 관절###RagdollFree" ) ) setRagdollJointFriction( 0.0f );
        ImGui::SameLine();
        if( ImGui::Button( "약한 마찰###RagdollLightFriction" ) ) setRagdollJointFriction( 0.03f );
        ImGui::SameLine();
        if( ImGui::Button( "강한 마찰###RagdollStrongFriction" ) ) setRagdollJointFriction( 0.3f );
        float friction = getRagdollJointFriction();
        if( ImGui::SliderFloat( "관절 마찰 토크###RagdollFriction", &friction, 0.0f, 1.0f, "%.3f N·m", ImGuiSliderFlags_AlwaysClamp ) ) setRagdollJointFriction( friction );
        bool selfCollision = getRagdollSelfCollision();
        if( ImGui::Checkbox( "비연결 부위 자기 충돌###RagdollSelfCollision", &selfCollision ) ) setRagdollSelfCollision( selfCollision );
        int internalContacts = 0;
        const auto floor = getShapes()[0].shapeHandle;
        for( const auto& contact : getContacts() ) if( contact.shapeA != floor && contact.shapeB != floor ) ++internalContacts;
        ImGui::Text( "신체 내부 접촉: %d", internalContacts );
        ImGui::TextWrapped( "설정을 바꾼 뒤 자세 복귀로 같은 낙하를 비교하세요. 복귀는 관절 제한·마찰·충돌 설정을 보존합니다. 마찰은 회전을 늦추며 특정 자세로 되돌리는 모터가 아닙니다. 자기 충돌을 끄면 Box2D처럼 같은 렉돌의 부위끼리 통과하지만 바닥 접촉은 유지합니다. 직접 연결된 부위는 항상 충돌하지 않습니다." );
        ImGui::TextWrapped( "A/D: 몸통 밀기 · Space: 위로 충격 · S: 회전 충격 · 왼쪽 드래그: 부위 잡기. 초록·빨강 선과 회색 호는 각 관절의 허용 각도입니다." );
    }
    else
    {
        ImGui::TextUnformatted( "충격량 실험" );

        const float circleMass = getWorld().GetBodyMass( getImpulseBody() );
        if( getKind() == demoKind::playground )
        {
            if( ImGui::Button( "설정 보존 위치 복귀###PlaygroundReset" ) ) resetPlayground();
            ImGui::TextWrapped( "A/D는 일정한 가속도를 주는 지속적인 힘입니다. 충격량은 질량에 비례시켜 밀도를 바꿔도 같은 속도 변화를 줍니다. 복귀는 물성·중력 설정을 보존하며 발판은 좌우로 왕복합니다." );
            ImGui::TextWrapped( "점프: J %.3f N·s, 속도 +5 m/s", circleMass * 5.0f );
            ImGui::TextWrapped( "중심 밖: J %.3f N·s, 수평 +4 m/s", circleMass * 4.0f );
        }

        if( ImGui::Button( "점프 충격량###Jump impulse", ImVec2( -1.0f, 0.0f ) ) )
        {
            // DeltaV = J / M = 5 m/s
            getWorld().ApplyLinearImpulseToCenter( getImpulseBody(), { 0.0f, circleMass * 5.0f } );
        }

        if( ImGui::Button( "중심 밖에서 밀기###Off-center kick", ImVec2( -1.0f, 0.0f ) ) )
        {
            const vec2 center = getWorldCenter( getWorld(), getImpulseBody() );

            // COM 위쪽을 오른쪽으로 밀어 translation + rotation을 동시에 확인함.
            getWorld().ApplyLinearImpulse( getImpulseBody(), { circleMass * 4.0f, 0.0f }, center + vec2{ 0.0f, 0.8f } );
        }

        const float boxInertia = getWorld().GetBodyRotationalInertia( getTorqueBody() );

        if( ImGui::Button( "회전 충격량###Spin box", ImVec2( -1.0f, 0.0f ) ) )
        {
            // DeltaW = L / I = 3 rad/s
            getWorld().ApplyAngularImpulse( getTorqueBody(), boxInertia * 3.0f );
        }

        const vec2 circleVelocity = getWorld().GetBodyLinearVelocity( getImpulseBody() );

        const float circleAngularVelocity = getWorld().GetBodyAngularVelocity( getImpulseBody() );

        ImGui::Text( "밀기 대상 선속도: (%.2f, %.2f)", circleVelocity.x, circleVelocity.y );
        ImGui::Text( "밀기 대상 각속도: %.2f rad/s", circleAngularVelocity );

        ImGui::Spacing();
        ImGui::Separator();
    }
}

void rigidBodyDemoUi::drawMouseControls()
{
    float hertz = getMouseSettings().hertz, damping = getMouseSettings().dampingRatio, force = getMouseSettings().maxForce;
    // Ctrl+click의 숫자 입력도 solver의 비음수 전제조건과 화면 범위를 지켜야 함.
    bool changed = ImGui::SliderFloat( "잡기 주파수###Mouse Hertz", &hertz, 0.0f, 30.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
    changed |= ImGui::SliderFloat( "잡기 감쇠 비율###Mouse damping", &damping, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
    changed |= ImGui::SliderFloat( "잡기 최대 힘###Mouse max force", &force, 0.0f, 5000.0f, "%.1f N", ImGuiSliderFlags_AlwaysClamp );
    if( changed )
    {
        setMouseSettings( hertz, damping, force );
    }
    ImGui::TextWrapped( "왼쪽 드래그로 동적 도형을 잡습니다. 주황색 선은 잡은 점과 목표점을 잇습니다. UI 조작·포커스 상실·캔버스 이탈 시 취소합니다. 오른쪽 클릭은 충격량 실험입니다." );
    ImGui::TextWrapped( "주파수는 점 스프링의 반응을 정하고 감쇠 비율은 진동을 줄입니다. 0 Hz에서는 속도 감쇠만 적용하며 최대 힘이 0이면 끌어당기지 않습니다." );
    if( getWorld().IsValid( getMouseJoint() ) )
    {
        const auto joint = getWorld().getMouseJointData( getMouseJoint() );
        ImGui::Text( "잡은 점의 오차: %.3f m / 힘: %.2f N", Length( joint.anchorB - joint.target ), Length( joint.force ) );
    }
    ImGui::Spacing();
}

void rigidBodyDemoUi::drawDebugSettings()
{
    ImGui::Checkbox( "격자와 축###Grid / Axis", &showGrid_ );
    ImGui::Checkbox( "도형 경계 상자###Shape AABBs", &showShapeAABBs_ );
    ImGui::Checkbox( "확장 경계 상자###Fat AABBs", &showFatAABBs_ );
    ImGui::Checkbox( "접촉점###Contact points", &showContacts_ );
    ImGui::Checkbox( "접촉 상세 정보###Contact details", &showContactDetails_ );
    bool contactEvents = isContactEventVisualizationEnabled();
    if( ImGui::Checkbox( "접촉 이벤트###Contact events", &contactEvents ) )
        setContactEventVisualizationEnabled( contactEvents );

    if( contactEvents )
    {
        float hitThreshold = getWorld().getContactHitSpeedThreshold();
        if( ImGui::DragFloat( "충격 최소 접근 속도###Hit speed threshold", &hitThreshold, 0.1f, 0.0f, 100.0f, "%.2f m/s" ) )
            getWorld().setContactHitSpeedThreshold( hitThreshold );

        float duration = getContactEventTrailDuration();
        if( ImGui::SliderFloat( "표시 유지 시간###Contact trace lifetime", &duration, 0.05f, 2.0f, "%.2f s" ) )
            setContactEventTrailDuration( duration );

        ImGui::Text( "최근 Step: 시작 %zu / 종료 %zu / 충격 %zu",
            getWorld().GetContactBeginEvents().size(),
            getWorld().GetContactEndEvents().size(),
            getWorld().getContactHitEvents().size() );
        ImGui::Text( "표시 중인 이벤트: %zu / 256", getContactEventTraces().size() );
        if( ImGui::Button( "이벤트 흔적 지우기###Clear contact traces" ) ) clearContactEventTrail();
    }
    ImGui::Checkbox( "질량 중심###Center of mass", &showCOM_ );
    ImGui::Checkbox( "속도 벡터###Velocity vectors", &showVelocities_ );
    ImGui::Checkbox( "이름 표시###Labels", &showLabels_ );

    ImGui::Spacing();
    ImGui::TextUnformatted( "경계 상자 트리" );

    ImGui::Checkbox( "동적 오브젝트 트리###Dynamic Tree", &showDynamicTree_ );
    ImGui::Checkbox( "키네마틱 오브젝트 트리###Kinematic Tree", &showKinematicTree_ );
    ImGui::Checkbox( "정적 오브젝트 트리###Static Tree", &showStaticTree_ );

    ImGui::Checkbox( "트리 잎 노드###Tree Leaves", &showTreeLeaves_ );
    ImGui::Checkbox( "트리 내부 노드###Tree Internal", &showTreeInternal_ );
    ImGui::Checkbox( "트리 노드 정보###Tree Labels", &showTreeLabels_ );

    const broadPhase& phase = getWorld().GetBroadPhase();

    const dynamicTree& dynamicTreeRef = phase.GetTree( bodyType::Dynamic );

    const dynamicTree& kinematicTree = phase.GetTree( bodyType::Kinematic );

    const dynamicTree& staticTree = phase.GetTree( bodyType::Static );

    ImGui::Spacing();

    ImGui::Text( "오브젝트 수: %zu", getWorld().GetBodyCount() );
    ImGui::Text( "도형 수: %zu", getWorld().GetShapeCount() );
    ImGui::Text( "유지 중인 접촉 수: %zu", getWorld().GetContactCount() );
    ImGui::Text( "실제로 닿은 접촉 수: %zu", getContacts().size() );
    ImGui::Text( "재사용한 접촉 수: %zu", getWorld().GetRecycledContactCount() );

    ImGui::Text( "동적 트리: %zu / 높이 %d", dynamicTreeRef.GetProxyCount(), dynamicTreeRef.GetHeight() );
    ImGui::Text( "키네마틱 트리: %zu / 높이 %d", kinematicTree.GetProxyCount(), kinematicTree.GetHeight() );
    ImGui::Text( "정적 트리: %zu / 높이 %d", staticTree.GetProxyCount(), staticTree.GetHeight() );

    ImGui::Spacing();

    ImGui::TextWrapped( "접촉 정보는 간격·수직 충격량·마찰 충격량을 표시합니다. 청록색 화살표는 속도이며 어두운 오브젝트는 수면 중입니다." );
    ImGui::TextWrapped( "이벤트 표시: 녹색 시작·주황색 종료는 발생 당시 두 Shape의 AABB 중심 사이에 남긴 근사 위치입니다. 빨간색 충격은 Solver 접점·법선입니다. 시간 경과에 따라 투명해지며 삭제된 Shape의 종료는 위치 없이 최근 Step 개수에만 포함됩니다." );
}

#pragma endregion Controls

bool initializeDemoUi()
{
    // 작업 디렉터리와 무관하게 실행 파일과 함께 배포한 한글 폰트를 읽음.
    wchar_t executable[32768];
    const DWORD length = GetModuleFileNameW( nullptr, executable, static_cast<DWORD>( std::size( executable ) ) );
    if( length == 0 || length >= std::size( executable ) ) return false;

    const auto path = std::filesystem::path( executable ).parent_path() / "assets" / "NanumGothic-Regular.ttf";
    if( !std::filesystem::exists( path ) ) return false;

    const auto utf8Path = path.u8string();
    ImGuiIO& io = ImGui::GetIO();
    if( !io.Fonts->AddFontFromFileTTF( reinterpret_cast<const char*>( utf8Path.c_str() ), 16.0f, nullptr, io.Fonts->GetGlyphRangesKorean() ) ) return false;

    // ImGui가 생성하는 공통 메뉴만 번역함. 내부 API이므로 vendored 버전에 맞춰 유지함.
    static const ImGuiLocEntry entries[] = {
        { ImGuiLocKey_TableSizeOne, "열 너비 맞추기###SizeOne" },
        { ImGuiLocKey_TableSizeAllFit, "모든 열 너비 맞추기###SizeAll" },
        { ImGuiLocKey_TableSizeAllDefault, "모든 열 너비 초기화###SizeAll" },
        { ImGuiLocKey_TableReset, "초기화" },
        { ImGuiLocKey_TableResetOrder, "순서 초기화###ResetOrder" },
        { ImGuiLocKey_TableResetVisibility, "표시 초기화###ResetVisibility" },
        { ImGuiLocKey_WindowingMainMenuBar, "(주 메뉴)" },
        { ImGuiLocKey_WindowingPopup, "(팝업)" },
        { ImGuiLocKey_WindowingUntitled, "(제목 없음)" },
        { ImGuiLocKey_OpenLink_s, "'%s' 열기" },
        { ImGuiLocKey_CopyLink, "링크 복사###CopyLink" }
    };
    ImGui::LocalizeRegisterEntries( entries, IM_ARRAYSIZE( entries ) );

    return true;
}

std::unique_ptr<demo> createDemoView( demoKind kind )
{
    return std::make_unique<rigidBodyDemoUi>( kind );
}

void drawProjectCollisionSettings( demoSession& session, collisionSettingsUi& state )
{
    if( ImGui::Button( "충돌 레이어 설정###Project collision matrix", ImVec2( -1.0f, 0.0f ) ) )
    {
        ImGui::OpenPopup( "충돌 레이어 설정###Project collision settings" );
    }
    const float minWidth = std::max( 460.0f, 140.0f + 56.0f * std::min( std::popcount( state.visibleLayers ), 8 ) );
    ImGui::SetNextWindowSizeConstraints( { minWidth, 0.0f }, { 640.0f, 900.0f } );
    if( ImGui::BeginPopupModal( "충돌 레이어 설정###Project collision settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize ) )
    {
        ImGui::TextUnformatted( "선택한 레이어 사이의 충돌 허용 여부를 편집합니다." );
        drawCollisionBits( "표시할 레이어###VisibleCollisionLayers", "VisibleLayers", state.visibleLayers, 8 );
        ImGui::TextUnformatted( "한 번에 8개까지 표시합니다. 숨긴 레이어의 규칙은 유지됩니다." );
        collisionMatrix matrix = session.getCollisionMatrix();
        bool changed = false;
        int layers[8]{}, count = 0;
        for( int bit = 0; bit < 64 && count < 8; ++bit )
        {
            if( ( state.visibleLayers & ( std::uint64_t{ 1 } << bit ) ) != 0 )
            {
                layers[count++] = bit;
            }
        }
        if( count == 0 )
        {
            ImGui::TextUnformatted( "편집할 레이어를 선택하세요." );
        }
        else
        {
            ImGui::BeginChild( "MatrixGrid", { 0.0f, ImGui::GetFrameHeightWithSpacing() * ( count + 1 ) + 8.0f } );
            if( ImGui::BeginTable( "CollisionPairs", count + 1, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit ) )
            {
                ImGui::TableSetupColumn( "레이어", ImGuiTableColumnFlags_WidthFixed, 90.0f );
                for( int column = 0; column < count; ++column )
                {
                    char label[8];
                    std::snprintf( label, sizeof( label ), "%02d", layers[column] + 1 );
                    ImGui::TableSetupColumn( label, ImGuiTableColumnFlags_WidthFixed, 44.0f );
                }
                ImGui::TableHeadersRow();
                for( int row = 0; row < count; ++row )
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex( 0 );
                    ImGui::Text( "레이어 %02d", layers[row] + 1 );
                    for( int column = 0; column < count; ++column )
                    {
                        ImGui::TableSetColumnIndex( column + 1 );
                        // 대칭 관계는 한 번만 편집함. 같은 레이어끼리의 충돌도 설정 가능함.
                        if( column < row )
                        {
                            ImGui::TextUnformatted( "—" );
                            continue;
                        }
                        char label[32];
                        std::snprintf( label, sizeof( label ), "##Pair%d_%d", layers[row], layers[column] );
                        bool allowed = matrix.allows( std::uint64_t{ 1 } << layers[row], std::uint64_t{ 1 } << layers[column] );
                        if( ImGui::Checkbox( label, &allowed ) )
                        {
                            matrix.setPair( layers[row], layers[column], allowed );
                            changed = true;
                        }
                        ImGui::SetItemTooltip( "레이어 %02d / 레이어 %02d", layers[row] + 1, layers[column] + 1 );
                    }
                }
                ImGui::EndTable();
            }
            ImGui::EndChild();
        }
        if( changed )
        {
            session.setCollisionMatrix( matrix );
        }
        if( ImGui::CollapsingHeader( "규칙 설명###CollisionRulesHelp" ) )
        {
            ImGui::TextWrapped( "체크한 쌍은 충돌을 허용합니다. 같은 관계는 위쪽에서 한 번만 편집하며 개별 마스크는 추가 제한입니다. 모든 데모에 적용되고 현재 실행 중에는 전환·초기화 후에도 유지됩니다." );
        }
        if( ImGui::Button( "닫기###Close" ) )
        {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

} // namespace zonai::sandbox
