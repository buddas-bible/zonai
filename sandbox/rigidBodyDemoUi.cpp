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
    selectedShapeIndex_ = kind == demoKind::playground || kind == demoKind::motorCar ? 2 : 1;
    if( getShapes().size() > static_cast<std::size_t>( selectedShapeIndex_ ) ) selectedShape_ = getShapes()[selectedShapeIndex_].shapeHandle;
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

        const shapeColors colors = getShapeColors( bodyRef.type, bodyRef.awake );

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
            draw.DrawLabel( transform.position, visual.label, LABEL_COLOR );
        }
    }

    if( getKind() == demoKind::splineTerrain && !getShapes().empty() && getWorld().IsValid( getShapes()[0].bodyHandle ) )
    {
        const auto pose = getWorld().GetBodyTransform( getShapes()[0].bodyHandle );
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

    if( getWorld().IsValid( getMouseJoint() ) )
    {
        const auto joint = getWorld().getMouseJointData( getMouseJoint() );
        constexpr ImU32 MOUSE_COLOR = IM_COL32( 255, 170, 70, 255 );
        draw.DrawSegment( { joint.anchorB, joint.target }, MOUSE_COLOR );
        draw.DrawPoint( joint.anchorB, MOUSE_COLOR );
        draw.DrawPoint( joint.target, MOUSE_COLOR );
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

    for( const jointId id : { getWheelJoint(), getCarJoints()[0], getCarJoints()[1] } )
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
        draw.DrawSegment( { getWorld().GetBodyTransform( joint.bodyA ).position, joint.anchorB }, WHEEL_COLOR );
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

void rigidBodyDemoUi::drawExperimentControls()
{
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
            ImGui::BeginDisabled( !canUndoSplineEdit() );
            if( ImGui::Button( "되돌리기###SplineUndo" ) ) undoSplineEdit();
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled( !canRedoSplineEdit() );
            if( ImGui::Button( "다시하기###SplineRedo" ) ) redoSplineEdit();
            ImGui::EndDisabled();
            if( ImGui::Button( "지형 원형 복원###SplineTerrainReset" ) ) resetSplineTerrain();
            float error = getSplineSamplingError() * 100.0f;
            const bool errorChanged = ImGui::SliderFloat( "근사 오차###SplineSamplingError", &error, 0.5f, 5.0f, "%.2f cm", ImGuiSliderFlags_AlwaysClamp );
            if( ImGui::IsItemActivated() ) beginSplineSamplingEdit();
            if( errorChanged ) setSplineSamplingError( error / 100.0f );
            if( ImGui::IsItemDeactivated() ) endSplineSamplingEdit();
            ImGui::Text( "선분 수: %zu", getTerrainPointCount() - 1 );
            ImGui::TextWrapped( "충돌 선분이 너무 짧아지는 오차 변경이나 제어점 이동은 적용하지 않습니다." );
            const auto controls = getSplineControlPoints();
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
            ImGui::BeginDisabled( splineControlIndex_ + 1 >= controls.size() );
            if( ImGui::Button( "다음 구간에 추가###SplineInsert" ) && insertSplineControlPoint( splineControlIndex_ ) ) ++splineControlIndex_;
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled( getSplineControlPoints().size() <= 2 );
            if( ImGui::Button( "제어점 삭제###SplineRemove" ) && removeSplineControlPoint( splineControlIndex_ ) ) splineControlIndex_ = std::min( splineControlIndex_, getSplineControlPoints().size() - 1 );
            ImGui::EndDisabled();
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
                ImGui::Text( "발사 물체 물성: 마찰 %.3f / 반발 %.3f", value.launchFriction, value.launchRestitution );
                ImGui::Text( "발사 물체 질량: %.3f kg", value.launchMass );
                ImGui::Text( "경과: %.3f s / 지형 접촉: %s", value.elapsed, value.touching ? "있음" : "없음" );
                if( value.firstContactTime )
                {
                    ImGui::Text( "첫 접촉 관측: %.3f s", *value.firstContactTime );
                    ImGui::Text( "관측 단계: %.3f ms / 내부 분할 %d", value.firstContactTimeStep * 1000.0f, value.firstContactSubStepCount );
                    ImGui::Text( "첫 접촉까지 진행 설정 변경: %s", value.preContactSettingsChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 중력 변경: %s", value.preContactGravityChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 물체 물성 변경: %s", value.preContactMaterialChanged ? "있음" : "없음" );
                    ImGui::Text( "첫 접촉까지 관측된 물체 질량 변경: %s", value.preContactMassChanged ? "있음" : "없음" );
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
                    ImGui::Text( "첫 접촉 관측 차이 (현재 - 기준): %+.3f s", comparison.firstContactTimeDifference );
                    break;
            }
            ImGui::TextWrapped( "비교는 기록된 발사 조건·중력·물체 물성·질량과 첫 접촉 관측 단계 설정만 확인합니다. 전체 진행 이력이나 지형 물성·발사 이후 변경까지 같다는 뜻은 아닙니다." );
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
