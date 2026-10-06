#include "rigidBodyDemoUi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui.h>

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
        return "Static";

    case bodyType::Kinematic:
        return "Kinematic";

    case bodyType::Dynamic:
        return "Dynamic";

    default:
        return "Unknown";
    }
}

shapeColors getShapeColors( bodyType type, bool awake )
{
    if( type == bodyType::Dynamic && !awake )
    {
        return
        {
            IM_COL32( 100, 125, 145, 220 ),
            IM_COL32( 100, 125, 145, 45 )
        };
    }

    switch( type )
    {
    case bodyType::Static:
        return
        {
            IM_COL32( 120, 220, 140, 255 ),
            IM_COL32( 120, 220, 140, 55 )
        };

    case bodyType::Kinematic:
        return
        {
            IM_COL32( 245, 205, 90, 255 ),
            IM_COL32( 245, 205, 90, 60 )
        };

    case bodyType::Dynamic:
        return
        {
            IM_COL32( 90, 190, 255, 255 ),
            IM_COL32( 90, 190, 255, 70 )
        };

    default:
        return
        {
            IM_COL32( 230, 230, 230, 255 ),
            IM_COL32( 230, 230, 230, 50 )
        };
    }
}

vec2 getWorldCenter(
    const world& world,
    bodyId bodyId )
{
    return TransformPoint(
        world.GetBodyTransform( bodyId ),
        world.GetBodyLocalCenter( bodyId )
    );
}

} // namespace

#pragma region DemoView
rigidBodyDemoUi::rigidBodyDemoUi( demoKind kind ) : rigidBodyDemo( kind )
{
    selectedShapeIndex_ = kind == demoKind::playground ? 2 : 1;
}

void rigidBodyDemoUi::drawControls()
{
    drawWorldSettings();
    drawInspector();
    drawExperimentControls();
    drawMouseControls();
    drawDebugSettings();
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

    constexpr ImU32 DYNAMIC_TREE_LEAF =
        IM_COL32( 70, 200, 255, 220 );
    constexpr ImU32 DYNAMIC_TREE_INTERNAL =
        IM_COL32( 80, 120, 255, 140 );

    constexpr ImU32 KINEMATIC_TREE_LEAF =
        IM_COL32( 245, 205, 90, 220 );
    constexpr ImU32 KINEMATIC_TREE_INTERNAL =
        IM_COL32( 210, 165, 70, 130 );

    constexpr ImU32 STATIC_TREE_LEAF =
        IM_COL32( 110, 220, 130, 220 );
    constexpr ImU32 STATIC_TREE_INTERNAL =
        IM_COL32( 80, 160, 100, 130 );

    if( showStaticTree_ )
    {
        draw.DrawTree(
            staticTree,
            "S",
            showTreeLeaves_,
            showTreeInternal_,
            showTreeLabels_,
            STATIC_TREE_LEAF,
            STATIC_TREE_INTERNAL
        );
    }

    if( showKinematicTree_ )
    {
        draw.DrawTree(
            kinematicTree,
            "K",
            showTreeLeaves_,
            showTreeInternal_,
            showTreeLabels_,
            KINEMATIC_TREE_LEAF,
            KINEMATIC_TREE_INTERNAL
        );
    }

    if( showDynamicTree_ )
    {
        draw.DrawTree(
            dynamicTreeRef,
            "D",
            showTreeLeaves_,
            showTreeInternal_,
            showTreeLabels_,
            DYNAMIC_TREE_LEAF,
            DYNAMIC_TREE_INTERNAL
        );
    }

    constexpr ImU32 AABB_COLOR =
        IM_COL32( 210, 100, 230, 210 );

    constexpr ImU32 FAT_AABB_COLOR =
        IM_COL32( 255, 185, 80, 190 );

    constexpr ImU32 LABEL_COLOR =
        IM_COL32( 230, 232, 238, 255 );

    constexpr ImU32 COM_COLOR =
        IM_COL32( 255, 90, 180, 255 );

    constexpr ImU32 VELOCITY_COLOR =
        IM_COL32( 80, 220, 255, 245 );

    for( const visualShape& visual : getShapes() )
    {
        if( !getWorld().IsValid( visual.bodyHandle ) || !getWorld().IsValid( visual.shapeHandle ) ) { continue; }
        if( !getWorld().IsValid( visual.bodyHandle ) || !getWorld().IsValid( visual.shapeHandle ) )
        { ImGui::TextUnformatted( "Selected object was deleted." ); return; }
        const body& bodyRef =
            getWorld().GetBody(
                visual.bodyHandle
            );

        const shape& shapeRef =
            getWorld().GetShape(
                visual.shapeHandle
            );

        const transform2 transform =
            getWorld().GetBodyTransform(
                visual.bodyHandle
            );

        const shapeColors colors =
            getShapeColors(
                bodyRef.type,
                bodyRef.awake
            );

        draw.DrawShape(
            shapeRef.geometry,
            transform,
            colors.outline,
            colors.fill
        );

        if( showShapeAABBs_ )
        {
            draw.DrawAABB(
                getWorld().GetShapeAABB(
                    visual.shapeHandle
                ),
                AABB_COLOR
            );
        }

        if( showFatAABBs_ )
        {
            draw.DrawAABB(
                getWorld().GetShapeFatAABB(
                    visual.shapeHandle
                ),
                FAT_AABB_COLOR
            );
        }

        const vec2 worldCenter =
            getWorldCenter(
                getWorld(),
                visual.bodyHandle
            );

        if( showCOM_ &&
            bodyRef.type != bodyType::Static )
        {
            draw.DrawPoint(
                worldCenter,
                COM_COLOR,
                4.5f
            );
        }

        if( showVelocities_ &&
            bodyRef.type != bodyType::Static )
        {
            const vec2 velocity =
                getWorld().GetBodyLinearVelocity(
                    visual.bodyHandle
                );

            if( LengthSquared( velocity ) > 1e-6f )
            {
                const float arrowLength =
                    std::clamp(
                        Length( velocity ) * 0.18f,
                        0.2f,
                        2.0f
                    );

                draw.DrawArrow(
                    worldCenter,
                    velocity,
                    VELOCITY_COLOR,
                    arrowLength
                );
            }
        }

        if( showLabels_ )
        {
            draw.DrawLabel(
                transform.position,
                visual.label,
                LABEL_COLOR
            );
        }
    }

    if( getWorld().IsValid( getMouseJoint() ) )
    {
        const auto joint = getWorld().getMouseJointData( getMouseJoint() );
        constexpr ImU32 MOUSE_COLOR = IM_COL32( 255, 170, 70, 255 );
        draw.DrawSegment( { joint.anchorB, joint.target }, MOUSE_COLOR );
        draw.DrawPoint( joint.anchorB, MOUSE_COLOR ); draw.DrawPoint( joint.target, MOUSE_COLOR );
    }
    if( getWorld().IsValid( getPendulumJoint() ) )
    {
        constexpr ImU32 JOINT_COLOR = IM_COL32( 230, 170, 255, 255 );
        const distanceJointData joint = getWorld().getDistanceJointData( getPendulumJoint() );
        draw.DrawSegment( { joint.anchorA, joint.anchorB }, JOINT_COLOR );
        draw.DrawPoint( joint.anchorA, JOINT_COLOR );
        draw.DrawPoint( joint.anchorB, JOINT_COLOR );
        if( joint.enableSpring && joint.enableLimit && joint.minLength < joint.maxLength )
        {
            const vec2 axis = Normalize( joint.anchorB - joint.anchorA );
            const vec2 lower = joint.anchorA + joint.minLength * axis, upper = joint.anchorA + joint.maxLength * axis;
            draw.DrawSegment( { lower, upper }, IM_COL32( 160, 160, 160, 255 ) );
            draw.DrawPoint( lower, IM_COL32( 100, 255, 140, 255 ) ); draw.DrawPoint( upper, IM_COL32( 255, 105, 90, 255 ) );
        }
    }

    if( showContacts_ )
    {
        constexpr ImU32 CONTACT_COLOR =
            IM_COL32( 255, 220, 70, 255 );

        constexpr ImU32 PENETRATION_COLOR =
            IM_COL32( 255, 105, 90, 255 );

        constexpr ImU32 NORMAL_COLOR =
            IM_COL32( 100, 255, 140, 255 );

        constexpr ImU32 CONTACT_TEXT_COLOR =
            IM_COL32( 245, 245, 245, 255 );

        for( const contactData& contact : getContacts() )
        {
            for( int i = 0;
                 i < contact.manifold.pointCount;
                 ++i )
            {
                const manifoldPoint2& point =
                    contact.manifold.points[i];

                const ImU32 pointColor =
                    point.separation < -0.01f
                        ? PENETRATION_COLOR
                        : CONTACT_COLOR;

                draw.DrawPoint(
                    point.point,
                    pointColor,
                    5.0f
                );

                draw.DrawArrow(
                    point.point,
                    contact.manifold.normal,
                    NORMAL_COLOR,
                    0.7f
                );

                if( showContactDetails_ )
                {
                    char label[96]{};

                    std::snprintf(
                        label,
                        sizeof( label ),
                        "s %.3f  Jn %.2f  Jt %.2f",
                        point.separation,
                        point.normalImpulse,
                        point.tangentImpulse
                    );

                    draw.DrawLabel(
                        point.point,
                        label,
                        CONTACT_TEXT_COLOR
                    );
                }
            }
        }
    }

}
#pragma endregion

#pragma region Controls
void rigidBodyDemoUi::drawWorldSettings()
{
    vec2 gravity =
        getWorld().GetGravity();

    float gravityValues[2]
    {
        gravity.x,
        gravity.y
    };

    if( ImGui::DragFloat2(
        "Gravity",
        gravityValues,
        0.1f,
        -30.0f,
        30.0f,
        "%.2f" ) )
    {
        getWorld().SetGravity(
            {
                gravityValues[0],
                gravityValues[1]
            }
        );
    }


    float maximumLinearSpeed =
        getWorld().GetMaximumLinearSpeed();

    if( ImGui::DragFloat(
        "Max linear speed",
        &maximumLinearSpeed,
        1.0f,
        1.0f,
        1000.0f,
        "%.1f m/s" ) )
    {
        getWorld().SetMaximumLinearSpeed(
            maximumLinearSpeed
        );
    }

    bool sleepingEnabled =
        getWorld().IsSleepingEnabled();

    if( ImGui::Checkbox(
        "Enable sleeping",
        &sleepingEnabled ) )
    {
        getWorld().SetSleepingEnabled(
            sleepingEnabled
        );
    }

    bool continuousEnabled =
        getWorld().IsContinuousEnabled();

    if( ImGui::Checkbox(
        "Continuous collision",
        &continuousEnabled ) )
    {
        getWorld().SetContinuousEnabled(
            continuousEnabled
        );
    }

    float contactRecycleDistance =
        getWorld().GetContactRecycleDistance();

    if( ImGui::DragFloat(
        "Contact recycle dist",
        &contactRecycleDistance,
        0.001f,
        0.0f,
        0.2f,
        "%.3f m" ) )
    {
        getWorld().SetContactRecycleDistance(
            contactRecycleDistance
        );
    }

}

void rigidBodyDemoUi::drawInspector()
{
    ImGui::Separator();
    ImGui::TextUnformatted(
        "Object Inspector"
    );

    if( !getShapes().empty() )
    {
        selectedShapeIndex_ =
            std::clamp(
                selectedShapeIndex_,
                0,
                static_cast<int>( getShapes().size() ) - 1
            );

        const visualShape& selectedVisual =
            getShapes()[selectedShapeIndex_];

        if( ImGui::BeginCombo(
            "Object",
            selectedVisual.label ) )
        {
            for( int i = 0;
                 i < static_cast<int>( getShapes().size() );
                 ++i )
            {
                const bool selected =
                    i == selectedShapeIndex_;

                if( ImGui::Selectable(
                    getShapes()[i].label,
                    selected ) )
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

        const visualShape& visual =
            getShapes()[selectedShapeIndex_];

        if( !getWorld().IsValid( visual.bodyHandle ) || !getWorld().IsValid( visual.shapeHandle ) )
        { ImGui::TextUnformatted( "Selected object was deleted." ); return; }
        const body& bodyRef =
            getWorld().GetBody(
                visual.bodyHandle
            );

        ImGui::Text(
            "Body type: %s",
            getBodyTypeName( bodyRef.type )
        );

        transform2 transform =
            getWorld().GetBodyTransform(
                visual.bodyHandle
            );

        float position[2]
        {
            transform.position.x,
            transform.position.y
        };

        bool transformChanged = false;

        if( ImGui::DragFloat2(
            "Position",
            position,
            0.05f,
            -100.0f,
            100.0f,
            "%.2f" ) )
        {
            transform.position =
            {
                position[0],
                position[1]
            };

            transformChanged = true;
        }

        float rotation =
            std::atan2(
                transform.rotation.s,
                transform.rotation.c
            );

        if( ImGui::DragFloat(
            "Rotation",
            &rotation,
            0.01f,
            -3.14159265f,
            3.14159265f,
            "%.3f rad" ) )
        {
            transform.rotation =
                rot2::FromRadians(
                    rotation
                );

            transformChanged = true;
        }

        if( transformChanged )
        {
            getWorld().SetBodyTransform(
                visual.bodyHandle,
                transform
            );

            refreshContacts();
        }

        if( bodyRef.type != bodyType::Static )
        {
            vec2 linearVelocity =
                getWorld().GetBodyLinearVelocity(
                    visual.bodyHandle
                );

            float velocity[2]
            {
                linearVelocity.x,
                linearVelocity.y
            };

            if( ImGui::DragFloat2(
                "Linear velocity",
                velocity,
                0.05f,
                -100.0f,
                100.0f,
                "%.2f" ) )
            {
                getWorld().SetBodyLinearVelocity(
                    visual.bodyHandle,
                    {
                        velocity[0],
                        velocity[1]
                    }
                );
            }

            float angularVelocity =
                getWorld().GetBodyAngularVelocity(
                    visual.bodyHandle
                );

            if( ImGui::DragFloat(
                "Angular velocity",
                &angularVelocity,
                0.05f,
                -100.0f,
                100.0f,
                "%.2f rad/s" ) )
            {
                getWorld().SetBodyAngularVelocity(
                    visual.bodyHandle,
                    angularVelocity
                );
            }

            float linearDamping =
                getWorld().GetBodyLinearDamping(
                    visual.bodyHandle
                );

            if( ImGui::DragFloat(
                "Linear damping",
                &linearDamping,
                0.05f,
                0.0f,
                20.0f,
                "%.2f" ) )
            {
                getWorld().SetBodyLinearDamping(
                    visual.bodyHandle,
                    linearDamping
                );
            }

            float angularDamping =
                getWorld().GetBodyAngularDamping(
                    visual.bodyHandle
                );

            if( ImGui::DragFloat(
                "Angular damping",
                &angularDamping,
                0.05f,
                0.0f,
                20.0f,
                "%.2f" ) )
            {
                getWorld().SetBodyAngularDamping(
                    visual.bodyHandle,
                    angularDamping
                );
            }

            float gravityScale =
                getWorld().GetBodyGravityScale(
                    visual.bodyHandle
                );

            if( ImGui::DragFloat(
                "Gravity scale",
                &gravityScale,
                0.05f,
                -10.0f,
                10.0f,
                "%.2f" ) )
            {
                getWorld().SetBodyGravityScale(
                    visual.bodyHandle,
                    gravityScale
                );
            }

            bool awake =
                getWorld().IsBodyAwake(
                    visual.bodyHandle
                );

            if( ImGui::Checkbox(
                "Awake",
                &awake ) )
            {
                getWorld().SetBodyAwake(
                    visual.bodyHandle,
                    awake
                );
            }

            bool sleepEnabled =
                getWorld().IsBodySleepEnabled(
                    visual.bodyHandle
                );

            if( ImGui::Checkbox(
                "Body sleep",
                &sleepEnabled ) )
            {
                getWorld().SetBodySleepEnabled(
                    visual.bodyHandle,
                    sleepEnabled
                );
            }

            float sleepThreshold =
                getWorld().GetBodySleepThreshold(
                    visual.bodyHandle
                );

            if( ImGui::DragFloat(
                "Sleep threshold",
                &sleepThreshold,
                0.005f,
                0.0f,
                5.0f,
                "%.3f m/s" ) )
            {
                getWorld().SetBodySleepThreshold(
                    visual.bodyHandle,
                    sleepThreshold
                );
            }

            float safetyFactor =
                getWorld().GetBodySafetyFactor(
                    visual.bodyHandle
                );

            if( ImGui::DragFloat(
                "CCD safety factor",
                &safetyFactor,
                0.01f,
                0.01f,
                2.0f,
                "%.2f" ) )
            {
                getWorld().SetBodySafetyFactor(
                    visual.bodyHandle,
                    safetyFactor
                );
            }

            bool contactRecycling =
                getWorld().IsBodyContactRecyclingEnabled(
                    visual.bodyHandle
                );

            if( ImGui::Checkbox(
                "Contact recycling",
                &contactRecycling ) )
            {
                getWorld().SetBodyContactRecyclingEnabled(
                    visual.bodyHandle,
                    contactRecycling
                );
            }

            bool bullet =
                getWorld().IsBodyBullet(
                    visual.bodyHandle
                );

            if( ImGui::Checkbox(
                "Bullet",
                &bullet ) )
            {
                getWorld().SetBodyBullet(
                    visual.bodyHandle,
                    bullet
                );
            }

            ImGui::Text(
                "Fast body: %s",
                getWorld().IsBodyFast(
                    visual.bodyHandle
                ) ? "yes" : "no"
            );

            ImGui::Text(
                "TOI this step: %s",
                getWorld().HadBodyTimeOfImpact(
                    visual.bodyHandle
                ) ? "yes" : "no"
            );

            bool fastRotation =
                getWorld().IsBodyFastRotationAllowed(
                    visual.bodyHandle
                );

            if( ImGui::Checkbox(
                "Allow fast rotation",
                &fastRotation ) )
            {
                getWorld().SetBodyFastRotationAllowed(
                    visual.bodyHandle,
                    fastRotation
                );
            }
        }

        float density =
            getWorld().GetShapeDensity(
                visual.shapeHandle
            );

        if( ImGui::DragFloat(
            "Density",
            &density,
            0.05f,
            0.0f,
            100.0f,
            "%.2f" ) )
        {
            getWorld().SetShapeDensity(
                visual.shapeHandle,
                density
            );
        }

        float friction =
            getWorld().GetShapeFriction(
                visual.shapeHandle
            );

        if( ImGui::DragFloat(
            "Friction",
            &friction,
            0.02f,
            0.0f,
            5.0f,
            "%.2f" ) )
        {
            getWorld().SetShapeFriction(
                visual.shapeHandle,
                friction
            );
        }

        float restitution =
            getWorld().GetShapeRestitution(
                visual.shapeHandle
            );

        if( ImGui::DragFloat(
            "Restitution",
            &restitution,
            0.02f,
            0.0f,
            2.0f,
            "%.2f" ) )
        {
            getWorld().SetShapeRestitution(
                visual.shapeHandle,
                restitution
            );
        }

        const bool isSensor =
            getWorld().IsShapeSensor(
                visual.shapeHandle
            );

        ImGui::Text(
            "Sensor: %s",
            isSensor ? "yes" : "no"
        );

        bool sensorEventsEnabled =
            getWorld().AreShapeSensorEventsEnabled(
                visual.shapeHandle
            );

        if( ImGui::Checkbox(
                "Sensor events",
                &sensorEventsEnabled ) )
        {
            getWorld().SetShapeSensorEventsEnabled(
                visual.shapeHandle,
                sensorEventsEnabled
            );
        }

        if( isSensor )
        {
            ImGui::Text(
                "Sensor overlaps: %zu",
                getWorld().GetShapeSensorCapacity(
                    visual.shapeHandle
                )
            );
        }

        collisionFilter filter =
            getWorld().GetShapeFilter(
                visual.shapeHandle
            );

        bool filterChanged = false;

        filterChanged =
            ImGui::InputScalar(
                "Category bits",
                ImGuiDataType_U64,
                &filter.categoryBits
            ) ||
            filterChanged;

        filterChanged =
            ImGui::InputScalar(
                "Mask bits",
                ImGuiDataType_U64,
                &filter.maskBits
            ) ||
            filterChanged;

        filterChanged =
            ImGui::InputInt(
                "Group index",
                &filter.groupIndex
            ) ||
            filterChanged;

        if( filterChanged )
        {
            getWorld().SetShapeFilter(
                visual.shapeHandle,
                filter
            );

            refreshContacts();
        }

        ImGui::Text(
            "Mass: %.3f",
            getWorld().GetBodyMass(
                visual.bodyHandle
            )
        );

        ImGui::Text(
            "Inertia: %.3f",
            getWorld().GetBodyRotationalInertia(
                visual.bodyHandle
            )
        );
    }

    ImGui::Spacing();
    ImGui::Separator();
}

void rigidBodyDemoUi::drawExperimentControls()
{
    if( getKind() == demoKind::distancePendulum && !getWorld().IsValid( getPendulumJoint() ) ) { return; }
    if( getKind() == demoKind::playground && ( !getWorld().IsValid( getImpulseBody() ) || !getWorld().IsValid( getTorqueBody() ) ) ) { return; }
    if( getKind() == demoKind::distancePendulum )
    {
        ImGui::TextUnformatted( "Distance Joint" );
        distanceJointData pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
        bool enableSpring = pendulum.enableSpring;
        float hertz = pendulum.hertz, dampingRatio = pendulum.dampingRatio;
        bool changed = ImGui::Checkbox( "Distance spring", &enableSpring );
        changed |= ImGui::SliderFloat( "Distance Hertz", &hertz, 0.0f, 30.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
        changed |= ImGui::SliderFloat( "Distance damping", &dampingRatio, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
        if( changed )
        {
            getWorld().setDistanceJointSpring( getPendulumJoint(), enableSpring, hertz, dampingRatio );
            pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
        }
        bool enableLimit = pendulum.enableLimit;
        float minLength = pendulum.minLength, maxLength = pendulum.maxLength;
        bool limitChanged = ImGui::Checkbox( "Distance limit", &enableLimit );
        limitChanged |= ImGui::SliderFloat( "Distance min", &minLength, LINEAR_SLOP, maxLength, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
        limitChanged |= ImGui::SliderFloat( "Distance max", &maxLength, minLength, 4.0f, "%.2f m", ImGuiSliderFlags_AlwaysClamp );
        if( limitChanged )
        {
            getWorld().setDistanceJointLimit( getPendulumJoint(), enableLimit, minLength, maxLength );
            pendulum = getWorld().getDistanceJointData( getPendulumJoint() );
        }
        const bool rigid = !enableSpring || ( enableLimit && minLength == maxLength );
        ImGui::TextUnformatted( rigid ? "Rigid distance at Target" : hertz > 0.0f ? "Spring distance" : enableLimit ? "Limit only (0 Hz)" : "Free distance axis (0 Hz)" );
        ImGui::Text( "Target: %.3f m / Current: %.3f m", pendulum.length, pendulum.currentLength );
        ImGui::Text( "Extension: %.3f m / Axial force: %.2f N", pendulum.currentLength - pendulum.length, pendulum.axialForce );
        ImGui::TextWrapped( "Hertz controls spring stiffness; damping controls oscillation. Negative force is tension. Limits need spring enabled; 0 Hz keeps limits active. Equal limits use rigid Target." );
        if( enableLimit && !rigid ) { ImGui::TextWrapped( "Green: min / Red: max. Limits correct softly; small errors can remain under load." ); }
        if( ImGui::Button( "Kick pendulum", ImVec2( -1.0f, 0.0f ) ) )
        {
            getWorld().ApplyLinearImpulseToCenter( getPendulumBody(), { getWorld().GetBodyMass( getPendulumBody() ) * 2.0f, 0.0f } );
        }
        if( ImGui::Button( "Radial kick", ImVec2( -1.0f, 0.0f ) ) )
        {
            const vec2 axis = Normalize( pendulum.anchorB - pendulum.anchorA );
            getWorld().ApplyLinearImpulseToCenter( getPendulumBody(), getWorld().GetBodyMass( getPendulumBody() ) * 2.0f * axis );
        }
        ImGui::Spacing();
    }
    else
    {
        ImGui::TextUnformatted(
            "Impulse Test"
        );

        const float circleMass =
            getWorld().GetBodyMass(
                getImpulseBody()
            );

        if( ImGui::Button(
            "Jump impulse",
            ImVec2( -1.0f, 0.0f ) ) )
        {
            // DeltaV = J / M = 5 m/s
            getWorld().ApplyLinearImpulseToCenter(
                getImpulseBody(),
                { 0.0f, circleMass * 5.0f }
            );
        }

        if( ImGui::Button(
            "Off-center kick",
            ImVec2( -1.0f, 0.0f ) ) )
        {
            const vec2 center =
                getWorldCenter(
                    getWorld(),
                    getImpulseBody()
                );

            // COM 위쪽을 오른쪽으로 밀어 translation + rotation을 동시에 확인함.
            getWorld().ApplyLinearImpulse(
                getImpulseBody(),
                { circleMass * 4.0f, 0.0f },
                center + vec2{ 0.0f, 0.8f }
            );
        }

        const float boxInertia =
            getWorld().GetBodyRotationalInertia(
                getTorqueBody()
            );

        if( ImGui::Button(
            "Spin box",
            ImVec2( -1.0f, 0.0f ) ) )
        {
            // DeltaW = L / I = 3 rad/s
            getWorld().ApplyAngularImpulse(
                getTorqueBody(),
                boxInertia * 3.0f
            );
        }

        const vec2 circleVelocity =
            getWorld().GetBodyLinearVelocity(
                getImpulseBody()
            );

        const float circleAngularVelocity =
            getWorld().GetBodyAngularVelocity(
                getImpulseBody()
            );

        ImGui::Text(
            "Circle v: (%.2f, %.2f)",
            circleVelocity.x,
            circleVelocity.y
        );
        ImGui::Text(
            "Circle w: %.2f rad/s",
            circleAngularVelocity
        );

        ImGui::Spacing();
        ImGui::Separator();
    }
}

void rigidBodyDemoUi::drawMouseControls()
{
    ImGui::Separator(); ImGui::TextUnformatted( "Mouse Joint" );
    float hertz = getMouseSettings().hertz, damping = getMouseSettings().dampingRatio, force = getMouseSettings().maxForce;
    // Ctrl+click의 숫자 입력도 solver의 비음수 전제조건과 화면 범위를 지켜야 함.
    bool changed = ImGui::SliderFloat( "Mouse Hertz", &hertz, 0.0f, 30.0f, "%.1f Hz", ImGuiSliderFlags_AlwaysClamp );
    changed |= ImGui::SliderFloat( "Mouse damping", &damping, 0.0f, 2.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp );
    changed |= ImGui::SliderFloat( "Mouse max force", &force, 0.0f, 5000.0f, "%.1f N", ImGuiSliderFlags_AlwaysClamp );
    if( changed ) { setMouseSettings( hertz, damping, force ); }
    ImGui::TextWrapped( "Left drag: pick a Dynamic solid. Orange line: grabbed point to target. UI/focus/canvas exit cancels drag. Right click: impulse experiment." );
    ImGui::TextWrapped( "Hertz sets the point spring response, damping reduces oscillation. Zero Hertz gives velocity damping only; zero force disables the pull." );
    if( getWorld().IsValid( getMouseJoint() ) )
    {
        const auto joint = getWorld().getMouseJointData( getMouseJoint() );
        ImGui::Text( "Point error: %.3f m / Force: %.2f N", Length( joint.anchorB - joint.target ), Length( joint.force ) );
    }
    ImGui::Spacing();
}

void rigidBodyDemoUi::drawDebugSettings()
{
    ImGui::TextUnformatted(
        "Debug Draw"
    );

    ImGui::Checkbox(
        "Grid / Axis",
        &showGrid_
    );
    ImGui::Checkbox(
        "Shape AABBs",
        &showShapeAABBs_
    );
    ImGui::Checkbox(
        "Fat AABBs",
        &showFatAABBs_
    );
    ImGui::Checkbox(
        "Contact points",
        &showContacts_
    );
    ImGui::Checkbox(
        "Contact details",
        &showContactDetails_
    );
    ImGui::Checkbox(
        "Center of mass",
        &showCOM_
    );
    ImGui::Checkbox(
        "Velocity vectors",
        &showVelocities_
    );
    ImGui::Checkbox(
        "Labels",
        &showLabels_
    );

    ImGui::Spacing();
    ImGui::TextUnformatted(
        "AABB Tree"
    );

    ImGui::Checkbox(
        "Dynamic Tree",
        &showDynamicTree_
    );
    ImGui::Checkbox(
        "Kinematic Tree",
        &showKinematicTree_
    );
    ImGui::Checkbox(
        "Static Tree",
        &showStaticTree_
    );

    ImGui::Checkbox(
        "Tree Leaves",
        &showTreeLeaves_
    );
    ImGui::Checkbox(
        "Tree Internal",
        &showTreeInternal_
    );
    ImGui::Checkbox(
        "Tree Labels",
        &showTreeLabels_
    );

    const broadPhase& phase =
        getWorld().GetBroadPhase();

    const dynamicTree& dynamicTreeRef =
        phase.GetTree(
            bodyType::Dynamic
        );

    const dynamicTree& kinematicTree =
        phase.GetTree(
            bodyType::Kinematic
        );

    const dynamicTree& staticTree =
        phase.GetTree(
            bodyType::Static
        );

    ImGui::Spacing();

    ImGui::Text(
        "Bodies: %zu",
        getWorld().GetBodyCount()
    );
    ImGui::Text(
        "Shapes: %zu",
        getWorld().GetShapeCount()
    );
    ImGui::Text(
        "Persistent contacts: %zu",
        getWorld().GetContactCount()
    );
    ImGui::Text(
        "Touching contacts: %zu",
        getContacts().size()
    );
    ImGui::Text(
        "Recycled contacts: %zu",
        getWorld().GetRecycledContactCount()
    );

    ImGui::Text(
        "Dynamic tree: %zu / h%d",
        dynamicTreeRef.GetProxyCount(),
        dynamicTreeRef.GetHeight()
    );
    ImGui::Text(
        "Kinematic tree: %zu / h%d",
        kinematicTree.GetProxyCount(),
        kinematicTree.GetHeight()
    );
    ImGui::Text(
        "Static tree: %zu / h%d",
        staticTree.GetProxyCount(),
        staticTree.GetHeight()
    );

    ImGui::Spacing();

    ImGui::TextWrapped( "Contact labels: s = separation, Jn = normal impulse, Jt = friction impulse. Cyan arrows show velocity; dim bodies are sleeping." );
}
#pragma endregion

std::unique_ptr<demo> createDemoView( demoKind kind ) { return std::make_unique<rigidBodyDemoUi>( kind ); }
} // namespace zonai::sandbox
