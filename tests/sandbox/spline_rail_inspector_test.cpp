#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <limits>
#include <imgui.h>
#include <imgui_internal.h>
#include "rigidBodyDemoUi.h"

using namespace zonai;
using namespace zonai::sandbox;

namespace
{
void check( bool ok, const char* message )
{
    if( ok ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

std::string frame( demoSession& session, const char* control = nullptr, bool input = false, bool popup = false )
{
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0, 0 } );
    ImGui::SetNextWindowSize( { 340, 4000 } );
    ImGui::Begin( "Spline rail rotation inspector" );
    if( control )
    {
        if( popup ) check( !GImGui->OpenPopupStack.empty() && GImGui->OpenPopupStack.back().Window, "selection popup is open" );
        GImGui->NavActivateId = popup ? GImGui->OpenPopupStack.back().Window->GetID( control ) : ImGui::GetID( control );
        if( input ) GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        else GImGui->NavActivateDownId = GImGui->NavActivateId;
    }
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "###Collision material" ), 1 );
    ImGui::LogToBuffer();
    session.getDemo().drawControls();
    const std::string text = GImGui->LogBuffer.c_str();
    ImGui::LogFinish();
    ImGui::End();
    ImGui::Render();
    return text;
}

void edit( demoSession& session, const char* control, const char* value )
{
    frame( session, control, true );
    ImGui::GetIO().AddInputCharactersUTF8( value );
    frame( session );
    ImGui::GetIO().AddKeyEvent( ImGuiKey_Enter, true );
    frame( session );
    ImGui::GetIO().AddKeyEvent( ImGuiKey_Enter, false );
    frame( session );
}

void checkDisplay( const std::string& text, const splineRailJointData& data )
{
    char expected[128];
    std::snprintf( expected, sizeof( expected ), "회전 오차: %.3f rad", data.currentAngle );
    check( text.find( expected ) != std::string::npos, "displayed angular error is refreshed from the current rail state" );
    std::snprintf( expected, sizeof( expected ), "오차 변화 속도: %.3f rad/s", data.currentAngularSpeed );
    check( text.find( expected ) != std::string::npos, "displayed angular error rate is refreshed with rad/s units" );
}

void checkModeTransition()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto body = scene.getImpulseBody();
    auto pose = physics.GetBodyTransform( body );
    pose.rotation = rot2::FromRadians( 0.43f );
    physics.SetBodyTransform( body, pose );
    frame( session );
    edit( session, "###SplineRailRotationReference", "0.75" );
    const auto before = physics.getSplineRailJointData( id );
    const auto text = frame( session, "###SplineRailFollow" );
    const auto follow = physics.getSplineRailJointData( id );
    check( follow.rotationMode == splineRailRotationMode::followTangent && follow.referenceAngle == 0.75f && follow.tangentOffset != before.tangentOffset && std::abs( follow.currentAngle ) < 0.000001f, "mode checkbox captures the current tangent reference without overwriting the fixed reference" );
    char expected[64];
    std::snprintf( expected, sizeof( expected ), "%.3f rad", follow.tangentOffset );
    check( text.find( "접선 기준각도" ) != std::string::npos && text.find( expected ) != std::string::npos, "same-frame mode change displays the newly captured tangent offset" );
    checkDisplay( text, follow );
    edit( session, "###SplineRailRotationReference", "-0.35" );
    check( physics.getSplineRailJointData( id ).tangentOffset == -0.35f && physics.getSplineRailJointData( id ).referenceAngle == 0.75f, "tangent input edits only the selected mode reference" );
    frame( session, "###SplineRailFollow" );
    const auto fixed = physics.getSplineRailJointData( id );
    check( fixed.rotationMode == splineRailRotationMode::fixed && fixed.referenceAngle == 0.75f && fixed.tangentOffset == -0.35f, "returning to fixed mode retains both stored references" );
}

void checkSessionState()
{
    for( bool playing : { false, true } )
        for( int mode : { 0, 1, 2 } )
        {
            demoSession session{ createDemoView };
            session.selectDemo( demoKind::splineRail );
            session.setPlaying( playing );
            auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
            auto& physics = scene.getWorld();
            physics.SetGravity( {} );
            const auto id = scene.getSplineRailJoint();
            const auto body = scene.getImpulseBody();
            const auto visual = scene.getShapes()[0];
            const auto count = scene.getSplineControlPoints().size();
            check( scene.insertSplineControlPoint( 1 ), "state fixture has a real geometry edit" );
            if( playing ) check( scene.undoSplineEdit(), "playing fixture retains a pending redo" );
            frame( session );
            frame( session, "###Object" );
            frame( session, visual.label, false, true );
            frame( session, "###SplineControl" );
            frame( session, "제어점 3", false, true );
            if( mode != 0 ) frame( session, "###SplineRailFollow" );
            if( mode == 1 ) frame( session, "###SplineRailAngularHard" );
            if( mode == 2 )
            {
                frame( session, "###SplineRailAngularSpring" );
                edit( session, "###SplineRailAngularHertz", "4.5" );
                edit( session, "###SplineRailAngularDamping", "0.8" );
            }
            edit( session, "###SplineRailRotationReference", mode == 0 ? "0.65" : "-0.35" );
            physics.SetBodyLinearVelocity( body, { 1.2f, -0.3f } );
            physics.SetBodyAngularVelocity( body, 0.8f );
            const auto pose = physics.GetBodyTransform( body );
            const auto velocity = physics.GetBodyLinearVelocity( body );
            const auto before = physics.getSplineRailJointData( id );
            const auto stepCount = session.getStepCount();
            const std::vector<vec2> points( scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() );
            const bool undo = scene.canUndoSplineEdit(), redo = scene.canRedoSplineEdit();
            check( before.rotationMode == ( mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent ) && before.enableAngularSpring == ( mode != 1 ), "mode and hard/spring widgets apply the requested configuration" );
            if( mode == 2 ) check( before.angularHertz == 4.5f && before.angularDampingRatio == 0.8f, "actual angular tuning inputs apply their settings" );
            auto text = frame( session, "###SplineRailRotationEnabled" );
            const auto free = physics.getSplineRailJointData( id );
            check( !free.enableRotation && free.rotationMode == before.rotationMode && free.referenceAngle == before.referenceAngle && free.tangentOffset == before.tangentOffset && free.enableAngularSpring == before.enableAngularSpring && free.angularHertz == before.angularHertz && free.angularDampingRatio == before.angularDampingRatio, "rotation checkbox preserves mode, both references and angular tuning" );
            check( text.find( "제어점 3" ) != std::string::npos && text.find( "회전은 자유롭습니다." ) != std::string::npos, "control-point selection survives rotation changes and free mode is explained" );
            edit( session, "###SplineRailRotationReference", mode == 0 ? "0.9" : "-0.6" );
            auto edited = physics.getSplineRailJointData( id );
            check( !edited.enableRotation && ( mode == 0 ? edited.referenceAngle == 0.9f : edited.tangentOffset == -0.6f ), "references remain editable while rotation is free without re-enabling it" );
            text = frame( session, "###SplineRailRotationRebase" );
            const auto rebased = physics.getSplineRailJointData( id );
            check( !rebased.enableRotation && std::abs( rebased.currentAngle ) < 0.000001f && ( mode == 0 ? rebased.tangentOffset == edited.tangentOffset : rebased.referenceAngle == edited.referenceAngle ), "current-pose button rebases only the selected mode while retaining free rotation" );
            checkDisplay( text, rebased );
            text = frame( session, "###SplineRailRotationEnabled" );
            const auto resumed = physics.getSplineRailJointData( id );
            check( resumed.enableRotation && resumed.referenceAngle == rebased.referenceAngle && resumed.tangentOffset == rebased.tangentOffset && resumed.enableAngularSpring == before.enableAngularSpring && resumed.angularHertz == before.angularHertz && resumed.angularDampingRatio == before.angularDampingRatio, "reactivation uses the saved reference and tuning" );
            checkDisplay( text, resumed );
            edit( session, "###Friction", "0.42" );
            check( physics.GetShapeFriction( visual.shapeHandle ) == 0.42f, "object inspector selection still targets the original shape after rail controls" );
            check( session.isPlaying() == playing && session.getStepCount() == stepCount && scene.canUndoSplineEdit() == undo && scene.canRedoSplineEdit() == redo && scene.getSplineControlPoints().size() == points.size(), "UI operations preserve playback and pending spline undo/redo records" );
            for( std::size_t i = 0; i < points.size(); ++i ) check( LengthSquared( points[i] - scene.getSplineControlPoints()[i] ) == 0.0f, "rotation controls do not alter spline control points" );
            const auto after = physics.getSplineRailJointData( id );
            check( physics.IsValid( body ) && physics.IsValid( id ) && physics.IsValid( visual.shapeHandle ) && LengthSquared( physics.GetBodyTransform( body ).position - pose.position ) == 0.0f && physics.GetBodyTransform( body ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( body ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( body ) == 0.8f && after.currentTranslation == before.currentTranslation && after.referenceDistance == before.referenceDistance && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.enableSpring == before.enableSpring && after.targetTranslation == before.targetTranslation && after.enableLimit == before.enableLimit && after.enableMotor == before.enableMotor, "rotation controls preserve body state, handles, travel, branch and axial control settings" );
            frame( session, playing ? "###SplineRedo" : "###SplineUndo" );
            const auto restored = physics.getSplineRailJointData( id );
            check( scene.getSplineControlPoints().size() == count + ( playing ? 1 : 0 ) && restored.enableRotation && restored.referenceAngle == after.referenceAngle && restored.tangentOffset == after.tangentOffset, "actual geometry undo/redo buttons restore their original edit and retain later rotation settings" );
            check( frame( session ).find( "제어점 3" ) != std::string::npos, "control-point selection is retained after inspector edits and geometry undo/redo" );
        }
}

void checkNoOpAndFailure()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto body = scene.getImpulseBody();
    physics.SetGravity( {} );
    physics.setSplineRailJointRotationReference( id, 7.123456f );
    physics.SetBodyAwake( body, false );
    for( int i = 0; i < 3; ++i ) frame( session );
    check( !physics.IsBodyAwake( body ) && physics.getSplineRailJointData( id ).referenceAngle == 7.123456f, "idle drawing does not wake or truncate or clamp an existing reference" );
    edit( session, "###SplineRailRotationReference", "nan" );
    check( !physics.IsBodyAwake( body ) && physics.getSplineRailJointData( id ).referenceAngle == 7.123456f, "nonfinite reference text is rejected without changing the saved value or sleep" );
    physics.setSplineRailJointSpring( id, true, std::numeric_limits<float>::max(), 0.7f, 1.0f );
    session.stepOnce( 4 );
    auto pose = physics.GetBodyTransform( body );
    pose.rotation = rot2::FromRadians( 0.4f );
    physics.SetBodyTransform( body, pose );
    physics.SetBodyAwake( body, false );
    const auto before = physics.getSplineRailJointData( id );
    check( !before.trackingValid, "failed tracking fixture disables pose rebasing" );
    frame( session, "###SplineRailRotationRebase" );
    const auto after = physics.getSplineRailJointData( id );
    check( !after.trackingValid && after.referenceAngle == before.referenceAngle && after.tangentOffset == before.tangentOffset && after.currentAngle == before.currentAngle && after.currentTranslation == before.currentTranslation && after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && !physics.IsBodyAwake( body ), "unavailable current-pose action preserves failed tracking state, references, cache and sleep" );
}

}

void checkSplineRailInspector()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    frame( session );
    check( physics.getSplineRailJointData( id ).enableRotation, "inspector starts with existing angular constraint enabled" );
    frame( session, "###SplineRailRotationEnabled" );
    check( !physics.getSplineRailJointData( id ).enableRotation, "actual rotation checkbox disables the angular constraint" );
    edit( session, "###SplineRailRotationReference", "0.75" );
    check( physics.getSplineRailJointData( id ).referenceAngle == 0.75f, "actual reference input edits the fixed relative angle" );
    frame( session, "###SplineRailRotationRebase" );
    check( std::abs( physics.getSplineRailJointData( id ).currentAngle ) < 0.000001f, "actual current-pose button rebases the rotation reference" );
    physics.SetBodyAngularVelocity( scene.getImpulseBody(), 1.25f );
    const auto text = frame( session );
    check( text.find( "회전 오차: 0.000 rad" ) != std::string::npos && text.find( "오차 변화 속도: 1.250 rad/s" ) != std::string::npos, "inspector displays current error and its actual rate with their units" );
    checkModeTransition();
    checkSessionState();
    checkNoOpAndFailure();
}
