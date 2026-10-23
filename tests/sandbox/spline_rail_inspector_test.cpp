#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <limits>
#include <utility>
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

void checkTranslationControls()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto points = scene.getSplineControlPoints();
    physics.SetBodyTransform( scene.getImpulseBody(), { Evaluate( { points }, 1, 0.4f ), {} } );
    frame( session );
    const auto before = physics.getSplineRailJointData( id );
    check( before.trackingValid && before.currentTranslation > 1.0, "translation button fixture starts away from the existing origin" );
    frame( session, "###SplineRailRebase" );
    const auto after = physics.getSplineRailJointData( id );
    check( after.currentTranslation == 0.0 && after.referenceDistance == before.referenceDistance + before.currentTranslation, "actual translation button rebases the current position without moving it" );
    physics.setSplineRailJointMotor( id, true, 9.0f, 0.0f );
    physics.SetBodyLinearVelocity( scene.getImpulseBody(), -2.0f * after.tangent );
    const auto text = frame( session );
    check( text.find( "실제 이동 속도: -2.000 m/s" ) != std::string::npos, "inspector displays the signed actual speed independently of motor target" );
}

void checkTranslationDisplay( const std::string& text, const splineRailJointData& data )
{
    char expected[128];
    std::snprintf( expected, sizeof( expected ), "이동 거리: %.2f / %.2f m", data.currentTranslation, data.length );
    check( text.find( expected ) != std::string::npos, "distance display uses the post-button coordinates in the same frame" );
    std::snprintf( expected, sizeof( expected ), "실제 이동 속도: %.3f m/s", data.currentSpeed );
    check( text.find( expected ) != std::string::npos, "speed display uses the actual signed speed with m/s units" );
}

void checkTranslationSnapshot( const splineRailJointData& before, const splineRailJointData& after )
{
    check( after.currentTranslation == 0.0 && after.referenceDistance == before.referenceDistance + before.currentTranslation && after.currentSpeed == before.currentSpeed && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t, "button changes only the linear origin and preserves speed and tracked branch" );
    check( std::abs( after.referenceDistance + after.targetTranslation - before.referenceDistance - before.targetTranslation ) < 0.00001 && std::abs( after.referenceDistance + after.lowerTranslation - before.referenceDistance - before.lowerTranslation ) < 0.00001 && std::abs( after.referenceDistance + after.upperTranslation - before.referenceDistance - before.upperTranslation ) < 0.00001, "target and limit physical coordinates are invariant within float rounding" );
    check( after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && after.springForce == before.springForce && after.motorForce == before.motorForce, "translation button preserves cached constraint reactions" );
    check( after.enableRotation == before.enableRotation && after.rotationMode == before.rotationMode && after.referenceAngle == before.referenceAngle && after.tangentOffset == before.tangentOffset && after.currentAngle == before.currentAngle && after.currentAngularSpeed == before.currentAngularSpeed && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio && after.enableSpring == before.enableSpring && after.hertz == before.hertz && after.dampingRatio == before.dampingRatio && after.enableLimit == before.enableLimit && after.enableMotor == before.enableMotor && after.motorSpeed == before.motorSpeed && after.maxMotorForce == before.maxMotorForce, "translation button preserves all rotation and axial tuning" );
}

void checkTranslationSessionState()
{
    for( bool playing : { false, true } )
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
        check( scene.insertSplineControlPoint( 1 ), "translation fixture starts with pending geometry history" );
        if( playing ) check( scene.undoSplineEdit(), "translation fixture can preserve a pending redo" );
        physics.SetBodyTransform( body, { Evaluate( { scene.getSplineControlPoints() }, 1, 0.4f ), rot2::FromRadians( 0.4f ) } );
        const auto start = physics.getSplineRailJointData( id );
        physics.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.5f, 0.8f, false );
        physics.setSplineRailJointRotationEnabled( id, !playing );
        physics.setSplineRailJointSpring( id, true, 0.5f, 0.7f, static_cast<float>( start.currentTranslation + 0.8 ) );
        physics.setSplineRailJointLimit( id, true, static_cast<float>( start.currentTranslation - 0.6 ), static_cast<float>( start.currentTranslation + 0.9 ) );
        physics.setSplineRailJointMotor( id, true, 9.0f, 0.2f );
        physics.SetBodyLinearVelocity( body, 1.2f * start.tangent );
        physics.SetBodyAngularVelocity( body, 0.8f );
        for( int step = 0; step < 5; ++step ) session.stepOnce( 4 );
        frame( session );
        frame( session, "###Object" );
        frame( session, visual.label, false, true );
        frame( session, "###SplineControl" );
        frame( session, "제어점 3", false, true );
        if( playing ) physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        const auto pose = physics.GetBodyTransform( body );
        const auto velocity = physics.GetBodyLinearVelocity( body );
        const float spin = physics.GetBodyAngularVelocity( body );
        const bool awake = physics.IsBodyAwake( body ), undo = scene.canUndoSplineEdit(), redo = scene.canRedoSplineEdit();
        const auto steps = session.getStepCount();
        const std::vector<vec2> points( scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() );
        check( LengthSquared( before.force ) > 0.0f, "translation fixture has a live cached reaction to preserve" );
        auto text = frame( session, "###SplineRailRebase" );
        const auto after = physics.getSplineRailJointData( id );
        checkTranslationSnapshot( before, after );
        checkTranslationDisplay( text, after );
        check( text.find( "제어점 3" ) != std::string::npos && session.isPlaying() == playing && session.getStepCount() == steps && scene.canUndoSplineEdit() == undo && scene.canRedoSplineEdit() == redo && physics.IsBodyAwake( body ) == awake, "translation button preserves selection, playback, geometry history and sleep" );
        frame( session, "###SplineRailRebase" );
        const auto repeated = physics.getSplineRailJointData( id );
        check( repeated.referenceDistance == after.referenceDistance && repeated.force.x == after.force.x && repeated.force.y == after.force.y && repeated.torque == after.torque && physics.IsBodyAwake( body ) == awake, "repeated zero button does not clear cache or wake the body" );
        check( LengthSquared( physics.GetBodyTransform( body ).position - pose.position ) == 0.0f && physics.GetBodyTransform( body ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( body ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( body ) == spin && physics.IsValid( visual.shapeHandle ) && physics.IsValid( id ) && scene.getSplineControlPoints().size() == points.size(), "translation button leaves physical state and handles intact" );
        for( std::size_t point = 0; point < points.size(); ++point ) check( LengthSquared( points[point] - scene.getSplineControlPoints()[point] ) == 0.0f, "translation button never edits control points" );
        edit( session, "###SplineRailTarget", "-0.25" );
        edit( session, "###SplineRailLower", "-0.5" );
        edit( session, "###SplineRailUpper", "1000" );
        const auto edited = physics.getSplineRailJointData( id );
        check( edited.targetTranslation == -0.25f && edited.lowerTranslation == -0.5f && edited.upperTranslation == static_cast<float>( edited.length - edited.referenceDistance ), "target and limit inputs use the translated slider ranges rather than the old zero-based bounds" );
        edit( session, "###Friction", "0.42" );
        check( physics.GetShapeFriction( visual.shapeHandle ) == 0.42f, "object selection still targets the original shape after zero reset" );
        frame( session, playing ? "###SplineRedo" : "###SplineUndo" );
        check( scene.getSplineControlPoints().size() == count + ( playing ? 1 : 0 ) && physics.getSplineRailJointData( id ).referenceDistance == after.referenceDistance && frame( session ).find( "제어점 3" ) != std::string::npos, "the original geometry history and control-point selection survive zero reset and later setting edits" );
    }
}

void configureTranslationMotion( demoSession& session, int mode, int control, int substeps )
{
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    physics.SetGravity( {} );
    const auto body = scene.getImpulseBody();
    const auto id = scene.getSplineRailJoint();
    physics.SetBodyTransform( body, { Evaluate( { scene.getSplineControlPoints() }, 1, 0.3f ), rot2::FromRadians( 0.3f ) } );
    physics.SetBodySleepEnabled( body, false );
    physics.setSplineRailJointRotation( id, mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent, 3.0f, 0.7f, mode == 2 );
    physics.setSplineRailJointRotationEnabled( id, mode != 3 );
    const auto data = physics.getSplineRailJointData( id );
    physics.setSplineRailJointSpring( id, control == 1, 1.0f, 0.7f, static_cast<float>( data.currentTranslation + 0.8 ) );
    physics.setSplineRailJointLimit( id, control == 2, static_cast<float>( data.currentTranslation - 0.7 ), static_cast<float>( data.currentTranslation + 0.7 ) );
    physics.setSplineRailJointMotor( id, control != 1, 1.2f, 1.0f );
    physics.SetBodyLinearVelocity( body, data.tangent );
    physics.SetBodyAngularVelocity( body, 0.4f );
    for( int step = 0; step < 10; ++step ) session.stepOnce( substeps );
}

void checkTranslationMotion()
{
    for( int mode : { 0, 1, 2, 3 } )
        for( int control : { 0, 1, 2 } )
            for( int substeps : { 1, 4, 8 } )
            {
                demoSession actual{ createDemoView }, reference{ createDemoView };
                configureTranslationMotion( actual, mode, control, substeps );
                configureTranslationMotion( reference, mode, control, substeps );
                auto& scene = static_cast<rigidBodyDemo&>( actual.getDemo() );
                auto& other = static_cast<rigidBodyDemo&>( reference.getDemo() );
                auto& physics = scene.getWorld();
                auto& expected = other.getWorld();
                const auto id = scene.getSplineRailJoint(), otherId = other.getSplineRailJoint();
                const auto before = physics.getSplineRailJointData( id );
                frame( actual );
                frame( actual, "###SplineRailRebase" );
                checkTranslationSnapshot( before, physics.getSplineRailJointData( id ) );
                for( int step = 0; step < 240; ++step )
                {
                    if( step == 120 && control != 1 )
                    {
                        physics.setSplineRailJointMotor( id, true, -1.2f, 1.0f );
                        expected.setSplineRailJointMotor( otherId, true, -1.2f, 1.0f );
                    }
                    actual.stepOnce( substeps );
                    reference.stepOnce( substeps );
                    const auto data = physics.getSplineRailJointData( id ), baseline = expected.getSplineRailJointData( otherId );
                    check( data.trackingValid && baseline.trackingValid && std::abs( data.currentTranslation + before.currentTranslation - baseline.currentTranslation ) < 0.002 && std::abs( data.currentSpeed - baseline.currentSpeed ) < 0.002f, "button-rebased motor, spring or limits retain travel and speed of an independent unre-based scene" );
                    const auto pose = physics.GetBodyTransform( data.bodyB ), otherPose = expected.GetBodyTransform( baseline.bodyB );
                    check( Length( pose.position - otherPose.position ) < 0.002f && std::abs( pose.rotation.s - otherPose.rotation.s ) < 0.002f && std::abs( pose.rotation.c - otherPose.rotation.c ) < 0.002f && Length( physics.GetBodyLinearVelocity( data.bodyB ) - expected.GetBodyLinearVelocity( baseline.bodyB ) ) < 0.002f && std::abs( physics.GetBodyAngularVelocity( data.bodyB ) - expected.GetBodyAngularVelocity( baseline.bodyB ) ) < 0.002f, "actual UI zero reset preserves subsequent physical motion across rotation modes and substeps" );
                }
            }
}

void checkTranslationFailures()
{
    for( bool numeric : { false, true } )
    {
        demoSession session{ createDemoView };
        session.selectDemo( demoKind::splineRail );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        const auto body = scene.getImpulseBody();
        physics.SetGravity( {} );
        if( numeric )
        {
            while( scene.getSplineControlPoints().size() > 2 ) check( scene.removeSplineControlPoint( 1 ), "numeric failure fixture reduces its editable spline to a line" );
            const auto points = scene.getSplineControlPoints();
            const auto tangent = Normalize( points[1] - points[0] );
            physics.SetBodyTransform( body, { points[1] + 1.0e10f * tangent, {} } );
        }
        else
        {
            physics.setSplineRailJointSpring( id, true, std::numeric_limits<float>::max(), 0.7f, 1.0f );
            session.stepOnce( 4 );
        }
        physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        check( before.trackingValid == numeric && !physics.rebaseSplineRailJoint( id ), "failure fixture rejects an invalid track or numerically collapsed endpoint range" );
        frame( session );
        const auto text = frame( session, "###SplineRailRebase" );
        const auto after = physics.getSplineRailJointData( id );
        check( after.trackingValid == before.trackingValid && after.referenceDistance == before.referenceDistance && after.currentTranslation == before.currentTranslation && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && !physics.IsBodyAwake( body ), "unavailable or rejected UI zero reset preserves all coordinates, branch, cache and sleep" );
        checkTranslationDisplay( text, after );
        if( !numeric ) check( after.currentSpeed == 0.0f, "invalid tracking speed display reports zero" );
    }
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
    checkTranslationControls();
    checkTranslationSessionState();
    checkTranslationMotion();
    checkTranslationFailures();
}
