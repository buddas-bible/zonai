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
#include "debug/debugDraw.h"

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


void selectReprojectionTarget( demoSession& session, const char* target, const char* segment = nullptr )
{
    frame( session, "###SplineRailReprojectTarget" );
    check( !GImGui->OpenPopupStack.empty(), "actual reprojection target list opens for user selection" );
    frame( session, target, false, true );
    if( segment )
    {
        frame( session, "###SplineRailReprojectSegment" );
        frame( session, segment, false, true );
    }
}

void checkReprojectionButton()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto body = scene.getImpulseBody();
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    check( physics.setSplineRailJointPoints( id, points ), "button fixture accepts a straight rail" );
    physics.SetBodyTransform( body, { { 1, 0.25f }, {} } );
    physics.SetBodyAwake( body, false );
    frame( session );
    const auto text = frame( session, "###SplineRailReproject" );
    check( physics.IsBodyAwake( body ), "actual reprojection button commits a changed projection within the default 0.5m limit" );
    check( text.find( "최근 재투영: 성공" ) != std::string::npos, "accepted reprojection is reported in the same frame" );
    check( physics.GetBodyTransform( body ).position.x == 1.0f && physics.GetBodyTransform( body ).position.y == 0.25f, "reprojection button does not teleport the body to the rail" );
}


void checkReprojectionSettings( const splineRailJointData& before, const splineRailJointData& after )
{
    check( after.referenceDistance == before.referenceDistance && after.length == before.length && after.isLoop == before.isLoop && after.collideConnected == before.collideConnected && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation, "UI reprojection preserves geometry, origin and axial target/limits" );
    check( after.enableRotation == before.enableRotation && after.rotationMode == before.rotationMode && after.referenceAngle == before.referenceAngle && after.tangentOffset == before.tangentOffset && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio && after.enableSpring == before.enableSpring && after.hertz == before.hertz && after.dampingRatio == before.dampingRatio && after.enableLimit == before.enableLimit && after.enableMotor == before.enableMotor && after.motorSpeed == before.motorSpeed && after.maxMotorForce == before.maxMotorForce, "UI reprojection preserves rotation and movement control settings" );
}

void checkReprojectionUnchanged( const splineRailJointData& before, const splineRailJointData& after )
{
    checkReprojectionSettings( before, after );
    check( after.trackingValid == before.trackingValid && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.currentTranslation == before.currentTranslation && after.currentSpeed == before.currentSpeed && after.currentAngle == before.currentAngle && after.currentAngularSpeed == before.currentAngularSpeed && after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && after.springForce == before.springForce && after.motorForce == before.motorForce, "rejected, unchanged or idle UI reprojection preserves tracking, coordinates, speed and cached reactions" );
}

void prepareStraightReprojection( demoSession& session )
{
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    physics.SetBodyTransform( scene.getImpulseBody(), {} );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    check( physics.setSplineRailJointPoints( scene.getSplineRailJoint(), points ), "bounded UI fixture accepts a straight rail" );
    for( int step = 0; step < 10; ++step ) session.stepOnce( 4 );
    check( LengthSquared( physics.getSplineRailJointData( scene.getSplineRailJoint() ).force ) > 0.0f, "straight fixture has a load reaction before teleport clears its cache" );
}

void checkReprojectionBoundaries()
{
    for( float distance : { std::nextafter( 0.5f, 0.0f ), 0.5f, std::nextafter( 0.5f, 1.0f ) } )
    {
        demoSession session{ createDemoView };
        prepareStraightReprojection( session );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        const auto body = scene.getImpulseBody();
        physics.SetBodyTransform( body, { { 1, distance }, {} } );
        physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        frame( session );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        check( !physics.IsBodyAwake( body ), "drawing controls alone does not reproject or wake a displaced body" );
        const auto text = frame( session, "###SplineRailReproject" );
        const auto after = physics.getSplineRailJointData( id );
        const bool accepted = distance <= 0.5f;
        check( text.find( accepted ? "최근 재투영: 성공" : "최근 재투영: 거절" ) != std::string::npos && physics.IsBodyAwake( body ) == accepted, "default 0.5m button accepts below/equal and rejects the next representable distance above the boundary" );
        checkReprojectionSettings( before, after );
        if( accepted ) check( LengthSquared( after.force ) == 0.0f && after.torque == 0.0f, "changed successful projection clears old reactions through the existing API" );
        else checkReprojectionUnchanged( before, after );
        check( physics.GetBodyTransform( body ).position.x == 1.0f && physics.GetBodyTransform( body ).position.y == distance, "distance acceptance and rejection preserve body position" );
        checkTranslationDisplay( text, after );
        checkDisplay( text, after );
    }
}

void checkReprojectionInput()
{
    for( bool selected : { false, true } )
    for( const char* value : { "0", "0.25", "0.75", "-1", "nan", "inf" } )
    {
        demoSession session{ createDemoView };
        prepareStraightReprojection( session );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        const auto body = scene.getImpulseBody();
        physics.SetBodyTransform( body, { { 1, 0.25f }, {} } );
        physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        frame( session );
        if( selected ) selectReprojectionTarget( session, "지정 구간", "구간 1" );
        edit( session, "###SplineRailReprojectDistance", value );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        check( !physics.IsBodyAwake( body ), "editing the allowance never performs reprojection" );
        const bool accepted = std::string( value ) == "0.25" || std::string( value ) == "0.75";
        const auto text = frame( session, "###SplineRailReproject" );
        const auto after = physics.getSplineRailJointData( id );
        check( text.find( accepted ? "최근 재투영: 성공" : "최근 재투영: 거절" ) != std::string::npos && physics.IsBodyAwake( body ) == accepted, "actual distance input reaches the bounded API, including zero, negative and nonfinite rejection" );
        if( !accepted ) checkReprojectionUnchanged( before, after );
    }
    demoSession session{ createDemoView };
    prepareStraightReprojection( session );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto body = scene.getImpulseBody();
    physics.SetBodyTransform( body, { { 1, 0 }, {} } );
    physics.SetBodyAwake( body, false );
    frame( session );
    edit( session, "###SplineRailReprojectDistance", "0" );
    check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 성공" ) != std::string::npos && physics.IsBodyAwake( body ), "zero allowance accepts an exact point on the rail" );
    for( int step = 0; step < 10; ++step ) session.stepOnce( 4 );
    physics.SetBodyAwake( body, false );
    const auto before = physics.getSplineRailJointData( id );
    check( LengthSquared( before.force ) > 0.0f, "repeated projection fixture has a loaded reaction cache" );
    frame( session, "###SplineRailReproject" );
    checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
    check( !physics.IsBodyAwake( body ), "identical successful button activation preserves loaded cache and sleep" );
    edit( session, "###SplineRailReprojectDistance", "1" );
    physics.SetBodyTransform( body, { { 2, 0.75f }, {} } );
    check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 성공" ) != std::string::npos, "allowance edit permits a farther point" );
    session.selectDemo( demoKind::playground );
    prepareStraightReprojection( session );
    auto& fresh = static_cast<rigidBodyDemo&>( session.getDemo() );
    fresh.getWorld().SetBodyTransform( fresh.getImpulseBody(), { { 1, 0.75f }, {} } );
    check( frame( session ).find( "최근 재투영:" ) == std::string::npos, "a new view starts without a previous request result" );
    check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 거절" ) != std::string::npos, "a new view restores the default 0.5m allowance" );
}

void checkReprojectionRecovery()
{
    for( bool selected : { false, true } )
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession session{ createDemoView };
        prepareStraightReprojection( session );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        const auto body = scene.getImpulseBody();
        physics.setSplineRailJointSpring( id, true, std::numeric_limits<float>::max(), 0.7f, 1.0f );
        session.stepOnce( substeps );
        physics.setSplineRailJointSpring( id, false, 2.0f, 0.7f, 1.0f );
        physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        check( !before.trackingValid, "recovery fixture has a stored tracking failure" );
        frame( session );
        if( selected ) selectReprojectionTarget( session, "지정 구간", "구간 1" );
        edit( session, "###SplineRailReprojectDistance", "-1" );
        auto text = frame( session, "###SplineRailReproject" );
        check( text.find( "최근 재투영: 거절" ) != std::string::npos && !physics.IsBodyAwake( body ), "invalid allowance reports rejection even when tracking is unavailable" );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        edit( session, "###SplineRailReprojectDistance", "0.5" );
        text = frame( session );
        check( text.find( "최근 재투영: 거절" ) != std::string::npos && !physics.getSplineRailJointData( id ).trackingValid && !physics.IsBodyAwake( body ), "last result persists and editing a valid allowance does not automatically retry recovery" );
        physics.SetBodyLinearVelocity( body, { -0.75f, 0.0f } );
        physics.SetBodyAngularVelocity( body, 0.6f );
        check( physics.getSplineRailJointData( id ).currentSpeed == 0.0f, "failed tracking hides speed before recovery" );
        text = frame( session, "###SplineRailReproject" );
        const auto after = physics.getSplineRailJointData( id );
        check( after.trackingValid && physics.IsBodyAwake( body ) && text.find( "최근 재투영: 성공" ) != std::string::npos, "explicit UI recovery remains available while tracking is invalid" );
        check( after.currentSpeed == -0.75f && after.currentAngularSpeed == 0.6f, "recovery immediately exposes preserved physical velocities in the same frame" );
        checkReprojectionSettings( before, after );
        checkTranslationDisplay( text, after );
        physics.SetGravity( {} );
        physics.setSplineRailJointMotor( id, true, 1.0f, 100.0f );
        for( int step = 0; step < 30; ++step ) session.stepOnce( substeps );
        const auto resumed = physics.getSplineRailJointData( id );
        check( resumed.trackingValid && resumed.currentTranslation > after.currentTranslation + 0.3 && std::abs( resumed.currentSpeed - 1.0f ) < 0.001f, "normal motor motion resumes after inspector recovery" );
    }
}

void checkReprojectionCrossing()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto body = scene.getImpulseBody();
    const vec2 points[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    physics.SetBodyTransform( body, { Evaluate( { points }, 3, 0.9f ), {} } );
    check( physics.setSplineRailJointPoints( id, points ), "UI crossing fixture accepts an intersecting curve" );
    physics.SetBodyTransform( body, { { 0, 0 }, rot2::FromRadians( 0.3f ) } );
    physics.SetBodyLinearVelocity( body, { 1, -2 } );
    physics.SetBodyAngularVelocity( body, 0.7f );
    const auto before = physics.getSplineRailJointData( id );
    check( before.trackingValid && before.parameter.segmentIndex >= 3, "local tracking starts on the later crossing branch" );
    for( int draw = 0; draw < 3; ++draw ) frame( session );
    checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
    const auto text = frame( session, "###SplineRailReproject" );
    const auto after = physics.getSplineRailJointData( id );
    check( after.trackingValid && after.parameter.segmentIndex < 2 && after.currentTranslation < before.currentTranslation - 1.0 && text.find( "최근 재투영: 성공" ) != std::string::npos, "explicit button selects the earlier global nearest crossing branch" );
    checkReprojectionSettings( before, after );
    check( physics.GetBodyTransform( body ).position.x == 0.0f && physics.GetBodyTransform( body ).position.y == 0.0f && physics.GetBodyTransform( body ).rotation.s == rot2::FromRadians( 0.3f ).s && physics.GetBodyLinearVelocity( body ).x == 1.0f && physics.GetBodyLinearVelocity( body ).y == -2.0f && physics.GetBodyAngularVelocity( body ) == 0.7f, "crossing branch changes never teleport or change body velocity" );
    checkTranslationDisplay( text, after );
    checkDisplay( text, after );
}


void checkReprojectionSessionState()
{
    for( bool selected : { false, true } )
    for( bool playing : { false, true } )
        for( bool accepted : { false, true } )
        {
            demoSession session{ createDemoView };
            session.selectDemo( demoKind::splineRail );
            session.setPlaying( playing );
            auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
            auto& physics = scene.getWorld();
            const auto id = scene.getSplineRailJoint();
            const auto body = scene.getImpulseBody();
            const auto visual = scene.getShapes()[0];
            const auto count = scene.getSplineControlPoints().size();
            check( scene.insertSplineControlPoint( 1 ), "reprojection preservation fixture has a pending geometry edit" );
            if( playing ) check( scene.undoSplineEdit(), "playing reprojection fixture has a pending redo" );
            physics.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.5f, 0.8f, playing );
            physics.setSplineRailJointRotationEnabled( id, !playing );
            physics.setSplineRailJointRotationReference( id, -0.35f );
            physics.setSplineRailJointSpring( id, true, 0.5f, 0.7f, 0.8f );
            physics.setSplineRailJointLimit( id, true, 0.0f, 5.0f );
            physics.setSplineRailJointMotor( id, true, 1.2f, 0.2f );
            for( int step = 0; step < 10; ++step ) session.stepOnce( 4 );
            check( physics.rebaseSplineRailJoint( id ), "reprojection fixture starts with a rebased translation origin" );
            frame( session );
            frame( session, "###Object" );
            frame( session, visual.label, false, true );
            frame( session, "###SplineControl" );
            frame( session, "제어점 3", false, true );
            if( accepted ) physics.SetBodyTransform( body, { Evaluate( { scene.getSplineControlPoints() }, 2, 0.6f ), rot2::FromRadians( 0.3f ) } );
            else edit( session, "###SplineRailReprojectDistance", "-1" );
            if( selected ) selectReprojectionTarget( session, "지정 구간", accepted ? "구간 3" : "구간 1" );
            physics.SetBodyAwake( body, false );
            const auto before = physics.getSplineRailJointData( id );
            const auto pose = physics.GetBodyTransform( body );
            const auto velocity = physics.GetBodyLinearVelocity( body );
            const float spin = physics.GetBodyAngularVelocity( body ), mass = physics.GetBodyMass( body );
            const bool undo = scene.canUndoSplineEdit(), redo = scene.canRedoSplineEdit();
            const auto steps = session.getStepCount();
            const std::vector<vec2> points( scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() );
            if( !accepted ) check( LengthSquared( before.force ) > 0.0f, "rejected UI request has a nonzero reaction cache to preserve" );
            const auto text = frame( session, "###SplineRailReproject" );
            const auto after = physics.getSplineRailJointData( id );
            checkReprojectionSettings( before, after );
            if( !accepted ) checkReprojectionUnchanged( before, after );
            check( physics.IsBodyAwake( body ) == accepted && physics.IsValid( id ) && physics.IsValid( body ) && physics.IsValid( visual.shapeHandle ) && physics.GetBodyMass( body ) == mass && LengthSquared( physics.GetBodyTransform( body ).position - pose.position ) == 0.0f && physics.GetBodyTransform( body ).rotation.s == pose.rotation.s && physics.GetBodyTransform( body ).rotation.c == pose.rotation.c && LengthSquared( physics.GetBodyLinearVelocity( body ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( body ) == spin, "UI request preserves handles, mass, pose and velocity and uses the existing wake policy" );
            check( text.find( accepted ? "최근 재투영: 성공" : "최근 재투영: 거절" ) != std::string::npos && text.find( "제어점 3" ) != std::string::npos && session.isPlaying() == playing && session.getStepCount() == steps && scene.canUndoSplineEdit() == undo && scene.canRedoSplineEdit() == redo && scene.getSplineControlPoints().size() == points.size(), "success and rejection preserve selected point, playback and pending undo/redo" );
            for( std::size_t point = 0; point < points.size(); ++point ) check( LengthSquared( points[point] - scene.getSplineControlPoints()[point] ) == 0.0f, "reprojection never edits the spline control points" );
            checkTranslationDisplay( text, after );
            checkDisplay( text, after );
            edit( session, "###Friction", "0.42" );
            check( physics.GetShapeFriction( visual.shapeHandle ) == 0.42f, "object inspector still edits the originally selected shape after reprojection" );
            frame( session, playing ? "###SplineRedo" : "###SplineUndo" );
            const auto restored = physics.getSplineRailJointData( id );
            check( scene.getSplineControlPoints().size() == count + ( playing ? 1 : 0 ) && restored.referenceDistance == after.referenceDistance && restored.enableRotation == after.enableRotation && restored.rotationMode == after.rotationMode && restored.tangentOffset == after.tangentOffset && restored.targetTranslation == after.targetTranslation && restored.lowerTranslation == after.lowerTranslation && restored.upperTranslation == after.upperTranslation, "real undo/redo restores the pending geometry edit and retains rail control settings and origin" );
        }
}


void checkSegmentReprojectionCrossing()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto body = scene.getImpulseBody();
    const vec2 points[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    physics.SetBodyTransform( body, { { 0, 0 }, {} } );
    check( physics.setSplineRailJointPoints( id, points ), "segment UI crossing fixture accepts its geometry" );
    frame( session );
    selectReprojectionTarget( session, "지정 구간", "구간 4" );
    const auto before = physics.getSplineRailJointData( id );
    check( before.parameter.segmentIndex < 2, "list selection alone retains the earlier crossing branch" );
    const auto text = frame( session, "###SplineRailReproject" );
    const auto data = physics.getSplineRailJointData( id );
    check( data.trackingValid && data.parameter.segmentIndex >= 3 && text.find( "최근 재투영: 성공" ) != std::string::npos, "actual target and segment lists route the button to the requested later crossing branch" );
    checkReprojectionSettings( before, data );
    checkTranslationDisplay( text, data );
    checkDisplay( text, data );
    selectReprojectionTarget( session, "지정 구간", "구간 5" );
    const auto last = frame( session, "###SplineRailReproject" );
    check( last.find( "구간 5" ) != std::string::npos && last.find( "최근 재투영: 성공" ) != std::string::npos && physics.getSplineRailJointData( id ).parameter.segmentIndex >= 3, "segment list uses all five world geometry segments rather than four model editor segments" );
    const auto lastData = physics.getSplineRailJointData( id );
    selectReprojectionTarget( session, "지정 구간", "구간 1" );
    checkReprojectionUnchanged( lastData, physics.getSplineRailJointData( id ) );
    frame( session, "###SplineRailReproject" );
    check( physics.getSplineRailJointData( id ).parameter.segmentIndex < 2, "first listed segment selects the earlier crossing branch" );
    selectReprojectionTarget( session, "지정 구간", "구간 4" );
    frame( session, "###SplineRailReproject" );
    check( physics.getSplineRailJointData( id ).parameter.segmentIndex >= 3, "repeated selection can restore the later crossing branch" );
    selectReprojectionTarget( session, "전체 곡선" );
    check( physics.getSplineRailJointData( id ).parameter.segmentIndex >= 3, "returning to global mode does not automatically change the branch" );
    frame( session, "###SplineRailReproject" );
    check( physics.getSplineRailJointData( id ).parameter.segmentIndex < 2, "global button explicitly restores the existing nearest-branch policy" );
}


void checkSegmentReprojectionBoundaries()
{
    for( float distance : { std::nextafter( 0.5f, 0.0f ), 0.5f, std::nextafter( 0.5f, 1.0f ) } )
    {
        demoSession session{ createDemoView };
        session.selectDemo( demoKind::splineRail );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        const auto body = scene.getImpulseBody();
        const vec2 points[] = { { -4, 0 }, { 0, 0 }, { 4, 0 } };
        physics.SetBodyTransform( body, { { -2, 0 }, {} } );
        check( physics.setSplineRailJointPoints( id, points ), "selected distance fixture has two straight segments" );
        physics.SetBodyTransform( body, { { distance, 0 }, {} } );
        physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        frame( session );
        selectReprojectionTarget( session, "지정 구간", "구간 1" );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        check( !physics.IsBodyAwake( body ), "actual segment selection does not wake or reproject the body" );
        const auto text = frame( session, "###SplineRailReproject" );
        const bool accepted = distance <= 0.5f;
        check( text.find( accepted ? "최근 재투영: 성공" : "최근 재투영: 거절" ) != std::string::npos && physics.IsBodyAwake( body ) == accepted, "bounded segment button accepts below/equal and rejects above 0.5m despite a closer neighboring segment" );
        const auto after = physics.getSplineRailJointData( id );
        checkReprojectionSettings( before, after );
        if( !accepted ) checkReprojectionUnchanged( before, after );
        check( physics.GetBodyTransform( body ).position.x == distance && physics.GetBodyTransform( body ).position.y == 0.0f, "segment endpoint distance decisions never move the body" );
        checkTranslationDisplay( text, after );
        selectReprojectionTarget( session, "전체 곡선" );
        physics.SetBodyAwake( body, false );
        check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 성공" ) != std::string::npos, "the explicitly selected global mode can use the closer neighboring segment" );
    }
}

void checkSegmentReprojectionFailureAndTravel()
{
    for( int substeps : { 1, 4, 8 } )
    {
        demoSession session{ createDemoView };
        session.selectDemo( demoKind::splineRail );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        const auto body = scene.getImpulseBody();
        const vec2 points[] = { { -4, 0 }, { 0, 0 }, { 4, 0 } };
        physics.SetBodyTransform( body, { { 1, 0 }, {} } );
        check( physics.setSplineRailJointPoints( id, points ), "loaded segment failure fixture accepts geometry" );
        for( int step = 0; step < 10; ++step ) session.stepOnce( substeps );
        physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        check( LengthSquared( before.force ) > 0.0f, "segment failure fixture has a loaded reaction cache" );
        frame( session );
        selectReprojectionTarget( session, "지정 구간", "구간 1" );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        const auto text = frame( session, "###SplineRailReproject" );
        check( text.find( "최근 재투영: 거절" ) != std::string::npos && !physics.IsBodyAwake( body ), "distant selected segment rejects without falling back or waking the body" );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        selectReprojectionTarget( session, "지정 구간", "구간 2" );
        check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 성공" ) != std::string::npos && !physics.IsBodyAwake( body ), "identical selected projection succeeds without clearing cache or waking" );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        physics.SetGravity( {} );
        physics.SetBodyTransform( body, { { -0.25f, 0 }, {} } );
        selectReprojectionTarget( session, "지정 구간", "구간 1" );
        check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 성공" ) != std::string::npos, "travel fixture explicitly starts on the first segment" );
        physics.setSplineRailJointMotor( id, true, 1.0f, 100.0f );
        for( int step = 0; step < 60; ++step ) session.stepOnce( substeps );
        const auto traveled = physics.getSplineRailJointData( id );
        check( traveled.trackingValid && traveled.parameter.segmentIndex == 1 && std::abs( traveled.currentSpeed - 1.0f ) < 0.001f && physics.GetBodyTransform( body ).position.x > 0.6f, "selected segment limits only reprojection and ordinary motor tracking crosses into the next segment" );
    }
}


void checkSegmentReprojectionEditing()
{
    for( bool playing : { false, true } )
    {
        demoSession session{ createDemoView };
        session.selectDemo( demoKind::splineRail );
        session.setPlaying( playing );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        const auto body = scene.getImpulseBody();
        const auto visual = scene.getShapes()[0];
        physics.SetBodyTransform( body, { Evaluate( { scene.getSplineControlPoints() }, 3, 0.4f ), {} } );
        frame( session );
        frame( session, "###Object" );
        frame( session, visual.label, false, true );
        selectReprojectionTarget( session, "지정 구간", "구간 4" );
        check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 성공" ) != std::string::npos, "editable fixture starts with a valid last-segment selection" );
        edit( session, "###SplineRailReprojectDistance", "-1" );
        check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 거절" ) != std::string::npos, "editable fixture records a rejected request" );
        edit( session, "###SplineRailReprojectDistance", "0.5" );
        frame( session, "###SplineControl" );
        frame( session, "제어점 5", false, true );
        const auto pose = physics.GetBodyTransform( body );
        const auto beforeEdit = physics.getSplineRailJointData( id );
        frame( session, "###SplineRemove" );
        check( scene.getSplineControlPoints().size() == 4 && scene.canUndoSplineEdit() && !scene.canRedoSplineEdit(), "real control-point delete removes the selected last segment and records its edit" );
        const auto edited = physics.getSplineRailJointData( id );
        check( physics.IsValid( id ) && physics.IsValid( visual.shapeHandle ) && LengthSquared( physics.GetBodyTransform( body ).position - pose.position ) == 0.0f && edited.referenceDistance == beforeEdit.referenceDistance && edited.targetTranslation == beforeEdit.targetTranslation && edited.lowerTranslation == beforeEdit.lowerTranslation && edited.upperTranslation == beforeEdit.upperTranslation && edited.rotationMode == beforeEdit.rotationMode && edited.referenceAngle == beforeEdit.referenceAngle, "segment invalidation preserves handles, physical pose and existing control coordinates" );
        for( int step = 0; step < 10; ++step ) session.stepOnce( 4 );
        physics.SetBodyAwake( body, false );
        const auto before = physics.getSplineRailJointData( id );
        const auto steps = session.getStepCount();
        check( LengthSquared( before.force ) > 0.0f, "invalid segment selection has a loaded cache to preserve" );
        auto text = frame( session, "###SplineRailReproject" );
        check( text.find( "구간 재선택" ) != std::string::npos && text.find( "최근 재투영: 거절" ) != std::string::npos && text.find( "제어점 4" ) != std::string::npos, "removed segment requires explicit reselection while retaining the last request and editor selection" );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
        check( !physics.IsBodyAwake( body ) && session.isPlaying() == playing && session.getStepCount() == steps && scene.canUndoSplineEdit() && !scene.canRedoSplineEdit(), "disabled invalid selection preserves sleep, playback and pending geometry undo" );
        frame( session, "###SplineUndo" );
        text = frame( session );
        check( scene.getSplineControlPoints().size() == 5 && !scene.canUndoSplineEdit() && scene.canRedoSplineEdit() && text.find( "구간 재선택" ) == std::string::npos && text.find( "구간 4" ) != std::string::npos && text.find( "최근 재투영: 거절" ) != std::string::npos, "undo restores the same selected segment number without silently replacing it or reprojecting" );
        frame( session, "###SplineRedo" );
        check( frame( session ).find( "구간 재선택" ) != std::string::npos && scene.getSplineControlPoints().size() == 4, "redo invalidates the retained segment number again" );
        selectReprojectionTarget( session, "전체 곡선" );
        text = frame( session, "###SplineRailReproject" );
        check( text.find( "최근 재투영: 성공" ) != std::string::npos, "explicit global mode ignores the old invalid segment selection" );
        selectReprojectionTarget( session, "지정 구간" );
        check( frame( session ).find( "구간 재선택" ) != std::string::npos, "returning to selected mode retains its invalid number until reselection" );
        selectReprojectionTarget( session, "지정 구간", "구간 1" );
        physics.SetBodyTransform( body, { Evaluate( { scene.getSplineControlPoints() }, 0, 0.4f ), {} } );
        physics.SetBodyAwake( body, false );
        text = frame( session, "###SplineRailReproject" );
        check( text.find( "최근 재투영: 성공" ) != std::string::npos && physics.IsBodyAwake( body ), "actual segment list reselection re-enables the explicit action with the new index" );
        check( scene.canUndoSplineEdit() && !scene.canRedoSplineEdit() && session.isPlaying() == playing && session.getStepCount() == steps, "scope changes and reprojecting do not add or replace geometry history or simulation steps" );
        edit( session, "###Friction", "0.42" );
        check( physics.GetShapeFriction( visual.shapeHandle ) == 0.42f && frame( session ).find( "제어점 4" ) != std::string::npos, "segment selection remains independent of object and control-point inspector selections" );
        session.selectDemo( demoKind::playground );
        session.selectDemo( demoKind::splineRail );
        text = frame( session );
        check( text.find( "전체 곡선" ) != std::string::npos && text.find( "재투영 구간" ) == std::string::npos && text.find( "최근 재투영:" ) == std::string::npos, "new views reset segment targeting to global and discard the old request result" );
    }
}


std::vector<ImDrawVert> highlightFrame( demoSession& session )
{
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0, 0 } );
    ImGui::SetNextWindowSize( { 1000, 800 } );
    ImGui::Begin( "Spline rail segment highlight" );
    auto* list = ImGui::GetWindowDrawList();
    const int start = list->VtxBuffer.Size;
    debugCamera camera{};
    debugDraw draw{ list, camera, { 0, 0 }, { 1000, 800 } };
    session.getDemo().draw( draw );
    std::vector<ImDrawVert> vertices;
    for( int i = start; i < list->VtxBuffer.Size; ++i )
        if( list->VtxBuffer[i].col == IM_COL32( 255, 220, 80, 255 ) ) vertices.push_back( list->VtxBuffer[i] );
    ImGui::End();
    ImGui::Render();
    return vertices;
}

bool highlightNear( const std::vector<ImDrawVert>& vertices, vec2 point, float pixels = 7.0f )
{
    const vec2 screen{ 500.0f + 60.0f * point.x, 400.0f - 60.0f * point.y };
    for( const auto& vertex : vertices )
        if( LengthSquared( vec2{ vertex.pos.x, vertex.pos.y } - screen ) <= pixels * pixels ) return true;
    return false;
}

void checkSegmentHighlight()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const vec2 points[] = { { -4, 0 }, { -2, 0 }, { 0, 0 }, { 2, 0 }, { 4, 0 } };
    check( physics.setSplineRailJointPoints( id, points ), "highlight fixture uses straight world geometry" );
    frame( session );
    check( highlightFrame( session ).empty(), "global reprojection mode does not draw a selected segment highlight" );
    selectReprojectionTarget( session, "지정 구간", "구간 2" );
    const auto vertices = highlightFrame( session );
    check( !vertices.empty() && highlightNear( vertices, { -2, 0 } ) && highlightNear( vertices, { 0, 0 } ), "actual segment list draws the selected curve and both endpoint markers" );
    check( !highlightNear( vertices, { -4, 0 } ) && !highlightNear( vertices, { 2, 0 } ), "highlight excludes neighboring segment endpoints" );
}


void checkHighlightCurveAndTransform()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const vec2 points[] = { { -2, 0 }, { 0, 2 }, { 2, 0 }, { 4, 2 } };
    check( physics.setSplineRailJointPoints( id, points ), "curved highlight fixture accepts its world geometry" );
    frame( session );
    selectReprojectionTarget( session, "지정 구간", "구간 2" );
    auto vertices = highlightFrame( session );
    check( highlightNear( vertices, { 0.5f, 1.6875f }, 4.0f ) && highlightNear( vertices, { 1.5f, 0.3125f }, 4.0f ), "highlight follows the cubic interior rather than a chord between control points" );
    check( highlightNear( vertices, { 0, 2 } ) && highlightNear( vertices, { 2, 0 } ) && !highlightNear( vertices, { -2, 0 } ) && !highlightNear( vertices, { 4, 2 } ), "curved highlight contains only its selected endpoints" );
    const auto rail = physics.getSplineRailJointData( id );
    physics.SetBodyTransform( rail.bodyA, { { 1, -1 }, { 0, 1 } } );
    vertices = highlightFrame( session );
    check( highlightNear( vertices, { -1, -1 } ) && highlightNear( vertices, { 1, 1 } ) && highlightNear( vertices, { -0.6875f, -0.5f }, 4.0f ) && highlightNear( vertices, { 0.6875f, 0.5f }, 4.0f ), "highlight endpoints and cubic interior apply the current rail translation and ninety-degree rotation exactly once" );
    check( !highlightNear( vertices, { 0, 2 } ) && !highlightNear( vertices, { 2, 0 } ), "transformed overlay does not retain stale local-space endpoints" );
}

void checkHighlightCrossing()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const vec2 points[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    check( physics.setSplineRailJointPoints( id, points ), "highlight crossing fixture has five world segments" );
    frame( session );
    selectReprojectionTarget( session, "지정 구간", "구간 1" );
    auto vertices = highlightFrame( session );
    check( highlightNear( vertices, { -2, 0 } ) && highlightNear( vertices, { 0, 0 } ) && !highlightNear( vertices, { 2, -2 } ), "first crossing segment highlights its own branch and shared crossing" );
    const auto before = physics.getSplineRailJointData( id );
    selectReprojectionTarget( session, "지정 구간", "구간 4" );
    vertices = highlightFrame( session );
    check( highlightNear( vertices, { 2, -2 } ) && highlightNear( vertices, { 0, 0 } ) && !highlightNear( vertices, { -2, 0 } ), "later crossing segment replaces the first branch highlight" );
    checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
    selectReprojectionTarget( session, "지정 구간", "구간 5" );
    vertices = highlightFrame( session );
    check( highlightNear( vertices, { 0, 0 } ) && highlightNear( vertices, { -2, 2 } ) && !highlightNear( vertices, { 2, -2 } ), "highlight uses the actual fifth world segment rather than the four-segment editor model" );
    selectReprojectionTarget( session, "전체 곡선" );
    check( highlightFrame( session ).empty(), "switching to global mode removes the selected overlay immediately" );
}

void checkHighlightMarkersAndLabel()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    const vec2 points[] = { { -4, 0 }, { -2, 0 }, { 0, 0 }, { 2, 0 }, { 4, 0 } };
    check( scene.getWorld().setSplineRailJointPoints( scene.getSplineRailJoint(), points ), "label fixture uses a horizontal segment" );
    frame( session );
    selectReprojectionTarget( session, "지정 구간", "구간 2" );
    const auto vertices = highlightFrame( session );
    for( float x : { 380.0f, 500.0f } )
    {
        bool marker = false;
        for( const auto& vertex : vertices )
        {
            const float r2 = LengthSquared( vec2{ vertex.pos.x - x, vertex.pos.y - 400.0f } );
            marker |= r2 >= 25.0f && r2 <= 49.0f;
        }
        check( marker, "both endpoints draw a visible marker larger than the curve stroke" );
    }
    bool label = false, thickLine = false;
    for( const auto& vertex : vertices )
    {
        label |= vertex.pos.x >= 448.0f && vertex.pos.x < 490.0f && vertex.pos.y >= 380.0f && vertex.pos.y < 394.0f;
        thickLine |= std::abs( vertex.pos.x - 380.0f ) < 0.75f && vertex.pos.y - 400.0f > 1.0f && vertex.pos.y - 400.0f < 4.0f;
    }
    check( label && thickLine, "selected segment draws its number near the curve midpoint and a stroke thicker than the base rail" );
}


void checkSameHighlight( const std::vector<ImDrawVert>& first, const std::vector<ImDrawVert>& second )
{
    check( !first.empty() && first.size() == second.size(), "restored geometry reproduces the selected overlay" );
    for( std::size_t i = 0; i < first.size(); ++i )
        check( first[i].pos.x == second[i].pos.x && first[i].pos.y == second[i].pos.y && first[i].uv.x == second[i].uv.x && first[i].uv.y == second[i].uv.y, "undo/redo restores curve, markers and label geometry exactly" );
}

void checkHighlightEditing()
{
    for( bool playing : { false, true } )
    {
        demoSession session{ createDemoView };
        session.selectDemo( demoKind::splineRail );
        session.setPlaying( playing );
        auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
        auto& physics = scene.getWorld();
        const auto id = scene.getSplineRailJoint();
        frame( session );
        selectReprojectionTarget( session, "지정 구간", "구간 4" );
        const auto original = highlightFrame( session );
        check( highlightNear( original, { 2, 0 } ) && highlightNear( original, { 4, 0 } ), "editable highlight starts on the selected last segment" );
        frame( session, "###SplineControl" );
        frame( session, "제어점 5", false, true );
        frame( session, "###SplineRemove" );
        check( scene.getSplineControlPoints().size() == 4 && scene.canUndoSplineEdit() && highlightFrame( session ).empty(), "real deletion hides the invalid selection rather than clamping it to a remaining segment" );
        const auto steps = session.getStepCount();
        frame( session, "###SplineUndo" );
        checkSameHighlight( original, highlightFrame( session ) );
        check( scene.getSplineControlPoints().size() == 5 && !scene.canUndoSplineEdit() && scene.canRedoSplineEdit(), "drawing after undo preserves the pending redo" );
        frame( session, "###SplineRedo" );
        check( highlightFrame( session ).empty() && scene.canUndoSplineEdit() && !scene.canRedoSplineEdit(), "drawing after redo keeps the original invalid number and geometry history" );
        selectReprojectionTarget( session, "지정 구간", "구간 1" );
        const auto reselected = highlightFrame( session );
        check( highlightNear( reselected, { -4, 0 } ) && highlightNear( reselected, { -2, 0 } ) && session.isPlaying() == playing && session.getStepCount() == steps && physics.IsValid( id ), "actual reselection restores only the new segment highlight and preserves playback, steps and joint handle" );
    }
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    frame( session );
    selectReprojectionTarget( session, "지정 구간", "구간 2" );
    const auto original = highlightFrame( session );
    demoInput drag{};
    drag.mousePosition = { -2, 0 };
    drag.mousePressed = true;
    drag.mouseHeld = true;
    session.handleInput( drag, true );
    check( scene.getTerrainPointDragIndex() == 1, "real editor input starts dragging the selected segment endpoint" );
    drag.mousePosition = { -2, 1 };
    drag.mousePressed = false;
    session.handleInput( drag, true );
    drag.mouseHeld = false;
    session.handleInput( drag, true );
    const auto moved = highlightFrame( session );
    check( highlightNear( moved, { -2, 1 } ) && !highlightNear( moved, { -2, 0 } ) && scene.canUndoSplineEdit(), "dragging a control point immediately moves the highlighted endpoint" );
    frame( session, "###SplineUndo" );
    checkSameHighlight( original, highlightFrame( session ) );
    frame( session, "###SplineRedo" );
    checkSameHighlight( moved, highlightFrame( session ) );
}

void checkHighlightState()
{
    for( bool playing : { false, true } )
        for( bool failed : { false, true } )
        {
            demoSession session{ createDemoView };
            session.selectDemo( demoKind::splineRail );
            session.setPlaying( playing );
            auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
            auto& physics = scene.getWorld();
            const auto id = scene.getSplineRailJoint();
            const auto body = scene.getImpulseBody();
            const auto visual = scene.getShapes()[0];
            check( scene.insertSplineControlPoint( 1 ), "read-only drawing fixture has a pending spline edit" );
            if( playing ) check( scene.undoSplineEdit(), "playing drawing fixture has a pending redo" );
            physics.SetBodyTransform( body, { Evaluate( { scene.getSplineControlPoints() }, 2, 0.4f ), rot2::FromRadians( 0.3f ) } );
            physics.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.5f, 0.8f, playing );
            physics.setSplineRailJointRotationEnabled( id, !playing );
            physics.setSplineRailJointRotationReference( id, 7.123456f );
            physics.setSplineRailJointSpring( id, true, 0.5f, 0.7f, 1.0f );
            physics.setSplineRailJointLimit( id, true, 0.0f, 8.0f );
            physics.setSplineRailJointMotor( id, true, 1.2f, 0.2f );
            for( int step = 0; step < 5; ++step ) session.stepOnce( 4 );
            check( physics.rebaseSplineRailJoint( id ), "read-only drawing fixture has a nondefault origin" );
            if( failed )
            {
                physics.setSplineRailJointSpring( id, true, std::numeric_limits<float>::max(), 0.7f, 1.0f );
                session.stepOnce( 4 );
                physics.setSplineRailJointSpring( id, true, 0.5f, 0.7f, 1.0f );
            }
            frame( session );
            frame( session, "###Object" );
            frame( session, visual.label, false, true );
            frame( session, "###SplineControl" );
            frame( session, "제어점 3", false, true );
            selectReprojectionTarget( session, "지정 구간", "구간 3" );
            if( !playing || failed ) physics.SetBodyAwake( body, false );
            const auto before = physics.getSplineRailJointData( id );
            const auto pose = physics.GetBodyTransform( body );
            const auto velocity = physics.GetBodyLinearVelocity( body );
            const float spin = physics.GetBodyAngularVelocity( body ), mass = physics.GetBodyMass( body ), friction = physics.GetShapeFriction( visual.shapeHandle );
            const bool awake = physics.IsBodyAwake( body ), undo = scene.canUndoSplineEdit(), redo = scene.canRedoSplineEdit();
            const auto steps = session.getStepCount();
            const std::vector<vec2> points( scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() );
            check( before.trackingValid != failed, "drawing fixture covers valid and failed stored tracking" );
            if( !failed ) check( LengthSquared( before.force ) > 0.0f, "valid drawing fixture has cached reactions" );
            for( int draw = 0; draw < 3; ++draw )
            {
                check( !highlightFrame( session ).empty(), "selected geometry remains visible even when tracking has failed" );
                checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
            }
            check( physics.IsBodyAwake( body ) == awake && physics.IsValid( id ) && physics.IsValid( visual.shapeHandle ) && physics.GetBodyMass( body ) == mass && physics.GetShapeFriction( visual.shapeHandle ) == friction && LengthSquared( physics.GetBodyTransform( body ).position - pose.position ) == 0.0f && physics.GetBodyTransform( body ).rotation.s == pose.rotation.s && physics.GetBodyTransform( body ).rotation.c == pose.rotation.c && LengthSquared( physics.GetBodyLinearVelocity( body ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( body ) == spin, "repeated highlighting preserves sleep, handles, mass, material, pose and velocities" );
            check( session.isPlaying() == playing && session.getStepCount() == steps && scene.canUndoSplineEdit() == undo && scene.canRedoSplineEdit() == redo && scene.getSplineControlPoints().size() == points.size(), "highlighting preserves playback, steps and pending geometry history" );
            for( std::size_t point = 0; point < points.size(); ++point ) check( LengthSquared( points[point] - scene.getSplineControlPoints()[point] ) == 0.0f, "drawing never edits control points" );
            check( frame( session ).find( "제어점 3" ) != std::string::npos, "drawing preserves inspector control-point selection" );
            edit( session, "###Friction", "0.42" );
            check( physics.GetShapeFriction( visual.shapeHandle ) == 0.42f, "drawing preserves inspector object selection" );
        }
}

void checkHighlightSamplingFailure()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    frame( session );
    selectReprojectionTarget( session, "지정 구간", "구간 2" );
    check( !highlightFrame( session ).empty(), "sampling failure fixture starts with a visible selection" );
    const vec2 points[] = { { -2000000, 0 }, { 0, 2000000 }, { 2000000, 0 }, { 4000000, 2000000 } };
    physics.SetBodyTransform( scene.getImpulseBody(), { points[0], {} } );
    check( physics.setSplineRailJointPoints( id, points ), "large regular geometry remains valid for rail tracking" );
    check( SampleAdaptive( { points }, { 1, 0.0f }, { 1, 1.0f }, 0.01f ).empty(), "large curve cannot satisfy the existing 1cm adaptive sampling budget" );
    physics.SetBodyAwake( scene.getImpulseBody(), false );
    const auto before = physics.getSplineRailJointData( id );
    check( highlightFrame( session ).empty() && !physics.IsBodyAwake( scene.getImpulseBody() ), "failed sampling hides the entire overlay without stale endpoints, label or wakeup" );
    checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
}


std::string mouseFrame( demoSession& session, vec2 point, bool held, bool alt, bool ctrl = false, float scale = 60.0f, bool enabled = true, bool jump = false )
{
    debugCamera camera{};
    camera.pixelsPerMeter = scale;
    const auto screen = camera.WorldToScreen( point, { 0, 0 }, { 1000, 800 } );
    auto& io = ImGui::GetIO();
    io.AddMousePosEvent( screen.x, screen.y );
    io.AddKeyEvent( ImGuiMod_Alt, alt );
    io.AddKeyEvent( ImGuiMod_Ctrl, ctrl );
    io.AddMouseButtonEvent( ImGuiMouseButton_Left, held );
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0, 0 } );
    ImGui::SetNextWindowSize( { 340, 4000 } );
    ImGui::Begin( "Spline rail rotation inspector" );
    demoInput input{};
    input.mousePosition = camera.ScreenToWorld( io.MousePos, { 0, 0 }, { 1000, 800 } );
    input.mousePressed = ImGui::IsMouseClicked( ImGuiMouseButton_Left );
    input.mouseHeld = ImGui::IsMouseDown( ImGuiMouseButton_Left );
    input.selectReprojectionSegment = io.KeyAlt;
    input.insertControlPoint = io.KeyCtrl;
    input.pixelsPerMeter = scale;
    input.jumpPressed = jump;
    session.handleInput( input, enabled );
    ImGui::LogToBuffer();
    session.getDemo().drawControls();
    const std::string text = GImGui->LogBuffer.c_str();
    ImGui::LogFinish();
    ImGui::End();
    ImGui::Render();
    return text;
}

std::string altClick( demoSession& session, vec2 point, float scale = 60.0f )
{
    mouseFrame( session, point, false, false, false, scale );
    const auto text = mouseFrame( session, point, true, true, false, scale );
    mouseFrame( session, point, false, false, false, scale );
    return text;
}

void checkAltSelection()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    const vec2 points[] = { { -4, 0 }, { -2, 0 }, { 0, 0 }, { 2, 0 }, { 4, 0 } };
    check( scene.getWorld().setSplineRailJointPoints( scene.getSplineRailJoint(), points ), "Alt-click fixture uses a straight world curve" );
    frame( session );
    const auto text = altClick( session, { 1, 0 } );
    check( text.find( "지정 구간" ) != std::string::npos && text.find( "구간 3" ) != std::string::npos, "actual Alt and mouse events select the clicked segment in the existing inspector list" );
    const auto vertices = highlightFrame( session );
    check( highlightNear( vertices, { 0, 0 } ) && highlightNear( vertices, { 2, 0 } ) && !highlightNear( vertices, { -4, 0 } ), "Alt selection updates the existing highlight without a separate apply action" );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const auto before = physics.getSplineRailJointData( id );
    check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 거절" ) != std::string::npos, "explicit button uses the Alt-selected distant segment instead of a closer global candidate" );
    checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
    physics.SetBodyTransform( scene.getImpulseBody(), { { 1, 0.25f }, {} } );
    physics.SetBodyAwake( scene.getImpulseBody(), false );
    altClick( session, { 1, 0 } );
    check( !physics.IsBodyAwake( scene.getImpulseBody() ), "another Alt selection does not apply the pending projection" );
    check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 성공" ) != std::string::npos && physics.IsBodyAwake( scene.getImpulseBody() ), "explicit button applies a valid projection using the clicked segment and existing allowance" );
}


void checkAltBoundaries()
{
    for( float scale : { 10.0f, 20.0f, 55.0f, 60.0f, 80.0f, 160.0f, 240.0f } )
        for( int pixels : { 9, 10, 11 } )
            for( bool selected : { false, true } )
            {
                demoSession session{ createDemoView };
                session.selectDemo( demoKind::splineRail );
                auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
                auto& physics = scene.getWorld();
                const auto id = scene.getSplineRailJoint();
                const vec2 points[] = { { -4, 0 }, { -2, 0 }, { 0, 0 }, { 2, 0 }, { 4, 0 } };
                check( physics.setSplineRailJointPoints( id, points ), "pixel boundary fixture accepts a straight world rail" );
                frame( session );
                if( selected ) selectReprojectionTarget( session, "지정 구간", "구간 1" );
                physics.SetBodyAwake( scene.getImpulseBody(), false );
                const auto before = physics.getSplineRailJointData( id );
                const auto text = altClick( session, { 1, static_cast<float>( pixels ) / scale }, scale );
                const bool hit = pixels <= 10;
                check( text.find( hit ? "구간 3" : selected ? "구간 1" : "전체 곡선" ) != std::string::npos, "actual 9/10/11 pixel clicks accept an inclusive 10px radius at different zooms without replacing a missed selection" );
                const auto vertices = highlightFrame( session );
                if( hit ) check( highlightNear( vertices, { 0, 0 } ) && highlightNear( vertices, { 2, 0 } ), "accepted pixel hit immediately highlights the clicked segment" );
                else if( selected ) check( highlightNear( vertices, { -4, 0 } ) && !highlightNear( vertices, { 2, 0 } ), "missed click retains the existing segment highlight" );
                else check( vertices.empty(), "missed global-mode click does not introduce a selection" );
                checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
                check( !physics.IsBodyAwake( scene.getImpulseBody() ) && !physics.IsValid( scene.getMouseJoint() ) && !scene.getTerrainPointDragIndex() && !scene.canUndoSplineEdit(), "pixel hit and miss never wake, drag or edit geometry" );
            }
}

void checkAltGeometry()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    const vec2 curve[] = { { -2, 0 }, { 0, 2 }, { 2, 0 }, { 4, 2 } };
    check( physics.setSplineRailJointPoints( id, curve ), "Alt curve fixture accepts curved world geometry" );
    frame( session );
    auto text = altClick( session, { 0.5f, 1.6875f } );
    check( text.find( "구간 2" ) != std::string::npos && highlightNear( highlightFrame( session ), { 0.5f, 1.6875f }, 4.0f ), "click hits the true cubic more than 10px away from its control chord" );
    const auto rail = physics.getSplineRailJointData( id );
    physics.SetBodyTransform( rail.bodyA, { { 2, 1 }, { 0, 1 } } );
    text = altClick( session, { 0.3125f, 1.5f } );
    check( text.find( "구간 2" ) != std::string::npos, "curve hit testing applies the inverse translation and ninety-degree rail rotation" );
    const auto vertices = highlightFrame( session );
    check( highlightNear( vertices, { 0, 1 } ) && highlightNear( vertices, { 2, 3 } ), "transformed click and highlight agree on the selected world-space endpoints" );

    demoSession crossing{ createDemoView };
    crossing.selectDemo( demoKind::splineRail );
    auto& crossed = static_cast<rigidBodyDemo&>( crossing.getDemo() );
    const vec2 points[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    check( crossed.getWorld().setSplineRailJointPoints( crossed.getSplineRailJoint(), points ), "Alt crossing fixture accepts five world segments" );
    frame( crossing );
    text = altClick( crossing, Evaluate( { points }, 3, 0.5f ) );
    check( text.find( "구간 4" ) != std::string::npos && highlightNear( highlightFrame( crossing ), { 2, -2 } ), "click beside the crossing selects its later branch" );
    text = altClick( crossing, { 0, 0 } );
    check( text.find( "구간 1" ) != std::string::npos && highlightNear( highlightFrame( crossing ), { -2, 0 } ), "exact crossing tie uses the existing earliest nearest-point branch policy" );
    text = altClick( crossing, Evaluate( { points }, 4, 0.5f ) );
    check( text.find( "구간 5" ) != std::string::npos && highlightNear( highlightFrame( crossing ), { -2, 2 } ), "direct selection uses the fifth current world segment rather than the editor model count" );
}

void checkAltGestureConflicts()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    const auto count = scene.getSplineControlPoints().size();
    const vec2 point = Evaluate( { scene.getSplineControlPoints() }, 1, 0.5f );
    mouseFrame( session, point, false, false );
    auto text = mouseFrame( session, point, true, true, true );
    check( scene.getSplineControlPoints().size() == count + 1 && scene.canUndoSplineEdit() && text.find( "전체 곡선" ) != std::string::npos && highlightFrame( session ).empty(), "Ctrl+Alt click preserves Ctrl insertion priority instead of selecting a rail segment" );
    mouseFrame( session, point, true, true, false );
    check( scene.getSplineControlPoints().size() == count + 1 && highlightFrame( session ).empty(), "releasing Ctrl during an insertion hold does not turn it into Alt selection" );
    mouseFrame( session, point, false, false );
    frame( session, "###SplineUndo" );
    check( scene.getSplineControlPoints().size() == count, "existing insertion undo remains usable after the modifier conflict" );

    mouseFrame( session, { -2, 0 }, true, false );
    check( scene.getTerrainPointDragIndex() == 1, "ordinary click still starts control-point dragging" );
    mouseFrame( session, { -2, 1 }, true, true );
    check( scene.getSplineControlPoints()[1].y == 1.0f && highlightFrame( session ).empty(), "pressing Alt during an ordinary drag does not change its purpose" );
    mouseFrame( session, { -2, 1 }, false, false );
    frame( session, "###SplineUndo" );
    const std::vector<vec2> points( scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() );
    text = mouseFrame( session, { 0, 2 }, true, true );
    check( text.find( "지정 구간" ) != std::string::npos && !scene.getTerrainPointDragIndex(), "Alt over a control point selects the curve without starting an editor drag" );
    const auto selected = highlightFrame( session );
    mouseFrame( session, { 0, 3 }, true, false, true );
    checkSameHighlight( selected, highlightFrame( session ) );
    check( scene.getSplineControlPoints().size() == points.size() && !scene.getTerrainPointDragIndex() && !scene.getWorld().IsValid( scene.getMouseJoint() ), "releasing Alt or pressing Ctrl during a selection hold cannot start a drag or insertion" );
    for( std::size_t i = 0; i < points.size(); ++i ) check( LengthSquared( points[i] - scene.getSplineControlPoints()[i] ) == 0.0f, "selection gesture leaves every control point unchanged" );
    mouseFrame( session, { 0, 3 }, false, false );

    auto& physics = scene.getWorld();
    const auto body = scene.getImpulseBody();
    physics.SetBodyTransform( body, { { 0, -3 }, {} } );
    physics.SetBodyAwake( body, false );
    mouseFrame( session, { 0, -3 }, true, true );
    mouseFrame( session, { 1, -3 }, true, false );
    check( !physics.IsValid( scene.getMouseJoint() ) && !physics.IsBodyAwake( body ) && physics.GetBodyTransform( body ).position.y == -3.0f, "an Alt miss over the body consumes the whole left gesture without creating a mouse joint" );
    mouseFrame( session, { 1, -3 }, false, false );
    mouseFrame( session, { 0, -3 }, true, false );
    check( physics.IsValid( scene.getMouseJoint() ), "a fresh ordinary body click still starts the existing mouse joint" );
    mouseFrame( session, { 0, -3 }, false, false );
}


void checkAltCancellationAndOtherInputs()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    mouseFrame( session, { -2, 0 }, false, false );
    auto text = mouseFrame( session, { -2, 0 }, true, true, false, 60.0f, false );
    check( text.find( "전체 곡선" ) != std::string::npos && !scene.getTerrainPointDragIndex(), "disabled canvas input ignores a real Alt press and cancels pending input" );
    text = mouseFrame( session, { -2, 0 }, true, true );
    check( text.find( "전체 곡선" ) != std::string::npos, "regaining canvas focus while held does not synthesize another click" );
    mouseFrame( session, { -2, 0 }, false, false );
    text = mouseFrame( session, { -2, 0 }, true, true );
    check( text.find( "지정 구간" ) != std::string::npos, "fresh click after focus cancellation can select normally" );
    session.setPlaying( true );
    session.setPlaying( false );
    mouseFrame( session, { -2, 0 }, false, false );
    mouseFrame( session, { -2, 0 }, true, false );
    check( scene.getTerrainPointDragIndex() == 1 && !session.isPlaying(), "pause cancellation preserves ordinary point dragging on the next fresh click" );
    mouseFrame( session, { -2, 0 }, false, false );

    demoSession baseline{ createDemoView };
    baseline.selectDemo( demoKind::splineRail );
    auto& reference = static_cast<rigidBodyDemo&>( baseline.getDemo() );
    mouseFrame( baseline, { 0, 2 }, false, false, false, 60.0f, true, true );
    demoSession simultaneous{ createDemoView };
    simultaneous.selectDemo( demoKind::splineRail );
    auto& combined = static_cast<rigidBodyDemo&>( simultaneous.getDemo() );
    mouseFrame( simultaneous, { 0, 2 }, false, false );
    mouseFrame( simultaneous, { 0, 2 }, true, true, false, 60.0f, true, true );
    check( combined.getWorld().GetBodyLinearVelocity( combined.getImpulseBody() ).y == reference.getWorld().GetBodyLinearVelocity( reference.getImpulseBody() ).y && reference.getWorld().GetBodyLinearVelocity( reference.getImpulseBody() ).y > 0.0f, "selection consumes only the left gesture and forwards an independent simultaneous jump impulse" );
    mouseFrame( simultaneous, { 0, 2 }, false, false );

    demoSession path{ createDemoView };
    path.selectDemo( demoKind::splinePath );
    auto& pathScene = static_cast<rigidBodyDemo&>( path.getDemo() );
    mouseFrame( path, { -2, 0 }, false, false );
    mouseFrame( path, { -2, 0 }, true, true );
    check( pathScene.getTerrainPointDragIndex() == 1, "Alt leaves the existing spline path point drag behavior intact" );
    mouseFrame( path, { -2, 1 }, true, false );
    mouseFrame( path, { -2, 1 }, false, false );
    check( pathScene.getSplineControlPoints()[1].y == 1.0f && pathScene.canUndoSplineEdit(), "other demo dragging and edit history retain their existing behavior" );
}

void checkAltState()
{
    for( bool playing : { false, true } )
        for( bool failed : { false, true } )
        {
            demoSession session{ createDemoView };
            session.selectDemo( demoKind::splineRail );
            session.setPlaying( playing );
            auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
            auto& physics = scene.getWorld();
            const auto id = scene.getSplineRailJoint();
            const auto body = scene.getImpulseBody();
            const auto visual = scene.getShapes()[0];
            check( scene.insertSplineControlPoint( 1 ), "Alt state fixture has a pending geometry edit" );
            if( playing ) check( scene.undoSplineEdit(), "Alt state fixture retains a pending redo" );
            physics.SetBodyTransform( body, { Evaluate( { scene.getSplineControlPoints() }, 2, 0.4f ), rot2::FromRadians( 0.3f ) } );
            physics.setSplineRailJointRotation( id, splineRailRotationMode::followTangent, 4.5f, 0.8f, playing );
            physics.setSplineRailJointRotationEnabled( id, !playing );
            physics.setSplineRailJointRotationReference( id, 7.123456f );
            physics.setSplineRailJointSpring( id, true, 0.5f, 0.7f, 1.0f );
            physics.setSplineRailJointLimit( id, true, 0.0f, 8.0f );
            physics.setSplineRailJointMotor( id, true, 1.2f, 0.2f );
            for( int step = 0; step < 5; ++step ) session.stepOnce( 4 );
            check( physics.rebaseSplineRailJoint( id ), "Alt fixture starts with a rebased translation origin" );
            if( failed )
            {
                physics.setSplineRailJointSpring( id, true, std::numeric_limits<float>::max(), 0.7f, 1.0f );
                session.stepOnce( 4 );
                physics.setSplineRailJointSpring( id, true, 0.5f, 0.7f, 1.0f );
            }
            frame( session );
            frame( session, "###Object" );
            frame( session, visual.label, false, true );
            frame( session, "###SplineControl" );
            frame( session, "제어점 3", false, true );
            edit( session, "###SplineRailReprojectDistance", "-1" );
            check( frame( session, "###SplineRailReproject" ).find( "최근 재투영: 거절" ) != std::string::npos, "Alt fixture has a last explicit request result to preserve" );
            edit( session, "###SplineRailReprojectDistance", "0.5" );
            physics.SetBodyAwake( body, false );
            const auto before = physics.getSplineRailJointData( id );
            const auto pose = physics.GetBodyTransform( body );
            const auto velocity = physics.GetBodyLinearVelocity( body );
            const float spin = physics.GetBodyAngularVelocity( body ), mass = physics.GetBodyMass( body );
            const auto steps = session.getStepCount();
            const bool undo = scene.canUndoSplineEdit(), redo = scene.canRedoSplineEdit();
            const std::vector<vec2> points( scene.getSplineControlPoints().begin(), scene.getSplineControlPoints().end() );
            check( before.trackingValid != failed, "Alt fixture covers valid and failed tracking without implicit recovery" );
            if( !failed ) check( LengthSquared( before.force ) > 0.0f, "Alt fixture has a loaded reaction cache" );
            auto text = altClick( session, Evaluate( { points }, 2, 0.5f ) );
            check( text.find( "구간 3" ) != std::string::npos && text.find( "최근 재투영: 거절" ) != std::string::npos && text.find( "제어점 3" ) != std::string::npos, "hit updates only the target list and retains the explicit request result and editor selection" );
            checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
            text = altClick( session, { 0, -5 } );
            check( text.find( "구간 3" ) != std::string::npos && text.find( "최근 재투영: 거절" ) != std::string::npos, "miss retains selection and last request result" );
            checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
            check( !physics.IsBodyAwake( body ) && !physics.IsValid( scene.getMouseJoint() ) && !scene.getTerrainPointDragIndex() && physics.IsValid( id ) && physics.IsValid( visual.shapeHandle ) && physics.GetBodyMass( body ) == mass && LengthSquared( physics.GetBodyTransform( body ).position - pose.position ) == 0.0f && physics.GetBodyTransform( body ).rotation.s == pose.rotation.s && physics.GetBodyTransform( body ).rotation.c == pose.rotation.c && LengthSquared( physics.GetBodyLinearVelocity( body ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( body ) == spin, "Alt hit/miss preserve sleep, handles, mass, pose and velocities without dragging or reprojection" );
            check( session.isPlaying() == playing && session.getStepCount() == steps && scene.canUndoSplineEdit() == undo && scene.canRedoSplineEdit() == redo && scene.getSplineControlPoints().size() == points.size(), "Alt hit/miss preserve playback, steps and pending undo/redo" );
            for( std::size_t i = 0; i < points.size(); ++i ) check( LengthSquared( points[i] - scene.getSplineControlPoints()[i] ) == 0.0f, "Alt click never changes the geometry" );
            edit( session, "###Friction", "0.42" );
            check( physics.GetShapeFriction( visual.shapeHandle ) == 0.42f, "Alt selection retains the selected object inspector target" );
            frame( session, playing ? "###SplineRedo" : "###SplineUndo" );
            check( scene.getSplineControlPoints().size() == ( playing ? 6 : 5 ) && physics.getSplineRailJointData( id ).referenceDistance == before.referenceDistance, "Alt click does not replace the original undo/redo entry or change its stored rail origin" );
        }
}

void checkAltInvalidInput()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    auto& physics = scene.getWorld();
    const auto id = scene.getSplineRailJoint();
    frame( session );
    selectReprojectionTarget( session, "지정 구간", "구간 2" );
    physics.SetBodyAwake( scene.getImpulseBody(), false );
    const auto before = physics.getSplineRailJointData( id );
    for( float scale : { 0.0f, -1.0f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::denorm_min() } )
    {
        demoInput input{};
        input.mousePosition = { -4, 0 };
        input.mousePressed = input.mouseHeld = input.selectReprojectionSegment = true;
        input.pixelsPerMeter = scale;
        session.handleInput( input, true );
        session.handleInput( {}, true );
        check( frame( session ).find( "구간 2" ) != std::string::npos && !physics.IsBodyAwake( scene.getImpulseBody() ) && !physics.IsValid( scene.getMouseJoint() ) && !scene.getTerrainPointDragIndex(), "invalid zoom/radius rejects without changing selection or leaking into dragging" );
        checkReprojectionUnchanged( before, physics.getSplineRailJointData( id ) );
    }
    physics.SetBodyTransform( before.bodyA, { { std::numeric_limits<float>::max(), 0 }, {} } );
    physics.SetBodyAwake( scene.getImpulseBody(), false );
    const auto transformed = physics.getSplineRailJointData( id );
    demoInput input{};
    input.mousePosition = { -std::numeric_limits<float>::max(), 0 };
    input.mousePressed = input.mouseHeld = input.selectReprojectionSegment = true;
    check( !IsFinite( InverseTransformPoint( physics.GetBodyTransform( before.bodyA ), input.mousePosition ) ), "failed closest-point fixture overflows only its local query coordinate" );
    session.handleInput( input, true );
    session.handleInput( {}, true );
    check( frame( session ).find( "구간 2" ) != std::string::npos && !physics.IsBodyAwake( scene.getImpulseBody() ), "failed local-space nearest query preserves selection and sleep" );
    checkReprojectionUnchanged( transformed, physics.getSplineRailJointData( id ) );
}


void checkAltEditing()
{
    demoSession session{ createDemoView };
    session.selectDemo( demoKind::splineRail );
    auto& scene = static_cast<rigidBodyDemo&>( session.getDemo() );
    frame( session );
    check( altClick( session, Evaluate( { scene.getSplineControlPoints() }, 3, 0.5f ) ).find( "구간 4" ) != std::string::npos, "click initially selects the last editable segment" );
    frame( session, "###SplineControl" );
    frame( session, "제어점 5", false, true );
    frame( session, "###SplineRemove" );
    check( scene.getSplineControlPoints().size() == 4 && highlightFrame( session ).empty(), "deleting the clicked segment retains its invalid selection until another choice" );
    check( altClick( session, Evaluate( { scene.getSplineControlPoints() }, 0, 0.5f ) ).find( "구간 1" ) != std::string::npos, "click uses current edited geometry and repairs the invalid target selection" );
    const auto selected = highlightFrame( session );
    check( highlightNear( selected, { -4, 0 } ) && highlightNear( selected, { -2, 0 } ) && scene.canUndoSplineEdit() && !scene.canRedoSplineEdit(), "click replaces only the target selection while retaining the deletion history" );
    frame( session, "###SplineUndo" );
    check( scene.getSplineControlPoints().size() == 5 && frame( session ).find( "구간 1" ) != std::string::npos && scene.canRedoSplineEdit(), "undo restores the geometry without reverting the later clicked target" );
    frame( session, "###SplineRedo" );
    checkSameHighlight( selected, highlightFrame( session ) );
    check( scene.getSplineControlPoints().size() == 4 && scene.canUndoSplineEdit() && !scene.canRedoSplineEdit(), "redo restores edited geometry and its clicked highlight without a new history record" );
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
    checkReprojectionButton();
    checkReprojectionBoundaries();
    checkReprojectionInput();
    checkReprojectionRecovery();
    checkReprojectionCrossing();
    checkReprojectionSessionState();
    checkSegmentReprojectionCrossing();
    checkSegmentReprojectionBoundaries();
    checkSegmentReprojectionFailureAndTravel();
    checkSegmentReprojectionEditing();
    checkSegmentHighlight();
    checkHighlightCurveAndTransform();
    checkHighlightCrossing();
    checkHighlightMarkersAndLabel();
    checkHighlightEditing();
    checkHighlightState();
    checkHighlightSamplingFailure();
    checkAltSelection();
    checkAltBoundaries();
    checkAltGeometry();
    checkAltGestureConflicts();
    checkAltCancellationAndOtherInputs();
    checkAltState();
    checkAltInvalidInput();
    checkAltEditing();
}
