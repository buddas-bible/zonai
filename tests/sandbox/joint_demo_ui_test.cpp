#include <cstdio>
#include <cstdlib>
#include <string_view>

#include <imgui.h>
#include <imgui_internal.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"
#include "jointDemoView.h"
#include "rigidBodyDemoUi.h"

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

[[noreturn]] void fail( const char* message )
{
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

}

int main()
{
    ImGui::CreateContext();
    if( !initializeDemoUi() ) fail( "Korean font initialization failed for joint view" );

    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = { 1280.0f, 720.0f };
    io.DeltaTime = 1.0f / 60.0f;
    unsigned char* pixels = nullptr;
    int width = 0, height = 0;
    io.Fonts->GetTexDataAsRGBA32( &pixels, &width, &height );
    io.Fonts->SetTexID( static_cast<ImTextureID>( 1 ) );

    auto view = createJointDemoView( demoKind::prismaticRail );
    int baselineVertices = 0;
    int springVertices = 0;
    int limitVertices = 0;

    for( int frame = 0; frame < 3; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 420.0f, 4000.0f } );
        ImGui::Begin( "Prismatic inspector" );

        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "PrismaticJointInspector" ), 1 );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "PrismaticSpringSettings" ), 1 );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "PrismaticLimitSettings" ), 1 );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "PrismaticMotorSettings" ), 1 );

        if( frame == 0 )
        {
            // 기본값이 motor+limit이므로 개별 overlay 비교는 free에서 시작함.
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "PrismaticPresetFree" );
        }
        else if( frame == 1 )
        {
            ImGui::PushID( "PrismaticSpringSettings" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "PrismaticSpringEnabled" );
            ImGui::PopID();
        }
        else if( frame == 2 )
        {
            ImGui::PushID( "PrismaticLimitSettings" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "PrismaticLimitEnabled" );
            ImGui::PopID();
        }

        view->drawControls();

        debugCamera camera{};
        camera.center = getDemoEntry( demoKind::prismaticRail ).cameraCenter;
        camera.pixelsPerMeter = 110.0f;
        ImDrawList* list = ImGui::GetWindowDrawList();
        const int before = list->VtxBuffer.Size;
        debugDraw draw{ list, camera, { 0.0f, 0.0f }, { 800.0f, 600.0f } };
        view->draw( draw );
        const int vertices = list->VtxBuffer.Size - before;

        if( frame == 0 ) baselineVertices = vertices;
        if( frame == 1 ) springVertices = vertices;
        if( frame == 2 ) limitVertices = vertices;

        ImGui::End();
        ImGui::Render();
    }

    if( baselineVertices <= 0 ) fail( "Prismatic joint view drew no base overlay" );
    if( springVertices <= baselineVertices ) fail( "Prismatic spring inspector toggle drew no target overlay" );
    if( limitVertices <= springVertices ) fail( "Prismatic limit inspector toggle drew no boundary overlay" );

    // Weld inspector의 두 softness 편집 트리와 preset 경로를 headless frame에서 함께 실행함.
    auto weldView = createJointDemoView( demoKind::weldPair );
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
    ImGui::SetNextWindowSize( { 420.0f, 1800.0f } );
    ImGui::Begin( "Weld inspector" );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WeldJointInspector" ), 1 );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WeldLinearSettings" ), 1 );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WeldAngularSettings" ), 1 );
    GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "WeldPresetSoftBoth" );
    weldView->drawControls();

    debugCamera weldCamera{};
    weldCamera.center = getDemoEntry( demoKind::weldPair ).cameraCenter;
    weldCamera.pixelsPerMeter = getDemoEntry( demoKind::weldPair ).pixelsPerMeter;
    ImDrawList* weldList = ImGui::GetWindowDrawList();
    const int weldBefore = weldList->VtxBuffer.Size;
    debugDraw weldDraw{ weldList, weldCamera, { 0.0f, 0.0f }, { 800.0f, 600.0f } };
    weldView->draw( weldDraw );
    const int weldVertices = weldList->VtxBuffer.Size - weldBefore;

    ImGui::End();
    ImGui::Render();

    if( weldVertices <= 0 ) fail( "Weld joint view drew no anchor or frame overlay" );

    // Ragdoll은 대표 관절 하나가 아니라 모든 Revolute limit을 관찰할 수 있어야 함.
    rigidBodyDemoUi ragdollView{ demoKind::ragdoll };
    const auto drawRagdoll = [&ragdollView]()
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 800.0f, 800.0f } );
        ImGui::Begin( "Ragdoll joint limits" );

        debugCamera camera{};
        camera.center = getDemoEntry( demoKind::ragdoll ).cameraCenter;
        camera.pixelsPerMeter = getDemoEntry( demoKind::ragdoll ).pixelsPerMeter;
        ImDrawList* list = ImGui::GetWindowDrawList();
        const int before = list->VtxBuffer.Size;
        debugDraw draw{ list, camera, { 0.0f, 0.0f }, { 800.0f, 700.0f } };
        ragdollView.draw( draw );
        drawRagdollJointLimits( draw, ragdollView );
        const int vertices = list->VtxBuffer.Size - before;

        ImGui::End();
        ImGui::Render();
        return vertices;
    };

    const int allRagdollLimitVertices = drawRagdoll();
    if( allRagdollLimitVertices <= 0 ) fail( "Ragdoll view drew no geometry" );

    for( const jointId id : ragdollView.getRagdollJoints() )
    {
        const revoluteJointData data = ragdollView.getWorld().getRevoluteJointData( id );
        ragdollView.getWorld().setRevoluteJointLimit( id, false, data.lowerAngle, data.upperAngle );
        const int withoutOneLimit = drawRagdoll();
        if( withoutOneLimit >= allRagdollLimitVertices ) fail( "Ragdoll view did not draw every joint limit" );
        ragdollView.getWorld().setRevoluteJointLimit( id, true, data.lowerAngle, data.upperAngle );
    }

    for( int action = 0; action < 3; ++action )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 420.0f, 4000.0f } );
        ImGui::Begin( "Ragdoll controls" );
        const char* control = action == 0 ? "RagdollStrongFriction" : action == 1 ? "RagdollSelfCollision" : "RagdollReset";
        GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( control );
        ragdollView.drawControls();
        ImGui::End();
        ImGui::Render();
    }
    if( ragdollView.getRagdollJointFriction() != 0.3f || ragdollView.getRagdollSelfCollision() ) fail( "Ragdoll controls must apply friction and preserve collision selection on reset" );
    auto carView = createJointDemoView( demoKind::motorCar );
    if( !carView->getCameraTarget() ) fail( "Car view must expose its tracking target to the host" );
    demoInput drive{};
    drive.right = true;
    carView->handleInput( drive );
    for( int frame = 0; frame < 120; ++frame ) carView->step( 1.0f / 60.0f, 4 );
    if( carView->getCameraTarget()->x < 2.0f ) fail( "Car tracking target must move with the simulated chassis" );
    for( const demoKind kind : { demoKind::terrainChain, demoKind::splineTerrain } )
    {
        rigidBodyDemoUi terrainView{ kind };
        if( kind == demoKind::splineTerrain ) terrainView.getWorld().SetBodyTransform( terrainView.getShapes()[0].bodyHandle, { { 1.5f, 0.75f }, rot2::FromRadians( 0.25f ) } );
        for( const char* control : { "TerrainSeams", "TerrainFront", "TerrainBack", "TerrainReset" } )
        {
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
            ImGui::SetNextWindowSize( { 420.0f, 4000.0f } );
            ImGui::Begin( "Terrain controls" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( control );
            terrainView.drawControls();
            const vec2 velocity = terrainView.getWorld().GetBodyLinearVelocity( terrainView.getImpulseBody() );
            if( control == std::string_view{ "TerrainReset" } )
            {
                if( LengthSquared( velocity ) != 0.0f ) fail( "Terrain reset button must stop the probe" );
            }
            else if( LengthSquared( velocity ) < 30.0f ) fail( "Terrain trial button must launch the probe" );
            debugCamera camera{};
            ImDrawList* list = ImGui::GetWindowDrawList();
            const int before = list->VtxBuffer.Size;
            debugDraw draw{ list, camera, { 0.0f, 0.0f }, { 800.0f, 600.0f } };
            if( kind == demoKind::splineTerrain )
            {
                const auto pose = terrainView.getWorld().GetBodyTransform( terrainView.getShapes()[0].bodyHandle );
                demoInput press{};
                press.mousePosition = TransformPoint( pose, terrainView.getSplineControlPoints()[4] );
                press.mousePressed = press.mouseHeld = true;
                terrainView.handleInput( press );
            }
            terrainView.draw( draw );
            bool sawNormal = false;
            for( int i = before; i < list->VtxBuffer.Size; ++i ) sawNormal |= list->VtxBuffer[i].col == IM_COL32( 100, 255, 140, 255 );
            if( !sawNormal ) fail( "Terrain view must draw front-face arrows" );
            if( kind == demoKind::splineTerrain )
            {
                const auto pose = terrainView.getWorld().GetBodyTransform( terrainView.getShapes()[0].bodyHandle );
                for( const std::size_t index : { std::size_t{ 0 }, std::size_t{ 4 } } )
                {
                    const ImVec2 expected = camera.WorldToScreen( TransformPoint( pose, terrainView.getSplineControlPoints()[index] ), { 0, 0 }, { 800, 600 } );
                    const ImU32 color = index == 4 ? IM_COL32( 255, 170, 70, 255 ) : IM_COL32( 230, 170, 255, 255 );
                    bool found = false;
                    for( int i = before; i < list->VtxBuffer.Size; ++i )
                    {
                        const auto& vertex = list->VtxBuffer[i];
                        const float dx = vertex.pos.x - expected.x, dy = vertex.pos.y - expected.y;
                        found |= vertex.col == color && dx * dx + dy * dy < 64.0f;
                    }
                    if( !found ) fail( "Spline controls must draw transformed control handles and highlight the selected point" );
                }
            }
            ImGui::End();
            ImGui::Render();
        }
    }
    {
        rigidBodyDemoUi adaptiveView{ demoKind::splineTerrain };
        for( int frame = 0; frame < 18; ++frame )
        {
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Adaptive inspector" );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 )
            {
                auto* popup = ImGui::FindWindowByName( "##Combo_00" );
                if( !popup ) fail( "Adaptive inspector selection popup fixture" );
                GImGui->NavActivateId = GImGui->NavActivateDownId = popup->GetID( "원 [Chain Probe]" );
            }
            ImGui::LogToBuffer();
            adaptiveView.drawControls();
            const bool selectedProbe = std::string_view{ GImGui->LogBuffer.c_str() }.find( "원 [Chain Probe]" ) != std::string_view::npos;
            if( frame == 2 && !selectedProbe ) fail( "Probe selection fixture must select the circle inspector" );
            if( frame == 3 && !selectedProbe ) fail( "Inspector must retain the selected probe after adaptive topology changes" );
            if( frame == 5 && !selectedProbe ) fail( "Inspector must retain the selected probe after sampling error changes" );
            if( ( frame == 7 || frame == 9 ) && !selectedProbe ) fail( "Inspector must retain the selected probe after control insertion and deletion" );
            if( ( frame == 11 || frame == 13 ) && !selectedProbe ) fail( "Inspector must retain the selected probe after undo and redo" );
            if( ( frame == 15 || frame == 17 ) && !selectedProbe ) fail( "Inspector must retain the selected probe after terrain reset and its undo" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
            if( frame == 2 )
            {
                const std::size_t before = adaptiveView.getTerrainPointCount();
                demoInput drag{};
                drag.mousePosition = adaptiveView.getSplineControlPoints()[1];
                drag.mousePressed = drag.mouseHeld = true;
                adaptiveView.handleInput( drag );
                drag.mousePosition.y += 1.0f;
                drag.mousePressed = false;
                adaptiveView.handleInput( drag );
                if( adaptiveView.getTerrainPointCount() == before ) fail( "Adaptive inspector fixture must change topology" );
            }
            if( frame == 4 )
            {
                const std::size_t before = adaptiveView.getTerrainPointCount();
                adaptiveView.setSplineSamplingError( 0.05f );
                if( adaptiveView.getTerrainPointCount() == before ) fail( "Sampling error inspector fixture must change topology" );
            }
            if( frame == 6 && !adaptiveView.insertSplineControlPoint( 1 ) ) fail( "Inspector insertion fixture must apply" );
            if( frame == 8 && !adaptiveView.removeSplineControlPoint( 2 ) ) fail( "Inspector deletion fixture must apply" );
            if( frame == 10 && !adaptiveView.undoSplineEdit() ) fail( "Inspector undo fixture must apply" );
            if( frame == 12 && !adaptiveView.redoSplineEdit() ) fail( "Inspector redo fixture must apply" );
            if( frame == 14 && !adaptiveView.resetSplineTerrain() ) fail( "Inspector terrain reset fixture must apply" );
            if( frame == 16 && !adaptiveView.undoSplineEdit() ) fail( "Inspector terrain reset undo fixture must apply" );
        }
    }
    {
        rigidBodyDemoUi samplingView{ demoKind::splineTerrain };
        const std::size_t initialCount = samplingView.getTerrainPointCount();
        for( int frame = 0; frame < 6; ++frame )
        {
            if( frame == 1 ) io.AddInputCharactersUTF8( "5" );
            if( frame == 2 )
            {
                GImGui->InputTextState.SelectAll();
                io.AddInputCharactersUTF8( "2" );
            }
            if( frame == 3 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline sampling controls" );
            if( frame == 0 )
            {
                GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "SplineSamplingError" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame == 4 || frame == 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( frame == 4 ? "SplineUndo" : "SplineRedo" );
            // 드래그처럼 한 번의 활성화 동안 여러 값을 즉시 적용해 편집 묶음을 검증함.
            ImGui::PushItemFlag( ImGuiItemFlags_LiveEditOnInputScalar, true );
            ImGui::LogToBuffer();
            samplingView.drawControls();
            ImGui::PopItemFlag();
            const std::string_view log{ GImGui->LogBuffer.c_str() };
            if( log.find( "근사 오차" ) == std::string_view::npos || log.find( "선분 수:" ) == std::string_view::npos ) fail( "Spline controls must expose sampling error and edge count" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
            if( frame == 1 && samplingView.getSplineSamplingError() != 0.05f ) fail( "Sampling gesture must apply its first value" );
            if( frame == 2 && samplingView.getSplineSamplingError() != 0.02f ) fail( "Sampling gesture must apply its second value" );
            if( frame == 4 && samplingView.getSplineSamplingError() != 0.01f ) fail( "Undo button must restore the entire sampling gesture" );
        }
        if( samplingView.getSplineSamplingError() != 0.02f || samplingView.getTerrainPointCount() >= initialCount ) fail( "Redo button must restore the final centimeter input and terrain geometry" );
    }
    {
        rigidBodyDemoUi topologyView{ demoKind::splineTerrain };
        const std::vector<vec2> original{ topologyView.getSplineControlPoints().begin(), topologyView.getSplineControlPoints().end() };
        for( int frame = 0; frame < 6; ++frame )
        {
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline topology controls" );
            const char* action = frame == 2 ? "SplineControl" : ( frame == 0 || frame == 4 ? "SplineInsert" : "SplineRemove" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( action );
            if( frame == 3 )
            {
                auto* popup = ImGui::FindWindowByName( "##Combo_00" );
                if( !popup ) fail( "Spline control selector popup fixture" );
                GImGui->NavActivateId = GImGui->NavActivateDownId = popup->GetID( "제어점 9" );
            }
            ImGui::LogToBuffer();
            topologyView.drawControls();
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
            const std::size_t count = topologyView.getSplineControlPoints().size();
            if( frame == 0 && count != 10 ) fail( "Insert button must add a spline control" );
            if( frame == 1 && count != 9 ) fail( "Delete button must remove the newly selected inserted control" );
            if( frame == 1 )
                for( std::size_t i = 0; i < original.size(); ++i )
                    if( LengthSquared( topologyView.getSplineControlPoints()[i] - original[i] ) > 1.0e-8f ) fail( "Deletion must target the newly selected inserted control" );
            if( frame == 4 && count != 9 ) fail( "Insertion must be disabled at the selected last control" );
            if( frame == 5 && count != 8 ) fail( "Deletion must support the selected last control" );
        }
        while( topologyView.getSplineControlPoints().size() > 2 )
            if( !topologyView.removeSplineControlPoint( topologyView.getSplineControlPoints().size() - 1 ) ) fail( "Two-point UI fixture must apply" );
        for( int frame = 0; frame < 2; ++frame )
        {
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline topology minimum" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( frame == 0 ? "SplineRemove" : "SplineInsert" );
            topologyView.drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame == 0 && topologyView.getSplineControlPoints().size() != 2 ) fail( "Delete button must preserve the two-control minimum" );
        }
    }
    {
        rigidBodyDemoUi resetView{ demoKind::splineTerrain };
        if( !resetView.insertSplineControlPoint( 1 ) ) fail( "Terrain reset UI fixture must insert a control" );
        resetView.setSplineSamplingError( 0.03f );
        const char* actions[] = { "SplineTerrainReset", "SplineUndo", "SplineRedo", "SplineTerrainReset", "SplineUndo" };
        for( int frame = 0; frame < 5; ++frame )
        {
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline terrain reset" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( actions[frame] );
            resetView.drawControls();
            ImGui::End();
            ImGui::Render();
            const bool edited = frame == 1 || frame == 4;
            if( resetView.getSplineControlPoints().size() != ( edited ? 10u : 9u ) || resetView.getSplineSamplingError() != ( edited ? 0.03f : 0.01f ) ) fail( "Terrain reset button and history must restore controls and error without replacing unchanged history" );
        }
    }
    {
        rigidBodyDemoUi positionView{ demoKind::splineTerrain };
        while( positionView.getSplineControlPoints().size() > 2 )
            if( !positionView.removeSplineControlPoint( positionView.getSplineControlPoints().size() - 1 ) ) fail( "Trial slider fixture reduces to one edge" );
        const vec2 a = positionView.getTerrainPoint( 0 ), b = positionView.getTerrainPoint( 1 );
        const vec2 normal = Cross( Normalize( b - a ), 1.0f );
        for( int frame = 0; frame < 5; ++frame )
        {
            if( frame == 1 ) io.AddInputCharactersUTF8( "25" );
            if( frame == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 3 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline trial position" );
            if( frame == 0 )
            {
                GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "SplineTrialFraction" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame == 3 || frame == 4 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( frame == 3 ? "TerrainFront" : "TerrainSeams" );
            positionView.drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame == 3 )
            {
                if( positionView.getSplineTrialFraction() != 0.25f ) fail( "Trial slider must convert percentage to length fraction" );
                const vec2 expected = a + 0.25f * ( b - a ) + 1.5f * normal;
                if( LengthSquared( positionView.getWorld().GetBodyTransform( positionView.getImpulseBody() ).position - expected ) > 1.0e-8f ) fail( "Front trial button must use the selected length fraction" );
            }
            if( frame == 4 )
                if( LengthSquared( positionView.getWorld().GetBodyTransform( positionView.getImpulseBody() ).position - ( 0.5f * ( a + b ) + 1.5f * normal ) ) > 1.0e-8f ) fail( "Seam trial button must retain its original launch position" );
        }
    }
    {
        rigidBodyDemoUi preview{ demoKind::splineTerrain };
        preview.getWorld().SetBodyTransform( preview.getShapes()[0].bodyHandle, { { 1.5f, 0.75f }, rot2::FromRadians( 0.25f ) } );
        preview.getWorld().SetBodyLinearVelocity( preview.getImpulseBody(), { 2, 3 } );
        for( int frame = 0; frame < 6; ++frame )
        {
            preview.setSplineTrialFraction( frame == 0 ? 0.25f : 0.75f );
            if( frame == 1 )
            {
                const vec2 a = preview.getTerrainPoint( 0 ), b = preview.getTerrainPoint( 1 );
                demoInput pick{};
                pick.mousePosition = TransformPoint( preview.getWorld().GetBodyTransform( preview.getShapes()[0].bodyHandle ), a + 2.0f * Normalize( a - b ) );
                pick.mousePressed = pick.mouseHeld = pick.selectTrialPosition = true;
                preview.handleInput( pick );
                preview.cancelInput();
                if( preview.getSplineTrialFraction() != 0.0f ) fail( "Shift trial selection must update the preview target before drawing" );
            }
            if( frame == 2 && !preview.insertSplineControlPoint( 1 ) ) fail( "Preview edit fixture must apply" );
            if( frame == 3 && !preview.undoSplineEdit() ) fail( "Preview undo fixture must apply" );
            if( frame == 4 )
            {
                preview.setSplineSamplingError( 0.005f );
                if( !preview.resetSplineTerrain() ) fail( "Preview reset fixture must apply" );
            }
            if( frame == 5 ) preview.getWorld().DestroyChain( preview.getWorld().GetShapeParentChain( preview.getShapes()[0].shapeHandle ) );
            const auto target = preview.getTerrainTrialTarget( terrainTrial::front );
            const transform2 probePose = preview.getWorld().GetBodyTransform( preview.getImpulseBody() );
            const vec2 velocity = preview.getWorld().GetBodyLinearVelocity( preview.getImpulseBody() );
            const int contacts = preview.getWorld().GetContactCount();
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 800, 600 } );
            ImGui::Begin( "Spline trial preview" );
            debugCamera camera{};
            camera.center = target ? target->point : vec2{};
            camera.pixelsPerMeter = 60.0f;
            ImDrawList* list = ImGui::GetWindowDrawList();
            const int before = list->VtxBuffer.Size;
            debugDraw draw{ list, camera, { 0, 0 }, { 800, 600 } };
            preview.draw( draw );
            bool pointFound = false, arrowFound = false, previewFound = false;
            const ImVec2 point = target ? camera.WorldToScreen( target->point, { 0, 0 }, { 800, 600 } ) : ImVec2{};
            const ImVec2 tip = target ? camera.WorldToScreen( target->point + 0.8f * Cross( target->tangent, 1.0f ), { 0, 0 }, { 800, 600 } ) : ImVec2{};
            for( int i = before; i < list->VtxBuffer.Size; ++i )
            {
                const auto& vertex = list->VtxBuffer[i];
                if( vertex.col != IM_COL32( 80, 220, 255, 255 ) ) continue;
                previewFound = true;
                const float dx = vertex.pos.x - point.x, dy = vertex.pos.y - point.y;
                const float tx = vertex.pos.x - tip.x, ty = vertex.pos.y - tip.y;
                pointFound |= dx * dx + dy * dy < 64.0f;
                arrowFound |= tx * tx + ty * ty < 16.0f;
            }
            if( target && ( !pointFound || !arrowFound ) ) fail( "Preview must draw the selected world point and front-normal arrow after edits" );
            if( !target && previewFound ) fail( "Retired terrain must not draw a stale preview" );
            if( LengthSquared( preview.getWorld().GetBodyTransform( preview.getImpulseBody() ).position - probePose.position ) != 0.0f || LengthSquared( preview.getWorld().GetBodyLinearVelocity( preview.getImpulseBody() ) - velocity ) != 0.0f || preview.getWorld().GetContactCount() != contacts ) fail( "Preview drawing must not change probe motion or contacts" );
            ImGui::End();
            ImGui::Render();
        }
    }
    {
        rigidBodyDemoUi speedView{ demoKind::splineTerrain };
        const transform2 initialPose = speedView.getWorld().GetBodyTransform( speedView.getImpulseBody() );
        for( int frame = 0; frame < 5; ++frame )
        {
            if( frame == 1 ) io.AddInputCharactersUTF8( "12.5" );
            if( frame == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 3 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline trial speed" );
            if( frame == 0 )
            {
                GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "SplineTrialSpeed" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame == 3 || frame == 4 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( frame == 3 ? "TerrainFront" : "TerrainSeams" );
            speedView.drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame == 2 )
            {
                if( speedView.getSplineTrialSpeed() != 12.5f ) fail( "Trial speed slider must apply meters-per-second input" );
                if( LengthSquared( speedView.getWorld().GetBodyTransform( speedView.getImpulseBody() ).position - initialPose.position ) != 0.0f || LengthSquared( speedView.getWorld().GetBodyLinearVelocity( speedView.getImpulseBody() ) ) != 0.0f ) fail( "Trial speed editing must wait for launch before changing motion" );
            }
            if( frame == 3 || frame == 4 )
            {
                const auto target = speedView.getTerrainTrialTarget( frame == 3 ? terrainTrial::front : terrainTrial::seams );
                if( !target ) fail( "Trial speed UI fixture needs a target" );
                const vec2 expected = frame == 3 ? -12.5f * Cross( target->tangent, 1.0f ) : 6.0f * target->tangent;
                if( LengthSquared( speedView.getWorld().GetBodyLinearVelocity( speedView.getImpulseBody() ) - expected ) > 1.0e-8f ) fail( "Launch buttons must use selected front speed and unchanged seam speed" );
            }
        }
    }
    {
        rigidBodyDemoUi observationView{ demoKind::splineTerrain };
        for( int frame = 0; frame < 5; ++frame )
        {
            if( frame == 1 )
            {
                observationView.launchTerrainProbe( 1, terrainTrial::front );
                for( int step = 0; step < 120; ++step ) observationView.step( 1.0f / 60.0f, 4 );
                if( !observationView.getTerrainTrialObservation().firstContactTime ) fail( "Observation UI fixture must contact terrain" );
                observationView.setSplineTrialFraction( 0.75f );
                observationView.setSplineTrialSpeed( 12.5f );
            }
            if( frame == 2 ) observationView.resetTerrainProbes();
            if( frame == 3 ) observationView.launchTerrainProbe( 2, terrainTrial::back );
            if( frame == 4 ) observationView.launchTerrainProbe( 0, terrainTrial::seams );
            const auto before = observationView.getTerrainTrialObservation();
            const bodyId body = observationView.getShapes()[observationView.getTerrainPointCount()].bodyHandle;
            const transform2 pose = observationView.getWorld().GetBodyTransform( body );
            const vec2 velocity = observationView.getWorld().GetBodyLinearVelocity( body );
            const int contacts = observationView.getWorld().GetContactCount();
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline trial observation" );
            ImGui::LogToBuffer();
            observationView.drawControls();
            const std::string_view log{ GImGui->LogBuffer.c_str() };
            if( ( frame == 0 || frame == 2 ) && log.find( "최근 발사: 없음" ) == std::string_view::npos ) fail( "Observation UI must show cleared state" );
            if( frame == 1 && ( log.find( "최근 발사: 캡슐 / 앞면" ) == std::string_view::npos || log.find( "경과:" ) == std::string_view::npos || log.find( "첫 접촉 관측:" ) == std::string_view::npos || log.find( "아직 없음" ) != std::string_view::npos ) ) fail( "Observation UI must show the launched probe, elapsed time and first contact" );
            if( frame == 3 && ( log.find( "최근 발사: 상자 / 뒷면" ) == std::string_view::npos || log.find( "첫 접촉 관측: 아직 없음" ) == std::string_view::npos ) ) fail( "New launch must replace the prior displayed observation" );
            if( frame == 1 && log.find( "발사 조건: 위치 50.0 % / 초기 속도 8.0 m/s" ) == std::string_view::npos ) fail( "UI must show recorded launch conditions instead of changed next-launch settings" );
            if( frame == 3 && log.find( "발사 조건: 위치 75.0 % / 초기 속도 12.5 m/s" ) == std::string_view::npos ) fail( "UI must show the new launch's applied conditions" );
            if( frame == 4 && log.find( "발사 조건: 첫 선분 중앙 / 초기 속도 6.0 m/s" ) == std::string_view::npos ) fail( "UI must show fixed seam conditions" );
            if( ( frame == 0 || frame == 2 ) && log.find( "발사 조건:" ) != std::string_view::npos ) fail( "Cleared observations must not show old launch conditions" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
            const auto after = observationView.getTerrainTrialObservation();
            if( after.launchSpeed != before.launchSpeed || after.launchFraction != before.launchFraction || after.elapsed != before.elapsed || after.firstContactTime != before.firstContactTime || after.touching != before.touching || LengthSquared( observationView.getWorld().GetBodyTransform( body ).position - pose.position ) != 0.0f || LengthSquared( observationView.getWorld().GetBodyLinearVelocity( body ) - velocity ) != 0.0f || observationView.getWorld().GetContactCount() != contacts ) fail( "Observation UI must not change recorded conditions, time, motion or contacts" );
        }
    }
    {
        rigidBodyDemoUi referenceView{ demoKind::splineTerrain };
        for( int frame = 0; frame < 7; ++frame )
        {
            if( frame == 1 )
            {
                referenceView.launchTerrainProbe( 1, terrainTrial::front );
                for( int step = 0; step < 120; ++step ) referenceView.step( 1.0f / 60.0f, 4 );
            }
            if( frame == 2 )
            {
                referenceView.setSplineTrialFraction( 0.75f );
                referenceView.setSplineTrialSpeed( 12.5f );
                referenceView.launchTerrainProbe( 2, terrainTrial::back );
                referenceView.step( 1.0f / 60.0f, 4 );
            }
            if( frame == 5 ) referenceView.resetTerrainProbes();
            if( frame == 6 && !referenceView.insertSplineControlPoint( 1 ) ) fail( "Reference UI geometry fixture must apply" );
            const auto before = referenceView.getTerrainTrialObservation();
            const transform2 pose = referenceView.getWorld().GetBodyTransform( referenceView.getImpulseBody() );
            const vec2 velocity = referenceView.getWorld().GetBodyLinearVelocity( referenceView.getImpulseBody() );
            const int contacts = referenceView.getWorld().GetContactCount();
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline trial reference" );
            if( frame == 0 || frame == 1 || frame == 4 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "SplineTrialPin" );
            if( frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "SplineTrialReferenceClear" );
            ImGui::LogToBuffer();
            referenceView.drawControls();
            const std::string_view log{ GImGui->LogBuffer.c_str() };
            if( frame == 0 || frame == 3 || frame == 6 )
                if( referenceView.getTerrainTrialReference() || log.find( "비교 기준: 없음" ) == std::string_view::npos ) fail( "Reference UI must show unavailable or cleared reference" );
            if( frame == 1 || frame == 2 )
                if( !referenceView.getTerrainTrialReference() || log.find( "비교 기준: 캡슐 / 앞면" ) == std::string_view::npos || log.find( "발사 조건: 위치 50.0 % / 초기 속도 8.0 m/s" ) == std::string_view::npos ) fail( "Pin button and UI must preserve the original reference conditions" );
            if( frame == 2 && log.find( "최근 발사: 상자 / 뒷면" ) == std::string_view::npos ) fail( "UI must show current and reference trials together" );
            if( frame == 4 || frame == 5 )
                if( !referenceView.getTerrainTrialReference() || log.find( "비교 기준: 상자 / 뒷면" ) == std::string_view::npos ) fail( "Reference replacement survives probe reset" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
            const auto after = referenceView.getTerrainTrialObservation();
            if( after.elapsed != before.elapsed || after.firstContactTime != before.firstContactTime || after.touching != before.touching || LengthSquared( referenceView.getWorld().GetBodyTransform( referenceView.getImpulseBody() ).position - pose.position ) != 0.0f || LengthSquared( referenceView.getWorld().GetBodyLinearVelocity( referenceView.getImpulseBody() ) - velocity ) != 0.0f || referenceView.getWorld().GetContactCount() != contacts ) fail( "Reference controls must not change current observation or physics" );
        }
    }
    {
        rigidBodyDemoUi relaunchView{ demoKind::splineTerrain };
        for( int frame = 0; frame < 3; ++frame )
        {
            if( frame == 1 )
            {
                relaunchView.setSplineTrialFraction( 0.25f );
                relaunchView.setSplineTrialSpeed( 12.5f );
                relaunchView.launchTerrainProbe( 1, terrainTrial::back );
                relaunchView.step( 1.0f / 60.0f, 4 );
                if( !relaunchView.pinTerrainTrialReference() ) fail( "Relaunch UI fixture must pin a trial" );
                relaunchView.setSplineTrialFraction( 0.75f );
                relaunchView.setSplineTrialSpeed( 3.0f );
                relaunchView.resetTerrainProbes();
            }
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline reference relaunch" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( frame == 2 ? "TerrainFront" : "SplineTrialRelaunch" );
            ImGui::LogToBuffer();
            relaunchView.drawControls();
            const auto current = relaunchView.getTerrainTrialObservation();
            const std::string_view log{ GImGui->LogBuffer.c_str() };
            if( frame == 0 && current.active ) fail( "Relaunch button without a reference must not launch" );
            if( frame == 1 && ( !current.active || current.probeIndex != 1 || current.trial != terrainTrial::back || current.launchFraction != 0.25f || current.launchSpeed != 12.5f || current.elapsed != 0.0f ) ) fail( "Relaunch button must apply the pinned launch conditions" );
            if( frame == 1 && ( log.find( "최근 발사: 캡슐 / 뒷면" ) == std::string_view::npos || !relaunchView.getTerrainTrialReference() ) ) fail( "Relaunch UI must show the fresh trial immediately and retain reference" );
            if( frame == 2 && ( current.probeIndex != 1 || current.trial != terrainTrial::front || current.launchFraction != 0.25f || current.launchSpeed != 12.5f ) ) fail( "Relaunch must synchronize the probe selector and front/back settings" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
        }
    }
    {
        rigidBodyDemoUi comparisonView{ demoKind::splineTerrain };
        comparisonView.getWorld().SetGravity( {} );
        const auto probe = comparisonView.getShapes()[comparisonView.getTerrainPointCount() - 1];
        const auto target = *comparisonView.getTerrainTrialTarget( terrainTrial::front );
        const char* expected[] = { "비교: 기준 없음", "첫 접촉 관측 차이 (현재 - 기준): +0.000 s", "비교: 현재 관측 없음", "비교: 발사 조건 다름", "비교: 발사 조건 같음 / 접촉 관측 부족", "첫 접촉 관측 차이 (현재 - 기준): +0.125 s", "첫 접촉 관측 차이 (현재 - 기준): +0.125 s", "비교: 기준 없음", "첫 접촉 관측 차이 (현재 - 기준): +0.125 s", "첫 접촉 관측 차이 (현재 - 기준): -0.125 s", "첫 접촉 관측 차이 (현재 - 기준): +0.000 s" };
        for( int frame = 0; frame < 11; ++frame )
        {
            if( frame == 1 ) comparisonView.launchTerrainProbe( 0, terrainTrial::front );
            if( frame == 1 || frame == 5 )
            {
                comparisonView.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
                comparisonView.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                comparisonView.step( frame == 1 ? 0.125f : 0.25f, 4 );
            }
            if( frame == 1 && !comparisonView.pinTerrainTrialReference() ) fail( "Comparison UI fixture must pin first contact" );
            if( frame == 2 ) comparisonView.resetTerrainProbes();
            if( frame == 3 ) comparisonView.launchTerrainProbe( 1, terrainTrial::front );
            if( frame == 4 && !comparisonView.relaunchTerrainTrialReference() ) fail( "Comparison UI fixture must relaunch" );
            if( frame == 6 )
            {
                comparisonView.step( 0.0625f, 8 );
                comparisonView.setSplineTrialFraction( 0.75f );
                comparisonView.setSplineTrialSpeed( 12.5f );
            }
            if( frame == 7 ) comparisonView.clearTerrainTrialReference();
            if( frame >= 8 )
            {
                if( ( frame != 10 && !comparisonView.pinTerrainTrialReference() ) || !comparisonView.relaunchTerrainTrialReference() ) fail( "Subdivision comparison UI fixture must pin and relaunch" );
                if( frame != 9 )
                {
                    comparisonView.getWorld().SetBodyTransform( probe.bodyHandle, { { 0.0f, 10.0f }, {} } );
                    comparisonView.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                    comparisonView.step( 0.125f, 4 );
                }
                comparisonView.getWorld().SetBodyTransform( probe.bodyHandle, { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
                comparisonView.getWorld().SetBodyLinearVelocity( probe.bodyHandle, {} );
                comparisonView.step( 0.25f, 8 );
            }
            const auto before = comparisonView.getTerrainTrialObservation();
            const auto reference = comparisonView.getTerrainTrialReference();
            const auto pose = comparisonView.getWorld().GetBodyTransform( probe.bodyHandle );
            const vec2 velocity = comparisonView.getWorld().GetBodyLinearVelocity( probe.bodyHandle );
            const int contacts = comparisonView.getWorld().GetContactCount();
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline trial comparison" );
            ImGui::LogToBuffer();
            comparisonView.drawControls();
            if( std::string_view{ GImGui->LogBuffer.c_str() }.find( expected[frame] ) == std::string_view::npos ) fail( "Comparison UI must distinguish availability and show signed current-minus-reference time" );
            const std::string_view log{ GImGui->LogBuffer.c_str() };
            const auto referenceSettings = log.find( "관측 단계: 125.000 ms / 내부 분할 4" );
            if( frame >= 1 && frame != 10 && log.find( "첫 접촉까지 진행 설정 변경: 없음" ) == std::string_view::npos ) fail( "Step history UI must show unchanged recorded history" );
            if( ( frame == 8 || frame == 9 ) && log.find( "첫 접촉까지 진행 설정 변경: 있음" ) == std::string_view::npos ) fail( "Step history UI must show current and pinned setting changes" );
            if( frame == 0 && log.find( "첫 접촉까지 진행 설정 변경:" ) != std::string_view::npos ) fail( "Step history UI must wait for first contact" );
            if( frame == 1 && log.find( "첫 접촉 관측 단계 설정: 같음" ) == std::string_view::npos ) fail( "Step comparison UI must show matching recorded settings" );
            if( ( frame == 5 || frame == 6 || frame == 8 ) && log.find( "첫 접촉 관측 단계 설정: 다름" ) == std::string_view::npos ) fail( "Step comparison UI must show different recorded settings while retaining time difference" );
            if( frame != 1 && frame != 5 && frame != 6 && frame < 8 && log.find( "첫 접촉 관측 단계 설정:" ) != std::string_view::npos ) fail( "Step comparison UI must not judge unavailable or mismatched launch observations" );
            if( frame == 1 && log.find( "첫 접촉까지 진행 설정 비교: 일정·같음" ) == std::string_view::npos ) fail( "Step summary UI must show constant matching settings" );
            if( ( frame == 5 || frame == 6 ) && log.find( "첫 접촉까지 진행 설정 비교: 일정·다름" ) == std::string_view::npos ) fail( "Step summary UI must show constant different settings" );
            if( frame >= 8 && log.find( "첫 접촉까지 진행 설정 비교: 변경 포함" ) == std::string_view::npos ) fail( "Step summary UI must prioritize current, reference or both changed histories" );
            if( frame != 1 && frame != 5 && frame != 6 && frame < 8 && log.find( "첫 접촉까지 진행 설정 비교:" ) != std::string_view::npos ) fail( "Step summary UI must wait for comparable observations" );
            if( frame >= 1 && frame <= 6 && referenceSettings == std::string_view::npos ) fail( "Contact UI must display recorded reference step settings" );
            if( frame == 1 && log.find( "관측 단계: 125.000 ms / 내부 분할 4", referenceSettings + 1 ) == std::string_view::npos ) fail( "Contact UI must display current and pinned settings independently" );
            if( ( frame == 5 || frame == 6 || frame == 7 ) && log.find( "관측 단계: 250.000 ms / 내부 분할 4" ) == std::string_view::npos ) fail( "Contact UI must retain the current contact step settings after later steps" );
            if( frame == 0 && log.find( "관측 단계:" ) != std::string_view::npos ) fail( "Contact UI must not invent settings before contact" );
            if( ( frame == 2 || frame == 3 || frame == 4 ) && log.find( "관측 단계: 125.000 ms / 내부 분할 4", referenceSettings + 1 ) != std::string_view::npos ) fail( "Contact UI must not show stale current settings after reset or relaunch" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
            const auto after = comparisonView.getTerrainTrialObservation();
            const auto afterReference = comparisonView.getTerrainTrialReference();
            if( after.preContactSettingsChanged != before.preContactSettingsChanged || ( reference && afterReference && reference->preContactSettingsChanged != afterReference->preContactSettingsChanged ) ) fail( "Step history display must preserve current and pinned change flags" );
            if( after.elapsed != before.elapsed || after.firstContactTime != before.firstContactTime || after.firstContactTimeStep != before.firstContactTimeStep || after.firstContactSubStepCount != before.firstContactSubStepCount || reference.has_value() != afterReference.has_value() || ( reference && ( reference->firstContactTime != afterReference->firstContactTime || reference->firstContactTimeStep != afterReference->firstContactTimeStep || reference->firstContactSubStepCount != afterReference->firstContactSubStepCount ) ) || LengthSquared( comparisonView.getWorld().GetBodyTransform( probe.bodyHandle ).position - pose.position ) != 0.0f || LengthSquared( comparisonView.getWorld().GetBodyLinearVelocity( probe.bodyHandle ) - velocity ) != 0.0f || comparisonView.getWorld().GetContactCount() != contacts ) fail( "Comparison display must preserve observations and physics" );
        }
    }
    {
        rigidBodyDemoUi gravityView{ demoKind::splineTerrain };
        gravityView.getWorld().SetGravity( { 0.0f, -3.0f } );
        for( int frame = 0; frame < 10; ++frame )
        {
            if( frame == 1 )
            {
                gravityView.launchTerrainProbe( 0, terrainTrial::front );
                gravityView.step( 0.0625f, 4 );
                if( !gravityView.pinTerrainTrialReference() ) fail( "Gravity UI fixture must pin a launch" );
                gravityView.getWorld().SetGravity( { 1.0f, -4.0f } );
            }
            if( frame == 2 && !gravityView.relaunchTerrainTrialReference() ) fail( "Gravity UI fixture must relaunch" );
            if( frame == 3 ) gravityView.resetTerrainProbes();
            if( frame == 4 && !gravityView.insertSplineControlPoint( 1 ) ) fail( "Gravity UI fixture must edit terrain" );
            if( frame == 5 || frame == 6 )
            {
                gravityView.launchTerrainProbe( 0, terrainTrial::front );
                if( frame == 6 )
                {
                    gravityView.getWorld().SetGravity( {} );
                    gravityView.step( 0.0625f, 4 );
                    gravityView.getWorld().SetGravity( { 1.0f, -4.0f } );
                }
                const auto target = *gravityView.getTerrainTrialTarget( terrainTrial::front );
                gravityView.getWorld().SetBodyTransform( gravityView.getImpulseBody(), { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
                gravityView.getWorld().SetBodyLinearVelocity( gravityView.getImpulseBody(), {} );
                gravityView.step( 0.125f, 4 );
                if( frame == 5 && !gravityView.pinTerrainTrialReference() ) fail( "Gravity history UI fixture must pin first contact" );
            }
            if( frame == 7 )
            {
                if( !gravityView.pinTerrainTrialReference() ) fail( "Gravity history UI fixture must pin changed history" );
                gravityView.resetTerrainProbes();
            }
            if( frame == 8 || frame == 9 )
            {
                if( !gravityView.relaunchTerrainTrialReference() ) fail( "Gravity summary UI fixture must relaunch reference" );
                if( frame == 9 )
                {
                    gravityView.getWorld().SetGravity( {} );
                    gravityView.step( 0.0625f, 4 );
                    gravityView.getWorld().SetGravity( { 1.0f, -4.0f } );
                }
                const auto target = *gravityView.getTerrainTrialTarget( terrainTrial::front );
                gravityView.getWorld().SetBodyTransform( gravityView.getImpulseBody(), { target.point + 0.4f * Cross( target.tangent, 1.0f ), {} } );
                gravityView.getWorld().SetBodyLinearVelocity( gravityView.getImpulseBody(), {} );
                gravityView.step( 0.125f, 4 );
            }
            const auto before = gravityView.getTerrainTrialObservation();
            const auto reference = gravityView.getTerrainTrialReference();
            const auto body = gravityView.getImpulseBody();
            const auto pose = gravityView.getWorld().GetBodyTransform( body );
            const vec2 velocity = gravityView.getWorld().GetBodyLinearVelocity( body );
            const vec2 gravity = gravityView.getWorld().GetGravity();
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline launch gravity" );
            ImGui::LogToBuffer();
            gravityView.drawControls();
            const std::string_view log{ GImGui->LogBuffer.c_str() };
            const auto recorded = log.find( "발사 중력: (0.000, -3.000)" );
            if( frame >= 1 && frame <= 3 && recorded == std::string_view::npos ) fail( "Gravity UI must display recorded reference gravity" );
            if( frame == 1 && log.find( "발사 중력: (0.000, -3.000)", recorded + 1 ) == std::string_view::npos ) fail( "Gravity UI must display recorded current gravity after world settings change" );
            if( frame == 2 && ( log.find( "발사 중력: (1.000, -4.000)" ) == std::string_view::npos || log.find( "비교: 발사 조건 다름" ) == std::string_view::npos ) ) fail( "Gravity UI must distinguish new launch gravity from the pinned reference" );
            if( ( frame == 0 || frame == 4 ) && log.find( "발사 중력:" ) != std::string_view::npos ) fail( "Cleared observations must not display stale gravity" );
            if( ( frame == 5 || frame == 6 ) && log.find( "첫 접촉까지 관측된 중력 변경: 없음" ) == std::string_view::npos ) fail( "Gravity history UI must show unchanged current or reference history" );
            if( ( frame == 6 || frame == 7 ) && log.find( "첫 접촉까지 관측된 중력 변경: 있음" ) == std::string_view::npos ) fail( "Gravity history UI must show changed current and pinned history" );
            if( ( frame == 0 || frame == 4 ) && log.find( "첫 접촉까지 관측된 중력 변경:" ) != std::string_view::npos ) fail( "Gravity history UI must not invent a result before contact" );
            if( frame == 5 && log.find( "비교 중력 변경 관측: 없음" ) == std::string_view::npos ) fail( "Gravity summary UI must show no observed changes for either trial" );
            if( ( frame == 6 || frame == 8 || frame == 9 ) && log.find( "비교 중력 변경 관측: 포함" ) == std::string_view::npos ) fail( "Gravity summary UI must include current, reference or both changed histories" );
            if( frame != 5 && frame != 6 && frame != 8 && frame != 9 && log.find( "비교 중력 변경 관측:" ) != std::string_view::npos ) fail( "Gravity summary UI must wait for comparable observations" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
            const auto after = gravityView.getTerrainTrialObservation();
            const auto afterReference = gravityView.getTerrainTrialReference();
            if( after.preContactGravityChanged != before.preContactGravityChanged || ( reference && afterReference && afterReference->preContactGravityChanged != reference->preContactGravityChanged ) ) fail( "Gravity history display must preserve current and pinned flags" );
            if( LengthSquared( after.launchGravity - before.launchGravity ) != 0.0f || after.elapsed != before.elapsed || reference.has_value() != afterReference.has_value() || ( reference && LengthSquared( afterReference->launchGravity - reference->launchGravity ) != 0.0f ) || LengthSquared( gravityView.getWorld().GetGravity() - gravity ) != 0.0f || LengthSquared( gravityView.getWorld().GetBodyTransform( body ).position - pose.position ) != 0.0f || LengthSquared( gravityView.getWorld().GetBodyLinearVelocity( body ) - velocity ) != 0.0f ) fail( "Gravity display must preserve recorded conditions and physics" );
        }
    }
    ImGui::DestroyContext();
    return EXIT_SUCCESS;
}
