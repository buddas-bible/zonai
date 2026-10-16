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
        for( int frame = 0; frame < 10; ++frame )
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
        }
    }
    {
        rigidBodyDemoUi samplingView{ demoKind::splineTerrain };
        const std::size_t initialCount = samplingView.getTerrainPointCount();
        for( int frame = 0; frame < 4; ++frame )
        {
            if( frame == 1 ) io.AddInputCharactersUTF8( "5" );
            if( frame == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 3 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 420, 4000 } );
            ImGui::Begin( "Spline sampling controls" );
            if( frame == 0 )
            {
                GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "SplineSamplingError" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            ImGui::LogToBuffer();
            samplingView.drawControls();
            const std::string_view log{ GImGui->LogBuffer.c_str() };
            if( log.find( "근사 오차" ) == std::string_view::npos || log.find( "선분 수:" ) == std::string_view::npos ) fail( "Spline controls must expose sampling error and edge count" );
            ImGui::LogFinish();
            ImGui::End();
            ImGui::Render();
        }
        if( samplingView.getSplineSamplingError() != 0.05f || samplingView.getTerrainPointCount() >= initialCount ) fail( "Sampling slider must apply centimeter input to terrain geometry" );
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
    ImGui::DestroyContext();
    return EXIT_SUCCESS;
}
