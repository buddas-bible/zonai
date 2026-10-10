#include <cstdio>
#include <cstdlib>

#include <imgui.h>
#include <imgui_internal.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"
#include "jointDemoView.h"
#include "rigidBodyDemoUi.h"

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

        if( frame == 1 )
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

    ImGui::DestroyContext();
    return EXIT_SUCCESS;
}