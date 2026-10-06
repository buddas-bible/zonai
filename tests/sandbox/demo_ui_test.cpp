#include <cstdio>
#include <cstdlib>

#include <imgui.h>
#include "debug/debugDraw.h"
#include "rigidBodyDemoUi.h"

using namespace zonai::sandbox;

// ImGui frame을 GPU/window 없이 만들어 두 실제 view의 control/draw 수명을 검사함.
// 실제 OS focus/클릭 routing이나 최종 화면의 시각 검증을 대신하지 않음.
void checkDemoUi()
{
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = { 1280.0f, 720.0f };
    io.DeltaTime = 1.0f / 60.0f;
    unsigned char* pixels = nullptr;
    int width = 0, height = 0;
    io.Fonts->GetTexDataAsRGBA32( &pixels, &width, &height );
    io.Fonts->SetTexID( static_cast<ImTextureID>( 1 ) );
    demoSession session{ createDemoView };
    for( int frame = 0; frame < 6; ++frame )
    {
        session.selectDemo( frame % 2 == 0 ? demoKind::playground : demoKind::distancePendulum );
        session.stepOnce( 4 );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 1280.0f, 720.0f } );
        ImGui::Begin( "Demo smoke" );
        ImGui::BeginChild( "Controls", { 340.0f, 650.0f } );
        session.getDemo().drawControls();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild( "Canvas", { 850.0f, 650.0f } );
        debugCamera camera{};
        camera.center = getDemoEntry( session.getKind() ).cameraCenter;
        camera.pixelsPerMeter = getDemoEntry( session.getKind() ).pixelsPerMeter;
        ImDrawList* list = ImGui::GetWindowDrawList();
        const int before = list->VtxBuffer.Size;
        debugDraw draw{ list, camera, ImGui::GetCursorScreenPos(), { 850.0f, 650.0f } };
        session.getDemo().draw( draw );
        if( list->VtxBuffer.Size <= before ) { std::fprintf( stderr, "demo canvas drew no geometry\n" ); std::exit( EXIT_FAILURE ); }
        ImGui::EndChild(); ImGui::End();
        ImGui::Render();
        session.reset();
    }
    ImGui::DestroyContext();
}
