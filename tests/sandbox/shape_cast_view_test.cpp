#include <cstdio>
#include <cstdlib>

#include <imgui.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"
#include "shapeCastDemoView.h"

using namespace zonai::sandbox;

int main()
{
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = { 1280.0f, 900.0f };
    io.DeltaTime = 1.0f / 60.0f;

    unsigned char* pixels = nullptr;
    int width = 0;
    int height = 0;
    io.Fonts->GetTexDataAsRGBA32( &pixels, &width, &height );
    io.Fonts->SetTexID( static_cast<ImTextureID>( 1 ) );

    auto scene = createShapeCastDemoView();
    for( int frame = 0; frame < 3; ++frame )
    {
        if( frame == 1 ) scene->step( 0.5f, 4 );
        if( frame == 2 )
        {
            zonai::sandbox::demoInput click{};
            click.mousePressed = true;
            click.mousePosition = { 6.0f, 1.0f };
            scene->handleInput( click );
        }

        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 1200.0f, 820.0f } );
        ImGui::Begin( "ShapeCast visual smoke" );
        ImGui::BeginChild( "Controls", { 340.0f, 720.0f } );
        scene->drawControls();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild( "Canvas", { 820.0f, 720.0f } );

        debugCamera camera{};
        camera.center = { 0.0f, 0.0f };
        camera.pixelsPerMeter = 72.0f;
        ImDrawList* list = ImGui::GetWindowDrawList();
        const int before = list->VtxBuffer.Size;
        debugDraw draw{ list, camera, ImGui::GetCursorScreenPos(), { 820.0f, 720.0f } };
        scene->draw( draw );

        if( list->VtxBuffer.Size <= before )
        {
            std::fprintf( stderr, "ShapeCast scene failed to render geometry at phase %d\n", frame );
            std::exit( EXIT_FAILURE );
        }

        ImGui::EndChild();
        ImGui::End();
        ImGui::Render();
    }
    ImGui::DestroyContext();
}
