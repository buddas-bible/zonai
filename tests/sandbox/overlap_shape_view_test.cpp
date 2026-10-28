#include <cstdio>
#include <cstdlib>

#include <imgui.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"
#include "overlapShapeDemo.h"
#include "overlapShapeDemoView.h"

using namespace zonai;
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

    auto scene = createOverlapShapeDemoView();
    auto& model = static_cast<overlapShapeDemo&>( *scene );
    int aabbVertices = 0;
    int hiddenVertices = 0;

    for( int phase = 0; phase < 3; ++phase )
    {
        if( phase == 1 ) model.setCandidateAABBsVisible( false );
        if( phase == 2 )
        {
            model.setCandidateAABBsVisible( true );
            demoInput click{};
            click.mousePosition = { 1.2f, 1.2f };
            click.mousePressed = true;
            scene->handleInput( click );
        }

        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 1200.0f, 820.0f } );
        ImGui::Begin( "Overlap exactness smoke" );
        ImGui::BeginChild( "Controls", { 350.0f, 720.0f } );
        scene->drawControls();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild( "Canvas", { 810.0f, 720.0f } );

        debugCamera camera{};
        camera.center = { 0.0f, 0.7f };
        camera.pixelsPerMeter = 76.0f;
        ImDrawList* list = ImGui::GetWindowDrawList();
        const int previousVertices = list->VtxBuffer.Size;
        debugDraw draw{ list, camera, ImGui::GetCursorScreenPos(), { 810.0f, 720.0f } };
        scene->draw( draw );
        const int renderedVertices = list->VtxBuffer.Size - previousVertices;

        if( renderedVertices <= 0 )
        {
            std::fprintf( stderr, "Overlap comparison did not render at phase %d\n", phase );
            std::exit( EXIT_FAILURE );
        }

        if( phase == 0 ) aabbVertices = renderedVertices;
        if( phase == 1 ) hiddenVertices = renderedVertices;
        if( phase == 2 && ( !model.isExactOverlap( model.getTargets()[0].shape ) ||
            model.getExactOverlaps().size() != 1 ) )
        {
            std::fprintf( stderr, "Click must visibly change candidate-only classification to exact overlap\n" );
            std::exit( EXIT_FAILURE );
        }

        ImGui::EndChild();
        ImGui::End();
        ImGui::Render();
    }

    if( aabbVertices <= hiddenVertices )
    {
        std::fprintf( stderr, "AABB visibility toggle did not change rendered vertices\n" );
        std::exit( EXIT_FAILURE );
    }

    ImGui::DestroyContext();
}
