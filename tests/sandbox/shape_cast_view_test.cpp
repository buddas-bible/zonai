#include <cstdio>
#include <cstdlib>

#include <imgui.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"
#include "shapeCastDemoView.h"
#include "shapeCastDemo.h"

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
    auto& model = static_cast<shapeCastDemo&>( *scene );
    int overlayVertices = 0;
    int hiddenVertices = 0;
    int rotatingVertices = 0;
    int denseVertices = 0;
    int tiltedLinearVertices = 0;

    for( int frame = 0; frame < 6; ++frame )
    {
        if( frame == 1 )
        {
            model.setRayVisible( false );
            model.setSweepAABBVisible( false );
        }
        if( frame == 2 )
        {
            model.setRayVisible( true );
            model.setSweepAABBVisible( true );

            zonai::sandbox::demoInput click{};
            click.mousePressed = true;
            click.mousePosition = { -6.0f, 0.0f };
            click.selectTrialPosition = true;
            scene->handleInput( click );
        }

        if( frame == 3 )
        {
            model.setProbe( shapeCastProbe::box );
            model.setStart( { -1.0f, 1.25f } );
            model.setTarget( { -1.0f, 1.25f } );
            model.setStartRotationRadians( 0.7853981633974483f );
            model.setRotationRadians( 1.5707963267948966f );
            model.step( 0.25f, 1 );
        }

        if( frame == 4 )
            model.setDenseTargets( true );
        if( frame == 5 )
        {
            model.setProbe( shapeCastProbe::circle );
            model.setRotationRadians( 0.0f );
            model.setStart( { -5.0f, 0.0f } );
            model.setTarget( { 6.0f, 0.0f } );
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

        const int geometryVertices = list->VtxBuffer.Size - before;
        if( geometryVertices <= 0 )
        {
            std::fprintf( stderr, "ShapeCast scene failed to render geometry at phase %d\n", frame );
            std::exit( EXIT_FAILURE );
        }

        if( frame == 0 ) overlayVertices = geometryVertices;
        if( frame == 1 ) hiddenVertices = geometryVertices;
        if( frame == 3 ) rotatingVertices = geometryVertices;
        if( frame == 4 ) denseVertices = geometryVertices;
        if( frame == 5 ) tiltedLinearVertices = geometryVertices;

        ImGui::EndChild();
        ImGui::End();
        ImGui::Render();
    }
    if( overlayVertices <= hiddenVertices )
    {
        std::fprintf( stderr, "RayCast and swept AABB overlay toggles did not affect the rendered geometry\n" );
        std::exit( EXIT_FAILURE );
    }
    if( rotatingVertices <= 0 || model.getStartRotationRadians() == 0.0f )
    {
        std::fprintf( stderr, "Rotating ShapeCast view failed to render the TOI pose\n" );
        std::exit( EXIT_FAILURE );
    }
    if( denseVertices <= rotatingVertices || model.getDenseTargets().size() != 40 )
    {
        std::fprintf( stderr, "Dense ShapeCast view failed to render additional bodies\n" );
        std::exit( EXIT_FAILURE );
    }
    if( tiltedLinearVertices <= 0 || model.getRotationRadians() != 0.0f ||
        model.getSweepStats().toiCalls != 0 )
    {
        std::fprintf( stderr, "Initial-angle-only linear ShapeCast view failed to render\n" );
        std::exit( EXIT_FAILURE );
    }
    ImGui::DestroyContext();
}
