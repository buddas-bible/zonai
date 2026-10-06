#include <cstdio>
#include <cstdlib>

#include <imgui.h>
#include <imgui_internal.h>
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
        auto& model = static_cast<rigidBodyDemo&>( session.getDemo() );
        demoInput input{}; input.mousePressed = true; input.mouseHeld = true;
        const auto body = model.getKind() == demoKind::playground ? model.getImpulseBody() : model.getPendulumBody();
        input.mousePosition = model.getWorld().GetBodyTransform( body ).position;
        session.handleInput( input, true );
        if( !model.getWorld().IsValid( model.getMouseJoint() ) ) { std::fprintf( stderr, "smoke drag did not start\n" ); std::exit( EXIT_FAILURE ); }
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
    // Keyboard navigation의 PreferInput은 Ctrl+click과 같은 실제 SliderFloat text 경로임.
    // 충분히 큰 viewport로 Inspector 아래의 세 tuning widget도 clipping 없이 실행함.
    io.DisplaySize = { 1280.0f, 4096.0f };
    const char* labels[] = { "Mouse Hertz", "Mouse damping", "Mouse max force" };
    for( int setting = 0; setting < 3; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto view = createDemoView( demoKind::playground );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) { io.AddInputCharactersUTF8( upper ? "9000" : "-1" ); }
                if( phase == 2 ) { io.AddKeyEvent( ImGuiKey_Enter, true ); }
                ImGui::NewFrame();
                ImGui::SetNextWindowPos( { 0.0f, 0.0f } ); ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Mouse tuning input" );
                if( phase == 0 )
                {
                    GImGui->NavActivateId = ImGui::GetID( labels[setting] );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                }
                view->drawControls();
                ImGui::End(); ImGui::Render();
                if( phase == 2 ) { io.AddKeyEvent( ImGuiKey_Enter, false ); }
            }
            const auto& tuning = static_cast<rigidBodyDemo&>( *view ).getMouseSettings();
            const float value = setting == 0 ? tuning.hertz : setting == 1 ? tuning.dampingRatio : tuning.maxForce;
            const float limit = setting == 0 ? 30.0f : setting == 1 ? 2.0f : 5000.0f;
            if( value != ( upper ? limit : 0.0f ) )
            { std::fprintf( stderr, "manual tuning %s not clamped: %f\n", labels[setting], value ); std::exit( EXIT_FAILURE ); }
        }
    }
    ImGui::DestroyContext();
}
