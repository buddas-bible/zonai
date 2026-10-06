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
    // 충분히 큰 viewport로 Inspector 아래의 tuning widget도 clipping 없이 실행함.
    io.DisplaySize = { 1280.0f, 4096.0f };
    const char* labels[] = { "Mouse Hertz", "Mouse damping", "Mouse max force", "Distance Hertz", "Distance damping", "Distance min", "Distance max" };
    for( int setting = 0; setting < 7; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto view = createDemoView( setting < 3 ? demoKind::playground : demoKind::distancePendulum );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) { io.AddInputCharactersUTF8( upper ? "9000" : "-1" ); }
                if( phase == 2 ) { io.AddKeyEvent( ImGuiKey_Enter, true ); }
                ImGui::NewFrame();
                ImGui::SetNextWindowPos( { 0.0f, 0.0f } ); ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Joint tuning input" );
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
            auto& model = static_cast<rigidBodyDemo&>( *view );
            const auto spring = setting < 3 ? zonai::distanceJointData{} : model.getWorld().getDistanceJointData( model.getPendulumJoint() );
            const float value = setting == 0 ? tuning.hertz : setting == 1 ? tuning.dampingRatio : setting == 2 ? tuning.maxForce : setting == 3 ? spring.hertz : setting == 4 ? spring.dampingRatio : setting == 5 ? spring.minLength : spring.maxLength;
            const float limit = setting == 0 || setting == 3 ? 30.0f : setting == 2 ? 5000.0f : setting == 5 ? 2.5f : setting == 6 ? 4.0f : 2.0f;
            const float minimum = setting == 5 ? zonai::LINEAR_SLOP : setting == 6 ? 1.5f : 0.0f;
            if( value != ( upper ? limit : minimum ) || ( setting >= 5 && spring.minLength > spring.maxLength ) )
            { std::fprintf( stderr, "manual tuning %s not clamped: %f\n", labels[setting], value ); std::exit( EXIT_FAILURE ); }
        }
    }
    auto view = createDemoView( demoKind::distancePendulum );
    auto& model = static_cast<rigidBodyDemo&>( *view );
    for( int frame = 0; frame < 4; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } ); ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Spring actions" );
        // Checkbox/Button의 keyboard press는 ActivateId와 DownId가 함께 전달됨.
        if( frame == 0 || frame == 2 ) { GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Distance spring" ); }
        if( frame == 1 ) { GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Radial kick" ); }
        view->drawControls(); ImGui::End(); ImGui::Render();
        const auto data = model.getWorld().getDistanceJointData( model.getPendulumJoint() );
        if( data.enableSpring != ( frame < 2 ) ) { std::fprintf( stderr, "spring checkbox did not switch mode\n" ); std::exit( EXIT_FAILURE ); }
        if( frame == 1 && model.getWorld().GetBodyLinearVelocity( model.getPendulumBody() ).y != -2.0f )
        { std::fprintf( stderr, "radial kick did not excite the distance axis\n" ); std::exit( EXIT_FAILURE ); }
    }
    for( int frame = 0; frame < 4; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } ); ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Limit actions" );
        if( frame == 0 || frame == 2 ) { GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Distance limit" ); }
        view->drawControls(); ImGui::End(); ImGui::Render();
        const auto data = model.getWorld().getDistanceJointData( model.getPendulumJoint() );
        if( data.enableLimit != ( frame < 2 ) ) { std::fprintf( stderr, "limit checkbox did not switch mode\n" ); std::exit( EXIT_FAILURE ); }
    }
    ImGui::DestroyContext();
}
