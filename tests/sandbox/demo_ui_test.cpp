#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <numbers>

#include <imgui.h>
#include <imgui_internal.h>
#include "debug/debugDraw.h"
#include "rigidBodyDemoUi.h"

using namespace zonai::sandbox;

// ImGui frame을 GPU/window 없이 만들어 실제 view의 control/draw 수명을 검사함.
// 실제 OS focus/클릭 routing이나 최종 화면의 시각 검증을 대신하지 않음.
void checkDemoUi()
{
    ImGui::CreateContext();
    if( !initializeDemoUi() )
    {
        std::fprintf( stderr, "Korean font initialization failed\n" );
        std::exit( EXIT_FAILURE );
    }
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = { 1280.0f, 720.0f };
    io.DeltaTime = 1.0f / 60.0f;
    unsigned char* pixels = nullptr;
    int width = 0, height = 0;
    io.Fonts->GetTexDataAsRGBA32( &pixels, &width, &height );
    io.Fonts->SetTexID( static_cast<ImTextureID>( 1 ) );
    for( const ImWchar codepoint : { 0xAC00, 0xD55C, 0xD7A3 } )
    {
        if( !io.Fonts->Fonts[0]->IsGlyphInFont( codepoint ) )
        {
            std::fprintf( stderr, "Korean glyph missing\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    if( std::strcmp( ImGui::LocalizeGetMsg( ImGuiLocKey_TableReset ), "초기화" ) != 0 )
    {
        std::fprintf( stderr, "ImGui common menu not localized\n" );
        std::exit( EXIT_FAILURE );
    }
    demoSession session{ createDemoView };
    for( int frame = 0; frame < 6; ++frame )
    {
        session.selectDemo( getDemoEntries()[static_cast<std::size_t>( frame ) % getDemoEntries().size()].kind );
        session.stepOnce( 4 );
        auto& model = static_cast<rigidBodyDemo&>( session.getDemo() );
        demoInput input{};
        input.mousePressed = true;
        input.mouseHeld = true;
        const auto body = model.getKind() == demoKind::distancePendulum ? model.getPendulumBody() : model.getImpulseBody();
        input.mousePosition = model.getWorld().GetBodyTransform( body ).position;
        session.handleInput( input, true );
        if( !model.getWorld().IsValid( model.getMouseJoint() ) )
        {
            std::fprintf( stderr, "smoke drag did not start\n" );
            std::exit( EXIT_FAILURE );
        }
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
        if( list->VtxBuffer.Size <= before )
        {
            std::fprintf( stderr, "demo canvas drew no geometry\n" );
            std::exit( EXIT_FAILURE );
        }
        ImGui::EndChild();
        ImGui::End();
        ImGui::Render();
        session.reset();
    }
    // Keyboard navigation의 PreferInput은 Ctrl+click과 같은 실제 SliderFloat text 경로임.
    // 충분히 큰 viewport로 Inspector 아래의 tuning widget도 clipping 없이 실행함.
    io.DisplaySize = { 1280.0f, 4096.0f };
    const char* labels[] = { "Mouse Hertz", "Mouse damping", "Mouse max force", "Distance Hertz", "Distance damping", "Distance min", "Distance max", "Distance motor speed", "Distance motor force" };
    for( int setting = 0; setting < 9; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto view = createDemoView( setting < 3 ? demoKind::playground : demoKind::distancePendulum );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 )
                {
                    io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                }
                if( phase == 2 )
                {
                    io.AddKeyEvent( ImGuiKey_Enter, true );
                }
                ImGui::NewFrame();
                ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Joint tuning input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "MouseControls" ), 1 );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "DistanceMotorSettings" ), 1 );
                if( phase == 0 )
                {
                    GImGui->NavActivateId = ImGui::GetID( labels[setting] );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                }
                view->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 )
                {
                    io.AddKeyEvent( ImGuiKey_Enter, false );
                }
            }
            const auto& tuning = static_cast<rigidBodyDemo&>( *view ).getMouseSettings();
            auto& model = static_cast<rigidBodyDemo&>( *view );
            const auto spring = setting < 3 ? zonai::distanceJointData{} : model.getWorld().getDistanceJointData( model.getPendulumJoint() );
            const float value = setting == 0 ? tuning.hertz : setting == 1 ? tuning.dampingRatio : setting == 2 ? tuning.maxForce : setting == 3 ? spring.hertz : setting == 4 ? spring.dampingRatio : setting == 5 ? spring.minLength : setting == 6 ? spring.maxLength : setting == 7 ? spring.motorSpeed : spring.maxMotorForce;
            const float limit = setting == 0 || setting == 3 ? 30.0f : setting == 2 ? 5000.0f : setting == 5 ? 2.5f : setting == 6 ? 4.0f : setting == 7 ? 5.0f : setting == 8 ? 50.0f : 2.0f;
            const float minimum = setting == 5 ? zonai::LINEAR_SLOP : setting == 6 ? 1.5f : setting == 7 ? -5.0f : 0.0f;
            if( value != ( upper ? limit : minimum ) || ( setting >= 5 && spring.minLength > spring.maxLength ) )
            {
                std::fprintf( stderr, "manual tuning %s not clamped: %f\n", labels[setting], value );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto view = createDemoView( demoKind::distancePendulum );
    auto& model = static_cast<rigidBodyDemo&>( *view );
    for( int frame = 0; frame < 4; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Spring actions" );
        // Checkbox/Button의 keyboard press는 ActivateId와 DownId가 함께 전달됨.
        if( frame == 0 || frame == 2 )
        {
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Distance spring" );
        }
        if( frame == 1 )
        {
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Radial kick" );
        }
        view->drawControls();
        ImGui::End();
        ImGui::Render();
        const auto data = model.getWorld().getDistanceJointData( model.getPendulumJoint() );
        if( data.enableSpring != ( frame < 2 ) )
        {
            std::fprintf( stderr, "spring checkbox did not switch mode\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 1 && model.getWorld().GetBodyLinearVelocity( model.getPendulumBody() ).y != -2.0f )
        {
            std::fprintf( stderr, "radial kick did not excite the distance axis\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    for( int frame = 0; frame < 4; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Limit actions" );
        if( frame == 0 || frame == 2 )
        {
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Distance limit" );
        }
        view->drawControls();
        ImGui::End();
        ImGui::Render();
        const auto data = model.getWorld().getDistanceJointData( model.getPendulumJoint() );
        if( data.enableLimit != ( frame < 2 ) )
        {
            std::fprintf( stderr, "limit checkbox did not switch mode\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    for( int frame = 0; frame < 4; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Motor actions" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "DistanceMotorSettings" ), 1 );
        if( frame == 0 || frame == 2 )
        {
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Distance motor" );
        }
        view->drawControls();
        ImGui::End();
        ImGui::Render();
        const auto data = model.getWorld().getDistanceJointData( model.getPendulumJoint() );
        if( data.enableMotor != ( frame < 2 ) )
        {
            std::fprintf( stderr, "motor checkbox did not switch mode\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    auto inspector = createDemoView( demoKind::playground );
    auto& inspected = static_cast<rigidBodyDemo&>( *inspector );
    const auto shape = inspected.getShapes()[2].shapeHandle;
    for( int field = 0; field < 2; ++field )
    {
        const char* label = field == 0 ? "Category membership" : "Collision partners";
        const char* bitLabel = field == 0 ? "레이어 64##Category" : "레이어 64##Mask";
        for( int frame = 0; frame < 3; ++frame )
        {
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
            ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
            ImGui::Begin( "Inspector filter" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision mask" ), 1 );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "ObjectCollisionOverrides" ), 1 );
            if( frame == 0 )
            {
                GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( label );
            }
            if( frame == 1 && !GImGui->OpenPopupStack.empty() )
            {
                auto* popup = GImGui->OpenPopupStack.back().Window;
                GImGui->NavActivateId = GImGui->NavActivateDownId = popup->GetID( bitLabel );
            }
            inspector->drawControls();
            ImGui::End();
            ImGui::Render();
        }
        const auto filter = inspected.getWorld().GetShapeFilter( shape );
        const auto highBit = std::uint64_t{ 1 } << 63;
        if( field == 0 ? filter.categoryBits != ( highBit | 1 ) : filter.maskBits != ( ~highBit ) )
        {
            std::fprintf( stderr, "inspector bit toggle failed: %s\n", label );
            std::exit( EXIT_FAILURE );
        }
        ImGui::ClosePopupToLevel( 0, true );
    }
    demoSession project{ createDemoView };
    collisionSettingsUi collisionUi;
    for( int frame = 0; frame < 5; ++frame )
    {
        ImGui::NewFrame();
        ImGui::Begin( "Project controls" );
        if( frame == 0 )
        {
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Project collision matrix" );
        }
        if( frame == 1 )
        {
            auto* modal = GImGui->OpenPopupStack.back().Window;
            GImGui->NavActivateId = GImGui->NavActivateDownId = modal->GetID( "VisibleCollisionLayers" );
        }
        if( frame == 2 )
        {
            auto* popup = GImGui->OpenPopupStack.back().Window;
            GImGui->NavActivateId = GImGui->NavActivateDownId = popup->GetID( "레이어 64##VisibleLayers" );
        }
        if( frame == 3 )
        {
            for( auto* window : GImGui->Windows )
            {
                if( std::strstr( window->Name, "MatrixGrid" ) != nullptr )
                {
                    GImGui->NavActivateId = GImGui->NavActivateDownId = ImHashStr( "##Pair0_63", 0, window->GetID( "CollisionPairs" ) );
                }
            }
        }
        if( frame == 4 )
        {
            collisionUi.visibleLayers = 1;
        }
        drawProjectCollisionSettings( project, collisionUi );
        ImGui::End();
        ImGui::Render();
        for( auto* window : GImGui->Windows )
        {
            if( std::strstr( window->Name, "MatrixGrid" ) != nullptr && window->ContentSize.x > 640.0f )
            {
                std::fprintf( stderr, "collision editor expands all 64 layers\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        if( frame == 2 )
        {
            if( collisionUi.visibleLayers != ( 1 | ( std::uint64_t{ 1 } << 63 ) ) )
            {
                std::fprintf( stderr, "matrix layer selection failed\n" );
                std::exit( EXIT_FAILURE );
            }
            ImGui::ClosePopupToLevel( 1, true );
        }
    }
    if( project.getCollisionMatrix().allows( 1, std::uint64_t{ 1 } << 63 ) || project.getCollisionMatrix().allows( std::uint64_t{ 1 } << 63, 1 ) )
    {
        std::fprintf( stderr, "project matrix UI did not edit/preserve symmetric pair\n" );
        std::exit( EXIT_FAILURE );
    }
    collisionUi.visibleLayers = 0xFF;
    for( int frame = 0; frame < 3; ++frame )
    {
        ImGui::NewFrame();
        ImGui::Begin( "Project controls" );
        if( frame == 0 )
        {
            auto* modal = GImGui->OpenPopupStack.back().Window;
            GImGui->NavActivateId = GImGui->NavActivateDownId = modal->GetID( "VisibleCollisionLayers" );
        }
        if( frame == 1 )
        {
            auto* popup = GImGui->OpenPopupStack.back().Window;
            GImGui->NavActivateId = GImGui->NavActivateDownId = popup->GetID( "레이어 64##VisibleLayers" );
        }
        drawProjectCollisionSettings( project, collisionUi );
        ImGui::End();
        ImGui::Render();
    }
    bool checkedEightLayers = false;
    for( auto* window : GImGui->Windows )
    {
        if( std::strstr( window->Name, "MatrixGrid" ) == nullptr ) continue;

        const auto* table = GImGui->Tables.GetByKey( window->GetID( "CollisionPairs" ) );
        if( table == nullptr || table->ColumnsCount != 9 ) continue;

        checkedEightLayers = true;
        for( int column = 1; column < 9; ++column )
        {
            if( table->Columns[column].WorkMinX + ImGui::GetFrameHeight() > table->Columns[column].ClipRect.Max.x )
            {
                std::fprintf( stderr, "eight-layer cell clipped: %d / %.1f / %.1f\n", column, table->Columns[column].WidthGiven, table->Columns[column].ClipRect.GetWidth() );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    if( !checkedEightLayers )
    {
        std::fprintf( stderr, "eight-layer table not drawn\n" );
        std::exit( EXIT_FAILURE );
    }
    if( collisionUi.visibleLayers != 0xFF )
    {
        std::fprintf( stderr, "collision editor exceeded eight visible layers\n" );
        std::exit( EXIT_FAILURE );
    }
    auto hingeView = createDemoView( demoKind::revoluteHinge );
    auto& hingeModel = static_cast<rigidBodyDemo&>( *hingeView );
    for( int frame = 0; frame < 2; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Hinge actions" );
        const char* action = frame == 0 ? "Spin hinge" : "Kick hinge";
        GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( action );
        hingeView->drawControls();
        ImGui::End();
        ImGui::Render();
    }
    if( hingeModel.getWorld().GetBodyAngularVelocity( hingeModel.getImpulseBody() ) < 2.9f || hingeModel.getWorld().GetBodyLinearVelocity( hingeModel.getImpulseBody() ).x < 1.9f )
    {
        std::fprintf( stderr, "hinge experiment buttons did not apply impulses\n" );
        std::exit( EXIT_FAILURE );
    }
    auto limitView = createDemoView( demoKind::revoluteHinge );
    auto& limitModel = static_cast<rigidBodyDemo&>( *limitView );
    int unrestrictedVertices = 0;
    for( int frame = 0; frame < 4; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Angular limit actions" );
        // 접혀 있을 때는 숨겨진 조작이 실행되지 않아야 함.
        if( frame > 0 ) ImGui::GetStateStorage()->SetInt( ImGui::GetID( "RevoluteLimitSettings" ), 1 );
        ImGui::PushID( "RevoluteLimitSettings" );
        if( frame < 2 || frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Revolute limit" );
        ImGui::PopID();
        limitView->drawControls();
        debugCamera camera{};
        camera.center = getDemoEntry( demoKind::revoluteHinge ).cameraCenter;
        camera.pixelsPerMeter = 130.0f;
        ImDrawList* list = ImGui::GetWindowDrawList();
        const int before = list->VtxBuffer.Size;
        debugDraw draw{ list, camera, { 0.0f, 0.0f }, { 600.0f, 600.0f } };
        limitView->draw( draw );
        const int vertices = list->VtxBuffer.Size - before;
        if( frame == 0 ) unrestrictedVertices = vertices;
        if( frame == 2 && vertices <= unrestrictedVertices )
        {
            std::fprintf( stderr, "enabled angular limit drew no boundary geometry\n" );
            std::exit( EXIT_FAILURE );
        }
        ImGui::End();
        ImGui::Render();
        const auto data = limitModel.getWorld().getRevoluteJointData( limitModel.getRevoluteJoint() );
        if( data.enableLimit != ( frame == 1 || frame == 2 ) )
        {
            std::fprintf( stderr, "folded angular controls or limit toggle failed: frame %d\n", frame );
            std::exit( EXIT_FAILURE );
        }
    }
    for( int setting = 0; setting < 2; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto tuningView = createDemoView( demoKind::revoluteHinge );
            auto& tuningModel = static_cast<rigidBodyDemo&>( *tuningView );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
                ImGui::NewFrame();
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Angular limit input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "RevoluteLimitSettings" ), 1 );
                if( phase == 0 )
                {
                    ImGui::PushID( "RevoluteLimitSettings" );
                    GImGui->NavActivateId = ImGui::GetID( setting == 0 ? "Revolute lower" : "Revolute upper" );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                    ImGui::PopID();
                }
                tuningView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            }
            const auto data = tuningModel.getWorld().getRevoluteJointData( tuningModel.getRevoluteJoint() );
            const float value = setting == 0 ? data.lowerAngle : data.upperAngle;
            const float expected = setting == 0 ? ( upper ? 0.25f : -0.99f ) : ( upper ? 0.99f : -0.25f );
            if( std::abs( value - expected * std::numbers::pi_v<float> ) > 0.00001f || data.lowerAngle > data.upperAngle )
            {
                std::fprintf( stderr, "manual angular limit input escaped valid range\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto motorView = createDemoView( demoKind::revoluteHinge );
    auto& motorModel = static_cast<rigidBodyDemo&>( *motorView );
    for( int frame = 0; frame < 6; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Revolute motor actions" );
        if( frame > 0 ) ImGui::GetStateStorage()->SetInt( ImGui::GetID( "RevoluteMotorSettings" ), 1 );
        ImGui::PushID( "RevoluteMotorSettings" );
        const char* action = frame < 2 || frame == 5 ? "Revolute motor" : frame == 2 ? "Reverse revolute motor" : frame == 3 ? "Brake revolute motor" : nullptr;
        if( action ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( action );
        ImGui::PopID();
        motorView->drawControls();
        ImGui::End();
        ImGui::Render();
        const auto data = motorModel.getWorld().getRevoluteJointData( motorModel.getRevoluteJoint() );
        const float expectedSpeed = frame < 2 ? 2.0f : frame == 2 ? -2.0f : 0.0f;
        if( data.enableMotor != ( frame > 0 && frame < 5 ) || data.motorSpeed != expectedSpeed || data.maxMotorTorque != 10.0f )
        {
            std::fprintf( stderr, "folded motor controls, toggle, reverse or brake failed: frame %d\n", frame );
            std::exit( EXIT_FAILURE );
        }
    }
    for( int setting = 0; setting < 2; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto tuningView = createDemoView( demoKind::revoluteHinge );
            auto& tuningModel = static_cast<rigidBodyDemo&>( *tuningView );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
                ImGui::NewFrame();
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Revolute motor input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "RevoluteMotorSettings" ), 1 );
                if( phase == 0 )
                {
                    ImGui::PushID( "RevoluteMotorSettings" );
                    GImGui->NavActivateId = ImGui::GetID( setting == 0 ? "Revolute motor speed" : "Revolute motor torque" );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                    ImGui::PopID();
                }
                tuningView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            }
            const auto data = tuningModel.getWorld().getRevoluteJointData( tuningModel.getRevoluteJoint() );
            const float value = setting == 0 ? data.motorSpeed : data.maxMotorTorque;
            const float expected = setting == 0 ? ( upper ? 5.0f : -5.0f ) : ( upper ? 50.0f : 0.0f );
            if( value != expected )
            {
                std::fprintf( stderr, "manual revolute motor input escaped valid range\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto wheelView = createDemoView( demoKind::wheelSuspension );
    auto& wheelModel = static_cast<rigidBodyDemo&>( *wheelView );
    for( int frame = 0; frame < 5; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Wheel suspension actions" );
        if( frame > 0 ) ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WheelSpringSettings" ), 1 );
        if( frame < 3 )
        {
            ImGui::PushID( "WheelSpringSettings" );
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Wheel spring" );
            ImGui::PopID();
        }
        else
        {
            GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( frame == 3 ? "Kick wheel" : "Spin wheel" );
        }
        wheelView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( wheelModel.getWorld().getWheelJointData( wheelModel.getWheelJoint() ).enableSpring != ( frame != 1 ) )
        {
            std::fprintf( stderr, "folded suspension controls or spring toggle failed: frame %d\n", frame );
            std::exit( EXIT_FAILURE );
        }
    }
    if( wheelModel.getWorld().GetBodyLinearVelocity( wheelModel.getImpulseBody() ).y < 1.9f || wheelModel.getWorld().GetBodyAngularVelocity( wheelModel.getImpulseBody() ) < 2.9f )
    {
        std::fprintf( stderr, "wheel experiment buttons did not apply impulses\n" );
        std::exit( EXIT_FAILURE );
    }
    for( int setting = 0; setting < 2; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto tuningView = createDemoView( demoKind::wheelSuspension );
            auto& tuningModel = static_cast<rigidBodyDemo&>( *tuningView );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
                ImGui::NewFrame();
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Wheel suspension input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WheelSpringSettings" ), 1 );
                if( phase == 0 )
                {
                    ImGui::PushID( "WheelSpringSettings" );
                    GImGui->NavActivateId = ImGui::GetID( setting == 0 ? "Wheel hertz" : "Wheel damping" );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                    ImGui::PopID();
                }
                tuningView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            }
            const auto data = tuningModel.getWorld().getWheelJointData( tuningModel.getWheelJoint() );
            const float value = setting == 0 ? data.hertz : data.dampingRatio;
            const float expected = upper ? ( setting == 0 ? 10.0f : 2.0f ) : 0.0f;
            if( value != expected )
            {
                std::fprintf( stderr, "manual suspension input escaped valid range\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto wheelLimitView = createDemoView( demoKind::wheelSuspension );
    auto& wheelLimitModel = static_cast<rigidBodyDemo&>( *wheelLimitView );
    for( int frame = 0; frame < 3; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Wheel limit actions" );
        if( frame > 0 ) ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WheelLimitSettings" ), 1 );
        ImGui::PushID( "WheelLimitSettings" );
        GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Wheel limit" );
        ImGui::PopID();
        wheelLimitView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( wheelLimitModel.getWorld().getWheelJointData( wheelLimitModel.getWheelJoint() ).enableLimit != ( frame == 1 ) )
        {
            std::fprintf( stderr, "folded wheel limit controls or toggle failed: frame %d\n", frame );
            std::exit( EXIT_FAILURE );
        }
    }
    for( int setting = 0; setting < 2; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto tuningView = createDemoView( demoKind::wheelSuspension );
            auto& tuningModel = static_cast<rigidBodyDemo&>( *tuningView );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
                ImGui::NewFrame();
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Wheel limit input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WheelLimitSettings" ), 1 );
                if( phase == 0 )
                {
                    ImGui::PushID( "WheelLimitSettings" );
                    GImGui->NavActivateId = ImGui::GetID( setting == 0 ? "Wheel lower" : "Wheel upper" );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                    ImGui::PopID();
                }
                tuningView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            }
            const auto data = tuningModel.getWorld().getWheelJointData( tuningModel.getWheelJoint() );
            const float value = setting == 0 ? data.lowerTranslation : data.upperTranslation;
            const float expected = setting == 0 ? ( upper ? 0.5f : -1.0f ) : ( upper ? 1.0f : -0.5f );
            if( value != expected || data.lowerTranslation > data.upperTranslation )
            {
                std::fprintf( stderr, "manual wheel limit input escaped valid ordered range\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto wheelMotorView = createDemoView( demoKind::wheelSuspension );
    auto& wheelMotorModel = static_cast<rigidBodyDemo&>( *wheelMotorView );
    for( int frame = 0; frame < 6; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Wheel motor actions" );
        if( frame > 0 ) ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WheelMotorSettings" ), 1 );
        ImGui::PushID( "WheelMotorSettings" );
        const char* action = frame < 2 || frame == 5 ? "Wheel motor" : frame == 2 ? "Reverse wheel motor" : frame == 3 ? "Brake wheel motor" : nullptr;
        if( action ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( action );
        ImGui::PopID();
        wheelMotorView->drawControls();
        ImGui::End();
        ImGui::Render();
        const auto data = wheelMotorModel.getWorld().getWheelJointData( wheelMotorModel.getWheelJoint() );
        const float expectedSpeed = frame < 2 ? 3.0f : frame == 2 ? -3.0f : 0.0f;
        if( data.enableMotor != ( frame > 0 && frame < 5 ) || data.motorSpeed != expectedSpeed || data.maxMotorTorque != 1.0f )
        {
            std::fprintf( stderr, "folded motor controls, toggle, reverse or brake failed: frame %d\n", frame );
            std::exit( EXIT_FAILURE );
        }
    }
    for( int setting = 0; setting < 2; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto tuningView = createDemoView( demoKind::wheelSuspension );
            auto& tuningModel = static_cast<rigidBodyDemo&>( *tuningView );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
                ImGui::NewFrame();
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Wheel motor input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WheelMotorSettings" ), 1 );
                if( phase == 0 )
                {
                    ImGui::PushID( "WheelMotorSettings" );
                    GImGui->NavActivateId = ImGui::GetID( setting == 0 ? "Wheel motor speed" : "Wheel motor torque" );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                    ImGui::PopID();
                }
                tuningView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            }
            const auto data = tuningModel.getWorld().getWheelJointData( tuningModel.getWheelJoint() );
            const float value = setting == 0 ? data.motorSpeed : data.maxMotorTorque;
            const float expected = setting == 0 ? ( upper ? 10.0f : -10.0f ) : ( upper ? 5.0f : 0.0f );
            if( value != expected )
            {
                std::fprintf( stderr, "manual wheel motor input escaped valid range\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    for( int setting = 0; setting < 2; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto tuningView = createDemoView( demoKind::motorCar );
            auto& tuningModel = static_cast<rigidBodyDemo&>( *tuningView );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
                ImGui::NewFrame();
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Car motor input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "CarMotorSettings" ), 1 );
                if( phase == 0 )
                {
                    ImGui::PushID( "CarMotorSettings" );
                    GImGui->NavActivateId = ImGui::GetID( setting == 0 ? "Car motor speed" : "Car motor torque" );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                    ImGui::PopID();
                }
                tuningView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            }
            const float value = setting == 0 ? tuningModel.getCarMotorSpeed() : tuningModel.getCarMaxMotorTorque();
            const float expected = upper ? 20.0f : 0.0f;
            if( value != expected )
            {
                std::fprintf( stderr, "manual car motor input escaped valid range\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto carView = createDemoView( demoKind::motorCar );
    auto& carModel = static_cast<rigidBodyDemo&>( *carView );
    for( int frame = 0; frame < 5; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
        ImGui::Begin( "Car suspension actions" );
        if( frame > 0 ) ImGui::GetStateStorage()->SetInt( ImGui::GetID( "CarSuspensionSettings" ), 1 );
        ImGui::PushID( "CarSuspensionSettings" );
        GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( frame == 2 || frame == 4 ? "Car limit" : "Car spring" );
        ImGui::PopID();
        carView->drawControls();
        ImGui::End();
        ImGui::Render();
        for( const auto id : carModel.getCarJoints() )
        {
            const auto data = carModel.getWorld().getWheelJointData( id );
            if( data.enableSpring != ( frame == 0 || frame >= 3 ) || data.enableLimit != ( frame < 2 || frame == 4 ) )
            {
                std::fprintf( stderr, "folded car suspension toggle failed to update both wheels\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    for( int setting = 0; setting < 4; ++setting )
    {
        for( int upper = 0; upper < 2; ++upper )
        {
            auto tuningView = createDemoView( demoKind::motorCar );
            auto& tuningModel = static_cast<rigidBodyDemo&>( *tuningView );
            for( int phase = 0; phase < 3; ++phase )
            {
                if( phase == 1 ) io.AddInputCharactersUTF8( upper ? "9000" : "-9000" );
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
                ImGui::NewFrame();
                ImGui::SetNextWindowSize( { 340.0f, 4000.0f } );
                ImGui::Begin( "Car suspension input" );
                ImGui::GetStateStorage()->SetInt( ImGui::GetID( "CarSuspensionSettings" ), 1 );
                if( phase == 0 )
                {
                    ImGui::PushID( "CarSuspensionSettings" );
                    GImGui->NavActivateId = ImGui::GetID( setting == 0 ? "Car hertz" : setting == 1 ? "Car damping" : setting == 2 ? "Car lower" : "Car upper" );
                    GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                    ImGui::PopID();
                }
                tuningView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( phase == 2 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            }
            for( const auto id : tuningModel.getCarJoints() )
            {
                const auto data = tuningModel.getWorld().getWheelJointData( id );
                const float value = setting == 0 ? data.hertz : setting == 1 ? data.dampingRatio : setting == 2 ? data.lowerTranslation : data.upperTranslation;
                const float expected = setting == 0 ? ( upper ? 10.0f : 0.0f ) : setting == 1 ? ( upper ? 2.0f : 0.0f ) : setting == 2 ? ( upper ? 0.25f : -0.5f ) : ( upper ? 0.5f : -0.25f );
                if( value != expected || data.lowerTranslation > data.upperTranslation )
                {
                    std::fprintf( stderr, "car suspension input failed to clamp or update both wheels\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
    }
    ImGui::DestroyContext();
}
