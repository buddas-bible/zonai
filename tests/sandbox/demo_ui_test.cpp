#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <numbers>
#include <array>

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
    for( int frame = 0; frame < 33; ++frame )
    {
        session.selectDemo( frame == 32 ? demoKind::revoluteMotorTorqueComparison : frame == 31 ? demoKind::distanceLimitMotorComparison : frame == 30 ? demoKind::distanceMotorForceComparison : frame == 29 ? demoKind::springDampingComparison : frame == 28 ? demoKind::centralOffsetImpactComparison : frame == 27 ? demoKind::collisionMassRatioComparison : frame == 26 ? demoKind::obliqueImpactFrictionComparison : frame == 25 ? demoKind::frictionForceComparison : frame == 24 ? demoKind::rollingSlidingFrictionComparison : frame == 23 ? demoKind::movingPlatformFrictionComparison : frame == 22 ? demoKind::collisionGroupComparison : frame == 21 ? demoKind::lowSpeedRestitutionComparison : frame == 20 ? demoKind::restitutionMixingComparison : frame == 19 ? demoKind::frictionMixingComparison : frame == 18 ? demoKind::maximumLinearSpeedComparison : frame == 17 ? demoKind::fastRotationComparison : frame == 16 ? demoKind::compoundCenterComparison : frame == 15 ? demoKind::dynamicKinematicComparison : frame == 14 ? demoKind::torqueImpulseComparison : frame == 13 ? demoKind::forceImpulseComparison : frame == 12 ? demoKind::rotationalInertiaComparison : frame == 11 ? demoKind::impulsePointComparison : frame == 10 ? demoKind::massImpulseComparison : frame == 9 ? demoKind::gravityScaleComparison : frame == 8 ? demoKind::dampingComparison : frame == 7 ? demoKind::sleepWakeComparison : frame == 6 ? demoKind::sensorWallComparison : getDemoEntries()[static_cast<std::size_t>( frame ) % getDemoEntries().size()].kind );
        session.stepOnce( 4 );
        auto& model = static_cast<rigidBodyDemo&>( session.getDemo() );
        demoInput input{};
        input.mousePressed = true;
        input.mouseHeld = true;
        const auto body = model.getKind() == demoKind::distancePendulum ? model.getPendulumBody() : model.getImpulseBody();
        input.mousePosition = model.getWorld().GetBodyTransform( body ).position;
        if( model.getKind() == demoKind::compoundCenterComparison ) input.mousePosition = zonai::TransformPoint( model.getWorld().GetBodyTransform( body ), { -0.75f, 0 } );
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
    auto pointView = createDemoView( demoKind::impulsePointComparison );
    auto& pointTrial = static_cast<rigidBodyDemo&>( *pointView );
    const auto pointMiddle = pointTrial.getShapes()[1];
    pointTrial.getWorld().SetBodyTransform( pointMiddle.bodyHandle, { { 1, 2 }, zonai::rot2::FromRadians( 0.5f * std::numbers::pi_v<float> ) } );
    pointTrial.getWorld().SetBodyTransform( pointTrial.getShapes()[2].bodyHandle, { { 4, 2 }, zonai::rot2::FromRadians( std::numbers::pi_v<float> ) } );
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0, 0 } );
    ImGui::SetNextWindowSize( { 900, 700 } );
    ImGui::Begin( "Point trial markers" );
    debugCamera pointCamera{};
    pointCamera.center = { 0, 1 };
    pointCamera.pixelsPerMeter = 75.0f;
    const auto viewport = ImGui::GetCursorScreenPos();
    ImDrawList* pointList = ImGui::GetWindowDrawList();
    const int pointStart = pointList->VtxBuffer.Size;
    debugDraw pointDraw{ pointList, pointCamera, viewport, { 850, 650 } };
    pointView->draw( pointDraw );
    for( const zonai::vec2 expected : { zonai::vec2{ -3, 0 }, zonai::vec2{ 1, 1.5f }, zonai::vec2{ 3.5f, 2 } } )
    {
        const auto center = pointCamera.WorldToScreen( expected, viewport, { 850, 650 } );
        const auto tip = pointCamera.WorldToScreen( expected + zonai::vec2{ 0, 0.75f }, viewport, { 850, 650 } );
        bool markerFound = false, arrowFound = false;
        for( int i = pointStart; i < pointList->VtxBuffer.Size; ++i )
        {
            const auto& vertex = pointList->VtxBuffer[i];
            if( vertex.col != IM_COL32( 255, 185, 80, 255 ) ) continue;
            const float dx = std::abs( vertex.pos.x - center.x );
            markerFound |= dx >= 3.0f && dx <= 6.0f && std::abs( vertex.pos.y - center.y ) <= 6.0f;
            arrowFound |= std::abs( vertex.pos.x - tip.x ) < 2.0f && std::abs( vertex.pos.y - tip.y ) < 2.0f;
        }
        if( !markerFound || !arrowFound )
        {
            std::fprintf( stderr, "point markers or world-up arrows did not follow edited body transforms\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    ImGui::End();
    ImGui::Render();
    pointTrial.resetCollisionTrial();
    for( int frame = 0; frame < 14; ++frame )
    {
        if( frame == 4 ) io.AddInputCharactersUTF8( "1.570796" );
        if( frame == 8 ) io.AddInputCharactersUTF8( "2" );
        if( frame == 5 || frame == 9 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 6 || frame == 10 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Point trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Transform" ), 1 );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
        if( frame == 0 || frame == 1 || frame == 6 || frame == 11 || frame == 12 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "PointTrialImpulse" );
        if( frame == 2 || frame == 10 || frame == 13 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        if( frame == 3 || frame == 7 )
        {
            GImGui->NavActivateId = ImGui::GetID( frame == 3 ? "Rotation" : "Density" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        pointView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame == 0 || frame == 1 || frame == 6 || frame == 11 || frame == 12 )
        {
            const float multiplier = frame == 1 || frame == 12 ? 2.0f : 1.0f;
            for( std::size_t i = 0; i < pointTrial.getShapes().size(); ++i )
            {
                const auto body = pointTrial.getShapes()[i].bodyHandle;
                const float speed = multiplier * ( i == 1 && frame >= 11 ? 1.0f : 2.0f );
                const float spin = multiplier * ( i == 0 || ( i == 1 && frame == 6 ) ? 0.0f : i == 1 ? ( frame >= 11 ? -3.0f : -6.0f ) : 6.0f );
                if( pointTrial.getWorld().GetBodyLinearVelocity( body ).y != speed || std::abs( pointTrial.getWorld().GetBodyAngularVelocity( body ) - spin ) > 1.0e-4f || pointTrial.getWorld().GetBodyTransform( body ).position.y != 0.0f )
                {
                    std::fprintf( stderr, "point UI action did not use current rotation density or accumulated velocities\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 2 || frame == 10 || frame == 13 )
        {
            for( const auto& visual : pointTrial.getShapes() )
                if( zonai::LengthSquared( pointTrial.getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ) != 0.0f || pointTrial.getWorld().GetBodyAngularVelocity( visual.bodyHandle ) != 0.0f || pointTrial.getWorld().GetBodyTransform( visual.bodyHandle ).rotation.s != 0.0f )
                {
                    std::fprintf( stderr, "point UI reset did not clear rotation and velocities\n" );
                    std::exit( EXIT_FAILURE );
                }
        }
        if( frame >= 9 && pointTrial.getWorld().GetShapeDensity( pointMiddle.shapeHandle ) != 2.0f )
        {
            std::fprintf( stderr, "point inspector density edit or selection was lost by reset\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 6 )
        {
            const auto point = pointTrial.getImpulseTrialPoint( 1 );
            if( !point || zonai::Length( *point - zonai::vec2{ 0, -0.5f } ) > 1.0e-5f )
            {
                std::fprintf( stderr, "point query did not follow inspector rotation editing\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    for( int parameter : { 0, 1, 2, 3, 4 } )
    {
        auto limitView = createDemoView( demoKind::distanceLimitMotorComparison );
        auto& limitTrial = static_cast<rigidBodyDemo&>( *limitView );
        const auto joints = limitTrial.getLimitComparisonJoints();
        const int row = parameter == 0 ? 0 : 1;
        const char* controls[] = { "Distance limit enabled", "Distance limit enabled", "Distance lower limit", "Distance upper limit", "Motor target speed" };
        if( parameter == 2 ) limitTrial.getWorld().setDistanceJointMotor( joints[1], true, -1.0f, 2.0f );
        for( int step = 0; step < 30; ++step ) limitTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 && parameter >= 2 ) io.AddInputCharactersUTF8( parameter == 2 ? "0.9" : parameter == 3 ? "1.25" : "0.5" );
            if( frame == 4 && parameter >= 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Limit motor comparison settings" );
            if( frame == 2 )
            {
                ImGui::PushID( row );
                GImGui->NavActivateId = ImGui::GetID( controls[parameter] );
                if( parameter < 2 ) GImGui->NavActivateDownId = GImGui->NavActivateId;
                else GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                ImGui::PopID();
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            limitView->drawControls();
            ImGui::End();
            ImGui::Render();
            const auto edited = limitTrial.getWorld().getDistanceJointData( joints[row] );
            if( frame >= 4 && ( parameter == 0 ? !edited.enableLimit : parameter == 1 ? edited.enableLimit : parameter == 2 ? edited.minLength != 0.9f : parameter == 3 ? edited.maxLength != 1.25f : edited.motorSpeed != 0.5f ) )
            {
                std::fprintf( stderr, "limit motor controls must preserve independent enable bounds and speed edits\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < limitTrial.getShapes().size(); ++i )
                {
                    const auto body = limitTrial.getShapes()[i].bodyHandle;
                    const auto pose = limitTrial.getWorld().GetBodyTransform( body );
                    if( pose.position.x != -2.0f + 4.0f * static_cast<float>( i ) || pose.position.y != 0.5f || zonai::LengthSquared( limitTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                    {
                        std::fprintf( stderr, "limit motor UI reset must restore initial positions and rest\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 45; ++step ) limitTrial.step( 1.0f / 60.0f, 4 );
                    const auto result = limitTrial.getWorld().getDistanceJointData( joints[row] );
                    const float expected = parameter == 0 ? 1.5f : parameter == 1 ? 1.75f : parameter == 2 ? 0.9f : parameter == 3 ? 1.25f : 1.375f;
                    if( std::abs( result.currentLength - expected ) > ( parameter == 1 ? 0.04f : 0.015f ) )
                    {
                        std::fprintf( stderr, "limit motor UI edits must change repeated trial motion: parameter %d actual %g expected %g\n", parameter, result.currentLength, expected );
                        std::exit( EXIT_FAILURE );
                    }
                }
            }
        }
    }
    for( int parameter : { 0, 1, 2 } )
    {
        auto motorView = createDemoView( demoKind::revoluteMotorTorqueComparison );
        auto& motorTrial = static_cast<rigidBodyDemo&>( *motorView );
        const auto joints = motorTrial.getTorqueComparisonJoints();
        for( int step = 0; step < 30; ++step ) motorTrial.step( 1.0f / 60.0f, 4 );
        for( int step = 0; step < 30; ++step ) motorTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( parameter == 0 ? "0.2" : parameter == 1 ? "1" : "-2" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Torque comparison settings" );
            if( frame == 2 )
            {
                ImGui::PushID( parameter == 0 ? 0 : 2 );
                GImGui->NavActivateId = ImGui::GetID( parameter == 0 ? "Motor torque limit" : "Motor target angular speed" );
                ImGui::PopID();
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            motorView->drawControls();
            ImGui::End();
            ImGui::Render();
            const auto edited = motorTrial.getWorld().getRevoluteJointData( joints[parameter == 0 ? 0 : 2] );
            if( frame >= 4 && ( parameter == 0 ? edited.maxMotorTorque != 0.2f : edited.motorSpeed != ( parameter == 1 ? 1.0f : -2.0f ) ) )
            {
                std::fprintf( stderr, "torque comparison controls must preserve independently edited torque and angular speed\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < motorTrial.getShapes().size(); ++i )
                {
                    const auto body = motorTrial.getShapes()[i].bodyHandle;
                    const auto pose = motorTrial.getWorld().GetBodyTransform( body );
                    if( pose.position.x != -2.0f + 2.0f * static_cast<float>( i ) || pose.position.y != 0.0f || pose.rotation.s != 0.0f || motorTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f || zonai::LengthSquared( motorTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                    {
                        std::fprintf( stderr, "torque comparison UI reset must restore initial positions and rest\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 30; ++step ) motorTrial.step( 1.0f / 60.0f, 4 );
                    const std::size_t index = parameter == 0 ? 0 : 2;
                    const float speed = motorTrial.getWorld().GetBodyAngularVelocity( motorTrial.getShapes()[index].bodyHandle );
                    if( std::abs( speed - ( parameter == 0 ? 2.0f : parameter == 1 ? 1.0f : -2.0f ) ) > 1.0e-4f )
                    {
                        std::fprintf( stderr, "edited torque controls must change repeated target tracking after UI reset\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
            }
        }
    }
    for( int parameter : { 0, 1 } )
    {
        auto motorView = createDemoView( demoKind::distanceMotorForceComparison );
        auto& motorTrial = static_cast<rigidBodyDemo&>( *motorView );
        const auto joints = motorTrial.getMotorComparisonJoints();
        for( int step = 0; step < 15; ++step ) motorTrial.step( 1.0f / 60.0f, 4 );
        for( int step = 0; step < 30; ++step ) motorTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( parameter == 0 ? "2" : "0.5" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Motor comparison settings" );
            if( frame == 2 )
            {
                ImGui::PushID( parameter == 0 ? 0 : 2 );
                GImGui->NavActivateId = ImGui::GetID( parameter == 0 ? "Motor force limit" : "Motor target speed" );
                ImGui::PopID();
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            motorView->drawControls();
            ImGui::End();
            ImGui::Render();
            const auto edited = motorTrial.getWorld().getDistanceJointData( joints[parameter == 0 ? 0 : 2] );
            if( frame >= 4 && ( parameter == 0 ? edited.maxMotorForce != 2.0f : edited.motorSpeed != 0.5f ) )
            {
                std::fprintf( stderr, "motor comparison controls must preserve independently edited force and target speed\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < motorTrial.getShapes().size(); ++i )
                {
                    const auto body = motorTrial.getShapes()[i].bodyHandle;
                    const auto pose = motorTrial.getWorld().GetBodyTransform( body );
                    if( pose.position.x != -3.0f + 3.0f * static_cast<float>( i ) || pose.position.y != 0.5f || zonai::LengthSquared( motorTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                    {
                        std::fprintf( stderr, "motor comparison UI reset must restore initial positions and rest\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 15; ++step ) motorTrial.step( 1.0f / 60.0f, 4 );
                    const std::size_t index = parameter == 0 ? 0 : 2;
                    const float speed = -motorTrial.getWorld().GetBodyLinearVelocity( motorTrial.getShapes()[index].bodyHandle ).y;
                    if( std::abs( speed - ( parameter == 0 ? 1.0f : 0.5f ) ) > 1.0e-4f )
                    {
                        std::fprintf( stderr, "edited motor controls must change repeated target tracking after UI reset\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
            }
        }
    }
    for( int parameter : { 0, 1 } )
    {
        auto springView = createDemoView( demoKind::springDampingComparison );
        auto& springTrial = static_cast<rigidBodyDemo&>( *springView );
        const auto joints = springTrial.getSpringComparisonJoints();
        for( int step = 0; step < 15; ++step ) springTrial.step( 1.0f / 60.0f, 4 );
        const float originalLength = springTrial.getWorld().getDistanceJointData( joints[1] ).currentLength;
        for( int step = 0; step < 30; ++step ) springTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( parameter == 0 ? "2" : "1" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Spring comparison settings" );
            if( frame == 2 )
            {
                ImGui::PushID( parameter );
                GImGui->NavActivateId = ImGui::GetID( parameter == 0 ? "Spring damping" : "Spring Hertz" );
                ImGui::PopID();
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            springView->drawControls();
            ImGui::End();
            ImGui::Render();
            const auto edited = springTrial.getWorld().getDistanceJointData( joints[parameter] );
            if( frame >= 4 && ( parameter == 0 ? edited.dampingRatio != 2.0f : edited.hertz != 1.0f ) )
            {
                std::fprintf( stderr, "spring comparison controls must preserve independently edited damping and frequency\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < springTrial.getShapes().size(); ++i )
                {
                    const auto body = springTrial.getShapes()[i].bodyHandle;
                    const auto pose = springTrial.getWorld().GetBodyTransform( body );
                    if( pose.position.x != -3.0f + 3.0f * static_cast<float>( i ) || pose.position.y != 0.0f || zonai::LengthSquared( springTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                    {
                        std::fprintf( stderr, "spring comparison UI reset must restore all stretched poses and rest\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 15; ++step ) springTrial.step( 1.0f / 60.0f, 4 );
                    const float length = springTrial.getWorld().getDistanceJointData( joints[parameter] ).currentLength;
                    const float reference = springTrial.getWorld().getDistanceJointData( joints[2] ).currentLength;
                    if( parameter == 0 ? length <= 1.0f || std::abs( length - reference ) > 1.0e-4f : length <= originalLength + 0.05f )
                    {
                        std::fprintf( stderr, "edited spring controls must change the repeated trajectory after UI reset\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
            }
        }
    }
    for( std::size_t component : { 3u, 5u } )
    {
        auto offsetView = createDemoView( demoKind::centralOffsetImpactComparison );
        auto& offsetTrial = static_cast<rigidBodyDemo&>( *offsetView );
        const auto selected = offsetTrial.getShapes()[component];
        for( int step = 0; step < 45; ++step ) offsetTrial.step( 1.0f / 60.0f, 4 );
        std::array<float, 3> originalSpin{};
        for( std::size_t row = 0; row < 3; ++row ) originalSpin[row] = offsetTrial.getWorld().GetBodyAngularVelocity( offsetTrial.getShapes()[2 * row + 1].bodyHandle );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( "2" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Offset impact inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Density" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            offsetView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && offsetTrial.getWorld().GetShapeDensity( selected.shapeHandle ) != 2.0f )
            {
                std::fprintf( stderr, "offset inspector must preserve the selected upper or lower target density edit\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < offsetTrial.getShapes().size(); ++i )
                {
                    const auto body = offsetTrial.getShapes()[i].bodyHandle;
                    const auto pose = offsetTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = offsetTrial.getWorld().GetBodyLinearVelocity( body );
                    const float offset = i == 2 ? 0.25f : i == 4 ? -0.25f : 0.0f;
                    if( pose.position.x != ( i % 2 == 0 ? -2.0f : 0.0f ) || pose.position.y != 2.0f - 2.0f * static_cast<float>( i / 2 ) + offset || pose.rotation.s != 0.0f || velocity.x != ( i % 2 == 0 ? 4.0f : 0.0f ) || velocity.y != 0.0f || offsetTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                    {
                        std::fprintf( stderr, "offset UI reset must restore mirrored launch poses and zero-spin velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 45; ++step ) offsetTrial.step( 1.0f / 60.0f, 4 );
                    for( std::size_t row = 0; row < 3; ++row )
                    {
                        const float spin = offsetTrial.getWorld().GetBodyAngularVelocity( offsetTrial.getShapes()[2 * row + 1].bodyHandle );
                        const bool response = 2 * row + 1 == component ? spin * originalSpin[row] > 0.0f && std::abs( spin ) < 0.7f * std::abs( originalSpin[row] ) && std::abs( spin ) > 0.3f * std::abs( originalSpin[row] ) : std::abs( spin - originalSpin[row] ) < 0.03f;
                        if( !response )
                        {
                            std::fprintf( stderr, "inspector density editing must reduce only the selected target spin while preserving its rotation direction\n" );
                            std::exit( EXIT_FAILURE );
                        }
                    }
                }
            }
        }
    }
    for( std::size_t component : { 3u, 4u } )
    {
        auto massView = createDemoView( demoKind::collisionMassRatioComparison );
        auto& massTrial = static_cast<rigidBodyDemo&>( *massView );
        const auto selected = massTrial.getShapes()[component];
        for( int step = 0; step < 45; ++step ) massTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( "1" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Collision mass ratio inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Density" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            massView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && massTrial.getWorld().GetShapeDensity( selected.shapeHandle ) != 1.0f )
            {
                std::fprintf( stderr, "mass ratio inspector must preserve the selected target or moving circle density edit\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < massTrial.getShapes().size(); ++i )
                {
                    const auto body = massTrial.getShapes()[i].bodyHandle;
                    const auto pose = massTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = massTrial.getWorld().GetBodyLinearVelocity( body );
                    if( pose.position.x != ( i % 2 == 0 ? -2.0f : 0.0f ) || pose.position.y != 2.0f - 2.0f * static_cast<float>( i / 2 ) || pose.rotation.s != 0.0f || velocity.x != ( i % 2 == 0 ? 4.0f : 0.0f ) || velocity.y != 0.0f || massTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                    {
                        std::fprintf( stderr, "mass ratio UI reset must restore original poses and launch velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 45; ++step ) massTrial.step( 1.0f / 60.0f, 4 );
                    std::array<float, 6> expected{ 0.0f, 4.0f, -2.4f, 1.6f, 2.4f, 6.4f };
                    const std::size_t pair = component == 3 ? 2 : 4;
                    expected[pair] = 0.0f;
                    expected[pair + 1] = 4.0f;
                    for( std::size_t i = 0; i < expected.size(); ++i )
                        if( std::abs( massTrial.getWorld().GetBodyLinearVelocity( massTrial.getShapes()[i].bodyHandle ).x - expected[i] ) > 0.03f )
                        {
                            std::fprintf( stderr, "inspector density editing must change the selected pair impact and preserve the other mass ratios\n" );
                            std::exit( EXIT_FAILURE );
                        }
                }
            }
        }
    }
    for( std::size_t component : { 0u, 3u } )
    {
        auto forceView = createDemoView( demoKind::frictionForceComparison );
        auto& forceTrial = static_cast<rigidBodyDemo&>( *forceView );
        const auto selected = forceTrial.getShapes()[component];
        const float coefficient = component == 0 ? 0.0f : 2.0f;
        for( int step = 0; step < 60; ++step ) forceTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( component == 0 ? "0" : "2" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Friction force inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Friction" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame == 5 || frame == 6 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "FrictionForceStart" );
            if( frame == 7 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            forceView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && forceTrial.getWorld().GetShapeFriction( selected.shapeHandle ) != coefficient )
            {
                std::fprintf( stderr, "force inspector must preserve selected floor or circle friction edits\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                if( forceTrial.getForceTrialTime() != 0.0 || forceTrial.isForceTrialActive() != ( frame != 7 ) )
                {
                    std::fprintf( stderr, "force start and reset buttons must restart or cancel the timed trial\n" );
                    std::exit( EXIT_FAILURE );
                }
                for( std::size_t i = 0; i < forceTrial.getShapes().size(); ++i )
                {
                    const auto body = forceTrial.getShapes()[i].bodyHandle;
                    const auto pose = forceTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = forceTrial.getWorld().GetBodyLinearVelocity( body );
                    const float lane = i < 2 ? 1.5f : -1.5f;
                    if( pose.position.x != ( i % 2 == 0 ? 0.0f : -3.0f ) || pose.position.y != lane + ( i % 2 == 0 ? 0.0f : 0.65f ) || pose.rotation.s != 0.0f || velocity.x != 0.0f || velocity.y != 0.0f || forceTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                    {
                        std::fprintf( stderr, "force UI start and reset must restore initial poses and rest\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 60; ++step ) forceTrial.step( 1.0f / 60.0f, 4 );
                    for( std::size_t row = 0; row < 2; ++row )
                    {
                        const auto box = forceTrial.getShapes()[2 * row + 1].bodyHandle;
                        const float speed = forceTrial.getWorld().GetBodyLinearVelocity( box ).x;
                        const bool response = component == 0 ? std::abs( speed - ( row == 0 ? 2.0f : 3.0f ) ) < 0.3f : std::abs( speed ) < 0.05f;
                        if( !response || forceTrial.isForceTrialActive() )
                        {
                            std::fprintf( stderr, "inspector friction edits must change repeated rest and motion while timed forces end\n" );
                            std::exit( EXIT_FAILURE );
                        }
                    }
                }
            }
        }
    }
    for( std::size_t component : { 0u, 3u } )
    {
        auto impactView = createDemoView( demoKind::obliqueImpactFrictionComparison );
        auto& impactTrial = static_cast<rigidBodyDemo&>( *impactView );
        const auto selected = impactTrial.getShapes()[component];
        const float coefficient = component == 0 ? 0.0f : 0.8f;
        for( int step = 0; step < 36; ++step ) impactTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( component == 0 ? "0" : "0.8" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Oblique impact inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Friction" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            impactView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && impactTrial.getWorld().GetShapeFriction( selected.shapeHandle ) != coefficient )
            {
                std::fprintf( stderr, "impact inspector must preserve selected floor or circle friction edits\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < impactTrial.getShapes().size(); ++i )
                {
                    const auto body = impactTrial.getShapes()[i].bodyHandle;
                    const auto pose = impactTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = impactTrial.getWorld().GetBodyLinearVelocity( body );
                    const float lane = i < 2 ? 1.5f : -1.5f;
                    if( pose.position.x != ( i % 2 == 0 ? 0.0f : -2.0f ) || pose.position.y != lane + ( i % 2 == 0 ? 0.0f : 1.75f ) || pose.rotation.s != 0.0f || velocity.x != ( i % 2 == 0 ? 0.0f : 3.0f ) || velocity.y != ( i % 2 == 0 ? 0.0f : -4.0f ) || impactTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                    {
                        std::fprintf( stderr, "impact UI reset must restore initial poses and zero-spin oblique velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 36; ++step ) impactTrial.step( 1.0f / 60.0f, 4 );
                    for( std::size_t row = 0; row < 2; ++row )
                    {
                        const auto circle = impactTrial.getShapes()[2 * row + 1].bodyHandle;
                        const float speed = impactTrial.getWorld().GetBodyLinearVelocity( circle ).x;
                        const float angularVelocity = impactTrial.getWorld().GetBodyAngularVelocity( circle );
                        const float normalSpeed = impactTrial.getWorld().GetBodyLinearVelocity( circle ).y;
                        const bool response = component == 0 ? std::abs( speed - 3.0f ) < 0.02f && std::abs( angularVelocity ) < 0.02f : std::abs( speed - 2.0f ) < 0.15f && std::abs( angularVelocity + 5.0f ) < 0.4f;
                        if( !response || std::abs( normalSpeed - 4.0f ) > 0.15f )
                        {
                            std::fprintf( stderr, "inspector friction edits must change tangent speed and spin without changing normal rebound\n" );
                            std::exit( EXIT_FAILURE );
                        }
                    }
                }
            }
        }
    }
    for( std::size_t component : { 0u, 3u } )
    {
        auto rollingView = createDemoView( demoKind::rollingSlidingFrictionComparison );
        auto& rollingTrial = static_cast<rigidBodyDemo&>( *rollingView );
        const auto selected = rollingTrial.getShapes()[component];
        const float coefficient = component == 0 ? 0.0f : 0.8f;
        for( int step = 0; step < 60; ++step ) rollingTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( component == 0 ? "0" : "0.8" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Rolling sliding inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Friction" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            rollingView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && rollingTrial.getWorld().GetShapeFriction( selected.shapeHandle ) != coefficient )
            {
                std::fprintf( stderr, "rolling inspector must preserve selected floor or circle friction edits\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < rollingTrial.getShapes().size(); ++i )
                {
                    const auto body = rollingTrial.getShapes()[i].bodyHandle;
                    const auto pose = rollingTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = rollingTrial.getWorld().GetBodyLinearVelocity( body );
                    const float lane = i < 2 ? 1.5f : -1.5f;
                    if( pose.position.x != ( i % 2 == 0 ? 0.0f : -3.0f ) || pose.position.y != lane + ( i % 2 == 0 ? 0.0f : 0.55f ) || pose.rotation.s != 0.0f || velocity.x != ( i % 2 == 0 ? 0.0f : 4.0f ) || velocity.y != 0.0f || rollingTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                    {
                        std::fprintf( stderr, "rolling UI reset must restore initial poses and zero-spin sliding velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 60; ++step ) rollingTrial.step( 1.0f / 60.0f, 4 );
                    for( std::size_t row = 0; row < 2; ++row )
                    {
                        const auto circle = rollingTrial.getShapes()[2 * row + 1].bodyHandle;
                        const float speed = rollingTrial.getWorld().GetBodyLinearVelocity( circle ).x;
                        const float angularVelocity = rollingTrial.getWorld().GetBodyAngularVelocity( circle );
                        const bool response = component == 0 ? std::abs( speed - 4.0f ) < 0.02f && std::abs( angularVelocity ) < 0.02f : std::abs( speed - 8.0f / 3.0f ) < 0.12f && angularVelocity < -6.0f && std::abs( speed + 0.4f * angularVelocity ) < 0.03f;
                        if( !response )
                        {
                            std::fprintf( stderr, "inspector friction edits must switch repeated rolling and sliding responses\n" );
                            std::exit( EXIT_FAILURE );
                        }
                    }
                }
            }
        }
    }
    for( std::size_t component : { 0u, 3u } )
    {
        auto platformView = createDemoView( demoKind::movingPlatformFrictionComparison );
        auto& platformTrial = static_cast<rigidBodyDemo&>( *platformView );
        const auto selected = platformTrial.getShapes()[component];
        const float coefficient = component == 0 ? 0.0f : 0.8f;
        for( int step = 0; step < 60; ++step ) platformTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( component == 0 ? "0" : "0.8" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Moving platform inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Friction" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            platformView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && platformTrial.getWorld().GetShapeFriction( selected.shapeHandle ) != coefficient )
            {
                std::fprintf( stderr, "platform inspector must preserve selected platform or box friction edits\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < platformTrial.getShapes().size(); ++i )
                {
                    const auto body = platformTrial.getShapes()[i].bodyHandle;
                    const auto pose = platformTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = platformTrial.getWorld().GetBodyLinearVelocity( body );
                    const float lane = i < 2 ? 1.5f : -1.5f;
                    if( pose.position.x != -2.0f || pose.position.y != lane + ( i % 2 == 0 ? 0.0f : 0.55f ) || pose.rotation.s != 0.0f || velocity.x != ( i % 2 == 0 ? 2.0f : 0.0f ) || velocity.y != 0.0f )
                    {
                        std::fprintf( stderr, "platform UI reset must restore initial poses and distinct platform and box velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 60; ++step ) platformTrial.step( 1.0f / 60.0f, 4 );
                    for( std::size_t row = 0; row < 2; ++row )
                    {
                        const auto platform = platformTrial.getShapes()[2 * row].bodyHandle;
                        const auto box = platformTrial.getShapes()[2 * row + 1].bodyHandle;
                        const float speed = platformTrial.getWorld().GetBodyLinearVelocity( box ).x;
                        if( ( component == 0 ? std::abs( speed ) > 0.02f : speed < 1.8f || speed > 2.1f ) || platformTrial.getWorld().GetBodyLinearVelocity( platform ).x != 2.0f )
                        {
                            std::fprintf( stderr, "inspector friction edits must change repeated carrying while kinematic platforms retain their speed\n" );
                            std::exit( EXIT_FAILURE );
                        }
                    }
                }
            }
        }
    }
    for( std::size_t component : { 2u, 4u } )
    {
        auto groupView = createDemoView( demoKind::collisionGroupComparison );
        auto& groupTrial = static_cast<rigidBodyDemo&>( *groupView );
        const auto selected = groupTrial.getShapes()[component];
        for( int step = 0; step < 90; ++step ) groupTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( "0" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Group trial inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision mask" ), 1 );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "ObjectCollisionOverrides" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Group index" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            groupView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && groupTrial.getWorld().GetShapeFilter( selected.shapeHandle ).groupIndex != 0 )
            {
                std::fprintf( stderr, "group inspector edit and trial reset must preserve the selected group value\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < groupTrial.getShapes().size(); ++i )
                {
                    const auto body = groupTrial.getShapes()[i].bodyHandle;
                    const auto pose = groupTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = groupTrial.getWorld().GetBodyLinearVelocity( body );
                    if( pose.position.x != ( i % 2 == 0 ? -2.0f : 2.0f ) || pose.position.y != 2.0f - 2.0f * static_cast<float>( i / 2 ) || velocity.x != ( i % 2 == 0 ? 2.0f : -2.0f ) || velocity.y != 0.0f )
                    {
                        std::fprintf( stderr, "group UI reset must restore all initial poses and launch velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 90; ++step ) groupTrial.step( 1.0f / 60.0f, 4 );
                    for( std::size_t row = 0; row < 3; ++row )
                    {
                        const float velocity = groupTrial.getWorld().GetBodyLinearVelocity( groupTrial.getShapes()[2 * row].bodyHandle ).x;
                        const bool collision = component == 4 && row != 0;
                        if( collision ? velocity >= 0 : velocity <= 0 )
                        {
                            std::fprintf( stderr, "inspector group edits must change the selected pair collision and preserve other rows\n" );
                            std::exit( EXIT_FAILURE );
                        }
                    }
                }
            }
        }
        if( component == 4 )
        {
            for( int frame = 0; frame < 4; ++frame )
            {
                ImGui::NewFrame();
                ImGui::SetNextWindowPos( { 0, 0 } );
                ImGui::SetNextWindowSize( { 340, 4000 } );
                ImGui::Begin( "Group trial inspector" );
                if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Collision partners" );
                if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( "레이어 02##Mask" );
                if( frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
                groupView->drawControls();
                ImGui::End();
                ImGui::Render();
                if( frame == 2 ) ImGui::ClosePopupToLevel( 0, true );
            }
            for( int step = 0; step < 90; ++step ) groupTrial.step( 1.0f / 60.0f, 4 );
            if( groupTrial.getWorld().GetShapeFilter( selected.shapeHandle ).maskBits != ~std::uint64_t{ 2 } || groupTrial.getWorld().GetBodyLinearVelocity( selected.bodyHandle ).x <= 0 )
            {
                std::fprintf( stderr, "group zero must respect the edited inspector mask after trial reset\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    for( std::size_t component : { 1u, 3u } )
    {
        auto impactView = createDemoView( demoKind::lowSpeedRestitutionComparison );
        auto& impactTrial = static_cast<rigidBodyDemo&>( *impactView );
        const auto edited = impactTrial.getShapes()[component];
        for( int step = 0; step < 30; ++step ) impactTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( "0.5" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Low speed restitution inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( edited.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Restitution" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            impactView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && impactTrial.getWorld().GetShapeRestitution( edited.shapeHandle ) != 0.5f )
            {
                std::fprintf( stderr, "impact inspector must preserve the selected restitution edit\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < impactTrial.getShapes().size(); ++i )
                {
                    const auto body = impactTrial.getShapes()[i].bodyHandle;
                    const auto pose = impactTrial.getWorld().GetBodyTransform( body );
                    const auto velocity = impactTrial.getWorld().GetBodyLinearVelocity( body );
                    if( pose.position.x != ( i < 2 ? -2.0f : 2.0f ) || pose.position.y != ( i % 2 == 0 ? -0.15f : 0.4f ) || pose.rotation.s != 0.0f || velocity.x != 0.0f || velocity.y != ( i % 2 == 0 ? 0.0f : i == 1 ? -0.5f : -2.0f ) )
                    {
                        std::fprintf( stderr, "impact UI reset must restore touching poses and original impact velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 30; ++step ) impactTrial.step( 1.0f / 60.0f, 4 );
                    const float slow = impactTrial.getWorld().GetBodyLinearVelocity( impactTrial.getShapes()[1].bodyHandle ).y;
                    const float fast = impactTrial.getWorld().GetBodyLinearVelocity( impactTrial.getShapes()[3].bodyHandle ).y;
                    if( std::abs( slow ) > 0.02f || std::abs( fast - ( component == 1 ? 2.0f : 1.0f ) ) > 0.02f )
                    {
                        std::fprintf( stderr, "inspector restitution editing must change the repeated fast rebound while preserving slow impact suppression\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
            }
        }
    }
    for( std::size_t component : { 0u, 1u } )
    {
        auto bounceView = createDemoView( demoKind::restitutionMixingComparison );
        auto& bounceTrial = static_cast<rigidBodyDemo&>( *bounceView );
        const auto edited = bounceTrial.getShapes()[component];
        const float coefficient = component == 0 ? 1.0f : 0.4f;
        for( int step = 0; step < 60; ++step )
            bounceTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( component == 0 ? "1" : "0.4" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Restitution mixing inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( edited.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Restitution" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            bounceView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && bounceTrial.getWorld().GetShapeRestitution( edited.shapeHandle ) != coefficient )
            {
                std::fprintf( stderr, "restitution inspector must preserve the selected floor or circle coefficient edit\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < bounceTrial.getShapes().size(); ++i )
                {
                    const auto body = bounceTrial.getShapes()[i].bodyHandle;
                    const auto pose = bounceTrial.getWorld().GetBodyTransform( body );
                    if( pose.position.x != -3.0f + 3.0f * static_cast<float>( i / 2 ) || pose.position.y != ( i % 2 == 0 ? -0.15f : 3.0f ) || pose.rotation.s != 0.0f || zonai::LengthSquared( bounceTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                    {
                        std::fprintf( stderr, "restitution mixing UI reset must restore initial drop poses and rest\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    std::array<float, 3> peaks{};
                    for( int step = 0; step < 90; ++step )
                    {
                        bounceTrial.step( 1.0f / 60.0f, 4 );
                        for( std::size_t i = 0; i < peaks.size(); ++i )
                        {
                            const auto body = bounceTrial.getShapes()[2 * i + 1].bodyHandle;
                            if( bounceTrial.getWorld().GetBodyLinearVelocity( body ).y > 0.1f ) peaks[i] = std::max( peaks[i], bounceTrial.getWorld().GetBodyTransform( body ).position.y );
                        }
                    }
                    const bool editedResponse = component == 0 ? peaks[0] > 2.5f : peaks[0] > 0.55f && peaks[0] < 1.2f;
                    if( !editedResponse || peaks[1] <= 1.5f || peaks[1] >= 2.5f || peaks[2] != 0.0f )
                    {
                        std::fprintf( stderr, "editing either material in the inspector must change the repeated restitution mixing response\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
            }
        }
    }
    for( std::size_t component : { 0u, 1u } )
    {
        auto mixingView = createDemoView( demoKind::frictionMixingComparison );
        auto& mixingTrial = static_cast<rigidBodyDemo&>( *mixingView );
        const auto edited = mixingTrial.getShapes()[component];
        for( int step = 0; step < 60; ++step )
            mixingTrial.step( 1.0f / 60.0f, 4 );
        for( int frame = 0; frame < 8; ++frame )
        {
            if( frame == 3 ) io.AddInputCharactersUTF8( "0" );
            if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Friction mixing inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( edited.label );
            if( frame == 2 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Friction" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame >= 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            mixingView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 4 && mixingTrial.getWorld().GetShapeFriction( edited.shapeHandle ) != 0.0f )
            {
                std::fprintf( stderr, "mixing inspector must retain the selected floor or box friction edit\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame >= 5 )
            {
                for( std::size_t i = 0; i < mixingTrial.getShapes().size(); ++i )
                {
                    const auto body = mixingTrial.getShapes()[i].bodyHandle;
                    const auto pose = mixingTrial.getWorld().GetBodyTransform( body );
                    const float lane = 2.0f - 2.0f * static_cast<float>( i / 2 );
                    if( pose.position.x != ( i % 2 == 0 ? 0.0f : -3.0f ) || std::abs( pose.position.y - lane - ( i % 2 == 0 ? 0.0f : 0.55f ) ) > 1.0e-6f || pose.rotation.s != 0.0f || mixingTrial.getWorld().GetBodyLinearVelocity( body ).x != ( i % 2 == 0 ? 0.0f : 4.0f ) )
                    {
                        std::fprintf( stderr, "mixing UI reset must restore the same poses and horizontal launch velocities\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
                if( frame != 7 )
                {
                    for( int step = 0; step < 30; ++step )
                        mixingTrial.step( 1.0f / 60.0f, 4 );
                    const auto& physics = mixingTrial.getWorld();
                    const float speedA = physics.GetBodyLinearVelocity( mixingTrial.getShapes()[1].bodyHandle ).x;
                    const float speedB = physics.GetBodyLinearVelocity( mixingTrial.getShapes()[3].bodyHandle ).x;
                    const float speedC = physics.GetBodyLinearVelocity( mixingTrial.getShapes()[5].bodyHandle ).x;
                    if( std::abs( speedA - 4.0f ) > 1.0e-4f || speedB <= 1.0f || speedB >= 2.0f || std::abs( speedC - 4.0f ) > 1.0e-4f )
                    {
                        std::fprintf( stderr, "zeroing either material in the inspector must change repeated friction mixing response\n" );
                        std::exit( EXIT_FAILURE );
                    }
                }
            }
        }
    }
    auto speedView = createDemoView( demoKind::maximumLinearSpeedComparison );
    auto& speedTrial = static_cast<rigidBodyDemo&>( *speedView );
    speedTrial.step( 1.0f / 60.0f, 4 );
    for( int frame = 0; frame < 10; ++frame )
    {
        if( frame == 1 ) io.AddInputCharactersUTF8( "8" );
        if( frame == 6 ) io.AddInputCharactersUTF8( "1" );
        if( frame == 2 || frame == 7 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 3 || frame == 8 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Maximum linear speed inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WorldSettings" ), 1 );
        if( frame == 0 || frame == 5 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Max linear speed" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        if( frame == 3 || frame == 4 || frame == 8 || frame == 9 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        speedView->drawControls();
        ImGui::End();
        ImGui::Render();
        const float maximumSpeed = frame >= 7 ? 1.0f : frame >= 2 ? 8.0f : 4.0f;
        if( speedTrial.getWorld().GetMaximumLinearSpeed() != maximumSpeed )
        {
            std::fprintf( stderr, "maximum linear speed inspector edit was lost across repeated resets\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 2 && std::abs( zonai::Length( speedTrial.getWorld().GetBodyLinearVelocity( speedTrial.getShapes()[1].bodyHandle ) ) - 4.0f ) > 1.0e-5f )
        {
            std::fprintf( stderr, "raising the world limit must not automatically restore a previously capped velocity\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 3 || frame == 4 || frame == 8 || frame == 9 )
        {
            for( std::size_t i = 0; i < speedTrial.getShapes().size(); ++i )
            {
                const auto body = speedTrial.getShapes()[i].bodyHandle;
                const auto pose = speedTrial.getWorld().GetBodyTransform( body );
                const auto velocity = speedTrial.getWorld().GetBodyLinearVelocity( body );
                if( pose.position.x != ( i == 0 ? -2.5f : 2.5f ) || pose.position.y != 0.0f || pose.rotation.s != 0.0f || zonai::Length( velocity - zonai::vec2{ 0.6f, 0.8f } * ( i == 0 ? 2.0f : 8.0f ) ) > 1.0e-6f )
                {
                    std::fprintf( stderr, "linear speed UI reset must restore initial poses and launch velocities\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
            speedTrial.step( 1.0f / 60.0f, 4 );
            for( std::size_t i = 0; i < speedTrial.getShapes().size(); ++i )
            {
                const auto body = speedTrial.getShapes()[i].bodyHandle;
                const float expected = std::min( maximumSpeed, i == 0 ? 2.0f : 8.0f );
                if( zonai::Length( speedTrial.getWorld().GetBodyLinearVelocity( body ) - zonai::vec2{ 0.6f, 0.8f } * expected ) > 1.0e-5f || speedTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                {
                    std::fprintf( stderr, "world speed edit must change repeated trial results while preserving direction and spin\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
    }
    auto rotationView = createDemoView( demoKind::fastRotationComparison );
    auto& rotationTrial = static_cast<rigidBodyDemo&>( *rotationView );
    const auto rotationLeft = rotationTrial.getShapes()[0].bodyHandle;
    const auto rotationRight = rotationTrial.getShapes()[1].bodyHandle;
    rotationTrial.step( 1.0f / 60.0f, 4 );
    for( int frame = 0; frame < 6; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Fast rotation trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Sleep and CCD" ), 1 );
        if( frame == 0 || frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Allow fast rotation" );
        if( frame == 1 || frame == 2 || frame == 4 || frame == 5 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        rotationView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( rotationTrial.getWorld().IsBodyFastRotationAllowed( rotationRight ) != ( frame >= 3 ) )
        {
            std::fprintf( stderr, "fast rotation inspector flag edit was lost across reset or changed the wrong rod\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 1 || frame == 2 || frame == 4 || frame == 5 )
        {
            if( rotationTrial.getRotationTrialTimeStep() != 0.0f || rotationTrial.getWorld().GetBodyAngularVelocity( rotationLeft ) != 120.0f || rotationTrial.getWorld().GetBodyAngularVelocity( rotationRight ) != 120.0f || rotationTrial.getWorld().GetBodyTransform( rotationLeft ).rotation.s != 0.0f || rotationTrial.getWorld().GetBodyTransform( rotationRight ).rotation.s != 0.0f )
            {
                std::fprintf( stderr, "fast rotation UI reset must restore initial spin and poses and clear the step readout\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame != 5 )
            {
                rotationTrial.step( 1.0f / 60.0f, 4 );
                const float limit = 15.0f * std::numbers::pi_v<float>;
                const float rightExpected = frame >= 3 ? 120.0f : limit;
                if( std::abs( rotationTrial.getWorld().GetBodyAngularVelocity( rotationLeft ) - limit ) > 1.0e-4f || std::abs( rotationTrial.getWorld().GetBodyAngularVelocity( rotationRight ) - rightExpected ) > 1.0e-4f || rotationTrial.getRotationTrialTimeStep() != 1.0f / 60.0f || zonai::LengthSquared( rotationTrial.getWorld().GetBodyLinearVelocity( rotationRight ) ) != 0.0f )
                {
                    std::fprintf( stderr, "fast rotation inspector edit must change the repeated trial cap without linear motion\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
    }
    auto compoundView = createDemoView( demoKind::compoundCenterComparison );
    auto& compoundTrial = static_cast<rigidBodyDemo&>( *compoundView );
    const auto compoundPart = compoundTrial.getShapes()[1];
    const auto compoundOffset = compoundTrial.getShapes()[2].bodyHandle;
    compoundTrial.getWorld().SetBodyTransform( compoundOffset, { { 1, 2 }, zonai::rot2::FromRadians( 0.5f * std::numbers::pi_v<float> ) } );
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0, 0 } );
    ImGui::SetNextWindowSize( { 900, 700 } );
    ImGui::Begin( "Compound trial markers" );
    debugCamera compoundCamera{};
    compoundCamera.center = { 0, 1 };
    compoundCamera.pixelsPerMeter = 75.0f;
    const auto compoundViewport = ImGui::GetCursorScreenPos();
    ImDrawList* compoundList = ImGui::GetWindowDrawList();
    const int compoundStart = compoundList->VtxBuffer.Size;
    debugDraw compoundDraw{ compoundList, compoundCamera, compoundViewport, { 850, 650 } };
    compoundView->draw( compoundDraw );
    for( const zonai::vec2 expected : { zonai::vec2{ -3, 0 }, zonai::vec2{ 1, 2 } } )
    {
        const auto center = compoundCamera.WorldToScreen( expected, compoundViewport, { 850, 650 } );
        const auto tip = compoundCamera.WorldToScreen( expected + zonai::vec2{ 0, 0.75f }, compoundViewport, { 850, 650 } );
        bool originFound = false, arrowFound = false;
        for( int i = compoundStart; i < compoundList->VtxBuffer.Size; ++i )
        {
            const auto& vertex = compoundList->VtxBuffer[i];
            if( vertex.col != IM_COL32( 255, 185, 80, 255 ) ) continue;
            const float dx = std::abs( vertex.pos.x - center.x );
            originFound |= dx >= 3.0f && dx <= 6.0f && std::abs( vertex.pos.y - center.y ) <= 6.0f;
            arrowFound |= std::abs( vertex.pos.x - tip.x ) < 2.0f && std::abs( vertex.pos.y - tip.y ) < 2.0f;
        }
        if( !originFound || !arrowFound )
        {
            std::fprintf( stderr, "compound body origins and impulse arrows must follow edited transforms\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    const auto compoundCenter = compoundCamera.WorldToScreen( { 1, 2.375f }, compoundViewport, { 850, 650 } );
    bool compoundCenterFound = false;
    for( int i = compoundStart; i < compoundList->VtxBuffer.Size; ++i )
    {
        const auto& vertex = compoundList->VtxBuffer[i];
        compoundCenterFound |= vertex.col == IM_COL32( 255, 90, 180, 255 ) && std::abs( vertex.pos.x - compoundCenter.x ) < 6.0f && std::abs( vertex.pos.y - compoundCenter.y ) < 6.0f;
    }
    if( !compoundCenterFound )
    {
        std::fprintf( stderr, "compound center marker must follow the density-weighted world center\n" );
        std::exit( EXIT_FAILURE );
    }
    ImGui::End();
    ImGui::Render();
    compoundTrial.resetCollisionTrial();
    for( int frame = 0; frame < 9; ++frame )
    {
        if( frame == 3 ) io.AddInputCharactersUTF8( "1" );
        if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Compound trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CompoundTrialImpulse" );
        if( frame == 5 || frame == 8 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        if( frame == 2 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Density" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        compoundView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 )
        {
            const float multiplier = frame == 1 || frame == 7 ? 2.0f : 1.0f;
            for( std::size_t i : { 0u, 2u } )
            {
                const auto body = compoundTrial.getShapes()[i].bodyHandle;
                const auto& physics = compoundTrial.getWorld();
                const float velocity = multiplier * 2.0f / physics.GetBodyMass( body );
                const float spin = -multiplier * 2.0f * physics.GetBodyLocalCenter( body ).x / physics.GetBodyRotationalInertia( body );
                if( std::abs( physics.GetBodyLinearVelocity( body ).y - velocity ) > 1.0e-5f || std::abs( physics.GetBodyAngularVelocity( body ) - spin ) > 1.0e-5f || physics.GetBodyTransform( body ).position.y != 0.0f )
                {
                    std::fprintf( stderr, "compound UI impulse must apply once per body using current component densities\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame >= 4 && ( compoundTrial.getWorld().GetShapeDensity( compoundPart.shapeHandle ) != 1.0f || std::abs( compoundTrial.getWorld().GetBodyLocalCenter( compoundPart.bodyHandle ).x + 0.25f ) > 1.0e-6f ) )
        {
            std::fprintf( stderr, "compound inspector must retain the component density edit and shifted center\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 5 || frame == 8 )
        {
            for( std::size_t i : { 0u, 2u } )
            {
                const auto body = compoundTrial.getShapes()[i].bodyHandle;
                const auto pose = compoundTrial.getWorld().GetBodyTransform( body );
                if( pose.position.x != ( i == 0 ? -3.0f : 3.0f ) || pose.position.y != 0.0f || pose.rotation.s != 0.0f || zonai::LengthSquared( compoundTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f || compoundTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                {
                    std::fprintf( stderr, "compound UI reset must restore initial body origins and rest\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 7 ) compoundTrial.step( 1.0f / 60.0f, 4 );
    }
    auto typeView = createDemoView( demoKind::dynamicKinematicComparison );
    auto& typeTrial = static_cast<rigidBodyDemo&>( *typeView );
    const auto typeTarget = typeTrial.getShapes()[1];
    for( int frame = 0; frame < 60; ++frame )
        typeTrial.step( 1.0f / 60.0f, 4 );
    for( int frame = 0; frame < 6; ++frame )
    {
        if( frame == 1 ) io.AddInputCharactersUTF8( "3" );
        if( frame == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 3 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Body type trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
        if( frame == 0 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Density" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        if( frame >= 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        typeView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame >= 2 && ( typeTrial.getWorld().GetShapeDensity( typeTarget.shapeHandle ) != 3.0f || typeTrial.getWorld().GetBodyMass( typeTarget.bodyHandle ) != 3.0f ) )
        {
            std::fprintf( stderr, "body type inspector lost edited target density across repeated trials\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame >= 3 )
        {
            for( std::size_t i = 0; i < typeTrial.getShapes().size(); ++i )
            {
                const auto body = typeTrial.getShapes()[i].bodyHandle;
                const auto pose = typeTrial.getWorld().GetBodyTransform( body );
                const auto velocity = typeTrial.getWorld().GetBodyLinearVelocity( body );
                if( pose.position.x != ( i % 2 == 0 ? -2.0f : 0.0f ) || pose.position.y != ( i < 2 ? 1.5f : -1.5f ) || pose.rotation.s != 0.0f || velocity.x != ( i % 2 == 0 ? 2.0f : 0.0f ) || velocity.y != 0.0f )
                {
                    std::fprintf( stderr, "body type UI reset failed to restore identical initial poses and prescribed velocities\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 3 || frame == 4 )
        {
            for( int step = 0; step < 60; ++step )
                typeTrial.step( 1.0f / 60.0f, 4 );
            if( std::abs( typeTrial.getWorld().GetBodyLinearVelocity( typeTarget.bodyHandle ).x - 0.5f ) > 0.1f || typeTrial.getWorld().GetBodyLinearVelocity( typeTrial.getShapes()[2].bodyHandle ).x != 2.0f )
            {
                std::fprintf( stderr, "body type UI density edit and reset must change dynamic response while retaining kinematic speed\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto torqueView = createDemoView( demoKind::torqueImpulseComparison );
    auto& torqueTrial = static_cast<rigidBodyDemo&>( *torqueView );
    const auto torqueBox = torqueTrial.getShapes()[0];
    const auto angularImpulseBox = torqueTrial.getShapes()[1];
    for( int frame = 0; frame < 9; ++frame )
    {
        if( frame == 3 ) io.AddInputCharactersUTF8( "2" );
        if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Torque trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "TorqueTrialStart" );
        if( frame == 5 || frame == 8 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        if( frame == 2 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Density" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        torqueView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 )
        {
            const float expected = frame < 6 ? 6.0f : 3.0f;
            if( torqueTrial.getTorqueTrialTime() != 0.0 || torqueTrial.getWorld().GetBodyAngularVelocity( torqueBox.bodyHandle ) != 0.0f || zonai::LengthSquared( torqueTrial.getWorld().GetBodyLinearVelocity( torqueBox.bodyHandle ) ) != 0.0f || std::abs( torqueTrial.getWorld().GetBodyAngularVelocity( angularImpulseBox.bodyHandle ) - expected ) > 1.0e-5f || zonai::LengthSquared( torqueTrial.getWorld().GetBodyLinearVelocity( angularImpulseBox.bodyHandle ) ) != 0.0f || torqueTrial.getWorld().GetBodyTransform( angularImpulseBox.bodyHandle ).rotation.s != 0.0f )
            {
                std::fprintf( stderr, "torque UI start must reset poses time and velocities before applying one current-inertia angular impulse\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        if( frame >= 4 && torqueTrial.getWorld().GetShapeDensity( angularImpulseBox.shapeHandle ) != 2.0f )
        {
            std::fprintf( stderr, "torque UI lost density edit or selection across restart and reset\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 5 || frame == 8 )
        {
            torqueTrial.step( 0.1f, 4 );
            for( std::size_t i = 0; i < torqueTrial.getShapes().size(); ++i )
            {
                const auto body = torqueTrial.getShapes()[i].bodyHandle;
                if( torqueTrial.getTorqueTrialTime() != 0.0 || torqueTrial.getWorld().GetBodyTransform( body ).position.x != ( i == 0 ? -2.5f : 2.5f ) || torqueTrial.getWorld().GetBodyTransform( body ).position.y != 0.0f || zonai::LengthSquared( torqueTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f || torqueTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f || torqueTrial.getWorld().GetBodyTransform( body ).rotation.s != 0.0f )
                {
                    std::fprintf( stderr, "torque UI reset must cancel the active trial without relaunching\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 0 || frame == 7 ) torqueTrial.step( 0.1f, 4 );
    }
    auto forceView = createDemoView( demoKind::forceImpulseComparison );
    auto& forceTrial = static_cast<rigidBodyDemo&>( *forceView );
    const auto forceBox = forceTrial.getShapes()[0];
    const auto impulseBox = forceTrial.getShapes()[1];
    for( int frame = 0; frame < 9; ++frame )
    {
        if( frame == 3 ) io.AddInputCharactersUTF8( "2" );
        if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Force trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "ForceTrialStart" );
        if( frame == 5 || frame == 8 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        if( frame == 2 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Density" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        forceView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 )
        {
            const float expected = frame < 6 ? 2.0f : 1.0f;
            if( forceTrial.getForceTrialTime() != 0.0 || zonai::LengthSquared( forceTrial.getWorld().GetBodyLinearVelocity( forceBox.bodyHandle ) ) != 0.0f || forceTrial.getWorld().GetBodyLinearVelocity( impulseBox.bodyHandle ).y != expected || forceTrial.getWorld().GetBodyTransform( impulseBox.bodyHandle ).position.y != 0.0f )
            {
                std::fprintf( stderr, "force UI start must reset poses time and velocities before applying one current-mass impulse\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        if( frame >= 4 && forceTrial.getWorld().GetShapeDensity( impulseBox.shapeHandle ) != 2.0f )
        {
            std::fprintf( stderr, "force UI lost density edit or selection across restart and reset\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 5 || frame == 8 )
        {
            forceTrial.step( 0.25f, 4 );
            for( std::size_t i = 0; i < forceTrial.getShapes().size(); ++i )
            {
                const auto body = forceTrial.getShapes()[i].bodyHandle;
                if( forceTrial.getForceTrialTime() != 0.0 || forceTrial.getWorld().GetBodyTransform( body ).position.x != ( i == 0 ? -2.5f : 2.5f ) || forceTrial.getWorld().GetBodyTransform( body ).position.y != 0.0f || zonai::LengthSquared( forceTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                {
                    std::fprintf( stderr, "force UI reset must cancel the active trial without relaunching\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 0 || frame == 7 ) forceTrial.step( 0.25f, 4 );
    }
    auto inertiaView = createDemoView( demoKind::rotationalInertiaComparison );
    auto& inertiaTrial = static_cast<rigidBodyDemo&>( *inertiaView );
    const auto inertiaMiddle = inertiaTrial.getShapes()[1];
    for( int frame = 0; frame < 9; ++frame )
    {
        if( frame == 3 ) io.AddInputCharactersUTF8( "2" );
        if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Inertia trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "InertiaTrialImpulse" );
        if( frame == 5 || frame == 8 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        if( frame == 2 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Density" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        inertiaView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 )
        {
            const float multiplier = frame == 1 || frame == 7 ? 2.0f : 1.0f;
            for( std::size_t i = 0; i < inertiaTrial.getShapes().size(); ++i )
            {
                const auto body = inertiaTrial.getShapes()[i].bodyHandle;
                const float expected = multiplier * ( i == 0 ? 6.0f : frame < 6 ? 48.0f / 17.0f : 24.0f / 17.0f );
                if( std::abs( inertiaTrial.getWorld().GetBodyAngularVelocity( body ) - expected ) > 1.0e-5f || inertiaTrial.getWorld().GetBodyTransform( body ).position.y != 0.0f || zonai::LengthSquared( inertiaTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                {
                    std::fprintf( stderr, "inertia UI button failed to apply or accumulate an angular impulse without advancing motion\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame >= 4 && ( inertiaTrial.getWorld().GetShapeDensity( inertiaMiddle.shapeHandle ) != 2.0f || inertiaTrial.getWorld().GetBodyMass( inertiaMiddle.bodyHandle ) != 2.0f ) )
        {
            std::fprintf( stderr, "inertia inspector density edit was lost or changed the wrong body\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 5 || frame == 8 )
        {
            for( std::size_t i = 0; i < inertiaTrial.getShapes().size(); ++i )
            {
                const auto body = inertiaTrial.getShapes()[i].bodyHandle;
                if( inertiaTrial.getWorld().GetBodyTransform( body ).position.x != ( i == 0 ? -2.5f : 2.5f ) || inertiaTrial.getWorld().GetBodyTransform( body ).position.y != 0.0f || zonai::LengthSquared( inertiaTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f || inertiaTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f || inertiaTrial.getWorld().GetBodyTransform( body ).rotation.s != 0.0f )
                {
                    std::fprintf( stderr, "inertia UI reset did not restore initial pose and zero velocity\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 7 ) inertiaTrial.step( 1.0f / 60.0f, 4 );
    }
    auto massView = createDemoView( demoKind::massImpulseComparison );
    auto& massTrial = static_cast<rigidBodyDemo&>( *massView );
    const auto massMiddle = massTrial.getShapes()[1];
    for( int frame = 0; frame < 9; ++frame )
    {
        if( frame == 3 ) io.AddInputCharactersUTF8( "4" );
        if( frame == 4 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 5 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Mass trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "MassTrialImpulse" );
        if( frame == 5 || frame == 8 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        if( frame == 2 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Density" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        massView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame == 0 || frame == 1 || frame == 6 || frame == 7 )
        {
            const float multiplier = frame == 1 || frame == 7 ? 2.0f : 1.0f;
            for( std::size_t i = 0; i < massTrial.getShapes().size(); ++i )
            {
                const auto body = massTrial.getShapes()[i].bodyHandle;
                const float expected = multiplier * ( i == 0 ? 2.0f : i == 1 && frame < 6 ? 1.0f : 0.5f );
                if( massTrial.getWorld().GetBodyLinearVelocity( body ).y != expected || massTrial.getWorld().GetBodyTransform( body ).position.y != 0.0f || massTrial.getWorld().GetBodyAngularVelocity( body ) != 0.0f )
                {
                    std::fprintf( stderr, "mass UI button failed to apply or accumulate a center impulse without advancing motion\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame >= 4 && ( massTrial.getWorld().GetShapeDensity( massMiddle.shapeHandle ) != 4.0f || massTrial.getWorld().GetBodyMass( massMiddle.bodyHandle ) != 4.0f ) )
        {
            std::fprintf( stderr, "mass inspector density edit was lost or changed the wrong body\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 5 || frame == 8 )
        {
            for( std::size_t i = 0; i < massTrial.getShapes().size(); ++i )
            {
                const auto body = massTrial.getShapes()[i].bodyHandle;
                if( massTrial.getWorld().GetBodyTransform( body ).position.x != -3.0f + 3.0f * static_cast<float>( i ) || massTrial.getWorld().GetBodyTransform( body ).position.y != 0.0f || zonai::LengthSquared( massTrial.getWorld().GetBodyLinearVelocity( body ) ) != 0.0f )
                {
                    std::fprintf( stderr, "mass UI reset did not restore initial pose and zero velocity\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 7 ) massTrial.step( 1.0f / 60.0f, 4 );
    }
    for( demoKind kind : { demoKind::collisionComparison, demoKind::frictionComparison, demoKind::restitutionComparison, demoKind::boxStack, demoKind::ccdComparison, demoKind::collisionFilterComparison, demoKind::sensorWallComparison, demoKind::sleepWakeComparison, demoKind::dampingComparison, demoKind::gravityScaleComparison, demoKind::massImpulseComparison, demoKind::impulsePointComparison, demoKind::rotationalInertiaComparison, demoKind::forceImpulseComparison, demoKind::torqueImpulseComparison, demoKind::dynamicKinematicComparison, demoKind::compoundCenterComparison, demoKind::fastRotationComparison, demoKind::maximumLinearSpeedComparison, demoKind::frictionMixingComparison, demoKind::restitutionMixingComparison, demoKind::lowSpeedRestitutionComparison, demoKind::collisionGroupComparison, demoKind::movingPlatformFrictionComparison, demoKind::rollingSlidingFrictionComparison, demoKind::frictionForceComparison, demoKind::obliqueImpactFrictionComparison, demoKind::collisionMassRatioComparison, demoKind::centralOffsetImpactComparison, demoKind::springDampingComparison, demoKind::distanceMotorForceComparison, demoKind::distanceLimitMotorComparison, demoKind::revoluteMotorTorqueComparison } )
    {
        auto trialView = createDemoView( kind );
        auto& trial = static_cast<rigidBodyDemo&>( *trialView );
        const auto selected = trial.getShapes().back();
        const auto initial = trial.getWorld().GetBodyTransform( selected.bodyHandle );
        for( int frame = 0; frame <= 10; ++frame )
        {
            if( frame == 4 ) io.AddInputCharactersUTF8( "0.42" );
            if( frame == 9 ) io.AddInputCharactersUTF8( "0.73" );
            if( frame == 5 || frame == 10 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 6 )
            {
                io.AddKeyEvent( ImGuiKey_Enter, false );
                trial.getWorld().SetBodyTransform( selected.bodyHandle, { { 30, 40 }, {} } );
            }
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Collision trial inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision material" ), 1 );
            if( frame == 1 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
            if( frame == 2 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( selected.label );
            if( frame == 3 || frame == 8 )
            {
                GImGui->NavActivateId = ImGui::GetID( "Friction" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame == 7 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            trialView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame == 5 || frame == 7 || frame == 10 )
            {
                const float expected = frame == 10 ? 0.73f : 0.42f;
                if( std::abs( trial.getWorld().GetShapeFriction( selected.shapeHandle ) - expected ) > 1.0e-5f )
                {
                    std::fprintf( stderr, "collision trial inspector lost edited material or selection\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
            if( frame == 7 && zonai::Length( trial.getWorld().GetBodyTransform( selected.bodyHandle ).position - initial.position ) > 1.0e-5f )
            {
                std::fprintf( stderr, "collision trial reset button did not restore selected body\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        io.AddKeyEvent( ImGuiKey_Enter, false );
    }
    auto filterView = createDemoView( demoKind::collisionFilterComparison );
    auto& filterTrial = static_cast<rigidBodyDemo&>( *filterView );
    const auto upperRight = filterTrial.getShapes()[1];
    const auto lowerLeft = filterTrial.getShapes()[2].bodyHandle;
    for( int frame = 0; frame < 90; ++frame )
        filterTrial.step( 1.0f / 60.0f, 4 );
    for( int phase = 0; phase < 2; ++phase )
    {
        for( int frame = 0; frame < 4; ++frame )
        {
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Filter trial inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Collision mask" ), 1 );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "ObjectCollisionOverrides" ), 1 );
            if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Collision partners" );
            if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( "레이어 01##Mask" );
            if( frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            filterView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame == 2 ) ImGui::ClosePopupToLevel( 0, true );
        }
        const auto mask = filterTrial.getWorld().GetShapeFilter( upperRight.shapeHandle ).maskBits;
        if( mask != ( phase == 0 ? ~std::uint64_t{ 1 } : ~std::uint64_t{ 0 } ) || filterTrial.getWorld().GetBodyTransform( upperRight.bodyHandle ).position.x != 2.0f || filterTrial.getWorld().GetBodyLinearVelocity( upperRight.bodyHandle ).x != -2.0f )
        {
            std::fprintf( stderr, "filter inspector or reset lost edited mask selection or initial conditions\n" );
            std::exit( EXIT_FAILURE );
        }
        for( int frame = 0; frame < 90; ++frame )
            filterTrial.step( 1.0f / 60.0f, 4 );
        const float velocity = filterTrial.getWorld().GetBodyLinearVelocity( upperRight.bodyHandle ).x;
        if( ( phase == 0 ? velocity >= 0 : velocity <= 0 ) || filterTrial.getWorld().GetBodyTransform( lowerLeft ).position.x <= 0 )
        {
            std::fprintf( stderr, "inspector mask edits did not switch repeated upper collision behavior while preserving lower exclusion\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    auto sensorView = createDemoView( demoKind::sensorWallComparison );
    auto& sensorTrial = static_cast<rigidBodyDemo&>( *sensorView );
    const auto sensorVisual = sensorTrial.getShapes()[2];
    const auto sensorVisitor = sensorTrial.getShapes()[3];
    for( int frame = 0; frame < 90; ++frame )
        sensorTrial.step( 1.0f / 60.0f, 4 );
    for( int frame = 0; frame < 7; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Sensor trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Sensors" ), 1 );
        if( frame == 0 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Object" );
        if( frame == 1 && !GImGui->OpenPopupStack.empty() ) GImGui->NavActivateId = GImGui->NavActivateDownId = GImGui->OpenPopupStack.back().Window->GetID( sensorVisual.label );
        if( frame == 2 || frame == 4 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Sensor events" );
        if( frame == 3 || frame == 6 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        sensorView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( frame >= 2 && sensorTrial.getWorld().AreShapeSensorEventsEnabled( sensorVisual.shapeHandle ) != ( frame >= 4 ) )
        {
            std::fprintf( stderr, "sensor inspector toggle or trial reset lost event settings or selection\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 2 )
        {
            sensorTrial.step( 1.0f / 60.0f, 4 );
            if( sensorTrial.getWorld().GetShapeSensorCapacity( sensorVisual.shapeHandle ) != 0 || sensorTrial.getWorld().GetBodyLinearVelocity( sensorVisitor.bodyHandle ).x != 2.0f )
            {
                std::fprintf( stderr, "sensor UI event opt-out failed to stop detection without blocking motion\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        if( frame == 3 || frame == 6 )
        {
            if( sensorTrial.getWorld().GetShapeSensorCapacity( sensorVisual.shapeHandle ) != 0 || sensorTrial.getWorld().GetBodyTransform( sensorVisitor.bodyHandle ).position.x != -3.0f || sensorTrial.getWorld().GetBodyLinearVelocity( sensorVisitor.bodyHandle ).x != 2.0f )
            {
                std::fprintf( stderr, "sensor UI trial reset retained stale overlap or failed to restore launch conditions\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        if( frame == 5 )
        {
            for( int step = 0; step < 90; ++step )
                sensorTrial.step( 1.0f / 60.0f, 4 );
            if( sensorTrial.getWorld().GetShapeSensorCapacity( sensorVisual.shapeHandle ) != 1 )
            {
                std::fprintf( stderr, "sensor UI event opt-in failed to detect repeated entry\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    auto sleepView = createDemoView( demoKind::sleepWakeComparison );
    auto& sleepTrial = static_cast<rigidBodyDemo&>( *sleepView );
    const auto sleepLeft = sleepTrial.getShapes()[1].bodyHandle;
    const auto sleepRight = sleepTrial.getShapes()[2].bodyHandle;
    for( int frame = 0; frame < 240; ++frame )
        sleepTrial.step( 1.0f / 60.0f, 4 );
    for( int frame = 0; frame < 9; ++frame )
    {
        if( frame == 6 ) io.AddInputCharactersUTF8( "0.2" );
        if( frame == 7 ) io.AddKeyEvent( ImGuiKey_Enter, true );
        if( frame == 8 ) io.AddKeyEvent( ImGuiKey_Enter, false );
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "Sleep trial inspector" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Sleep and CCD" ), 1 );
        if( frame == 0 || frame == 2 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Body sleep" );
        if( frame == 1 || frame == 3 || frame == 8 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        if( frame == 4 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Awake" );
        if( frame == 5 )
        {
            GImGui->NavActivateId = ImGui::GetID( "Sleep threshold" );
            GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
        }
        sleepView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( sleepTrial.getWorld().IsBodySleepEnabled( sleepLeft ) != ( frame >= 2 ) || sleepTrial.getWorld().IsBodySleepEnabled( sleepRight ) )
        {
            std::fprintf( stderr, "sleep inspector toggle or reset lost edited permission or selection\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 1 || frame == 3 || frame == 8 )
        {
            if( !sleepTrial.getWorld().IsBodyAwake( sleepLeft ) || !sleepTrial.getWorld().IsBodyAwake( sleepRight ) || sleepTrial.getWorld().GetBodyTransform( sleepLeft ).position.y != 1.5f || zonai::LengthSquared( sleepTrial.getWorld().GetBodyLinearVelocity( sleepLeft ) ) != 0.0f )
            {
                std::fprintf( stderr, "sleep UI reset failed to wake and restore both boxes\n" );
                std::exit( EXIT_FAILURE );
            }
            if( frame != 8 )
            {
                for( int step = 0; step < 240; ++step )
                    sleepTrial.step( 1.0f / 60.0f, 4 );
                if( sleepTrial.getWorld().IsBodyAwake( sleepLeft ) != ( frame == 1 ) || !sleepTrial.getWorld().IsBodyAwake( sleepRight ) )
                {
                    std::fprintf( stderr, "sleep inspector edits failed to change the repeated trial result\n" );
                    std::exit( EXIT_FAILURE );
                }
            }
        }
        if( frame == 4 && !sleepTrial.getWorld().IsBodyAwake( sleepLeft ) )
        {
            std::fprintf( stderr, "Awake inspector failed to wake the sleeping box\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame >= 7 && std::abs( sleepTrial.getWorld().GetBodySleepThreshold( sleepLeft ) - 0.2f ) > 1.0e-5f )
        {
            std::fprintf( stderr, "sleep threshold inspector edit was not retained by reset\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    for( int field = 0; field < 2; ++field )
    {
        auto dampingView = createDemoView( demoKind::dampingComparison );
        auto& dampingTrial = static_cast<rigidBodyDemo&>( *dampingView );
        const auto middle = dampingTrial.getShapes()[1].bodyHandle;
        for( int step = 0; step < 60; ++step )
            dampingTrial.step( 1.0f / 60.0f, 4 );
        const float speed = zonai::Length( dampingTrial.getWorld().GetBodyLinearVelocity( middle ) );
        const float spin = dampingTrial.getWorld().GetBodyAngularVelocity( middle );
        for( int frame = 0; frame < 4; ++frame )
        {
            if( frame == 1 ) io.AddInputCharactersUTF8( "2.0" );
            if( frame == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 3 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Damping trial inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
            if( frame == 0 )
            {
                GImGui->NavActivateId = ImGui::GetID( field == 0 ? "Linear damping" : "Angular damping" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
            }
            if( frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            dampingView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 2 && ( dampingTrial.getWorld().GetBodyLinearDamping( middle ) != ( field == 0 ? 2.0f : 0.5f ) || dampingTrial.getWorld().GetBodyAngularDamping( middle ) != ( field == 1 ? 2.0f : 0.5f ) ) )
            {
                std::fprintf( stderr, "damping inspector edit changed the wrong coefficient or was lost by reset\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        const auto pose = dampingTrial.getWorld().GetBodyTransform( middle );
        if( zonai::LengthSquared( pose.position ) != 0.0f || pose.rotation.c != 1.0f || pose.rotation.s != 0.0f || dampingTrial.getWorld().GetBodyLinearVelocity( middle ).y != 2.0f || dampingTrial.getWorld().GetBodyAngularVelocity( middle ) != 2.0f )
        {
            std::fprintf( stderr, "damping UI reset did not restore initial transforms and velocities\n" );
            std::exit( EXIT_FAILURE );
        }
        for( int step = 0; step < 60; ++step )
            dampingTrial.step( 1.0f / 60.0f, 4 );
        const float editedSpeed = zonai::Length( dampingTrial.getWorld().GetBodyLinearVelocity( middle ) );
        const float editedSpin = dampingTrial.getWorld().GetBodyAngularVelocity( middle );
        if( field == 0 ? !( editedSpeed < speed - 0.4f && std::abs( editedSpin - spin ) < 1.0e-6f ) : !( std::abs( editedSpeed - speed ) < 1.0e-6f && editedSpin < spin - 0.4f ) )
        {
            std::fprintf( stderr, "damping inspector edit failed to change only the corresponding motion on the repeated trial\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    auto gravityView = createDemoView( demoKind::gravityScaleComparison );
    auto& gravityTrial = static_cast<rigidBodyDemo&>( *gravityView );
    const auto gravityMiddle = gravityTrial.getShapes()[1].bodyHandle;
    const auto gravityLast = gravityTrial.getShapes()[2].bodyHandle;
    for( int step = 0; step < 60; ++step )
        gravityTrial.step( 1.0f / 60.0f, 4 );
    for( int field = 0; field < 2; ++field )
    {
        for( int frame = 0; frame < 4; ++frame )
        {
            if( frame == 1 ) io.AddInputCharactersUTF8( field == 0 ? "-1" : "4" );
            if( frame == 2 ) io.AddKeyEvent( ImGuiKey_Enter, true );
            if( frame == 3 ) io.AddKeyEvent( ImGuiKey_Enter, false );
            ImGui::NewFrame();
            ImGui::SetNextWindowPos( { 0, 0 } );
            ImGui::SetNextWindowSize( { 340, 4000 } );
            ImGui::Begin( "Gravity trial inspector" );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "Physics quantities" ), 1 );
            ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WorldSettings" ), 1 );
            if( frame == 0 )
            {
                if( field == 1 )
                {
                    ImGui::PushID( "중력###Gravity" );
                    ImGui::PushID( 1 );
                }
                GImGui->NavActivateId = ImGui::GetID( field == 0 ? "Gravity scale" : "" );
                GImGui->NavActivateFlags = ImGuiActivateFlags_PreferInput;
                if( field == 1 )
                {
                    ImGui::PopID();
                    ImGui::PopID();
                }
            }
            if( frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
            gravityView->drawControls();
            ImGui::End();
            ImGui::Render();
            if( frame >= 2 && ( gravityTrial.getWorld().GetBodyGravityScale( gravityMiddle ) != -1.0f || gravityTrial.getWorld().GetGravity().x != 0.0f || gravityTrial.getWorld().GetGravity().y != ( field == 0 ? -10.0f : 4.0f ) ) )
            {
                std::fprintf( stderr, "gravity inspector or reset lost edited scale world gravity or selection\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        if( gravityTrial.getWorld().GetBodyTransform( gravityMiddle ).position.y != 3.0f || zonai::LengthSquared( gravityTrial.getWorld().GetBodyLinearVelocity( gravityMiddle ) ) != 0.0f )
        {
            std::fprintf( stderr, "gravity UI reset did not restore initial height and zero velocity\n" );
            std::exit( EXIT_FAILURE );
        }
        for( int step = 0; step < 60; ++step )
            gravityTrial.step( 1.0f / 60.0f, 4 );
        const float expected = field == 0 ? 10.0f : -4.0f;
        if( std::abs( gravityTrial.getWorld().GetBodyLinearVelocity( gravityMiddle ).y - expected ) > 2.0e-4f || std::abs( gravityTrial.getWorld().GetBodyLinearVelocity( gravityLast ).y + expected ) > 2.0e-4f || gravityTrial.getWorld().GetBodyTransform( gravityTrial.getShapes()[0].bodyHandle ).position.y != 3.0f )
        {
            std::fprintf( stderr, "gravity inspector edits failed to reverse the repeated motion while preserving the zero-scale box\n" );
            std::exit( EXIT_FAILURE );
        }
    }
    auto ccdView = createDemoView( demoKind::ccdComparison );
    auto& ccd = static_cast<rigidBodyDemo&>( *ccdView );
    const auto projectile = ccd.getImpulseBody();
    const auto ccdShape = ccd.getShapes()[1].shapeHandle;
    ccd.getWorld().SetShapeRestitution( ccdShape, 0.6f );
    for( int frame = 0; frame < 6; ++frame )
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos( { 0, 0 } );
        ImGui::SetNextWindowSize( { 340, 4000 } );
        ImGui::Begin( "CCD trial actions" );
        ImGui::GetStateStorage()->SetInt( ImGui::GetID( "WorldSettings" ), 1 );
        if( frame == 0 || frame == 3 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "Continuous collision" );
        if( frame == 1 || frame == 4 ) GImGui->NavActivateId = GImGui->NavActivateDownId = ImGui::GetID( "CollisionTrialReset" );
        ccdView->drawControls();
        ImGui::End();
        ImGui::Render();
        if( ccd.getWorld().IsContinuousEnabled() != ( frame >= 3 ) || ccd.getWorld().GetShapeRestitution( ccdShape ) != 0.6f )
        {
            std::fprintf( stderr, "CCD UI toggle or reset lost edited settings\n" );
            std::exit( EXIT_FAILURE );
        }
        if( frame == 1 || frame == 4 )
        {
            if( ccd.getWorld().GetBodyTransform( projectile ).position.x != -0.8f || ccd.getWorld().GetBodyLinearVelocity( projectile ).x != 100.0f )
            {
                std::fprintf( stderr, "CCD UI reset did not restore launch conditions\n" );
                std::exit( EXIT_FAILURE );
            }
        }
        if( frame == 0 || frame == 2 || frame == 5 )
        {
            ccd.step( 1.0f / 60.0f, 4 );
            const bool enabled = frame == 5;
            if( ccd.getWorld().HadBodyTimeOfImpact( projectile ) != enabled || ( enabled ? ccd.getWorld().GetBodyTransform( projectile ).position.x >= -0.1f : ccd.getWorld().GetBodyTransform( projectile ).position.x <= 0.15f ) )
            {
                std::fprintf( stderr, "CCD UI settings did not change repeated collision behavior\n" );
                std::exit( EXIT_FAILURE );
            }
        }
    }
    ImGui::DestroyContext();
}
