#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"

#include "dynamics/world.h"
#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hwnd,
    UINT message,
    WPARAM wParam,
    LPARAM lParam
);

using Microsoft::WRL::ComPtr;

namespace
{

using namespace zonai;
using namespace zonai::sandbox;

struct d3d11Context
{
    ComPtr<ID3D11Device> device;
    ComPtr<ID3D11DeviceContext> deviceContext;
    ComPtr<IDXGISwapChain> swapChain;
    ComPtr<ID3D11RenderTargetView> renderTargetView;
};

d3d11Context gD3d;

bool CreateRenderTarget()
{
    ComPtr<ID3D11Texture2D> backBuffer;

    HRESULT result =
        gD3d.swapChain->GetBuffer(
            0,
            IID_PPV_ARGS( &backBuffer )
        );

    if( FAILED( result ) )
    {
        return false;
    }

    result =
        gD3d.device->CreateRenderTargetView(
            backBuffer.Get(),
            nullptr,
            &gD3d.renderTargetView
        );

    return SUCCEEDED( result );
}

void DestroyRenderTarget()
{
    gD3d.renderTargetView.Reset();
}

LRESULT CALLBACK WndProc(
    HWND hwnd,
    UINT message,
    WPARAM wParam,
    LPARAM lParam )
{
    if( ImGui_ImplWin32_WndProcHandler(
        hwnd,
        message,
        wParam,
        lParam ) )
    {
        return true;
    }

    switch( message )
    {
    case WM_DESTROY:
        PostQuitMessage( 0 );
        return 0;

    case WM_SIZE:
        if( gD3d.swapChain &&
            wParam != SIZE_MINIMIZED )
        {
            DestroyRenderTarget();

            const UINT width = LOWORD( lParam );
            const UINT height = HIWORD( lParam );

            gD3d.swapChain->ResizeBuffers(
                0,
                width,
                height,
                DXGI_FORMAT_UNKNOWN,
                0
            );

            CreateRenderTarget();
        }

        return 0;
    }

    return DefWindowProcW(
        hwnd,
        message,
        wParam,
        lParam
    );
}

struct visualShape
{
    bodyId bodyHandle{};
    shapeId shapeHandle{};
    const char* label = "";
};

struct visualScene
{
    world world{};
    std::vector<visualShape> shapes;

    bodyId impulseBody{};
    bodyId torqueBody{};
    bodyId kinematicBody{};

    visualScene()
    {
        world.SetGravity( { 0.0f, -10.0f } );
        shapes.reserve( 8 );

        // 넓은 정적 바닥.
        const bodyId ground =
            world.CreateBody(
                bodyType::Static,
                {
                    { 0.0f, -4.0f },
                    {}
                }
            );

        const shapeId groundShape =
            world.CreateShape(
                ground,
                MakeBox( { 8.0f, 0.5f } )
            );

        shapes.push_back(
            { ground, groundShape, "Ground [Static]" }
        );

        // 기울어진 정적 ramp.
        const bodyId ramp =
            world.CreateBody(
                bodyType::Static,
                {
                    { 4.0f, -1.6f },
                    rot2::FromRadians( 0.22f )
                }
            );

        const shapeId rampShape =
            world.CreateShape(
                ramp,
                segment2
                {
                    { -2.0f, 0.0f },
                    {  2.0f, 0.0f }
                }
            );

        shapes.push_back(
            { ramp, rampShape, "Ramp [Static]" }
        );

        // impulse 버튼으로 직접 밀어볼 Dynamic Circle.
        impulseBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { -3.2f, 3.5f },
                    {}
                }
            );

        const shapeId circleShape =
            world.CreateShape(
                impulseBody,
                circle2{ {}, 0.65f }
            );

        shapes.push_back(
            { impulseBody, circleShape, "Circle [Dynamic]" }
        );

        // COM이 origin과 일치하는 Dynamic Box.
        torqueBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 0.0f, 5.0f },
                    rot2::FromRadians( 0.15f )
                }
            );

        const shapeId boxShape =
            world.CreateShape(
                torqueBody,
                MakeBox( { 0.75f, 0.55f } )
            );

        shapes.push_back(
            { torqueBody, boxShape, "Box [Dynamic]" }
        );

        // 길쭉한 Dynamic Capsule.
        const bodyId capsuleBody =
            world.CreateBody(
                bodyType::Dynamic,
                {
                    { 3.0f, 4.0f },
                    rot2::FromRadians( -0.25f )
                }
            );

        const shapeId capsuleShape =
            world.CreateShape(
                capsuleBody,
                capsule2
                {
                    { -0.8f, 0.0f },
                    {  0.8f, 0.0f },
                    0.35f
                }
            );

        shapes.push_back(
            { capsuleBody, capsuleShape, "Capsule [Dynamic]" }
        );

        // gravity와 force의 영향을 받지 않고 지정한 velocity로만 움직이는 Kinematic Body.
        kinematicBody =
            world.CreateBody(
                bodyType::Kinematic,
                {
                    { -5.5f, -1.5f },
                    {}
                }
            );

        const shapeId kinematicShape =
            world.CreateShape(
                kinematicBody,
                MakeBox( { 0.8f, 0.3f } )
            );

        world.SetBodyLinearVelocity(
            kinematicBody,
            { 1.25f, 0.0f }
        );

        shapes.push_back(
            {
                kinematicBody,
                kinematicShape,
                "Platform [Kinematic]"
            }
        );
    }
};

struct shapeColors
{
    ImU32 outline = 0;
    ImU32 fill = 0;
};

shapeColors GetShapeColors( bodyType type )
{
    switch( type )
    {
    case bodyType::Static:
        return
        {
            IM_COL32( 120, 220, 140, 255 ),
            IM_COL32( 120, 220, 140, 55 )
        };

    case bodyType::Kinematic:
        return
        {
            IM_COL32( 245, 205, 90, 255 ),
            IM_COL32( 245, 205, 90, 60 )
        };

    case bodyType::Dynamic:
        return
        {
            IM_COL32( 90, 190, 255, 255 ),
            IM_COL32( 90, 190, 255, 70 )
        };

    default:
        return
        {
            IM_COL32( 230, 230, 230, 255 ),
            IM_COL32( 230, 230, 230, 50 )
        };
    }
}

void RefreshContacts(
    visualScene& scene,
    std::vector<contactData>& contacts )
{
    contacts.clear();

    scene.world.UpdateCollisions(
        [&]( const contactData& data )
        {
            contacts.push_back( data );
        }
    );
}

vec2 GetWorldCenter(
    const world& world,
    bodyId bodyId )
{
    return TransformPoint(
        world.GetBodyTransform( bodyId ),
        world.GetBodyLocalCenter( bodyId )
    );
}

} // namespace

int main()
{
    using namespace zonai;
    using namespace zonai::sandbox;

    HINSTANCE instance =
        GetModuleHandleW( nullptr );

    const wchar_t* className =
        L"ZonaiSandboxWindow";

    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = WndProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = className;

    if( !RegisterClassW( &windowClass ) )
    {
        return 1;
    }

    HWND hwnd =
        CreateWindowExW(
            0,
            className,
            L"Zonai Physics Sandbox",
            WS_OVERLAPPEDWINDOW,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            1280,
            720,
            nullptr,
            nullptr,
            instance,
            nullptr
        );

    if( !hwnd )
    {
        return 1;
    }

    ShowWindow( hwnd, SW_SHOW );

    // ---------------------------------------------------------
    // D3D11
    // ---------------------------------------------------------

    DXGI_SWAP_CHAIN_DESC swapChainDesc{};
    swapChainDesc.BufferCount = 2;
    swapChainDesc.BufferDesc.Width = 0;
    swapChainDesc.BufferDesc.Height = 0;
    swapChainDesc.BufferDesc.Format =
        DXGI_FORMAT_R8G8B8A8_UNORM;
    swapChainDesc.BufferUsage =
        DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapChainDesc.OutputWindow = hwnd;
    swapChainDesc.SampleDesc.Count = 1;
    swapChainDesc.Windowed = TRUE;
    swapChainDesc.SwapEffect =
        DXGI_SWAP_EFFECT_DISCARD;

    D3D_FEATURE_LEVEL featureLevel{};

    const HRESULT result =
        D3D11CreateDeviceAndSwapChain(
            nullptr,
            D3D_DRIVER_TYPE_HARDWARE,
            nullptr,
            0,
            nullptr,
            0,
            D3D11_SDK_VERSION,
            &swapChainDesc,
            &gD3d.swapChain,
            &gD3d.device,
            &featureLevel,
            &gD3d.deviceContext
        );

    if( FAILED( result ) )
    {
        return 1;
    }

    if( !CreateRenderTarget() )
    {
        return 1;
    }

    // ---------------------------------------------------------
    // ImGui
    // ---------------------------------------------------------

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();

    ImGui::StyleColorsDark();

    if( !ImGui_ImplWin32_Init( hwnd ) )
    {
        return 1;
    }

    if( !ImGui_ImplDX11_Init(
        gD3d.device.Get(),
        gD3d.deviceContext.Get() ) )
    {
        return 1;
    }

    // ---------------------------------------------------------
    // Physics visual test
    // ---------------------------------------------------------

    debugCamera camera{};
    camera.center = { 0.0f, 0.5f };
    camera.pixelsPerMeter = 55.0f;

    std::unique_ptr<visualScene> scene =
        std::make_unique<visualScene>();

    std::vector<contactData> contacts;
    contacts.reserve( 16 );

    bool playing = false;
    bool showGrid = true;
    bool showAABBs = true;
    bool showContacts = true;
    bool showCOM = true;
    bool showLabels = true;

    bool showDynamicTree = false;
    bool showKinematicTree = false;
    bool showStaticTree = false;
    bool showTreeLeaves = true;
    bool showTreeInternal = true;
    bool showTreeLabels = false;

    constexpr float FIXED_TIME_STEP =
        1.0f / 60.0f;

    constexpr int MAX_STEPS_PER_FRAME = 8;

    float accumulator = 0.0f;
    std::uint64_t stepCount = 0;

    RefreshContacts(
        *scene,
        contacts
    );

    // ---------------------------------------------------------
    // Main loop
    // ---------------------------------------------------------

    MSG message{};
    bool running = true;

    while( running )
    {
        while( PeekMessageW(
            &message,
            nullptr,
            0,
            0,
            PM_REMOVE ) )
        {
            if( message.message == WM_QUIT )
            {
                running = false;
                break;
            }

            TranslateMessage( &message );
            DispatchMessageW( &message );
        }

        if( !running )
        {
            break;
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // 렌더링 FPS와 물리 simulation 주기를 분리하기 위해 fixed time step을 사용함.
        if( playing )
        {
            accumulator +=
                std::min( io.DeltaTime, 0.25f );

            int frameStepCount = 0;

            while( accumulator >= FIXED_TIME_STEP &&
                   frameStepCount < MAX_STEPS_PER_FRAME )
            {
                scene->world.Step(
                    FIXED_TIME_STEP
                );

                accumulator -= FIXED_TIME_STEP;
                ++frameStepCount;
                ++stepCount;
            }

            // 너무 긴 frame 뒤에 끝없이 따라잡는 상황은 방지함.
            if( frameStepCount == MAX_STEPS_PER_FRAME )
            {
                accumulator = 0.0f;
            }

            RefreshContacts(
                *scene,
                contacts
            );
        }

        const ImGuiViewport* viewport =
            ImGui::GetMainViewport();

        ImGui::SetNextWindowPos(
            viewport->WorkPos
        );
        ImGui::SetNextWindowSize(
            viewport->WorkSize
        );

        constexpr ImGuiWindowFlags WINDOW_FLAGS =
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize;

        ImGui::Begin(
            "Zonai Physics Sandbox",
            nullptr,
            WINDOW_FLAGS
        );

        // -----------------------------------------------------
        // Controls
        // -----------------------------------------------------

        ImGui::BeginChild(
            "Controls",
            ImVec2( 270.0f, 0.0f ),
            true
        );

        ImGui::TextUnformatted(
            "world Simulation"
        );
        ImGui::Separator();

        if( ImGui::Button(
            playing ? "Pause" : "Play",
            ImVec2( 78.0f, 0.0f ) ) )
        {
            playing = !playing;
        }

        ImGui::SameLine();

        if( ImGui::Button(
            "Step",
            ImVec2( 72.0f, 0.0f ) ) )
        {
            scene->world.Step(
                FIXED_TIME_STEP
            );

            ++stepCount;
            accumulator = 0.0f;

            RefreshContacts(
                *scene,
                contacts
            );
        }

        ImGui::SameLine();

        if( ImGui::Button(
            "Reset",
            ImVec2( 72.0f, 0.0f ) ) )
        {
            scene =
                std::make_unique<visualScene>();

            contacts.clear();
            accumulator = 0.0f;
            stepCount = 0;
            playing = false;

            RefreshContacts(
                *scene,
                contacts
            );
        }

        ImGui::Spacing();

        vec2 gravity =
            scene->world.GetGravity();

        float gravityValues[2]
        {
            gravity.x,
            gravity.y
        };

        if( ImGui::DragFloat2(
            "Gravity",
            gravityValues,
            0.1f,
            -30.0f,
            30.0f,
            "%.2f" ) )
        {
            scene->world.SetGravity(
                {
                    gravityValues[0],
                    gravityValues[1]
                }
            );
        }

        ImGui::Text(
            "Fixed dt: %.5f s",
            FIXED_TIME_STEP
        );
        ImGui::Text(
            "Steps: %llu",
            static_cast<unsigned long long>( stepCount )
        );

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextUnformatted(
            "Impulse Test"
        );

        const float circleMass =
            scene->world.GetBodyMass(
                scene->impulseBody
            );

        if( ImGui::Button(
            "Jump impulse",
            ImVec2( -1.0f, 0.0f ) ) )
        {
            // DeltaV = J / M = 5 m/s
            scene->world.ApplyLinearImpulseToCenter(
                scene->impulseBody,
                { 0.0f, circleMass * 5.0f }
            );
        }

        if( ImGui::Button(
            "Off-center kick",
            ImVec2( -1.0f, 0.0f ) ) )
        {
            const vec2 center =
                GetWorldCenter(
                    scene->world,
                    scene->impulseBody
                );

            // COM 위쪽을 오른쪽으로 밀어 translation + rotation을 동시에 확인함.
            scene->world.ApplyLinearImpulse(
                scene->impulseBody,
                { circleMass * 4.0f, 0.0f },
                center + vec2{ 0.0f, 0.8f }
            );
        }

        const float boxInertia =
            scene->world.GetBodyRotationalInertia(
                scene->torqueBody
            );

        if( ImGui::Button(
            "Spin box",
            ImVec2( -1.0f, 0.0f ) ) )
        {
            // DeltaW = L / I = 3 rad/s
            scene->world.ApplyAngularImpulse(
                scene->torqueBody,
                boxInertia * 3.0f
            );
        }

        const vec2 circleVelocity =
            scene->world.GetBodyLinearVelocity(
                scene->impulseBody
            );

        const float circleAngularVelocity =
            scene->world.GetBodyAngularVelocity(
                scene->impulseBody
            );

        ImGui::Text(
            "Circle v: (%.2f, %.2f)",
            circleVelocity.x,
            circleVelocity.y
        );
        ImGui::Text(
            "Circle w: %.2f rad/s",
            circleAngularVelocity
        );

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextUnformatted(
            "Debug Draw"
        );

        ImGui::Checkbox(
            "Grid / Axis",
            &showGrid
        );
        ImGui::Checkbox(
            "shape AABBs",
            &showAABBs
        );
        ImGui::Checkbox(
            "Contact points",
            &showContacts
        );
        ImGui::Checkbox(
            "Center of mass",
            &showCOM
        );
        ImGui::Checkbox(
            "Labels",
            &showLabels
        );

        ImGui::Spacing();
        ImGui::TextUnformatted(
            "AABB Tree"
        );

        ImGui::Checkbox(
            "Dynamic Tree",
            &showDynamicTree
        );
        ImGui::Checkbox(
            "Kinematic Tree",
            &showKinematicTree
        );
        ImGui::Checkbox(
            "Static Tree",
            &showStaticTree
        );

        ImGui::Checkbox(
            "Tree Leaves",
            &showTreeLeaves
        );
        ImGui::Checkbox(
            "Tree Internal",
            &showTreeInternal
        );
        ImGui::Checkbox(
            "Tree Labels",
            &showTreeLabels
        );

        const broadPhase& broadPhase =
            scene->world.GetBroadPhase();

        const dynamicTree& dynamicTree =
            broadPhase.GetTree(
                bodyType::Dynamic
            );

        const dynamicTree& kinematicTree =
            broadPhase.GetTree(
                bodyType::Kinematic
            );

        const dynamicTree& staticTree =
            broadPhase.GetTree(
                bodyType::Static
            );

        ImGui::Spacing();

        ImGui::Text(
            "Bodies: %zu",
            scene->world.GetBodyCount()
        );
        ImGui::Text(
            "Shapes: %zu",
            scene->world.GetShapeCount()
        );
        ImGui::Text(
            "Persistent contacts: %zu",
            scene->world.GetContactCount()
        );
        ImGui::Text(
            "Touching contacts: %zu",
            contacts.size()
        );

        ImGui::Text(
            "Dynamic tree: %zu / h%d",
            dynamicTree.GetProxyCount(),
            dynamicTree.GetHeight()
        );
        ImGui::Text(
            "Kinematic tree: %zu / h%d",
            kinematicTree.GetProxyCount(),
            kinematicTree.GetHeight()
        );
        ImGui::Text(
            "Static tree: %zu / h%d",
            staticTree.GetProxyCount(),
            staticTree.GetHeight()
        );

        ImGui::Spacing();

        if( ImGui::Button(
            "Reset Camera",
            ImVec2( -1.0f, 0.0f ) ) )
        {
            camera = {};
            camera.center = { 0.0f, 0.5f };
            camera.pixelsPerMeter = 55.0f;
        }

        ImGui::Text(
            "FPS: %.1f",
            io.Framerate
        );
        ImGui::Text(
            "Scale: %.1f px/m",
            camera.pixelsPerMeter
        );

        ImGui::Spacing();
        ImGui::Separator();

        ImGui::TextColored(
            ImVec4( 1.0f, 0.65f, 0.25f, 1.0f ),
            "Contact Solver: not implemented"
        );

        ImGui::TextWrapped(
            "Contacts and normals are visualized, "
            "but Dynamic bodies currently pass through other shapes."
        );

        ImGui::Spacing();

        ImGui::TextWrapped(
            "Mouse wheel: zoom\n"
            "Middle drag: pan"
        );

        ImGui::EndChild();

        ImGui::SameLine();

        // -----------------------------------------------------
        // Physics Canvas
        // -----------------------------------------------------

        ImGui::BeginChild(
            "PhysicsCanvas",
            ImVec2( 0.0f, 0.0f ),
            true,
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse
        );

        const ImVec2 canvasMin =
            ImGui::GetCursorScreenPos();

        ImVec2 canvasSize =
            ImGui::GetContentRegionAvail();

        canvasSize.x =
            std::max( canvasSize.x, 1.0f );
        canvasSize.y =
            std::max( canvasSize.y, 1.0f );

        ImGui::InvisibleButton(
            "CanvasInput",
            canvasSize,
            ImGuiButtonFlags_MouseButtonMiddle
        );

        const bool canvasHovered =
            ImGui::IsItemHovered();

        if( canvasHovered &&
            io.MouseWheel != 0.0f )
        {
            const vec2 beforeZoom =
                camera.ScreenToWorld(
                    io.MousePos,
                    canvasMin,
                    canvasSize
                );

            camera.Zoom(
                io.MouseWheel
            );

            const vec2 afterZoom =
                camera.ScreenToWorld(
                    io.MousePos,
                    canvasMin,
                    canvasSize
                );

            // cursor 아래 world 위치가 zoom 전후에도 고정되게 camera center를 보정함.
            camera.center +=
                beforeZoom - afterZoom;
        }

        if( canvasHovered &&
            ImGui::IsMouseDragging(
                ImGuiMouseButton_Middle ) )
        {
            camera.PanPixels(
                io.MouseDelta
            );
        }

        ImDrawList* drawList =
            ImGui::GetWindowDrawList();

        const ImVec2 canvasMax
        {
            canvasMin.x + canvasSize.x,
            canvasMin.y + canvasSize.y
        };

        drawList->PushClipRect(
            canvasMin,
            canvasMax,
            true
        );

        drawList->AddRectFilled(
            canvasMin,
            canvasMax,
            IM_COL32( 23, 25, 31, 255 )
        );

        debugDraw debugDraw
        {
            drawList,
            camera,
            canvasMin,
            canvasSize
        };

        if( showGrid )
        {
            debugDraw.DrawGrid();
        }

        constexpr ImU32 DYNAMIC_TREE_LEAF =
            IM_COL32( 70, 200, 255, 220 );
        constexpr ImU32 DYNAMIC_TREE_INTERNAL =
            IM_COL32( 80, 120, 255, 140 );

        constexpr ImU32 KINEMATIC_TREE_LEAF =
            IM_COL32( 245, 205, 90, 220 );
        constexpr ImU32 KINEMATIC_TREE_INTERNAL =
            IM_COL32( 210, 165, 70, 130 );

        constexpr ImU32 STATIC_TREE_LEAF =
            IM_COL32( 110, 220, 130, 220 );
        constexpr ImU32 STATIC_TREE_INTERNAL =
            IM_COL32( 80, 160, 100, 130 );

        if( showStaticTree )
        {
            debugDraw.DrawTree(
                staticTree,
                "S",
                showTreeLeaves,
                showTreeInternal,
                showTreeLabels,
                STATIC_TREE_LEAF,
                STATIC_TREE_INTERNAL
            );
        }

        if( showKinematicTree )
        {
            debugDraw.DrawTree(
                kinematicTree,
                "K",
                showTreeLeaves,
                showTreeInternal,
                showTreeLabels,
                KINEMATIC_TREE_LEAF,
                KINEMATIC_TREE_INTERNAL
            );
        }

        if( showDynamicTree )
        {
            debugDraw.DrawTree(
                dynamicTree,
                "D",
                showTreeLeaves,
                showTreeInternal,
                showTreeLabels,
                DYNAMIC_TREE_LEAF,
                DYNAMIC_TREE_INTERNAL
            );
        }

        constexpr ImU32 AABB_COLOR =
            IM_COL32( 210, 100, 230, 210 );

        constexpr ImU32 LABEL_COLOR =
            IM_COL32( 230, 232, 238, 255 );

        constexpr ImU32 COM_COLOR =
            IM_COL32( 255, 90, 180, 255 );

        for( const visualShape& visual : scene->shapes )
        {
            const Body& body =
                scene->world.GetBody(
                    visual.bodyHandle
                );

            const shape& shape =
                scene->world.GetShape(
                    visual.shapeHandle
                );

            const transform2 transform =
                scene->world.GetBodyTransform(
                    visual.bodyHandle
                );

            const shapeColors colors =
                GetShapeColors(
                    body.type
                );

            debugDraw.DrawShape(
                shape.geometry,
                transform,
                colors.outline,
                colors.fill
            );

            if( showAABBs &&
                shape.proxyKey != shape::NULL_INDEX )
            {
                const dynamicTree& tree =
                    broadPhase.GetTree(
                        GetProxyType(
                            shape.proxyKey
                        )
                    );

                debugDraw.DrawAABB(
                    tree.GetProxyAABB(
                        GetProxyId(
                            shape.proxyKey
                        )
                    ),
                    AABB_COLOR
                );
            }

            if( showCOM &&
                body.type != bodyType::Static )
            {
                debugDraw.DrawPoint(
                    GetWorldCenter(
                        scene->world,
                        visual.bodyHandle
                    ),
                    COM_COLOR,
                    4.5f
                );
            }

            if( showLabels )
            {
                debugDraw.DrawLabel(
                    transform.position,
                    visual.label,
                    LABEL_COLOR
                );
            }
        }

        if( showContacts )
        {
            constexpr ImU32 CONTACT_COLOR =
                IM_COL32( 255, 220, 70, 255 );

            constexpr ImU32 NORMAL_COLOR =
                IM_COL32( 100, 255, 140, 255 );

            for( const contactData& contact : contacts )
            {
                for( int i = 0;
                     i < contact.manifold.pointCount;
                     ++i )
                {
                    const manifoldPoint2& point =
                        contact.manifold.points[i];

                    debugDraw.DrawPoint(
                        point.point,
                        CONTACT_COLOR,
                        5.0f
                    );

                    debugDraw.DrawArrow(
                        point.point,
                        contact.manifold.normal,
                        NORMAL_COLOR,
                        0.8f
                    );
                }
            }
        }

        drawList->PopClipRect();

        ImGui::EndChild();
        ImGui::End();

        ImGui::Render();

        // -----------------------------------------------------
        // Render
        // -----------------------------------------------------

        const float clearColor[4]
        {
            0.08f,
            0.08f,
            0.1f,
            1.0f
        };

        gD3d.deviceContext->OMSetRenderTargets(
            1,
            gD3d.renderTargetView.GetAddressOf(),
            nullptr
        );

        gD3d.deviceContext->ClearRenderTargetView(
            gD3d.renderTargetView.Get(),
            clearColor
        );

        ImGui_ImplDX11_RenderDrawData(
            ImGui::GetDrawData()
        );

        gD3d.swapChain->Present(
            1,
            0
        );
    }

    // ---------------------------------------------------------
    // Shutdown
    // ---------------------------------------------------------

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    gD3d.renderTargetView.Reset();
    gD3d.swapChain.Reset();
    gD3d.deviceContext.Reset();
    gD3d.device.Reset();

    DestroyWindow( hwnd );
    UnregisterClassW(
        className,
        instance
    );

    return 0;
}
