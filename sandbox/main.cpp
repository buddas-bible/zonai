#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"

#include "collision/broadphase/broadPhase.h"
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

struct D3D11Context
{
    ComPtr<ID3D11Device> device;
    ComPtr<ID3D11DeviceContext> deviceContext;
    ComPtr<IDXGISwapChain> swapChain;
    ComPtr<ID3D11RenderTargetView> renderTargetView;
};

D3D11Context g_d3d;

bool CreateRenderTarget()
{
    ComPtr<ID3D11Texture2D> backBuffer;

    HRESULT result = g_d3d.swapChain->GetBuffer(
        0,
        IID_PPV_ARGS( &backBuffer )
    );

    if( FAILED( result ) )
    {
        return false;
    }

    result = g_d3d.device->CreateRenderTargetView(
        backBuffer.Get(),
        nullptr,
        &g_d3d.renderTargetView
    );

    return SUCCEEDED( result );
}

void DestroyRenderTarget()
{
    g_d3d.renderTargetView.Reset();
}

LRESULT CALLBACK WndProc(
    HWND hwnd,
    UINT message,
    WPARAM wParam,
    LPARAM lParam )
{
    if( ImGui_ImplWin32_WndProcHandler( hwnd, message, wParam, lParam ) )
    {
        return true;
    }

    switch( message )
    {
    case WM_DESTROY:
        PostQuitMessage( 0 );
        return 0;

    case WM_SIZE:
        if( g_d3d.swapChain && wParam != SIZE_MINIMIZED )
        {
            DestroyRenderTarget();

            const UINT width = LOWORD( lParam );
            const UINT height = HIWORD( lParam );

            g_d3d.swapChain->ResizeBuffers(
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

    return DefWindowProcW( hwnd, message, wParam, lParam );
}

} // namespace

int main()
{
    using namespace zonai;
    using namespace zonai::sandbox;

    HINSTANCE instance = GetModuleHandleW( nullptr );

    const wchar_t* className = L"ZonaiSandboxWindow";

    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = WndProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = className;

    if( !RegisterClassW( &windowClass ) )
    {
        return 1;
    }

    HWND hwnd = CreateWindowExW(
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
    swapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapChainDesc.OutputWindow = hwnd;
    swapChainDesc.SampleDesc.Count = 1;
    swapChainDesc.Windowed = TRUE;
    swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    D3D_FEATURE_LEVEL featureLevel{};

    HRESULT result =
        D3D11CreateDeviceAndSwapChain(
            nullptr,
            D3D_DRIVER_TYPE_HARDWARE,
            nullptr,
            0,
            nullptr,
            0,
            D3D11_SDK_VERSION,
            &swapChainDesc,
            &g_d3d.swapChain,
            &g_d3d.device,
            &featureLevel,
            &g_d3d.deviceContext
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
        g_d3d.device.Get(),
        g_d3d.deviceContext.Get() ) )
    {
        return 1;
    }

    // ---------------------------------------------------------
    // Visual test scene
    // ---------------------------------------------------------

    DebugCamera camera{};

    bool showGrid = true;
    bool showAABBs = true;
    bool showLabels = true;

    bool showDynamicTree = true;
    bool showStaticTree = true;
    bool showTreeLeaves = true;
    bool showTreeInternal = true;
    bool showTreeLabels = false;

    circle2 circle{
        { -4.0f, 2.0f },
        0.9f
    };

    const capsule2 capsule{
        { -1.5f, -1.5f },
        { 1.0f, -0.4f },
        0.45f
    };

    const segment2 segment{
        { 2.5f, 2.2f },
        { 5.2f, 1.0f }
    };

    const std::array<vec2, 5> polygonVertices{
        vec2{ 2.4f, -0.8f },
        vec2{ 3.8f, 0.2f },
        vec2{ 5.3f, -0.7f },
        vec2{ 4.8f, -2.3f },
        vec2{ 2.5f, -2.5f }
    };

    const polygon2 polygon = MakePolygon( polygonVertices );

    aabb2 circleAABB = ComputeAABB( circle );
    const aabb2 capsuleAABB = ComputeAABB( capsule );
    const aabb2 segmentAABB = ComputeAABB( segment );
    const aabb2 polygonAABB = ComputeAABB( polygon );

    // ---------------------------------------------------------
    // BroadPhase visual test
    // ---------------------------------------------------------

    constexpr std::int32_t CIRCLE_SHAPE = 0;
    constexpr std::int32_t CAPSULE_SHAPE = 1;
    constexpr std::int32_t SEGMENT_SHAPE = 2;
    constexpr std::int32_t POLYGON_SHAPE = 3;

    BroadPhase broadPhase{};

    const ProxyKey circleProxy =
        broadPhase.CreateProxy( BodyType::Dynamic, circleAABB, CIRCLE_SHAPE );

    broadPhase.CreateProxy( BodyType::Static, capsuleAABB, CAPSULE_SHAPE );
    broadPhase.CreateProxy( BodyType::Static, segmentAABB, SEGMENT_SHAPE );
    broadPhase.CreateProxy( BodyType::Static, polygonAABB, POLYGON_SHAPE );

    std::array<Shape, 4> shapes{};

    for( std::int32_t i = 0; i < static_cast<std::int32_t>( shapes.size() ); ++i )
    {
        shapes[i].bodyId = i;
    }

    std::array<std::int32_t, 256> movedSiblings{};
    std::vector<std::pair<std::int32_t, std::int32_t>> candidatePairs;

    // 초기 proxy의 moved 상태를 한 번 소비해 drag 전 상태를 깨끗하게 맞춤.
    broadPhase.UpdatePairs(
        movedSiblings,
        shapes,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            candidatePairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    candidatePairs.clear();

    const DynamicTree& dynamicTree = broadPhase.GetTree( BodyType::Dynamic );
    const DynamicTree& staticTree = broadPhase.GetTree( BodyType::Static );

    bool draggingCircle = false;
    vec2 circleGrabOffset{};

    // ---------------------------------------------------------
    // Main Loop
    // ---------------------------------------------------------

    MSG message{};
    bool running = true;

    while( running )
    {
        while( PeekMessageW( &message, nullptr, 0, 0, PM_REMOVE ) )
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

        const ImGuiViewport* viewport = ImGui::GetMainViewport();

        ImGui::SetNextWindowPos( viewport->WorkPos );
        ImGui::SetNextWindowSize( viewport->WorkSize );

        constexpr ImGuiWindowFlags WINDOW_FLAGS =
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize;

        ImGui::Begin( "Zonai Physics Sandbox", nullptr, WINDOW_FLAGS );

        // -----------------------------------------------------
        // Controls
        // -----------------------------------------------------

        ImGui::BeginChild( "Controls", ImVec2( 230.0f, 0.0f ), true );

        ImGui::TextUnformatted( "Geometry Debug View" );
        ImGui::Separator();

        ImGui::Checkbox( "Grid / Axis", &showGrid );
        ImGui::Checkbox( "Shape AABBs", &showAABBs );
        ImGui::Checkbox( "Labels", &showLabels );

        ImGui::Separator();
        ImGui::TextUnformatted( "AABB Tree" );

        ImGui::Checkbox( "Dynamic Tree", &showDynamicTree );
        ImGui::Checkbox( "Static Tree", &showStaticTree );
        ImGui::Checkbox( "Tree Leaves", &showTreeLeaves );
        ImGui::Checkbox( "Tree Internal", &showTreeInternal );
        ImGui::Checkbox( "Tree Labels", &showTreeLabels );

        ImGui::Spacing();

        ImGui::Text(
            "Dynamic: proxies %zu / height %d",
            dynamicTree.GetProxyCount(),
            dynamicTree.GetHeight()
        );
        ImGui::Text( "Dynamic area ratio: %.2f", dynamicTree.GetAreaRatio() );

        ImGui::Text(
            "Static: proxies %zu / height %d",
            staticTree.GetProxyCount(),
            staticTree.GetHeight()
        );
        ImGui::Text( "Static area ratio: %.2f", staticTree.GetAreaRatio() );

        ImGui::Spacing();

        if( ImGui::Button( "Reset Camera" ) )
        {
            camera = {};
        }

        ImGui::Separator();

        ImGui::Text( "FPS: %.1f", io.Framerate );
        ImGui::Text( "Scale: %.1f px/m", camera.pixelsPerMeter );
        ImGui::Text( "BroadPhase pairs: %zu", candidatePairs.size() );

        ImGui::Spacing();
        ImGui::TextWrapped(
            "Left drag Circle: move proxy\n"
            "Mouse wheel: zoom\n"
            "Middle drag: pan"
        );

        if( !candidatePairs.empty() )
        {
            ImGui::Separator();
            ImGui::TextUnformatted( "Candidate pairs" );

            for( const auto& pair : candidatePairs )
            {
                ImGui::BulletText( "%d <-> %d", pair.first, pair.second );
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted( "Visible geometry" );
        ImGui::BulletText( "Circle" );
        ImGui::BulletText( "Capsule" );
        ImGui::BulletText( "Segment" );
        ImGui::BulletText( "Polygon" );

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

        ImVec2 canvasMin = ImGui::GetCursorScreenPos();
        ImVec2 canvasSize = ImGui::GetContentRegionAvail();

        canvasSize.x = canvasSize.x < 1.0f ? 1.0f : canvasSize.x;
        canvasSize.y = canvasSize.y < 1.0f ? 1.0f : canvasSize.y;

        ImGui::InvisibleButton(
            "CanvasInput",
            canvasSize,
            ImGuiButtonFlags_MouseButtonLeft |
            ImGuiButtonFlags_MouseButtonMiddle
        );

        const bool canvasHovered = ImGui::IsItemHovered();

        if( canvasHovered && io.MouseWheel != 0.0f )
        {
            const vec2 beforeZoom =
                camera.ScreenToWorld( io.MousePos, canvasMin, canvasSize );

            camera.Zoom( io.MouseWheel );

            const vec2 afterZoom =
                camera.ScreenToWorld( io.MousePos, canvasMin, canvasSize );

            // cursor 아래 world 좌표가 줌 전후에도 같은 위치에 머물게 함.
            camera.center += beforeZoom - afterZoom;
        }

        if( canvasHovered && ImGui::IsMouseDragging( ImGuiMouseButton_Middle ) )
        {
            camera.PanPixels( io.MouseDelta );
        }

        const vec2 mouseWorld =
            camera.ScreenToWorld( io.MousePos, canvasMin, canvasSize );

        if( canvasHovered &&
            ImGui::IsMouseClicked( ImGuiMouseButton_Left ) &&
            Contains( circle, mouseWorld ) )
        {
            draggingCircle = true;
            circleGrabOffset = circle.center - mouseWorld;
        }

        if( draggingCircle )
        {
            if( ImGui::IsMouseDown( ImGuiMouseButton_Left ) )
            {
                circle.center = mouseWorld + circleGrabOffset;
                circleAABB = ComputeAABB( circle );

                // 실제 BroadPhase proxy를 새 Circle AABB로 이동시킴.
                broadPhase.MoveProxy( circleProxy, circleAABB );

                candidatePairs.clear();

                broadPhase.UpdatePairs(
                    movedSiblings,
                    shapes,
                    [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
                    {
                        candidatePairs.emplace_back( shapeIndexA, shapeIndexB );
                    }
                );
            }
            else
            {
                draggingCircle = false;
            }
        }

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 canvasMax{
            canvasMin.x + canvasSize.x,
            canvasMin.y + canvasSize.y
        };

        drawList->PushClipRect( canvasMin, canvasMax, true );
        drawList->AddRectFilled(
            canvasMin,
            canvasMax,
            IM_COL32( 23, 25, 31, 255 )
        );

        DebugDraw debugDraw{
            drawList,
            camera,
            canvasMin,
            canvasSize
        };

        if( showGrid )
        {
            debugDraw.DrawGrid();
        }

        constexpr ImU32 DYNAMIC_TREE_LEAF = IM_COL32( 70, 200, 255, 220 );
        constexpr ImU32 DYNAMIC_TREE_INTERNAL = IM_COL32( 80, 120, 255, 150 );

        constexpr ImU32 STATIC_TREE_LEAF = IM_COL32( 110, 220, 130, 220 );
        constexpr ImU32 STATIC_TREE_INTERNAL = IM_COL32( 230, 180, 80, 150 );

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

        constexpr ImU32 CIRCLE_OUTLINE = IM_COL32( 90, 200, 255, 255 );
        constexpr ImU32 CIRCLE_FILL = IM_COL32( 90, 200, 255, 70 );

        constexpr ImU32 CAPSULE_OUTLINE = IM_COL32( 120, 220, 130, 255 );
        constexpr ImU32 CAPSULE_FILL = IM_COL32( 120, 220, 130, 70 );

        constexpr ImU32 SEGMENT_COLOR = IM_COL32( 245, 205, 90, 255 );

        constexpr ImU32 POLYGON_OUTLINE = IM_COL32( 235, 135, 80, 255 );
        constexpr ImU32 POLYGON_FILL = IM_COL32( 235, 135, 80, 70 );

        constexpr ImU32 AABB_COLOR = IM_COL32( 210, 100, 230, 210 );
        constexpr ImU32 LABEL_COLOR = IM_COL32( 230, 232, 238, 255 );
        constexpr ImU32 PAIR_COLOR = IM_COL32( 255, 80, 100, 255 );

        debugDraw.DrawCircle( circle, CIRCLE_OUTLINE, CIRCLE_FILL );
        debugDraw.DrawCapsule( capsule, CAPSULE_OUTLINE, CAPSULE_FILL );
        debugDraw.DrawSegment( segment, SEGMENT_COLOR, 3.0f );
        debugDraw.DrawPolygon( polygon, POLYGON_OUTLINE, POLYGON_FILL );

        if( showAABBs )
        {
            debugDraw.DrawAABB( circleAABB, AABB_COLOR );
            debugDraw.DrawAABB( capsuleAABB, AABB_COLOR );
            debugDraw.DrawAABB( segmentAABB, AABB_COLOR );
            debugDraw.DrawAABB( polygonAABB, AABB_COLOR );
        }

        if( showLabels )
        {
            debugDraw.DrawLabel( circle.center, "Circle [0] Dynamic", LABEL_COLOR );
            debugDraw.DrawLabel( capsule.center1, "Capsule [1] Static", LABEL_COLOR );
            debugDraw.DrawLabel( segment.a, "Segment [2] Static", LABEL_COLOR );
            debugDraw.DrawLabel( polygon.centroid, "Polygon [3] Static", LABEL_COLOR );
        }

        for( const auto& pair : candidatePairs )
        {
            const std::int32_t otherShape =
                pair.first == CIRCLE_SHAPE ? pair.second : pair.first;

            vec2 otherCenter{};

            switch( otherShape )
            {
            case CAPSULE_SHAPE:
                otherCenter = ( capsule.center1 + capsule.center2 ) * 0.5f;
                break;

            case SEGMENT_SHAPE:
                otherCenter = ( segment.a + segment.b ) * 0.5f;
                break;

            case POLYGON_SHAPE:
                otherCenter = polygon.centroid;
                break;

            default:
                continue;
            }

            debugDraw.DrawSegment(
                { circle.center, otherCenter },
                PAIR_COLOR,
                3.0f
            );
        }

        drawList->PopClipRect();

        ImGui::EndChild();
        ImGui::End();

        ImGui::Render();

        // -----------------------------------------------------
        // Render
        // -----------------------------------------------------

        const float clearColor[4]{
            0.08f,
            0.08f,
            0.1f,
            1.0f
        };

        g_d3d.deviceContext->OMSetRenderTargets(
            1,
            g_d3d.renderTargetView.GetAddressOf(),
            nullptr
        );

        g_d3d.deviceContext->ClearRenderTargetView(
            g_d3d.renderTargetView.Get(),
            clearColor
        );

        ImGui_ImplDX11_RenderDrawData( ImGui::GetDrawData() );

        g_d3d.swapChain->Present( 1, 0 );
    }

    // ---------------------------------------------------------
    // Shutdown
    // ---------------------------------------------------------

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    g_d3d.renderTargetView.Reset();
    g_d3d.swapChain.Reset();
    g_d3d.deviceContext.Reset();
    g_d3d.device.Reset();

    DestroyWindow( hwnd );
    UnregisterClassW( className, instance );

    return 0;
}
