#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>

#include <array>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"

#include "collision/broadphase/broadPhase.h"
#include "collision/narrowphase/collide.h"
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

// MakeBox()는 원점 기준이므로 Sandbox 배치용으로 world 위치만 이동함.
polygon2 MakeDebugBox( const vec2& center, const vec2& halfExtents )
{
    polygon2 polygon = MakeBox( halfExtents );

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        polygon.vertices[i] += center;
    }

    polygon.centroid += center;

    return polygon;
}

using DebugGeometryRef =
    std::variant<
        const circle2*,
        const capsule2*,
        const segment2*,
        const polygon2*
    >;

struct DebugContact
{
    std::int32_t otherShape = -1;
    localManifold2 manifold{};
};

// Sandbox geometry는 이미 world 좌표로 배치되어 있으므로 identity transform을 사용함.
// 시각화 normal은 항상 움직이는 Circle에서 상대 Shape를 향하도록 맞춤.
localManifold2 CollideDebugCircle(
    const circle2& circle,
    const DebugGeometryRef& otherGeometry )
{
    return std::visit(
        [&]( const auto* geometry ) -> localManifold2
        {
            using Geometry =
                std::remove_cv_t<std::remove_pointer_t<decltype( geometry )>>;

            if constexpr( std::is_same_v<Geometry, circle2> )
            {
                return CollideCircles( circle, *geometry, {} );
            }
            else
            {
                localManifold2 manifold{};

                if constexpr( std::is_same_v<Geometry, capsule2> )
                {
                    manifold = CollideCapsuleCircle( *geometry, circle, {} );
                }
                else if constexpr( std::is_same_v<Geometry, segment2> )
                {
                    manifold = CollideSegmentCircle( *geometry, circle, {} );
                }
                else if constexpr( std::is_same_v<Geometry, polygon2> )
                {
                    manifold = CollidePolygonCircle( *geometry, circle, {} );
                }

                if( manifold.pointCount > 0 )
                {
                    manifold.normal = -manifold.normal;
                }

                return manifold;
            }
        },
        otherGeometry
    );
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

    // AABB Tree 구조를 관찰하기 위한 추가 geometry.
    const std::array<circle2, 5> extraCircles{
        circle2{ { -7.0f, -3.6f }, 0.65f },
        circle2{ { -6.2f,  4.0f }, 0.80f },
        circle2{ { -2.0f,  4.6f }, 0.55f },
        circle2{ {  5.7f,  3.8f }, 0.75f },
        circle2{ {  7.0f, -3.4f }, 0.90f }
    };

    const std::array<capsule2, 5> extraCapsules{
        capsule2{ { -7.2f,  0.5f }, { -5.9f,  1.4f }, 0.30f },
        capsule2{ { -5.0f, -4.1f }, { -3.5f, -3.2f }, 0.38f },
        capsule2{ { -0.3f, -3.8f }, {  1.4f, -4.5f }, 0.30f },
        capsule2{ {  3.7f,  4.1f }, {  5.1f,  4.6f }, 0.34f },
        capsule2{ {  5.8f, -0.6f }, {  7.1f,  0.6f }, 0.42f }
    };

    const std::array<segment2, 5> extraSegments{
        segment2{ { -7.5f, -1.1f }, { -6.0f, -2.3f } },
        segment2{ { -3.6f,  3.0f }, { -2.2f,  3.8f } },
        segment2{ {  0.8f,  2.7f }, {  2.4f,  3.4f } },
        segment2{ {  3.8f, -2.1f }, {  5.4f, -3.1f } },
        segment2{ {  6.0f,  2.0f }, {  7.4f,  1.0f } }
    };

    const std::array<polygon2, 5> extraPolygons{
        MakeDebugBox( { -5.2f,  2.5f }, { 0.65f, 0.45f } ),
        MakeDebugBox( { -2.4f, -2.7f }, { 0.55f, 0.85f } ),
        MakeDebugBox( {  0.3f,  0.8f }, { 0.75f, 0.50f } ),
        MakeDebugBox( {  3.1f,  1.1f }, { 0.50f, 0.90f } ),
        MakeDebugBox( {  6.1f, -4.0f }, { 0.70f, 0.45f } )
    };

    aabb2 circleAABB = ComputeAABB( circle );
    const aabb2 capsuleAABB = ComputeAABB( capsule );
    const aabb2 segmentAABB = ComputeAABB( segment );
    const aabb2 polygonAABB = ComputeAABB( polygon );

    // ---------------------------------------------------------
    // BroadPhase visual test
    // ---------------------------------------------------------

    BroadPhase broadPhase{};

    std::vector<Shape> shapes;
    std::vector<vec2> shapeCenters;
    std::vector<DebugGeometryRef> geometryRefs;

    shapes.reserve( 24 );
    shapeCenters.reserve( 24 );
    geometryRefs.reserve( 24 );

    auto createSceneProxy =
        [&]( BodyType type,
             const aabb2& aabb,
             const vec2& center,
             DebugGeometryRef geometry )
        {
            const std::int32_t shapeIndex =
                static_cast<std::int32_t>( shapes.size() );

            Shape shape{};
            shape.bodyId = shapeIndex;

            shapes.push_back( shape );
            shapeCenters.push_back( center );
            geometryRefs.push_back( geometry );

            const ProxyKey proxyKey =
                broadPhase.CreateProxy( type, aabb, shapeIndex );

            return std::pair{ shapeIndex, proxyKey };
        };

    const auto [circleShape, circleProxy] =
        createSceneProxy(
            BodyType::Dynamic,
            circleAABB,
            circle.center,
            &circle
        );

    createSceneProxy(
        BodyType::Static,
        capsuleAABB,
        ( capsule.center1 + capsule.center2 ) * 0.5f,
        &capsule
    );

    createSceneProxy(
        BodyType::Static,
        segmentAABB,
        ( segment.a + segment.b ) * 0.5f,
        &segment
    );

    createSceneProxy(
        BodyType::Static,
        polygonAABB,
        polygon.centroid,
        &polygon
    );

    for( std::size_t i = 0; i < extraCircles.size(); ++i )
    {
        const BodyType type =
            ( i & 1u ) == 0 ? BodyType::Dynamic : BodyType::Static;

        createSceneProxy(
            type,
            ComputeAABB( extraCircles[i] ),
            extraCircles[i].center,
            &extraCircles[i]
        );
    }

    for( std::size_t i = 0; i < extraCapsules.size(); ++i )
    {
        const BodyType type =
            ( i & 1u ) == 0 ? BodyType::Static : BodyType::Dynamic;

        createSceneProxy(
            type,
            ComputeAABB( extraCapsules[i] ),
            ( extraCapsules[i].center1 + extraCapsules[i].center2 ) * 0.5f,
            &extraCapsules[i]
        );
    }

    for( std::size_t i = 0; i < extraSegments.size(); ++i )
    {
        const BodyType type =
            ( i & 1u ) == 0 ? BodyType::Dynamic : BodyType::Static;

        createSceneProxy(
            type,
            ComputeAABB( extraSegments[i] ),
            ( extraSegments[i].a + extraSegments[i].b ) * 0.5f,
            &extraSegments[i]
        );
    }

    for( std::size_t i = 0; i < extraPolygons.size(); ++i )
    {
        const BodyType type =
            ( i & 1u ) == 0 ? BodyType::Static : BodyType::Dynamic;

        createSceneProxy(
            type,
            ComputeAABB( extraPolygons[i] ),
            extraPolygons[i].centroid,
            &extraPolygons[i]
        );
    }

    std::vector<std::pair<std::int32_t, std::int32_t>> candidatePairs;
    std::vector<DebugContact> contacts;

    // 초기 proxy의 moved 상태를 한 번 소비해 drag 전 상태를 깨끗하게 맞춤.
    broadPhase.UpdatePairs(
        shapes,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            if( shapeIndexA == circleShape || shapeIndexB == circleShape )
            {
                candidatePairs.emplace_back( shapeIndexA, shapeIndexB );
            }
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
        ImGui::Text( "Scene shapes: %zu", shapes.size() );
        ImGui::Text( "Circle BroadPhase candidates: %zu", candidatePairs.size() );
        ImGui::Text( "Circle NarrowPhase contacts: %zu", contacts.size() );

        ImGui::Spacing();
        ImGui::TextWrapped(
            "Left drag blue Circle: move proxy\n"
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

        if( !contacts.empty() )
        {
            ImGui::Separator();
            ImGui::TextUnformatted( "NarrowPhase contacts" );

            for( const DebugContact& contact : contacts )
            {
                ImGui::Text(
                    "Circle %d <-> Shape %d",
                    circleShape,
                    contact.otherShape
                );

                ImGui::Text(
                    "  normal: (%.2f, %.2f)",
                    contact.manifold.normal.x,
                    contact.manifold.normal.y
                );

                for( int i = 0; i < contact.manifold.pointCount; ++i )
                {
                    const localManifoldPoint2& point =
                        contact.manifold.points[i];

                    ImGui::Text(
                        "  p%d (%.2f, %.2f) sep %.3f",
                        i,
                        point.point.x,
                        point.point.y,
                        point.separation
                    );
                }
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted( "Visible geometry" );
        ImGui::BulletText( "Circle x6" );
        ImGui::BulletText( "Capsule x6" );
        ImGui::BulletText( "Segment x6" );
        ImGui::BulletText( "Polygon x6" );

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
                shapeCenters[circleShape] = circle.center;

                // 실제 BroadPhase proxy를 새 Circle AABB로 이동시킴.
                broadPhase.MoveProxy( circleProxy, circleAABB );

                candidatePairs.clear();

                broadPhase.UpdatePairs(
                    shapes,
                    [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
                    {
                        if( shapeIndexA == circleShape || shapeIndexB == circleShape )
                        {
                            candidatePairs.emplace_back( shapeIndexA, shapeIndexB );
                        }
                    }
                );

                contacts.clear();

                for( const auto& pair : candidatePairs )
                {
                    if( pair.first != circleShape && pair.second != circleShape )
                    {
                        continue;
                    }

                    const std::int32_t otherShape =
                        pair.first == circleShape ? pair.second : pair.first;

                    if( otherShape < 0 ||
                        static_cast<std::size_t>( otherShape ) >= geometryRefs.size() )
                    {
                        continue;
                    }

                    const localManifold2 manifold =
                        CollideDebugCircle(
                            circle,
                            geometryRefs[otherShape]
                        );

                    if( manifold.pointCount > 0 )
                    {
                        contacts.push_back( { otherShape, manifold } );
                    }
                }
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
        constexpr ImU32 CONTACT_COLOR = IM_COL32( 255, 220, 70, 255 );
        constexpr ImU32 NORMAL_COLOR = IM_COL32( 100, 255, 140, 255 );

        debugDraw.DrawCircle( circle, CIRCLE_OUTLINE, CIRCLE_FILL );
        debugDraw.DrawCapsule( capsule, CAPSULE_OUTLINE, CAPSULE_FILL );
        debugDraw.DrawSegment( segment, SEGMENT_COLOR, 3.0f );
        debugDraw.DrawPolygon( polygon, POLYGON_OUTLINE, POLYGON_FILL );

        for( const circle2& extraCircle : extraCircles )
        {
            debugDraw.DrawCircle( extraCircle, CIRCLE_OUTLINE, CIRCLE_FILL );
        }

        for( const capsule2& extraCapsule : extraCapsules )
        {
            debugDraw.DrawCapsule( extraCapsule, CAPSULE_OUTLINE, CAPSULE_FILL );
        }

        for( const segment2& extraSegment : extraSegments )
        {
            debugDraw.DrawSegment( extraSegment, SEGMENT_COLOR, 3.0f );
        }

        for( const polygon2& extraPolygon : extraPolygons )
        {
            debugDraw.DrawPolygon( extraPolygon, POLYGON_OUTLINE, POLYGON_FILL );
        }

        if( showAABBs )
        {
            debugDraw.DrawAABB( circleAABB, AABB_COLOR );
            debugDraw.DrawAABB( capsuleAABB, AABB_COLOR );
            debugDraw.DrawAABB( segmentAABB, AABB_COLOR );
            debugDraw.DrawAABB( polygonAABB, AABB_COLOR );

            for( const circle2& extraCircle : extraCircles )
            {
                debugDraw.DrawAABB( ComputeAABB( extraCircle ), AABB_COLOR );
            }

            for( const capsule2& extraCapsule : extraCapsules )
            {
                debugDraw.DrawAABB( ComputeAABB( extraCapsule ), AABB_COLOR );
            }

            for( const segment2& extraSegment : extraSegments )
            {
                debugDraw.DrawAABB( ComputeAABB( extraSegment ), AABB_COLOR );
            }

            for( const polygon2& extraPolygon : extraPolygons )
            {
                debugDraw.DrawAABB( ComputeAABB( extraPolygon ), AABB_COLOR );
            }
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
            if( pair.first != circleShape && pair.second != circleShape )
            {
                continue;
            }

            const std::int32_t otherShape =
                pair.first == circleShape ? pair.second : pair.first;

            if( otherShape < 0 ||
                static_cast<std::size_t>( otherShape ) >= shapeCenters.size() )
            {
                continue;
            }

            debugDraw.DrawSegment(
                { circle.center, shapeCenters[otherShape] },
                PAIR_COLOR,
                3.0f
            );
        }

        for( const DebugContact& contact : contacts )
        {
            for( int i = 0; i < contact.manifold.pointCount; ++i )
            {
                const localManifoldPoint2& point =
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
