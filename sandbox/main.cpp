#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>

#include <algorithm>

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

#include "debug/debugCamera.h"
#include "debug/debugDraw.h"

#include "demo.h"
#include "rigidBodyDemoUi.h"

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
            L"조나이 물리 실험실",
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

    if( !initializeDemoUi() )
    {
        MessageBoxW( hwnd, L"한글 폰트를 읽을 수 없습니다. 실행 파일 옆의 assets 폴더를 확인하세요.", L"조나이 물리 실험실", MB_OK | MB_ICONERROR );
        ImGui::DestroyContext();
        return 1;
    }

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

    demoSession session{ createDemoView };
    int subStepCount = 1;
    const auto resetCamera = [&]()
    {
        const demoEntry& entry = getDemoEntry( session.getKind() );
        camera = {}; camera.center = entry.cameraCenter; camera.pixelsPerMeter = entry.pixelsPerMeter;
    };
    resetCamera();

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

        bool stepRequested = false;
        bool demoChanged = false;

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
            "조나이 물리 실험실",
            nullptr,
            WINDOW_FLAGS
        );

        // -----------------------------------------------------
        // Controls
        // -----------------------------------------------------

        ImGui::BeginChild(
            "조작 안내###Controls",
            ImVec2( 340.0f, 0.0f ),
            true
        );

        const demoEntry& selectedEntry = getDemoEntry( session.getKind() );
        ImGui::PushItemWidth( ImGui::GetContentRegionAvail().x * 0.45f );
        if( ImGui::BeginCombo( "데모###Demo", selectedEntry.name ) )
        {
            for( const demoEntry& entry : getDemoEntries() )
            {
                const bool selected = entry.kind == session.getKind();
                if( ImGui::Selectable( entry.name, selected ) )
                {
                    session.selectDemo( entry.kind ); resetCamera(); demoChanged = true;
                }
                if( selected ) { ImGui::SetItemDefaultFocus(); }
            }
            ImGui::EndCombo();
        }
        ImGui::TextUnformatted( getDemoEntry( session.getKind() ).category );
        if( ImGui::Button( session.isPlaying() ? "일시 정지###Pause" : "재생###Play", ImVec2( 78.0f, 0.0f ) ) ) { session.setPlaying( !session.isPlaying() ); }
        ImGui::SameLine();
        if( ImGui::Button( "한 단계###Step", ImVec2( 72.0f, 0.0f ) ) ) { stepRequested = true; }
        ImGui::SameLine();
        if( ImGui::Button( "초기화###Reset", ImVec2( 72.0f, 0.0f ) ) ) { session.reset(); resetCamera(); demoChanged = true; }
        ImGui::SliderInt( "하위 단계 수###Sub-steps", &subStepCount, 1, 16 );
        ImGui::Text( "고정 시간 간격: %.5f s / 진행 단계: %llu", 1.0f / 60.0f, static_cast<unsigned long long>( session.getStepCount() ) );
        if( ImGui::Button( "카메라 초기화###Reset Camera", ImVec2( -1.0f, 0.0f ) ) ) { resetCamera(); }
        ImGui::Text( "초당 프레임: %.1f / 화면 배율: %.1f px/m", io.Framerate, camera.pixelsPerMeter );
        drawProjectCollisionSettings( session );
        ImGui::Separator();
        ImGui::TextUnformatted( "조작 안내" );
        ImGui::TextWrapped( "%s", getDemoEntry( session.getKind() ).controls );
        ImGui::TextWrapped( "마우스 휠로 확대·축소하고 가운데 버튼 드래그로 화면을 이동합니다. 데모 키 조작은 캔버스 위에서 작동합니다. UI 편집 중에는 캔버스 입력을 중단합니다." );
        ImGui::Separator();
        session.getDemo().drawControls();
        ImGui::PopItemWidth();

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

        // Canvas hover는 다른 active UI widget에 의해 차단됨. 입력을 먼저 처리해야
        // UI/focus 상실 frame에 이전 held force로 physics step이 추가되지 않음.
        const bool inputEnabled = canvasHovered && !io.AppFocusLost && !io.WantCaptureKeyboard && !io.WantTextInput && !demoChanged && !ImGui::IsMouseDown( ImGuiMouseButton_Middle );
        demoInput input{};
        if( inputEnabled )
        {
            input.mousePosition = camera.ScreenToWorld( io.MousePos, canvasMin, canvasSize );
            input.mousePressed = ImGui::IsMouseClicked( ImGuiMouseButton_Left );
            input.mouseHeld = ImGui::IsMouseDown( ImGuiMouseButton_Left );
            input.impulsePressed = ImGui::IsMouseClicked( ImGuiMouseButton_Right );
            input.left = ImGui::IsKeyDown( ImGuiKey_A ); input.right = ImGui::IsKeyDown( ImGuiKey_D );
            input.jumpPressed = ImGui::IsKeyPressed( ImGuiKey_Space, false );
            input.spinPressed = ImGui::IsKeyPressed( ImGuiKey_S, false );
        }
        session.handleInput( input, inputEnabled );
        if( stepRequested ) { session.stepOnce( subStepCount ); }
        else { session.advance( io.DeltaTime, subStepCount ); }

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

        session.getDemo().draw( debugDraw );

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
