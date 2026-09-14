#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>

#include <imgui.h>
#include <imgui_impl_win32.h>
#include <imgui_impl_dx11.h>

using Microsoft::WRL::ComPtr;

struct D3D11Context
{
    Microsoft::WRL::ComPtr<ID3D11Device> device;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> deviceContext;
    Microsoft::WRL::ComPtr<IDXGISwapChain> swapChain;
    Microsoft::WRL::ComPtr<ID3D11RenderTargetView> renderTargetView;
};

D3D11Context g_d3d;

bool CreateRenderTarget()
{
    Microsoft::WRL::ComPtr<ID3D11Texture2D> backBuffer;

    HRESULT result = g_d3d.swapChain->GetBuffer(
        0,
        IID_PPV_ARGS(&backBuffer)
    );

    if (FAILED(result))
    {
        return false;
    }

    result = g_d3d.device->CreateRenderTargetView(
        backBuffer.Get(),
        nullptr,
        &g_d3d.renderTargetView
    );

    return SUCCEEDED(result);
}

void DestroyRenderTarget()
{
    g_d3d.renderTargetView.Reset();
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hwnd,
    UINT message,
    WPARAM wParam,
    LPARAM lParam
);

LRESULT CALLBACK WndProc(
    HWND hwnd,
    UINT message,
    WPARAM wParam,
    LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(
        hwnd,
        message,
        wParam,
        lParam))
    {
        return true;
    }

    switch (message)
    {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;

    case WM_SIZE:
    {
        if (g_d3d.swapChain && wParam != SIZE_MINIMIZED)
        {
            DestroyRenderTarget();

            UINT width = LOWORD(lParam);
            UINT height = HIWORD(lParam);

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
    }



    return DefWindowProcW(
        hwnd,
        message,
        wParam,
        lParam
    );
}

int main()
{
    HINSTANCE instance = GetModuleHandleW(nullptr);

    const wchar_t* className = L"ZonaiSandboxWindow";

    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = WndProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = className;

    if (!RegisterClassW(&windowClass))
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

    if (!hwnd)
    {
        return 1;
    }

    ShowWindow(hwnd, SW_SHOW);

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

    ComPtr<ID3D11Device> device;
    ComPtr<ID3D11DeviceContext> deviceContext;
    ComPtr<IDXGISwapChain> swapChain;

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
            &swapChain,
            &device,
            &featureLevel,
            &deviceContext
        );

    if (FAILED(result))
    {
        return 1;
    }

    // ---------------------------------------------------------
    // Render Target
    // ---------------------------------------------------------

    ComPtr<ID3D11Texture2D> backBuffer;

    result = swapChain->GetBuffer(
        0,
        IID_PPV_ARGS(&backBuffer)
    );

    if (FAILED(result))
    {
        return 1;
    }

    ComPtr<ID3D11RenderTargetView> renderTargetView;

    result = device->CreateRenderTargetView(
        backBuffer.Get(),
        nullptr,
        &renderTargetView
    );

    if (FAILED(result))
    {
        return 1;
    }

    // ---------------------------------------------------------
    // ImGui
    // ---------------------------------------------------------

    IMGUI_CHECKVERSION();

    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    if (!ImGui_ImplWin32_Init(hwnd))
    {
        return 1;
    }

    if (!ImGui_ImplDX11_Init(
        device.Get(),
        deviceContext.Get()))
    {
        return 1;
    }

    // ---------------------------------------------------------
    // Main Loop
    // ---------------------------------------------------------

    MSG message{};
    bool running = true;

    while (running)
    {
        while (PeekMessageW(
            &message,
            nullptr,
            0,
            0,
            PM_REMOVE))
        {
            if (message.message == WM_QUIT)
            {
                running = false;
                break;
            }

            TranslateMessage(&message);
            DispatchMessageW(&message);
        }

        if (!running)
        {
            break;
        }

        // -----------------------------------------------------
        // ImGui Frame
        // -----------------------------------------------------

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Zonai Physics Sandbox");

        ImGui::Text("Hello, Zonai!");
        ImGui::Separator();

        ImGui::Text(
            "FPS: %.1f",
            io.Framerate
        );

        ImGui::End();

        ImGui::Render();

        // -----------------------------------------------------
        // Render
        // -----------------------------------------------------

        const float clearColor[4] =
        {
            0.1f,
            0.1f,
            0.15f,
            1.0f
        };

        deviceContext->OMSetRenderTargets(
            1,
            renderTargetView.GetAddressOf(),
            nullptr
        );

        deviceContext->ClearRenderTargetView(
            g_d3d.renderTargetView.Get(),
            clearColor
        );

        ImGui_ImplDX11_RenderDrawData(
            ImGui::GetDrawData()
        );

        swapChain->Present(1, 0);
    }

    // ---------------------------------------------------------
    // Shutdown
    // ---------------------------------------------------------

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();

    ImGui::DestroyContext();

    renderTargetView.Reset();
    swapChain.Reset();
    deviceContext.Reset();
    device.Reset();

    DestroyWindow(hwnd);
    UnregisterClassW(
        className,
        instance
    );

    return 0;
}