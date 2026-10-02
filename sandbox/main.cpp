#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
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

const char* GetBodyTypeName( bodyType type )
{
    switch( type )
    {
    case bodyType::Static:
        return "Static";

    case bodyType::Kinematic:
        return "Kinematic";

    case bodyType::Dynamic:
        return "Dynamic";

    default:
        return "Unknown";
    }
}

shapeColors GetShapeColors( bodyType type, bool awake )
{
    if( type == bodyType::Dynamic && !awake )
    {
        return
        {
            IM_COL32( 100, 125, 145, 220 ),
            IM_COL32( 100, 125, 145, 45 )
        };
    }

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
    bool showShapeAABBs = true;
    bool showFatAABBs = false;
    bool showContacts = true;
    bool showContactDetails = true;
    bool showCOM = true;
    bool showVelocities = true;
    bool showLabels = true;

    bool showDynamicTree = false;
    bool showKinematicTree = false;
    bool showStaticTree = false;
    bool showTreeLeaves = true;
    bool showTreeInternal = true;
    bool showTreeLabels = false;

    int subStepCount = 1;
    int selectedShapeIndex = 2;

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
                    FIXED_TIME_STEP,
                    subStepCount
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
            ImVec2( 340.0f, 0.0f ),
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
                FIXED_TIME_STEP,
                subStepCount
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
            selectedShapeIndex = 2;

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

        ImGui::SliderInt(
            "Sub-steps",
            &subStepCount,
            1,
            16
        );

        float maximumLinearSpeed =
            scene->world.GetMaximumLinearSpeed();

        if( ImGui::DragFloat(
            "Max linear speed",
            &maximumLinearSpeed,
            1.0f,
            1.0f,
            1000.0f,
            "%.1f m/s" ) )
        {
            scene->world.SetMaximumLinearSpeed(
                maximumLinearSpeed
            );
        }

        bool sleepingEnabled =
            scene->world.IsSleepingEnabled();

        if( ImGui::Checkbox(
            "Enable sleeping",
            &sleepingEnabled ) )
        {
            scene->world.SetSleepingEnabled(
                sleepingEnabled
            );
        }

        bool continuousEnabled =
            scene->world.IsContinuousEnabled();

        if( ImGui::Checkbox(
            "Continuous collision",
            &continuousEnabled ) )
        {
            scene->world.SetContinuousEnabled(
                continuousEnabled
            );
        }

        float contactRecycleDistance =
            scene->world.GetContactRecycleDistance();

        if( ImGui::DragFloat(
            "Contact recycle dist",
            &contactRecycleDistance,
            0.001f,
            0.0f,
            0.2f,
            "%.3f m" ) )
        {
            scene->world.SetContactRecycleDistance(
                contactRecycleDistance
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
            "Object Inspector"
        );

        if( !scene->shapes.empty() )
        {
            selectedShapeIndex =
                std::clamp(
                    selectedShapeIndex,
                    0,
                    static_cast<int>( scene->shapes.size() ) - 1
                );

            const visualShape& selectedVisual =
                scene->shapes[selectedShapeIndex];

            if( ImGui::BeginCombo(
                "Object",
                selectedVisual.label ) )
            {
                for( int i = 0;
                     i < static_cast<int>( scene->shapes.size() );
                     ++i )
                {
                    const bool selected =
                        i == selectedShapeIndex;

                    if( ImGui::Selectable(
                        scene->shapes[i].label,
                        selected ) )
                    {
                        selectedShapeIndex = i;
                    }

                    if( selected )
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }

            const visualShape& visual =
                scene->shapes[selectedShapeIndex];

            const body& bodyRef =
                scene->world.GetBody(
                    visual.bodyHandle
                );

            ImGui::Text(
                "Body type: %s",
                GetBodyTypeName( bodyRef.type )
            );

            transform2 transform =
                scene->world.GetBodyTransform(
                    visual.bodyHandle
                );

            float position[2]
            {
                transform.position.x,
                transform.position.y
            };

            bool transformChanged = false;

            if( ImGui::DragFloat2(
                "Position",
                position,
                0.05f,
                -100.0f,
                100.0f,
                "%.2f" ) )
            {
                transform.position =
                {
                    position[0],
                    position[1]
                };

                transformChanged = true;
            }

            float rotation =
                std::atan2(
                    transform.rotation.s,
                    transform.rotation.c
                );

            if( ImGui::DragFloat(
                "Rotation",
                &rotation,
                0.01f,
                -3.14159265f,
                3.14159265f,
                "%.3f rad" ) )
            {
                transform.rotation =
                    rot2::FromRadians(
                        rotation
                    );

                transformChanged = true;
            }

            if( transformChanged )
            {
                scene->world.SetBodyTransform(
                    visual.bodyHandle,
                    transform
                );

                RefreshContacts(
                    *scene,
                    contacts
                );
            }

            if( bodyRef.type != bodyType::Static )
            {
                vec2 linearVelocity =
                    scene->world.GetBodyLinearVelocity(
                        visual.bodyHandle
                    );

                float velocity[2]
                {
                    linearVelocity.x,
                    linearVelocity.y
                };

                if( ImGui::DragFloat2(
                    "Linear velocity",
                    velocity,
                    0.05f,
                    -100.0f,
                    100.0f,
                    "%.2f" ) )
                {
                    scene->world.SetBodyLinearVelocity(
                        visual.bodyHandle,
                        {
                            velocity[0],
                            velocity[1]
                        }
                    );
                }

                float angularVelocity =
                    scene->world.GetBodyAngularVelocity(
                        visual.bodyHandle
                    );

                if( ImGui::DragFloat(
                    "Angular velocity",
                    &angularVelocity,
                    0.05f,
                    -100.0f,
                    100.0f,
                    "%.2f rad/s" ) )
                {
                    scene->world.SetBodyAngularVelocity(
                        visual.bodyHandle,
                        angularVelocity
                    );
                }

                float linearDamping =
                    scene->world.GetBodyLinearDamping(
                        visual.bodyHandle
                    );

                if( ImGui::DragFloat(
                    "Linear damping",
                    &linearDamping,
                    0.05f,
                    0.0f,
                    20.0f,
                    "%.2f" ) )
                {
                    scene->world.SetBodyLinearDamping(
                        visual.bodyHandle,
                        linearDamping
                    );
                }

                float angularDamping =
                    scene->world.GetBodyAngularDamping(
                        visual.bodyHandle
                    );

                if( ImGui::DragFloat(
                    "Angular damping",
                    &angularDamping,
                    0.05f,
                    0.0f,
                    20.0f,
                    "%.2f" ) )
                {
                    scene->world.SetBodyAngularDamping(
                        visual.bodyHandle,
                        angularDamping
                    );
                }

                float gravityScale =
                    scene->world.GetBodyGravityScale(
                        visual.bodyHandle
                    );

                if( ImGui::DragFloat(
                    "Gravity scale",
                    &gravityScale,
                    0.05f,
                    -10.0f,
                    10.0f,
                    "%.2f" ) )
                {
                    scene->world.SetBodyGravityScale(
                        visual.bodyHandle,
                        gravityScale
                    );
                }

                bool awake =
                    scene->world.IsBodyAwake(
                        visual.bodyHandle
                    );

                if( ImGui::Checkbox(
                    "Awake",
                    &awake ) )
                {
                    scene->world.SetBodyAwake(
                        visual.bodyHandle,
                        awake
                    );
                }

                bool sleepEnabled =
                    scene->world.IsBodySleepEnabled(
                        visual.bodyHandle
                    );

                if( ImGui::Checkbox(
                    "Body sleep",
                    &sleepEnabled ) )
                {
                    scene->world.SetBodySleepEnabled(
                        visual.bodyHandle,
                        sleepEnabled
                    );
                }

                float sleepThreshold =
                    scene->world.GetBodySleepThreshold(
                        visual.bodyHandle
                    );

                if( ImGui::DragFloat(
                    "Sleep threshold",
                    &sleepThreshold,
                    0.005f,
                    0.0f,
                    5.0f,
                    "%.3f m/s" ) )
                {
                    scene->world.SetBodySleepThreshold(
                        visual.bodyHandle,
                        sleepThreshold
                    );
                }

                float safetyFactor =
                    scene->world.GetBodySafetyFactor(
                        visual.bodyHandle
                    );

                if( ImGui::DragFloat(
                    "CCD safety factor",
                    &safetyFactor,
                    0.01f,
                    0.01f,
                    2.0f,
                    "%.2f" ) )
                {
                    scene->world.SetBodySafetyFactor(
                        visual.bodyHandle,
                        safetyFactor
                    );
                }

                bool contactRecycling =
                    scene->world.IsBodyContactRecyclingEnabled(
                        visual.bodyHandle
                    );

                if( ImGui::Checkbox(
                    "Contact recycling",
                    &contactRecycling ) )
                {
                    scene->world.SetBodyContactRecyclingEnabled(
                        visual.bodyHandle,
                        contactRecycling
                    );
                }

                bool bullet =
                    scene->world.IsBodyBullet(
                        visual.bodyHandle
                    );

                if( ImGui::Checkbox(
                    "Bullet",
                    &bullet ) )
                {
                    scene->world.SetBodyBullet(
                        visual.bodyHandle,
                        bullet
                    );
                }

                ImGui::Text(
                    "Fast body: %s",
                    scene->world.IsBodyFast(
                        visual.bodyHandle
                    ) ? "yes" : "no"
                );

                ImGui::Text(
                    "TOI this step: %s",
                    scene->world.HadBodyTimeOfImpact(
                        visual.bodyHandle
                    ) ? "yes" : "no"
                );

                bool fastRotation =
                    scene->world.IsBodyFastRotationAllowed(
                        visual.bodyHandle
                    );

                if( ImGui::Checkbox(
                    "Allow fast rotation",
                    &fastRotation ) )
                {
                    scene->world.SetBodyFastRotationAllowed(
                        visual.bodyHandle,
                        fastRotation
                    );
                }
            }

            float density =
                scene->world.GetShapeDensity(
                    visual.shapeHandle
                );

            if( ImGui::DragFloat(
                "Density",
                &density,
                0.05f,
                0.0f,
                100.0f,
                "%.2f" ) )
            {
                scene->world.SetShapeDensity(
                    visual.shapeHandle,
                    density
                );
            }

            float friction =
                scene->world.GetShapeFriction(
                    visual.shapeHandle
                );

            if( ImGui::DragFloat(
                "Friction",
                &friction,
                0.02f,
                0.0f,
                5.0f,
                "%.2f" ) )
            {
                scene->world.SetShapeFriction(
                    visual.shapeHandle,
                    friction
                );
            }

            float restitution =
                scene->world.GetShapeRestitution(
                    visual.shapeHandle
                );

            if( ImGui::DragFloat(
                "Restitution",
                &restitution,
                0.02f,
                0.0f,
                2.0f,
                "%.2f" ) )
            {
                scene->world.SetShapeRestitution(
                    visual.shapeHandle,
                    restitution
                );
            }

            collisionFilter filter =
                scene->world.GetShapeFilter(
                    visual.shapeHandle
                );

            bool filterChanged = false;

            filterChanged =
                ImGui::InputScalar(
                    "Category bits",
                    ImGuiDataType_U64,
                    &filter.categoryBits
                ) ||
                filterChanged;

            filterChanged =
                ImGui::InputScalar(
                    "Mask bits",
                    ImGuiDataType_U64,
                    &filter.maskBits
                ) ||
                filterChanged;

            filterChanged =
                ImGui::InputInt(
                    "Group index",
                    &filter.groupIndex
                ) ||
                filterChanged;

            if( filterChanged )
            {
                scene->world.SetShapeFilter(
                    visual.shapeHandle,
                    filter
                );

                RefreshContacts(
                    *scene,
                    contacts
                );
            }

            ImGui::Text(
                "Mass: %.3f",
                scene->world.GetBodyMass(
                    visual.bodyHandle
                )
            );

            ImGui::Text(
                "Inertia: %.3f",
                scene->world.GetBodyRotationalInertia(
                    visual.bodyHandle
                )
            );
        }

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
            "Shape AABBs",
            &showShapeAABBs
        );
        ImGui::Checkbox(
            "Fat AABBs",
            &showFatAABBs
        );
        ImGui::Checkbox(
            "Contact points",
            &showContacts
        );
        ImGui::Checkbox(
            "Contact details",
            &showContactDetails
        );
        ImGui::Checkbox(
            "Center of mass",
            &showCOM
        );
        ImGui::Checkbox(
            "Velocity vectors",
            &showVelocities
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

        const broadPhase& phase =
            scene->world.GetBroadPhase();

        const dynamicTree& dynamicTreeRef =
            phase.GetTree(
                bodyType::Dynamic
            );

        const dynamicTree& kinematicTree =
            phase.GetTree(
                bodyType::Kinematic
            );

        const dynamicTree& staticTree =
            phase.GetTree(
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
            "Recycled contacts: %zu",
            scene->world.GetRecycledContactCount()
        );

        ImGui::Text(
            "Dynamic tree: %zu / h%d",
            dynamicTreeRef.GetProxyCount(),
            dynamicTreeRef.GetHeight()
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
            ImVec4( 0.45f, 0.9f, 0.55f, 1.0f ),
            "Contact Solver: active"
        );

        ImGui::TextWrapped(
            "Contact labels: s = separation, "
            "Jn = normal impulse, Jt = friction impulse."
        );

        ImGui::TextWrapped(
            "Cyan arrows show body velocity. "
            "Dim blue Dynamic bodies are sleeping."
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
                dynamicTreeRef,
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

        constexpr ImU32 FAT_AABB_COLOR =
            IM_COL32( 255, 185, 80, 190 );

        constexpr ImU32 LABEL_COLOR =
            IM_COL32( 230, 232, 238, 255 );

        constexpr ImU32 COM_COLOR =
            IM_COL32( 255, 90, 180, 255 );

        constexpr ImU32 VELOCITY_COLOR =
            IM_COL32( 80, 220, 255, 245 );

        for( const visualShape& visual : scene->shapes )
        {
            const body& bodyRef =
                scene->world.GetBody(
                    visual.bodyHandle
                );

            const shape& shapeRef =
                scene->world.GetShape(
                    visual.shapeHandle
                );

            const transform2 transform =
                scene->world.GetBodyTransform(
                    visual.bodyHandle
                );

            const shapeColors colors =
                GetShapeColors(
                    bodyRef.type,
                    bodyRef.awake
                );

            debugDraw.DrawShape(
                shapeRef.geometry,
                transform,
                colors.outline,
                colors.fill
            );

            if( showShapeAABBs )
            {
                debugDraw.DrawAABB(
                    scene->world.GetShapeAABB(
                        visual.shapeHandle
                    ),
                    AABB_COLOR
                );
            }

            if( showFatAABBs )
            {
                debugDraw.DrawAABB(
                    scene->world.GetShapeFatAABB(
                        visual.shapeHandle
                    ),
                    FAT_AABB_COLOR
                );
            }

            const vec2 worldCenter =
                GetWorldCenter(
                    scene->world,
                    visual.bodyHandle
                );

            if( showCOM &&
                bodyRef.type != bodyType::Static )
            {
                debugDraw.DrawPoint(
                    worldCenter,
                    COM_COLOR,
                    4.5f
                );
            }

            if( showVelocities &&
                bodyRef.type != bodyType::Static )
            {
                const vec2 velocity =
                    scene->world.GetBodyLinearVelocity(
                        visual.bodyHandle
                    );

                if( LengthSquared( velocity ) > 1e-6f )
                {
                    const float arrowLength =
                        std::clamp(
                            Length( velocity ) * 0.18f,
                            0.2f,
                            2.0f
                        );

                    debugDraw.DrawArrow(
                        worldCenter,
                        velocity,
                        VELOCITY_COLOR,
                        arrowLength
                    );
                }
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

            constexpr ImU32 PENETRATION_COLOR =
                IM_COL32( 255, 105, 90, 255 );

            constexpr ImU32 NORMAL_COLOR =
                IM_COL32( 100, 255, 140, 255 );

            constexpr ImU32 CONTACT_TEXT_COLOR =
                IM_COL32( 245, 245, 245, 255 );

            for( const contactData& contact : contacts )
            {
                for( int i = 0;
                     i < contact.manifold.pointCount;
                     ++i )
                {
                    const manifoldPoint2& point =
                        contact.manifold.points[i];

                    const ImU32 pointColor =
                        point.separation < -0.01f
                            ? PENETRATION_COLOR
                            : CONTACT_COLOR;

                    debugDraw.DrawPoint(
                        point.point,
                        pointColor,
                        5.0f
                    );

                    debugDraw.DrawArrow(
                        point.point,
                        contact.manifold.normal,
                        NORMAL_COLOR,
                        0.7f
                    );

                    if( showContactDetails )
                    {
                        char label[96]{};

                        std::snprintf(
                            label,
                            sizeof( label ),
                            "s %.3f  Jn %.2f  Jt %.2f",
                            point.separation,
                            point.normalImpulse,
                            point.tangentImpulse
                        );

                        debugDraw.DrawLabel(
                            point.point,
                            label,
                            CONTACT_TEXT_COLOR
                        );
                    }
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
