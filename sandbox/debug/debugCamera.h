#pragma once

#include <imgui.h>

#include "math/vec2.h"

namespace zonai::sandbox
{

// Physics world 좌표와 ImGui canvas의 screen 좌표를 서로 변환함.
struct DebugCamera
{
    vec2 center{};
    float pixelsPerMeter = 60.0f;

    ImVec2 WorldToScreen(
        const vec2& world,
        const ImVec2& viewportMin,
        const ImVec2& viewportSize ) const;

    vec2 ScreenToWorld(
        const ImVec2& screen,
        const ImVec2& viewportMin,
        const ImVec2& viewportSize ) const;

    // mouse wheel 입력을 받아 world 확대 / 축소 비율을 조절함.
    void Zoom( float wheelDelta );

    // screen pixel 이동량을 world camera 이동량으로 변환함.
    void PanPixels( const ImVec2& delta );
};

} // namespace zonai::sandbox
