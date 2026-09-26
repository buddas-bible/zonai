#include "debugCamera.h"

#include <algorithm>

namespace zonai::sandbox
{

ImVec2 DebugCamera::WorldToScreen(
    const vec2& world,
    const ImVec2& viewportMin,
    const ImVec2& viewportSize ) const
{
    const float centerX = viewportMin.x + viewportSize.x * 0.5f;
    const float centerY = viewportMin.y + viewportSize.y * 0.5f;

    return
    {
        centerX + ( world.x - center.x ) * pixelsPerMeter,
        centerY - ( world.y - center.y ) * pixelsPerMeter
    };
}

vec2 DebugCamera::ScreenToWorld(
    const ImVec2& screen,
    const ImVec2& viewportMin,
    const ImVec2& viewportSize ) const
{
    const float centerX = viewportMin.x + viewportSize.x * 0.5f;
    const float centerY = viewportMin.y + viewportSize.y * 0.5f;

    return
    {
        center.x + ( screen.x - centerX ) / pixelsPerMeter,
        center.y - ( screen.y - centerY ) / pixelsPerMeter
    };
}

void DebugCamera::Zoom( float wheelDelta )
{
    constexpr float ZOOM_STEP = 1.1f;
    constexpr float MIN_PIXELS_PER_METER = 10.0f;
    constexpr float MAX_PIXELS_PER_METER = 240.0f;

    if( wheelDelta > 0.0f )
    {
        pixelsPerMeter *= ZOOM_STEP;
    }
    else if( wheelDelta < 0.0f )
    {
        pixelsPerMeter /= ZOOM_STEP;
    }

    pixelsPerMeter =
        std::clamp( pixelsPerMeter, MIN_PIXELS_PER_METER, MAX_PIXELS_PER_METER );
}

void DebugCamera::PanPixels( const ImVec2& delta )
{
    center.x -= delta.x / pixelsPerMeter;
    center.y += delta.y / pixelsPerMeter;
}

} // namespace zonai::sandbox
