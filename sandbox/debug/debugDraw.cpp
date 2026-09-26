#include "debugDraw.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace zonai::sandbox
{

DebugDraw::DebugDraw(
    ImDrawList* drawList,
    const DebugCamera& camera,
    const ImVec2& viewportMin,
    const ImVec2& viewportSize )
    : drawList_( drawList ),
      camera_( camera ),
      viewportMin_( viewportMin ),
      viewportSize_( viewportSize )
{
}

void DebugDraw::DrawGrid( float spacing ) const
{
    if( spacing <= 0.0f )
    {
        return;
    }

    const ImVec2 viewportMax{
        viewportMin_.x + viewportSize_.x,
        viewportMin_.y + viewportSize_.y
    };

    const vec2 worldTopLeft =
        camera_.ScreenToWorld( viewportMin_, viewportMin_, viewportSize_ );
    const vec2 worldBottomRight =
        camera_.ScreenToWorld( viewportMax, viewportMin_, viewportSize_ );

    const float minX = std::min( worldTopLeft.x, worldBottomRight.x );
    const float maxX = std::max( worldTopLeft.x, worldBottomRight.x );
    const float minY = std::min( worldTopLeft.y, worldBottomRight.y );
    const float maxY = std::max( worldTopLeft.y, worldBottomRight.y );

    const float startX = std::floor( minX / spacing ) * spacing;
    const float startY = std::floor( minY / spacing ) * spacing;

    constexpr ImU32 GRID_COLOR = IM_COL32( 62, 66, 78, 120 );
    constexpr ImU32 X_AXIS_COLOR = IM_COL32( 210, 90, 90, 230 );
    constexpr ImU32 Y_AXIS_COLOR = IM_COL32( 90, 200, 120, 230 );

    constexpr int MAX_GRID_LINES = 256;

    int lineCount = 0;

    for( float x = startX; x <= maxX && lineCount < MAX_GRID_LINES; x += spacing, ++lineCount )
    {
        const ImVec2 a = ToScreen( { x, minY } );
        const ImVec2 b = ToScreen( { x, maxY } );

        const bool isAxis = std::abs( x ) < spacing * 0.001f;
        drawList_->AddLine( a, b, isAxis ? Y_AXIS_COLOR : GRID_COLOR, isAxis ? 2.0f : 1.0f );
    }

    lineCount = 0;

    for( float y = startY; y <= maxY && lineCount < MAX_GRID_LINES; y += spacing, ++lineCount )
    {
        const ImVec2 a = ToScreen( { minX, y } );
        const ImVec2 b = ToScreen( { maxX, y } );

        const bool isAxis = std::abs( y ) < spacing * 0.001f;
        drawList_->AddLine( a, b, isAxis ? X_AXIS_COLOR : GRID_COLOR, isAxis ? 2.0f : 1.0f );
    }
}

void DebugDraw::DrawAABB(
    const aabb2& box,
    ImU32 color,
    float thickness ) const
{
    const ImVec2 min = ToScreen( { box.min.x, box.max.y } );
    const ImVec2 max = ToScreen( { box.max.x, box.min.y } );

    drawList_->AddRect( min, max, color, 0.0f, 0, thickness );
}

void DebugDraw::DrawSegment(
    const segment2& segment,
    ImU32 color,
    float thickness ) const
{
    drawList_->AddLine(
        ToScreen( segment.a ),
        ToScreen( segment.b ),
        color,
        thickness
    );
}

void DebugDraw::DrawCircle(
    const circle2& circle,
    ImU32 outlineColor,
    ImU32 fillColor ) const
{
    const ImVec2 center = ToScreen( circle.center );
    const float radius = circle.radius * camera_.pixelsPerMeter;

    drawList_->AddCircleFilled( center, radius, fillColor );
    drawList_->AddCircle( center, radius, outlineColor, 0, 2.0f );
}

void DebugDraw::DrawCapsule(
    const capsule2& capsule,
    ImU32 outlineColor,
    ImU32 fillColor ) const
{
    const ImVec2 center1 = ToScreen( capsule.center1 );
    const ImVec2 center2 = ToScreen( capsule.center2 );
    const float radius = capsule.radius * camera_.pixelsPerMeter;

    // 굵은 axis와 양 끝 원으로 capsule 내부를 채움.
    drawList_->AddLine( center1, center2, fillColor, radius * 2.0f );
    drawList_->AddCircleFilled( center1, radius, fillColor );
    drawList_->AddCircleFilled( center2, radius, fillColor );

    const vec2 axis = capsule.center2 - capsule.center1;
    const float axisLength = Length( axis );

    if( axisLength == 0.0f )
    {
        drawList_->AddCircle( center1, radius, outlineColor, 0, 2.0f );
        return;
    }

    const vec2 normal{
        -axis.y / axisLength * capsule.radius,
        axis.x / axisLength * capsule.radius
    };

    drawList_->AddLine(
        ToScreen( capsule.center1 + normal ),
        ToScreen( capsule.center2 + normal ),
        outlineColor,
        2.0f
    );

    drawList_->AddLine(
        ToScreen( capsule.center1 - normal ),
        ToScreen( capsule.center2 - normal ),
        outlineColor,
        2.0f
    );

    drawList_->AddCircle( center1, radius, outlineColor, 0, 2.0f );
    drawList_->AddCircle( center2, radius, outlineColor, 0, 2.0f );
}

void DebugDraw::DrawPolygon(
    const polygon2& polygon,
    ImU32 outlineColor,
    ImU32 fillColor ) const
{
    if( polygon.vertexCount < 2 )
    {
        return;
    }

    std::array<ImVec2, MAX_POLYGON_VERTICES> vertices{};

    for( int i = 0; i < polygon.vertexCount; ++i )
    {
        vertices[i] = ToScreen( polygon.vertices[i] );
    }

    if( polygon.vertexCount >= 3 )
    {
        drawList_->AddConvexPolyFilled(
            vertices.data(),
            polygon.vertexCount,
            fillColor
        );
    }

    drawList_->AddPolyline(
        vertices.data(),
        polygon.vertexCount,
        outlineColor,
        ImDrawFlags_Closed,
        2.0f
    );
}

void DebugDraw::DrawLabel(
    const vec2& worldPosition,
    const char* text,
    ImU32 color ) const
{
    const ImVec2 screen = ToScreen( worldPosition );

    drawList_->AddText(
        { screen.x + 8.0f, screen.y - 18.0f },
        color,
        text
    );
}

ImVec2 DebugDraw::ToScreen( const vec2& world ) const
{
    return camera_.WorldToScreen( world, viewportMin_, viewportSize_ );
}

} // namespace zonai::sandbox
