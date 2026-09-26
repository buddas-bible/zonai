#pragma once

#include <imgui.h>

#include "debugCamera.h"

#include "collision/aabb2.h"
#include "collision/broadphase/dynamicTree.h"
#include "geometry/capsule2.h"
#include "geometry/circle2.h"
#include "geometry/polygon2.h"
#include "geometry/segment2.h"

namespace zonai::sandbox
{

// Zonai의 geometry / broad-phase 데이터를 ImGui canvas에 그리는 debug 전용 renderer.
class DebugDraw
{
public:
    DebugDraw(
        ImDrawList* drawList,
        const DebugCamera& camera,
        const ImVec2& viewportMin,
        const ImVec2& viewportSize );

    void DrawGrid( float spacing = 1.0f ) const;

    void DrawAABB(
        const aabb2& box,
        ImU32 color,
        float thickness = 1.0f ) const;

    // DynamicTree의 live node AABB와 debug metadata를 world 위에 표시함.
    void DrawTree(
        const DynamicTree& tree,
        const char* treeName,
        bool showLeaves,
        bool showInternal,
        bool showLabels,
        ImU32 leafColor,
        ImU32 internalColor ) const;

    void DrawSegment(
        const segment2& segment,
        ImU32 color,
        float thickness = 2.0f ) const;

    void DrawPoint(
        const vec2& point,
        ImU32 color,
        float radiusPixels = 4.0f ) const;

    void DrawArrow(
        const vec2& start,
        const vec2& direction,
        ImU32 color,
        float length = 1.0f ) const;

    void DrawCircle(
        const circle2& circle,
        ImU32 outlineColor,
        ImU32 fillColor ) const;

    void DrawCapsule(
        const capsule2& capsule,
        ImU32 outlineColor,
        ImU32 fillColor ) const;

    void DrawPolygon(
        const polygon2& polygon,
        ImU32 outlineColor,
        ImU32 fillColor ) const;

    void DrawLabel(
        const vec2& worldPosition,
        const char* text,
        ImU32 color ) const;

private:
    ImVec2 ToScreen( const vec2& world ) const;

    ImDrawList* drawList_ = nullptr;
    const DebugCamera& camera_;
    ImVec2 viewportMin_{};
    ImVec2 viewportSize_{};
};

} // namespace zonai::sandbox
