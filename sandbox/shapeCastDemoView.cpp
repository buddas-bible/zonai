#include "shapeCastDemoView.h"

#include <cstdio>

#include <imgui.h>

#include "debug/debugDraw.h"
#include "shapeCastDemo.h"

namespace zonai::sandbox
{
namespace
{

class shapeCastDemoView final : public shapeCastDemo
{
public:
    void drawControls() override
    {
        ImGui::TextUnformatted( "도형 이동 충돌 조회" );
        ImGui::TextWrapped( "이동 도형은 물리적으로 배치되지 않습니다. 시작·종료 형태와 가장 가까운 충돌 지점을 미리 봅니다." );

        int probe = static_cast<int>( getProbe() );
        if( ImGui::Combo( "이동 도형###ShapeCastProbe", &probe, "원\0캡슐\0상자\0" ) )
            setProbe( static_cast<shapeCastProbe>( probe ) );

        vec2 start = getInput().transform.position;
        if( ImGui::DragFloat2( "시작점 (m)###ShapeCastStart", &start.x, 0.05f, -10.0f, 10.0f ) ) setStart( start );

        vec2 end = getInput().transform.position + getInput().translation;
        if( ImGui::DragFloat2( "도착점 (m)###ShapeCastEnd", &end.x, 0.05f, -10.0f, 10.0f ) ) setTarget( end );

        bool sensor = isSensorIncluded();
        if( ImGui::Checkbox( "센서 포함###ShapeCastSensor", &sensor ) ) setSensorIncluded( sensor );

        bool encroach = getInput().canEncroach;
        if( ImGui::Checkbox( "초기 근접 허용###ShapeCastEncroach", &encroach ) ) setCanEncroach( encroach );

        bool showRay = isRayVisible();
        if( ImGui::Checkbox( "RayCast 비교 표시###ShapeCastRayVisible", &showRay ) ) setRayVisible( showRay );

        bool showSweep = isSweepAABBVisible();
        if( ImGui::Checkbox( "이동 AABB 후보 표시###ShapeCastAABBVisible", &showSweep ) ) setSweepAABBVisible( showSweep );

        if( ImGui::Button( "미리보기 처음부터###ShapeCastResetPreview" ) ) resetPreview();
        ImGui::TextWrapped( "캔버스 왼쪽 클릭: 이동 목표점 / Shift+왼쪽 클릭: 시작점. 재생/1프레임: 미리보기 이동. 상단 초기화는 경로를 복원합니다." );

        const auto hit = getHit();
        if( hit.hit )
        {
            const char* label = "알 수 없음";
            for( const auto& target : getTargets() )
                if( target.shape == hit.shape ) label = target.name;

            ImGui::Text( "첫 충돌: %s / 이동 비율 %.4f", label, hit.fraction );
            ImGui::Text( "충돌 지점: (%.3f, %.3f) m", hit.point.x, hit.point.y );
            ImGui::Text( "표면 법선: (%.3f, %.3f)", hit.normal.x, hit.normal.y );
        }
        else
        {
            ImGui::TextUnformatted( "현재 경로에는 충돌이 없습니다." );
        }
        const auto rayHit = getRayHit();
        if( rayHit.hit )
            ImGui::Text( "RayCast 첫 충돌: t=%.4f / 지점 (%.3f, %.3f) m", rayHit.fraction, rayHit.point.x, rayHit.point.y );
        else
            ImGui::TextUnformatted( "RayCast: 중심 광선과 교차하지 않음" );

        ImGui::Text( "이동 AABB 후보: %zu개 (정확한 충돌 수가 아님)", getAABBCandidateCount() );
        ImGui::Text( "미리보기 진행: %.1f%%", 100.0f * getPreviewFraction() );
        ImGui::TextWrapped( "초록: 정적 대상 / 보라: 센서 / 흰색: 시작 / 회색: 전체 끝점 / 노랑: 현재 위치 / 파랑: ShapeCast 충돌 / 주황: 법선 / 분홍: RayCast / 청록: 이동 AABB" );
    }

    void draw( debugDraw& draw ) const override
    {
        constexpr ImU32 staticColor = IM_COL32( 105, 225, 155, 255 );
        constexpr ImU32 sensorColor = IM_COL32( 205, 145, 255, 255 );
        constexpr ImU32 startColor = IM_COL32( 245, 245, 245, 255 );
        constexpr ImU32 endColor = IM_COL32( 140, 150, 165, 170 );
        constexpr ImU32 progressColor = IM_COL32( 250, 215, 75, 255 );
        constexpr ImU32 hitColor = IM_COL32( 75, 170, 255, 255 );
        constexpr ImU32 normalColor = IM_COL32( 255, 130, 75, 255 );
        constexpr ImU32 rayColor = IM_COL32( 255, 110, 205, 255 );
        constexpr ImU32 boundsColor = IM_COL32( 75, 230, 215, 180 );

        draw.DrawGrid();

        const world& simulation = getWorld();
        for( const auto& target : getTargets() )
        {
            if( !simulation.IsValid( target.shape ) ) continue;

            const auto& stored = simulation.GetShape( target.shape );
            const auto pose = simulation.GetBodyTransform( target.body );
            const bool sensor = simulation.IsShapeSensor( target.shape );
            draw.DrawShape( stored.geometry, pose, sensor ? sensorColor : staticColor, sensor ? IM_COL32( 205, 145, 255, 40 ) : IM_COL32( 105, 225, 155, 50 ) );
            draw.DrawLabel( pose.position + vec2{ 0.0f, 1.05f }, target.name, sensor ? sensorColor : staticColor );
        }

        const worldShapeCastInput2& input = getInput();
        const vec2 start = input.transform.position;
        const vec2 end = start + input.translation;

        if( isSweepAABBVisible() ) draw.DrawAABB( getSweepAABB(), boundsColor, 1.5f );

        draw.DrawSegment( { start, end }, endColor, 2.0f );
        draw.DrawPoint( end, endColor, 4.0f );

        if( isRayVisible() )
        {
            const worldRayCastResult2 ray = getRayHit();
            if( ray.hit )
            {
                draw.DrawSegment( { start, ray.point }, rayColor, 2.5f );
                draw.DrawPoint( ray.point, rayColor, 5.0f );
                if( LengthSquared( ray.normal ) > 0.0f ) draw.DrawArrow( ray.point, ray.normal, rayColor, 0.55f );

                char rayLabel[48];
                std::snprintf( rayLabel, sizeof( rayLabel ), "Ray t = %.3f", ray.fraction );
                draw.DrawLabel( ray.point + vec2{ 0.15f, 0.45f }, rayLabel, rayColor );
            }
        }
        draw.DrawShape( input.geometry, input.transform, startColor, IM_COL32( 245, 245, 245, 24 ) );
        draw.DrawShape( input.geometry, { end, input.transform.rotation }, endColor, 0 );

        const worldShapeCastResult2 hit = getHit();
        if( hit.hit )
        {
            const transform2 atHit{ start + input.translation * hit.fraction, input.transform.rotation };
            draw.DrawShape( input.geometry, atHit, hitColor, IM_COL32( 75, 170, 255, 35 ) );
            draw.DrawPoint( hit.point, normalColor, 5.0f );
            if( LengthSquared( hit.normal ) > 0.0f ) draw.DrawArrow( hit.point, hit.normal, normalColor, 0.8f );

            char fractionLabel[48];
            std::snprintf( fractionLabel, sizeof( fractionLabel ), "t = %.3f", hit.fraction );
            draw.DrawLabel( hit.point + vec2{ 0.2f, -0.65f }, fractionLabel, normalColor );
        }

        const transform2 current{ start + input.translation * getPreviewFraction(), input.transform.rotation };
        draw.DrawShape( input.geometry, current, progressColor, IM_COL32( 250, 215, 75, 30 ) );
    }
};

} // namespace

std::unique_ptr<demo> createShapeCastDemoView()
{
    return std::make_unique<shapeCastDemoView>();
}

} // namespace zonai::sandbox
