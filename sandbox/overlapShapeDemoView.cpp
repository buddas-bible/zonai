#include "overlapShapeDemoView.h"

#include <imgui.h>

#include "debug/debugDraw.h"
#include "overlapShapeDemo.h"

namespace zonai::sandbox
{
namespace
{

class overlapShapeDemoView final : public overlapShapeDemo
{
public:
    void drawControls() override
    {
        ImGui::TextUnformatted( "AABB 후보 · 실제 도형 겹침" );
        ImGui::TextWrapped( "AABB 후보는 충돌 확정이 아닙니다. GJK가 실제 도형 거리까지 확인한 결과와 비교하세요." );

        int probe = static_cast<int>( getProbe() );
        if( ImGui::Combo( "조회 도형###OverlapProbe", &probe, "원\0캡슐\0상자\0" ) )
            setProbe( static_cast<overlapProbe>( probe ) );

        vec2 position = getQueryTransform().position;
        if( ImGui::DragFloat2( "조회 중심 (m)###OverlapPosition", &position.x, 0.025f, -10.0f, 10.0f ) )
            setQueryPosition( position );

        float rotation = getQueryRotation();
        if( ImGui::SliderAngle( "조회 회전###OverlapRotation", &rotation, -180.0f, 180.0f ) )
            setQueryRotation( rotation );

        bool sensor = isSensorIncluded();
        if( ImGui::Checkbox( "센서 포함###OverlapSensor", &sensor ) ) setSensorIncluded( sensor );

        bool showAABBs = areCandidateAABBsVisible();
        if( ImGui::Checkbox( "후보 AABB 표시###OverlapShowAABBs", &showAABBs ) )
            setCandidateAABBsVisible( showAABBs );

        ImGui::TextWrapped( "캔버스 왼쪽 클릭: 조회 도형을 클릭한 위치로 이동. 물리 Step이나 Body 위치는 바뀌지 않습니다." );

        ImGui::Separator();
        ImGui::Text( "BroadPhase 후보: %zu", getCandidates().size() );
        ImGui::Text( "실제 도형 겹침: %zu", getExactOverlaps().size() );
        ImGui::Text( "AABB만 겹침: %zu", getCandidates().size() - getExactOverlaps().size() );

        for( const auto& target : getTargets() )
        {
            if( isExactOverlap( target.shape ) )
                ImGui::Text( "%s: 실제 겹침", target.name );
            else if( isCandidate( target.shape ) )
                ImGui::Text( "%s: AABB 후보만", target.name );
            else
                ImGui::Text( "%s: 범위 밖", target.name );
        }

        ImGui::TextWrapped( "초록: 실제 겹침 / 주황: AABB 후보만 / 회색: 후보 아님 / 청록: 조회 AABB / 흰색: 조회 도형. 대상의 후보 AABB는 speculative margin이 포함된 저장 AABB입니다." );
    }

    void draw( debugDraw& draw ) const override
    {
        constexpr ImU32 exactColor = IM_COL32( 90, 240, 155, 255 );
        constexpr ImU32 candidateColor = IM_COL32( 255, 185, 75, 255 );
        constexpr ImU32 idleColor = IM_COL32( 115, 135, 150, 210 );
        constexpr ImU32 queryColor = IM_COL32( 240, 245, 250, 255 );
        constexpr ImU32 queryBoundsColor = IM_COL32( 75, 225, 230, 255 );

        draw.DrawGrid();

        const world& simulation = getWorld();

        for( const auto& target : getTargets() )
        {
            if( !simulation.IsValid( target.shape ) || !simulation.IsValid( target.body ) ) continue;

            const auto& geometry = simulation.GetShape( target.shape ).geometry;
            const transform2 pose = simulation.GetBodyTransform( target.body );
            const bool exact = isExactOverlap( target.shape );
            const bool candidate = isCandidate( target.shape );
            const ImU32 color = exact ? exactColor : candidate ? candidateColor : idleColor;

            draw.DrawShape( geometry, pose, color, exact ? IM_COL32( 90, 240, 155, 55 ) : 0 );
            if( areCandidateAABBsVisible() && candidate )
                draw.DrawAABB( simulation.GetShape( target.shape ).aabb, color, 1.5f );

            draw.DrawLabel( pose.position + vec2{ 0.0f, 1.1f }, target.name, color );
        }

        draw.DrawShape( getQueryGeometry(), getQueryTransform(), queryColor, IM_COL32( 240, 245, 250, 25 ) );
        draw.DrawAABB( getQueryAABB(), queryBoundsColor, 2.0f );
        draw.DrawPoint( getQueryTransform().position, queryColor, 4.0f );
    }
};

} // namespace

std::unique_ptr<demo> createOverlapShapeDemoView()
{
    return std::make_unique<overlapShapeDemoView>();
}

} // namespace zonai::sandbox
