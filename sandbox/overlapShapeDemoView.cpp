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
        ImGui::TextUnformatted( "AABB 후보 · 실제 도형 / 점 포함 비교" );

        int mode = static_cast<int>( getQueryMode() );
        if( ImGui::Combo( "조회 방식###OverlapMode", &mode, "도형 겹침\0점 포함\0" ) )
            setQueryMode( static_cast<overlapQueryMode>( mode ) );

        const bool pointMode = getQueryMode() == overlapQueryMode::point;
        ImGui::TextWrapped( pointMode ?
            "점의 AABB로 후보를 찾고 실제 도형 내부에 점이 들어가는지 검사합니다. 도형 반경이나 GJK 접촉과는 다릅니다." :
            "도형의 AABB 후보 중 GJK가 실제 겹침까지 확인한 결과를 비교합니다." );

        ImGui::BeginDisabled( pointMode );
        int probe = static_cast<int>( getProbe() );
        if( ImGui::Combo( "조회 도형###OverlapProbe", &probe, "원\0캡슐\0상자\0" ) )
            setProbe( static_cast<overlapProbe>( probe ) );
        ImGui::EndDisabled();

        vec2 position = getQueryTransform().position;
        if( ImGui::DragFloat2( "조회 위치 (m)###OverlapPosition", &position.x, 0.025f, -10.0f, 10.0f ) )
            setQueryPosition( position );

        ImGui::BeginDisabled( pointMode );
        float rotation = getQueryRotation();
        if( ImGui::SliderAngle( "조회 회전###OverlapRotation", &rotation, -180.0f, 180.0f ) )
            setQueryRotation( rotation );
        ImGui::EndDisabled();

        bool sensor = isSensorIncluded();
        if( ImGui::Checkbox( "센서 포함###OverlapSensor", &sensor ) ) setSensorIncluded( sensor );

        bool showLines = hasLineTargets();
        if( ImGui::Checkbox( "선분 / Chain 대상 표시###OverlapLineTargets", &showLines ) )
            setLineTargetsEnabled( showLines );

        if( showLines )
            ImGui::TextWrapped( "아래쪽 두 선분은 면적이 없습니다. 도형 모드는 양쪽에서 겹침을 찾지만, 점 모드는 선 위에서도 실제 포함으로 보지 않습니다." );

        bool showAABBs = areCandidateAABBsVisible();
        if( ImGui::Checkbox( "후보 AABB 표시###OverlapShowAABBs", &showAABBs ) )
            setCandidateAABBsVisible( showAABBs );

        ImGui::TextWrapped( "캔버스 왼쪽 클릭: 조회 위치 변경. 물리 Step이나 Body 위치는 바뀌지 않습니다." );

        ImGui::Separator();
        ImGui::Text( "BroadPhase 후보: %zu", getCandidates().size() );
        ImGui::Text( pointMode ? "점 포함: %zu" : "실제 도형 겹침: %zu", getExactOverlaps().size() );
        ImGui::Text( "AABB 후보만: %zu", getCandidates().size() - getExactOverlaps().size() );

        const auto drawStatus = [&]( const overlapTarget& target )
        {
            if( isExactOverlap( target.shape ) )
                ImGui::Text( pointMode ? "%s: 점 포함" : "%s: 실제 겹침", target.name );
            else if( isCandidate( target.shape ) )
                ImGui::Text( "%s: AABB 후보만", target.name );
            else
                ImGui::Text( "%s: 범위 밖", target.name );
        };
        for( const auto& target : getTargets() ) drawStatus( target );
        for( const auto& target : getLineTargets() ) drawStatus( target );

        ImGui::TextWrapped( pointMode ?
            "초록: 점 포함 / 주황: AABB 후보만 / 회색: 후보 아님 / 흰색: 조회 점. 대상의 저장 AABB에는 speculative margin이 포함됩니다." :
            "초록: 실제 겹침 / 주황: AABB 후보만 / 회색: 후보 아님 / 청록: 조회 AABB / 흰색: 조회 도형. 대상의 저장 AABB에는 speculative margin이 포함됩니다." );
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

        const auto drawTarget = [&]( const overlapTarget& target )
        {
            if( !simulation.IsValid( target.shape ) || !simulation.IsValid( target.body ) ) return;

            const auto& geometry = simulation.GetShape( target.shape ).geometry;
            const transform2 pose = simulation.GetBodyTransform( target.body );
            const bool exact = isExactOverlap( target.shape );
            const bool candidate = isCandidate( target.shape );
            const ImU32 color = exact ? exactColor : candidate ? candidateColor : idleColor;

            draw.DrawShape( geometry, pose, color, exact ? IM_COL32( 90, 240, 155, 55 ) : 0 );
            if( areCandidateAABBsVisible() && candidate )
                draw.DrawAABB( simulation.GetShape( target.shape ).aabb, color, 1.5f );

            draw.DrawLabel( pose.position + vec2{ 0.0f, 1.1f }, target.name, color );
        };
        for( const auto& target : getTargets() ) drawTarget( target );
        for( const auto& target : getLineTargets() ) drawTarget( target );

        if( getQueryMode() == overlapQueryMode::point )
        {
            // 점 조회는 넓이가 없는 AABB이므로 도형 윤곽 대신 조회 좌표를 강조함.
            draw.DrawPoint( getQueryTransform().position, queryColor, 7.0f );
            draw.DrawLabel( getQueryTransform().position + vec2{ 0.1f, 0.3f }, "점 조회", queryColor );
        }
        else
        {
            draw.DrawShape( getQueryGeometry(), getQueryTransform(), queryColor, IM_COL32( 240, 245, 250, 25 ) );
            draw.DrawAABB( getQueryAABB(), queryBoundsColor, 2.0f );
            draw.DrawPoint( getQueryTransform().position, queryColor, 4.0f );
        }
    }
};

} // namespace

std::unique_ptr<demo> createOverlapShapeDemoView()
{
    return std::make_unique<overlapShapeDemoView>();
}

} // namespace zonai::sandbox
