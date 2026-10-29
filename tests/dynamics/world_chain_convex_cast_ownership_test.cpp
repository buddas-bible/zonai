#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "collision/distance2.h"
#include "collision/narrowphase/collide.h"
#include "dynamics/world.h"
#include "geometry/polygon2.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

enum class probeKind { capsule, box, triangle };

struct scene
{
    world simulation{};
    std::array<shapeId, 2> handles{};
    std::array<chainSegment2, 2> segments{};
    transform2 ground{};

    scene( float bend, float groundAngle )
    {
        ground = { { 1.3f, -0.7f }, rot2::FromRadians( groundAngle ) };
        const bodyId body = simulation.CreateBody( bodyType::Static, ground );
        const std::array<vec2, 3> points{ vec2{ -2.0f, 0.0f },
            vec2{ 0.0f, 0.0f }, vec2{ 2.0f, bend } };
        chainDef definition{};
        definition.points = points;
        definition.ghost1 = { -4.0f, 0.0f };
        definition.ghost2 = { 4.0f, 2.0f * bend };
        const chainId parent = simulation.CreateChain( body, definition );
        check( simulation.GetChainSegments( parent, handles ) == handles.size(),
            "Convex seam test needs two adjacent chain children" );
        segments = {
            chainSegment2{ definition.ghost1, { points[0], points[1] }, points[2] },
            chainSegment2{ points[0], { points[1], points[2] }, definition.ghost2 }
        };
    }
};

shapeGeometry makeProbe( probeKind kind, vec2 offset )
{
    if( kind == probeKind::capsule )
        return capsule2{ offset + vec2{ -0.28f, 0.0f },
            offset + vec2{ 0.28f, 0.0f }, 0.17f };

    if( kind == probeKind::box )
    {
        polygon2 box = MakeBox( { 0.24f, 0.19f } );
        for( int i = 0; i < box.vertexCount; ++i )
            box.vertices[i] += offset;
        box.centroid += offset;
        return box;
    }

    const std::array<vec2, 3> vertices{
        offset + vec2{ -0.30f, -0.17f },
        offset + vec2{ 0.28f, -0.18f },
        offset + vec2{ 0.03f, 0.26f }
    };
    return MakePolygon( vertices );
}

bool narrowphaseOwns( const chainSegment2& chain, const shapeGeometry& geometry,
    transform2 movingInChain )
{
    simplexCache2 cache{};
    if( const capsule2* capsule = std::get_if<capsule2>( &geometry ) )
        return CollideChainSegmentCapsule( chain, *capsule, movingInChain, cache ).pointCount > 0;
    const polygon2& polygon = std::get<polygon2>( geometry );
    return CollideChainSegmentPolygon( chain, polygon, movingInChain, cache ).pointCount > 0;
}

struct result
{
    shapeId shape{};
    vec2 normal{};
    float fraction = 0.0f;
};

struct counts
{
    int positive[3][2]{};
    int backFace = 0;
    int visited = 0;
};

void checkCase( scene& target, probeKind kind, float x, float angle,
    bool rotating, bool backFace, bool offset, counts& totals )
{
    const vec2 localOffset = offset ? vec2{ 0.18f, -0.08f } : vec2{};
    const shapeGeometry geometry = makeProbe( kind, localOffset );
    const rot2 localRotation = rot2::FromRadians( angle );
    const vec2 center = { x, backFace ? 1.06f : -1.12f };
    const vec2 origin = TransformPoint( target.ground, center - Rotate( localRotation, localOffset ) );
    const transform2 start{ origin, target.ground.rotation * localRotation };
    const vec2 movement = TransformVector( target.ground,
        backFace ? vec2{ 0.0f, -1.5f } : vec2{ 0.0f, 1.55f } );
    const float relativeRotation = rotating ? ( offset ? -0.5f : 0.45f ) : 0.0f;

    std::vector<result> hits{};
    const auto callback = [&]( shapeId id, vec2 point, vec2 normal, float fraction )
    {
        check( IsFinite( point ) && IsFinite( normal ) && std::isfinite( fraction ) &&
            fraction >= 0.0f && fraction < 1.0f, "Convex Chain query returned invalid hit data" );
        check( id == target.handles[0] || id == target.handles[1],
            "Convex Chain query returned an unrelated shape handle" );

        transform2 pose = start;
        if( rotating )
        {
            const rot2 endRotation = start.rotation * rot2::FromRadians( relativeRotation );
            const sweep2 sweep{ {}, start.position, start.position + movement, start.rotation, endRotation };
            pose = GetSweepTransform( sweep, fraction );
        }
        else pose.position += fraction * movement;

        const int edge = id == target.handles[0] ? 0 : 1;
        const transform2 localPose = InverseMul( target.ground, pose );
        if( !narrowphaseOwns( target.segments[edge], geometry, localPose ) )
        {
            std::fprintf( stderr,
                "Chain invalid convex owner: kind=%d x=%.4f angle=%.4f rotation=%d "
                "offset=%d back=%d edge=%d fraction=%.6f\n",
                static_cast<int>( kind ), x, angle, rotating, offset, backFace, edge, fraction );
            check( false, "World Chain convex hit must be owned by its narrowphase manifold" );
        }

        const vec2 front = TransformVector( target.ground, { 0.0f, -1.0f } );
        check( fraction == 0.0f || Dot( normal, front ) > 0.10f,
            "World Chain convex query returned a backward-facing contact normal" );
        hits.push_back( { id, normal, fraction } );
        return 1.0f;
    };

    worldShapeCastResult2 closest{};
    if( rotating )
    {
        worldShapeSweepInput2 input{};
        input.geometry = geometry;
        input.transform = start;
        input.translation = movement;
        input.rotationRadians = relativeRotation;
        target.simulation.castShapeSweep( input, callback );
        closest = target.simulation.castShapeSweepClosest( input );
    }
    else
    {
        worldShapeCastInput2 input{};
        input.geometry = geometry;
        input.transform = start;
        input.translation = movement;
        target.simulation.castShape( input, callback );
        closest = target.simulation.castShapeClosest( input );
    }

    if( backFace )
    {
        check( hits.empty() && !closest.hit, "Chain convex cast must reject back-face approach" );
        ++totals.backFace;
        return;
    }

    for( std::size_t i = 0; i < hits.size(); ++i )
    {
        for( std::size_t j = i + 1; j < hits.size(); ++j )
            check( hits[i].shape != hits[j].shape, "Convex Chain edge returned multiple callbacks" );
    }

    if( !hits.empty() )
    {
        const auto earliest = std::min_element( hits.begin(), hits.end(),
            []( const result& a, const result& b ) { return a.fraction < b.fraction; } );
        if( !closest.hit || closest.shape != earliest->shape ||
            std::fabs( closest.fraction - earliest->fraction ) >= 0.0005f )
        {
            std::fprintf( stderr,
                "Closest mismatch: kind=%d x=%.4f angle=%.4f rotation=%d offset=%d "
                "hit=%d shape=%d fraction=%.7f earliest_shape=%d earliest=%.7f callbacks=%zu\\n",
                static_cast<int>( kind ), x, angle, rotating, offset,
                static_cast<int>( closest.hit ), closest.shape.index1, closest.fraction,
                earliest->shape.index1, earliest->fraction, hits.size() );
            for( const result& h : hits )
                std::fprintf( stderr, "  callback shape=%d fraction=%.7f\\n",
                    h.shape.index1, h.fraction );
        }
        check( closest.hit && closest.shape == earliest->shape &&
            std::fabs( closest.fraction - earliest->fraction ) < 0.0005f,
            "Closest convex Chain cast must return the earliest non-overlap callback" );
        ++totals.positive[static_cast<int>( kind )][static_cast<int>( rotating )];
    }
    else check( !closest.hit, "Convex Chain closest reported a missing callback hit" );
    ++totals.visited;
}

void verifyDegenerateCapsule( scene& target )
{
    // World ShapeCast는 퇴화 Capsule proxy를 허용하므로, ghost 검사의
    // MakeCapsule(2-vertex polygon) assert 없이 Circle 대체 규칙을 사용해야 함.
    worldShapeCastInput2 input{};
    input.geometry = capsule2{ {}, {}, 0.25f };
    input.transform.position = TransformPoint( target.ground, { -0.8f, -1.0f } );
    input.translation = TransformVector( target.ground, { 0.0f, 1.4f } );
    check( target.simulation.castShapeClosest( input ).hit,
        "Degenerate Capsule should remain a valid World cast geometry" );
    worldShapeSweepInput2 sweep{};
    sweep.geometry = input.geometry;
    sweep.transform = input.transform;
    sweep.translation = input.translation;
    sweep.rotationRadians = 0.3f;
    check( target.simulation.castShapeSweepClosest( sweep ).hit,
        "Rotating degenerate Capsule should not assert in seam ownership" );
}

} // namespace

int main()
{
    counts totals{};
    for( const float bend : { -0.5f, 0.0f, 0.5f } )
    {
        for( const float terrainAngle : { 0.0f, 0.38f } )
        {
            scene target{ bend, terrainAngle };
            for( const probeKind kind : { probeKind::capsule, probeKind::box, probeKind::triangle } )
            {
                for( const bool rotating : { false, true } )
                {
                    for( const float x : { -0.5f, -0.15f, 0.0f, 0.15f, 0.5f } )
                        checkCase( target, kind, x, 0.12f, rotating, false, false, totals );
                    checkCase( target, kind, -0.25f, -0.32f, rotating, false, true, totals );
                    checkCase( target, kind, 0.25f, 0.28f, rotating, false, true, totals );
                    checkCase( target, kind, 0.1f, 0.0f, rotating, true, false, totals );
                }
            }
            verifyDegenerateCapsule( target );
        }
    }

    for( int kind = 0; kind < 3; ++kind )
        for( int rotating = 0; rotating < 2; ++rotating )
            check( totals.positive[kind][rotating] >= 12,
                "Convex Chain ownership suite lacks positive contacts for a geometry/mode" );
    check( totals.backFace == 36 && totals.visited >= 250,
        "Convex Chain ownership suite lacks back-face and seam coverage" );
    std::printf( "Chain convex World cast ownership: positive Capsule=%d/%d Box=%d/%d "
        "Triangle=%d/%d back=%d total=%d\n",
        totals.positive[0][0], totals.positive[0][1],
        totals.positive[1][0], totals.positive[1][1],
        totals.positive[2][0], totals.positive[2][1],
        totals.backFace, totals.visited );
}
