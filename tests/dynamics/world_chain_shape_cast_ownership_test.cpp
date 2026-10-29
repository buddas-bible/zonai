#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct chainScene
{
    world simulation{};
    std::array<shapeId, 2> edges{};
    transform2 groundTransform{};

    chainScene( float bend, transform2 groundTransform )
        : groundTransform( groundTransform )
    {
        const bodyId ground = simulation.CreateBody( bodyType::Static, groundTransform );
        const std::array<vec2, 3> vertices{ vec2{ -2.0f, 0.0f },
            vec2{ 0.0f, 0.0f }, vec2{ 2.0f, bend } };
        chainDef definition{};
        definition.points = vertices;
        definition.ghost1 = { -4.0f, 0.0f };
        definition.ghost2 = { 4.0f, 2.0f * bend };
        // CreateChain copies every vertex and ghost into owned segment shapes.
        const chainId chain = simulation.CreateChain( ground, definition );
        check( simulation.GetChainSegmentCount( chain ) == edges.size(),
            "A joined Chain must expose both neighboring edges" );
        check( simulation.GetChainSegments( chain, edges ) == edges.size(),
            "A joined Chain must return stable child handles" );
        check( simulation.GetShapeParentChain( edges[0] ) == chain &&
            simulation.GetShapeParentChain( edges[1] ) == chain,
            "Both test edges must belong to one Chain" );
    }
};

struct collectedHit
{
    shapeId id{};
    vec2 normal{};
    float fraction = 0.0f;
};

void verifyCase( chainScene& scene, float x, int ownedEdge, bool rotational,
    bool offCenter = false )
{
    // Chain의 오른쪽이 앞면. 아래에서 위로 접근하면 두 edge의 endpoint
    // distance circle이 모두 닿을 수 있지만 ghost ownership은 한쪽만 허용해야 함.
    const float radius = 0.32f;
    const vec2 localCenter{ x, -1.0f };
    const vec2 centerOffset = offCenter ? vec2{ 0.10f, 0.0f } : vec2{};
    const rot2 queryRotation = rot2::FromRadians( offCenter ? -0.3f : 0.0f );
    const vec2 startOrigin = TransformPoint( scene.groundTransform,
        localCenter - Rotate( queryRotation, centerOffset ) );
    const vec2 travel = TransformVector( scene.groundTransform, { 0.0f, 1.16f } );

    std::vector<collectedHit> hits{};
    const auto addHit = [&]( shapeId id, vec2, vec2 normal, float fraction )
    {
        check( id == scene.edges[0] || id == scene.edges[1],
            "Chain ShapeCast must return only its own child shape handles" );
        check( std::isfinite( fraction ) && fraction > 0.0f && fraction < 1.0f &&
            IsFinite( normal ), "Chain ShapeCast reported invalid positive contact" );
        hits.push_back( { id, normal, fraction } );
        return 1.0f; // clip을 하지 않아도 이웃 edge를 중복 보고하면 안 됨.
    };

    if( rotational )
    {
        worldShapeSweepInput2 input{};
        input.geometry = circle2{ centerOffset, radius };
        input.transform = { startOrigin, scene.groundTransform.rotation * queryRotation };
        // queryRotation은 ground local 기준. 오프셋 제거에서 ground 회전도 반영.
        if( offCenter )
            input.transform.position = TransformPoint( scene.groundTransform,
                localCenter - Rotate( queryRotation, centerOffset ) );
        input.translation = travel;
        input.rotationRadians = offCenter ? 0.6f : 0.5f;
        scene.simulation.castShapeSweep( input, addHit );
        const auto closest = scene.simulation.castShapeSweepClosest( input );
        if( !closest.hit || ( ownedEdge >= 0 && closest.shape != scene.edges[ownedEdge] ) )
            std::fprintf( stderr, "Rotating owner mismatch: x=%.4f expected=%d actual=%d hit=%d fraction=%.5f offset=%d rotation=(%.3f,%.3f) callbacks=%zu\\n",
                x, ownedEdge >= 0 ? scene.edges[ownedEdge].index1 : 0, closest.shape.index1,
                static_cast<int>( closest.hit ), closest.fraction, offCenter,
                scene.groundTransform.rotation.c, scene.groundTransform.rotation.s, hits.size() );
        check( closest.hit && ( ownedEdge < 0 || closest.shape == scene.edges[ownedEdge] ),
            "Rotating ShapeCastClosest must select the owned chain edge" );
    }
    else
    {
        worldShapeCastInput2 input{};
        input.geometry = circle2{ centerOffset, radius };
        input.transform = { startOrigin, scene.groundTransform.rotation * queryRotation };
        input.translation = travel;
        scene.simulation.castShape( input, addHit );
        const auto closest = scene.simulation.castShapeClosest( input );
        if( !closest.hit || ( ownedEdge >= 0 && closest.shape != scene.edges[ownedEdge] ) )
            std::fprintf( stderr, "Linear owner mismatch: x=%.4f expected=%d actual=%d hit=%d fraction=%.5f offset=%d rotation=(%.3f,%.3f) callbacks=%zu\\n",
                x, ownedEdge >= 0 ? scene.edges[ownedEdge].index1 : 0, closest.shape.index1,
                static_cast<int>( closest.hit ), closest.fraction, offCenter,
                scene.groundTransform.rotation.c, scene.groundTransform.rotation.s, hits.size() );
        check( closest.hit && ( ownedEdge < 0 || closest.shape == scene.edges[ownedEdge] ),
            "Linear ShapeCastClosest must select the owned chain edge" );
    }

    if( hits.size() != 1 )
        std::fprintf( stderr, "Chain seam: bend? x=%.4f rotational=%d offCenter=%d hits=%zu\n",
            x, rotational, offCenter, hits.size() );
    check( hits.size() == 1 && ( ownedEdge < 0 || hits[0].id == scene.edges[ownedEdge] ),
        "Chain seam must report only the Voronoi-owned edge" );
    const vec2 front = TransformVector( scene.groundTransform, { 0.0f, -1.0f } );
    check( Dot( hits[0].normal, front ) > 0.65f,
        "Chain seam must not return a ghost endpoint normal" );
}

void verifyBackFace( chainScene& scene, bool rotational )
{
    const vec2 start = TransformPoint( scene.groundTransform, { 0.15f, 1.0f } );
    const vec2 travel = TransformVector( scene.groundTransform, { 0.0f, -1.2f } );
    int calls = 0;
    const auto countHit = [&]( shapeId, vec2, vec2, float )
    {
        ++calls;
        return 1.0f;
    };
    if( rotational )
    {
        worldShapeSweepInput2 input{};
        input.geometry = circle2{ {}, 0.25f };
        input.transform.position = start;
        input.translation = travel;
        input.rotationRadians = 0.45f;
        scene.simulation.castShapeSweep( input, countHit );
    }
    else
    {
        worldShapeCastInput2 input{};
        input.geometry = circle2{ {}, 0.25f };
        input.transform.position = start;
        input.translation = travel;
        scene.simulation.castShape( input, countHit );
    }
    check( calls == 0, "One-sided Chain must not report back-face approaches" );
}

} // namespace

int main()
{
    // 양쪽 edge의 endpoint circle이 겹치는 seam 부근에서 owner 하나만 보고해야 함.
    for( const float bend : { 0.0f, -0.4f, 0.4f } )
    {
        for( const float angle : { 0.0f, 0.4f } )
        {
            chainScene scene{ bend, { { 1.5f, -0.8f }, rot2::FromRadians( angle ) } };
            for( const bool rotating : { false, true } )
            {
                verifyCase( scene, -0.12f, 0, rotating );
                // 정확한 공유 vertex는 회전된 좌표에서 반올림으로 소유 edge가
                // 뒤바뀔 수 있음. 어느 쪽이든 하나의 handle만 보고해야 함.
                verifyCase( scene, 0.0f, -1, rotating );
                verifyCase( scene, 0.12f, 1, rotating );
                verifyBackFace( scene, rotating );
            }
            if( bend == 0.0f )
                verifyCase( scene, 0.12f, 1, true, true );
        }
    }
    std::printf( "World Chain Circle ShapeCast seam ownership passed\n" );
}
