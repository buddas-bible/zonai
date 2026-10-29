#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "collision/rotatingSweepBounds2.h"
#include "collision/shapeGeometry.h"
#include "collision/sweep2.h"
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

struct randomSequence
{
    std::uint32_t state = 0xC172B89Du;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    float between( float low, float high )
    {
        return low + ( high - low ) * static_cast<float>( next() >> 8 ) * ( 1.0f / 16777216.0f );
    }
};

shapeGeometry makeGeometry( randomSequence& random, int index )
{
    const vec2 offset{ random.between( -3.0f, 3.0f ), random.between( -3.0f, 3.0f ) };
    const float radius = random.between( 0.12f, 0.6f );
    if( index % 3 == 0 ) return circle2{ offset, radius };
    if( index % 3 == 1 )
        return capsule2{ offset + vec2{ -0.45f, 0.0f }, offset + vec2{ 0.45f, 0.0f }, radius };

    polygon2 box = MakeBox( { random.between( 0.15f, 0.75f ), random.between( 0.15f, 0.75f ) } );
    for( int i = 0; i < box.vertexCount; ++i ) box.vertices[i] += offset;
    box.centroid += offset;
    return box;
}

void verifyRandomSweeps()
{
    randomSequence random{};
    constexpr int CASE_COUNT = 420;
    constexpr int SAMPLES = 256;

    for( int n = 0; n < CASE_COUNT; ++n )
    {
        const shapeGeometry geometry = makeGeometry( random, n );
        const shapeProxy2 proxy = MakeShapeProxy( geometry );
        const transform2 start{
            { random.between( -12.0f, 12.0f ), random.between( -12.0f, 12.0f ) },
            rot2::FromRadians( random.between( -3.1f, 3.1f ) )
        };
        const vec2 translation{ random.between( -5.0f, 5.0f ), random.between( -5.0f, 5.0f ) };
        const float angle = random.between( -1.5707963267948966f, 1.5707963267948966f );
        const vec2 endPosition = start.position + translation;
        const rot2 endRotation = start.rotation * rot2::FromRadians( angle );

        aabb2 bound{};
        check( TryComputeRotatingSweepBounds( proxy, start, endPosition, endRotation, angle, bound ),
            "Valid rotation sweep must produce finite bounds" );

        const sweep2 sweep{ {}, start.position, endPosition, start.rotation, endRotation };

        for( int sample = 0; sample <= SAMPLES; ++sample )
        {
            const float fraction = static_cast<float>( sample ) / static_cast<float>( SAMPLES );
            const transform2 pose = GetSweepTransform( sweep, fraction );
            const aabb2 exact = ComputeShapeAABB( geometry, pose );

            if( !ContainsAABB( bound, exact ) )
            {
                std::fprintf( stderr, "Sweep bound omitted sample: case=%d fraction=%.6f\n", n, fraction );
                check( false, "Rotating sweep bounds must enclose geometry over nlerp" );
            }
        }
    }

    std::printf( "Rotating arc bounds verified: %d cases x %d time samples\n", CASE_COUNT, SAMPLES + 1 );
}

void verifyPivotReductionAndWorldHits()
{
    const shapeGeometry probe = circle2{ { 3.0f, 0.0f }, 0.2f };
    const transform2 start{};
    const float quarterTurn = 1.5707963267948966f;
    const rot2 endRotation = rot2::FromRadians( quarterTurn );
    aabb2 arc{};
    check( TryComputeRotatingSweepBounds( MakeShapeProxy( probe ), start, {},
        endRotation, quarterTurn, arc ), "Quarter-turn arc bounds should be valid" );

    check( arc.min.x > -0.1f && arc.min.y > -0.1f &&
        arc.max.x > 3.1f && arc.max.y > 3.1f,
        "Offset-circle first-quadrant arc should exclude opposite quadrants" );

    world simulation{};
    const bodyId cornerBody = simulation.CreateBody( bodyType::Static, { { -2.0f, -2.0f }, {} } );
    ( void )simulation.CreateShape( cornerBody, circle2{ {}, 0.2f } );

    worldShapeSweepInput2 input{};
    input.geometry = probe;
    input.rotationRadians = quarterTurn;

    worldShapeSweepStats2 stats{};
    const auto miss = simulation.castShapeSweepClosest( input, {}, &stats );
    check( !miss.hit && stats.treeCandidates == 0,
        "Offset-arc broadphase must reject opposite-quadrant candidate" );

    const bodyId targetBody = simulation.CreateBody( bodyType::Static, { { 2.12f, 2.12f }, {} } );
    const shapeId target = simulation.CreateShape( targetBody, circle2{ {}, 0.3f } );
    const auto hit = simulation.castShapeSweepClosest( input );
    check( hit.hit && hit.shape == target && hit.fraction > 0.0f && hit.fraction < 1.0f,
        "Tight arc bounds must preserve an actual mid-rotation hit" );

    input.rotationRadians = -quarterTurn;
    const bodyId downBody = simulation.CreateBody( bodyType::Static, { { 2.12f, -2.12f }, {} } );
    const shapeId downTarget = simulation.CreateShape( downBody, circle2{ {}, 0.3f } );
    check( simulation.castShapeSweepClosest( input ).shape == downTarget,
        "Negative arc bounds must preserve lower-quadrant hit" );
}

}

int main()
{
    verifyRandomSweeps();
    verifyPivotReductionAndWorldHits();
}
