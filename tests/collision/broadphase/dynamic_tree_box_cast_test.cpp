#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "collision/broadphase/dynamicTree.h"

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
    std::uint32_t state = 0x75BE1234u;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    float between( float low, float high )
    {
        return low + ( high - low ) * static_cast<float>( next() & 0xFFFFFFu ) / 16777216.0f;
    }
};

struct target
{
    aabb2 bounds{};
    std::int32_t proxyId = -1;
    bool live = true;
};

// 이동 AABB와 고정 AABB가 [0, limit] 중 어느 순간에라도 겹치는지
// 양 축의 독립적인 허용 시간 구간을 구해 확인함.
bool overlapsDuringSweep( aabb2 moving, aabb2 fixed, vec2 translation, float limit )
{
    float lowerTime = 0.0f;
    float upperTime = limit;

    for( int axis = 0; axis < 2; ++axis )
    {
        const float minA = axis == 0 ? moving.min.x : moving.min.y;
        const float maxA = axis == 0 ? moving.max.x : moving.max.y;
        const float minB = axis == 0 ? fixed.min.x : fixed.min.y;
        const float maxB = axis == 0 ? fixed.max.x : fixed.max.y;
        const float velocity = axis == 0 ? translation.x : translation.y;

        if( velocity == 0.0f )
        {
            if( minA > maxB || maxA < minB ) return false;
            continue;
        }

        const float from = ( minB - maxA ) / velocity;
        const float to = ( maxB - minA ) / velocity;
        lowerTime = std::max( lowerTime, std::min( from, to ) );
        upperTime = std::min( upperTime, std::max( from, to ) );
        if( lowerTime > upperTime ) return false;
    }

    return lowerTime <= upperTime;
}

aabb2 randomBox( randomSequence& random )
{
    const vec2 center{ random.between( -18.0f, 18.0f ), random.between( -18.0f, 18.0f ) };
    const vec2 radius{ random.between( 0.1f, 1.25f ), random.between( 0.1f, 1.25f ) };
    return { center - radius, center + radius };
}

void verifyAgainstOracle( const dynamicTree& tree, const std::vector<target>& targets, randomSequence& random )
{
    constexpr std::array fractions{ 0.18f, 0.55f, 1.0f };

    for( int sample = 0; sample < 125; ++sample )
    {
        const aabb2 moving = randomBox( random );
        vec2 translation{ random.between( -25.0f, 25.0f ), random.between( -25.0f, 25.0f ) };

        // 정지 축과 시작 겹침도 포함. BoxCast는 translation이 0인 입력도 처리함.
        if( sample % 9 == 0 ) translation.y = 0.0f;
        if( sample % 13 == 0 ) translation.x = 0.0f;

        for( float limit : fractions )
        {
            float actualLimit = limit;
            std::vector<std::int32_t> found;
            tree.BoxCast( moving, translation, actualLimit,
                [&]( std::int32_t id, float callbackLimit )
                {
                    check( callbackLimit == limit, "An ignored hit must not change the clip fraction" );
                    found.push_back( id );
                    return -1.0f;
                } );

            std::vector<std::int32_t> expected;
            for( const target& object : targets )
            {
                if( object.live && overlapsDuringSweep( moving, object.bounds, translation, limit ) )
                    expected.push_back( object.proxyId );
            }

            std::sort( found.begin(), found.end() );
            std::sort( expected.begin(), expected.end() );
            check( found == expected, "BoxCast candidates differ from swept-AABB oracle" );
            check( actualLimit == limit, "No clipping must preserve maxFraction" );
        }
    }
}

void verifyClipping()
{
    dynamicTree tree{};
    ( void )tree.CreateProxy( { { 1.0f, -0.5f }, { 2.0f, 0.5f } }, 0 );
    ( void )tree.CreateProxy( { { 5.0f, -0.5f }, { 6.0f, 0.5f } }, 1 );

    const aabb2 start{ { -3.0f, -0.25f }, { -2.5f, 0.25f } };
    const vec2 travel{ 12.0f, 0.0f };

    float fraction = 1.0f;
    int count = 0;
    tree.BoxCast( start, travel, fraction, [&]( std::int32_t, float )
    {
        ++count;
        return 0.25f;
    } );
    check( count >= 1 && std::fabs( fraction - 0.25f ) < 1.0e-6f, "BoxCast must retain clipped fraction" );

    fraction = 1.0f;
    count = 0;
    tree.BoxCast( start, travel, fraction, [&]( std::int32_t, float )
    {
        ++count;
        return 0.0f;
    } );
    check( count == 1 && fraction == 0.0f, "BoxCast must stop on zero fraction" );
}

void verifyDiagonalPruning()
{
    dynamicTree tree{};

    for( int x = -4; x < 4; ++x )
    {
        for( int y = -4; y < 4; ++y )
        {
            const vec2 center{ static_cast<float>( x * 2 ), static_cast<float>( y * 2 ) };
            ( void )tree.CreateProxy( { center - vec2{ 0.3f, 0.3f }, center + vec2{ 0.3f, 0.3f } }, x * 8 + y + 36 );
        }
    }

    const aabb2 moving{ { -9.0f, -9.0f }, { -8.8f, -8.8f } };
    const vec2 translation{ 19.0f, 19.0f };
    const aabb2 end{ moving.min + translation, moving.max + translation };
    int broadCandidates = 0;
    tree.Query( Union( moving, end ), [&]( std::int32_t )
    {
        ++broadCandidates;
        return true;
    } );

    float limit = 1.0f;
    int boxCandidates = 0;
    tree.BoxCast( moving, translation, limit, [&]( std::int32_t, float )
    {
        ++boxCandidates;
        return 1.0f;
    } );

    check( broadCandidates >= 60 && boxCandidates > 0 && boxCandidates < 25,
        "Diagonal BoxCast should prune candidates outside the actual swept corridor" );
}

}

int main()
{
    verifyClipping();
    verifyDiagonalPruning();

    dynamicTree emptyTree{};
    const aabb2 box{ { -1.0f, -1.0f }, { 1.0f, 1.0f } };
    float emptyLimit = 1.0f;
    emptyTree.BoxCast( box, { 2.0f, 0.0f }, emptyLimit, [&]( std::int32_t, float )
    {
        check( false, "Empty tree must not invoke BoxCast callback" );
        return -1.0f;
    } );

    dynamicTree tree{};
    randomSequence random{};
    std::vector<target> targets;

    for( int i = 0; i < 84; ++i )
    {
        const aabb2 bounds = randomBox( random );
        targets.push_back( { bounds, tree.CreateProxy( bounds, i ), true } );
    }

    check( tree.Validate(), "Initial DynamicTree must be valid" );
    verifyAgainstOracle( tree, targets, random );

    for( std::size_t i = 0; i < targets.size(); ++i )
    {
        if( i % 5 == 0 )
        {
            targets[i].bounds = randomBox( random );
            tree.MoveProxy( targets[i].proxyId, targets[i].bounds );
        }
        else if( i % 7 == 0 )
        {
            tree.DestroyProxy( targets[i].proxyId );
            targets[i].live = false;
        }
    }

    check( tree.Validate(), "DynamicTree must remain valid after updates" );
    verifyAgainstOracle( tree, targets, random );

    ( void )tree.Rebuild( true );
    check( tree.Validate(), "Rebuilt DynamicTree must be valid" );
    verifyAgainstOracle( tree, targets, random );
}
