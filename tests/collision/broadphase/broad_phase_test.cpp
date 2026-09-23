#include <algorithm>
#include <array>
#include <cassert>
#include <utility>
#include <vector>

#include "collision/broadphase/broadPhase.h"
#include "dynamics/bodyType.h"

using namespace zonai;

int main()
{
    static_assert( MakeProxyKey( 0, BodyType::Static ) == 0 );
    static_assert( MakeProxyKey( 0, BodyType::Kinematic ) == 1 );
    static_assert( MakeProxyKey( 0, BodyType::Dynamic ) == 2 );
    static_assert( MakeProxyKey( 7, BodyType::Dynamic ) == 30 );

    constexpr ProxyKey key = MakeProxyKey( 17, BodyType::Kinematic );

    static_assert( GetProxyId( key ) == 17 );
    static_assert( GetProxyType( key ) == BodyType::Kinematic );

    BroadPhase broadPhase{};

    DynamicTree& staticTree = broadPhase.GetTree( BodyType::Static );
    DynamicTree& kinematicTree = broadPhase.GetTree( BodyType::Kinematic );
    DynamicTree& dynamicTree = broadPhase.GetTree( BodyType::Dynamic );

    assert( &staticTree != &kinematicTree );
    assert( &staticTree != &dynamicTree );
    assert( &kinematicTree != &dynamicTree );

    const aabb2 box{
        { 0.0f, 0.0f },
        { 1.0f, 1.0f }
    };

    staticTree.CreateProxy( box, 1 );

    assert( staticTree.GetProxyCount() == 1 );
    assert( kinematicTree.GetProxyCount() == 0 );
    assert( dynamicTree.GetProxyCount() == 0 );

    kinematicTree.CreateProxy( box, 2 );

    assert( staticTree.GetProxyCount() == 1 );
    assert( kinematicTree.GetProxyCount() == 1 );
    assert( dynamicTree.GetProxyCount() == 0 );

    dynamicTree.CreateProxy( box, 3 );

    assert( staticTree.GetProxyCount() == 1 );
    assert( kinematicTree.GetProxyCount() == 1 );
    assert( dynamicTree.GetProxyCount() == 1 );

    BroadPhase proxyBroadPhase{};

    const ProxyKey staticKey = proxyBroadPhase.CreateProxy( BodyType::Static, box, 11 );
    const ProxyKey kinematicKey = proxyBroadPhase.CreateProxy( BodyType::Kinematic, box, 12 );
    const ProxyKey dynamicKey = proxyBroadPhase.CreateProxy( BodyType::Dynamic, box, 13 );

    assert( GetProxyType( staticKey ) == BodyType::Static );
    assert( GetProxyType( kinematicKey ) == BodyType::Kinematic );
    assert( GetProxyType( dynamicKey ) == BodyType::Dynamic );

    assert( GetProxyId( staticKey ) == 0 );
    assert( GetProxyId( kinematicKey ) == 0 );
    assert( GetProxyId( dynamicKey ) == 0 );

    assert( proxyBroadPhase.GetTree( BodyType::Static ).GetProxyCount() == 1 );
    assert( proxyBroadPhase.GetTree( BodyType::Kinematic ).GetProxyCount() == 1 );
    assert( proxyBroadPhase.GetTree( BodyType::Dynamic ).GetProxyCount() == 1 );

    // Box2D처럼 static은 기본적으로 moved 처리하지 않고 kinematic / dynamic은 moved 처리함.
    assert( proxyBroadPhase.GetTree( BodyType::Static ).HasMoved() == false );
    assert( proxyBroadPhase.GetTree( BodyType::Kinematic ).HasMoved() );
    assert( proxyBroadPhase.GetTree( BodyType::Dynamic ).HasMoved() );

    proxyBroadPhase.CreateProxy( BodyType::Static, box, 14, true );

    assert( proxyBroadPhase.GetTree( BodyType::Static ).HasMoved() );

    proxyBroadPhase.DestroyProxy( kinematicKey );

    assert( proxyBroadPhase.GetTree( BodyType::Static ).GetProxyCount() == 2 );
    assert( proxyBroadPhase.GetTree( BodyType::Kinematic ).GetProxyCount() == 0 );
    assert( proxyBroadPhase.GetTree( BodyType::Dynamic ).GetProxyCount() == 1 );

    assert( proxyBroadPhase.GetTree( BodyType::Static ).Validate() );
    assert( proxyBroadPhase.GetTree( BodyType::Kinematic ).Validate() );
    assert( proxyBroadPhase.GetTree( BodyType::Dynamic ).Validate() );

    BroadPhase moveBroadPhase{};

    const ProxyKey movedStaticKey = moveBroadPhase.CreateProxy( BodyType::Static, box, 21 );

    assert( moveBroadPhase.GetTree( BodyType::Static ).HasMoved() == false );

    const aabb2 movedBox{
        { 2.0f, 0.0f },
        { 3.0f, 1.0f }
    };

    moveBroadPhase.MoveProxy( movedStaticKey, movedBox );

    assert( moveBroadPhase.GetTree( BodyType::Static ).HasMoved() );

    const aabb2& proxyAABB = moveBroadPhase.GetTree( BodyType::Static ).GetProxyAABB( GetProxyId( movedStaticKey ) );

    assert( proxyAABB.min.x == movedBox.min.x );
    assert( proxyAABB.min.y == movedBox.min.y );
    assert( proxyAABB.max.x == movedBox.max.x );
    assert( proxyAABB.max.y == movedBox.max.y );
    assert( moveBroadPhase.GetTree( BodyType::Static ).Validate() );

    BroadPhase pairBroadPhase{};

    const aabb2 pairBoxA{
        { 0.0f, 0.0f },
        { 2.0f, 2.0f }
    };

    const aabb2 pairBoxB{
        { 1.0f, 0.0f },
        { 3.0f, 2.0f }
    };

    const aabb2 pairBoxC{
        { 10.0f, 0.0f },
        { 11.0f, 1.0f }
    };

    pairBroadPhase.CreateProxy( BodyType::Dynamic, pairBoxA, 31 );
    pairBroadPhase.CreateProxy( BodyType::Dynamic, pairBoxB, 32 );
    const ProxyKey pairKeyC = pairBroadPhase.CreateProxy( BodyType::Dynamic, pairBoxC, 33 );

    std::array<std::int32_t, 16> movedSiblings{};
    std::vector<std::pair<std::int32_t, std::int32_t>> pairs;

    pairBroadPhase.FindDynamicSelfPairs(
        movedSiblings,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            pairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    std::sort( pairs.begin(), pairs.end() );

    const std::pair<std::int32_t, std::int32_t> initialPair{ 31, 32 };

    assert( pairs.size() == 1 );
    assert( pairs[0] == initialPair );

    pairBroadPhase.GetTree( BodyType::Dynamic ).ClearMoved();
    pairs.clear();

    pairBroadPhase.FindDynamicSelfPairs(
        movedSiblings,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            pairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    assert( pairs.empty() );

    const aabb2 movedPairBoxC{
        { 1.5f, 0.0f },
        { 2.5f, 2.0f }
    };

    pairBroadPhase.MoveProxy( pairKeyC, movedPairBoxC );
    pairs.clear();

    pairBroadPhase.FindDynamicSelfPairs(
        movedSiblings,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            pairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    std::sort( pairs.begin(), pairs.end() );

    const std::vector<std::pair<std::int32_t, std::int32_t>> expectedPairs{
        { 31, 33 },
        { 32, 33 }
    };

    assert( pairs == expectedPairs );
    assert( pairBroadPhase.GetTree( BodyType::Dynamic ).Validate() );

    BroadPhase crossBroadPhase{};

    const aabb2 staticBoxA{
        { 0.0f, 0.0f },
        { 2.0f, 2.0f }
    };

    const aabb2 staticBoxB{
        { 10.0f, 0.0f },
        { 12.0f, 2.0f }
    };

    const aabb2 dynamicBoxA{
        { 1.0f, 0.0f },
        { 3.0f, 2.0f }
    };

    const aabb2 dynamicBoxB{
        { 20.0f, 0.0f },
        { 21.0f, 1.0f }
    };

    crossBroadPhase.CreateProxy( BodyType::Static, staticBoxA, 41 );
    const ProxyKey staticKeyB = crossBroadPhase.CreateProxy( BodyType::Static, staticBoxB, 42 );
    crossBroadPhase.CreateProxy( BodyType::Dynamic, dynamicBoxA, 51 );
    crossBroadPhase.CreateProxy( BodyType::Dynamic, dynamicBoxB, 52 );

    std::vector<std::pair<std::int32_t, std::int32_t>> crossPairs;

    crossBroadPhase.FindDynamicStaticPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            crossPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    const std::pair<std::int32_t, std::int32_t> initialCrossPair{ 41, 51 };

    assert( crossPairs.size() == 1 );
    assert( crossPairs[0] == initialCrossPair );

    crossBroadPhase.GetTree( BodyType::Static ).ClearMoved();
    crossBroadPhase.GetTree( BodyType::Dynamic ).ClearMoved();
    crossPairs.clear();

    crossBroadPhase.FindDynamicStaticPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            crossPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    assert( crossPairs.empty() );

    const aabb2 movedStaticBoxB{
        { 20.0f, 0.0f },
        { 21.0f, 1.0f }
    };

    crossBroadPhase.MoveProxy( staticKeyB, movedStaticBoxB );
    crossPairs.clear();

    crossBroadPhase.FindDynamicStaticPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            crossPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    const std::pair<std::int32_t, std::int32_t> movedStaticPair{ 42, 52 };

    assert( crossPairs.size() == 1 );
    assert( crossPairs[0] == movedStaticPair );
    assert( crossBroadPhase.GetTree( BodyType::Static ).Validate() );
    assert( crossBroadPhase.GetTree( BodyType::Dynamic ).Validate() );

    return 0;
}
