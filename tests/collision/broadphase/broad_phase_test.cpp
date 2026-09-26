#include <algorithm>
#include <array>
#include <cassert>
#include <utility>
#include <vector>

#include "collision/broadphase/broadPhase.h"
#include "collision/shape.h"
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

    constexpr ShapePairKey shapePairKey = MakeShapePairKey( 5, 10 );

    static_assert( shapePairKey == ( std::uint64_t{ 5 } << 32 | std::uint64_t{ 10 } ) );
    static_assert( MakeShapePairKey( 10, 5 ) == shapePairKey );
    static_assert( MakeShapePairKey( 7, 7 ) == ( std::uint64_t{ 7 } << 32 | std::uint64_t{ 7 } ) );

    BroadPhase pairSetBroadPhase{};
    const ShapePairKey contactPairKey = MakeShapePairKey( 21, 34 );

    assert( pairSetBroadPhase.HasPair( contactPairKey ) == false );
    assert( pairSetBroadPhase.AddPair( contactPairKey ) == false );
    assert( pairSetBroadPhase.HasPair( contactPairKey ) );
    assert( pairSetBroadPhase.AddPair( contactPairKey ) );
    assert( pairSetBroadPhase.RemovePair( contactPairKey ) );
    assert( pairSetBroadPhase.HasPair( contactPairKey ) == false );
    assert( pairSetBroadPhase.RemovePair( contactPairKey ) == false );

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

    BroadPhase kinematicCrossBroadPhase{};

    const aabb2 kinematicBoxA{
        { 0.0f, 0.0f },
        { 2.0f, 2.0f }
    };

    const aabb2 kinematicBoxB{
        { 10.0f, 0.0f },
        { 12.0f, 2.0f }
    };

    const aabb2 kinematicDynamicBoxA{
        { 1.0f, 0.0f },
        { 3.0f, 2.0f }
    };

    const aabb2 kinematicDynamicBoxB{
        { 20.0f, 0.0f },
        { 21.0f, 1.0f }
    };

    kinematicCrossBroadPhase.CreateProxy( BodyType::Kinematic, kinematicBoxA, 61 );
    const ProxyKey kinematicKeyB = kinematicCrossBroadPhase.CreateProxy( BodyType::Kinematic, kinematicBoxB, 62 );
    kinematicCrossBroadPhase.CreateProxy( BodyType::Dynamic, kinematicDynamicBoxA, 71 );
    kinematicCrossBroadPhase.CreateProxy( BodyType::Dynamic, kinematicDynamicBoxB, 72 );

    std::vector<std::pair<std::int32_t, std::int32_t>> kinematicPairs;

    kinematicCrossBroadPhase.FindDynamicKinematicPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            kinematicPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    const std::pair<std::int32_t, std::int32_t> initialKinematicPair{ 61, 71 };

    assert( kinematicPairs.size() == 1 );
    assert( kinematicPairs[0] == initialKinematicPair );

    kinematicCrossBroadPhase.GetTree( BodyType::Kinematic ).ClearMoved();
    kinematicCrossBroadPhase.GetTree( BodyType::Dynamic ).ClearMoved();
    kinematicPairs.clear();

    kinematicCrossBroadPhase.FindDynamicKinematicPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            kinematicPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    assert( kinematicPairs.empty() );

    const aabb2 movedKinematicBoxB{
        { 20.0f, 0.0f },
        { 21.0f, 1.0f }
    };

    kinematicCrossBroadPhase.MoveProxy( kinematicKeyB, movedKinematicBoxB );
    kinematicPairs.clear();

    kinematicCrossBroadPhase.FindDynamicKinematicPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            kinematicPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    const std::pair<std::int32_t, std::int32_t> movedKinematicPair{ 62, 72 };

    assert( kinematicPairs.size() == 1 );
    assert( kinematicPairs[0] == movedKinematicPair );
    assert( kinematicCrossBroadPhase.GetTree( BodyType::Kinematic ).Validate() );
    assert( kinematicCrossBroadPhase.GetTree( BodyType::Dynamic ).Validate() );

    BroadPhase candidateBroadPhase{};

    const aabb2 candidateBox{
        { 0.0f, 0.0f },
        { 2.0f, 2.0f }
    };

    constexpr std::int32_t candidateDynamicShape = 200;
    constexpr std::int32_t candidateStaticBase = 300;
    constexpr std::int32_t candidateStaticCount = 40;

    candidateBroadPhase.CreateProxy( BodyType::Dynamic, candidateBox, candidateDynamicShape );

    for( std::int32_t i = 0; i < candidateStaticCount; ++i )
    {
        candidateBroadPhase.CreateProxy( BodyType::Static, candidateBox, candidateStaticBase + i );
    }

    std::vector<std::pair<std::int32_t, std::int32_t>> candidatePairs;

    candidateBroadPhase.FindDynamicStaticPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            candidatePairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    assert( candidatePairs.size() == candidateStaticCount );

    constexpr std::int32_t existingStaticShape = candidateStaticBase + 15;
    const ShapePairKey existingPairKey = MakeShapePairKey( candidateDynamicShape, existingStaticShape );

    assert( candidateBroadPhase.AddPair( existingPairKey ) == false );

    candidatePairs.clear();

    candidateBroadPhase.FindDynamicStaticPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            candidatePairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    assert( candidatePairs.size() == candidateStaticCount - 1 );
    assert(
        std::find(
            candidatePairs.begin(),
            candidatePairs.end(),
            std::pair<std::int32_t, std::int32_t>{ candidateDynamicShape, existingStaticShape }
        ) == candidatePairs.end()
    );

    assert( candidateBroadPhase.RemovePair( existingPairKey ) );

    candidatePairs.clear();

    candidateBroadPhase.FindDynamicStaticPairs(
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            candidatePairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    assert( candidatePairs.size() == candidateStaticCount );

    BroadPhase combinedBroadPhase{};

    const aabb2 selfBoxA{
        { 0.0f, 0.0f },
        { 1.0f, 1.0f }
    };

    const aabb2 selfBoxB{
        { 0.5f, 0.0f },
        { 1.5f, 1.0f }
    };

    const aabb2 staticDynamicBox{
        { 10.0f, 0.0f },
        { 11.0f, 1.0f }
    };

    const aabb2 staticCrossBox{
        { 10.5f, 0.0f },
        { 11.5f, 1.0f }
    };

    const aabb2 kinematicDynamicBox{
        { 20.0f, 0.0f },
        { 21.0f, 1.0f }
    };

    const aabb2 kinematicCrossBox{
        { 20.5f, 0.0f },
        { 21.5f, 1.0f }
    };

    combinedBroadPhase.CreateProxy( BodyType::Dynamic, selfBoxA, 501 );
    combinedBroadPhase.CreateProxy( BodyType::Dynamic, selfBoxB, 502 );
    combinedBroadPhase.CreateProxy( BodyType::Dynamic, staticDynamicBox, 503 );
    combinedBroadPhase.CreateProxy( BodyType::Dynamic, kinematicDynamicBox, 504 );
    combinedBroadPhase.CreateProxy( BodyType::Static, staticCrossBox, 601 );
    combinedBroadPhase.CreateProxy( BodyType::Kinematic, kinematicCrossBox, 701 );

    std::array<std::int32_t, 16> combinedMovedSiblings{};
    std::vector<std::pair<std::int32_t, std::int32_t>> combinedPairs;

    combinedBroadPhase.FindPairs(
        combinedMovedSiblings,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            combinedPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    std::sort( combinedPairs.begin(), combinedPairs.end() );

    const std::vector<std::pair<std::int32_t, std::int32_t>> expectedCombinedPairs{
        { 501, 502 },
        { 503, 601 },
        { 504, 701 }
    };

    assert( combinedPairs == expectedCombinedPairs );

    // pair 탐색과 moved 소비를 분리해 이후 Rebuild 단계에서 Box2D와 같은 lifecycle을 연결함.
    assert( combinedBroadPhase.GetTree( BodyType::Dynamic ).HasMoved() );
    assert( combinedBroadPhase.GetTree( BodyType::Kinematic ).HasMoved() );

    BroadPhase updateBroadPhase{};

    const aabb2 updateDynamicA{
        { 0.0f, 0.0f },
        { 1.0f, 1.0f }
    };

    const aabb2 updateDynamicB{
        { 0.5f, 0.0f },
        { 1.5f, 1.0f }
    };

    const aabb2 updateDynamicStatic{
        { 10.0f, 0.0f },
        { 11.0f, 1.0f }
    };

    const aabb2 updateStatic{
        { 10.5f, 0.0f },
        { 11.5f, 1.0f }
    };

    const aabb2 updateDynamicKinematic{
        { 20.0f, 0.0f },
        { 21.0f, 1.0f }
    };

    const aabb2 updateKinematic{
        { 20.5f, 0.0f },
        { 21.5f, 1.0f }
    };

    updateBroadPhase.CreateProxy( BodyType::Dynamic, updateDynamicA, 801 );
    updateBroadPhase.CreateProxy( BodyType::Dynamic, updateDynamicB, 802 );
    updateBroadPhase.CreateProxy( BodyType::Dynamic, updateDynamicStatic, 803 );

    // static moved lifecycle도 검증하기 위해 pair 생성을 강제로 요청함.
    updateBroadPhase.CreateProxy( BodyType::Static, updateStatic, 901, true );

    updateBroadPhase.CreateProxy( BodyType::Dynamic, updateDynamicKinematic, 804 );
    updateBroadPhase.CreateProxy( BodyType::Kinematic, updateKinematic, 1001 );

    assert( updateBroadPhase.GetTree( BodyType::Static ).HasMoved() );
    assert( updateBroadPhase.GetTree( BodyType::Dynamic ).NeedsRebuild() );
    assert( updateBroadPhase.GetTree( BodyType::Kinematic ).NeedsRebuild() );

    std::array<std::int32_t, 16> updateMovedSiblings{};
    std::vector<std::pair<std::int32_t, std::int32_t>> updatePairs;

    updateBroadPhase.UpdatePairs(
        updateMovedSiblings,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            updatePairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    std::sort( updatePairs.begin(), updatePairs.end() );

    const std::vector<std::pair<std::int32_t, std::int32_t>> expectedUpdatePairs{
        { 801, 802 },
        { 803, 901 },
        { 804, 1001 }
    };

    assert( updatePairs == expectedUpdatePairs );

    // 한 update가 끝나면 static moved는 clear되고 dynamic / kinematic stale tree는 rebuild됨.
    assert( updateBroadPhase.GetTree( BodyType::Static ).HasMoved() == false );
    assert( updateBroadPhase.GetTree( BodyType::Dynamic ).HasMoved() == false );
    assert( updateBroadPhase.GetTree( BodyType::Kinematic ).HasMoved() == false );
    assert( updateBroadPhase.GetTree( BodyType::Dynamic ).NeedsRebuild() == false );
    assert( updateBroadPhase.GetTree( BodyType::Kinematic ).NeedsRebuild() == false );

    assert( updateBroadPhase.GetTree( BodyType::Static ).Validate() );
    assert( updateBroadPhase.GetTree( BodyType::Dynamic ).Validate() );
    assert( updateBroadPhase.GetTree( BodyType::Kinematic ).Validate() );

    updatePairs.clear();

    // 아무 tree도 dirty하지 않으면 다음 update는 바로 끝나서 후보를 다시 만들지 않음.
    updateBroadPhase.UpdatePairs(
        updateMovedSiblings,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            updatePairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    assert( updatePairs.empty() );

    // 반대쪽 tree가 비어 있으면 empty root를 leaf처럼 취급해
    // shape 0과 가짜 (0, 0) pair를 만들면 안 됨.
    {
        BroadPhase emptyCrossBroadPhase{};

        const aabb2 originSpanningBox{
            { -1.0f, -1.0f },
            {  1.0f,  1.0f }
        };

        emptyCrossBroadPhase.CreateProxy(
            BodyType::Dynamic,
            originSpanningBox,
            0
        );

        std::vector<std::pair<std::int32_t, std::int32_t>> emptyCrossPairs;

        emptyCrossBroadPhase.FindDynamicStaticPairs(
            [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
            {
                emptyCrossPairs.emplace_back( shapeIndexA, shapeIndexB );
            }
        );

        emptyCrossBroadPhase.FindDynamicKinematicPairs(
            [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
            {
                emptyCrossPairs.emplace_back( shapeIndexA, shapeIndexB );
            }
        );

        assert( emptyCrossPairs.empty() );
    }

    BroadPhase sameBodyBroadPhase{};

    const aabb2 sameBodyBoxA{
        { 0.0f, 0.0f },
        { 2.0f, 2.0f }
    };

    const aabb2 sameBodyBoxB{
        { 1.0f, 0.0f },
        { 3.0f, 2.0f }
    };

    sameBodyBroadPhase.CreateProxy( BodyType::Dynamic, sameBodyBoxA, 0 );
    sameBodyBroadPhase.CreateProxy( BodyType::Dynamic, sameBodyBoxB, 1 );

    std::array<Shape, 2> sameBodyShapes{};
    sameBodyShapes[0].bodyId = 10;
    sameBodyShapes[1].bodyId = 10;

    std::array<std::int32_t, 4> sameBodyMovedSiblings{};
    std::vector<std::pair<std::int32_t, std::int32_t>> sameBodyPairs;

    sameBodyBroadPhase.FindDynamicSelfPairs(
        sameBodyMovedSiblings,
        sameBodyShapes,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            sameBodyPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    // 같은 Body에 속한 두 Shape는 BroadPhase overlap이 있어도 Contact 후보가 되지 않음.
    assert( sameBodyPairs.empty() );

    sameBodyShapes[1].bodyId = 11;

    sameBodyBroadPhase.FindDynamicSelfPairs(
        sameBodyMovedSiblings,
        sameBodyShapes,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            sameBodyPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    const std::pair<std::int32_t, std::int32_t> differentBodyPair{ 0, 1 };

    assert( sameBodyPairs.size() == 1 );
    assert( sameBodyPairs[0] == differentBodyPair );

    BroadPhase sensorBroadPhase{};

    const aabb2 sensorBoxA{
        { 0.0f, 0.0f },
        { 2.0f, 2.0f }
    };

    const aabb2 sensorBoxB{
        { 1.0f, 0.0f },
        { 3.0f, 2.0f }
    };

    sensorBroadPhase.CreateProxy( BodyType::Dynamic, sensorBoxA, 0 );
    sensorBroadPhase.CreateProxy( BodyType::Dynamic, sensorBoxB, 1 );

    std::array<Shape, 2> sensorShapes{};
    sensorShapes[0].bodyId = 20;
    sensorShapes[1].bodyId = 21;
    sensorShapes[0].sensorIndex = 0;

    std::array<std::int32_t, 4> sensorMovedSiblings{};
    std::vector<std::pair<std::int32_t, std::int32_t>> sensorPairs;

    sensorBroadPhase.FindDynamicSelfPairs(
        sensorMovedSiblings,
        sensorShapes,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            sensorPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    // Sensor overlap은 일반 Contact 생성 경로에서 제외됨.
    assert( sensorPairs.empty() );

    sensorShapes[0].sensorIndex = Shape::NULL_INDEX;

    sensorBroadPhase.FindDynamicSelfPairs(
        sensorMovedSiblings,
        sensorShapes,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            sensorPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    const std::pair<std::int32_t, std::int32_t> nonSensorPair{ 0, 1 };

    assert( sensorPairs.size() == 1 );
    assert( sensorPairs[0] == nonSensorPair );

    BroadPhase filteredUpdateBroadPhase{};

    const aabb2 filterSelfBoxA{
        { 0.0f, 0.0f },
        { 2.0f, 2.0f }
    };

    const aabb2 filterSelfBoxB{
        { 1.0f, 0.0f },
        { 3.0f, 2.0f }
    };

    const aabb2 filterDynamicStaticBox{
        { 10.0f, 0.0f },
        { 12.0f, 2.0f }
    };

    const aabb2 filterStaticBox{
        { 11.0f, 0.0f },
        { 13.0f, 2.0f }
    };

    const aabb2 filterDynamicKinematicBox{
        { 20.0f, 0.0f },
        { 22.0f, 2.0f }
    };

    const aabb2 filterKinematicBox{
        { 21.0f, 0.0f },
        { 23.0f, 2.0f }
    };

    filteredUpdateBroadPhase.CreateProxy( BodyType::Dynamic, filterSelfBoxA, 0 );
    filteredUpdateBroadPhase.CreateProxy( BodyType::Dynamic, filterSelfBoxB, 1 );
    filteredUpdateBroadPhase.CreateProxy( BodyType::Dynamic, filterDynamicStaticBox, 2 );
    filteredUpdateBroadPhase.CreateProxy( BodyType::Static, filterStaticBox, 3 );
    filteredUpdateBroadPhase.CreateProxy( BodyType::Dynamic, filterDynamicKinematicBox, 4 );
    filteredUpdateBroadPhase.CreateProxy( BodyType::Kinematic, filterKinematicBox, 5 );

    std::array<Shape, 6> filteredShapes{};

    for( std::int32_t i = 0; i < static_cast<std::int32_t>( filteredShapes.size() ); ++i )
    {
        filteredShapes[i].bodyId = i;
    }

    // dynamic self pair는 서로의 category를 허용하지 않아 제거됨.
    filteredShapes[0].filter.categoryBits = 1ull << 0;
    filteredShapes[0].filter.maskBits = 1ull << 0;
    filteredShapes[1].filter.categoryBits = 1ull << 1;
    filteredShapes[1].filter.maskBits = 1ull << 1;

    // dynamic / static pair도 서로의 category를 허용하지 않아 제거됨.
    filteredShapes[2].filter.categoryBits = 1ull << 2;
    filteredShapes[2].filter.maskBits = 1ull << 2;
    filteredShapes[3].filter.categoryBits = 1ull << 3;
    filteredShapes[3].filter.maskBits = 1ull << 3;

    // dynamic / kinematic pair는 서로의 category를 허용하므로 후보로 남음.
    filteredShapes[4].filter.categoryBits = 1ull << 4;
    filteredShapes[4].filter.maskBits = 1ull << 5;
    filteredShapes[5].filter.categoryBits = 1ull << 5;
    filteredShapes[5].filter.maskBits = 1ull << 4;

    std::array<std::int32_t, 16> filteredMovedSiblings{};
    std::vector<std::pair<std::int32_t, std::int32_t>> filteredPairs;

    filteredUpdateBroadPhase.UpdatePairs(
        filteredMovedSiblings,
        filteredShapes,
        [&]( std::int32_t shapeIndexA, std::int32_t shapeIndexB )
        {
            filteredPairs.emplace_back( shapeIndexA, shapeIndexB );
        }
    );

    const std::pair<std::int32_t, std::int32_t> allowedFilteredPair{ 4, 5 };

    assert( filteredPairs.size() == 1 );
    assert( filteredPairs[0] == allowedFilteredPair );

    // Shape-aware UpdatePairs도 기존 update lifecycle과 동일하게 moved / stale 상태를 소비함.
    assert( filteredUpdateBroadPhase.GetTree( BodyType::Static ).HasMoved() == false );
    assert( filteredUpdateBroadPhase.GetTree( BodyType::Dynamic ).NeedsRebuild() == false );
    assert( filteredUpdateBroadPhase.GetTree( BodyType::Kinematic ).NeedsRebuild() == false );

    return 0;
}
