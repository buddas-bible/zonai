#include <array>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <variant>
#include "dynamics/world.h"

using namespace zonai;

namespace
{
void check( bool ok, const char* message )
{
    if( !ok )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}
}

int main()
{
    world simulation{};
    const bodyId ground = simulation.CreateBody();
    const vec2 original[] = { { 2, 0 }, { 0, 0 }, { -2, 0 } };
    chainDef definition{};
    definition.points = original;
    definition.ghost1 = { 4, 0 };
    definition.ghost2 = { -4, 0 };
    definition.friction = 0.3f;
    const chainId chain = simulation.CreateChain( ground, definition );
    std::array<shapeId, 2> children{};
    check( simulation.GetChainSegments( chain, children ) == children.size(), "batch fixture exposes both children" );

    const vec2 shifted[] = { { 0, 0 }, { -2, 0 }, { -4, 0 } };
    check( simulation.SetChainPoints( chain, shifted, { 2, 0 }, { -6, 0 } ), "whole-chain shift succeeds without a degenerate intermediate edge" );
    std::array<shapeId, 2> after{};
    check( simulation.GetChainSegments( chain, after ) == after.size(), "batch update preserves segment count" );
    check( after == children, "batch geometry update preserves child handles" );
    const auto& first = std::get<chainSegment2>( simulation.GetShape( children[0] ).geometry );
    const auto& last = std::get<chainSegment2>( simulation.GetShape( children[1] ).geometry );
    check( first.segment.a.x == 0 && first.segment.b.x == -2 && first.ghost2.x == -4 && last.ghost1.x == 0 && last.ghost2.x == -6, "batch update connects all edges and ghosts" );
    check( simulation.GetShapeFriction( children[0] ) == 0.3f, "batch update preserves material" );

    const vec2 collapsed[] = { { 0, 1 }, { -2, 1 }, { -2, 1 } };
    check( !simulation.SetChainPoints( chain, collapsed, { 2, 1 }, { -6, 1 } ), "invalid final edge rejects the entire batch" );
    check( first.segment.a.y == 0 && last.segment.b.y == 0, "rejected batch leaves earlier edges unchanged" );
    check( !simulation.SetChainPoints( chain, std::span{ shifted, 1 }, { 2, 0 }, { -6, 0 } ), "batch update rejects fewer than two points" );
    check( !simulation.SetChainPoints( {}, shifted, { 2, 0 }, { -6, 0 } ), "invalid chain handle is rejected" );
    check( !simulation.SetChainPoints( chain, shifted, { std::numeric_limits<float>::infinity(), 0 }, { -6, 0 } ), "nonfinite ghost is rejected" );
    const vec2 overflowing[] = { { 1.0e30f, 0 }, { 0, 0 }, { -2, 0 } };
    check( !simulation.SetChainPoints( chain, overflowing, { 2, 0 }, { -6, 0 } ), "finite coordinates with overflowing edge length are rejected" );

    const bodyId visitor = simulation.CreateBody( bodyType::Dynamic, { { -1, 0.5f }, {} } );
    (void)simulation.CreateShape( visitor, circle2{ {}, 0.5f } );
    simulation.Step( 0.001f );
    std::array<contactData, 4> contacts{};
    check( simulation.GetBodyContactData( visitor, contacts ) > 0, "batch contact fixture" );
    const contactId oldContact = contacts[0].id;
    simulation.SetBodyAwake( visitor, false );
    check( simulation.SetChainPoints( chain, shifted, { 2, 0 }, { -6, 0 } ), "unchanged batch succeeds" );
    check( simulation.IsValid( oldContact ) && !simulation.IsBodyAwake( visitor ), "unchanged batch preserves contacts and sleep" );
    const vec2 lowered[] = { { 0, -2 }, { -2, -2 }, { -4, -2 } };
    check( simulation.SetChainPoints( chain, lowered, { 2, -2 }, { -6, -2 } ), "lowered batch succeeds" );
    check( !simulation.IsValid( oldContact ) && simulation.IsBodyAwake( visitor ), "changed batch invalidates old contact and wakes visitor" );
    check( simulation.GetShapeAABB( children[0] ).max.y < -1.9f, "batch update refreshes collision bounds" );
    simulation.SetBodyTransform( visitor, { { -1, -1.5f }, {} } );
    simulation.Step( 0.001f );
    check( simulation.GetBodyContactData( visitor, contacts ) > 0, "new geometry creates contacts through refreshed broad phase" );
    definition.isLoop = true;
    const chainId loop = simulation.CreateChain( ground, definition );
    const vec2 openReplacement[] = { { 2, 0 }, { 0, 0 }, { -2, 0 }, { -4, 0 } };
    check( !simulation.SetChainPoints( loop, openReplacement, { 4, 0 }, { -6, 0 } ), "open batch update rejects closed chains even when the point count matches" );
    simulation.DestroyChain( chain );
    check( !simulation.SetChainPoints( chain, shifted, { 2, 0 }, { -6, 0 } ), "stale chain handle is rejected" );

    world resizing{};
    const bodyId owner = resizing.CreateBody();
    definition.isLoop = false;
    const chainId resized = resizing.CreateChain( owner, definition );
    std::array<shapeId, 2> prefix{};
    check( resizing.GetChainSegments( resized, prefix ) == 2, "resize fixture" );
    resizing.SetShapeFriction( prefix[1], 0.7f );
    resizing.SetShapeRestitution( prefix[1], 0.2f );
    const vec2 grown[] = { { 2, 0 }, { 1, 0 }, { 0, 0 }, { -1, 0 }, { -2, 0 } };
    check( resizing.SetChainPoints( resized, grown, { 3, 0 }, { -3, 0 } ), "chain batch grows to match adaptive samples" );
    std::array<shapeId, 4> expanded{};
    check( resizing.GetChainSegments( resized, expanded ) == 4 && expanded[0] == prefix[0] && expanded[1] == prefix[1], "growth preserves parent and existing child slots" );
    check( resizing.GetShapeFriction( expanded[3] ) == 0.7f && resizing.GetShapeRestitution( expanded[3] ) == 0.2f, "added children inherit the adjacent old tail material" );
    const bodyId resting = resizing.CreateBody( bodyType::Dynamic, { { -1.5f, 0.5f }, {} } );
    (void)resizing.CreateShape( resting, circle2{ {}, 0.5f } );
    resizing.Step( 0.001f );
    check( resizing.GetBodyContactData( resting, contacts ) > 0, "retired child contact fixture" );
    const contactId retiredContact = contacts[0].id;
    resizing.SetBodyAwake( resting, false );
    const vec2 shrunk[] = { { 2, 0 }, { -2, 0 } };
    check( resizing.SetChainPoints( resized, shrunk, { 4, 0 }, { -4, 0 } ), "chain batch shrinks to match adaptive samples" );
    check( resizing.IsValid( resized ) && resizing.IsValid( prefix[0] ) && !resizing.IsValid( expanded[1] ) && !resizing.IsValid( expanded[3] ), "shrink keeps the parent and invalidates retired children" );
    check( !resizing.IsValid( retiredContact ) && resizing.IsBodyAwake( resting ), "retired children invalidate contacts and wake visitors" );
    check( resizing.GetChainSegmentCount( resized ) == 1 && resizing.GetShapeCount() == 2, "shrink removes retired shapes and retains the visitor" );
    const vec2 invalidGrowth[] = { { 2, 1 }, { 1, 1 }, { 0, 1 }, { 0, 1 } };
    check( !resizing.SetChainPoints( resized, invalidGrowth, { 3, 1 }, { -1, 1 } ) && resizing.GetChainSegmentCount( resized ) == 1, "invalid growth does not allocate or partially update children" );
    check( resizing.SetChainPoints( resized, grown, { 3, 0 }, { -3, 0 } ) && !resizing.IsValid( expanded[3] ), "regrowth does not revive stale child handles" );
    return 0;
}
