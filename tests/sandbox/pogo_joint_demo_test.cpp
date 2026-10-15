#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "demo.h"
#include "rigidBodyDemo.h"

using namespace zonai;
using namespace zonai::sandbox;

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

bool near( float a, float b )
{
    return std::abs( a - b ) < 0.0001f;
}

}

void checkUprightSupport()
{
    for( const auto preset : { pogoJointDemoPreset::soft, pogoJointDemoPreset::stiff, pogoJointDemoPreset::compressionOnly, pogoJointDemoPreset::asymmetric } )
        for( const int substeps : { 1, 4, 8 } )
        {
            rigidBodyDemo demo{ demoKind::pogoJointPlayground };
            demo.applyPogoJointPreset( preset );
            const bodyId character = demo.getWorld().getPogoJointData( demo.getPogoJoint() ).bodyB;
            demoInput input{};
            input.spinPressed = true;
            demo.handleInput( input );
            for( int frame = 0; frame < 600; ++frame )
            {
                if( frame == 60 )
                {
                    input = {};
                    input.mousePressed = input.mouseHeld = true;
                    input.mousePosition = TransformPoint( demo.getWorld().GetBodyTransform( character ), { 0.25f, 0.2f } );
                    demo.handleInput( input );
                    check( demo.getWorld().IsValid( demo.getMouseJoint() ), "Pogo rotation regression must exercise dragging" );
                }
                else if( frame > 60 && frame < 120 )
                {
                    input.mousePressed = false;
                    input.mousePosition = { 0.2f, 1.5f };
                    demo.handleInput( input );
                }
                else if( frame == 120 )
                {
                    demo.cancelInput();
                }
                if( frame % 30 == 0 && ( frame < 60 || frame >= 120 ) )
                {
                    demoInput spin{};
                    spin.spinPressed = true;
                    demo.handleInput( spin );
                }
                demo.step( 1.0f / 60.0f, substeps );
                const auto rotation = demo.getWorld().GetBodyTransform( character ).rotation;
                check( near( rotation.c, 1.0f ) && near( rotation.s, 0.0f ) && near( demo.getWorld().GetBodyAngularVelocity( character ), 0.0f ), "Pogo character must stay upright after spin and off-center dragging" );
                const auto data = demo.getWorld().getPogoJointData( demo.getPogoJoint() );
                check( IsFinite( data.anchorB ) && data.anchorB.y < 3.0f, "rotating Pogo must not grow without bound" );
            }
        }
}

int main()
{
    checkUprightSupport();
    // Scene registration부터 preset/runtime response까지 하나의 학습용 Pogo playground 계약으로 확인함.
    check( std::string_view{ getDemoEntry( demoKind::pogoJointPlayground ).category } == "조인트", "Pogo Joint demo stays in joint category" );

    rigidBodyDemo pogo{ demoKind::pogoJointPlayground };
    check( pogo.getShapes().size() >= 5, "Pogo scene demonstrates steps, slopes and a gap" );
    check( pogo.getWorld().IsValid( pogo.getPogoJoint() ), "Pogo scene owns a valid Pogo Joint" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::soft );
    auto data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( data.hertz > 0.0f && data.hertz < 4.0f, "Pogo soft preset uses a low spring frequency" );
    check( data.maxTensionForce > 0.0f && data.maxCompressionForce > data.maxTensionForce, "Pogo soft preset supports both tension and compression" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::stiff );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( data.hertz >= 6.0f && data.maxCompressionForce >= 300.0f, "Pogo stiff preset raises spring response and force budget" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::compressionOnly );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( near( data.maxTensionForce, 0.0f ) && data.maxCompressionForce > 0.0f, "Pogo compression-only preset cannot pull the character down" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::asymmetric );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( data.maxCompressionForce > data.maxTensionForce && data.maxTensionForce > 0.0f, "Pogo asymmetric preset demonstrates separate tension/compression budgets" );

    pogo.setPogoJointSettings( 0.9f, 5.0f, 0.8f, 25.0f, 220.0f );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( near( data.restLength, 0.9f ) && near( data.hertz, 5.0f ) && near( data.dampingRatio, 0.8f ), "Pogo inspector applies spring settings" );
    check( near( data.maxTensionForce, 25.0f ) && near( data.maxCompressionForce, 220.0f ), "Pogo inspector applies asymmetric force limits" );

    const bodyId driven = data.bodyB;
    pogo.applyPogoJointPreset( pogoJointDemoPreset::compressionOnly );
    pogo.step( 1.0f / 60.0f, 1 );
    check( IsFinite( pogo.getWorld().GetBodyLinearVelocity( driven ) ), "Pogo demo produces a finite driven-body response" );
    check( IsFinite( pogo.getWorld().getPogoJointData( pogo.getPogoJoint() ).force ), "Pogo demo exposes a finite reaction force" );

    for( const int substeps : { 1, 4, 8 } )
    {
        rigidBodyDemo walker{ demoKind::pogoJointPlayground };
        const bodyId character = walker.getImpulseBody();
        auto& simulation = walker.getWorld();
        for( const float x : { -2.0f, 2.0f, 4.0f, 6.0f, 9.0f } )
        {
            const float surface = x < 3.0f ? 0.0f : x < 5.0f ? 0.35f : x < 7.0f ? 0.675f : 1.0f;
            simulation.SetBodyTransform( character, { { x, surface + 1.15f }, {} } );
            simulation.SetBodyLinearVelocity( character, {} );
            walker.step( 1.0f / 60.0f, substeps );
            check( simulation.IsValid( walker.getPogoJoint() ), "support must follow terrain beneath the character" );
            const auto support = simulation.getPogoJointData( walker.getPogoJoint() );
            check( std::abs( support.anchorA.x - support.anchorB.x ) < 0.02f, "support must not stretch sideways toward an old ground anchor" );
            if( x == 6.0f ) check( std::abs( support.normal.x ) > 0.1f, "slope support must use the surface normal" );
        }
        simulation.SetBodyTransform( character, { { 7.5f, 2.15f }, {} } );
        walker.step( 1.0f / 60.0f, substeps );
        check( !simulation.IsValid( walker.getPogoJoint() ), "gap must release ground support" );
        walker.applyPogoJointPreset( pogoJointDemoPreset::stiff );
        simulation.SetBodyTransform( character, { { 0.0f, 1.15f }, {} } );
        simulation.SetBodyLinearVelocity( character, {} );
        walker.step( 1.0f / 60.0f, substeps );
        check( simulation.IsValid( walker.getPogoJoint() ) && simulation.getPogoJointData( walker.getPogoJoint() ).hertz == 8.0f, "airborne preset must survive landing" );
        demoInput jump{};
        jump.jumpPressed = true;
        walker.handleInput( jump );
        float peak = 0.0f;
        for( int frame = 0; frame < 180; ++frame )
        {
            walker.step( 1.0f / 60.0f, substeps );
            peak = std::max( peak, simulation.GetBodyTransform( character ).position.y );
        }
        check( peak > 1.8f && simulation.IsValid( walker.getPogoJoint() ), "jump must leave support and land again" );

        rigidBodyDemo course{ demoKind::pogoJointPlayground };
        demoInput move{};
        move.right = true;
        course.handleInput( move );
        bool sawSlope = false;
        bool sawGap = false;
        for( int frame = 0; frame < 180; ++frame )
        {
            course.step( 1.0f / 60.0f, substeps );
            const auto& world = course.getWorld();
            const auto position = world.GetBodyTransform( course.getImpulseBody() ).position;
            if( world.IsValid( course.getPogoJoint() ) )
            {
                const auto support = world.getPogoJointData( course.getPogoJoint() );
                sawSlope |= support.normal.x < -0.1f;
                check( std::abs( support.anchorB.x - support.anchorA.x ) < 0.1f, "walking support must stay beneath the character" );
            }
            else if( position.x > 7.1f && position.x < 7.9f ) sawGap = true;
        }
        check( sawSlope && sawGap, "A/D walking must traverse the step and slope and release support over the gap" );
        course.cancelInput();
        for( int frame = 0; frame < 60; ++frame ) course.step( 1.0f / 60.0f, substeps );
        check( std::abs( course.getWorld().GetBodyLinearVelocity( course.getImpulseBody() ).x ) < 0.1f, "releasing movement must brake horizontal drift" );

        rigidBodyDemo jumper{ demoKind::pogoJointPlayground };
        bool jumped = false;
        bool airborne = false;
        for( int frame = 0; frame < 240; ++frame )
        {
            const float x = jumper.getWorld().GetBodyTransform( jumper.getImpulseBody() ).position.x;
            demoInput input{};
            input.right = x < 9.0f;
            input.jumpPressed = !jumped && x > 6.2f;
            jumped |= input.jumpPressed;
            jumper.handleInput( input );
            jumper.step( 1.0f / 60.0f, substeps );
            airborne |= jumped && !jumper.getWorld().IsValid( jumper.getPogoJoint() );
        }
        check( jumped && airborne && jumper.getWorld().IsValid( jumper.getPogoJoint() ), "walking jump must cross the gap and reconnect on the landing platform" );
        check( jumper.getWorld().getPogoJointData( jumper.getPogoJoint() ).anchorA.y > 0.95f, "jump must land on the far platform" );
    }

    rigidBodyDemo deletedCharacter{ demoKind::pogoJointPlayground };
    deletedCharacter.getWorld().DestroyBody( deletedCharacter.getImpulseBody() );
    deletedCharacter.step( 1.0f / 60.0f, 4 );
    check( !deletedCharacter.getWorld().IsValid( deletedCharacter.getPogoJoint() ), "deleted character must not leave active support" );
    rigidBodyDemo deletedGround{ demoKind::pogoJointPlayground };
    const auto originalSupport = deletedGround.getWorld().getPogoJointData( deletedGround.getPogoJoint() );
    deletedGround.getWorld().DestroyBody( originalSupport.bodyA );
    deletedGround.step( 1.0f / 60.0f, 4 );
    check( IsFinite( deletedGround.getWorld().GetBodyTransform( originalSupport.bodyB ).position ), "deleted ground must not invalidate the remaining simulation" );
    return EXIT_SUCCESS;
}
