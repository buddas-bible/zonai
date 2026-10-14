#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

#include "dynamics/world.h"

using namespace zonai;

namespace
{

void check( bool condition, const char* message )
{
    if( !condition )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

void update( world& simulation )
{
    simulation.UpdateCollisions(
        []( const contactData& )
        {
        } );
}

#pragma region SleepAndWake

void checkConnectedSleep()
{
    world simulation{};
    simulation.SetGravity( {} );
    std::array<bodyId, 3> chain{};
    for( int i = 0; i < 3; ++i )
    {
        chain[i] = simulation.CreateBody( bodyType::Dynamic, { { 2.0f * i, 0.0f }, {} } );
        ( void )simulation.CreateShape( chain[i], circle2{ {}, 1.0f } );
    }
    const bodyId isolated = simulation.CreateBody( bodyType::Dynamic, { { 20.0f, 0.0f }, {} } );
    ( void )simulation.CreateShape( isolated, circle2{ {}, 1.0f } );
    for( int i = 0; i < 40; ++i )
        simulation.Step( 1.0f / 60.0f, 4 );
    for( const bodyId id : chain )
        check( !simulation.IsBodyAwake( id ), "connected island failed to sleep" );
    check( !simulation.IsBodyAwake( isolated ), "isolated body failed to sleep" );
    simulation.ApplyLinearImpulseToCenter( chain[0], { 1.0f, 0.0f } );
    for( const bodyId id : chain )
        check( simulation.IsBodyAwake( id ), "wake did not propagate through island" );
    check( !simulation.IsBodyAwake( isolated ), "wake reached unrelated island" );
    simulation.SetBodyAwake( chain[0], false );
    simulation.SetBodySleepEnabled( chain[2], false );
    for( int i = 0; i < 40; ++i )
        simulation.Step( 1.0f / 60.0f );
    for( const bodyId id : chain )
        check( simulation.IsBodyAwake( id ), "sleep-disabled member allowed island sleep" );
    simulation.SetSleepingEnabled( false );
    check( simulation.IsBodyAwake( isolated ), "world sleep disable did not wake body" );
}

void checkStaticBoundary()
{
    world simulation{};
    simulation.SetGravity( {} );
    const bodyId ground = simulation.CreateBody();
    ( void )simulation.CreateShape( ground, circle2{ {}, 1.0f } );
    const bodyId left = simulation.CreateBody( bodyType::Dynamic, { { -2.0f, 0.0f }, {} } );
    const bodyId right = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, {} } );
    ( void )simulation.CreateShape( left, circle2{ {}, 1.0f } );
    ( void )simulation.CreateShape( right, circle2{ {}, 1.0f } );
    update( simulation );
    simulation.SetBodyAwake( left, false );
    simulation.SetBodyAwake( right, false );
    simulation.SetBodyAwake( left, true );
    check( simulation.IsBodyAwake( left ) && !simulation.IsBodyAwake( right ), "static body bridged wake islands" );
    simulation.SetBodyTransform( ground, {} );
    check( simulation.IsBodyAwake( left ) && simulation.IsBodyAwake( right ), "static mutation did not wake both neighbors" );
}

#pragma endregion SleepAndWake

#pragma region SubstepDeltas

void checkSubsteps()
{
    for( const int substeps : { 1, 4, 8 } )
    {
        world simulation{};
        simulation.SetGravity( {} );
        bodyDef definition{};
        definition.type = bodyType::Dynamic;
        definition.linearVelocity = { 2.0f, 0.0f };
        definition.angularVelocity = 0.5f;
        definition.enableSleep = false;
        const bodyId id = simulation.CreateBody( definition );
        ( void )simulation.CreateShape( id, circle2{ {}, 1.0f } );
        const float mass = simulation.GetBodyMass( id );
        simulation.ApplyForceToCenter( id, { mass, 0.0f } );
        simulation.Step( 0.1f, substeps );
        check( std::fabs( simulation.GetBodyLinearVelocity( id ).x - 2.1f ) < 1e-5f, "substep force was consumed or multiplied incorrectly" );
        const float position = simulation.GetBodyTransform( id ).position.x;
        simulation.Step( 0.1f, substeps );
        check( std::fabs( simulation.GetBodyLinearVelocity( id ).x - 2.1f ) < 1e-5f, "force leaked into next step" );
        check( std::fabs( simulation.GetBodyTransform( id ).position.x - position - 0.21f ) < 1e-5f, "position delta leaked into next step" );
        check( std::fabs( simulation.GetBodyTransform( id ).rotation.s - std::sin( 0.1f ) ) < 1e-5f, "rotation delta leaked into next step" );
    }
}

#pragma endregion SubstepDeltas

#pragma region ContinuousCollision

void checkStaticCcd()
{
    for( const int substeps : { 1, 4 } )
    {
        world simulation{};
        simulation.SetGravity( {} );
        const bodyId wall = simulation.CreateBody();
        ( void )simulation.CreateShape( wall, MakeBox( { 0.1f, 4.0f } ) );
        bodyDef definition{};
        definition.type = bodyType::Dynamic;
        definition.transform.position = { -5.0f, 0.0f };
        definition.linearVelocity = { 100.0f, 0.0f };
        definition.enableSleep = false;
        const bodyId fast = simulation.CreateBody( definition );
        ( void )simulation.CreateShape( fast, circle2{ {}, 0.25f } );
        simulation.Step( 0.1f, substeps );
        check( simulation.IsBodyFast( fast ) && simulation.HadBodyTimeOfImpact( fast ), "static CCD missed fast body" );
        check( simulation.GetBodyTransform( fast ).position.x < -0.3f, "fast body crossed static wall" );
        simulation.Step( 0.0f );
        check( !simulation.IsBodyFast( fast ) && !simulation.HadBodyTimeOfImpact( fast ), "transient CCD flags survived zero step" );
        simulation.SetContinuousEnabled( false );
        simulation.SetBodyTransform( fast, { { -5.0f, 0.0f }, {} } );
        simulation.SetBodyLinearVelocity( fast, { 100.0f, 0.0f } );
        simulation.Step( 0.1f, substeps );
        check( !simulation.HadBodyTimeOfImpact( fast ) && simulation.GetBodyTransform( fast ).position.x > 4.0f, "disabled CCD still clipped motion" );
    }
}

void checkInitialToiFallback()
{
    for( const int substeps : { 1, 4, 8 } )
        for( const vec2 offset : { vec2{}, vec2{ 2.0f, 1.0f } } )
        {
            world simulation{};
            simulation.SetGravity( {} );
            const bodyId wall = simulation.CreateBody();
            const shapeId wallShape = simulation.CreateShape( wall, MakeBox( { 0.05f, 20.0f } ) );
            simulation.SetShapeFriction( wallShape, 0.0f );

            bodyDef definition{};
            definition.type = bodyType::Dynamic;
            definition.transform.rotation = rot2::FromRadians( 0.56f );
            definition.transform.position = vec2{ -0.75f, 0.0f } - Rotate( definition.transform.rotation, offset );
            definition.linearVelocity = { 80.0f, 1.5f };
            definition.angularVelocity = 1.25f;
            definition.enableSleep = false;
            const bodyId fast = simulation.CreateBody( definition );
            polygon2 box = MakeBox( { 0.8f, 0.04f } );
            for( int i = 0; i < box.vertexCount; ++i ) box.vertices[i] += offset;
            box.centroid += offset;
            const shapeId boxShape = simulation.CreateShape( fast, box );
            simulation.SetShapeFriction( boxShape, 0.0f );

            simulation.Step( 0.1f, substeps );
            check( simulation.IsBodyFast( fast ) && simulation.HadBodyTimeOfImpact( fast ), "initial TOI fallback missed thin rotating box" );
            const vec2 center = TransformPoint( simulation.GetBodyTransform( fast ), box.centroid );
            check( center.x < -0.05f, "initial TOI fallback let box center cross wall" );
        }
}

void checkChainCcdSidedness()
{
    // point1 -> point2의 오른쪽(아래쪽)만 충돌하는 chain.
    const chainSegment2 chain{ { -4.0f, 0.0f }, { { -2.0f, 0.0f }, { 2.0f, 0.0f } }, { 4.0f, 0.0f } };

    {
        // 뒷면(위쪽)에서 아래로 통과하는 fast circle은 one-sided chain 때문에 잘리면 안 됨.
        world simulation{};
        simulation.SetGravity( {} );
        const bodyId ground = simulation.CreateBody();
        ( void )simulation.CreateShape( ground, chain );

        bodyDef definition{};
        definition.type = bodyType::Dynamic;
        definition.transform.position = { 0.0f, 5.0f };
        definition.linearVelocity = { 0.0f, -100.0f };
        definition.enableSleep = false;
        const bodyId fast = simulation.CreateBody( definition );
        ( void )simulation.CreateShape( fast, circle2{ {}, 0.25f } );

        simulation.Step( 0.1f, 1 );

        check( simulation.IsBodyFast( fast ), "chain CCD back-face fixture was not fast" );
        check( !simulation.HadBodyTimeOfImpact( fast ), "chain CCD clipped a back-face crossing" );
        check( simulation.GetBodyTransform( fast ).position.y < -4.0f, "fast body did not pass through chain back face" );
    }

    {
        // 앞면(아래쪽)에서 위로 들어오면 같은 chain이 CCD로 motion을 잘라야 함.
        world simulation{};
        simulation.SetGravity( {} );
        const bodyId ground = simulation.CreateBody();
        ( void )simulation.CreateShape( ground, chain );

        bodyDef definition{};
        definition.type = bodyType::Dynamic;
        definition.transform.position = { 0.0f, -5.0f };
        definition.linearVelocity = { 0.0f, 100.0f };
        definition.enableSleep = false;
        const bodyId fast = simulation.CreateBody( definition );
        ( void )simulation.CreateShape( fast, circle2{ {}, 0.25f } );

        simulation.Step( 0.1f, 1 );

        check( simulation.HadBodyTimeOfImpact( fast ), "chain CCD missed a front-face crossing" );
        check( simulation.GetBodyTransform( fast ).position.y < -0.2f, "fast body crossed chain front face" );
    }
}

void checkMovingTarget()
{
    // 생성 순서가 달라도 non-bullet 최종 bounds가 bullet query 전에 반영되어야 함.
    for( const bodyType targetType : { bodyType::Dynamic, bodyType::Kinematic } )
        for( const bool bulletFirst : { false, true } )
        {
            world simulation{};
            simulation.SetGravity( {} );
            bodyDef target{};
            target.type = targetType;
            target.transform.position = { 8.0f, 0.0f };
            target.linearVelocity = { -80.0f, 0.0f };
            target.enableSleep = false;
            bodyDef bullet = target;
            bullet.type = bodyType::Dynamic;
            bullet.transform.position = { -5.0f, 0.0f };
            bullet.linearVelocity = { 100.0f, 0.0f };
            bullet.isBullet = true;
            const bodyId first = simulation.CreateBody( bulletFirst ? bullet : target );
            const bodyId second = simulation.CreateBody( bulletFirst ? target : bullet );
            ( void )simulation.CreateShape( first, circle2{ {}, 0.25f } );
            ( void )simulation.CreateShape( second, circle2{ {}, 0.25f } );
            const bodyId bulletId = bulletFirst ? first : second;
            simulation.Step( 0.1f, 4 );
            check( simulation.HadBodyTimeOfImpact( bulletId ), "bullet missed moving target after bounds finalize" );
            check( simulation.GetBodyTransform( bulletId ).position.x < 4.0f, "bullet motion was not clipped" );
        }
}

void checkSensorToiBoundary()
{
    // 후보 순서와 무관하게 solid TOI 전의 sensor crossing만 남아야 함.
    for( const bool beforeWall : { false, true } )
        for( const bool sensorFirst : { false, true } )
        {
            world simulation{};
            simulation.SetGravity( {} );
            const bodyId staticBody = simulation.CreateBody();
            shapeId sensor{};
            if( sensorFirst ) sensor = simulation.CreateSensorShape( staticBody, circle2{ { beforeWall ? -2.0f : 2.0f, 0.0f }, 0.1f } );
            ( void )simulation.CreateShape( staticBody, MakeBox( { 0.1f, 4.0f } ) );
            if( !sensorFirst ) sensor = simulation.CreateSensorShape( staticBody, circle2{ { beforeWall ? -2.0f : 2.0f, 0.0f }, 0.1f } );
            bodyDef definition{};
            definition.type = bodyType::Dynamic;
            definition.transform.position = { -5.0f, 0.0f };
            definition.linearVelocity = { 100.0f, 0.0f };
            definition.enableSleep = false;
            const bodyId fast = simulation.CreateBody( definition );
            const shapeId visitor = simulation.CreateShape( fast, circle2{ {}, 0.25f } );
            simulation.SetShapeSensorEventsEnabled( sensor, true );
            simulation.SetShapeSensorEventsEnabled( visitor, true );
            simulation.Step( 0.1f, 4 );
            check( simulation.HadBodyTimeOfImpact( fast ), "sensor boundary solid TOI fixture" );
            check( simulation.GetSensorBeginEvents().size() == ( beforeWall ? 1u : 0u ), "sensor crossing escaped solid TOI boundary" );
            if( beforeWall ) check( simulation.GetSensorBeginEvents()[0].visitorShapeId == visitor, "continuous sensor visitor identity" );
            simulation.Step( 0.0f );
            check( simulation.GetSensorEndEvents().size() == ( beforeWall ? 1u : 0u ), "transient sensor crossing survived next update" );
        }
}

#pragma endregion ContinuousCollision

} // namespace

int main()
{
    checkConnectedSleep();
    checkStaticBoundary();
    checkSubsteps();
    checkStaticCcd();
    checkInitialToiFallback();
    checkChainCcdSidedness();
    checkMovingTarget();
    checkSensorToiBoundary();
}
