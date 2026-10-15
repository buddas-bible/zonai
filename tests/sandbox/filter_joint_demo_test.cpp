#include <array>
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

}

int main()
{
    // Filter playground는 Joint 생성/파괴 자체로 collision 상태를 전환하는 학습 장면임.
    check( std::string_view{ getDemoEntry( demoKind::filterJointPlayground ).category } == "조인트", "Filter Joint demo stays in joint category" );

    rigidBodyDemo filter{ demoKind::filterJointPlayground };
    check( filter.getWorld().GetBodyCount() == 3 && filter.getShapes().size() == 3, "Filter scene contains a filtered pair and a third collision body" );
    check( filter.getWorld().getJointCount() == 1 && filter.getWorld().IsValid( filter.getFilterJoint() ), "Filter scene starts with an enabled Filter Joint" );
    check( !filter.getWorld().getFilterJointData( filter.getFilterJoint() ).collideConnected, "Filter demo uses collision-blocking semantics" );

    filter.step( 1.0f / 60.0f, 1 );
    check( filter.getWorld().GetContactCount() == 0, "enabled Filter prevents the overlapping demo bodies from contacting" );

    // Off/On은 setting mutation이 아니라 같은 두 Body를 잇는 Joint의 destroy/create lifecycle을 직접 보여줌.
    filter.setFilterJointEnabled( false );
    check( filter.getWorld().getJointCount() == 0 && !filter.getWorld().IsValid( filter.getFilterJoint() ), "Filter Off destroys the solver-less Joint" );
    filter.step( 1.0f / 60.0f, 1 );
    check( filter.getWorld().GetContactCount() == 1, "Filter Off restores collision between the same bodies" );

    filter.setFilterJointEnabled( true );
    check( filter.getWorld().IsValid( filter.getFilterJoint() ) && filter.getWorld().GetContactCount() == 0, "Filter On recreates the Joint and removes the existing Contact immediately" );

    for( const int substeps : { 1, 4, 8 } )
        for( const bool enabled : { false, true } )
        {
            rigidBodyDemo demo{ demoKind::filterJointPlayground };
            auto& simulation = demo.getWorld();
            demo.setFilterJointEnabled( enabled );
            demo.testFilterCollision();
            const auto shapes = demo.getShapes();
            bool hitA = false, hitB = false;
            bool movedA = false, movedB = false;
            for( int frame = 0; frame < 120; ++frame )
            {
                demo.step( 1.0f / 60.0f, substeps );
                std::array<contactData, 3> contacts{};
                for( const auto& visual : shapes )
                {
                    const auto count = simulation.GetBodyContactData( visual.bodyHandle, contacts );
                    for( std::size_t index = 0; index < count; ++index )
                    {
                        const auto& contact = contacts[index];
                        const auto pair = [&]( int a, int b ) { return ( contact.shapeA == shapes[a].shapeHandle && contact.shapeB == shapes[b].shapeHandle ) || ( contact.shapeA == shapes[b].shapeHandle && contact.shapeB == shapes[a].shapeHandle ); };
                        if( enabled ) check( !pair( 0, 1 ), "Filter must block only the A-B contact" );
                        hitA |= pair( 0, 2 );
                        hitB |= pair( 1, 2 );
                    }
                }
                movedA |= LengthSquared( simulation.GetBodyLinearVelocity( demo.getImpulseBody() ) ) > 0.01f;
                movedB |= LengthSquared( simulation.GetBodyLinearVelocity( demo.getTorqueBody() ) ) > 0.01f;
            }
            check( hitA && hitB && movedA && movedB, "third body must collide with and push both filtered bodies" );
            demo.resetFilterJointDemo();
            check( simulation.IsValid( demo.getFilterJoint() ) == enabled, "reset must preserve Filter state" );
            check( Length( simulation.GetBodyTransform( demo.getImpulseBody() ).position - vec2{ -0.35f, 0.0f } ) < 0.001f && LengthSquared( simulation.GetBodyLinearVelocity( demo.getTorqueBody() ) ) == 0.0f, "reset must restore the overlap and stop movement" );
            demo.prepareFilterSleepTest();
            const auto a = demo.getImpulseBody();
            const auto b = demo.getTorqueBody();
            check( !simulation.IsBodyAwake( a ) && !simulation.IsBodyAwake( b ), "sleep test must start with both bodies asleep" );
            demo.wakeFilterBody();
            check( simulation.IsBodyAwake( a ) && simulation.IsBodyAwake( b ) == enabled, "Filter must couple waking without requiring a collision" );
            demo.step( 1.0f / 60.0f, substeps );
            check( LengthSquared( simulation.GetBodyLinearVelocity( a ) ) == 0.0f && LengthSquared( simulation.GetBodyLinearVelocity( b ) ) == 0.0f, "waking through Filter must not apply an impulse" );
            demo.resetFilterJointDemo();
            check( simulation.GetBodyCount() == 3 && simulation.getJointCount() == ( enabled ? 1 : 0 ), "repeated trials must not accumulate bodies or joints" );
        }
    return EXIT_SUCCESS;
}
