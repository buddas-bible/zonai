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
    check( getDemoEntries().size() == 13, "Filter Joint demo is independently selectable" );
    check( std::string_view{ getDemoEntry( demoKind::filterJointPlayground ).category } == "조인트", "Filter Joint demo stays in joint category" );

    rigidBodyDemo filter{ demoKind::filterJointPlayground };
    check( filter.getWorld().GetBodyCount() == 2 && filter.getShapes().size() == 2, "Filter scene contains two dynamic bodies" );
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

    return EXIT_SUCCESS;
}
