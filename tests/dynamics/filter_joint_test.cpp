#include <cstdio>
#include <cstdlib>

#include "dynamics/joints/filterJointConstraint2.h"

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
    filterJointSim2 joint{};
    joint.jointId = 7;
    joint.bodyIdA = 3;
    joint.bodyIdB = 5;

    filterJointConstraint2 constraint = prepareFilterJointConstraint( joint );
    check( constraint.jointId == 7 && constraint.bodyIdA == 3 && constraint.bodyIdB == 5, "Filter prepare preserves identity for island indexing" );

    // Filter Joint는 solver state가 없으므로 두 pass 모두 호출해도 packet 자체가 변하지 않음.
    warmStartFilterJointConstraint( constraint );
    solveFilterJointConstraint( constraint );
    solveFilterJointConstraint( constraint );
    check( constraint.jointId == 7 && constraint.bodyIdA == 3 && constraint.bodyIdB == 5, "Filter solver hooks are explicit no-ops" );

    return EXIT_SUCCESS;
}
