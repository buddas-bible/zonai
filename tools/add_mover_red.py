from pathlib import Path

cmake_path = Path("tests/CMakeLists.txt")
cmake = cmake_path.read_text(encoding="utf-8")
marker = "add_executable(mouseJointTests dynamics/mouse_joint_test.cpp)\n"
block = (
    "add_executable(moverJointTests dynamics/mover_joint_test.cpp)\n"
    "target_link_libraries(moverJointTests PRIVATE zonai::zonai)\n"
    "add_test(NAME moverJointTests COMMAND moverJointTests)\n\n"
)
if block not in cmake:
    if marker not in cmake:
        raise RuntimeError("CMake insertion marker not found")
    cmake = cmake.replace(marker, block + marker, 1)
    cmake_path.write_text(cmake, encoding="utf-8")

test_path = Path("tests/dynamics/mover_joint_test.cpp")
test_path.write_text(r'''#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/moverJointConstraint2.h"

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

bool near( float a, float b )
{
    return std::abs( a - b ) < 0.0001f;
}

}

int main()
{
    bodySim bodyA{};
    bodyA.bodyId = 0;

    bodySim bodyB{};
    bodyB.bodyId = 1;
    bodyB.invMass = 1.0f;
    bodyB.invInertia = 1.0f;

    moverJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.linearVelocity = { 2.0f, -1.0f };
    joint.maxVelocityForce = { 1000.0f, 1000.0f };

    const float h = 1.0f / 60.0f;
    auto constraint = prepareMoverJointConstraint( joint, bodyA, bodyB, h );
    bodyState stateA{};
    bodyState stateB{};

    solveMoverJointConstraint( constraint, stateA, stateB );

    check( near( stateB.linearVelocity.x - stateA.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.y - stateA.linearVelocity.y, -1.0f ), "Mover reaches desired relative linear velocity" );
    check( near( stateA.angularVelocity, 0.0f ) && near( stateB.angularVelocity, 0.0f ), "Mover does not affect rotation" );

    return EXIT_SUCCESS;
}
''', encoding="utf-8")
