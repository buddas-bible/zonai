from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected 1 match, got {count}")
    return text.replace(old, new, 1)


cmake_path = Path("tests/CMakeLists.txt")
cmake = cmake_path.read_text(encoding="utf-8")
pogo_block = """add_executable(pogoJointWorldTests dynamics/pogo_joint_world_test.cpp)
target_link_libraries(pogoJointWorldTests PRIVATE zonai::zonai)
add_test(NAME pogoJointWorldTests COMMAND pogoJointWorldTests)
"""
filter_block = """
add_executable(filterJointTests dynamics/filter_joint_test.cpp)
target_link_libraries(filterJointTests PRIVATE zonai::zonai)
add_test(NAME filterJointTests COMMAND filterJointTests)

add_executable(filterJointWorldTests dynamics/filter_joint_world_test.cpp)
target_link_libraries(filterJointWorldTests PRIVATE zonai::zonai)
add_test(NAME filterJointWorldTests COMMAND filterJointWorldTests)
"""
cmake = replace_once(cmake, pogo_block, pogo_block + filter_block, "Filter CMake targets")
cmake_path.write_text(cmake, encoding="utf-8")

pogo_path = Path("tests/dynamics/pogo_joint_world_test.cpp")
pogo = pogo_path.read_text(encoding="utf-8")
red_block = """    // Filter Joint는 solver 없이 두 Body 사이 collision만 차단하고 공용 Joint lifecycle에 참여함.
    {
        world filterWorld;
        filterWorld.SetGravity( {} );
        const bodyId filterA = filterWorld.CreateBody( bodyType::Dynamic );
        const bodyId filterB = filterWorld.CreateBody( bodyType::Dynamic );
        ( void )filterWorld.CreateShape( filterA, circle2{ {}, 0.5f } );
        ( void )filterWorld.CreateShape( filterB, circle2{ {}, 0.5f } );
        filterWorld.Step( h, 1 );
        check( filterWorld.GetContactCount() == 1, "Filter RED starts with an overlapping Contact" );

        filterJointDef filterDefinition{};
        filterDefinition.bodyA = filterA;
        filterDefinition.bodyB = filterB;
        const jointId filter = filterWorld.createFilterJoint( filterDefinition );
        check( filterWorld.GetContactCount() == 0, "Filter creation removes existing contacts immediately" );
        check( !filterWorld.getFilterJointData( filter ).collideConnected, "Filter disables connected collision by default" );
    }

"""
pogo = replace_once(pogo, red_block, "", "temporary Filter RED block")
pogo_path.write_text(pogo, encoding="utf-8")

workflow_path = Path(".github/workflows/cmake.yml")
workflow = workflow_path.read_text(encoding="utf-8")
workflow = replace_once(
    workflow,
    "pogoJointTests pogoJointWorldTests mouseJointTests",
    "pogoJointTests pogoJointWorldTests filterJointTests filterJointWorldTests mouseJointTests",
    "Release build target list",
)
workflow = replace_once(
    workflow,
    "pogoJointTests|pogoJointWorldTests|mouseJointTests",
    "pogoJointTests|pogoJointWorldTests|filterJointTests|filterJointWorldTests|mouseJointTests",
    "Release ctest regex",
)
workflow_path.write_text(workflow, encoding="utf-8")

print("Finalized Filter Joint focused tests and Release CI")
