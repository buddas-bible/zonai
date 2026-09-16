# Test targets will be added together with the first math and physics implementations.

add_executable(vec2Tests
    math/vec2_test.cpp
)

target_link_libraries(vec2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME vec2Tests
    COMMAND vec2Tests
)


add_executable(rot2Tests
    math/rot2_test.cpp
)

target_link_libraries(rot2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME rot2Tests
    COMMAND rot2Tests
)

add_executable(transform2Tests
    math/transform2_test.cpp
)

target_link_libraries(transform2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME transform2Tests
    COMMAND transform2Tests
)

add_executable(aabb2Tests
    collision/aabb2_test.cpp
)

target_link_libraries(aabb2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME aabb2Tests
    COMMAND aabb2Tests
)

add_executable(circle2Tests
    geometry/circle2_test.cpp
)

target_link_libraries(circle2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME circle2Tests
    COMMAND circle2Tests
)

add_executable(polygon2Tests
    geometry/polygon2_test.cpp
)

target_link_libraries(polygon2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME polygon2Tests
    COMMAND polygon2Tests
)

add_executable(collideTests
    collision/narrowphase/collide_test.cpp
)

target_link_libraries(collideTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME collideTests
    COMMAND collideTests
)