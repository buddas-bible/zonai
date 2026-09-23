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

add_executable(segmentCircleTests
    collision/narrowphase/segment_circle_test.cpp
)

target_link_libraries(segmentCircleTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME segmentCircleTests
    COMMAND segmentCircleTests
)

add_executable(capsuleCapsuleTests
    collision/narrowphase/capsule_capsule_test.cpp
)

target_link_libraries(capsuleCapsuleTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME capsuleCapsuleTests
    COMMAND capsuleCapsuleTests
)

add_executable(polygonPolygonTests
    collision/narrowphase/polygon_polygon_test.cpp
)

target_link_libraries(polygonPolygonTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME polygonPolygonTests
    COMMAND polygonPolygonTests
)

add_executable(polygonCapsuleTests
    collision/narrowphase/polygon_capsule_test.cpp
)

target_link_libraries(polygonCapsuleTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME polygonCapsuleTests
    COMMAND polygonCapsuleTests
)

add_executable(segmentCapsuleTests
    collision/narrowphase/segment_capsule_test.cpp
)

target_link_libraries(segmentCapsuleTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME segmentCapsuleTests
    COMMAND segmentCapsuleTests
)

add_executable(polygonSegmentTests
    collision/narrowphase/polygon_segment_test.cpp
)

target_link_libraries(polygonSegmentTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME polygonSegmentTests
    COMMAND polygonSegmentTests
)

add_executable(dynamicTreeTests
    collision/broadphase/dynamic_tree_test.cpp
)

target_link_libraries(dynamicTreeTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME dynamicTreeTests
    COMMAND dynamicTreeTests
)

add_executable(bodyTypeTests
    collision/broadphase/body_type_test.cpp
)

target_link_libraries(bodyTypeTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME bodyTypeTests
    COMMAND bodyTypeTests
)

add_executable(broadPhaseTests
    collision/broadphase/broad_phase_test.cpp
)

target_link_libraries(broadPhaseTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME broadPhaseTests
    COMMAND broadPhaseTests
)

add_executable(hashSetTests
    collision/broadphase/hash_set_test.cpp
)

target_link_libraries(hashSetTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME hashSetTests
    COMMAND hashSetTests
)
