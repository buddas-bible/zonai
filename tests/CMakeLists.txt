# Focused executables keep regression failures attributable to their subsystem.

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

add_executable(segment2Tests
    geometry/segment2_test.cpp
)

target_link_libraries(segment2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME segment2Tests
    COMMAND segment2Tests
)

add_executable(capsule2Tests
    geometry/capsule2_test.cpp
)

target_link_libraries(capsule2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME capsule2Tests
    COMMAND capsule2Tests
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

add_executable(broadPhaseStressTests
    collision/broadphase/broad_phase_stress_test.cpp
)

target_link_libraries(broadPhaseStressTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME broadPhaseStressTests
    COMMAND broadPhaseStressTests
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

add_executable(broadPhaseLifecycleTests collision/broadphase/broad_phase_lifecycle_test.cpp)
target_link_libraries(broadPhaseLifecycleTests PRIVATE zonai::zonai)
add_test(NAME broadPhaseLifecycleTests COMMAND broadPhaseLifecycleTests)


add_executable(filterTests
    collision/filter_test.cpp
)

target_link_libraries(filterTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME filterTests
    COMMAND filterTests
)


add_executable(timeOfImpact2Tests
    collision/time_of_impact_test.cpp
)

target_link_libraries(timeOfImpact2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME timeOfImpact2Tests
    COMMAND timeOfImpact2Tests
)


add_executable(shapeCast2Tests
    collision/shape_cast_test.cpp
)

target_link_libraries(shapeCast2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME shapeCast2Tests
    COMMAND shapeCast2Tests
)


add_executable(distance2Tests
    collision/distance_test.cpp
)

target_link_libraries(distance2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME distance2Tests
    COMMAND distance2Tests
)


add_executable(sweep2Tests
    collision/sweep_test.cpp
)

target_link_libraries(sweep2Tests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME sweep2Tests
    COMMAND sweep2Tests
)


add_executable(shapeTests
    collision/shape_test.cpp
)

target_link_libraries(shapeTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME shapeTests
    COMMAND shapeTests
)


add_executable(bodyTests
    dynamics/body_test.cpp
)

target_link_libraries(bodyTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME bodyTests
    COMMAND bodyTests
)


add_executable(bodyShapeTests
    dynamics/body_shape_test.cpp
)

target_link_libraries(bodyShapeTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME bodyShapeTests
    COMMAND bodyShapeTests
)


add_executable(worldTests
    dynamics/world_test.cpp
)

target_link_libraries(worldTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME worldTests
    COMMAND worldTests
)

add_executable(shapeDispatchTests
    collision/narrowphase/shape_dispatch_test.cpp
)

target_link_libraries(shapeDispatchTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME shapeDispatchTests
    COMMAND shapeDispatchTests
)


add_executable(contactConstraintTests
    dynamics/contact_constraint_test.cpp
)

target_link_libraries(contactConstraintTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME contactConstraintTests
    COMMAND contactConstraintTests
)

add_executable(restitutionTests
    dynamics/restitution_test.cpp
)

target_link_libraries(restitutionTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME restitutionTests
    COMMAND restitutionTests
)

add_executable(islandTests
    dynamics/island_test.cpp
)

target_link_libraries(islandTests
    PRIVATE
        zonai::zonai
)

add_test(
    NAME islandTests
    COMMAND islandTests
)

add_executable(bodyShapeContactLifecycleTests dynamics/body_shape_contact_lifecycle_test.cpp)
target_link_libraries(bodyShapeContactLifecycleTests PRIVATE zonai::zonai)
add_test(NAME bodyShapeContactLifecycleTests COMMAND bodyShapeContactLifecycleTests)

add_executable(islandSleepCcdTests dynamics/island_sleep_ccd_test.cpp)
target_link_libraries(islandSleepCcdTests PRIVATE zonai::zonai)
add_test(NAME islandSleepCcdTests COMMAND islandSleepCcdTests)

add_executable(worldQuerySensorTests dynamics/world_query_sensor_test.cpp)
target_link_libraries(worldQuerySensorTests PRIVATE zonai::zonai)
add_test(NAME worldQuerySensorTests COMMAND worldQuerySensorTests)

add_executable(contactCacheTests dynamics/contact_cache_test.cpp)
target_link_libraries(contactCacheTests PRIVATE zonai::zonai)
add_test(NAME contactCacheTests COMMAND contactCacheTests)

add_executable(distanceJointTests dynamics/distance_joint_test.cpp)
target_link_libraries(distanceJointTests PRIVATE zonai::zonai)
add_test(NAME distanceJointTests COMMAND distanceJointTests)

add_executable(distanceJointWorldTests dynamics/distance_joint_world_test.cpp)
target_link_libraries(distanceJointWorldTests PRIVATE zonai::zonai)
add_test(NAME distanceJointWorldTests COMMAND distanceJointWorldTests)

add_executable(motorJointTests dynamics/motor_joint_test.cpp)
target_link_libraries(motorJointTests PRIVATE zonai::zonai)
add_test(NAME motorJointTests COMMAND motorJointTests)

add_executable(motorJointSpringTests dynamics/motor_joint_spring_test.cpp)
target_link_libraries(motorJointSpringTests PRIVATE zonai::zonai)
add_test(NAME motorJointSpringTests COMMAND motorJointSpringTests)

add_executable(motorJointSpringWorldTests dynamics/motor_joint_spring_world_test.cpp)
target_link_libraries(motorJointSpringWorldTests PRIVATE zonai::zonai)
add_test(NAME motorJointSpringWorldTests COMMAND motorJointSpringWorldTests)

add_executable(moverJointTests dynamics/mover_joint_test.cpp)
target_link_libraries(moverJointTests PRIVATE zonai::zonai)
add_test(NAME moverJointTests COMMAND moverJointTests)

add_executable(moverJointWorldTests dynamics/mover_joint_world_test.cpp)
target_link_libraries(moverJointWorldTests PRIVATE zonai::zonai)
add_test(NAME moverJointWorldTests COMMAND moverJointWorldTests)

add_executable(pogoJointTests dynamics/pogo_joint_test.cpp)
target_link_libraries(pogoJointTests PRIVATE zonai::zonai)
add_test(NAME pogoJointTests COMMAND pogoJointTests)

add_executable(pogoJointWorldTests dynamics/pogo_joint_world_test.cpp)
target_link_libraries(pogoJointWorldTests PRIVATE zonai::zonai)
add_test(NAME pogoJointWorldTests COMMAND pogoJointWorldTests)

add_executable(mouseJointTests dynamics/mouse_joint_test.cpp)
target_link_libraries(mouseJointTests PRIVATE zonai::zonai)
add_test(NAME mouseJointTests COMMAND mouseJointTests)
add_executable(mouseJointWorldTests dynamics/mouse_joint_world_test.cpp)
target_link_libraries(mouseJointWorldTests PRIVATE zonai::zonai)
add_test(NAME mouseJointWorldTests COMMAND mouseJointWorldTests)

add_executable(revoluteJointTests dynamics/revolute_joint_test.cpp)
target_link_libraries(revoluteJointTests PRIVATE zonai::zonai)
add_test(NAME revoluteJointTests COMMAND revoluteJointTests)

add_executable(revoluteJointWorldTests dynamics/revolute_joint_world_test.cpp)
target_link_libraries(revoluteJointWorldTests PRIVATE zonai::zonai)
add_test(NAME revoluteJointWorldTests COMMAND revoluteJointWorldTests)

add_executable(wheelJointTests dynamics/wheel_joint_test.cpp)
target_link_libraries(wheelJointTests PRIVATE zonai::zonai)
add_test(NAME wheelJointTests COMMAND wheelJointTests)

add_executable(wheelJointWorldTests dynamics/wheel_joint_world_test.cpp)
target_link_libraries(wheelJointWorldTests PRIVATE zonai::zonai)
add_test(NAME wheelJointWorldTests COMMAND wheelJointWorldTests)

add_executable(prismaticJointTests dynamics/prismatic_joint_test.cpp)
target_link_libraries(prismaticJointTests PRIVATE zonai::zonai)
add_test(NAME prismaticJointTests COMMAND prismaticJointTests)

add_executable(prismaticJointWorldTests dynamics/prismatic_joint_world_test.cpp)
target_link_libraries(prismaticJointWorldTests PRIVATE zonai::zonai)
add_test(NAME prismaticJointWorldTests COMMAND prismaticJointWorldTests)

add_executable(weldJointTests dynamics/weld_joint_test.cpp)
target_link_libraries(weldJointTests PRIVATE zonai::zonai)
add_test(NAME weldJointTests COMMAND weldJointTests)

add_executable(weldJointWorldTests dynamics/weld_joint_world_test.cpp)
target_link_libraries(weldJointWorldTests PRIVATE zonai::zonai)
add_test(NAME weldJointWorldTests COMMAND weldJointWorldTests)

add_executable(sandboxDemoTests sandbox/demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxDemoTests COMMAND sandboxDemoTests)

add_executable(sandboxJointDemoTests sandbox/joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxJointDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxJointDemoTests COMMAND sandboxJointDemoTests)

add_executable(sandboxWeldSoftnessDemoTests sandbox/weld_softness_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxWeldSoftnessDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxWeldSoftnessDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxWeldSoftnessDemoTests COMMAND sandboxWeldSoftnessDemoTests)

# Windows Sandbox 구성에서는 실제 ImGui view도 headless frame으로 검사함.
if(TARGET imgui)
    target_sources(sandboxDemoTests PRIVATE sandbox/demo_ui_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemoUi.cpp" "${PROJECT_SOURCE_DIR}/sandbox/debug/debugCamera.cpp" "${PROJECT_SOURCE_DIR}/sandbox/debug/debugDraw.cpp")
    target_link_libraries(sandboxDemoTests PRIVATE imgui)
    target_compile_definitions(sandboxDemoTests PRIVATE ZONAI_TEST_SANDBOX_UI)
    add_custom_command(TARGET sandboxDemoTests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/sandbox/assets" "$<TARGET_FILE_DIR:sandboxDemoTests>/assets"
    )

    add_executable(sandboxJointViewTests
        sandbox/joint_demo_ui_test.cpp
        "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp"
        "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp"
        "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp"
        "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemoUi.cpp"
        "${PROJECT_SOURCE_DIR}/sandbox/jointDemoView.cpp"
        "${PROJECT_SOURCE_DIR}/sandbox/debug/debugCamera.cpp"
        "${PROJECT_SOURCE_DIR}/sandbox/debug/debugDraw.cpp"
    )
    target_include_directories(sandboxJointViewTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
    target_link_libraries(sandboxJointViewTests PRIVATE zonai::zonai imgui)
    add_test(NAME sandboxJointViewTests COMMAND sandboxJointViewTests)
    add_custom_command(TARGET sandboxJointViewTests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/sandbox/assets" "$<TARGET_FILE_DIR:sandboxJointViewTests>/assets"
    )
endif()

add_executable(sandboxMoverJointDemoTests sandbox/mover_joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxMoverJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxMoverJointDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxMoverJointDemoTests COMMAND sandboxMoverJointDemoTests)

add_executable(sandboxPogoJointDemoTests sandbox/pogo_joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxPogoJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxPogoJointDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxPogoJointDemoTests COMMAND sandboxPogoJointDemoTests)

add_executable(sandboxMotorJointDemoTests sandbox/motor_joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxMotorJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxMotorJointDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxMotorJointDemoTests COMMAND sandboxMotorJointDemoTests)
