cmake_minimum_required(VERSION 4.2)

project(
    zonai
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(ZONAI_BUILD_SANDBOX "Build the zonai sandbox application" ${WIN32})
option(ZONAI_BUILD_TESTS "Build zonai tests" ON)

if(ZONAI_BUILD_SANDBOX AND NOT WIN32)
    message(FATAL_ERROR "The zonai sandbox requires Windows. Set ZONAI_BUILD_SANDBOX=OFF to build the physics library and tests.")
endif()

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_subdirectory(src)

if(ZONAI_BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()

if(ZONAI_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)

    add_executable(jointIncludePathTests
        tests/dynamics/joint_include_path_test.cpp
    )
    target_link_libraries(jointIncludePathTests PRIVATE zonai::zonai)
    add_test(NAME jointIncludePathTests COMMAND jointIncludePathTests)
endif()
