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

option(ZONAI_BUILD_SANDBOX "Build the zonai sandbox application" ON)
option(ZONAI_BUILD_TESTS "Build zonai tests" ON)

add_subdirectory(src)

if(ZONAI_BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()

if(ZONAI_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
