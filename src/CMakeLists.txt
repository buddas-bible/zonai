file(GLOB_RECURSE ZONAI_HEADERS
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

file(GLOB_RECURSE ZONAI_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(zonai INTERFACE)
add_library(zonai::zonai ALIAS zonai)

target_sources(zonai
    INTERFACE
        FILE_SET HEADERS
        BASE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            ${ZONAI_HEADERS}
)

target_include_directories(zonai
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(zonai
    INTERFACE
        cxx_std_20
)