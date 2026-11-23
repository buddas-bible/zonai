file(GLOB_RECURSE ZONAI_HEADERS
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

file(GLOB_RECURSE ZONAI_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(zonai STATIC
    ${ZONAI_SOURCES}
)

add_library(zonai::zonai ALIAS zonai)

target_sources(zonai
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            ${ZONAI_HEADERS}
)

target_include_directories(zonai
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(zonai
    PUBLIC
        cxx_std_20
)

# Deterministic physics requires identical FP contraction/reassociation policy
# across the engine and consumers compiling math in zonai's public headers.
# This is not by itself a cross-platform determinism guarantee: libm routines,
# FP environment, traversal order and CPU behavior are audited separately.
if(MSVC)
    # Since VS 2022, /fp:precise leaves fp_contract off by default.
    # Do not append /fp:contract or /fp:fast from consumer targets.
    # Preserve UTF-8 for public headers with Korean comments.
    target_compile_options(zonai PUBLIC /utf-8 /fp:precise)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
    target_compile_options(zonai PUBLIC -fno-fast-math -ffp-contract=off)
endif()
