add_library(zonai INTERFACE)
add_library(zonai::zonai ALIAS zonai)

target_include_directories(zonai
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(zonai
    INTERFACE
        cxx_std_20
)