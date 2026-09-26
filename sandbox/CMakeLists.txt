# The zonai sandbox target will be added with the Win32 + Direct3D 11 + Dear ImGui bootstrap.
# Keep sandbox-only dependencies out of the zonai physics library.

set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/third_party/imgui")

add_library(imgui STATIC
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"

    "${IMGUI_DIR}/backends/imgui_impl_win32.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_dx11.cpp"
)

target_include_directories(imgui
    PUBLIC
        "${IMGUI_DIR}"
        "${IMGUI_DIR}/backends"
)

target_link_libraries(imgui
    PRIVATE
        d3d11
)

add_executable(zonaiSandbox
    main.cpp
    debug/debugCamera.cpp
    debug/debugDraw.cpp
)

target_link_libraries(zonaiSandbox
    PRIVATE
        zonai::zonai
        imgui
        d3d11
        dxgi
)

target_compile_features(zonaiSandbox
    PRIVATE
        cxx_std_20
)

target_compile_definitions(zonaiSandbox
    PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
)
