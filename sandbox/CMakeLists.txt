# Keep sandbox-only dependencies out of the zonai physics library.

# add_subdirectory로 포함해도 상위 프로젝트가 아닌 zonai의 vendored ImGui를 사용함.
set(IMGUI_DIR "${PROJECT_SOURCE_DIR}/third_party/imgui")

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
    demo.cpp
    rigidBodyDemo.cpp
    rigidBodyDemoUi.cpp
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

# 실행 파일 옆에서 폰트와 재배포 라이선스를 찾을 수 있도록 함께 복사함.
add_custom_command(TARGET zonaiSandbox POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:zonaiSandbox>/assets"
)
