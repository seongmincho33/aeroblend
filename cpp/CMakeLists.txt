set(AEROBLEND_SOURCES
    src/main.cpp
    src/window.cpp
    src/shader.cpp
    src/gpu_mesh.cpp
    src/renderer.cpp
    src/sky.cpp
    src/terrain.cpp
    src/camera.cpp
    src/input.cpp
    src/hud.cpp
    src/projectile_renderer.cpp
    # vendored
    third_party/glad/src/gl.c
    third_party/imgui/imgui.cpp
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
    third_party/imgui/imgui_widgets.cpp
    third_party/imgui/imgui_impl_sdl2.cpp
    third_party/imgui/imgui_impl_opengl3.cpp
)

add_executable(aeroblend ${AEROBLEND_SOURCES})

target_include_directories(aeroblend PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
    ${SDL2_INCLUDE_DIRS}
)

target_link_directories(aeroblend PRIVATE
    ${SDL2_LIBRARY_DIRS}
)

target_link_libraries(aeroblend PRIVATE
    aeroblend_ffi
    ${SDL2_LIBRARIES}
    OpenGL::GL
)

# macOS: link required frameworks
if(APPLE)
    target_link_libraries(aeroblend PRIVATE
        "-framework CoreFoundation"
        "-framework Security"
    )
endif()

# Windows: Rust 런타임이 필요로 하는 시스템 라이브러리
if(WIN32)
    target_link_libraries(aeroblend PRIVATE
        ws2_32 userenv ntdll bcrypt advapi32
    )
endif()

# Copy shaders to build dir
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# ── Integration test: mouse flight ──
add_executable(test_mouse_flight
    tests/test_mouse_flight.cpp
    src/input.cpp
    # ImGui needed — InputHandler::process_event() uses ImGui::GetIO()
    third_party/imgui/imgui.cpp
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
    third_party/imgui/imgui_widgets.cpp
    third_party/imgui/imgui_impl_sdl2.cpp
    third_party/imgui/imgui_impl_opengl3.cpp
)

target_include_directories(test_mouse_flight PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
    ${SDL2_INCLUDE_DIRS}
)

target_link_directories(test_mouse_flight PRIVATE
    ${SDL2_LIBRARY_DIRS}
)

target_link_libraries(test_mouse_flight PRIVATE
    aeroblend_ffi
    ${SDL2_LIBRARIES}
)

if(APPLE)
    target_link_libraries(test_mouse_flight PRIVATE
        "-framework CoreFoundation"
        "-framework Security"
    )
endif()

if(WIN32)
    target_link_libraries(test_mouse_flight PRIVATE
        ws2_32 userenv ntdll bcrypt advapi32
    )
endif()
