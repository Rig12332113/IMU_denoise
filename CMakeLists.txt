cmake_minimum_required(VERSION 3.10)

project(IMU_denoise LANGUAGES CXX)

find_package(nlohmann_json REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# ui_lib
add_library(ui_lib STATIC
    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_glfw.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp

    external/implot/implot.cpp
    external/implot/implot_items.cpp
)
target_include_directories(ui_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/external/implot
)
target_compile_definitions(ui_lib PUBLIC IMGUI_IMPL_OPENGL_LOADER_GL3W=0)

# IMU_viewer
add_executable(IMU_viewer
    src/IMUviewer.cpp
)
target_include_directories(IMU_viewer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(IMU_viewer PRIVATE
    ui_lib
    glfw
    OpenGL::GL
)

# IMU_server
add_executable(IMU_server
    src/receiver.cpp
    src/IMUsample.cpp
    src/waveletDenoiser.cpp
)

target_include_directories(IMU_server
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(IMU_server
    PRIVATE nlohmann_json::nlohmann_json
)
