cmake_minimum_required(VERSION 3.28)
project(draw)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_COLOR_DIAGNOSTICS ON)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_definitions(BUILD_DEBUG)
    add_compile_options(-fsanitize=address,undefined -fstack-protector)
    add_link_options(-fsanitize=address,undefined -fstack-protector)
endif()

add_executable(draw main.cpp controller.hpp serial.hpp point.hpp
        path.hpp)

add_library(imgui imgui/imgui_draw.cpp imgui/imgui_demo.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp imgui/imgui.cpp imgui/backends/imgui_impl_sdl2.cpp imgui/backends/imgui_impl_opengl3.cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
target_link_libraries(imgui PUBLIC PkgConfig::SDL2)

find_package(OpenGL REQUIRED)

add_subdirectory(nativefiledialog-extended)

target_include_directories(imgui PUBLIC imgui/)
target_include_directories(draw PUBLIC imgui/ cereal/include)
target_link_libraries(draw PUBLIC imgui PkgConfig::SDL2 OpenGL::GL nfd)

