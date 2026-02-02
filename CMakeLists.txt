cmake_minimum_required(VERSION 3.18)
project(HW3D CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()
find_package(GTest REQUIRED)

add_library(project_compiler_options INTERFACE)
target_compile_options(project_compiler_options INTERFACE
    -Wall -Wextra
    $<$<CONFIG:Debug>: -g -fno-omit-frame-pointer -fsanitize=address,leak>
    $<$<CONFIG:Release>:-O2>
)

include_directories(src)
add_subdirectory(src/Triangles)
add_subdirectory(src/Renderer)

set(TRIANGLES_SOURCES 
    src/main.cpp
)
    
add_executable(${PROJECT_NAME} ${TRIANGLES_SOURCES})

target_link_libraries(${PROJECT_NAME}
    PRIVATE 
        project_compiler_options
        Triangles
        Renderer
)

target_link_options(${PROJECT_NAME} PUBLIC
    $<$<CONFIG:Debug>: -g -fno-omit-frame-pointer -fsanitize=address,leak>
)
