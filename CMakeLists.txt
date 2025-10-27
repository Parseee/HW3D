cmake_minimum_required(VERSION 3.18)
project(HW3D CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

find_package(GTest REQUIRED)

# whatever windows needs
# For Windows: Prevent overriding the parent project's compiler/linker settings
if (WIN32)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif (WIN32)

add_library(project_compiler_options INTERFACE)
if (LINUX)
    target_compile_options(project_compiler_options INTERFACE
        $<$<CONFIG:Debug>:-Wall -Wextra -g -fno-omit-frame-pointer -fsanitize=address,leak>
        $<$<CONFIG:Release>:-O2>
    )
endif(LINUX)
if(APPLE)
    target_compile_options(project_compiler_options INTERFACE
        $<$<CONFIG:Debug>:-Wall -Wextra -g -fno-omit-frame-pointer>
        $<$<CONFIG:Release>:-O2>
    )
endif(APPLE)

add_subdirectory(src/Triangles)
include_directories(src)

set(TRIANGLES_SOURCES 
    src/main.cpp
    )
    
add_executable(HW3D ${TRIANGLES_SOURCES})

target_link_libraries(
    HW3D PRIVATE project_compiler_options
         PUBLIC GTest::gtest GTest::gtest_main)

if (LINUX)
    target_link_options(HW3D PRIVATE
    $<$<CONFIG:Debug>:-fsanitize=address,leak>
    )
endif(LINUX)