cmake_minimum_required(VERSION 3.18)
project(HW3D CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# whatever windows needs
# For Windows: Prevent overriding the parent project's compiler/linker settings
if (WIN32)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif (WIN32)

FetchContent_MakeAvailable(googletest)

add_library(project_compiler_options INTERFACE)
target_compile_options(project_compiler_options INTERFACE
    $<$<CONFIG:Debug>:-Wall -Wextra -g -fno-omit-frame-pointer -fsanitize=address,leak>
    $<$<CONFIG:Release>:-O2>
)

add_subdirectory(src/Triangles)
include_directories(src)

set(TRIANGLES_SOURCES 
    src/main.cpp
    )
    
add_executable(HW3D ${TRIANGLES_SOURCES})

target_link_libraries(
    HW3D PRIVATE project_compiler_options)

target_compile_options(project_compiler_options INTERFACE
  $<$<CONFIG:Debug>:-Wall -Wextra -g -fno-omit-frame-pointer -fsanitize=address,leak>
  $<$<CONFIG:Release>:-O2>
)

target_link_options(HW3D PRIVATE
  $<$<CONFIG:Debug>:-fsanitize=address,leak>
)