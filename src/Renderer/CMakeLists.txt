project(Renderer CXX)

add_library(${PROJECT_NAME} SHARED
    src/renderer.cpp
    src/renderer.hpp
)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

target_link_libraries(${PROJECT_NAME}
    PRIVATE glfw
    PRIVATE OpenGL::GL
    PUBLIC project_compiler_options
)


