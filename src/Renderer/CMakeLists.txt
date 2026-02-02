project(Renderer CXX)

add_library(${PROJECT_NAME} SHARED
    src/renderer.cpp
    src/renderer.hpp
)

if (NOT APPLE)
find_package(OpenGL REQUIRED)
else()
find_library(OpenGL_FRAMEWORK OpenGL REQUIRED)
endif()

find_package(glfw3 CONFIG REQUIRED)

target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # For the Triangles directory itself
        ${CMAKE_SOURCE_DIR}/src      # For the main src directory
)

if (NOT APPLE)
target_link_libraries(${PROJECT_NAME}
    PUBLIC project_compiler_options
    PRIVATE glfw
    PRIVATE OpenGL::GL
)
else()
target_link_libraries(${PROJECT_NAME}
    PUBLIC project_compiler_options
    PRIVATE glfw
    PRIVATE ${OpenGL_FRAMEWORK}
)
endif()

target_compile_definitions(${PROJECT_NAME}
    PRIVATE GL_SILENCE_DEPRECATION
)
