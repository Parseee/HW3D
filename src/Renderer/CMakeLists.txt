project(Renderer CXX)

add_library(${PROJECT_NAME} SHARED
    src/renderer.cpp
    src/renderer.hpp
)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

link_directories(src)

set_target_properties(${PROJECT_NAME} PROPERTIES 
    LINKER_LANGUAGE CXX
    FOLDER "src"
)

target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # For the Triangles directory itself
        ${CMAKE_SOURCE_DIR}/src      # For the main src directory
        ${GLFW_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC project_compiler_options
    PUBLIC glfw
    PUBLIC OpenGL::GL
)
