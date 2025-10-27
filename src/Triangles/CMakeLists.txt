project(Triangles CXX)

add_library(${PROJECT_NAME} STATIC
src/polygon.hpp
src/polygon.cpp
)

link_directories(src)

set_target_properties(${PROJECT_NAME} PROPERTIES 
    LINKER_LANGUAGE CXX
    FOLDER "src"
)

target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # For the Triangles directory itself
        ${CMAKE_SOURCE_DIR}/src      # For the main src directory
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC 
        project_compiler_options
)

# ------- TESTING --------

add_executable(runTests
    tests/test_polygon.cpp
)

target_include_directories(runTests 
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(runTests
    PRIVATE
        ${PROJECT_NAME}
        GTest::gtest
        GTest::gtest_main
        project_compiler_options
)

if (LINUX)
    target_link_options(runTests PRIVATE
        $<$<CONFIG:Debug>:-fsanitize=address,leak>
    )
endif()

add_test(NAME GeometryTests COMMAND runTests)