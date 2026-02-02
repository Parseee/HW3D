project(Triangles CXX)
add_library(${PROJECT_NAME} SHARED
src/polygon.hpp
src/polygon.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC project_compiler_options
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
    PUBLIC project_compiler_options
)

target_link_options(runTests PRIVATE
    $<$<CONFIG:Debug>:-fsanitize=address,leak>
)


add_test(NAME GeometryTests COMMAND runTests)