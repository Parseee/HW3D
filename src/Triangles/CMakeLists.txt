add_library(Triangles STATIC
    src/polygon.hpp
    src/polygon.cpp
)

set_target_properties(Triangles PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(Triangles PROPERTIES FOLDER "src")

set(CMAKE_CXX_STANDART 20)

target_include_directories(Triangles 
    PUBLIC src
)

target_link_libraries(Triangles
    PUBLIC project_compiler_options)

# ------- TESTING --------

enable_testing()

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(runTests
    tests/test_polygon.cpp
)

target_include_directories(runTests PRIVATE src)

target_link_libraries(runTests
    Triangles
    # gtest_main
    ${GTEST_LIBRARIES}
    pthread
    project_compiler_options
)

target_link_options(runTests PRIVATE
  $<$<CONFIG:Debug>:-fsanitize=address,leak>
)

add_test(NAME GeometryTests COMMAND runTests)