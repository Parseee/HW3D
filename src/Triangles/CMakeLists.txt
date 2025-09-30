add_library(Triangles STATIC
    src/polygon.hpp
    src/polygon.cpp
)

set_target_properties(Triangles PROPERTIES LINKER_LANGUAGE CXX)

set(CMAKE_CXX_STANDART 20)

set_target_properties(Triangles PROPERTIES FOLDER "src")

target_include_directories(Triangles
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Для сборки проекта
        $<INSTALL_INTERFACE:include>                           # Для будущей установки
)