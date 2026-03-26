cmake_minimum_required(VERSION 3.20)
project(pgo2d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(raylib REQUIRED)

# opt lib
add_library(opt STATIC
    src/vertex.cpp
    src/edge.cpp
    src/problem.cpp
)
target_include_directories(opt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(opt PUBLIC Eigen3::Eigen)

# app
add_executable(${PROJECT_NAME}
    app/main.cpp
    app/vis.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/app)
target_link_libraries(${PROJECT_NAME} PRIVATE opt raylib)
