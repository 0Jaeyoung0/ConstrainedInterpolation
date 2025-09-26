cmake_minimum_required(VERSION 3.16)
project(ConstrainedIntepolation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/third/Interpolate/include
    ${CMAKE_SOURCE_DIR}/third/glfw/include
    ${CMAKE_SOURCE_DIR}/third/glad/include
)

link_directories(${CMAKE_SOURCE_DIR}/third/Interpolate/lib)
link_directories(${CMAKE_SOURCE_DIR}/third/glfw/lib)

find_package(Eigen3 REQUIRED)

add_executable(ConstrainedIntepolation src/main.cpp src/ConstrainedIntepolation.cpp third/glad/src/glad.c)

target_link_libraries(ConstrainedIntepolation PRIVATE 
    Interpolate 
    Eigen3::Eigen
    glfw3
)