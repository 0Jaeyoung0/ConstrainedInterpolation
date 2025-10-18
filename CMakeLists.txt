cmake_minimum_required(VERSION 3.16)
project(ConstrainedIntepolation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ConstrainedIntepolation 
    src/main.cpp
    src/ConstrainedIntepolation.cpp 
    third/glad/src/glad.c
)

target_include_directories(ConstrainedIntepolation PRIVATE
    ${CMAKE_SOURCE_DIR}/third/eigen
    ${CMAKE_SOURCE_DIR}/third/Interpolate/include
    ${CMAKE_SOURCE_DIR}/third/glfw/include
    ${CMAKE_SOURCE_DIR}/third/glad/include
)

target_link_directories(ConstrainedIntepolation PRIVATE 
    ${CMAKE_SOURCE_DIR}/third/Interpolate/lib
    ${CMAKE_SOURCE_DIR}/third/glfw/lib
)

target_link_libraries(ConstrainedIntepolation PRIVATE 
    Interpolate
    glfw3
)