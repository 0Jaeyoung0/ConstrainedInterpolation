cmake_minimum_required(VERSION 3.16)
project(ConstrainedIntepolation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(third/glfw)

add_executable(ConstrainedIntepolation 
    src/main.cpp
    src/ConstrainedIntepolation.cpp 
    third/glad/src/glad.c
    src/Interpolate/Polynomial.cpp
    src/Interpolate/Explicit/ExplicitInterpolate.cpp
    src/Interpolate/Explicit/QuinticHermiteSpline.cpp
)

target_include_directories(ConstrainedIntepolation PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third/eigen
    ${CMAKE_SOURCE_DIR}/third/glad/include
)


target_link_libraries(ConstrainedIntepolation PRIVATE 
    glfw
)