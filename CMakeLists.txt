cmake_minimum_required(VERSION 3.16)
project(TerminalRaytracer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

add_executable(raytracer
    src/main.cpp
    src/vec3.cpp
    src/ray.cpp
    src/sphere.cpp
    src/camera.cpp
    src/renderer.cpp
)

target_include_directories(raytracer PRIVATE include)