cmake_minimum_required(VERSION 3.16)
project(TerminalRaytracer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(raytracer src/main.cpp)
add_executable(solar_system_demo src/solar_system.cpp)
add_executable(solar_sim_simple src/simple_main.cpp)

target_include_directories(raytracer PRIVATE include)
target_include_directories(solar_system_demo PRIVATE include)
target_include_directories(solar_sim_simple PRIVATE include)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(raytracer PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(solar_system_demo PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(solar_sim_simple PRIVATE -Wall -Wextra -Wpedantic)
endif()
