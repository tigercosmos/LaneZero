cmake_minimum_required(VERSION 3.20)
project(LaneZero VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(LaneZero
    src/main.cpp
    src/Map.cpp
    src/Vehicle.cpp
    src/Simulation.cpp
)

target_include_directories(LaneZero PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test)
    add_subdirectory(test)
endif()
