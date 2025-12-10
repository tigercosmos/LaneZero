cmake_minimum_required(VERSION 3.20)
project(LaneZero VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Fetch pybind11
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# C++ library sources
set(LANEZERO_SOURCES
    src/map/Map.cpp
    src/vehicle/Vehicle.cpp
    src/simulation/Simulation.cpp
)

# Python module
pybind11_add_module(_core
    ${LANEZERO_SOURCES}
    src/pymod.cpp
    src/vehicle/pymod/wrap_Vehicle.cpp
    src/map/pymod/wrap_Map.cpp
    src/simulation/pymod/Wrap_Simulation.cpp
)

target_include_directories(_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/map
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation
)

# Install the Python module
install(TARGETS _core LIBRARY DESTINATION LaneZero)

# Optionally build a standalone executable (if main.cpp exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
    add_executable(LaneZero
        src/main.cpp
        ${LANEZERO_SOURCES}
    )
    target_include_directories(LaneZero PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

# Enable testing
enable_testing()

# Google Test support
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/gtest)
    add_subdirectory(gtest)
endif()
