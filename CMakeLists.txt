cmake_minimum_required(VERSION 3.20)
project(LaneZero VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Create LaneZero include directory structure for proper header includes
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include)
file(GLOB_RECURSE HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")
foreach(HEADER ${HEADER_FILES})
    file(RELATIVE_PATH REL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src" ${HEADER})
    get_filename_component(HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/LaneZero/${REL_PATH}" DIRECTORY)
    file(MAKE_DIRECTORY ${HEADER_DIR})
    configure_file(${HEADER} "${CMAKE_CURRENT_BINARY_DIR}/include/LaneZero/${REL_PATH}" COPYONLY)
endforeach()

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Option to build with Qt GUI
option(LANEZERO_USE_QT "Build with Qt GUI support" ON)

if(LANEZERO_USE_QT)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    
    # Find system Qt6
    find_package(Qt6 COMPONENTS Core Widgets REQUIRED)
    
    # Find PySide6 libraries for linking (required when using Qt)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import PySide6; import os; print(os.path.dirname(PySide6.__file__))"
        OUTPUT_VARIABLE PYSIDE6_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE PYSIDE6_IMPORT_RESULT
        ERROR_QUIET
    )
    
    if(NOT PYSIDE6_IMPORT_RESULT EQUAL 0)
        message(FATAL_ERROR "PySide6 is required when LANEZERO_USE_QT=ON. Please install PySide6: pip install pyside6")
    endif()
    
    message(STATUS "PySide6 path: ${PYSIDE6_PATH}")
    
    # Add PySide6 include directories
    if(EXISTS "${PYSIDE6_PATH}/include")
        include_directories("${PYSIDE6_PATH}/include")
    endif()
    
    # Find PySide6 and shiboken6 libraries
    file(GLOB PYSIDE6_LIBRARY_CANDIDATES 
        "${PYSIDE6_PATH}/libpyside6.*.dylib"
        "${PYSIDE6_PATH}/libpyside6.*.so*"
    )
    if(PYSIDE6_LIBRARY_CANDIDATES)
        list(GET PYSIDE6_LIBRARY_CANDIDATES 0 PYSIDE6_LIBRARY)
        message(STATUS "PySide6 library: ${PYSIDE6_LIBRARY}")
    else()
        message(FATAL_ERROR "PySide6 library not found in ${PYSIDE6_PATH}. Please ensure PySide6 is properly installed.")
    endif()
    
    # Find shiboken6 library
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import shiboken6; import os; print(os.path.dirname(shiboken6.__file__))"
        OUTPUT_VARIABLE SHIBOKEN6_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE SHIBOKEN6_IMPORT_RESULT
        ERROR_QUIET
    )
    
    if(NOT SHIBOKEN6_IMPORT_RESULT EQUAL 0)
        message(FATAL_ERROR "shiboken6 is required when LANEZERO_USE_QT=ON. Please install PySide6: pip install pyside6")
    endif()
    
    file(GLOB SHIBOKEN6_LIBRARY_CANDIDATES
        "${SHIBOKEN6_PATH}/libshiboken6.*.dylib"
        "${SHIBOKEN6_PATH}/libshiboken6.*.so*"
    )
    if(SHIBOKEN6_LIBRARY_CANDIDATES)
        list(GET SHIBOKEN6_LIBRARY_CANDIDATES 0 SHIBOKEN6_LIBRARY)
        message(STATUS "shiboken6 library: ${SHIBOKEN6_LIBRARY}")
    else()
        message(FATAL_ERROR "shiboken6 library not found in ${SHIBOKEN6_PATH}. Please ensure PySide6 is properly installed.")
    endif()
endif()

# Fetch pybind11
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# Fetch RapidJSON
FetchContent_Declare(
    rapidjson
    GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
    GIT_TAG master
)
set(RAPIDJSON_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(RAPIDJSON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RAPIDJSON_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(rapidjson)

# C++ library sources
set(LANEZERO_SOURCES
    src/map/Map.cpp
    src/vehicle/Vehicle.cpp
    src/simulation/Simulation.cpp
    src/scenario/Scenario.cpp
    src/planner/BehaviorPlanner.cpp
    src/planner/MotionPlanner.cpp
    src/planner/ControlPlanner.cpp
    src/planner/PlanningOrchestrator.cpp
    src/physics_engine/BicycleModelEngine.cpp
)

# Viewer sources (Qt GUI)
set(LANEZERO_VIEWER_SOURCES
    src/viewer/RManager.cpp
    src/viewer/RenderWidget.cpp
    src/viewer/wrap_viewer.cpp
)

# Python module
pybind11_add_module(_core
    ${LANEZERO_SOURCES}
    src/pymod.cpp
    src/vehicle/pymod/wrap_Vehicle.cpp
    src/map/pymod/wrap_Map.cpp
    src/simulation/pymod/Wrap_Simulation.cpp
    src/simulation/pymod/Wrap_WorldState.cpp
    src/physics_engine/pymod/wrap_PhysicsEngine.cpp
)

target_include_directories(_core PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/map
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/scenario
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner
    ${CMAKE_CURRENT_SOURCE_DIR}/src/physics_engine
    ${rapidjson_SOURCE_DIR}/include
)

if(LANEZERO_USE_QT)
    target_sources(_core PRIVATE ${LANEZERO_VIEWER_SOURCES})
    target_link_libraries(_core PRIVATE Qt6::Core Qt6::Widgets)
    target_compile_definitions(_core PRIVATE LANEZERO_USE_QT)
    
    if (UNIX AND NOT APPLE)
        # Link PySide6 library (required for pybind11 Qt integration on Linux)
        if(PYSIDE6_LIBRARY)
            target_link_libraries(_core PRIVATE ${PYSIDE6_LIBRARY})
        else()
            message(FATAL_ERROR "PYSIDE6_LIBRARY not set but LANEZERO_USE_QT=ON")
        endif()
        
        # Link shiboken6 library (required for pybind11 Qt integration on Linux)
        if(SHIBOKEN6_LIBRARY)
            target_link_libraries(_core PRIVATE ${SHIBOKEN6_LIBRARY})
        else()
            message(FATAL_ERROR "SHIBOKEN6_LIBRARY not set but LANEZERO_USE_QT=ON")
        endif()
        
        # Enable full PySide6 integration with Qt object exposure
        target_compile_definitions(_core PRIVATE LANEZERO_PYSIDE6_FULL)
    endif()
endif()

# Install the Python module
install(TARGETS _core LIBRARY DESTINATION LaneZero)

# Configure and install LaneZeroView launcher script
if(LANEZERO_USE_QT)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/LaneZeroView.py
        ${CMAKE_CURRENT_BINARY_DIR}/bin/LaneZeroView
        COPYONLY
    )
    
    # Make the script executable
    file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/bin/LaneZeroView
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                   GROUP_READ GROUP_EXECUTE
                   WORLD_READ WORLD_EXECUTE
    )
endif()

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
