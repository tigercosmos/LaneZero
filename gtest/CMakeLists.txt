include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(tests
    test_main.cpp
    ../src/map/Map.cpp
    ../src/vehicle/Vehicle.cpp
    ../src/simulation/Simulation.cpp
    ../src/scenario/Scenario.cpp
    ../src/planner/BehaviorPlanner.cpp
    ../src/planner/MotionPlanner.cpp
    ../src/planner/ControlPlanner.cpp
    ../src/planner/PlanningOrchestrator.cpp
)

target_include_directories(tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/map
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/vehicle
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/simulation
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/scenario
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/planner
    ${rapidjson_SOURCE_DIR}/include
)
target_link_libraries(tests PRIVATE GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(tests
    DISCOVERY_MODE PRE_TEST
)
