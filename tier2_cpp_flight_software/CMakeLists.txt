cmake_minimum_required(VERSION 3.20)
project(Tier2FlightSoftware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

include_directories(include)

add_library(gnc_flight_core STATIC
    src/observable_ekf.cpp
    src/attitude_estimator.cpp
    src/mrac_controller.cpp
    src/control_allocator.cpp
    src/waypoint_guidance.cpp
)

target_include_directories(gnc_flight_core PUBLIC include)

add_executable(test_golden_trace sitl_tests/test_golden_trace.cpp)
target_link_libraries(test_golden_trace PRIVATE gnc_flight_core)

add_executable(test_sitl_closed_loop sitl_tests/test_sitl_closed_loop.cpp)
target_link_libraries(test_sitl_closed_loop PRIVATE gnc_flight_core)

enable_testing()
add_test(NAME GoldenTraceTest COMMAND test_golden_trace)
add_test(NAME SITLClosedLoopTest COMMAND test_sitl_closed_loop)