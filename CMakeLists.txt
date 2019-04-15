cmake_minimum_required(VERSION 3.5)
project(sensor_fusion)

#add_definitions(-std=c++14)
#
#set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 14)
set(CXX_EXTENSIONS OFF)
set(CXX_STANDARD_REQUIRED ON)

include_directories(third_parties/eigen-3.3.7/)
include_directories(third_parties/nlohmann-3.6.1)
include_directories(third_parties/catch2-2.7.0)
include_directories(include)

file(GLOB sensors_SRC src/sensors/*)
file(GLOB process_models_SRC src/process_models/*)
file(GLOB measurement_models_SRC src/measurement_models/*)
file(GLOB filters_SRC src/filters/*)
file(GLOB primitives_SRC src/primitives/*)
file(GLOB fusion_SRC src/fusion/*)
file(GLOB measurements_SRC src/measurements/*)

add_executable(sensor_fusion
        ${sensors_SRC}
        ${states_SRC}
        ${process_models_SRC}
        ${measurement_models_SRC}
        ${filters_SRC}
        ${fusion_SRC}
        src/main.cpp
)

target_link_libraries(sensor_fusion z ssl uv uWS)

add_library(Catch INTERFACE)
add_executable( sensor_fusion_test
        test/sensors_test.cpp ${sensors_SRC}
        test/process_models_test.cpp ${process_models_SRC}
        test/measurement_models_test.cpp ${measurement_models_SRC}
        test/filters_test.cpp ${filters_SRC}
        test/fusion_test.cpp ${fusion_SRC}
        test/beliefs_test.cpp
        test/test.cpp)
target_link_libraries(sensor_fusion_test Catch)

enable_testing()
add_test( NAME sensor_fusion_test COMMAND sensor_fusion_test )
