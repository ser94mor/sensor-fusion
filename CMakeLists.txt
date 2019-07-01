cmake_minimum_required(VERSION 3.14)
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 14)
set(CXX_EXTENSIONS OFF)
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -pedantic -Wextra")

include_directories(third_parties/eigen-3.3.7/)
include_directories(third_parties/nlohmann-3.6.1)
include_directories(third_parties/catch2-2.7.0)
include_directories(include)

file(GLOB process_models_SRC src/process_models/*)
file(GLOB measurement_models_SRC src/measurement_models/*)
file(GLOB filters_SRC src/filters/*)
file(GLOB primitives_SRC src/primitives/*)
file(GLOB fusion_SRC src/fusion/*)
file(GLOB measurements_SRC src/measurements/*)
file(GLOB utils_SRC src/utils/*)
file(GLOB state_vector_views_SRC src/state_vector_views/*)
file(GLOB measurement_vector_views_SRC src/measurement_vector_views/*)
file(GLOB beliefs_SRC src/beliefs/*)

set(SRC_EXCEPT_MAIN
        ${states_SRC}
        ${process_models_SRC}
        ${measurement_models_SRC}
        ${filters_SRC}
        ${fusion_SRC}
        ${utils_SRC}
        ${beliefs_SRC}
        ${state_vector_views_SRC}
        ${measurement_vector_views_SRC})

add_executable(sensor_fusion_udacity_sim
        ${SRC_EXCEPT_MAIN}
        src/mains/udacity_sim_main.cpp)
target_link_libraries(sensor_fusion_udacity_sim z ssl uv uWS)

add_executable(sensor_fusion_all
        ${SRC_EXCEPT_MAIN}
        src/mains/all_main.cpp)

add_library(Catch INTERFACE)
add_executable( sensor_fusion_test
        test/process_models_test.cpp ${process_models_SRC}
        test/measurement_models_test.cpp ${measurement_models_SRC}
        test/filters_test.cpp ${filters_SRC}
        test/fusion_test.cpp ${fusion_SRC}
        test/beliefs_test.cpp ${beliefs_SRC}
        test/state_vector_views_test.cpp ${state_vector_views_SRC}
        test/measurement_vector_views_test.cpp ${measurement_vector_views_SRC}
        test/utils_test.cpp ${utils_SRC}
        test/test.cpp)
target_link_libraries(sensor_fusion_test Catch)

enable_testing()
add_test( NAME sensor_fusion_test COMMAND sensor_fusion_test )
