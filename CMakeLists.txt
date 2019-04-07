cmake_minimum_required(VERSION 3.5)
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 17)

include_directories(third_parties/eigen-3.3.7/)
include_directories(third_parties/nlohmann-3.6.1)
include_directories(third_parties/catch2-2.7.0)
include_directories(include)

add_executable(sensor_fusion
        src/filters/ExtendedKalmanFilter.cpp
        src/filters/ExtendedKalmanFilter.h
        src/filters/BayesFilter.hpp
        src/filters/KalmanFilter.cpp
        src/filters/KalmanFilter.hpp
        src/filters/UnscentedKalmanFilter.cpp
        src/filters/UnscentedKalmanFilter.h
        src/models/ConstantTurnRateAndVelocityModel.cpp
        src/models/ConstantTurnRateAndVelocityModel.h
        src/models/ConstantVelocityModel.cpp
        src/models/ConstantVelocityModel.h
        src/models/MotionModel.cpp
        src/models/MotionModel.h
        src/sensors/SensorData.hpp
        src/main.cpp
        src/SensorFusion.cpp
        src/SensorFusion.hpp src/models/State.hpp src/models/CTRVState.hpp src/models/CVState.hpp src/sensors/Radar.hpp src/sensors/Sensor.hpp src/sensors/SensorFactory.hpp src/sensors/Radar.cpp src/sensors/Lidar.cpp src/sensors/Lidar.hpp test/sensors_test.cpp test/test.cpp include/sensors.hpp)

add_library(Catch INTERFACE)
add_executable( sensor_fusion_test
        test/sensors_test.cpp src/sensors/Radar.cpp src/sensors/Radar.hpp src/sensors/Sensor.hpp src/sensors/SensorFactory.hpp src/sensors/Lidar.hpp src/sensors/Lidar.cpp
        test/test.cpp
)
target_link_libraries(sensor_fusion_test Catch)

enable_testing()
add_test( NAME sensor_fusion_test COMMAND sensor_fusion_test )
