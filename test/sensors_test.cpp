/**
 * Copyright (C) 2018-2019  Sergey Morozov <sergey@morozov.ch>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "sensors.hpp"

#include <Eigen/Dense>
#include <catch.hpp>

using namespace Eigen;
using namespace ser94mor::sensor_fusion;


TEST_CASE("RadarSensor::FromJson", "[sensors]")
{
  const char* radar_json{"{\n"
                         "  \"type\": \"SENSOR\",\n"
                         "  \"name\": \"RADAR\",\n"
                         "  \"measurement_covariance_matrix\": [\n"
                         "    [0.09,    0.0,  0.0],\n"
                         "    [ 0.0, 0.0009,  0.0],\n"
                         "    [ 0.0,    0.0, 0.09]\n"
                         "  ]\n"
                         "}"};
  MatrixXd radar_mtx(3, 3);
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;

  auto radar{SensorFactory<RadarSensor>::FromJson(radar_json)};

  REQUIRE(radar.MeasurementCovarianceMatrix().isApprox(radar_mtx));
}


TEST_CASE("RadarSensor::Dims", "[sensors]")
{
  REQUIRE(RadarSensor::Dims() == 3);
}


TEST_CASE("RadarSensor::Type", "[sensors]")
{
  REQUIRE(std::string(RadarSensor::Type()) == "SENSOR");
}


TEST_CASE("RadarSensor::Name", "[sensors]")
{
  REQUIRE(std::string(RadarSensor::Name()) == "RADAR");
}


TEST_CASE("RadarSensor::FromParams", "[sensors]")
{
  Matrix<double, RadarSensor::Dims(), RadarSensor::Dims()> radar_mtx;
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;

  RadarSensor radar;
  radar.SetMeasurementCovarianceMatrix(radar_mtx);

  REQUIRE(radar.MeasurementCovarianceMatrix().isApprox(radar_mtx));
}


//TEST_CASE("RadarSensor::Data", "[sensor]")
//{
//  std::time_t t{333};
//  Vector3d z;
//  z << 1, 2, 3;
//
//  Matrix<double, RadarSensor::Dims(), RadarSensor::Dims()> radar_mtx;
//  radar_mtx << 0.09,    0.0,  0.0,
//                0.0, 0.0009,  0.0,
//                0.0,    0.0, 0.09;
//
//  RadarSensor radar;
//  radar.SetMeasurementCovarianceMatrix(radar_mtx);
//
//  auto radar_data{radar.Data(t, z)};
//
//  REQUIRE(radar_data.t == t);
//  REQUIRE(radar_data.z.isApprox(z));
//  REQUIRE(radar_data.Q.isApprox(radar_mtx));
//}


TEST_CASE("LidarSensor::FromJson", "[sensors]")
{
  const char* lidar_json{"{\n"
                         "  \"type\": \"SENSOR\",\n"
                         "  \"name\": \"LIDAR\",\n"
                         "  \"measurement_covariance_matrix\": [\n"
                         "    [0.0225,    0.0],\n"
                         "    [   0.0, 0.0225]\n"
                         "  ]\n"
                         "}"};
  MatrixXd lidar_mtx(2, 2);
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  auto lidar{LidarSensor::FromJson(lidar_json)};

  REQUIRE(lidar.MeasurementCovarianceMatrix().isApprox(lidar_mtx));
}


TEST_CASE("LidarSensor::Dims", "[sensors]")
{
  REQUIRE(LidarSensor::Dims() == 2);
}


TEST_CASE("LidarSensor::Type", "[sensors]")
{
  REQUIRE(std::string(LidarSensor::Type()) == "SENSOR");
}


TEST_CASE("LidarSensor::Name", "[sensors]")
{
  REQUIRE(std::string(LidarSensor::Name()) == "LIDAR");
}


TEST_CASE("LidarSensor::FromParams", "[sensors]")
{
  Matrix<double, LidarSensor::Dims(), LidarSensor::Dims()> lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  LidarSensor lidar;
  lidar.SetMeasurementCovarianceMatrix(lidar_mtx);

  REQUIRE(lidar.MeasurementCovarianceMatrix().isApprox(lidar_mtx));
}


//TEST_CASE("LidarSensor::Data", "[sensor]")
//{
//  std::time_t t{333};
//  Vector2d z;
//  z << 1, 2;
//
//  Matrix<double, LidarSensor::Dims(), LidarSensor::Dims()> lidar_mtx;
//  lidar_mtx << 0.0225,    0.0,
//                  0.0, 0.0225;
//
//  LidarSensor lidar;
//  lidar.SetMeasurementCovarianceMatrix(lidar_mtx);
//
//  auto lidar_data{lidar.Data(t, z)};
//
//  REQUIRE(lidar_data.t == t);
//  REQUIRE(lidar_data.z.isApprox(z));
//  REQUIRE(lidar_data.Q.isApprox(lidar_mtx));
//}
