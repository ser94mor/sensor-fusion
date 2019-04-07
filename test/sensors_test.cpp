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

#include "Eigen/Dense"
#include "sensors.hpp"

#include <catch.hpp>

using namespace Eigen;


TEST_CASE("Radar::FromJson", "[sensors]")
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

  auto radar{Radar::FromJson(radar_json)};

  REQUIRE(radar.MeasurementCovarianceMatrix().isApprox(radar_mtx));
}


TEST_CASE("Radar::Dims", "[sensors]")
{
  REQUIRE(Radar::Dims() == 3);
}


TEST_CASE("Radar::Type", "[sensors]")
{
  REQUIRE(std::string(Radar::Type()) == "SENSOR");
}


TEST_CASE("Radar::Name", "[sensors]")
{
  REQUIRE(std::string(Radar::Name()) == "RADAR");
}


TEST_CASE("Radar::FromParams", "[sensors]")
{
  Matrix<double, Radar::Dims(), Radar::Dims()> radar_mtx;
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;

  auto radar = Radar::FromParams(radar_mtx);

  REQUIRE(radar.MeasurementCovarianceMatrix().isApprox(radar_mtx));
}


TEST_CASE("Radar::Data", "[sensor]")
{
  std::time_t t{333};
  Vector3d z;
  z << 1, 2, 3;

  Matrix<double, Radar::Dims(), Radar::Dims()> radar_mtx;
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;

  auto radar{Radar::FromParams(radar_mtx)};

  auto radar_data{radar.Data(t, z)};

  REQUIRE(radar_data.t() == t);
  REQUIRE(radar_data.z().isApprox(z));
  REQUIRE(radar_data.Q().isApprox(radar_mtx));
}


TEST_CASE("Lidar::FromJson", "[sensors]")
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

  auto lidar{Lidar::FromJson(lidar_json)};

  REQUIRE(lidar.MeasurementCovarianceMatrix().isApprox(lidar_mtx));
}


TEST_CASE("Lidar::Dims", "[sensors]")
{
  REQUIRE(Lidar::Dims() == 2);
}


TEST_CASE("Lidar::Type", "[sensors]")
{
  REQUIRE(std::string(Lidar::Type()) == "SENSOR");
}


TEST_CASE("Lidar::Name", "[sensors]")
{
  REQUIRE(std::string(Lidar::Name()) == "LIDAR");
}


TEST_CASE("Lidar::FromParams", "[sensors]")
{
  Matrix<double, Lidar::Dims(), Lidar::Dims()> lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  auto lidar = Lidar::FromParams(lidar_mtx);

  REQUIRE(lidar.MeasurementCovarianceMatrix().isApprox(lidar_mtx));
}


TEST_CASE("Lidar::Data", "[sensor]")
{
  std::time_t t{333};
  Vector2d z;
  z << 1, 2;

  Matrix<double, Lidar::Dims(), Lidar::Dims()> lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  auto lidar{Lidar::FromParams(lidar_mtx)};

  auto lidar_data{lidar.Data(t, z)};

  REQUIRE(lidar_data.t() == t);
  REQUIRE(lidar_data.z().isApprox(z));
  REQUIRE(lidar_data.Q().isApprox(lidar_mtx));
}
