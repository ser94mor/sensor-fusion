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


TEST_CASE("Lidar::Sensor::Type", "[sensors]")
{
  REQUIRE(Lidar::Sensor::Type() == EntityType::Sensor);
}


TEST_CASE("Lidar::Sensor::TypeName", "[sensors]")
{
  REQUIRE(std::string(Lidar::Sensor::TypeName()) == "SENSOR");
}


TEST_CASE("Lidar::Sensor::Kind", "[sensors]")
{
  REQUIRE(Lidar::Sensor::Kind() == SensorKind::Lidar);
}


TEST_CASE("Lidar::Sensor::KindName", "[sensors]")
{
  REQUIRE(std::string(Lidar::Sensor::KindName()) == "LIDAR");
}


TEST_CASE("Radar::Sensor::Type", "[sensors]")
{
  REQUIRE(Radar::Sensor::Type() == EntityType::Sensor);
}


TEST_CASE("Radar::Sensor::TypeName", "[sensors]")
{
  REQUIRE(std::string(Radar::Sensor::TypeName()) == "SENSOR");
}


TEST_CASE("Radar::Sensor::Kind", "[sensors]")
{
  REQUIRE(Radar::Sensor::Kind() == SensorKind::Radar);
}


TEST_CASE("Radar::Sensor::KindName", "[sensors]")
{
  REQUIRE(std::string(Radar::Sensor::KindName()) == "RADAR");
}
