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


#include "process_models.hpp"
#include "measurement_models.hpp"

#include <catch.hpp>
#include <iostream>


using namespace ser94mor::sensor_fusion;


TEST_CASE("Lidar::MeasurementModel::C", "[measurement_models]")
{
  Lidar::MeasurementModel<CV::ProcessModel> lidar_mm;

  Lidar::MeasurementModel<CV::ProcessModel>::MeasurementMatrix_type meas_mtx;
  meas_mtx << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  const auto& Ct = lidar_mm.C();
  REQUIRE(Ct.isApprox(meas_mtx));
}


TEST_CASE("Lidar::MeasurementModel::Q", "[measurement_models]")
{
  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  Lidar::MeasurementModel<CV::ProcessModel> lidar_mm;
  lidar_mm.SetMeasurementCovarianceMatrix(lidar_mtx);

  REQUIRE(lidar_mm.Q().isApprox(lidar_mtx));
}


TEST_CASE("Lidar::MeasurementModel::Type", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::Type() == EntityType::MeasurementModel);
}


TEST_CASE("Lidar::MeasurementModel::TypeName", "[measurement_models]")
{
  REQUIRE(std::string(Lidar::MeasurementModel<CV::ProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
}


TEST_CASE("Lidar::MeasurementModel::Kind", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::Kind() == MeasurementModelKind::Lidar);
}


TEST_CASE("Lidar::MeasurementModel::KindName", "[measurement_models]")
{
  REQUIRE(std::string(Lidar::MeasurementModel<CV::ProcessModel>::KindName()) == "LIDAR");
}


TEST_CASE("Lidar::MeasurementModel::IsLinear", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::IsLinear());
}
