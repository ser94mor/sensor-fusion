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


///////////
// LIDAR //
///////////

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
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::IsLinear() == true);
}


TEST_CASE("Lidar::MeasurementModel::MeasurementDims", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::MeasurementDims() == 2);
}


TEST_CASE("Lidar::MeasurementModel::StateDims", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::StateDims() == 4);
}


///////////
// RADAR //
///////////

TEST_CASE("Radar::MeasurementModel::h", "[measurement_models]")
{
  Radar::MeasurementModel<CV::ProcessModel> radar_mm;

  CV::StateVector state_vector;
  state_vector << 3. , 4., 2., 1.;

  Radar::MeasurementVector measurement_vector;
  measurement_vector << 5., std::atan2(4., 3.) , 2.;

  REQUIRE(radar_mm.h(state_vector).isApprox(measurement_vector));
}


TEST_CASE("Radar::MeasurementModel::H", "[measurement_models]")
{
  Radar::MeasurementModel<CV::ProcessModel> radar_mm;

  CV::StateVector state_vector;
  state_vector << 3. , 4., 2., 1.;

  Radar::MeasurementModel<CV::ProcessModel>::MeasurementMatrix_type measurement_matrix;
  measurement_matrix <<   3./5.,   4./5.,    0.,    0.,
                        -4./25.,  3./25.,    0.,    0.,
                         4./25., -3./25., 3./5., 4./5.;


  REQUIRE(radar_mm.H(state_vector).isApprox(measurement_matrix));
}


TEST_CASE("Radar::MeasurementModel::Q", "[measurement_models]")
{
  Radar::MeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;;

  Radar::MeasurementModel<CV::ProcessModel> radar_mm;
  radar_mm.SetMeasurementCovarianceMatrix(radar_mtx);

  REQUIRE(radar_mm.Q().isApprox(radar_mtx));
}


TEST_CASE("Radar::MeasurementModel::Type", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::Type() == EntityType::MeasurementModel);
}


TEST_CASE("Radar::MeasurementModel::TypeName", "[measurement_models]")
{
  REQUIRE(std::string(Radar::MeasurementModel<CV::ProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
}


TEST_CASE("Radar::MeasurementModel::Kind", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::Kind() == MeasurementModelKind::Radar);
}


TEST_CASE("Radar::MeasurementModel::KindName", "[measurement_models]")
{
  REQUIRE(std::string(Radar::MeasurementModel<CV::ProcessModel>::KindName()) == "RADAR");
}


TEST_CASE("Radar::MeasurementModel::IsLinear", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::IsLinear() == false);
}


TEST_CASE("Radar::MeasurementModel::MeasurementDims", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::MeasurementDims() == 3);
}


TEST_CASE("Radar::MeasurementModel::StateDims", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::StateDims() == 4);
}
