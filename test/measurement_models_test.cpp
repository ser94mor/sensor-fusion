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


#include "measurement_models.hpp"

#include <catch.hpp>
#include <iostream>


using namespace ser94mor::sensor_fusion;


TEST_CASE("LidarMeasurementModel::C", "[measurement_model]")
{
  Eigen::Matrix<double, LidarSensor::Dims(), LidarSensor::Dims()> lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  LidarSensor lidar;
  lidar.SetMeasurementCovarianceMatrix(lidar_mtx);
  LidarMeasurementModel<CVStateVector> lidar_mm{lidar};

  Eigen::Matrix<double, LidarMeasurementModel<CVStateVector>::MeasurementDims(),
                        LidarMeasurementModel<CVStateVector>::StateDims()>
  meas_mtx;
  meas_mtx << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  auto Ct = lidar_mm.C(555);
  REQUIRE(Ct.isApprox(meas_mtx));
}


TEST_CASE("LidarMeasurementModel::Q", "[measurement_model]")
{
  Eigen::Matrix<double, LidarSensor::Dims(), LidarSensor::Dims()> lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  LidarSensor lidar;
  lidar.SetMeasurementCovarianceMatrix(lidar_mtx);
  LidarMeasurementModel<CVStateVector> lidar_mm{lidar};

  REQUIRE(lidar_mm.Q(222).isApprox(lidar_mtx));
}
