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


//#include "primitives.hpp"
#include "filters.hpp"
#include "definitions.hpp"
#include "process_models.hpp"

#include <catch.hpp>
#include <iostream>

using namespace ser94mor::sensor_fusion;


TEST_CASE("KalmanFilter::Predict", "[filters]")
{
  CVProcessModel cv_pm{9.0, 9.0, 0.0};

  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  Lidar::MeasurementModel<CVStateVector> l_mm;
  l_mm.SetMeasurementCovarianceMatrix(lidar_mtx);

  KalmanFilter<CVProcessModel, Lidar::MeasurementModel<CVStateVector>> kf{cv_pm, l_mm};

  CVProcessModel::Belief_type bel;
  Lidar::MeasurementVector v;
  Lidar::MeasurementModel<CVStateVector>::Measurement_type meas{
    .timestamp = 1,
    .measurement_vector = v,
  };
  auto foo = kf.Predict(bel, 1);

  std::cout << foo.Sigma() << std::endl;
}


TEST_CASE("KalmanFilter::Update", "[filters]")
{

}
