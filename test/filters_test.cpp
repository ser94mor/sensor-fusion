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
  CV::ProcessModel cv_pm;
  IndividualNoiseProcessesCovarianceMatrix mtx;
  mtx << 9.0, 0.0,
         0.0, 9.0;
  cv_pm.SetIndividualNoiseProcessCovarianceMatrix(mtx);

  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  Lidar::MeasurementModel<CV::StateVector> l_mm;
  l_mm.SetMeasurementCovarianceMatrix(lidar_mtx);

  using KF = KalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel::StateVector_type>>;

  //ExtendedKalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::StateVector>> ekf;
  //ekf.SetProcessModel(&cv_pm);
  //ekf.SetMeasurementModels(&l_mm);


  CV::ProcessModel::Belief_type bel;
  Lidar::MeasurementVector v;
  Lidar::MeasurementModel<CV::StateVector>::Measurement_type meas{
    .timestamp = 1,
    .measurement_vector = v,
    .source_name = "LIDAR",
  };
  auto foo = KF::Predict(bel, 1, cv_pm);
  auto bar = KF::Update(bel, meas, 55, l_mm);

  std::cout << foo.Sigma() << std::endl;
}


TEST_CASE("KalmanFilter::Update", "[filters]")
{

}
