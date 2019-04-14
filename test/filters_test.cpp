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

using namespace Eigen;
using namespace ser94mor::sensor_fusion;


TEST_CASE("KalmanFilter::Predict", "[filters]")
{
  CV::ProcessModel cv_pm;
  IndividualNoiseProcessesCovarianceMatrix mtx;
  mtx << 3.0, 7.0,
         7.0, 5.0;
  cv_pm.SetIndividualNoiseProcessCovarianceMatrix(mtx);

  Matrix4d R;
  R << 12.0, 28.0, 12.0, 28.0,
       28.0, 20.0, 28.0, 20.0,
       12.0, 28.0, 12.0, 28.0,
       28.0, 20.0, 28.0, 20.0;

  using KF = KalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel::StateVector_type>>;
  using BEL = Belief<CV::StateVector, CV::StateCovarianceMatrix>;

  Vector4d mu;
  mu << 1., 2., 3., 4.;

  Matrix4d Sigma;
  Sigma << 1., 2., 3., 4.,
           2., 5., 6., 7.,
           3., 6., 8., 9.,
           4., 7., 9., 10;
  BEL belief
  {
    .state_vector = mu,
    .state_covariance_matrix = Sigma,
  };

  std::time_t dt{2};

  Vector4d mu_prior;
  mu_prior << 7., 10., 3., 4.;

  Matrix4d Sigma_prior;
  Sigma_prior << 57., 86., 31., 50.,
                 86., 93., 52., 47.,
                 31., 52., 20., 37.,
                 50., 47., 37., 30.;
  BEL belief_prior_expected
  {
    .state_vector = mu_prior,
    .state_covariance_matrix = Sigma_prior,
  };

  BEL belief_prior{KF::Predict(belief, dt, cv_pm)};

  REQUIRE(belief_prior == belief_prior_expected);
}


TEST_CASE("KalmanFilter::Update", "[filters]")
{

}
