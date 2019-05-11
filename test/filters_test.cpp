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


#include "filters.hpp"
#include "definitions.hpp"
#include "process_models.hpp"

#include <iostream>
#include <cmath>
#include <catch.hpp>


using namespace Eigen;
using namespace ser94mor::sensor_fusion;


///////////////////
// KALMAN FILTER //
///////////////////

TEST_CASE("KalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel>>::Predict", "[filters]")
{
  CV::ProcessModel cv_pm;
  CV::ProcessNoiseCovarianceMatrix mtx;
  mtx << 3.0, 7.0,
         7.0, 5.0;
  cv_pm.SetProcessNoiseCovarianceMatrix(mtx);

  using KF = KalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel>>;
  using BEL = Belief<CV::StateVector, CV::StateCovarianceMatrix>;

  CV::ControlVector control_vector{CV::ControlVector::Zero()};

  Vector4d mu;
  mu << 1., 2., 3., 4.;

  Matrix4d Sigma;
  Sigma << 1., 2., 3., 4.,
           2., 5., 6., 7.,
           3., 6., 8., 9.,
           4., 7., 9., 10;
  BEL belief{0, mu, Sigma};

  double_t dt{2.};

  Vector4d mu_prior;
  mu_prior << 7., 10., 3., 4.;

  Matrix4d Sigma_prior;
  Sigma_prior << 57., 86., 31., 50.,
                 86., 93., 52., 47.,
                 31., 52., 20., 37.,
                 50., 47., 37., 30.;
  BEL belief_prior_expected{2, mu_prior, Sigma_prior};

  BEL belief_prior{KF::Predict(belief, control_vector, dt, cv_pm)};

  REQUIRE(belief_prior == belief_prior_expected);
}


TEST_CASE("KalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel>>::Update", "[filters]")
{
  using KF = KalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel>>;
  using BEL = Belief<CV::StateVector, CV::StateCovarianceMatrix>;

  Vector4d mu_prior;
  mu_prior << 7., 10., 3., 4.;
  Matrix4d Sigma_prior;
  Sigma_prior << 57., 86., 31., 50.,
                 86., 93., 52., 47.,
                 31., 52., 20., 37.,
                 50., 47., 37., 30.;
  BEL belief_prior{2, mu_prior, Sigma_prior};

  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 5., 4.,
               4., 3.;
  Lidar::MeasurementModel<CV::ProcessModel> lidar_mm;
  lidar_mm.SetMeasurementCovarianceMatrix(lidar_mtx);

  Lidar::MeasurementVector z;
  z << 11, 8;
  Lidar::Measurement measurement{2, z};

  CV::StateVector state_vector_expected;
  state_vector_expected << (6128./537.), (1499./179.), (3532./537.), (785./537.);

  CV::StateCovarianceMatrix state_covariance_matrix_expected;
  state_covariance_matrix_expected << (2633./537.),  (1411./358.), (1696./537.),  (1747./1074.),
                                      (1411./358.),  (1063./358.), (919./358.),   (413./358.),
                                      (1696./537.),  (919./358.),  (3176./537.),  (7337./1074.),
                                      (1747./1074.), (413./358.),  (7337./1074.), (9199./1074.);

  BEL belief_posterior_expected{2, state_vector_expected, state_covariance_matrix_expected};


  BEL belief_posterior{KF::Update(belief_prior, measurement, lidar_mm)};

  REQUIRE(belief_posterior == belief_posterior_expected);
}


////////////////////////////
// EXTENDED KALMAN FILTER //
////////////////////////////

TEST_CASE("ExtendedKalmanFilter<CV::ProcessModel, Radar::MeasurementModel<CV::ProcessModel>>::Predict", "[filters]")
{
  CV::ProcessModel cv_pm;
  CV::ProcessNoiseCovarianceMatrix mtx;
  mtx << 3.0, 7.0,
         7.0, 5.0;
  cv_pm.SetProcessNoiseCovarianceMatrix(mtx);

  using EKF = ExtendedKalmanFilter<CV::ProcessModel, Radar::MeasurementModel<CV::ProcessModel>>;
  using BEL = Belief<CV::StateVector, CV::StateCovarianceMatrix>;

  CV::ControlVector control_vector{CV::ControlVector::Zero()};

  Vector4d mu;
  mu << 1., 2., 3., 4.;

  Matrix4d Sigma;
  Sigma << 1., 2., 3., 4.,
           2., 5., 6., 7.,
           3., 6., 8., 9.,
           4., 7., 9., 10;
  BEL belief{0, mu, Sigma};

  double_t dt{2.};

  Vector4d mu_prior;
  mu_prior << 7., 10., 3., 4.;

  Matrix4d Sigma_prior;
  Sigma_prior << 57., 86., 31., 50.,
                 86., 93., 52., 47.,
                 31., 52., 20., 37.,
                 50., 47., 37., 30.;
  BEL belief_prior_expected{2, mu_prior, Sigma_prior};

  BEL belief_prior{EKF::Predict(belief, control_vector, dt, cv_pm)};

  REQUIRE(belief_prior == belief_prior_expected);
}


TEST_CASE("ExtendedKalmanFilter<CTRV::ProcessModel, Radar::MeasurementModel<CTRV::ProcessModel>>::Predict", "[filters]")
{
  // TODO: write this unit test
}


TEST_CASE("ExtendedKalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel>>::Update", "[filters]")
{
  using EKF = ExtendedKalmanFilter<CV::ProcessModel, Lidar::MeasurementModel<CV::ProcessModel>>;
  using BEL = Belief<CV::StateVector, CV::StateCovarianceMatrix>;

  Vector4d mu_prior;
  mu_prior << 7., 10., 3., 4.;
  Matrix4d Sigma_prior;
  Sigma_prior << 57., 86., 31., 50.,
                 86., 93., 52., 47.,
                 31., 52., 20., 37.,
                 50., 47., 37., 30.;
  BEL belief_prior{2, mu_prior, Sigma_prior};

  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 5., 4.,
               4., 3.;
  Lidar::MeasurementModel<CV::ProcessModel> lidar_mm;
  lidar_mm.SetMeasurementCovarianceMatrix(lidar_mtx);

  Lidar::MeasurementVector z;
  z << 11, 8;
  Lidar::Measurement measurement{2, z};

  CV::StateVector state_vector_expected;
  state_vector_expected << (6128./537.), (1499./179.), (3532./537.), (785./537.);

  CV::StateCovarianceMatrix state_covariance_matrix_expected;
  state_covariance_matrix_expected << (2633./537.),  (1411./358.), (1696./537.),  (1747./1074.),
                                      (1411./358.),  (1063./358.), (919./358.),   (413./358.),
                                      (1696./537.),  (919./358.),  (3176./537.),  (7337./1074.),
                                      (1747./1074.), (413./358.),  (7337./1074.), (9199./1074.);

  BEL belief_posterior_expected{2, state_vector_expected, state_covariance_matrix_expected};

  BEL belief_posterior{EKF::Update(belief_prior, measurement, lidar_mm)};

  REQUIRE(belief_posterior == belief_posterior_expected);
}


TEST_CASE("ExtendedKalmanFilter<CV::ProcessModel, Radar::MeasurementModel<CV::ProcessModel>>::Update", "[filters]")
{
  using EKF = ExtendedKalmanFilter<CV::ProcessModel, Radar::MeasurementModel<CV::ProcessModel>>;
  using BEL = Belief<CV::StateVector, CV::StateCovarianceMatrix>;

  Vector4d mu_prior;
  mu_prior << 7., 10., 3., 4.;
  Matrix4d Sigma_prior;
  Sigma_prior << 57., 86., 31., 50.,
                 86., 93., 52., 47.,
                 31., 52., 20., 37.,
                 50., 47., 37., 30.;
  BEL belief_prior{2, mu_prior, Sigma_prior};

  Radar::MeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 6., 5., 4.,
               4., 3., 2.,
               1., 3., 5.;
  Radar::MeasurementModel<CV::ProcessModel> radar_mm;
  radar_mm.SetMeasurementCovarianceMatrix(radar_mtx);

  Radar::MeasurementVector z;
  z << 11, M_PI_4,  8;
  Radar::Measurement measurement{2, z};

  CV::StateVector state_vector_expected;
  state_vector_expected << 6.5601563736393267, 8.8442906470847316, 4.421325574037267, 5.5173910728606534;

  CV::StateCovarianceMatrix state_covariance_matrix_expected;
  state_covariance_matrix_expected <<
     -7.7423273663513541,  6.0580378541632163, -7.0780179461910935,  6.4404125444715659,
      6.8607045995033218, -6.2275790258719192,  7.0179341272832367, -5.476430297456611,
     -8.1537415978815844,  5.1091222365733984, -6.4374919854575268,  7.1841315639005643,
      5.6807191159736803, -7.1102590161828161,  7.5671502297651809, -4.0525670391244084;

  BEL belief_posterior_expected{2, state_vector_expected, state_covariance_matrix_expected};

  BEL belief_posterior{EKF::Update(belief_prior, measurement, radar_mm)};
  
  REQUIRE(belief_posterior == belief_posterior_expected);
}


/////////////////////////////
// UNSCENTED KALMAN FILTER //
/////////////////////////////

TEST_CASE("UnscentedKalmanFilter<CTRV::ProcessModel, Radar::MeasurementModel>::Predict", "[filters]")
{
  using UKF = UnscentedKalmanFilter<CTRV::ProcessModel, Radar::MeasurementModel<CTRV::ProcessModel>>;
  using BEL = Belief<CTRV::StateVector, CTRV::StateCovarianceMatrix>;

  CTRV::ControlVector control_vector{CTRV::ControlVector::Zero()};

  CTRV::StateVector mu;
  mu << 1., 2., 3., 4., 5.;

  CTRV::StateCovarianceMatrix Sigma;
  Sigma << 1., 2., 3., 4., 5.,
      2., 5., 6., 7., 8.,
      3., 6., 8., 9., 10.,
      4., 7., 9., 10., 11.,
      12., 13., 14., 15., 16.;
  BEL belief{0, mu, Sigma};

  CTRV::ProcessModel pm;
  CTRV::ProcessNoiseCovarianceMatrix mtx;
  mtx << 3.0, 7.0,
      7.0, 5.0;
  pm.SetProcessNoiseCovarianceMatrix(mtx);
  
  Radar::MeasurementVector mv;
  mv << 1., M_PI_2, 2.;
  Radar::Measurement radar_meas{2, mv};

  Radar::MeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 6., 5., 4.,
      4., 3., 2.,
      1., 3., 5.;
  Radar::MeasurementModel<CTRV::ProcessModel> radar_mm;
  radar_mm.SetMeasurementCovarianceMatrix(radar_mtx);

  auto belief_prior{UKF::Predict(belief, control_vector, 2., pm)};

  // TODO: write this test
}

TEST_CASE("UnscentedKalmanFilter<CTRV::ProcessModel, Radar::MeasurementModel>::Update", "[filters]")
{
  // TODO: write this test
}
