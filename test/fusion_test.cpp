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
#include "sensors.hpp"
#include "filters.hpp"
#include "fusion.hpp"

#include <catch.hpp>

using namespace ser94mor::sensor_fusion;
using namespace Eigen;


TEST_CASE("Fusion::ProcessMeasurement", "[fusion]")
{
  using BEL = Belief<CV::StateVector, CV::StateCovarianceMatrix>;

  CV::ProcessNoiseCovarianceMatrix p_mtx;
  p_mtx << 3.0, 7.0,
           7.0, 5.0;

  Lidar::MeasurementCovarianceMatrix m_mtx;
  m_mtx << 5., 4.,
           4., 3.;

  Vector4d mu;
  mu << 1., 2., 3., 4.;

  Matrix4d Sigma;
  Sigma << 1., 2., 3., 4.,
           2., 5., 6., 7.,
           3., 6., 8., 9.,
           4., 7., 9., 10;
  BEL belief_initial{0, mu, Sigma};

  CV::StateVector state_vector_expected;
  state_vector_expected << (6128./537.), (1499./179.), (3532./537.), (785./537.);

  CV::StateCovarianceMatrix state_covariance_matrix_expected;
  state_covariance_matrix_expected <<
      (2633./537.),  (1411./358.), (1696./537.),  (1747./1074.),
      (1411./358.),  (1063./358.), (919./358.),   (413./358.),
      (1696./537.),  (919./358.),  (3176./537.),  (7337./1074.),
      (1747./1074.), (413./358.),  (7337./1074.), (9199./1074.);

  BEL belief_posterior_expected{2, state_vector_expected, state_covariance_matrix_expected};

  Fusion<KalmanFilter, CV::ProcessModel, Lidar::MeasurementModel>
  fusion{p_mtx, m_mtx};

  Lidar::MeasurementVector meas_vect;
  meas_vect << 11., 8.;
  Lidar::Measurement measurement{2, meas_vect};

  fusion.ProcessMeasurement(measurement); // increments processed counter and initializes initial belief
  fusion.SetBelief(belief_initial); // override initial belief for testing purposes

  auto belief{fusion.ProcessMeasurement(measurement)};

  REQUIRE(belief == belief_posterior_expected);
}
