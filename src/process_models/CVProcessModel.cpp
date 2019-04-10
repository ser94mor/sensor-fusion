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

#include "CVProcessModel.hpp"

#include <iostream>

namespace ser94mor::sensor_fusion
{

  const char kCVProcessModelName[]{"CV"};

  CVProcessModel::CVProcessModel(double cov_ax_ax, double cov_ay_ay, double cov_ax_ay)
  {
    state_transition_matrix_prototype_.setIdentity();
    individual_noise_processes_covariance_matrix_ << cov_ax_ax, cov_ax_ay,
                                                     cov_ax_ay, cov_ay_ay;
  }

  CVStateTransitionMatrix CVProcessModel::A(std::time_t dt) const
  {
    CVStateTransitionMatrix state_transition_matrix{state_transition_matrix_prototype_};
    state_transition_matrix(0, 2) = dt;
    state_transition_matrix(1, 3) = dt;
    return state_transition_matrix;
  }

  CVProcessCovarianceMatrix CVProcessModel::R(std::time_t dt) const
  {
    Eigen::Matrix<double, CVProcessModel::StateDims(), 2> Gt;
    double dt_2_2 = dt * dt / 2.0;
    Gt << dt_2_2,    0.0,
             0.0, dt_2_2,
              dt,    0.0,
             0.0,     dt;
    return Gt * individual_noise_processes_covariance_matrix_ * Gt.transpose();
  }

  CVControlTransitionMatrix CVProcessModel::B(std::time_t dt) const
  {
    return CVControlTransitionMatrix().setZero();
  }

}