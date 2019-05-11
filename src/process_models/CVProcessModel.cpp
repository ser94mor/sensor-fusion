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

namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace CV
    {

      ProcessModel::ProcessModel() : state_transition_matrix_prototype_{StateTransitionMatrix::Identity()}
      {

      }

      StateTransitionMatrix ProcessModel::A(double_t dt) const
      {
        StateTransitionMatrix state_transition_matrix{state_transition_matrix_prototype_};
        state_transition_matrix(0, 2) = dt;
        state_transition_matrix(1, 3) = dt;
        return state_transition_matrix;
      }

      ProcessCovarianceMatrix ProcessModel::R(double_t dt) const
      {
        Eigen::Matrix<double_t, ProcessModel::StateDims(), 2> Gt;
        double_t dt_2_2 = dt * dt / 2.;
        Gt << dt_2_2,    0.0,
                 0.0, dt_2_2,
                  dt,    0.0,
                 0.0,     dt;
        return Gt * process_noise_covariance_matrix_ * Gt.transpose();
      }

      ControlTransitionMatrix ProcessModel::B() const
      {
        return ControlTransitionMatrix::Zero();
      }

      StateVector ProcessModel::Subtract(const StateVector& state_vector_1, const StateVector& state_vector_2)
      {
        return state_vector_1 - state_vector_2;
      }

      StateVector ProcessModel::Add(const StateVector& state_vector_1, const StateVector& state_vector_2)
      {
        return state_vector_1 + state_vector_2;
      }

    }
  }
}