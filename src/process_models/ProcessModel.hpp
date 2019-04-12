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

#ifndef SENSOR_FUSION_PROCESSMODEL_HPP
#define SENSOR_FUSION_PROCESSMODEL_HPP


#include "definitions.hpp"
#include "primitives.hpp"
#include "measurement_models.hpp"

#include <ctime>

namespace ser94mor::sensor_fusion
{

  template <int state_dims, int control_dims, const char* name,
            class StateVector, class StateCovarianceMatrix, class ControlVector>
  class ProcessModel
  {
  public:

    using Belief_type = Belief<StateVector, StateCovarianceMatrix>;

    constexpr static const char* Type()
    {
      return kProcessModelType;
    }

    constexpr static const char* Name()
    {
      return name;
    }

    constexpr static int StateDims()
    {
      return state_dims;
    }

    constexpr static int ControlDims()
    {
      return control_dims;
    }

    template <typename Derived>
    void SetIndividualNoiseProcessCovarianceMatrix(const Eigen::MatrixBase<Derived>& mtx);

  protected:
    Eigen::Matrix<double, 2, 2> individual_noise_processes_covariance_matrix_;
  };

  template<int state_dims, int control_dims, const char* name, class StateVector, class StateCovarianceMatrix, class ControlVector>
  template<typename Derived>
  void
  ProcessModel<state_dims, control_dims, name,
               StateVector, StateCovarianceMatrix, ControlVector>::SetIndividualNoiseProcessCovarianceMatrix(
      const Eigen::MatrixBase<Derived>& mtx)
  {
    individual_noise_processes_covariance_matrix_ = mtx;
  }

}


#endif //SENSOR_FUSION_PROCESSMODEL_HPP
