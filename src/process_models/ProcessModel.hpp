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

  template <class StateVector, class StateCovarianceMatrix, class ControlVector, const char* name>
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
      return StateVector::SizeAtCompileTime;
    }

    constexpr static int ControlDims()
    {
      return ControlVector::SizeAtCompileTime;
    }

    template <typename Derived>
    void SetIndividualNoiseProcessCovarianceMatrix(const Eigen::MatrixBase<Derived>& mtx);

  protected:
    IndividualNoiseProcessesCovarianceMatrix individual_noise_processes_covariance_matrix_;
  };

  template<class StateVector, class StateCovarianceMatrix, class ControlVector, const char* name>
  template<typename Derived>
  void ProcessModel<StateVector, StateCovarianceMatrix, ControlVector, name>::SetIndividualNoiseProcessCovarianceMatrix(
      const Eigen::MatrixBase<Derived>& mtx)
  {
    individual_noise_processes_covariance_matrix_ = mtx;
  }

}


#endif //SENSOR_FUSION_PROCESSMODEL_HPP
