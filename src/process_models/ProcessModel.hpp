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
#include "beliefs.hpp"
#include "measurement_models.hpp"

#include <ctime>

namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base template class representing a process model.
     * It contains methods and functions common for all concrete process models.
     *
     * @tparam StateVector a state vector class
     * @tparam StateCovarianceMatrix a state covariance matrix class
     * @tparam StateVectorView a state vector view (accessor to the state vector dimensions)
     * @tparam ControlVector a control vector class
     * @tparam pmk a process model kind
     * @tparam is_linear a flag indicating whether the process model is linear or not
     */
    template<class StateVector, class StateCovarianceMatrix, class ControlVector, class ProcessNoiseCovarianceMatrix,
             class ROStateVectorView, class RWStateVectorView, ProcessModelKind pmk, bool is_linear>
    class ProcessModel : public ModelEntity<EntityType::ProcessModel, ProcessModelKind, pmk, is_linear>
    {
    public:
      /**
       * The typedefs below are needed in other places in the code. These typedefs, in fact, are attributes of the
       * process model.
       */
      using Belief_type = Belief<StateVector, StateCovarianceMatrix>;
      using StateVector_type = StateVector;
      using StateCovarianceMatrix_type = StateCovarianceMatrix;
      using ControlVector_type = ControlVector;
      using ProcessNoiseCovarianceMatrix_type = ProcessNoiseCovarianceMatrix;
      using RWStateVectorView_type = RWStateVectorView;
      using ROStateVectorView_type = ROStateVectorView;

      /**
       * @return a number of dimensions in the state vector
       */
      constexpr static int StateDims()
      {
        return StateVector::SizeAtCompileTime;
      }

      /**
       * @return a number of dimensions in the control vector
       */
      constexpr static int ControlDims()
      {
        return ControlVector::SizeAtCompileTime;
      }

      /**
       * @return a number of dimensions in the process noise vector
       */
      constexpr static int ProcessNoiseDims()
      {
        return ProcessNoiseCovarianceMatrix::RowsAtCompileTime;
      }

      /**
       * Set a process noise covariance matrix. It is done explicitly by the user of the process model
       * due to the variadic templates used in this code. ProcessModel needs a default constructor.
       * @param mtx a process noise covariance matrix
       */
      void SetProcessNoiseCovarianceMatrix(const ProcessNoiseCovarianceMatrix& mtx)
      {
        process_noise_covariance_matrix_ = mtx;
      }

      /**
       * Get a process noise covariance matrix.
       * @return a process noise covariance matirx.
       */
      const ProcessNoiseCovarianceMatrix& GetProcessNoiseCovarianceMatrix() const
      {
        return process_noise_covariance_matrix_;
      }

    protected:
      ProcessNoiseCovarianceMatrix process_noise_covariance_matrix_;
    };

  }
}


#endif //SENSOR_FUSION_PROCESSMODEL_HPP
