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

#ifndef SENSOR_FUSION_CTRVPROCESSMODEL_HPP
#define SENSOR_FUSION_CTRVPROCESSMODEL_HPP


#include "definitions.hpp"
#include "state_vector_views.hpp"
#include "ProcessModel.hpp"

#include <Eigen/Dense>


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace CTRV
    {

      /**
       * A concrete process model class for CTRV process model. The State vector for process model consists of
       *   [ px, py, v, yaw, yaw_rate ].
       * The naming of matrices and functions are taken from the
       * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
       */
      class ProcessModel
      : public ser94mor::sensor_fusion::ProcessModel<StateVector, StateCovarianceMatrix, ControlVector,
                                                     ProcessNoiseCovarianceMatrix,
                                                     ROStateVectorView, RWStateVectorView,
                                                     ProcessModelKind::CTRV, kIsLinear>
      {
      public:
        /**
         * Constructor.
         */
        ProcessModel();

        /**
         * A state transition function without the noise vector.
         *
         * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
         * @param control_vector a control vector
         * @param state_vector a state vector
         * @return a prior state vector, that is, a state vector after the state transition function applied.
         */
        StateVector g(double_t dt, const ControlVector& control_vector, const StateVector& state_vector) const;

        /**
         * A state transition function with the noise vector. Some filters may require noise terms to be incorporated
         * into this non-linear state transition function.
         *
         * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
         * @param control_vector a control vector
         * @param state_vector a state vector
         * @param noise_vector a noise vector
         * @return a prior state vector, that is, a state vector after the state transition function applied.
         */
        StateVector g(double_t dt, const ControlVector& control_vector,
                      const StateVector& state_vector, const ProcessNoiseVector& noise_vector) const;

        /**
         * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
         * @param state_vector a state vector
         * @return a state transition matrix
         */
        StateTransitionMatrix G(double_t dt, const StateVector& state_vector) const;

        /**
         * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
         * @return a process covariance matrix
         */
        ProcessCovarianceMatrix R(double_t dt, const StateVector& state_vector) const;

        StateVector Diff(const StateVector& state_vector_1, const StateVector& state_vector_2) const override;

      private:
        StateTransitionMatrix state_transition_matrix_prototype_;

      };

    }
  }
}


#endif //SENSOR_FUSION_CTRVPROCESSMODEL_HPP
