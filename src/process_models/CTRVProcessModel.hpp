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
#include "ProcessModel.hpp"
#include "CTRVStateVectorView.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace CTRV
    {

      class ProcessModel
      : public ser94mor::sensor_fusion::ProcessModel<StateVector, StateCovarianceMatrix, ControlVector,
                                                     ConstStateVectorView, StateVectorView,
                                                     ProcessModelKind::CTRV, kIsLinear>
      {
      public:
        /**
         * Constructor.
         */
        ProcessModel();

        StateVector g(double dt, const ControlVector& control_vector, const StateVector& state_vector) const;

        StateTransitionMatrix G(double dt, const StateVector& state_vector) const;

        /**
         * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
         * @return a process covariance matrix
         */
        ProcessCovarianceMatrix R(double dt, const StateVector& state_vector) const;

      private:
        StateTransitionMatrix state_transition_matrix_prototype_;

      };

    }
  }
}


#endif //SENSOR_FUSION_CTRVPROCESSMODEL_HPP
