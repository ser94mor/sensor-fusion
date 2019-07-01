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
    using CTRVProcessModelBase = ProcessModel<CTRVStateVector, CTRVStateCovarianceMatrix, CTRVControlVector,
                                              CTRVProcessNoiseCovarianceMatrix, CTRVROStateVectorView,
                                              CTRVRWStateVectorView, PMKind::e_CTRV, kCTRVIsLinear>;

    /**
     * A concrete process model class for CTRV process model. The State vector for process model consists of
     *   [ px, py, v, yaw, yaw_rate ].
     * The naming of matrices and functions are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     */
    class CTRVProcessModel : public CTRVProcessModelBase
    {
    public:
      /**
       * Constructor.
       */
      CTRVProcessModel();

      /**
       * A state transition function without the noise vector.
       *
       * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
       * @param cv a control vector
       * @param sv a state vector
       * @return a prior state vector, that is, a state vector after the state transition function applied.
       */
      static CTRVStateVector g(double_t dt, const CTRVControlVector& cv, const CTRVStateVector& sv);

      /**
       * A state transition function with the noise vector. Some filters may require noise terms to be incorporated
       * into this non-linear state transition function.
       *
       * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
       * @param cv a control vector
       * @param sv a state vector
       * @param nv a noise vector
       * @return a prior state vector, that is, a state vector after the state transition function applied.
       */
      static CTRVStateVector
      g(double_t dt, const CTRVControlVector& cv, const CTRVStateVector& sv, const CTRVProcessNoiseVector& nv);

      /**
       * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
       * @param sv a state vector
       * @return a state transition matrix
       */
      CTRVStateTransitionMatrix G(double_t dt, const CTRVStateVector& sv) const;

      /**
       * @param dt a difference between the current measurement timestamp and the previous measurement timestamp
       * @param sv a state vector
       * @return a process covariance matrix
       */
      CTRVProcessCovarianceMatrix R(double_t dt, const CTRVStateVector& sv) const;

      /**
       * Subtract one state vector from another.
       * Dimension representing yaw angle that is normalized to be within the [-pi, pi] range.
       *
       * @tparam StateVector_type a type of a state vector (ordinary or augmented, that is, that has more dimensions)
       * @param sv_1 a state vector to subtract from
       * @param sv_2 a state vector which to subtract
       *
       * @return a difference between two state vectors with normalized yaw angle
       */
      template <class StateVector_type>
      static auto Subtract(const StateVector_type& sv_1, const StateVector_type& sv_2)
      -> std::enable_if_t<StateVector_type::SizeAtCompileTime >= StateDims(), StateVector_type>
      {
        StateVector_type sv{sv_1 - sv_2};
        const CTRVRWStateVectorViewBase<StateVector_type> svv{sv};

        Utils::NormalizeAngle(&svv.yaw());

        return sv;
      }

      /**
       * Sums two vectors.
       * Dimension representing yaw angle that is normalized to be within the [-pi, pi] range.
       *
       * @tparam StateVector_type a type of a state vector (ordinary or augmented, that is, that has more dimensions)
       * @param sv_1 a first state vector
       * @param sv_2 a second state vector
       *
       * @return a sum of two state vectors with normalized yaw angle
       */
      template <class StateVector_type>
      static auto Add(const StateVector_type& sv_1, const StateVector_type& sv_2)
      -> std::enable_if_t<StateVector_type::SizeAtCompileTime >= StateDims(), StateVector_type>
      {
        StateVector_type sv{sv_1 + sv_2};
        const CTRVRWStateVectorViewBase<StateVector_type> svv{sv};

        Utils::NormalizeAngle(&svv.yaw());

        return sv;
      }

    private:
      CTRVStateTransitionMatrix state_transition_matrix_prototype_;

    };
  }
}


#endif //SENSOR_FUSION_CTRVPROCESSMODEL_HPP
