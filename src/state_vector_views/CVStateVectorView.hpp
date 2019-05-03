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

#ifndef SENSOR_FUSION_CVSTATEVECTORVIEW_HPP
#define SENSOR_FUSION_CVSTATEVECTORVIEW_HPP


#include "definitions.hpp"
#include "StateVectorView.hpp"

#include <cmath>


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace CV
    {

      /**
       * A read-write wrapper around StateVector for CV process model
       * that provides meaningful accessors and setters to the StateVector components.
       */
      class RWStateVectorView : public ser94mor::sensor_fusion::RWStateVectorView<StateVector>
      {
      public:
        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit RWStateVectorView(StateVector& state_vector)
        : ser94mor::sensor_fusion::RWStateVectorView<StateVector>{state_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double_t& px()
        {
          return state_vector_modifiable_(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double_t& py()
        {
          return state_vector_modifiable_(1);
        }

        /**
         * @return X-axis velocity
         */
        double_t& vx()
        {
          return state_vector_modifiable_(2);
        }

        /**
         * @return Y-axis velocity
         */
        double_t& vy()
        {
          return state_vector_modifiable_(3);
        }
      };

      /**
       * A read-only wrapper around StateVector for CV process model
       * that provides meaningful accessors to the StateVector components.
       */
      class ROStateVectorView : public ser94mor::sensor_fusion::ROStateVectorView<CV::StateVector>
      {
      public:

        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit ROStateVectorView(const StateVector& state_vector)
        : ser94mor::sensor_fusion::ROStateVectorView<CV::StateVector>{state_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double_t px() const override
        {
          return state_vector_(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double_t py() const override
        {
          return state_vector_(1);
        }

        /**
         * @return X-axis velocity
         */
        double_t vx() const override
        {
          return state_vector_(2);
        }

        /**
         * @return Y-axis velocity
         */
        double_t vy() const override
        {
          return state_vector_(3);
        }

        /**
         * @return velocity module
         */
        double_t v() const override
        {
          return std::sqrt(vx()*vx() + vy()*vy());
        }

        /**
         * @return yaw rotation angle
         */
        double_t yaw() const override
        {
          return std::acos(vx()/v());
        }

        /**
         * @return angular velocity of yaw rotation
         */
        double_t yaw_rate() const override
        {
          return 0.0;
        }

        /**
         * @return range: radial distance from origin
         */
        double_t range() const override
        {
          double_t rho{std::sqrt(px()*px() + py()*py())};
          return (rho < kEpsilon) ? kEpsilon : rho;
        }

        /**
         * @return bearing: angle between range and X-axis
         * (which points into the direction of heading of our car, where sensors are installed)
         */
        double_t bearing() const override
        {
          return std::atan2(py(), px());
        }

        /**
         * @return radial velocity: change of range, i.e., range rate
         */
        double_t range_rate() const override
        {
          return (px()*vx() + py()*vy()) / range();
        }

      };

    }
  }
}

#endif //SENSOR_FUSION_CVSTATEVECTORVIEW_HPP
