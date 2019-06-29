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
         * @param sv a state vector
         */
        explicit RWStateVectorView(StateVector& sv) : ser94mor::sensor_fusion::RWStateVectorView<StateVector>{sv}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double_t& px() const
        {
          return GetVector()(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double_t& py() const
        {
          return GetVector()(1);
        }

        /**
         * @return X-axis velocity
         */
        double_t& vx() const
        {
          return GetVector()(2);
        }

        /**
         * @return Y-axis velocity
         */
        double_t& vy() const
        {
          return GetVector()(3);
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
         * @param sv a state vector
         */
        explicit ROStateVectorView(const StateVector& sv)
        : ser94mor::sensor_fusion::ROStateVectorView<CV::StateVector>{sv}
        {

        }

        /**
         * @return X-axis coordinate
         */
        virtual double_t px() const override
        {
          return GetVector()(0);
        }

        /**
         * @return Y-axis coordinate
         */
        virtual double_t py() const override
        {
          return GetVector()(1);
        }

        /**
         * @return X-axis velocity
         */
        virtual double_t vx() const override
        {
          return GetVector()(2);
        }

        /**
         * @return Y-axis velocity
         */
        virtual double_t vy() const override
        {
          return GetVector()(3);
        }

        /**
         * @return velocity module
         */
        virtual double_t v() const override
        {
          return std::sqrt(vx()*vx() + vy()*vy());
        }

        /**
         * @return yaw rotation angle
         */
        virtual double_t yaw() const override
        {
          return std::acos(vx()/v());
        }

        /**
         * @return angular velocity of yaw rotation
         */
        virtual double_t yaw_rate() const override
        {
          return 0.0;
        }

        /**
         * @return range: radial distance from origin
         */
        virtual double_t range() const override
        {
          const double_t rho{std::sqrt(px()*px() + py()*py())};
          return (rho < kEpsilon) ? kEpsilon : rho;
        }

        /**
         * @return bearing: angle between range and X-axis
         * (which points into the direction of heading of our car, where sensors are installed)
         */
        virtual double_t bearing() const override
        {
          return std::atan2(py(), px());
        }

        /**
         * @return radial velocity: change of range, i.e., range rate
         */
        virtual double_t range_rate() const override
        {
          return (px()*vx() + py()*vy()) / range();
        }

      };

    }
  }
}

#endif //SENSOR_FUSION_CVSTATEVECTORVIEW_HPP
