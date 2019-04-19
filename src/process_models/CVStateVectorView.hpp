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
       * A wrapper around StateVector for CV process model (which is just an Eigen vector)
       * that provides meaningful accessors to the StateVector components.
       */
    class StateVectorView : public ser94mor::sensor_fusion::StateVectorView<CV::StateVector>
      {
      public:

        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit StateVectorView(StateVector& state_vector)
        : ser94mor::sensor_fusion::StateVectorView<CV::StateVector>{state_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double px() const override
        {
          return state_vector_(0);
        }

        /**
         * @return X-axis coordinate
         */
        double& px() override
        {
          return state_vector_(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double py() const override
        {
          return state_vector_(1);
        }

        /**
         * @return Y-axis coordinate
         */
        double& py() override
        {
          return state_vector_(1);
        }

        /**
         * @return X-axis velocity
         */
        double vx() const override
        {
          return state_vector_(2);
        }

        /**
         * @return Y-axis velocity
         */
        double vy() const override
        {
          return state_vector_(3);
        }

      double v() const override
      {
        return std::sqrt(vx()*vx() + vy()*vy());
      }

      double yaw() const override
      {
        // TODO: implement while adding CTRV process model
        return 0.;
      }

      double yaw_rate() const override
      {
        // TODO: implement while adding CTRV process model
        return 0.;
      }

      double range() const override
      {
        double epsilon{0.00001};
        double rho{std::sqrt(px()*px() + py()*py())};
        return (rho < epsilon) ? epsilon : rho;
      }

      double bearing() const override
      {
        return std::atan2(py(), px());
      }

      double range_rate() const override
      {
        return (px()*vx() + py()*vy()) / range();
      }

    };

    }
  }
}

#endif //SENSOR_FUSION_CVSTATEVECTORVIEW_HPP
