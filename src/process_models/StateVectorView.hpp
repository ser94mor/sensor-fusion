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

#ifndef SENSOR_FUSION_STATEVECTORVIEW_HPP
#define SENSOR_FUSION_STATEVECTORVIEW_HPP


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base class for wrappers around StateVector for some process model (which is just an Eigen vector)
     * that provides meaningful accessors to the StateVector components.
     */
    template <class StateVector>
    class StateVectorView
    {
    public:
      /**
       * Constructor.
       * @param state_vector a state vector
       */
      explicit StateVectorView(StateVector& state_vector) : state_vector_{state_vector}
      {

      }

      /**
       * @return X-axis coordinate
       */
      virtual double px() const = 0;

      /**
       * @return X-axis coordinate
       */
      virtual double& px() = 0;

      /**
       * @return Y-axis coordinate
       */
      virtual double py() const = 0;

      /**
       * @return Y-axis coordinate
       */
      virtual double& py() = 0;

      /**
       * @return X-axis velocity
       */
      virtual double vx() const = 0;

      /**
       * @return Y-axis velocity
       */
      virtual double vy() const = 0;

      /**
       * @return velocity module
       */
      virtual double v() const = 0;

      /**
       * @return yaw rotation angle
       */
      virtual double yaw() const = 0;

      /**
       * @return angular velocity of yaw rotation
       */
      virtual double yaw_rate() const = 0;

      /**
       * @return range: radial distance from origin
       */
      virtual double range() const = 0;

      /**
       * @return bearing: angle between range and X-axis
       * (which points into the direction of heading of our car, where sensors are installed)
       */
      virtual double bearing() const = 0;

      /**
       * @return radial velocity: change of range, i.e., range rate
       */
      virtual double range_rate() const = 0;

    protected:
      StateVector& state_vector_;
    };

  }
}


#endif //SENSOR_FUSION_STATEVECTORVIEW_HPP
