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
     * A base class for read-write wrappers around StateVector for some process model
     * that provides meaningful accessors and setters to the StateVector components.
     *
     * @tparam StateVector a class of the state vector
     */
    template <class StateVector>
    class RWStateVectorView
    {
    protected:
      /**
       * Constructor.
       * @param state_vector a state vector
       */
      explicit RWStateVectorView(StateVector& state_vector) : state_vector_modifiable_{state_vector}
      {

      }

      StateVector& state_vector_modifiable_;
    };

    /**
     * A base class for read-only wrappers around StateVector for some process model
     * that provides meaningful accessors to the StateVector components.
     *
     *  @tparam StateVector a class of the state vector
     */
    template <class StateVector>
    class ROStateVectorView
    {
    public:
      /**
       * @return X-axis coordinate
       */
      virtual double_t px() const = 0;

      /**
       * @return Y-axis coordinate
       */
      virtual double_t py() const = 0;

      /**
       * @return X-axis velocity
       */
      virtual double_t vx() const = 0;

      /**
       * @return Y-axis velocity
       */
      virtual double_t vy() const = 0;

      /**
       * @return velocity module
       */
      virtual double_t v() const = 0;

      /**
       * @return yaw rotation angle
       */
      virtual double_t yaw() const = 0;

      /**
       * @return angular velocity of yaw rotation
       */
      virtual double_t yaw_rate() const = 0;

      /**
       * @return range: radial distance from origin
       */
      virtual double_t range() const = 0;

      /**
       * @return bearing: angle between range and X-axis
       * (which points into the direction of heading of our car, where sensors are installed)
       */
      virtual double_t bearing() const = 0;

      /**
       * @return radial velocity: change of range, i.e., range rate
       */
      virtual double_t range_rate() const = 0;

    protected:
      /**
       * Constructor.
       * @param state_vector a state vector
       */
      explicit ROStateVectorView(const StateVector& state_vector) : state_vector_{state_vector}
      {

      }

      const StateVector& state_vector_;
    };

    /**
     * A base class for read-only wrappers around ProcessNoiseVector for some process model
     * that provides meaningful accessors to the ProcessNoiseVector components.
     *
     *  @tparam ProcessNoiseVector a class of the process noise vector
     */
    template <class ProcessNoiseVector>
    class ROProcessNoiseVectorView
    {
    protected:
      /**
       * Constructor.
       * @param process_noise_vector a process noise vector
       */
      explicit ROProcessNoiseVectorView(const ProcessNoiseVector& process_noise_vector)
      : process_noise_vector_{process_noise_vector}
      {

      }

      const ProcessNoiseVector& process_noise_vector_;
    };

  }
}


#endif //SENSOR_FUSION_STATEVECTORVIEW_HPP
