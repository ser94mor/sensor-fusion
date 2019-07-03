/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
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

#ifndef SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
#define SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP


#include "definitions.hpp"
#include "StateVectorView.hpp"

#include <cmath>


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A read-only wrapper around StateVector for CTRV process model
     * that provides meaningful accessors to the StateVector components.
     */
    class CTRVROStateVectorView : ROStateVectorView<CTRVStateVector>
    {
    public:

      /**
       * Constructor.
       * @param sv a state vector
       */
      explicit CTRVROStateVectorView(const CTRVStateVector& sv);

      /**
       * @return X-axis coordinate
       */
      virtual double_t px() const override;

      /**
       * @return Y-axis coordinate
       */
      virtual double_t py() const override;

      /**
       * @return X-axis velocity
       */
      virtual double_t vx() const override;

      /**
       * @return Y-axis velocity
       */
      virtual double_t vy() const override;

      /**
       * @return velocity module
       */
      virtual double_t v() const override;

      /**
       * @return yaw rotation angle
       */
      virtual double_t yaw() const override;

      /**
       * @return angular velocity of yaw rotation
       */
      virtual double_t yaw_rate() const override;

      /**
       * @return range: radial distance from origin
       */
      virtual double_t range() const override;

      /**
       * @return bearing: angle between range and X-axis
       * (which points into the direction of heading of our car, where sensors are installed)
       */
      virtual double_t bearing() const override;

      /**
       * @return radial velocity: change of range, i.e., range rate
       */
      virtual double_t range_rate() const override;

    };

    /**
     * A read-write wrapper around vector which first dimensions match those in the StateVector and
     * provides meaningful accessors and setters to those components.
     *
     * @tparam StateVector_type a class of a state vector
    */
    template<class StateVector_type>
    class CTRVRWStateVectorViewBase : RWStateVectorView<StateVector_type>
    {
    public:
      /**
       * Constructor.
       * @param sv a state vector
       */
      explicit CTRVRWStateVectorViewBase(StateVector_type& sv);

      /**
       * @return X-axis coordinate
       */
      double_t& px() const;

      /**
       * @return Y-axis coordinate
       */
      double_t& py() const;

      /**
       * @return velocity module
       */
      double_t& v() const;

      /**
       * @return yaw rotation angle
       */
      double_t& yaw() const;

      /**
       * @return angular velocity of yaw rotation
       */
      double_t& yaw_rate() const;

    };


    /**
     * A read-write wrapper around StateVector for CTRV process model
     * that provides meaningful accessors and setters to the StateVector components.
     */
    using CTRVRWStateVectorView = CTRVRWStateVectorViewBase<CTRVStateVector>;

    /**
     * A read-only wrapper around ProcessNoiseVector for CTRV process model
     * that provides meaningful accessors to the ProcessNoiseVector components.
     */
    class CTRVROProcessNoiseVectorView : ROProcessNoiseVectorView<CTRVProcessNoiseVector>
    {
    public:
      /**
       * Constructor.
       * @param pnv a process noise vector
       */
      explicit CTRVROProcessNoiseVectorView(const CTRVProcessNoiseVector& pnv);

      /**
       * @return a longitudinal acceleration
       */
      double_t longitudinal_acceleration() const;

      /**
       * @return a yaw acceleration
       */
      double_t yaw_acceleration() const;
    };


    ////////////////////
    // IMPLEMENTATION //
    ////////////////////

    template<class StateVector_type>
    CTRVRWStateVectorViewBase<StateVector_type>::CTRVRWStateVectorViewBase(StateVector_type& sv)
        : RWStateVectorView<StateVector_type>{sv}
    {

    }

    template<class StateVector_type>
    double_t& CTRVRWStateVectorViewBase<StateVector_type>::px() const
    {
      return this->GetVector()(0);
    }

    template<class StateVector_type>
    double_t& CTRVRWStateVectorViewBase<StateVector_type>::py() const
    {
      return this->GetVector()(1);
    }

    template<class StateVector_type>
    double_t& CTRVRWStateVectorViewBase<StateVector_type>::v() const
    {
      return this->GetVector()(2);
    }

    template<class StateVector_type>
    double_t& CTRVRWStateVectorViewBase<StateVector_type>::yaw() const
    {
      return this->GetVector()(3);
    }

    template<class StateVector_type>
    double_t& CTRVRWStateVectorViewBase<StateVector_type>::yaw_rate() const
    {
      return this->GetVector()(4);
    }


  }
}

#endif //SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
