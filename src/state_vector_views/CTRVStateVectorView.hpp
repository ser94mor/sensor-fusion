//
// Created by aoool on 06.04.19.
//

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
       * @param state_vector a state vector
       */
      explicit CTRVROStateVectorView(const CTRVStateVector& state_vector)
      : ROStateVectorView<CTRVStateVector>{state_vector}
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
        return v() * std::cos(yaw());
      }

      /**
       * @return Y-axis velocity
       */
      virtual double_t vy() const override
      {
        return v() * std::sin(yaw());
      }

      /**
       * @return velocity module
       */
      virtual double_t v() const override
      {
        return GetVector()(2);
      }

      /**
       * @return yaw rotation angle
       */
      virtual double_t yaw() const override
      {
        return GetVector()(3);
      }

      /**
       * @return angular velocity of yaw rotation
       */
      virtual double_t yaw_rate() const override
      {
        return GetVector()(4);
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
       * @param state_vector a state vector
       */
      explicit CTRVRWStateVectorViewBase(StateVector_type& state_vector)
      : RWStateVectorView<StateVector_type>{state_vector}
      {

      }

      /**
       * @return X-axis coordinate
       */
      double_t& px() const
      {
        return this->GetVector()(0);
      }

      /**
       * @return Y-axis coordinate
       */
      double_t& py() const
      {
        return this->GetVector()(1);
      }

      /**
       * @return velocity module
       */
      double_t& v() const
      {
        return this->GetVector()(2);
      }

      /**
       * @return yaw rotation angle
       */
      double_t& yaw() const
      {
        return this->GetVector()(3);
      }

      /**
       * @return angular velocity of yaw rotation
       */
      double_t& yaw_rate() const
      {
        return this->GetVector()(4);
      }

    };

    /**
     * A read-write wrapper around StateVector for CTRV process model
     * that provides meaningful accessors and setters to the StateVector components.
     */
    class CTRVRWStateVectorView : public CTRVRWStateVectorViewBase<CTRVStateVector>
    {
    public:
      /**
       * Constructor.
       * @param state_vector a state vector
       */
      explicit CTRVRWStateVectorView(CTRVStateVector& state_vector)
      : CTRVRWStateVectorViewBase<CTRVStateVector>{state_vector}
      {

      }
    };

    /**
     * A read-only wrapper around ProcessNoiseVector for CTRV process model
     * that provides meaningful accessors to the ProcessNoiseVector components.
     */
    class CTRVROProcessNoiseVectorView : ROProcessNoiseVectorView<CTRVProcessNoiseVector>
    {
    public:
      /**
       * Constructor.
       * @param process_noise_vector a process noise vector
       */
      explicit CTRVROProcessNoiseVectorView(const CTRVProcessNoiseVector& process_noise_vector)
      : ROProcessNoiseVectorView<CTRVProcessNoiseVector>{process_noise_vector}
      {

      }

      /**
       * @return a longitudinal acceleration
       */
      double_t longitudinal_acceleration() const
      {
        return GetVector()(0);
      }

      /**
       * @return a yaw acceleration
       */
      double_t yaw_acceleration() const
      {
        return GetVector()(1);
      }
    };

  }
}

#endif //SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
