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
    namespace CTRV
    {

      /**
       * A read-only wrapper around StateVector for CTRV process model
       * that provides meaningful accessors to the StateVector components.
       */
      class ROStateVectorView : ser94mor::sensor_fusion::ROStateVectorView<StateVector>
      {
      public:

        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit ROStateVectorView(const StateVector& state_vector)
        : ser94mor::sensor_fusion::ROStateVectorView<StateVector>{state_vector}
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
          return v() * std::cos(yaw());
        }

        /**
         * @return Y-axis velocity
         */
        double_t vy() const override
        {
          return v() * std::sin(yaw());
        }

        /**
         * @return velocity module
         */
        double_t v() const override
        {
          return state_vector_(2);
        }

        /**
         * @return yaw rotation angle
         */
        double_t yaw() const override
        {
          return state_vector_(3);
        }

        /**
         * @return angular velocity of yaw rotation
         */
        double_t yaw_rate() const override
        {
          return state_vector_(4);
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

      /**
       * A read-write wrapper around vector which first dimensions match those in the StateVector and
       * provides meaningful accessors and setters to those components.
       *
       * @tparam StateVector_type a class of a state vector
      */
      template<class StateVector_type>
      class RWStateVectorViewBase : ser94mor::sensor_fusion::RWStateVectorView<StateVector_type>
      {
      public:
        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit RWStateVectorViewBase(StateVector_type& state_vector)
        : ser94mor::sensor_fusion::RWStateVectorView<StateVector_type>{state_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double_t& px()
        {
          return this->state_vector_modifiable_(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double_t& py()
        {
          return this->state_vector_modifiable_(1);
        }

        /**
         * @return velocity module
         */
        double_t& v()
        {
          return this->state_vector_modifiable_(2);
        }

        /**
         * @return yaw rotation angle
         */
        double_t& yaw()
        {
          return this->state_vector_modifiable_(3);
        }

        /**
         * @return angular velocity of yaw rotation
         */
        double_t& yaw_rate()
        {
          return this->state_vector_modifiable_(4);
        }
        
      };

      /**
       * A read-write wrapper around StateVector for CTRV process model
       * that provides meaningful accessors and setters to the StateVector components.
       */
      class RWStateVectorView : public RWStateVectorViewBase<StateVector>
      {
      public:
        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit RWStateVectorView(StateVector& state_vector) : RWStateVectorViewBase<StateVector>{state_vector}
        {

        }
      };

      /**
       * A read-only wrapper around ProcessNoiseVector for CTRV process model
       * that provides meaningful accessors to the ProcessNoiseVector components.
       */
      class ROProcessNoiseVectorView : ser94mor::sensor_fusion::ROProcessNoiseVectorView<ProcessNoiseVector>
      {
      public:
        /**
         * Constructor.
         * @param process_noise_vector a process noise vector
         */
        explicit ROProcessNoiseVectorView(const ProcessNoiseVector& process_noise_vector)
        : ser94mor::sensor_fusion::ROProcessNoiseVectorView<ProcessNoiseVector>{process_noise_vector}
        {

        }

        /**
         * @return a longitudinal acceleration
         */
        double_t longitudinal_acceleration() const
        {
          return process_noise_vector_(0);
        }

        /**
         * @return a yaw acceleration
         */
        double_t yaw_acceleration() const
        {
          return process_noise_vector_(1);
        }
      };

    }
  }
}

#endif //SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
