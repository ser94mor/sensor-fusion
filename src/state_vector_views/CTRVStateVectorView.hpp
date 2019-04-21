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
       * A read-only wrapper around StateVector for CTRV process model (which is just an Eigen vector)
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
        double px() const override
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
         * @return X-axis velocity
         */
        double vx() const override
        {
          return v() * std::cos(yaw());
        }

        /**
         * @return Y-axis velocity
         */
        double vy() const override
        {
          return v() * std::sin(yaw());
        }

        /**
         * @return velocity module
         */
        double v() const override
        {
          return state_vector_(2);
        }

        /**
         * @return yaw rotation angle
         */
        double yaw() const override
        {
          return state_vector_(3);
        }

        /**
         * @return angular velocity of yaw rotation
         */
        double yaw_rate() const override
        {
          return state_vector_(4);
        }

        /**
         * @return range: radial distance from origin
         */
        double range() const override
        {
          double rho{std::sqrt(px()*px() + py()*py())};
          return (rho < kEpsilon) ? kEpsilon : rho;
        }

        /**
         * @return bearing: angle between range and X-axis
         * (which points into the direction of heading of our car, where sensors are installed)
         */
        double bearing() const override
        {
          return std::atan2(py(), px());
        }

        /**
         * @return radial velocity: change of range, i.e., range rate
         */
        double range_rate() const override
        {
          return (px()*vx() + py()*vy()) / range();
        }
        
      };

      /**
       * A read-write wrapper around StateVector for CTRV process model (which is just an Eigen vector)
       * that provides meaningful accessors and setters to the StateVector components.
       */
      class RWStateVectorView : ser94mor::sensor_fusion::RWStateVectorView<StateVector>
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
        double& px()
        {
          return state_vector_modifiable_(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double& py()
        {
          return state_vector_modifiable_(1);
        }

        /**
         * @return velocity module
         */
        double& v()
        {
          return state_vector_modifiable_(2);
        }

        /**
         * @return yaw rotation angle
         */
        double& yaw()
        {
          return state_vector_modifiable_(3);
        }

        /**
         * @return angular velocity of yaw rotation
         */
        double& yaw_rate()
        {
          return state_vector_modifiable_(4);
        }
        
      };

    }
  }
}

#endif //SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
