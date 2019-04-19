//
// Created by aoool on 06.04.19.
//

#ifndef SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
#define SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP


#include "definitions.hpp"
#include "StateVectorView.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace CTRV
    {

      /**
       * A wrapper around StateVector for CTRV process model (which is just an Eigen vector)
       * that provides meaningful accessors to the StateVector components.
       */
      class StateVectorView : ser94mor::sensor_fusion::StateVectorView<StateVector>
      {
      public:

        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit StateVectorView(StateVector& state_vector)
        : ser94mor::sensor_fusion::StateVectorView<StateVector>{state_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double px() const
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
        double py() const
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
         * @return velocity module
         */
        double v() const
        {
          return state_vector_(2);
        }

        /**
         * @return yaw rotation angle
         */
        double yaw() const
        {
          return state_vector_(3);
        }

        /**
         * @return angular velocity of yaw rotation
         */
        double yaw_rate() const
        {
          return state_vector_(4);
        }
      };

    }
  }
}

#endif //SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
