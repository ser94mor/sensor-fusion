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

      class ConstStateVectorView : ser94mor::sensor_fusion::ConstStateVectorView<StateVector>
      {
      public:

        explicit ConstStateVectorView(const StateVector& state_vector)
        : ser94mor::sensor_fusion::ConstStateVectorView<StateVector>{state_vector}
        {

        }

        double px() const override
        {
          return state_vector_(0);
        }

        double py() const override
        {
          return state_vector_(1);
        }

        double vx() const override
        {
          return v() * std::cos(yaw());
        }

        double vy() const override
        {
          return v() * std::sin(yaw());
        }

        double v() const override
        {
          return state_vector_(2);
        }

        double yaw() const override
        {
          return state_vector_(3);
        }

        double yaw_rate() const override
        {
          return state_vector_(4);
        }

        double range() const override
        {
          double rho{std::sqrt(px()*px() + py()*py())};
          return (rho < kEpsilon) ? kEpsilon : rho;
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

      /**
       * A wrapper around StateVector for CTRV process model (which is just an Eigen vector)
       * that provides meaningful accessors to the StateVector components.
       */
      class StateVectorView
      {
      public:

        /**
         * Constructor.
         * @param state_vector a state vector
         */
        explicit StateVectorView(StateVector& state_vector) : state_vector_modifiable_{state_vector}
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

        double& v()
        {
          return state_vector_modifiable_(2);
        }

        double& yaw()
        {
          return state_vector_modifiable_(3);
        }

        double& yaw_rate()
        {
          return state_vector_modifiable_(4);
        }

      private:
        StateVector& state_vector_modifiable_;
      };

    }
  }
}

#endif //SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
