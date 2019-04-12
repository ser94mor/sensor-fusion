//
// Created by aoool on 06.04.19.
//

#ifndef SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
#define SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP


#include "definitions.hpp"

namespace ser94mor::sensor_fusion::CTRV
{

  /**
   * A wrapper around StateVector for CTRV process model (which is just an Eigen vector)
   * that provides meaningful accessors to the StateVector components.
   */
  class StateVectorView
  {

    /**
     * @return X-axis coordinate
     */
    double px() const
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

    const StateVector& state_vector_;

  };

}

#endif //SENSOR_FUSION_CTRVSTATEVECTORVIEW_HPP
