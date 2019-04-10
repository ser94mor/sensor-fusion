//
// Created by aoool on 06.04.19.
//

#ifndef SENSOR_FUSION_CTRVSTATEVECTOR_HPP
#define SENSOR_FUSION_CTRVSTATEVECTOR_HPP


#include "definitions.hpp"


/**
 * A wrapper around StateVector for CTRV process model (which is just an Eigen vector)
 * that provides meaningful accessors to the StateVector components.
 */
struct CTRVStateVector : public StateVector
{

  /**
   * @return X-axis coordinate
   */
  double px() const
  {
    return (*this)(0);
  }

  /**
   * @return Y-axis coordinate
   */
  double py() const
  {
    return (*this)(1);
  }

  /**
   * @return velocity module
   */
  double v() const
  {
    return (*this)(2);
  }

  /**
   * @return yaw rotation angle
   */
  double yaw() const
  {
    return (*this)(3);
  }

  /**
   * @return angular velocity of yaw rotation
   */
  double yaw_rate() const
  {
    return (*this)(4);
  }

};

#endif //SENSOR_FUSION_CTRVSTATEVECTOR_HPP
