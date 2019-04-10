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

#ifndef SENSOR_FUSION_CVSTATEVECTOR_HPP
#define SENSOR_FUSION_CVSTATEVECTOR_HPP


#include "definitions.hpp"


/**
 * A wrapper around StateVector for CV process model (which is just an Eigen vector)
 * that provides meaningful accessors to the StateVector components.
 */
class CVStateVector: public StateVector
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
   * @return X-axis velocity
   */
  double vx() const
  {
    return (*this)(2);
  }

  /**
   * @return Y-axis velocity
   */
  double vy() const
  {
    return (*this)(3);
  }

};

#endif //SENSOR_FUSION_CVSTATEVECTOR_HPP
