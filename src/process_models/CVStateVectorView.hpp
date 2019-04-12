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

#ifndef SENSOR_FUSION_CVSTATEVECTORVIEW_HPP
#define SENSOR_FUSION_CVSTATEVECTORVIEW_HPP


#include "definitions.hpp"


namespace ser94mor::sensor_fusion::CV
{

  /**
   * A wrapper around StateVector for CV process model (which is just an Eigen vector)
   * that provides meaningful accessors to the StateVector components.
   */
  class StateVectorView
  {
  public:

    explicit StateVectorView(const StateVector& state_vector) : state_vector_{state_vector}
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
     * @return Y-axis coordinate
     */
    double py() const
    {
      return state_vector_(1);
    }

    /**
     * @return X-axis velocity
     */
    double vx() const
    {
      return state_vector_(2);
    }

    /**
     * @return Y-axis velocity
     */
    double vy() const
    {
      return state_vector_(3);
    }

  private:
    const StateVector& state_vector_;

  };

}

#endif //SENSOR_FUSION_CVSTATEVECTORVIEW_HPP
