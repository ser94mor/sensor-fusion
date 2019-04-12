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

#ifndef SENSOR_FUSION_BELIEF_HPP
#define SENSOR_FUSION_BELIEF_HPP

#include "definitions.hpp"


namespace ser94mor::sensor_fusion
{

  /**
   * A concrete representation of "belief" concept from Bayesian filtering for Gaussian filters.
   *
   * For Gaussian case, the belief consists of the mean and covariance matrix.
   */
  template<class StateVector, class StateCovarianceMatrix>
  struct Belief
  {
    /**
     * State vector notation from the Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press.
     * @return state vector
     */
    const StateVector& mu() const
    {
      return state_vector;
    }

    /**
     * State covariance matrix notation from the
     * Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press.
     * @return state covariance matrix
     */
    const StateCovarianceMatrix& Sigma() const
    {
      return state_covariance_matrix;
    }

    const StateVector state_vector;
    const StateCovarianceMatrix state_covariance_matrix;
  };

}

#endif //SENSOR_FUSION_BELIEF_HPP