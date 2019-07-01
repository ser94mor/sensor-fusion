/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
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


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base (template) class for representation of "belief" concept from Bayesian filtering.
     */
    template<class StateVector_t, class StateCovarianceMatrix_t>
    class Belief
    {
    public:
      /**
       * Belief's timestamp.
       * @return timestamp
       */
      double_t t() const
      {
        return timestamp_;
      }

      /**
       * State vector notation from the Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press.
       * @return state vector
       */
      const StateVector_t& mu() const
      {
        return state_vector_;
      }

      /**
       * State covariance matrix notation from the
       * Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press.
       * @return state covariance matrix
       */
      const StateCovarianceMatrix_t& Sigma() const
      {
        return state_covariance_matrix_;
      }

      Belief(double_t tm, const StateVector_t& sv, const StateCovarianceMatrix_t& scm)
      : timestamp_{tm}, state_vector_{sv}, state_covariance_matrix_{scm}
      {

      }

      Belief(const Belief& bel)
          : timestamp_{bel.timestamp_}, state_vector_{bel.state_vector_},
            state_covariance_matrix_{bel.state_covariance_matrix_}
      {

      }

      Belief(Belief&& bel) noexcept
          : timestamp_{bel.timestamp_},
            state_vector_{std::move(bel.state_vector_)},
            state_covariance_matrix_{std::move(bel.state_covariance_matrix_)}
      {

      }

      Belief& operator=(const Belief& bel)
      {
        if (&bel != this)
        {
          timestamp_ = bel.timestamp_;
          state_vector_ = bel.state_vector_;
          state_covariance_matrix_ = bel.state_covariance_matrix_;
        }
        return *this;
      }

      Belief& operator=(Belief&& bel) noexcept
      {
        if (&bel != this)
        {
          timestamp_ = bel.timestamp_;
          state_vector_ = std::move(bel.state_vector_);
          state_covariance_matrix_ = std::move(bel.state_covariance_matrix_);
        }
        return *this;
      }

      bool operator==(const Belief& bel) const
      {
        return timestamp_ == bel.timestamp_
               && state_vector_.isApprox(bel.state_vector_)
               && state_covariance_matrix_.isApprox(bel.state_covariance_matrix_);
      }

    private:
      double_t timestamp_;
      StateVector_t state_vector_;
      StateCovarianceMatrix_t state_covariance_matrix_;
    };

  }
}

#endif //SENSOR_FUSION_BELIEF_HPP
