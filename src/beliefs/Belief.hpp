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

#include <ctime>


/**
 * A macros to be used to define concrete Belief classes. Each concrete belief class "lives" in its own namespace
 * corresponding to a name of the process model, so that it is possible to rely on the similar class names for
 * state vector and state covariance matrix.
 */
#define BELIEF_DEFINITION() \
  class Belief : public ser94mor::sensor_fusion::Belief<StateVector, StateCovarianceMatrix> { }



namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base (template) class for representation of "belief" concept from Bayesian filtering.
     */
    template<class StateVector, class StateCovarianceMatrix>
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
      const StateVector& mu() const
      {
        return state_vector_;
      }

      /**
       * State covariance matrix notation from the
       * Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press.
       * @return state covariance matrix
       */
      const StateCovarianceMatrix& Sigma() const
      {
        return state_covariance_matrix_;
      }

      Belief(double_t timestamp, const StateVector& state_vector,
             const StateCovarianceMatrix& state_covariance_matrix)
      : timestamp_{timestamp}, state_vector_{state_vector}, state_covariance_matrix_{state_covariance_matrix}
      {

      }

      Belief(const Belief& belief)
          : timestamp_{belief.timestamp_}, state_vector_{belief.state_vector_},
            state_covariance_matrix_{belief.state_covariance_matrix_}
      {

      }

      Belief(Belief&& belief) noexcept
          : timestamp_{belief.timestamp_},
            state_vector_{std::move(belief.state_vector_)},
            state_covariance_matrix_{std::move(belief.state_covariance_matrix_)}
      {

      }

      Belief& operator=(const Belief& belief)
      {
        timestamp_ = belief.timestamp_;
        state_vector_ = belief.state_vector_;
        state_covariance_matrix_ = belief.state_covariance_matrix_;
        return *this;
      }

      Belief& operator=(Belief&& belief) noexcept
      {
        timestamp_ = belief.timestamp_;
        state_vector_ = std::move(belief.state_vector_);
        state_covariance_matrix_ = std::move(belief.state_covariance_matrix_);
        return *this;
      }

      bool operator==(const Belief& belief) const
      {
        return timestamp_ == belief.timestamp_
               && state_vector_.isApprox(belief.state_vector_)
               && state_covariance_matrix_.isApprox(belief.state_covariance_matrix_);
      }

    private:
      double_t timestamp_;
      StateVector state_vector_;
      StateCovarianceMatrix state_covariance_matrix_;
    };

  }
}

#endif //SENSOR_FUSION_BELIEF_HPP
