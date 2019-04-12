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

#ifndef SENSOR_FUSION_KALMANFILTER_HPP
#define SENSOR_FUSION_KALMANFILTER_HPP


#include <ctime>
#include <iostream>
#include <Eigen/Dense>


namespace ser94mor::sensor_fusion
{

  template<class ProcessModel, class MeasurementModel>
  class KalmanFilter
  {
    using Belief = typename ProcessModel::Belief_type;
    using Measurement = typename MeasurementModel::Measurement_type;
  public:
    virtual Belief Predict(const Belief& belief_posterior, std::time_t dt);

    virtual Belief Update(const Belief& belief_prior, const Measurement& measurement, std::time_t dt);

    KalmanFilter(const ProcessModel& process_model, const MeasurementModel& measurement_model);

  protected:
    const ProcessModel& process_model_;
    const MeasurementModel& measurement_model_;
  };

  template<class ProcessModel, class MeasurementModel>
  typename KalmanFilter<ProcessModel, MeasurementModel>::Belief
  KalmanFilter<ProcessModel, MeasurementModel>::Predict(const Belief& belief_posterior, std::time_t dt)
  {
    auto At{process_model_.A(dt)};
    return {
      .state_vector = At * belief_posterior.mu(),
      .state_covariance_matrix = At * belief_posterior.Sigma() * At.transpose() + process_model_.R(dt),
    };
  }

  template<class ProcessModel, class MeasurementModel>
  typename KalmanFilter<ProcessModel, MeasurementModel>::Belief
  KalmanFilter<ProcessModel, MeasurementModel>::Update(const Belief& belief_prior, const Measurement& measurement,
      std::time_t dt)
  {
    auto Ct{measurement_model_.C(dt)};
    auto mu{belief_prior.mu()};
    auto Sigma{belief_prior.Sigma()};
    auto Kt{Sigma * Ct.transpose() * (Ct * Sigma * Ct.transpose() + measurement_model_.Q(dt)).inverse()};
    auto I{Eigen::Matrix<double, ProcessModel::StateDims(), ProcessModel::StateDims()>::Identity()};

    return {
      .state_vector = mu + Kt * (measurement.z() - Ct * mu),
      .state_covariance_matrix = (I - Kt * Ct) * Sigma,
    };
  }

  template<class ProcessModel, class MeasurementModel>
  KalmanFilter<ProcessModel, MeasurementModel>::KalmanFilter(const ProcessModel& process_model,
                                                             const MeasurementModel& measurement_model) :
    process_model_{process_model}, measurement_model_{measurement_model}
  {

  }

}

#endif //SENSOR_FUSION_KALMANFILTER_HPP