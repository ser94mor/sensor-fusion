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

#ifndef SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP
#define SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP


#include "KalmanFilter.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A template class holding extended Kalman filter equations.
     * The naming of vectors and matrices are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     *
     * @tparam ProcessModel a class of the process model to use
     * @tparam MeasurementModel a class of measurement model to use; notice that here it is not a template class
     */
    template<class ProcessModel, class MeasurementModel>
    class ExtendedKalmanFilter : public KalmanFilterBase<ProcessModel, MeasurementModel, ExtendedKalmanFilter>
    {
    private:
      using Belief = typename ProcessModel::Belief_type;
      using ControlVector = typename ProcessModel::ControlVector_type;
      using Measurement = typename MeasurementModel::Measurement_type;
    public:
      using KalmanFilterBase<ProcessModel, MeasurementModel, ExtendedKalmanFilter>::Predict;
      using KalmanFilterBase<ProcessModel, MeasurementModel, ExtendedKalmanFilter>::Update;

      /**
       * Prediction step of the Extended Kalman filter. Predicts the object's state in dt time in the future
       * in accordance with NON-LINEAR process model and input control vector.
       *
       * Notice that for linear process models the compiler will choose the corresponding method from the
       * base class (KalmanFilterBase) which works with linear process models.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt time interval between the previous and current measurements
       * @param process_model an instance of the process model
       *
       * @return a prior belief, that is, after prediction but before incorporating the measurement
       */
      template<bool enable = true>
      static auto
      Predict(const Belief& bel, const ControlVector& ut, double_t dt, const ProcessModel& process_model)
      -> std::enable_if_t<not ProcessModel::IsLinear() and enable, Belief>
      {
        auto mu{bel.mu()};
        auto Gt{process_model.G(dt, mu)};
        return {
            /* timestamp */               bel.t() + dt,
            /* state vector */            process_model.g(dt, ut, mu),
            /* state covariance matrix */ Gt * bel.Sigma() * Gt.transpose() + process_model.R(dt, mu),
        };
      }

      /**
       * Update step of the Extended Kalman filter. Incorporates the sensor measurement into the given prior belief.
       * Works only with NON-LINEAR measurement models.
       *
       * Notice that for linear measurement models the compiler will choose the corresponding method from the
       * base class (KalmanFilter) which works with linear measurement models.
       *
       * @param bel a belief after the prediction Extended Kalman filter step
       * @param measurement a measurement from the sensor
       * @param measurement_model an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template<bool enable = true>
      static auto
      Update(const Belief& bel, const Measurement& measurement, const MeasurementModel& measurement_model)
      -> std::enable_if_t<not MeasurementModel::IsLinear() and enable, Belief>
      {
        auto mu{bel.mu()};
        auto Sigma{bel.Sigma()};
        auto Ht{measurement_model.H(mu)};
        auto Kt{Sigma * Ht.transpose() * (Ht * Sigma * Ht.transpose() + measurement_model.Q()).inverse()};
        auto I{Eigen::Matrix<double_t, ProcessModel::StateDims(), ProcessModel::StateDims()>::Identity()};

        return {
          /* timestamp */               measurement.t(),
          /* state vector */            mu + Kt * measurement_model.Diff(measurement.z(), measurement_model.h(mu)),
          /* state covariance matrix */ (I - Kt * Ht) * Sigma,
        };
      }

    };

  }
}


#endif //SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP
