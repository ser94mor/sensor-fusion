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
     * A template class holding Kalman Filter equations.
     * The naming of vectors and matrices are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     *
     * @tparam ProcessModel a class of the process model to use
     * @tparam MeasurementModel a class of measurement model to use; notice that here it is not a template class
     */
    template<class ProcessModel, class MeasurementModel>
    class ExtendedKalmanFilter : public KalmanFilter<ProcessModel, MeasurementModel>
    {
    public:
      /**
       * Prediction step of the Kalman filter. Predicts the object's state in dt time in the future in accordance with
       * process model and input control vector.
       *
       * @param belief_posterior a current belief of the object's state
       * @param ut a control vector
       * @param dt time interval between the previous and current measurements
       * @param process_model an instance of the process model
       *
       * @return a prior belief, that is, after prediction but before incorporating the measurement
       */
      static Belief Predict(const Belief& belief_posterior, const ControlVector& ut,
                            std::time_t dt, const ProcessModel& process_model);

      /**
       * Update step of the Kalman filter. Incorporates the sensor measurement into the given prior belief.
       *
       * @param belief_prior a belief after the prediction Kalman filter step
       * @param measurement a measurement from the sensor
       * @param measurement_model an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      static Belief Update(const Belief& belief_prior, const Measurement& measurement,
                           const MeasurementModel& measurement_model);
    };

    template<class ProcessModel, class MeasurementModel>
    typename ExtendedKalmanFilter<ProcessModel, MeasurementModel>::Belief
    ExtendedKalmanFilter<ProcessModel, MeasurementModel>::Predict(
        const Belief& belief_posterior, const ControlVector& ut, std::time_t dt, const ProcessModel& process_model)
    {
      if constexpr (ProcessModel::IsLinear())
      {
        return KalmanFilter<ProcessModel, MeasurementModel>::
               Predict(belief_posterior, ut, dt, process_model);
      }
      else
      {
        throw std::runtime_error("NOT IMPLEMENTED YET");
      }
    }

    template<class ProcessModel, class MeasurementModel>
    typename ExtendedKalmanFilter<ProcessModel, MeasurementModel>::Belief
    ExtendedKalmanFilter<ProcessModel, MeasurementModel>::Update(const Belief& belief_prior, const Measurement& measurement,
                                                         const MeasurementModel& measurement_model)
    {
      static_assert(MeasurementModel::IsLinear(),
                    "KalmanFilter works only with linear measurement models, "
                    "while the given measurement model is non-linear.");

      auto Ct{measurement_model.C()};
      auto mu{belief_prior.mu()};
      auto Sigma{belief_prior.Sigma()};
      auto Kt{Sigma * Ct.transpose() * (Ct * Sigma * Ct.transpose() + measurement_model.Q()).inverse()};
      auto I{Eigen::Matrix<double, ProcessModel::StateDims(), ProcessModel::StateDims()>::Identity()};

      return {
          /* timestamp */               measurement.t(),
          /* state vector */            mu + Kt * (measurement.z() - Ct * mu),
          /* state covariance matrix */ (I - Kt * Ct) * Sigma,
      };
    }

  }
}


#endif //SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP
