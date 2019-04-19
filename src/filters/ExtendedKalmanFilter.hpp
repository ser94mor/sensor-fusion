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

#include <type_traits>


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
    private:
      using Belief = typename ProcessModel::Belief_type;
      using ControlVector = typename ProcessModel::ControlVector_type;
      using Measurement = typename MeasurementModel::Measurement_type;
    public:
      using KalmanFilter<ProcessModel, MeasurementModel>::Predict;
      using KalmanFilter<ProcessModel, MeasurementModel>::Update;

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
      template<bool EnableBool = true>
      static Belief Predict(const Belief& belief_posterior,
                            const ControlVector& ut,
                            double dt,
                            const std::enable_if_t<not ProcessModel::IsLinear() && EnableBool, ProcessModel>&
                                process_model)
      {
        // For linear process models the compiler will choose the corresponding function from the KalmanFilter class,
        // which is a base class for this one.

        // TODO: implement while adding CTRV process model
        throw std::runtime_error("NOT IMPLEMENTED YET");
      }

      /**
       * Update step of the Kalman filter. Incorporates the sensor measurement into the given prior belief.
       *
       * @param belief_prior a belief after the prediction Kalman filter step
       * @param measurement a measurement from the sensor
       * @param measurement_model an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template<bool EnableBool = true>
      static Belief Update(const Belief& belief_prior,
                           const Measurement& measurement,
                           const std::enable_if_t<not MeasurementModel::IsLinear() && EnableBool,
                           MeasurementModel>& measurement_model)
      {
        // For linear measurement models the compiler will choose the corresponding function from the KalmanFilter class,
        // which is a base class for this one.

        auto mu{belief_prior.mu()};
        auto Sigma{belief_prior.Sigma()};
        auto Ht{measurement_model.H(mu)};
        auto Kt{Sigma * Ht.transpose() * (Ht * Sigma * Ht.transpose() + measurement_model.Q()).inverse()};
        auto I{Eigen::Matrix<double, ProcessModel::StateDims(), ProcessModel::StateDims()>::Identity()};

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
