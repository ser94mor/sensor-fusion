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
#include <tuple>
#include <Eigen/Dense>


namespace ser94mor
{
  namespace  sensor_fusion
  {

    /**
     * A base template class holding Kalman filter equations.
     * The naming of vectors and matrices are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     *
     * @tparam ProcessModel a class of the process model to use
     * @tparam MeasurementModel a class of measurement model to use; notice that here it is not a template class
     * @tparam DerivedKalmanFilter a concrete sub-template-class of the KalmanFilterBase;
     *                             it is needed to invoke methods from that class
     */
    template<class ProcessModel, class MeasurementModel, template<class, class> class DerivedKalmanFilter>
    class KalmanFilterBase
    {
    protected:
      using Belief = typename ProcessModel::Belief_type;
      using ControlVector = typename ProcessModel::ControlVector_type;
      using Measurement = typename MeasurementModel::Measurement_type;
    public:

      /**
       * Prediction step of the Kalman filter. Predicts the object's state in dt time in the future in accordance with
       * LINEAR process model and input control vector.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt time interval between the previous and current measurements
       * @param process_model an instance of the process model
       *
       * @return a prior belief, that is, after prediction but before incorporating the measurement
       */
      template <bool enable = true>
      static auto
      Predict(const Belief& bel, const ControlVector& ut, double_t dt, const ProcessModel& process_model)
      -> std::enable_if_t<ProcessModel::IsLinear() and enable, Belief>
      {
        auto At{process_model.A(dt)};
        return {
            /* timestamp */               bel.t() + dt,
            /* state vector */            At * bel.mu() + process_model.B() * ut,
            /* state covariance matrix */ At * bel.Sigma() * At.transpose() + process_model.R(dt),
        };
      }

      /**
       * Update step of the Kalman filter. Incorporates the sensor measurement into the given prior belief.
       * Works only with linear measurement models.
       *
       * @param bel a belief after the prediction Kalman filter step
       * @param measurement a measurement from the sensor
       * @param measurement_model an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template <bool enable = true>
      static auto
      Update(const Belief& bel, const Measurement& measurement, const MeasurementModel& measurement_model)
      -> std::enable_if_t<MeasurementModel::IsLinear() and enable, Belief>
      {
        auto Ct{measurement_model.C()};
        auto mu{bel.mu()};
        auto Sigma{bel.Sigma()};
        auto Kt{Sigma * Ct.transpose() * (Ct * Sigma * Ct.transpose() + measurement_model.Q()).inverse()};
        auto I{Eigen::Matrix<double_t, ProcessModel::StateDims(), ProcessModel::StateDims()>::Identity()};

        return {
            /* timestamp */               measurement.t(),
            /* state vector */            mu + Kt * (measurement.z() - Ct * mu),
            /* state covariance matrix */ (I - Kt * Ct) * Sigma,
        };
      }

      /**
       * Combined Predict and Update steps of the derived Kalman filter.
       * Predicts the object's state in dt time in the future in accordance with the process model
       * and input control vector and then incorporates the sensor measurement into the belief.
       *
       * Notice that process model and measurement model can be either linear and non-linear.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param measurement a measurement from the sensor
       * @param process_model an instance of the process model
       * @param measurement_model an instance of the measurement model
       * @return a posterior belief, that is, after the prediction and incorporation of the measurement
       */
      static Belief
      PredictUpdate(const Belief& bel, const ControlVector& ut, const Measurement& measurement,
                    const ProcessModel& process_model, const MeasurementModel& measurement_model)
      {
        auto dt = measurement.t() - bel.t();

        auto bel_prior{DerivedKalmanFilter<ProcessModel, MeasurementModel>::Predict(bel, ut, dt, process_model)};

        return DerivedKalmanFilter<ProcessModel, MeasurementModel>::Update(bel_prior, measurement, measurement_model);
      }
    };


    /**
     * A concrete class representing Kalman filter. All the equations are the same as in its base class.
     *
     * @tparam ProcessModel a class of the process model to use
     * @tparam MeasurementModel a class of measurement model to use; notice that here it is not a template class
     */
    template <class ProcessModel, class MeasurementModel>
    class KalmanFilter : public KalmanFilterBase<ProcessModel, MeasurementModel, KalmanFilter>
    {

    };

  }
}

#endif //SENSOR_FUSION_KALMANFILTER_HPP
