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

#ifndef SENSOR_FUSION_KALMANFILTER_HPP
#define SENSOR_FUSION_KALMANFILTER_HPP


#include "beliefs.hpp"
#include "measurements.hpp"
#include "measurement_models.hpp"
#include "process_models.hpp"

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
     * @tparam ProcessModel_t a class of the process model to use
     * @tparam MeasurementModel_t a class of measurement model to use; notice that here it is not a template class
     * @tparam DerivedKalmanFilterTemplate_t a concrete sub-template-class of the KalmanFilterBase;
     *                                       it is needed to invoke methods from that class
     */
    template<class ProcessModel_t, class MeasurementModel_t, template<class, class> class DerivedKalmanFilterTemplate_t>
    class KalmanFilterBase
    {
    private:
      using Belief_type = typename ProcessModel_t::Belief_type;
      using ControlVector_type = typename ProcessModel_t::ControlVector_type;
      using Measurement_type = typename MeasurementModel_t::Measurement_type;
    public:

      /**
       * Prediction step of the Kalman filter. Predicts the object's state in dt time in the future in accordance with
       * LINEAR process model and input control vector.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt time interval between the previous and current measurements
       * @param pm an instance of the process model
       *
       * @return a prior belief, that is, after prediction but before incorporating the measurement
       */
      template <bool enable = true>
      static auto
      Predict(const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm)
      -> std::enable_if_t<ProcessModel_t::IsLinear() && enable, Belief_type>;

      /**
       * Update step of the Kalman filter. Incorporates the sensor measurement into the given prior belief.
       * Works only with linear measurement models.
       *
       * @param bel a belief after the prediction Kalman filter step
       * @param meas a measurement from the sensor
       * @param mm an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template <bool enable = true>
      static auto
      Update(const Belief_type& bel, const Measurement_type& meas, const MeasurementModel_t& mm)
      -> std::enable_if_t<MeasurementModel_t::IsLinear() && enable, Belief_type>;

      /**
       * Combined Predict and Update steps of the derived Kalman filter.
       * Predicts the object's state in dt time in the future in accordance with the process model
       * and input control vector and then incorporates the sensor measurement into the belief.
       *
       * Notice that process model and measurement model can be either linear and non-linear.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param meas a measurement from the sensor
       * @param pm an instance of the process model
       * @param mm an instance of the measurement model
       * @return a posterior belief, that is, after the prediction and incorporation of the measurement
       */
      static auto
      PredictUpdate(const Belief_type& bel, const ControlVector_type& ut, const Measurement_type& meas,
                    const ProcessModel_t& pm, const MeasurementModel_t& mm)
      -> Belief_type;
    };


    /**
     * A concrete class representing Kalman filter. All the equations are the same as in its base class.
     *
     * @tparam ProcessModel_t a class of the process model to use
     * @tparam MeasurementModel_t a class of measurement model to use; notice that here it is not a template class
     */
    template <class ProcessModel_t, class MeasurementModel_t>
    class KalmanFilter : public KalmanFilterBase<ProcessModel_t, MeasurementModel_t, KalmanFilter>
    {

    };



    ////////////////////
    // IMPLEMENTATION //
    ////////////////////


    template<class ProcessModel_t, class MeasurementModel_t, template<class, class> class DerivedKalmanFilterTemplate_t>
    template<bool enable>
    auto KalmanFilterBase<ProcessModel_t, MeasurementModel_t, DerivedKalmanFilterTemplate_t>::Predict(
        const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm)
    -> std::enable_if_t<ProcessModel_t::IsLinear() && enable, Belief_type>
    {
      auto At{pm.A(dt)};
      return {
          /* timestamp */               bel.t() + dt,
          /* state vector */            At * bel.mu() + pm.B() * ut,
          /* state covariance matrix */ At * bel.Sigma() * At.transpose() + pm.R(dt),
      };
    }

    template<class ProcessModel_t, class MeasurementModel_t, template<class, class> class DerivedKalmanFilterTemplate_t>
    template<bool enable>
    auto KalmanFilterBase<ProcessModel_t, MeasurementModel_t, DerivedKalmanFilterTemplate_t>::Update(
        const Belief_type& bel, const Measurement_type& meas, const MeasurementModel_t& mm)
    -> std::enable_if_t<MeasurementModel_t::IsLinear() && enable, Belief_type>
    {
      const auto Ct{mm.C()};
      const auto mu{bel.mu()};
      const auto Sigma{bel.Sigma()};
      const auto Kt{Sigma * Ct.transpose() * (Ct * Sigma * Ct.transpose() + mm.Q()).inverse()};
      const auto I{Eigen::Matrix<double_t, ProcessModel_t::StateDims(), ProcessModel_t::StateDims()>::Identity()};

      return {
          /* timestamp */               meas.t(),
          /* state vector */            mu + Kt * (meas.z() - Ct * mu),
          /* state covariance matrix */ (I - Kt * Ct) * Sigma,
      };
    }

    template<class ProcessModel_t, class MeasurementModel_t, template<class, class> class DerivedKalmanFilterTemplate_t>
    auto
    KalmanFilterBase<ProcessModel_t, MeasurementModel_t, DerivedKalmanFilterTemplate_t>::PredictUpdate(
        const Belief_type& bel, const ControlVector_type& ut, const Measurement_type& meas, const ProcessModel_t& pm,
        const MeasurementModel_t& mm)
    -> Belief_type
    {
      const auto dt = meas.t() - bel.t();

      const auto
      bel_prior{DerivedKalmanFilterTemplate_t<ProcessModel_t, MeasurementModel_t>::Predict(bel, ut, dt, pm)};

      return DerivedKalmanFilterTemplate_t<ProcessModel_t, MeasurementModel_t>::Update(bel_prior, meas, mm);
    }

  }
}


#endif //SENSOR_FUSION_KALMANFILTER_HPP
