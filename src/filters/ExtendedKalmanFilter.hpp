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

#ifndef SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP
#define SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP


#include "KalmanFilter.hpp"
#include "process_models.hpp"
#include "measurement_models.hpp"
#include "beliefs.hpp"
#include "measurements.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A template class holding extended Kalman filter equations.
     * The naming of vectors and matrices are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     *
     * @tparam ProcessModel_t a class of the process model to use
     * @tparam MeasurementModel_t a class of measurement model to use; notice that here it is not a template class
     */
    template<class ProcessModel_t, class MeasurementModel_t>
    class ExtendedKalmanFilter : public KalmanFilterBase<ProcessModel_t, MeasurementModel_t, ExtendedKalmanFilter>
    {
    private:
      using KF_type = KalmanFilter<ProcessModel_t, MeasurementModel_t>;
      using Belief_type = typename ProcessModel_t::Belief_type;
      using ControlVector_type = typename ProcessModel_t::ControlVector_type;
      using Measurement_type = typename MeasurementModel_t::Measurement_type;

    public:
      /**
       * Prediction step of the Extended Kalman filter for non-linear process model.
       * Predicts the object's state in dt time in the future
       * in accordance with NON-LINEAR process model and input control vector.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt time interval between the previous and current measurements
       * @param pm an instance of the process model
       *
       * @return a prior belief, that is, after prediction but before incorporating the measurement
       */
      template<bool enable = true>
      static auto
      Predict(const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm)
      -> std::enable_if_t<!ProcessModel_t::IsLinear() && enable, Belief_type>;

      /**
       * Prediction step of the Extended Kalman filter for linear process model.
       * Predicts the object's state in dt time in the future
       * in accordance with LINEAR process model and input control vector.
       *
       * Notice that we use here a simple Kalman filter's Predict equations.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt time interval between the previous and current measurements
       * @param pm an instance of the process model
       *
       * @return a prior belief, that is, after prediction but before incorporating the measurement
       */
      template<bool enable = true>
      static auto
      Predict(const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm)
      -> std::enable_if_t<ProcessModel_t::IsLinear() && enable, Belief_type>;

      /**
       * Update step of the Extended Kalman filter for non-linear measurement model.
       * Incorporates the sensor measurement into the given prior belief.
       * Works only with NON-LINEAR measurement models.
       *
       * @param bel a belief after the prediction Extended Kalman filter step
       * @param meas a measurement from the sensor
       * @param mm an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template<bool enable = true>
      static auto
      Update(const Belief_type& bel, const Measurement_type& meas, const MeasurementModel_t& mm)
      -> std::enable_if_t<!MeasurementModel_t::IsLinear() && enable, Belief_type>;

      /**
       * Update step of the Extended Kalman filter for linear measurement model.
       * Incorporates the sensor measurement into the given prior belief.
       * Works only with LINEAR measurement models.
       *
       * Notice that we use here a simple Kalman filter's Update equations.
       *
       * @param bel a belief after the prediction Extended Kalman filter step
       * @param meas a measurement from the sensor
       * @param mm an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template<bool enable = true>
      static auto
      Update(const Belief_type& bel, const Measurement_type& meas, const MeasurementModel_t& mm)
      -> std::enable_if_t<MeasurementModel_t::IsLinear() && enable, Belief_type>;

    };


    ////////////////////
    // IMPLEMENTATION //
    ////////////////////

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto ExtendedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Predict(const Belief_type& bel,
                                                                           const ControlVector_type& ut, double_t dt,
                                                                           const ProcessModel_t& pm)
    -> std::enable_if_t<!ProcessModel_t::IsLinear() && enable, Belief_type>
    {
      const auto mu{bel.mu()};
      const auto Gt{pm.G(dt, mu)};
      return {
          /* timestamp */               bel.t() + dt,
          /* state vector */            pm.g(dt, ut, mu),
          /* state covariance matrix */ Gt * bel.Sigma() * Gt.transpose() + pm.R(dt, mu),
      };
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto ExtendedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Update(const Belief_type& bel,
                                                                          const Measurement_type& meas,
                                                                          const MeasurementModel_t& mm)
    -> std::enable_if_t<!MeasurementModel_t::IsLinear() && enable, Belief_type>
    {
      const auto mu{bel.mu()};
      const auto Sigma{bel.Sigma()};
      const auto Ht{mm.H(mu)};
      const auto Kt{Sigma * Ht.transpose() * (Ht * Sigma * Ht.transpose() + mm.Q()).inverse()};
      const auto I{
          Eigen::Matrix<double_t, ProcessModel_t::StateDims(), ProcessModel_t::StateDims()>::Identity()};

      return {
          /* timestamp */               meas.t(),
          /* state vector */            mu + Kt * MeasurementModel_t::Diff(meas.z(), mm.h(mu)),
          /* state covariance matrix */ (I - Kt * Ht) * Sigma,
      };
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto ExtendedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Update(const Belief_type& bel,
                                                                          const Measurement_type& meas,
                                                                          const MeasurementModel_t& mm)
    -> std::enable_if_t<MeasurementModel_t::IsLinear() && enable, Belief_type>
    {
      return KF_type::Update(bel, meas, mm);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto ExtendedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Predict(const Belief_type& bel,
                                                                           const ControlVector_type& ut, double_t dt,
                                                                           const ProcessModel_t& pm)
    -> std::enable_if_t<ProcessModel_t::IsLinear() && enable, Belief_type>
    {
      return KF_type::Predict(bel, ut, dt, pm);
    }

  }
}


#endif //SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP
