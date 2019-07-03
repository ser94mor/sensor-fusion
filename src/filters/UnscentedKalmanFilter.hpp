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

#ifndef SENSOR_FUSION_UNSCENTEDKALMANFILTER_HPP
#define SENSOR_FUSION_UNSCENTEDKALMANFILTER_HPP


#include "definitions.hpp"
#include "KalmanFilter.hpp"

#include <tuple>


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A template class holding unscented Kalman filter equations.
     * The naming of vectors and matrices are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     *
     * @tparam ProcessModel_t a class of the process model to use
     * @tparam MeasurementModel_t a class of measurement model to use; notice that here it is not a template class
     */
    template<class ProcessModel_t, class MeasurementModel_t>
    class UnscentedKalmanFilter : public KalmanFilterBase<ProcessModel_t, MeasurementModel_t, UnscentedKalmanFilter>
    {
    private:
      /**
       * @return a number of dimensions in the augmented state vector
       */
      constexpr static size_t AugmentedStateDims();

      /**
       * @param state_dims a number of dimensions in a state vector
       * @return a number of sigma points to generate for a state vector
       */
      constexpr static size_t SigmaPointsNumber(size_t state_dims);

      /**
       * @param state_dims a number of dimensions in a state vector
       * @return a sigma points spreading parameter (lambda) for augmented state vector
       */
      constexpr static double_t SigmaPointSpreadingParameter(size_t state_dims);

      /**
       * Some useful typedefs that are used in unscented Kalman filter methods.
       */
      using KF_type = KalmanFilter<ProcessModel_t, MeasurementModel_t>;

      using Belief_type = typename ProcessModel_t::Belief_type;
      using ControlVector_type = typename ProcessModel_t::ControlVector_type;
      using Measurement_type = typename MeasurementModel_t::Measurement_type;
      using MeasurementVector_type = typename MeasurementModel_t::MeasurementVector_type;
      using MeasurementCovarianceMatrix_type = typename MeasurementModel_t::MeasurementCovarianceMatrix_type;

      template <size_t state_dims>
      using StateVector = Eigen::Matrix<double_t, state_dims, 1>;
      using OrdinaryStateVector = StateVector<ProcessModel_t::StateDims()>;
      using AugmentedStateVector = StateVector<AugmentedStateDims()>;

      template <size_t state_dims>
      using StateCovarianceMatrix = Eigen::Matrix<double_t, state_dims, state_dims>;
      using OrdinaryStateCovarianceMatrix = StateCovarianceMatrix<ProcessModel_t::StateDims()>;
      using AugmentedStateCovarianceMatrix = StateCovarianceMatrix<AugmentedStateDims()>;

      template <size_t state_dims, size_t sigma_points_num>
      using SigmaPointsMatrix = Eigen::Matrix<double_t, state_dims, sigma_points_num>;
      using AugmentedSigmaPointsMatrix =
          SigmaPointsMatrix<AugmentedStateDims(), SigmaPointsNumber(AugmentedStateDims())>;
      using AugmentedPriorSigmaPointsMatrix =
          SigmaPointsMatrix<ProcessModel_t::StateDims(), SigmaPointsNumber(AugmentedStateDims())>;
      using OrdinarySigmaPointsMatrix =
          SigmaPointsMatrix<ProcessModel_t::StateDims(), SigmaPointsNumber(ProcessModel_t::StateDims())>;

      template <size_t sigma_points_num>
      using MeasurementSigmaPointsMatrix =
          SigmaPointsMatrix<MeasurementModel_t::MeasurementDims(), sigma_points_num>;
      using AugmentedMeasurementSigmaPointsMatrix =
          MeasurementSigmaPointsMatrix<SigmaPointsNumber(AugmentedStateDims())>;
      using OrdinaryMeasurementSigmaPointsMatrix =
          MeasurementSigmaPointsMatrix<SigmaPointsNumber(ProcessModel_t::StateDims())>;

      template <size_t sigma_points_num>
      using WeightsVector = Eigen::Matrix<double_t, sigma_points_num, 1>;
      using AugmentedWeightsVector = WeightsVector<SigmaPointsNumber(AugmentedStateDims())>;
      using OrdinaryWeightsVector = WeightsVector<SigmaPointsNumber(ProcessModel_t::StateDims())>;

      using CrossCorrelationMatrix =
          Eigen::Matrix<double_t, ProcessModel_t::StateDims(), MeasurementModel_t::MeasurementDims()>;

      /**
       * @tparam state_dims a number os dimentions in a state vector
       * @tparam sigma_points_num a number of sigma points
       * @return a sigma points weights
       */
      template <size_t state_dims, size_t sigma_points_num>
      static WeightsVector<sigma_points_num> Weights();

      /**
       * Generate a sigma points matrix based on the provided state vector and state covariance matrix.
       *
       * @tparam state_dims a number of state dimensions
       * @tparam sigma_points_num a number of sigma points to generate
       *
       * @param mu a state vector
       * @param Sigma a state covariance matrix
       *
       * @return a sigma points matrix
       */
      template <size_t state_dims, size_t sigma_points_num>
      static SigmaPointsMatrix<state_dims, sigma_points_num>
      GenerateSigmaPointsMatrix(const StateVector<state_dims>& mu, const StateCovarianceMatrix<state_dims>& Sigma);

      /**
       * Generate an augmented sigma points matrix based on the current belief.
       *
       * @param bel a current belief
       * @param pm an instance of process model
       * @return an augmented sigma points matrix
       */
      static AugmentedSigmaPointsMatrix
      GenerateAugmentedSigmaPointsMatrix(const Belief_type& bel, const ProcessModel_t& pm);

      /**
       * Predict step applied to each sigma point, that is, the non-linear transformation in accordance with the
       * process model is applied to each sigma point as if it were a state vector.
       *
       * @param bel a current belief
       * @param ut a control vector
       * @param dt a time interval between the previous and current measurements
       * @param pm an instance of the process model
       * @return a sigma points matrix having the same number of rows as in the ordinary state vector and
       *         the same number of columns as there are number of sigma points for augmented state vector
       */
      static AugmentedPriorSigmaPointsMatrix
      PredictSigmaPoints(const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm);

      /**
       * Apply non-linear measurement function to each sigma point.
       *
       * @tparam state_dims a number of dimensions in a state vector
       * @tparam sigma_points_num a number of sigma points
       *
       * @param Chi a matrix containing sigma points
       * @param mm an instance of measurement model
       *
       * @return a matrix where the number of rows equals to the number of dimension in a measurement vector and
       *         a number of columns is equal to the number of sigma points generated previously, that is,
       *         it is similar to the number of columns in the @param Chi matrix.
       */
      template <size_t state_dims, size_t sigma_points_num>
      static MeasurementSigmaPointsMatrix<sigma_points_num>
      ApplyMeasurementFunctionToEachSigmaPoint(const SigmaPointsMatrix<state_dims, sigma_points_num>& Chi,
                                               const MeasurementModel_t& mm);

      /**
       * Prediction step of the Unscented Kalman filter for non-linear process models.
       * Predicts the object's state in dt time in the future in accordance with NON-LINEAR process model
       * and input control vector.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt a time interval between the previous and current measurements
       * @param pm an instance of the process model
       *
       * @return a tuple containing a prior belief, that is, after prediction but before incorporating the measurement,
       *         a sigma points matrix, generated during execution of this method, and the weights vector
       */
      static std::tuple<Belief_type, AugmentedPriorSigmaPointsMatrix, AugmentedWeightsVector>
      PredictNonLinear(const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm);

      /**
       * Update step of the Unscented Kalman filter for the non-linear measurement models.
       * It incorporates the sensor measurement into the given prior belief.
       * Works only with NON-LINEAR measurement models.
       *
       * @param bel a belief after the prediction step of unscented Kalman filter
       * @param Chi a sigma points matrix
       * @param w a weights vector
       * @param meas a measurement from the sensor
       * @param mm an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template <size_t sigma_points_num>
      static Belief_type
      UpdateNonLinear(const Belief_type& bel,
                      const SigmaPointsMatrix<ProcessModel_t::StateDims(), sigma_points_num>& Chi,
                      const WeightsVector<sigma_points_num>& w,
                      const Measurement_type& meas,
                      const MeasurementModel_t& mm);

    public:
      /**
       * Prediction step of the Unscented Kalman filter for non-linear process model.
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
       * Prediction step of the Unscented Kalman filter for linear process model.
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
       * Update step of the Unscented Kalman filter for non-linear measurement model.
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
       * Update step of the Unscented Kalman filter for linear measurement model.
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

      /**
       * Combined Predict and Update steps of the Unscented Kalman filter.
       * Predicts the object's state in dt time in the future in accordance with the process model
       * and input control vector and then incorporates the sensor measurement into the belief.
       *
       * Notice that this method is a specific optimization of the unscented Kalman filter for the case, when
       * both process and measurement models are non-linear. In this case we can reuse the weights vector
       * and sigma points matrix generated during the Predict step in the Update step.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param meas a measurement from the sensor
       * @param pm an instance of the process model
       * @param mm an instance of the measurement model
       * @return a posterior belief, that is, after the prediction and incorporation of the measurement
       */
      template<bool enable = true>
      static auto
      PredictUpdate(const Belief_type& bel, const ControlVector_type& ut, const Measurement_type& meas,
                    const ProcessModel_t& pm, const MeasurementModel_t& mm)
      -> std::enable_if_t<!(ProcessModel_t::IsLinear() || MeasurementModel_t::IsLinear()) && enable, Belief_type>;

      /**
       * Combined Predict and Update steps of the derived Kalman filter.
       * Predicts the object's state in dt time in the future in accordance with the process model
       * and input control vector and then incorporates the sensor measurement into the belief.
       *
       * Notice that this method will be invoked in cases when either one or both process
       * and measurement models are linear.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param meas a measurement from the sensor
       * @param pm an instance of the process model
       * @param mm an instance of the measurement model
       * @return a posterior belief, that is, after the prediction and incorporation of the measurement
       */
      template<bool enable = true>
      static auto
      PredictUpdate(const Belief_type& bel, const ControlVector_type& ut, const Measurement_type& meas,
                    const ProcessModel_t& pm, const MeasurementModel_t& mm)
      -> std::enable_if_t<(ProcessModel_t::IsLinear() || MeasurementModel_t::IsLinear()) and enable, Belief_type>;
    };



    ////////////////////
    // IMPLEMENTATION //
    ////////////////////

    template<class ProcessModel_t, class MeasurementModel_t>
    constexpr size_t UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::AugmentedStateDims()
    {
      return ProcessModel_t::StateDims() + ProcessModel_t::ProcessNoiseDims();
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    constexpr size_t
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::SigmaPointsNumber(const size_t state_dims)
    {
      return 2 * state_dims + 1;
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    constexpr double_t
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::SigmaPointSpreadingParameter(const size_t state_dims)
    {
      // the value is chosen in accordance with a well known heuristics
      return 3.0 - static_cast<double_t>(state_dims);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<size_t state_dims, size_t sigma_points_num>
    typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::template WeightsVector<sigma_points_num>
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Weights()
    {
      const auto lambda{SigmaPointSpreadingParameter(state_dims)};
      WeightsVector<sigma_points_num> w{WeightsVector<sigma_points_num>::Constant(
          0.5 / (lambda + static_cast<double_t>(state_dims)))};
      w(0) = lambda / (lambda + static_cast<double_t>(state_dims));

      return w;
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<size_t state_dims, size_t sigma_points_num>
    typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::
      template SigmaPointsMatrix<state_dims, sigma_points_num>
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::GenerateSigmaPointsMatrix(
        const UnscentedKalmanFilter::StateVector<state_dims>& mu,
        const UnscentedKalmanFilter::StateCovarianceMatrix<state_dims>& Sigma)
    {
      constexpr auto lambda{SigmaPointSpreadingParameter(state_dims)};

      // calculate square root matrix out of an augmeneted state covariance matrix
      const StateCovarianceMatrix<state_dims> L{Sigma.llt().matrixL()};

      // initialize columns (i.e., sigma points) of the augmented sigma points matrix
      SigmaPointsMatrix<state_dims, sigma_points_num> Chi;
      Chi.col(0) = mu;
      for (size_t i = 0; i < state_dims; ++i)
      {
        const auto tmp{std::sqrt(lambda + static_cast<double_t>(state_dims)) * L.col(i)};
        Chi.col(i+1) = mu + tmp;
        Chi.col(i+1+state_dims) = mu - tmp;
      }

      return Chi;
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::AugmentedSigmaPointsMatrix
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::GenerateAugmentedSigmaPointsMatrix(
        const Belief_type& bel, const ProcessModel_t& pm)
    {
      constexpr auto aug_state_dims{AugmentedStateDims()};
      constexpr auto sigma_points_num{SigmaPointsNumber(aug_state_dims)};
      constexpr auto state_dims{ProcessModel_t::StateDims()};
      constexpr auto pn_dims{ProcessModel_t::ProcessNoiseDims()};

      // an augmented state vector
      AugmentedStateVector mu_aug{AugmentedStateVector::Zero()};
      mu_aug.head(state_dims) = bel.mu();

      // an augmented state covariance matrix
      AugmentedStateCovarianceMatrix Sigma_aug{AugmentedStateCovarianceMatrix::Zero()};
      Sigma_aug.topLeftCorner(state_dims, state_dims) = bel.Sigma();
      Sigma_aug.bottomRightCorner(pn_dims, pn_dims) = pm.GetProcessNoiseCovarianceMatrix();

      return GenerateSigmaPointsMatrix<aug_state_dims, sigma_points_num>(mu_aug, Sigma_aug);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::AugmentedPriorSigmaPointsMatrix
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::PredictSigmaPoints(const Belief_type& bel,
                                                                                  const ControlVector_type& ut,
                                                                                  double_t dt, const ProcessModel_t& pm)
    {
      constexpr auto sigma_points_num{SigmaPointsNumber(AugmentedStateDims())};

      const AugmentedSigmaPointsMatrix Chi_aug{GenerateAugmentedSigmaPointsMatrix(bel, pm)};

      // predict sigma points
      AugmentedPriorSigmaPointsMatrix Chi;
      for (size_t i = 0; i < sigma_points_num; ++i)
      {
        Chi.col(i) = pm.g(dt, ut, Chi_aug.col(i).head(5), Chi_aug.col(i).tail(2));
      }

      return Chi;
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<size_t state_dims, size_t sigma_points_num>
    typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::
      template MeasurementSigmaPointsMatrix<sigma_points_num>
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::ApplyMeasurementFunctionToEachSigmaPoint(
        const UnscentedKalmanFilter::SigmaPointsMatrix<state_dims, sigma_points_num>& Chi, const MeasurementModel_t& mm)
    {
      MeasurementSigmaPointsMatrix<sigma_points_num> Zeta;
      for (size_t i = 0; i < sigma_points_num; ++i)
      {
        Zeta.col(i) = mm.h(Chi.col(i));
      }
      return Zeta;
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    std::tuple<typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Belief_type,
               typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::AugmentedPriorSigmaPointsMatrix,
               typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::AugmentedWeightsVector>
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::PredictNonLinear(const Belief_type& bel,
                                                                                const ControlVector_type& ut,
                                                                                double_t dt, const ProcessModel_t& pm)
    {
      constexpr auto aug_state_dims{AugmentedStateDims()};
      constexpr auto sigma_points_num{SigmaPointsNumber(aug_state_dims)};

      const AugmentedWeightsVector w{Weights<aug_state_dims, sigma_points_num>()};

      const AugmentedPriorSigmaPointsMatrix Chi{PredictSigmaPoints(bel, ut, dt, pm)};

      // predict state vector
      OrdinaryStateVector mu_prior{OrdinaryStateVector::Zero()};
      for (size_t i = 0; i < sigma_points_num; ++i)
      {
        mu_prior += w(i) * Chi.col(i);
      }

      // Predict the state covariance matrix.
      // Notice that the process noise is accounted in the generation of the augmented sigma points,
      // since it has a non-linear effect. This is different from the formula for the UKF presented in
      // "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
      // where it is suggested to add R process covariance matrix in this step. Here we use different approach
      // due to the specifics of some non-linear process models, such as CTRV.
      OrdinaryStateCovarianceMatrix Sigma_prior{OrdinaryStateCovarianceMatrix::Zero()};
      for (size_t i = 0; i < sigma_points_num; ++i)
      {
        const OrdinaryStateVector sigma_point{Chi.col(i)};
        const auto diff{ProcessModel_t::Subtract(sigma_point, mu_prior)};
        Sigma_prior += w(i) * diff * diff.transpose();
      }

      return std::make_tuple(Belief_type{bel.t() + dt, mu_prior, Sigma_prior}, Chi, w);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<size_t sigma_points_num>
    typename UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Belief_type
    UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::UpdateNonLinear(
        const Belief_type& bel,
        const UnscentedKalmanFilter::SigmaPointsMatrix<ProcessModel_t::StateDims(), sigma_points_num>& Chi,
        const UnscentedKalmanFilter::WeightsVector<sigma_points_num>& w,
        const Measurement_type& meas,
        const MeasurementModel_t& mm)
    {
      constexpr auto state_dims{ProcessModel_t::StateDims()};

      const OrdinaryStateVector mu{bel.mu()};

      const MeasurementSigmaPointsMatrix<sigma_points_num>
          Zeta{ApplyMeasurementFunctionToEachSigmaPoint<state_dims, sigma_points_num>(Chi, mm)};

      MeasurementVector_type z{MeasurementVector_type::Zero()};
      for (size_t i=0; i < sigma_points_num; ++i)
      {
        z += w(i) * Zeta.col(i);
      }

      // here the measurement noise is additive
      MeasurementCovarianceMatrix_type S{mm.Q()};
      CrossCorrelationMatrix Sigma_x_z{CrossCorrelationMatrix::Zero()};
      for (size_t i = 0; i < sigma_points_num; ++i)
      {
        const auto z_diff{MeasurementModel_t::Diff(Zeta.col(i), z)};
        S += w(i) * z_diff * z_diff.transpose();

        const OrdinaryStateVector sigma_point{Chi.col(i)};
        const OrdinaryStateVector mu_diff{ProcessModel_t::Subtract(sigma_point, mu)};

        Sigma_x_z += w(i) * mu_diff * z_diff.transpose();
      }

      const auto Kt{Sigma_x_z * S.inverse()};

      return {
          /* timestamp */               meas.t(),
          /* state vector */            mu + Kt * MeasurementModel_t::Diff(meas.z(), z),
          /* state covariance matrix */ bel.Sigma() - Kt * S * Kt.transpose(),
      };
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Predict(
        const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm)
    -> std::enable_if_t<ProcessModel_t::IsLinear() && enable, Belief_type>
    {
      return KF_type::Predict(bel, ut, dt, pm);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Update(
        const Belief_type& bel, const Measurement_type& meas, const MeasurementModel_t& mm)
    -> std::enable_if_t<MeasurementModel_t::IsLinear() && enable, Belief_type>
    {
      return KF_type::Update(bel, meas, mm);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::PredictUpdate(const Belief_type& bel,
                                                                                  const ControlVector_type& ut,
                                                                                  const Measurement_type& meas,
                                                                                  const ProcessModel_t& pm,
                                                                                  const MeasurementModel_t& mm)
    -> std::enable_if_t<!(ProcessModel_t::IsLinear() || MeasurementModel_t::IsLinear()) && enable, Belief_type>
    {
      constexpr auto sigma_points_num{SigmaPointsNumber(AugmentedStateDims())};
      const auto dt = meas.t() - bel.t();
      const auto bel_Chi_w{PredictNonLinear(bel, ut, dt, pm)};
      return UpdateNonLinear<sigma_points_num>(
          std::get<0>(bel_Chi_w), std::get<1>(bel_Chi_w), std::get<2>(bel_Chi_w), meas, mm);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::PredictUpdate(const Belief_type& bel,
                                                                                  const ControlVector_type& ut,
                                                                                  const Measurement_type& meas,
                                                                                  const ProcessModel_t& pm,
                                                                                  const MeasurementModel_t& mm)
    -> std::enable_if_t<(ProcessModel_t::IsLinear() || MeasurementModel_t::IsLinear()) and enable, Belief_type>
    {
      return KalmanFilterBase<ProcessModel_t, MeasurementModel_t, UnscentedKalmanFilter>::
      PredictUpdate(bel, ut, meas, pm, mm);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Update(
        const Belief_type& bel, const Measurement_type& meas, const MeasurementModel_t& mm)
    -> std::enable_if_t<!MeasurementModel_t::IsLinear() && enable, Belief_type>
    {
      constexpr auto state_dims{ProcessModel_t::StateDims()};
      constexpr auto sigma_points_num{SigmaPointsNumber(state_dims)};

      const OrdinaryWeightsVector w{Weights<state_dims, sigma_points_num>()};
      const OrdinarySigmaPointsMatrix
          Chi{GenerateSigmaPointsMatrix<state_dims, sigma_points_num>(bel.mu(), bel.Sigma())};

      return UpdateNonLinear<sigma_points_num>(bel, Chi, w, meas, mm);
    }

    template<class ProcessModel_t, class MeasurementModel_t>
    template<bool enable>
    auto UnscentedKalmanFilter<ProcessModel_t, MeasurementModel_t>::Predict(
        const Belief_type& bel, const ControlVector_type& ut, double_t dt, const ProcessModel_t& pm)
    -> std::enable_if_t<!ProcessModel_t::IsLinear() && enable, Belief_type>
    {
      return std::get<0>(PredictNonLinear(bel, ut, dt, pm));
    }

  }
}


#endif //SENSOR_FUSION_UNSCENTEDKALMANFILTER_HPP
