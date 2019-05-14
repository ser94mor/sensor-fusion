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
     * @tparam ProcessModel a class of the process model to use
     * @tparam MeasurementModel a class of measurement model to use; notice that here it is not a template class
     */
    template<class ProcessModel, class MeasurementModel>
    class UnscentedKalmanFilter : public KalmanFilterBase<ProcessModel, MeasurementModel, UnscentedKalmanFilter>
    {
    private:
      /**
       * @return a number of dimensions in the augmented state vector
       */
      constexpr static int AugmentedStateDims()
      {
        return ProcessModel::StateDims() + ProcessModel::ProcessNoiseDims();
      }

      /**
       * @param state_dims a number of dimensions in a state vector
       * @return a number of sigma points to generate for a state vector
       */
      constexpr static int SigmaPointsNumber(int state_dims)
      {
        return 2 * state_dims + 1;
      }

      /**
       * @param state_dims a number of dimensions in a state vector
       * @return a sigma points spreading parameter (lambda) for augmented state vector
       */
      constexpr static double_t SigmaPointSpreadingParameter(int state_dims)
      {
        // the value is chosen in accordance with a well known heuristics
        return 3.0 - state_dims;
      }

      /**
       * Some useful typedefs that are used in unscented Kalman filter methods.
       */
      using Belief = typename ProcessModel::Belief_type;
      using ControlVector = typename ProcessModel::ControlVector_type;
      using Measurement = typename MeasurementModel::Measurement_type;
      using MeasurementVector = typename MeasurementModel::MeasurementVector_type;
      using MeasurementCovarianceMatrix = typename MeasurementModel::MeasurementCovarianceMatrix_type;

      template <int state_dims>
      using StateVector = Eigen::Matrix<double_t, state_dims, 1>;
      using OrdinaryStateVector = StateVector<ProcessModel::StateDims()>;
      using AugmentedStateVector = StateVector<AugmentedStateDims()>;

      template <int state_dims>
      using StateCovarianceMatrix = Eigen::Matrix<double_t, state_dims, state_dims>;
      using OrdinaryStateCovarianceMatrix = StateCovarianceMatrix<ProcessModel::StateDims()>;
      using AugmentedStateCovarianceMatrix = StateCovarianceMatrix<AugmentedStateDims()>;

      template <int state_dims, int sigma_points_num>
      using SigmaPointsMatrix = Eigen::Matrix<double_t, state_dims, sigma_points_num>;
      using AugmentedSigmaPointsMatrix =
          SigmaPointsMatrix<AugmentedStateDims(), SigmaPointsNumber(AugmentedStateDims())>;
      using AugmentedPriorSigmaPointsMatrix =
          SigmaPointsMatrix<ProcessModel::StateDims(), SigmaPointsNumber(AugmentedStateDims())>;
      using OrdinarySigmaPointsMatrix =
          SigmaPointsMatrix<ProcessModel::StateDims(), SigmaPointsNumber(ProcessModel::StateDims())>;

      template <int sigma_points_num>
      using MeasurementSigmaPointsMatrix = SigmaPointsMatrix<MeasurementModel::MeasurementDims(), sigma_points_num>;
      using AugmentedMeasurementSigmaPointsMatrix =
          MeasurementSigmaPointsMatrix<SigmaPointsNumber(AugmentedStateDims())>;
      using OrdinaryMeasurementSigmaPointsMatrix =
          MeasurementSigmaPointsMatrix<SigmaPointsNumber(ProcessModel::StateDims())>;

      template <int sigma_points_num>
      using WeightsVector = Eigen::Matrix<double_t, sigma_points_num, 1>;
      using AugmentedWeightsVector = WeightsVector<SigmaPointsNumber(AugmentedStateDims())>;
      using OrdinaryWeightsVector = WeightsVector<SigmaPointsNumber(ProcessModel::StateDims())>;

      using CrossCorrelationMatrix =
          Eigen::Matrix<double_t, ProcessModel::StateDims(), MeasurementModel::MeasurementDims()>;

      /**
       * @tparam state_dims a number os dimentions in a state vector
       * @tparam sigma_points_num a number of sigma points
       * @return a sigma points weights
       */
      template <int state_dims, int sigma_points_num>
      static WeightsVector<sigma_points_num> Weights()
      {
        double_t lambda{SigmaPointSpreadingParameter(state_dims)};
        WeightsVector<sigma_points_num> w{WeightsVector<sigma_points_num>::Constant(0.5 / (lambda + state_dims))};
        w(0) = lambda / (lambda + state_dims);

        return w;
      }

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
      template <int state_dims, int sigma_points_num>
      static SigmaPointsMatrix<state_dims, sigma_points_num>
      GenerateSigmaPointsMatrix(const StateVector<state_dims>& mu, const StateCovarianceMatrix<state_dims>& Sigma)
      {
        constexpr double_t lambda{SigmaPointSpreadingParameter(state_dims)};

        // calculate square root matrix out of an augmeneted state covariance matrix
        StateCovarianceMatrix<state_dims> L{Sigma.llt().matrixL()};

        // initialize columns (i.e., sigma points) of the augmented sigma points matrix
        SigmaPointsMatrix<state_dims, sigma_points_num> Chi;
        Chi.col(0) = mu;
        for (int i = 0; i < state_dims; ++i)
        {
          auto tmp{std::sqrt(lambda + state_dims) * L.col(i)};
          Chi.col(i+1) = mu + tmp;
          Chi.col(i+1+state_dims) = mu - tmp;
        }

        return Chi;
      }

      /**
       * Generate an augmented sigma points matrix based on the current belief.
       *
       * @param bel a current belief
       * @param process_model an instance of process model
       * @return an augmented sigma points matrix
       */
      static AugmentedSigmaPointsMatrix
      GenerateAugmentedSigmaPointsMatrix(const Belief& bel, const ProcessModel& process_model)
      {
        constexpr int aug_state_dims{AugmentedStateDims()};
        constexpr int sigma_points_num{SigmaPointsNumber(aug_state_dims)};
        constexpr int state_dims{ProcessModel::StateDims()};
        constexpr int pn_dims{ProcessModel::ProcessNoiseDims()};

        // an augmented state vector
        AugmentedStateVector mu_aug{AugmentedStateVector::Zero()};
        mu_aug.head(state_dims) = bel.mu();

        // an augmented state covariance matrix
        AugmentedStateCovarianceMatrix Sigma_aug{AugmentedStateCovarianceMatrix::Zero()};
        Sigma_aug.topLeftCorner(state_dims, state_dims) = bel.Sigma();
        Sigma_aug.bottomRightCorner(pn_dims, pn_dims) = process_model.GetProcessNoiseCovarianceMatrix();

        return GenerateSigmaPointsMatrix<aug_state_dims, sigma_points_num>(mu_aug, Sigma_aug);
      }

      /**
       * Predict step applied to each sigma point, that is, the non-linear transformation in accordance with the
       * process model is applied to each sigma point as if it were a state vector.
       *
       * @param bel a current belief
       * @param ut a control vector
       * @param dt a time interval between the previous and current measurements
       * @param process_model an instance of the process model
       * @return a sigma points matrix having the same number of rows as in the ordinary state vector and
       *         the same number of columns as there are number of sigma points for augmented state vector
       */
      static AugmentedPriorSigmaPointsMatrix
      PredictSigmaPoints(const Belief& bel, const ControlVector& ut, double_t dt, const ProcessModel& process_model)
      {
        constexpr int sigma_points_num{SigmaPointsNumber(AugmentedStateDims())};

        AugmentedSigmaPointsMatrix Chi_aug{GenerateAugmentedSigmaPointsMatrix(bel, process_model)};

        // predict sigma points
        AugmentedPriorSigmaPointsMatrix Chi;
        for (int i = 0; i < sigma_points_num; ++i)
        {
          Chi.col(i) = process_model.g(dt, ut, Chi_aug.col(i).head(5), Chi_aug.col(i).tail(2));
        }

        return Chi;
      }

      /**
       * Apply non-linear measurement function to each sigma point.
       *
       * @tparam state_dims a number of dimensions in a state vector
       * @tparam sigma_points_num a number of sigma points
       *
       * @param Chi a matrix containing sigma points
       * @param measurement_model an instance of measurement model
       *
       * @return a matrix where the number of rows equals to the number of dimension in a measurement vector and
       *         a number of columns is equal to the number of sigma points generated previously, that is,
       *         it is similar to the number of columns in the @param Chi matrix.
       */
      template <int state_dims, int sigma_points_num>
      static MeasurementSigmaPointsMatrix<sigma_points_num>
      ApplyMeasurementFunctionToEachSigmaPoint(const SigmaPointsMatrix<state_dims, sigma_points_num>& Chi,
                                               const MeasurementModel& measurement_model)
      {
        MeasurementSigmaPointsMatrix<sigma_points_num> Zeta;
        for (int i = 0; i < sigma_points_num; ++i)
        {
          Zeta.col(i) = measurement_model.h(Chi.col(i));
        }
        return Zeta;
      }

      /**
       * Prediction step of the Unscented Kalman filter for non-linear process models.
       * Predicts the object's state in dt time in the future in accordance with NON-LINEAR process model
       * and input control vector.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt a time interval between the previous and current measurements
       * @param process_model an instance of the process model
       *
       * @return a tuple containing a prior belief, that is, after prediction but before incorporating the measurement,
       *         a sigma points matrix, generated during execution of this method, and the weights vector
       */
      static std::tuple<Belief, AugmentedPriorSigmaPointsMatrix, AugmentedWeightsVector>
      PredictNonLinear(const Belief& bel, const ControlVector& ut, double_t dt, const ProcessModel& process_model)
      {
        constexpr int aug_state_dims{AugmentedStateDims()};
        constexpr int sigma_points_num{SigmaPointsNumber(aug_state_dims)};

        AugmentedWeightsVector w{Weights<aug_state_dims, sigma_points_num>()};

        AugmentedPriorSigmaPointsMatrix Chi{PredictSigmaPoints(bel, ut, dt, process_model)};

        // predict state vector
        OrdinaryStateVector mu_prior{OrdinaryStateVector::Zero()};
        for (int i = 0; i < sigma_points_num; ++i)
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
        for (int i = 0; i < sigma_points_num; ++i)
        {
          OrdinaryStateVector sigma_point{Chi.col(i)};
          auto diff{process_model.Subtract(sigma_point, mu_prior)};
          Sigma_prior += w(i) * diff * diff.transpose();
        }

        return std::make_tuple(Belief{bel.t() + dt, mu_prior, Sigma_prior}, Chi, w);
      }

      /**
       * Update step of the Unscented Kalman filter for the non-linear measurement models.
       * It incorporates the sensor measurement into the given prior belief.
       * Works only with NON-LINEAR measurement models.
       *
       * @param bel a belief after the prediction step of unscented Kalman filter
       * @param Chi a sigma points matrix
       * @param w a weights vector
       * @param measurement a measurement from the sensor
       * @param measurement_model an instance of the measurement model
       *
       * @return a posterior belief, that is, after the incorporation of the measurement
       */
      template <int sigma_points_num>
      static Belief
      UpdateNonLinear(const Belief& bel,
                      const SigmaPointsMatrix<ProcessModel::StateDims(), sigma_points_num>& Chi,
                      const WeightsVector<sigma_points_num>& w,
                      const Measurement& measurement,
                      const MeasurementModel& measurement_model)
      {
        constexpr auto state_dims{ProcessModel::StateDims()};

        OrdinaryStateVector mu{bel.mu()};
        OrdinaryStateCovarianceMatrix Sigma{bel.Sigma()};

        MeasurementSigmaPointsMatrix<sigma_points_num>
            Zeta{ApplyMeasurementFunctionToEachSigmaPoint<state_dims, sigma_points_num>(Chi, measurement_model)};

        MeasurementVector z{MeasurementVector::Zero()};
        for (int i=0; i < sigma_points_num; ++i)
        {
          z += w(i) * Zeta.col(i);
        }

        // here the measurement noise is additive
        MeasurementCovarianceMatrix S{measurement_model.Q()};
        CrossCorrelationMatrix Sigma_x_z{CrossCorrelationMatrix::Zero()};
        for (int i = 0; i < sigma_points_num; ++i)
        {
          auto z_diff{measurement_model.Diff(Zeta.col(i), z)};
          S += w(i) * z_diff * z_diff.transpose();

          OrdinaryStateVector sigma_point{Chi.col(i)};
          OrdinaryStateVector mu_diff{ProcessModel::Subtract(sigma_point, mu)};

          Sigma_x_z += w(i) * mu_diff * z_diff.transpose();
        }

        auto Kt{Sigma_x_z * S.inverse()};

        return {
            /* timestamp */               measurement.t(),
            /* state vector */            mu + Kt * measurement_model.Diff(measurement.z(), z),
            /* state covariance matrix */ bel.Sigma() - Kt * S * Kt.transpose(),
        };
      }

    public:
      using KalmanFilterBase<ProcessModel, MeasurementModel, UnscentedKalmanFilter>::Predict;
      using KalmanFilterBase<ProcessModel, MeasurementModel, UnscentedKalmanFilter>::Update;

      /**
       * Prediction step of the Unscented Kalman filter. Predicts the object's state in dt time in the future
       * in accordance with NON-LINEAR process model and input control vector.
       *
       * Notice that for linear process models the compiler will choose the corresponding method from the
       * base class (KalmanFilterBase) which works with linear process models.
       *
       * @param bel a current belief of the object's state
       * @param ut a control vector
       * @param dt a time interval between the previous and current measurements
       * @param process_model an instance of the process model
       *
       * @return a prior belief, that is, after prediction but before incorporating the measurement
       */
      template<bool enable = true>
      static auto
      Predict(const Belief& bel, const ControlVector& ut, double_t dt, const ProcessModel& process_model)
      -> std::enable_if_t<not ProcessModel::IsLinear() and enable, Belief>
      {
        return std::get<0>(PredictNonLinear(bel, ut, dt, process_model));
      }

      /**
       * Update step of the Unscented Kalman filter. Incorporates the sensor measurement into the given prior belief.
       * Works only with NON-LINEAR measurement models.
       *
       * Notice that for linear measurement models the compiler will choose the corresponding method from the
       * base class (KalmanFilterBase) which works with linear measurement models.
       *
       * @param bel a belief after the prediction step of the Unscented Kalman filter
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
        constexpr auto state_dims{ProcessModel::StateDims()};
        constexpr auto sigma_points_num{SigmaPointsNumber(state_dims)};

        OrdinaryWeightsVector w{Weights<state_dims, sigma_points_num>()};
        OrdinarySigmaPointsMatrix Chi{GenerateSigmaPointsMatrix<state_dims, sigma_points_num>(bel.mu(), bel.Sigma())};

        return UpdateNonLinear<sigma_points_num>(bel, Chi, w, measurement, measurement_model);
      }

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
       * @param measurement a measurement from the sensor
       * @param process_model an instance of the process model
       * @param measurement_model an instance of the measurement model
       * @return a posterior belief, that is, after the prediction and incorporation of the measurement
       */
      template<bool enable = true>
      static auto
      PredictUpdate(const Belief& bel, const ControlVector& ut, const Measurement& measurement,
                    const ProcessModel& process_model, const MeasurementModel& measurement_model)
      -> std::enable_if_t<not (ProcessModel::IsLinear() or MeasurementModel::IsLinear()) and enable, Belief>
      {
        auto dt = measurement.t() - bel.t();
        auto bel_Chi_w{PredictNonLinear(bel, ut, dt, process_model)};
        return UpdateNonLinear(
            std::get<0>(bel_Chi_w), std::get<1>(bel_Chi_w), std::get<2>(bel_Chi_w), measurement, measurement_model);
      }

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
       * @param measurement a measurement from the sensor
       * @param process_model an instance of the process model
       * @param measurement_model an instance of the measurement model
       * @return a posterior belief, that is, after the prediction and incorporation of the measurement
       */
      template<bool enable = true>
      static auto
      PredictUpdate(const Belief& bel, const ControlVector& ut, const Measurement& measurement,
                    const ProcessModel& process_model, const MeasurementModel& measurement_model)
      -> std::enable_if_t<(ProcessModel::IsLinear() or MeasurementModel::IsLinear()) and enable, Belief>
      {
        return KalmanFilterBase<ProcessModel, MeasurementModel, UnscentedKalmanFilter>::
                 PredictUpdate(bel, ut, measurement, process_model, measurement_model);
      }
    };

  }
}


#endif //SENSOR_FUSION_UNSCENTEDKALMANFILTER_HPP
