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
      using Belief = typename ProcessModel::Belief_type;
      using StateVector = typename ProcessModel::StateVector_type;
      using StateCovarianceMatrix = typename ProcessModel::StateCovarianceMatrix_type;
      using ControlVector = typename ProcessModel::ControlVector_type;
      using Measurement = typename MeasurementModel::Measurement_type;

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
        WeightsVector w{Weights()};

        auto Chi{PredictSigmaPoints(bel, ut, dt, process_model)};

        // predict state vector
        StateVector mu{StateVector::Zero()};
        for (int i = 0; i < SigmaPointsNumber(); ++i)
        {
          mu += w(i) * Chi.col(i);
        }

        // predict state covariance matrix
        StateCovarianceMatrix Sigma{StateCovarianceMatrix::Zero()};
        for (int i = 0; i < SigmaPointsNumber(); ++i)
        {
          auto diff{process_model.Diff(Chi.col(i), mu)};
          Sigma += w(i) * diff * diff.transpose();
        }

        return {
            /* timestamp */               bel.t() + dt,
            /* state vector */            mu,
            /* state covariance matrix */ Sigma,
        };
      }

    private:
      /**
       * @return a number of dimensions in the augmented state vector
       */
      constexpr static int AugmentedStateDims()
      {
        return ProcessModel::StateDims() + ProcessModel::ProcessNoiseDims();
      }

      /**
       * @return a number of sigmapoints to generate
       */
      constexpr static int SigmaPointsNumber()
      {
        return 2 * AugmentedStateDims() + 1;
      }

      /**
       * @return a sigma points spreading parameter (lambda)
       */
      constexpr static double_t SigmaPointSpreadingParameter()
      {
        // the value is chosen in accordance with a well known heuristics
        return 3.0 - AugmentedStateDims();
      }

      /**
       * Some useful typedefs that are used in unscented Kalman filter methods.
       */
      using AugmentedStateVector = Eigen::Matrix<double_t, AugmentedStateDims(), 1>;
      using AugmentedStateCovarianceMatrix = Eigen::Matrix<double_t, AugmentedStateDims(), AugmentedStateDims()>;
      using AugmentedSigmaPointsMatrix = Eigen::Matrix<double_t, AugmentedStateDims(), SigmaPointsNumber()>;
      using SigmaPointsMatrix = Eigen::Matrix<double_t, ProcessModel::StateDims(), SigmaPointsNumber()>;
      using WeightsVector = Eigen::Matrix<double_t, SigmaPointsNumber(), 1>;

      /**
       * @return a sigma points weights
       */
      constexpr static WeightsVector Weights()
      {
        WeightsVector w{WeightsVector::Ones() * (0.5 / (SigmaPointSpreadingParameter() + AugmentedStateDims()))};
        w(0) = SigmaPointSpreadingParameter() / (SigmaPointSpreadingParameter() + AugmentedStateDims());

        return w;
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
        // an augmented state vector
        AugmentedStateVector mu_aug{AugmentedStateVector::Zero()};
        mu_aug.head(ProcessModel::StateDims()) = bel.mu();

        // an augmented state covariance matrix
        AugmentedStateCovarianceMatrix Sigma_aug{AugmentedStateCovarianceMatrix::Zero()};
        Sigma_aug.topLeftCorner(ProcessModel::StateDims(), ProcessModel::StateDims()) = bel.Sigma();
        Sigma_aug.bottomRightCorner(ProcessModel::ProcessNoiseDims(), ProcessModel::ProcessNoiseDims())
          = process_model.GetProcessNoiseCovarianceMatrix();

        // calculate square root matrix out of an augmeneted state covariance matrix
        AugmentedStateCovarianceMatrix L{Sigma_aug.llt().matrixL()};

        // initialize columns (i.e., sigma points) of the augmented sigma points matrix
        AugmentedSigmaPointsMatrix Chi_aug;
        Chi_aug.col(0) = mu_aug;
        for (int i = 0; i < AugmentedStateDims(); ++i)
        {
          Chi_aug.col(i+1) = mu_aug + std::sqrt(SigmaPointSpreadingParameter() + AugmentedStateDims()) * L.col(i);
          Chi_aug.col(i+1+AugmentedStateDims()) =
              mu_aug - std::sqrt(SigmaPointSpreadingParameter() + AugmentedStateDims()) * L.col(i);
        }

        return Chi_aug;
      }

      /**
       * Predict step applied to each sigma point, that is, the non-linear transformation in accordance with the
       * process model is applied to each sigma point as if it were a state vector.
       *
       * @param bel a current belief
       * @param ut a control vector
       * @param dt a time interval between the previous and current measurements
       * @param process_model an instance of the process model
       * @return a sigma points matrix
       */
      static SigmaPointsMatrix
      PredictSigmaPoints(const Belief& bel, const ControlVector& ut, double_t dt, const ProcessModel& process_model)
      {
        AugmentedSigmaPointsMatrix Chi_aug{GenerateAugmentedSigmaPointsMatrix(bel, process_model)};

        // predict sigma points
        SigmaPointsMatrix Chi;
        for (int i = 0; i < SigmaPointsNumber(); ++i)
        {
          Chi.col(i) = process_model.g(dt, ut, Chi_aug.col(i).head(5), Chi_aug.col(i).tail(2));
        }

        return Chi;
      }

    };

  }
}


#endif //SENSOR_FUSION_UNSCENTEDKALMANFILTER_HPP
