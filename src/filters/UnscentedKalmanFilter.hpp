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
       * Prediction step of the Extended Kalman filter. Predicts the object's state in dt time in the future
       * in accordance with NON-LINEAR process model and input control vector.
       *
       * Notice that for linear process models the compiler will choose the corresponding method from the
       * base class (KalmanFilter) which works with linear process models.
       *
       * @param belief_posterior a current belief of the object's state
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
        WeightsVector weights_vector{CalculateWeights()};

        auto Xsig{PredictSigmaPoints(bel, ut, dt, process_model)};

        //predicted state mean
        StateVector mu{StateVector::Zero()};
        for (int i = 0; i < CalligraphicXMatrixCols(); ++i)
        {
          mu += weights_vector(i) * Xsig.col(i);
        }

        //predicted state covariance matrix
        StateCovarianceMatrix Sigma{StateCovarianceMatrix::Zero()};
        for (int i = 0; i < CalligraphicXMatrixCols(); ++i) {  //iterate over sigma points
          auto diff{process_model.Diff(Xsig.col(i), mu)};
          Sigma += weights_vector(i) * diff * diff.transpose();
        }

        return {
            /* timestamp */               bel.t() + dt,
            /* state vector */            mu,
            /* state covariance matrix */ Sigma,
        };
      }

    private:
      /**
       * @return TODO
       */
      constexpr static int AugmentedStateDims()
      {
        return ProcessModel::StateDims() + ProcessModel::ProcessNoiseDims();
      }

      /**
       * @return TODO
       */
      constexpr static int CalligraphicXMatrixCols()
      {
        return 2 * AugmentedStateDims() + 1;
      }

      /**
       * @return TODO
       */
      constexpr static int SigmaPointSpreadingParameter()
      {
        return 3 - AugmentedStateDims();
      }

      //
      // Some useful typedefs that are used in unscented Kalman filter methods
      //
      using AugmentedStateVector = Eigen::Matrix<double_t, AugmentedStateDims(), 1>;
      using AugmentedStateCovarianceMatrix = Eigen::Matrix<double_t, AugmentedStateDims(), AugmentedStateDims()>;
      using AugmentedSigmaPointsMatrix = Eigen::Matrix<double_t, AugmentedStateDims(), CalligraphicXMatrixCols()>;
      using SigmaPointsMatrix = Eigen::Matrix<double_t, ProcessModel::StateDims(), CalligraphicXMatrixCols()>;
      using WeightsVector = Eigen::Matrix<double_t, CalligraphicXMatrixCols(), 1>;

      /**
       * TODO
       * @return
       */
      constexpr static WeightsVector CalculateWeights()
      {
        WeightsVector weights_vector{
          WeightsVector::Ones() * (0.5 / (SigmaPointSpreadingParameter() + AugmentedStateDims()))};
        weights_vector(0) = SigmaPointSpreadingParameter() / (SigmaPointSpreadingParameter() + AugmentedStateDims());

        return weights_vector;
      }

      /**
       * TODO
       * @param belief
       * @param process_model
       * @return
       */
      static AugmentedSigmaPointsMatrix
      GenerateAugmentedSigmaPoints(const Belief& belief, const ProcessModel& process_model)
      {
        AugmentedStateVector mu_aug{AugmentedStateVector::Zero()};
        mu_aug.head(ProcessModel::StateDims()) = belief.mu();

        AugmentedStateCovarianceMatrix Sigma_aug{AugmentedStateCovarianceMatrix::Zero()};
        Sigma_aug.topLeftCorner(ProcessModel::StateDims(),ProcessModel::StateDims()) = belief.Sigma();
        Sigma_aug.bottomRightCorner(ProcessModel::ProcessNoiseDims(), ProcessModel::ProcessNoiseDims())
          = process_model.GetProcessNoiseCovarianceMatrix();

        // create square root matrix
        AugmentedStateCovarianceMatrix L{Sigma_aug.llt().matrixL()};

        AugmentedSigmaPointsMatrix X_aug;
        X_aug.col(0)  = mu_aug;
        for (int i = 0; i < AugmentedStateDims(); ++i)
        {
          X_aug.col(i+1) = mu_aug + std::sqrt(SigmaPointSpreadingParameter() + AugmentedStateDims()) * L.col(i);
          X_aug.col(i+1+AugmentedStateDims()) =
              mu_aug - std::sqrt(SigmaPointSpreadingParameter() + AugmentedStateDims()) * L.col(i);
        }

        return X_aug;
      }

      /**
       * TODO
       * @param belief
       * @param control_vector
       * @param dt
       * @param process_model
       * @return
       */
      static SigmaPointsMatrix
      PredictSigmaPoints(const Belief& belief, const ControlVector& control_vector, double_t dt,
                         const ProcessModel& process_model)
      {
        AugmentedSigmaPointsMatrix Xsig_aug{GenerateAugmentedSigmaPoints(belief, process_model)};

        SigmaPointsMatrix Xsig;

        //predict sigma points
        for (int i = 0; i < CalligraphicXMatrixCols(); ++i)
        {
          Xsig.col(i) = process_model.g(dt, control_vector, Xsig_aug.col(i).head(5), Xsig_aug.col(i).tail(2));
        }

        return Xsig;
      }



    };

  }
}


#endif //SENSOR_FUSION_UNSCENTEDKALMANFILTER_HPP
