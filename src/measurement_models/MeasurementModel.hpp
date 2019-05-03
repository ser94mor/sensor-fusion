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

#ifndef SENSOR_FUSION_MEASUREMENTMODEL_HPP
#define SENSOR_FUSION_MEASUREMENTMODEL_HPP


#include "definitions.hpp"
#include "measurements.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base template class representing a measurement model. It contains methods and functions common for
     * all concrete measurement models. Notice that in addition to
     * {@tparam MeasurementVector} and {@tparam MeasurementCovarianceMatrix} template parameters it also accepts
     * {@tparam ProcessModel} parameter.
     *
     * @tparam MeasurementVector a class of the measurement vector
     * @tparam MeasurementCovarianceMatrix a class of the measurement covariance matrix
     * @tparam MeasurementVectorView a class of the measurement vector view
     *                               (accessor to the measurement vector dimensions)
     * @tparam ProcessModel a class of the process model
     * @tparam mmk a kind of a measurement model (from a corresponding enum class)
     * @tparam is_linear flag indicating whether this measurement model is linear or not
     */
    template<class MeasurementVector, class MeasurementCovarianceMatrix, class ROMeasurementVectorView,
             class ProcessModel, MeasurementModelKind mmk, bool is_linear>
    class MeasurementModel : public ModelEntity<EntityType::MeasurementModel, MeasurementModelKind, mmk, is_linear>
    {
    public:
      /**
       * The typedefs below are needed in other places in the code. These typedefs, in fact, are attributes of the
       * measurement model.
       */
      using Measurement_type = Measurement<MeasurementVector, MeasurementCovarianceMatrix, mmk>;
      using MeasurementVector_type = MeasurementVector;
      using ROMeasurementVectorView_type = ROMeasurementVectorView;
      using MeasurementCovarianceMatrix_type = MeasurementCovarianceMatrix;

      using RWStateVectorView_type = typename ProcessModel::RWStateVectorView_type;
      using StateVector_type = typename ProcessModel::StateVector_type;

      using Belief_type = typename ProcessModel::Belief_type;

      /**
       * @return a number of dimensions in measurement vector
       */
      constexpr static int MeasurementDims()
      {
        return MeasurementVector::SizeAtCompileTime;
      }

      /**
       * @return a number of state dimensions
       */
      constexpr static int StateDims()
      {
        return ProcessModel::StateDims();
      }

      /**
       * The naming of measurement covariance matrix is taken from the
       * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
       * @return a measurement covariance matrix
       */
      const MeasurementCovarianceMatrix& Q() const
      {
        return measurement_covariance_matrix_;
      }

      /**
       * Calculate a difference between two measurement vectors. Some kinds of measurement vectors may have a dimensions
       * that need to be adjusted after the simple vector subtraction operation, such as dimensions representing
       * angles that has to be within in [-pi, pi].
       *
       * @param measurement_vector_1 the first measurement vector
       * @param measurement_vector_2 the second measurement vector
       * @return the difference between the two measurement vectors
       */
      virtual MeasurementVector Diff(const MeasurementVector& measurement_vector_1,
                                     const MeasurementVector& measurement_vector_2) const = 0;

      /**
       * Set measurement covariance matrix. It is done explicitly by the user of measurement model
       * due to the variadic templates used in this code. MeasurementModel needs a default constructor.
       * @param mtx a measurement covariance matrix
       */
      void SetMeasurementCovarianceMatrix(const MeasurementCovarianceMatrix& mtx)
      {
        measurement_covariance_matrix_ = mtx;
      }

      /**
       * During the sensor fusion process, we need to initialize our initial belief based on something. When we receive
       * a first measurement, it becomes the most precise notion of the object's state. So, it is reasonable to
       * form an initial belief based on the first measurement.
       *
       * TODO: measurement covariance matrix should have 1's in px, py dimensions and much higher values
       *       for the dimensions of the state vector about which we have no information from the measurement vector.
       *
       * @param measurement the first received measurement
       * @return an initial belief
       */
      static Belief_type GetInitialBeliefBasedOn(const Measurement_type& measurement)
      {
        StateVector_type state_vector{StateVector_type::Zero()};
        RWStateVectorView_type svv{state_vector};
        ROMeasurementVectorView_type mvv{measurement.z()};

        svv.px() = mvv.px();
        svv.py() = mvv.py();

        typename ProcessModel::StateCovarianceMatrix_type
          state_covariance_matrix{ProcessModel::StateCovarianceMatrix_type::Identity()};
        return Belief_type{measurement.t(), state_vector, state_covariance_matrix};
      }

    protected:
      MeasurementCovarianceMatrix measurement_covariance_matrix_;
    };

  }
}

#endif //SENSOR_FUSION_MEASUREMENTMODEL_HPP
