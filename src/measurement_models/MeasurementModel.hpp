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
     * @tparam ProcessModel a class of the process model
     * @tparam mmk a kind of a measurement model (from a corresponding enum class)
     * @tparam is_linear flag indicating whether this measurement model is linear or not
     */
    template<class MeasurementVector, class MeasurementCovarianceMatrix, class ProcessModel, class Sensor,
             MeasurementModelKind mmk, bool is_linear>
    class MeasurementModel : public ModelEntity<EntityType::MeasurementModel, MeasurementModelKind, mmk, is_linear>
    {
    public:
      /**
       * The typedefs below are needed in other places in the code. These typedefs, in fact, are attributes of the
       * measurement model.
       */
      using Measurement_type = Measurement<MeasurementVector, MeasurementCovarianceMatrix, mmk>;
      using MeasurementCovarianceMatrix_type = MeasurementCovarianceMatrix;
      using Sensor_type = Sensor;

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
       * Set measurement covariance matrix. It is done explicitly by the user of measurement model
       * due to the variadic templates used in this code. MeasurementModel needs a default constructor.
       * @param mtx a measurement covariance matrix
       */
      void SetMeasurementCovarianceMatrix(const MeasurementCovarianceMatrix& mtx);

    protected:
      MeasurementCovarianceMatrix measurement_covariance_matrix_;
    };

    template<class MeasurementVector, class MeasurementCovarianceMatrix, class StateVector, class Sensor,
        MeasurementModelKind mmk, bool is_linear>
    void MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix, StateVector, Sensor, mmk, is_linear>::
      SetMeasurementCovarianceMatrix(const MeasurementCovarianceMatrix& mtx)
    {
      measurement_covariance_matrix_ = mtx;
    }

  }
}

#endif //SENSOR_FUSION_MEASUREMENTMODEL_HPP
