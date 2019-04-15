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
#include "primitives.hpp"

namespace ser94mor
{
  namespace sensor_fusion
  {

    template<class MeasurementVector, class MeasurementCovarianceMatrix, class StateVector,
        MeasurementModelKind mmk, bool is_linear>
    class MeasurementModel
    {
    public:
      using Measurement_type = Measurement<MeasurementVector, MeasurementCovarianceMatrix>;
      using MeasurementCovarianceMatrix_type = MeasurementCovarianceMatrix;

      constexpr static const char* Type()
      {
        return kMeasurementModelType;
      }

      constexpr MeasurementModelKind Kind()
      {
        return mmk;
      }

      constexpr static const char* Name()
      {
        return MeasurementModelNameByKind(mmk);
      }

      constexpr static int MeasurementDims()
      {
        return MeasurementVector::SizeAtCompileTime;
      }

      constexpr static int StateDims()
      {
        return StateVector::SizeAtCompileTime;
      }

      constexpr static bool IsLinear()
      {
        return is_linear;
      }

      void SetMeasurementCovarianceMatrix(const MeasurementCovarianceMatrix& mtx);

    protected:
      MeasurementCovarianceMatrix measurement_covariance_matrix_;
    };

    template<class MeasurementVector, class MeasurementCovarianceMatrix, class StateVector,
        MeasurementModelKind mmk, bool is_linear>
    void MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
        StateVector, mmk, is_linear>::SetMeasurementCovarianceMatrix(
        const MeasurementCovarianceMatrix& mtx)
    {
      measurement_covariance_matrix_ = mtx;
    }

  }

}

#endif //SENSOR_FUSION_MEASUREMENTMODEL_HPP
