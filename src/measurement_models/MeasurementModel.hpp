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

namespace ser94mor::sensor_fusion
{

  template<class MeasurementVector, class MeasurementCovarianceMatrix,
           class StateVector, class Sensor, const char* name>
  class MeasurementModel
  {
  public:
    using Measurement_type = Measurement<MeasurementVector, MeasurementCovarianceMatrix>;

    constexpr static const char* Type()
    {
      return kMeasurementModelType;
    }

    constexpr static const char* Name()
    {
      return name;
    }

    constexpr static int MeasurementDims()
    {
      return MeasurementVector::SizeAtCompileTime;
    }

    constexpr static int StateDims()
    {
      return StateVector::SizeAtCompileTime;
    }

    void SetMeasurementCovarianceMatrix(const MeasurementCovarianceMatrix& mtx);

  protected:
    MeasurementCovarianceMatrix measurement_covariance_matrix_;
  };

  template<class MeasurementVector, class MeasurementCovarianceMatrix, class StateVector, class Sensor, const char* name>
  void MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
                        StateVector, Sensor, name>::SetMeasurementCovarianceMatrix(
      const MeasurementCovarianceMatrix& mtx)
  {
    measurement_covariance_matrix_ = mtx;
  }

}

#endif //SENSOR_FUSION_MEASUREMENTMODEL_HPP
