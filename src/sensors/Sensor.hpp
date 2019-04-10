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

#ifndef SENSOR_FUSION_SENSOR_HPP
#define SENSOR_FUSION_SENSOR_HPP


#include "definitions.hpp"
#include "SensorFactory.hpp"
//#include "primitives.hpp"

#include <ctime>


namespace  ser94mor::sensor_fusion
{

  template<int measurement_dims, const char* name, class CMeasurementCovarianceMatrix>
  class Sensor
  {
  public:
    //const GaussianMeasurement Data(std::time_t timestamp_sec, const Eigen::Matrix<double, measurement_dims, 1>& z) const
    //{
    //  return {
    //      .t = timestamp_sec,
    //      .z = z,
    //      .Q = measurement_covariance_matrix_,
    //  };
    //}

    const CMeasurementCovarianceMatrix& MeasurementCovarianceMatrix() const
    {
      return measurement_covariance_matrix_;
    }

    constexpr static const char* Type()
    {
      return kSensorType;
    }

    constexpr static const char* Name()
    {
      return name;
    }

    constexpr static int Dims()
    {
      return measurement_dims;
    }

    void SetMeasurementCovarianceMatrix(const CMeasurementCovarianceMatrix& mtx);
  private:

    CMeasurementCovarianceMatrix measurement_covariance_matrix_;
  };

  template<int measurement_dims, const char* name, class CMeasurementCovarianceMatrix>
  void Sensor<measurement_dims, name, CMeasurementCovarianceMatrix>::SetMeasurementCovarianceMatrix(
      const CMeasurementCovarianceMatrix& mtx)
  {
    measurement_covariance_matrix_ = mtx;
  }

}


#endif //SENSOR_FUSION_SENSOR_HPP
