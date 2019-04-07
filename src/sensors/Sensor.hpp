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


#include "SensorData.hpp"

#include <ctime>
#include <Eigen/Dense>
#include <unordered_map>


template <int dims, const char* name>
class Sensor
{
public:
  const SensorData<dims> Data(std::time_t timestamp_sec, const Eigen::Matrix<double, dims, 1>& z)
  {
    return SensorData<dims>(timestamp_sec, z, measurement_covariance_matrix_);
  }

  const Eigen::Matrix<double, dims, dims>& MeasurementCovarianceMatrix() const
  {
    return measurement_covariance_matrix_;
  }

  constexpr static const char* Type()
  {
    return "SENSOR";
  }

  constexpr static const char* Name()
  {
    return name;
  }

  constexpr static int Dims()
  {
    return dims;
  }

protected:
  explicit Sensor(const Eigen::Matrix<double, dims, dims>& measurement_covariance_matrix);

private:
  Eigen::Matrix<double, dims, dims> measurement_covariance_matrix_;
};


template<int dims, const char* name>
Sensor<dims, name>::Sensor(const Eigen::Matrix<double, dims, dims>& measurement_covariance_matrix) :
    measurement_covariance_matrix_{measurement_covariance_matrix}
{

}


#endif //SENSOR_FUSION_SENSOR_HPP
