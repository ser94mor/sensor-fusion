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

#ifndef SENSOR_FUSION_SENSORDATA_HPP
#define SENSOR_FUSION_SENSORDATA_HPP


#include <ctime>
#include <Eigen/Dense>

template<int dims>
class SensorData
{
  template <int, const char*>
  friend class Sensor;

public:
  constexpr static int Dims()
  {
    return dims;
  }

  std::time_t t();

  const Eigen::Matrix<double, dims, 1>& z();

  const Eigen::Matrix<double, dims, dims>& Q();

private:
  SensorData(std::time_t t, const Eigen::Matrix<double, dims, 1>& z, const Eigen::Matrix<double, dims, dims>& Q);

  std::time_t                              t_;
  Eigen::Matrix<double, dims, 1>           z_;
  const Eigen::Matrix<double, dims, dims>& Q_;
};


template<int dims>
std::time_t SensorData<dims>::t()
{
  return t_;
}


template<int dims>
const Eigen::Matrix<double, dims, 1>& SensorData<dims>::z()
{
  return z_;
}


template<int dims>
const Eigen::Matrix<double, dims, dims>& SensorData<dims>::Q()
{
  return Q_;
}

template<int dims>
SensorData<dims>::SensorData(
    std::time_t t, const Eigen::Matrix<double, dims, 1>& z, const Eigen::Matrix<double, dims, dims>& Q) :
    t_{t}, z_{z}, Q_{Q}
{

}


#endif //SENSOR_FUSION_SENSORDATA_HPP
