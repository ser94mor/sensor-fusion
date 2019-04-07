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

#ifndef SENSOR_FUSION_LIDAR_HPP
#define SENSOR_FUSION_LIDAR_HPP


#include "Sensor.hpp"
#include "SensorFactory.hpp"


extern const char kLidarName[];


class Lidar : public Sensor<2, kLidarName>
{
public:
  static Lidar FromJson(const char* json_str);

  static Lidar FromParams(const Eigen::Matrix<double, Lidar::Dims(), Lidar::Dims()>& mtx)
  {
    return SensorFactory<Lidar>::FromParams(mtx);
  }
};


#endif //SENSOR_FUSION_LIDAR_HPP
