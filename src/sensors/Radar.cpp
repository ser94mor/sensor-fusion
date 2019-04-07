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

#include "Radar.hpp"
#include "SensorFactory.hpp"


const char kRadarName[]{"RADAR"};

Radar Radar::FromJson(const char* json_str)
{
  return SensorFactory<Radar>::FromJson(json_str);
}

Radar::Radar(const Eigen::Matrix<double, Radar::Dims(), Radar::Dims()>& measurement_covariance_matrix) :
    Sensor{measurement_covariance_matrix}
{

}

