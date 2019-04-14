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


#include "process_models.hpp"
#include "measurement_models.hpp"
#include "sensors.hpp"
#include "filters.hpp"
#include "../src/fusion/Fusion.hpp"

#include <catch.hpp>
//#include "fusion.hpp"

using namespace ser94mor::sensor_fusion;


TEST_CASE("Fusion::Test", "[fusion]")
{
  IndividualNoiseProcessesCovarianceMatrix p_mtx;
  p_mtx << 9.0, 0.0,
           0.0, 9.0;

  Lidar::MeasurementCovarianceMatrix m_mtx;
  m_mtx << 0.0225,    0.0,
              0.0, 0.0225;

  Lidar::MeasurementCovarianceMatrix lidar_mtx2;
  lidar_mtx2 << 0.1, 0.0,
                0.0, 0.1;

  Fusion<KalmanFilter, CV::ProcessModel, Lidar::MeasurementModel, Lidar::MeasurementModel> fusion{p_mtx, m_mtx, lidar_mtx2};
  fusion.Start();
  //Lidar::Sensor sensor{fusion.GetSensor<Lidar::Sensor>()};
}
