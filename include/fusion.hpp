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

#include "../src/fusion/Fusion.hpp"

#ifndef SENSOR_FUSION_FUSION_HEADER_HPP
#define SENSOR_FUSION_FUSION_HEADER_HPP


#include "definitions.hpp"
#include "filters.hpp"
#include "process_models.hpp"
#include "measurement_models.hpp"

namespace ser94mor
{
  namespace sensor_fusion
  {
    ///////////////////////////////////////////
    // Definitions of sensor fusion classes. //
    ///////////////////////////////////////////

    //
    // (!) Notice that only reasonable definitions are provided. For example, EKF_CV_LIDAR_Fusion is not a reasonable
    // definition, since it boils down to the KF_CV_LIDAR_Fusion case.
    //

    // Kalman filter
    using KF_CV_LIDAR_Fusion = Fusion<KalmanFilter, CV::ProcessModel, Lidar::MeasurementModel>;

    // Extended Kalman filter
    using EKF_CV_RADAR_Fusion = Fusion<ExtendedKalmanFilter, CV::ProcessModel, Radar::MeasurementModel>;
    using EKF_CV_LIDAR_RADAR_Fusion =
        Fusion<ExtendedKalmanFilter, CV::ProcessModel, Lidar::MeasurementModel, Radar::MeasurementModel>;
    using EKF_CTRV_LIDAR_Fusion = Fusion<ExtendedKalmanFilter, CTRV::ProcessModel, Lidar::MeasurementModel>;
    using EKF_CTRV_RADAR_Fusion = Fusion<ExtendedKalmanFilter, CTRV::ProcessModel, Radar::MeasurementModel>;
    using EKF_CTRV_LIDAR_RADAR_Fusion =
        Fusion<ExtendedKalmanFilter, CTRV::ProcessModel, Lidar::MeasurementModel, Radar::MeasurementModel>;

    // Unscented Kalman filter
    using UKF_CV_RADAR_Fusion = Fusion<UnscentedKalmanFilter, CV::ProcessModel, Radar::MeasurementModel>;
    using UKF_CV_LIDAR_RADAR_Fusion =
        Fusion<UnscentedKalmanFilter, CV::ProcessModel, Lidar::MeasurementModel, Radar::MeasurementModel>;
    using UKF_CTRV_LIDAR_Fusion = Fusion<UnscentedKalmanFilter, CTRV::ProcessModel, Lidar::MeasurementModel>;
    using UKF_CTRV_RADAR_Fusion = Fusion<UnscentedKalmanFilter, CTRV::ProcessModel, Radar::MeasurementModel>;
    using UKF_CTRV_LIDAR_RADAR_Fusion =
        Fusion<UnscentedKalmanFilter, CTRV::ProcessModel, Lidar::MeasurementModel, Radar::MeasurementModel>;

  }
}
#endif // SENSOR_FUSION_FUSION_HEADER_HPP
