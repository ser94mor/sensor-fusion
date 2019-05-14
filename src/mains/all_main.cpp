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

#include <fusion.hpp>

#include <cstdlib>

using namespace ser94mor::sensor_fusion;

int main(int, char**)
{
  // parameter initialization
  CV::ProcessNoiseCovarianceMatrix cv_mtx;
  cv_mtx << 1.0, 2.0,
            3.0, 4.0;
  
  CTRV::ProcessNoiseCovarianceMatrix ctrv_mtx;
  ctrv_mtx << 5.0, 6.0,
              7.0, 8.0;

  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx <<  9.0, 10.0,
               11.0, 12.0;

  Radar::MeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 13.0, 14.0, 15.0,
               16.0, 17.0, 18.0,
               19.0, 20.0, 21.0;
  
  Lidar::Measurement lidar_measurement{22.0, Lidar::MeasurementVector::Constant(23.0)};
  
  Radar::Measurement radar_measurement{24.0, Radar::MeasurementVector::Constant(25.0)};
  
  // variations of Kalman filter
  KF_CV_LIDAR_Fusion kf_cv_lidar_fusion{cv_mtx, lidar_mtx};
  kf_cv_lidar_fusion.ProcessMeasurement(lidar_measurement);
  kf_cv_lidar_fusion.ProcessMeasurement(radar_measurement); 
  
  EKF_CV_RADAR_Fusion ekf_cv_radar_fusion{cv_mtx, radar_mtx};
  ekf_cv_radar_fusion.ProcessMeasurement(lidar_measurement);
  ekf_cv_radar_fusion.ProcessMeasurement(radar_measurement);
  
  EKF_CV_LIDAR_RADAR_Fusion ekf_cv_lidar_radar_fusion{cv_mtx, lidar_mtx, radar_mtx};
  ekf_cv_lidar_radar_fusion.ProcessMeasurement(lidar_measurement);
  ekf_cv_lidar_radar_fusion.ProcessMeasurement(radar_measurement);
  
  EKF_CTRV_LIDAR_Fusion ekf_ctrv_lidar_fusion{ctrv_mtx, lidar_mtx};
  ekf_ctrv_lidar_fusion.ProcessMeasurement(lidar_measurement);
  ekf_ctrv_lidar_fusion.ProcessMeasurement(radar_measurement);
  
  EKF_CTRV_RADAR_Fusion ekf_ctrv_radar_fusion{ctrv_mtx, radar_mtx};
  ekf_ctrv_radar_fusion.ProcessMeasurement(lidar_measurement);
  ekf_ctrv_radar_fusion.ProcessMeasurement(radar_measurement);
  
  EKF_CTRV_LIDAR_RADAR_Fusion ekf_ctrv_lidar_radar_fusion{ctrv_mtx, lidar_mtx, radar_mtx};
  ekf_ctrv_lidar_radar_fusion.ProcessMeasurement(lidar_measurement);
  ekf_ctrv_lidar_radar_fusion.ProcessMeasurement(radar_measurement);

  UKF_CV_RADAR_Fusion ukf_cv_radar_fusion{cv_mtx, radar_mtx};
  ukf_cv_radar_fusion.ProcessMeasurement(lidar_measurement);
  ukf_cv_radar_fusion.ProcessMeasurement(radar_measurement);
  
  UKF_CV_LIDAR_RADAR_Fusion ukf_cv_lidar_radar_fusion{cv_mtx, lidar_mtx, radar_mtx};
  ukf_cv_lidar_radar_fusion.ProcessMeasurement(lidar_measurement);
  ukf_cv_lidar_radar_fusion.ProcessMeasurement(radar_measurement);
  
  UKF_CTRV_LIDAR_Fusion ukf_ctrv_lidar_fusion{ctrv_mtx, lidar_mtx};
  ukf_ctrv_lidar_fusion.ProcessMeasurement(lidar_measurement);
  ukf_ctrv_lidar_fusion.ProcessMeasurement(radar_measurement);
  
  UKF_CTRV_RADAR_Fusion ukf_ctrv_radar_fusion{ctrv_mtx, radar_mtx};
  ukf_ctrv_radar_fusion.ProcessMeasurement(lidar_measurement);
  ukf_ctrv_radar_fusion.ProcessMeasurement(radar_measurement);
  
  UKF_CTRV_LIDAR_RADAR_Fusion ukf_ctrv_lidar_radar_fusion{ctrv_mtx, lidar_mtx, radar_mtx};
  ukf_ctrv_lidar_radar_fusion.ProcessMeasurement(lidar_measurement);
  ukf_ctrv_lidar_radar_fusion.ProcessMeasurement(radar_measurement);
  
  return EXIT_SUCCESS;
}
