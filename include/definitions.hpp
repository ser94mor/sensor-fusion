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

#ifndef SENSOR_FUSION_DEFINITIONS_HPP
#define SENSOR_FUSION_DEFINITIONS_HPP

#include <Eigen/Dense>

namespace ser94mor::sensor_fusion
{

  const std::array<const std::pair<const char*, const std::array<const char*, 2>>, 2> kEntityKinds{
      std::make_pair<const char*, const std::array<const char*, 2>>("PROCESS_MODEL", {"CV",    "CTRV" }),
      std::make_pair<const char*, const std::array<const char*, 2>>("SENSOR",        {"RADAR", "LIDAR"}),
  };

  ////////////////////
  // process models //
  ////////////////////
  const char kProcessModelType[]{"PROCESS_MODEL"};
  const char kSensorType[]{"SENSOR"};
  const char kMeasurementModelType[]{"MEASUREMENT_MODEL"};

  /////////////
  // sensors //
  /////////////
  const int kMaxSensors = 2;

  /////////////
  //  states //
  /////////////
  //const int kMaxStateDims = 5;
  //using StateVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxStateDims, 1>;
  //using StateCovarianceMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxStateDims, kMaxStateDims>;


  //////////////////
  // measurements //
  //////////////////
  //const int kMaxMeasurementDims = 3;
  //using MeasurementVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxMeasurementDims, 1>;
  //using CMeasurementCovarianceMatrix =
  //    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxMeasurementDims, kMaxMeasurementDims>;


  //////////////
  // controls //
  //////////////
  const int kMaxControlDims = -1;
  using ControlVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxControlDims, 1>;
  using ControlMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxControlDims, kMaxControlDims>;


  //////////////////////
  // CV PROCESS MODEL //
  //////////////////////
  extern const char kCVProcessModelName[];
  const int kCVStateVectorDims    = 4;
  const int kCVControlVectorDims  = 4;
  using CVStateVector             = Eigen::Matrix<double, kCVStateVectorDims, 1>;
  using CVStateCovarianceMatrix   = Eigen::Matrix<double, kCVStateVectorDims, kCVStateVectorDims>;
  using CVStateTransitionMatrix   = Eigen::Matrix<double, kCVStateVectorDims, kCVStateVectorDims>;
  using CVControlVector           = Eigen::Matrix<double, kCVControlVectorDims, 1>;
  using CVControlTransitionMatrix = Eigen::Matrix<double, kCVStateVectorDims, kCVControlVectorDims>;
  using CVProcessCovarianceMatrix = Eigen::Matrix<double, kCVStateVectorDims, kCVStateVectorDims>;


  /////////////////////////////
  // RADAR MEASUREMENT MODEL //
  /////////////////////////////
  extern const char kRadarSensorName[];
  extern const char kRadarMeasurementModelName[];
  const int kRadarMeasurementVectorDims = 3;
  using RadarMeasurementVector = Eigen::Matrix<double, kRadarMeasurementVectorDims, 1>;
  using RadarMeasurementCovarianceMatrix =
      Eigen::Matrix<double, kRadarMeasurementVectorDims, kRadarMeasurementVectorDims>;


  ////////////////////////
  // MEASUREMENT MODELS //
  ////////////////////////

#define MEASUREMENT_MODEL_DEFINITIONS(measurement_vector_dims) \
  extern const char kSensorName[]; \
  const int kMeasurementVectorDims = 2; \
  extern const char kMeasurementModelName[]; \
  using MeasurementVector = Eigen::Matrix<double, kMeasurementVectorDims, 1>; \
  using MeasurementCovarianceMatrix = Eigen::Matrix<double, kMeasurementVectorDims, kMeasurementVectorDims>;

  namespace Lidar
  {
    MEASUREMENT_MODEL_DEFINITIONS(2);
  }

  namespace Radar
  {
    MEASUREMENT_MODEL_DEFINITIONS(3);
  }

}

#endif //SENSOR_FUSION_DEFINITIONS_HPP
