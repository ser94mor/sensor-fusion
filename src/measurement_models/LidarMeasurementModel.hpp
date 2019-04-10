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

#ifndef SENSOR_FUSION_LIDARMEASUREMENTMODEL_HPP
#define SENSOR_FUSION_LIDARMEASUREMENTMODEL_HPP


#include "definitions.hpp"
#include "sensors.hpp"
#include "MeasurementModel.hpp"

#include <ctime>


namespace ser94mor::sensor_fusion
{

  template<class StateVector>
  class LidarMeasurementModel :
      public MeasurementModel<LidarMeasurementVector, LidarMeasurementCovarianceMatrix,
                              StateVector, LidarSensor, kLidarMeasurementModelName>
  {
  public:
    using MeasurementMatrix =
        Eigen::Matrix<double, LidarMeasurementModel::MeasurementDims(), LidarMeasurementModel::StateDims()>;

    explicit LidarMeasurementModel(const LidarSensor&  sensor) :
      MeasurementModel<LidarMeasurementVector, LidarMeasurementCovarianceMatrix,
                       StateVector, LidarSensor, kLidarMeasurementModelName>{sensor}
    {
      measurement_matrix_.setIdentity();
    }

    const MeasurementMatrix& C(std::time_t dt) const
    {
      return measurement_matrix_;
    }

    const LidarMeasurementCovarianceMatrix& Q(std::time_t dt) const
    {
      return this->sensor_.MeasurementCovarianceMatrix();
    }


  private:
    MeasurementMatrix measurement_matrix_;
  };

}


#endif //SENSOR_FUSION_LIDARMEASUREMENTMODEL_HPP
