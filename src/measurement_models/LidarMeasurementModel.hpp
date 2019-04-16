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


namespace ser94mor
{
  namespace  sensor_fusion
  {
    namespace Lidar
    {

      template<class StateVector>
      class MeasurementModel :
          public ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
              StateVector, MeasurementModelKind::Lidar, kIsLinear>
      {
      public:
        using MeasurementMatrix =
        Eigen::Matrix<double, MeasurementModel::MeasurementDims(), MeasurementModel::StateDims()>;
        using Sensor_type = Sensor;
        using MeasurementCovarianceMatrix_type = MeasurementCovarianceMatrix;

        MeasurementModel() : measurement_matrix_{MeasurementMatrix::Identity()}
        {

        }

        const MeasurementMatrix& C() const
        {
          return measurement_matrix_;
        }

        const MeasurementCovarianceMatrix& Q() const
        {
          return this->measurement_covariance_matrix_;
        }

      private:
        MeasurementMatrix measurement_matrix_;
      };

    }
  }
}


#endif //SENSOR_FUSION_LIDARMEASUREMENTMODEL_HPP
