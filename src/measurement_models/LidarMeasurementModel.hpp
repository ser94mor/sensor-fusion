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

      /**
       * A concrete (Lidar) measurement model. It is still a template because the dimensionality of
       * the measurement matrix depends on the process model kind, which we know only at compile time.
       * @tparam ProcessModel a process model class, which is needed to determine the number of state dimensions
       */
      template<class ProcessModel>
      class MeasurementModel :
        public ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
          ProcessModel, Sensor, MeasurementModelKind::Lidar, kIsLinear>
      {
      public:
        using MeasurementMatrix_type =
          Eigen::Matrix<double, MeasurementModel::MeasurementDims(), MeasurementModel::StateDims()>;

        /**
         * Constructor.
         * Measurement matrix is of the form
         *   1 0 0 0 ...
         *   0 1 0 0 ...
         * where the number of columns equal to the number of state dimensions.
         */
        MeasurementModel()
        : ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
            ProcessModel, Sensor, MeasurementModelKind::Lidar, kIsLinear>{},
          measurement_matrix_{MeasurementMatrix_type::Identity()}
        {

        }

        /**
         * @return a measurement matrix
         */
        const MeasurementMatrix_type& C() const
        {
          return measurement_matrix_;
        }

        /**
         * @return a measurement covariance matrix
         */
        const MeasurementCovarianceMatrix& Q() const
        {
          return this->measurement_covariance_matrix_;
        }

      private:
        MeasurementMatrix_type measurement_matrix_;
      };

    }
  }
}


#endif //SENSOR_FUSION_LIDARMEASUREMENTMODEL_HPP
