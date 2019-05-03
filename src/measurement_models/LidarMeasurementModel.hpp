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
#include "MeasurementModel.hpp"
#include "measurement_vector_views.hpp"

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
       * The naming of matrices are taken from the
       * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
       *
       * @tparam ProcessModel a process model class, which is needed to determine the number of state dimensions
       */
      template<class ProcessModel>
      class MeasurementModel :
        public ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
                                                         ROMeasurementVectorView, ProcessModel,
                                                         MeasurementModelKind::Lidar, kIsLinear>
      {
      public:
        using MeasurementMatrix_type =
          Eigen::Matrix<double_t, MeasurementModel::MeasurementDims(), MeasurementModel::StateDims()>;

        /**
         * Constructor.
         * Measurement matrix is of the form
         *   1 0 0 0 ...
         *   0 1 0 0 ...
         * where the number of columns equal to the number of state dimensions.
         */
        MeasurementModel()
        : ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
            ROMeasurementVectorView, ProcessModel, MeasurementModelKind::Lidar, kIsLinear>{},
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
         * Calculate a difference between two measurement vectors. In Lidar case, it is simply a vector subtraction.
         *
         * @param measurement_vector_1 the first measurement vector
         * @param measurement_vector_2 the second measurement vector
         * @return the difference between the two measurement vectors
         */
        MeasurementVector Diff(const MeasurementVector& measurement_vector_1,
                               const MeasurementVector& measurement_vector_2) const override
        {
          return (measurement_vector_1 - measurement_vector_2);
        }

      private:
        MeasurementMatrix_type measurement_matrix_;
      };

    }
  }
}


#endif //SENSOR_FUSION_LIDARMEASUREMENTMODEL_HPP
