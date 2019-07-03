/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
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

#include "MeasurementModel.hpp"
#include "definitions.hpp"
#include "process_models.hpp"
#include "measurement_vector_views.hpp"

#include <ctime>


namespace ser94mor
{
  namespace  sensor_fusion
  {

    template <class ProcessModel_t>
    using LidarMeasurementModelBase =
        MeasurementModel<LidarMeasurementVector, LidarMeasurementCovarianceMatrix, LidarROMeasurementVectorView,
                         ProcessModel_t, MMKind::e_Lidar, kLidarIsLinear>;

    /**
     * A concrete (Lidar) measurement model. It is still a template because the dimensionality of
     * the measurement matrix depends on the process model kind, which we know only at compile time.
     * The naming of matrices are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     *
     * @tparam ProcessModel_t a process model class, which is needed to determine the number of state dimensions
     */
    template<class ProcessModel_t>
    class LidarMeasurementModel : public LidarMeasurementModelBase<ProcessModel_t>
    {
    public:
      using MeasurementMatrix_type =
          Eigen::Matrix<double_t, LidarMeasurementModel::MeasurementDims(), LidarMeasurementModel::StateDims()>;

      /**
       * Constructor.
       * Measurement matrix is of the form
       *   1 0 0 0 ...
       *   0 1 0 0 ...
       * where the number of columns equal to the number of state dimensions.
       */
      LidarMeasurementModel();

      /**
       * @return a measurement matrix
       */
      auto C() const -> const MeasurementMatrix_type&;

      /**
       * Calculate a difference between two measurement vectors. In Lidar case, it is simply a vector subtraction.
       *
       * @param mv_1 the first measurement vector
       * @param mv_2 the second measurement vector
       * @return the difference between the two measurement vectors
       */
      static LidarMeasurementVector Diff(const LidarMeasurementVector& mv_1,
                                         const LidarMeasurementVector& mv_2);

    private:
      MeasurementMatrix_type measurement_matrix_;
    };



    ////////////////////
    // IMPLEMENTATION //
    ////////////////////

    template<class ProcessModel_t>
    LidarMeasurementModel<ProcessModel_t>::LidarMeasurementModel()
    : LidarMeasurementModelBase<ProcessModel_t>{}, measurement_matrix_{MeasurementMatrix_type::Identity()}
    {

    }

    template<class ProcessModel_t>
    auto LidarMeasurementModel<ProcessModel_t>::C() const -> const MeasurementMatrix_type&
    {
      return measurement_matrix_;
    }

    template<class ProcessModel_t>
    LidarMeasurementVector
    LidarMeasurementModel<ProcessModel_t>::Diff(const LidarMeasurementVector& mv_1, const LidarMeasurementVector& mv_2)
    {
      return (mv_1 - mv_2);
    }


  }
}


#endif //SENSOR_FUSION_LIDARMEASUREMENTMODEL_HPP
