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

#ifndef SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP
#define SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP


#include "definitions.hpp"
#include "MeasurementModel.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace Radar
    {

      /**
       * A concrete (Radar) measurement model. It is still a template because the dimensionality of
       * the measurement matrix depends on the process model kind, which we know only at compile time.
       * The naming of matrices are taken from the
       * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
       *
       * @tparam ProcessModel a process model class, which is needed to determine the number of state dimensions
       */
      template<class ProcessModel>
      class MeasurementModel
      : public ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix, ProcessModel,
                                                         MeasurementModelKind::Radar, kIsLinear>
      {
      public:
        using MeasurementMatrix_type =
          Eigen::Matrix<double, MeasurementModel::MeasurementDims(), MeasurementModel::StateDims()>;
        using StateVector_type = typename ProcessModel::StateVector_type;
        using StateVectorView_type = typename ProcessModel::StateVectorView_type;


        MeasurementModel()
        : ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix, ProcessModel,
                                                    MeasurementModelKind::Radar, kIsLinear>{}
        {

        }

        MeasurementVector h(const StateVector_type& state_vector) const
        {
          StateVectorView_type svw{state_vector};

          MeasurementVector measurement_vector;
          measurement_vector << svw.range(), svw.bearing(), svw.range_rate();

          return measurement_vector;
        }

        const MeasurementMatrix_type& H(std::time_t dt) const
        {

        }
      };

    }
  }
}


#endif //SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP
