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

#ifndef SENSOR_FUSION_RADARMEASUREMENTVECTORVIEW_HPP
#define SENSOR_FUSION_RADARMEASUREMENTVECTORVIEW_HPP


#include "MeasurementModel.hpp"

#include <cmath>


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace Radar
    {

      /**
       * A measurement vector view for the Radar measurement vector, that is,
       * it is a class  that provides a meaningful access to the
       * Radar measurement vector dimensions.
       */
      class MeasurementVectorView : ser94mor::sensor_fusion::MeasurementVectorView<MeasurementVector>
      {
      public:
        /**
         * Constructor.
         * @param measurement_vector a measurement vector
         */
        explicit MeasurementVectorView(const MeasurementVector& measurement_vector)
        : ser94mor::sensor_fusion::MeasurementVectorView<MeasurementVector>{measurement_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double px() const override
        {
          return measurement_vector_(0) * std::cos(measurement_vector_(1));
        }

        /**
         * @return Y-axis coordinate
         */
        double py() const override
        {
          return measurement_vector_(0) * std::sin(measurement_vector_(1));
        }

      };

    }
  }
}


#endif //SENSOR_FUSION_RADARMEASUREMENTVECTORVIEW_HPP
