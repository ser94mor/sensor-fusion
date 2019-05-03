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

#ifndef SENSOR_FUSION_LIDARMEASUREMENTVECTORVIEW_HPP
#define SENSOR_FUSION_LIDARMEASUREMENTVECTORVIEW_HPP


#include "definitions.hpp"
#include "MeasurementVectorView.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace Lidar
    {

      /**
       * A measurement vector view for the Lidar measurement vector, that is,
       * it is a class  that provides a meaningful access to the
       * Lidar measurement vector dimensions.
       */
      class ROMeasurementVectorView : ser94mor::sensor_fusion::ROMeasurementVectorView<MeasurementVector>
      {
      public:
        /**
         * Constructor.
         * @param measurement_vector a measurement vector
         */
        explicit ROMeasurementVectorView(const MeasurementVector& measurement_vector)
        : ser94mor::sensor_fusion::ROMeasurementVectorView<MeasurementVector>{measurement_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double_t px() const
        {
          return measurement_vector_(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double_t py() const
        {
          return measurement_vector_(1);
        }
      };

      class RWMeasurementVectorView : ser94mor::sensor_fusion::RWMeasurementVectorView<MeasurementVector>
      {
      public:
        /**
         * Constructor.
         * @param measurement_vector a measurement vector
         */
        explicit RWMeasurementVectorView(MeasurementVector& measurement_vector)
        : ser94mor::sensor_fusion::RWMeasurementVectorView<MeasurementVector>{measurement_vector}
        {

        }

        /**
         * @return X-axis coordinate
         */
        double_t& px()
        {
          return measurement_vector_modifiable_(0);
        }

        /**
         * @return Y-axis coordinate
         */
        double_t& py()
        {
          return measurement_vector_modifiable_(1);
        }
      };

    }
  }
}

#endif //SENSOR_FUSION_LIDARMEASUREMENTVECTORVIEW_HPP
