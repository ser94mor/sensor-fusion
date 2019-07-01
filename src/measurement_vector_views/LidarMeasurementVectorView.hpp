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

    /**
     * A read-only wrapper around MeasurementVector for Lidar measurement model
     * that provides meaningful accessors to the MeasurementVector components.
     */
    class LidarROMeasurementVectorView : ROMeasurementVectorView<LidarMeasurementVector>
    {
    public:
      /**
       * Constructor.
       * @param mv a measurement vector
       */
      explicit LidarROMeasurementVectorView(const LidarMeasurementVector& mv)
      : ROMeasurementVectorView<LidarMeasurementVector>{mv}
      {

      }

      /**
       * @return X-axis coordinate
       */
      virtual double_t px() const override
      {
        return GetVector()(0);
      }

      /**
       * @return Y-axis coordinate
       */
      virtual double_t py() const override
      {
        return GetVector()(1);
      }
    };

    /**
     * A read-write wrapper around MeasurementVector for Lidar measurement model
     * that provides meaningful accessors and setters to the MeasurementVector components.
     */
    class LidarRWMeasurementVectorView : RWMeasurementVectorView<LidarMeasurementVector>
    {
    public:
      /**
       * Constructor.
       * @param measurement_vector a measurement vector
       */
      explicit LidarRWMeasurementVectorView(LidarMeasurementVector& measurement_vector)
      : RWMeasurementVectorView<LidarMeasurementVector>{measurement_vector}
      {

      }

      /**
       * @return X-axis coordinate
       */
      double_t& px() const
      {
        return GetVector()(0);
      }

      /**
       * @return Y-axis coordinate
       */
      double_t& py() const
      {
        return GetVector()(1);
      }
    };

  }
}

#endif //SENSOR_FUSION_LIDARMEASUREMENTVECTORVIEW_HPP
