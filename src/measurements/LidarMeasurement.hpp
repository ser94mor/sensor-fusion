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

#ifndef SENSOR_FUSION_LIDARMEASUREMENT_HPP
#define SENSOR_FUSION_LIDARMEASUREMENT_HPP


#include "definitions.hpp"
#include "Measurement.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {

    using LidarMeasurementBase = Measurement<LidarMeasurementVector, LidarMeasurementCovarianceMatrix, MMKind::e_Lidar>;

    class LidarMeasurement : public LidarMeasurementBase
    {
    public:

      /**
       * Constructor.
       *
       * @param tm a timestamp
       * @param mv a measurement vector
       */
      LidarMeasurement(double_t tm, const LidarMeasurementVector& mv) : LidarMeasurementBase{tm, mv}
      {

      }

    };

  }
}

#endif //SENSOR_FUSION_LIDARMEASUREMENT_HPP
