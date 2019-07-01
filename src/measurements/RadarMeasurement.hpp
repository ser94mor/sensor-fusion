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

#ifndef SENSOR_FUSION_RADARMEASUREMENT_HPP
#define SENSOR_FUSION_RADARMEASUREMENT_HPP


namespace ser94mor
{
  namespace sensor_fusion
  {

    using RadarMeasurementBase =
    ser94mor::sensor_fusion::Measurement<RadarMeasurementVector, RadarMeasurementCovarianceMatrix, MMKind::Radar>;

    class RadarMeasurement : public RadarMeasurementBase
    {
    public:

      /**
       * Constructor.
       *
       * @param t a timestamp
       * @param mv a measurement vector
       */
      RadarMeasurement(double_t t, const RadarMeasurementVector& mv) : RadarMeasurementBase{t, mv}
      {

      }

    };

  }
}


#endif //SENSOR_FUSION_RADARMEASUREMENT_HPP
