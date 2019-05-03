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


#include "definitions.hpp"
#include "MeasurementVectorView.hpp"

#include <cmath>


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace Radar
    {

      /**
       * A read-only wrapper around MeasurementVector for Radar measurement model
       * that provides meaningful accessors to the MeasurementVector components.
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
        double_t px() const override
        {
          return measurement_vector_(0) * std::cos(measurement_vector_(1));
        }

        /**
         * @return Y-axis coordinate
         */
        double_t py() const override
        {
          return measurement_vector_(0) * std::sin(measurement_vector_(1));
        }

        /**
         * @return range: radial distance from origin
         */
        double_t range() const
        {
          return measurement_vector_(0);
        }

        /**
         * @return bearing: angle between range and X-axis
         * (which points into the direction of heading of our car, where sensors are installed)
         */
        double_t bearing() const
        {
          return measurement_vector_(1);
        }

        /**
         * @return radial velocity: change of range, i.e., range rate
         */
        double_t range_rate() const
        {
          return measurement_vector_(2);
        }

      };


      /**
       * A read-write wrapper around MeasurementVector for Radar measurement model
       * that provides meaningful accessors and setters to the MeasurementVector components.
       */
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
         * @return range: radial distance from origin
         */
        double_t& range()
        {
          return measurement_vector_modifiable_(0);
        }

        /**
         * @return bearing: angle between range and X-axis
         * (which points into the direction of heading of our car, where sensors are installed)
         */
        double_t& bearing()
        {
          return measurement_vector_modifiable_(1);
        }

        /**
         * @return radial velocity: change of range, i.e., range rate
         */
        double_t& range_rate()
        {
          return measurement_vector_modifiable_(2);
        }
      };

    }
  }
}


#endif //SENSOR_FUSION_RADARMEASUREMENTVECTORVIEW_HPP
