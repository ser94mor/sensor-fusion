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

#ifndef SENSOR_FUSION_MEASUREMENT_HPP
#define SENSOR_FUSION_MEASUREMENT_HPP


#include <ctime>


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A template class representing a measurement.
     * @tparam MeasurementVector_t a measurement vector class
     * @tparam MeasurementCovarianceMatrix_t a measurement covariance matrix class
     * @tparam mmk a measurement model kind
     */
    template<class MeasurementVector_t, class MeasurementCovarianceMatrix_t, MMKind mmk>
    class Measurement
    {
    public:

      /**
       * @return a measurement timestamp
       */
      double_t t() const
      {
        return timestamp_;
      }

      /**
       * A naming for measurement vector is taken from the
       * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
       * @return a measurement vector
       */
      const MeasurementVector_t& z() const
      {
        return measurement_vector_;
      }

      /**
       * @return a measurement model kind, which is determined by the sensor and measurement model which handles such
       *         a measurement
       */
      constexpr static enum MMKind MeasurementModelKind()
      {
        return mmk;
      }

      /**
       * Constructor.
       *
       * @param tm a timestamp
       * @param mv a measurement vector
       */
      Measurement(double_t tm, const MeasurementVector_t& mv) : timestamp_{tm}, measurement_vector_{mv}
      {

      }

    private:
      double_t timestamp_;
      MeasurementVector_t measurement_vector_;
    };

  }

}


#endif //SENSOR_FUSION_MEASUREMENT_HPP
