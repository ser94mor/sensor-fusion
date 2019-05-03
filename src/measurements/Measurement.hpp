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


/**
 * A macros to be used to define concrete Measurement classes. Each concrete measurement class "lives"
 * in its own namespace corresponding to a name of the measurement model,
 * so that it is possible to rely on the similar class names for measurement vector and measurment covariance matrix.
 */
#define MEASUREMENT_DEFINITION(mmk) \
  class Measurement : public ser94mor::sensor_fusion::Measurement<MeasurementVector, MeasurementCovarianceMatrix, mmk> \
  { \
  public: \
    Measurement(double_t timestamp, const MeasurementVector& measurement_vector) \
    : ser94mor::sensor_fusion::Measurement<MeasurementVector, MeasurementCovarianceMatrix, mmk> \
        {timestamp, measurement_vector} { } \
  }


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A template class representing a measurement.
     * @tparam MeasurementVector a measurement vector class
     * @tparam MeasurementCovarianceMatrix a measurement covariance matrix class
     * @tparam mmk a measurement model kind
     */
    template<class MeasurementVector, class MeasurementCovarianceMatrix, MeasurementModelKind mmk>
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
      const MeasurementVector& z() const
      {
        return measurement_vector_;
      }

      /**
       * @return a measurement model kind, which is determined by the sensor and measurement model which handles such
       *         a measurement
       */
      constexpr static enum MeasurementModelKind MeasurementModelKind()
      {
        return mmk;
      }

      /**
       * Constructor.
       * @param timestamp a timestamp
       * @param measurement_vector a measurement vector
       */
      Measurement(double_t timestamp, const MeasurementVector& measurement_vector)
      : timestamp_{timestamp}, measurement_vector_{measurement_vector}
      {

      }

    private:
      double_t timestamp_;
      MeasurementVector measurement_vector_;
    };

  }

}


#endif //SENSOR_FUSION_MEASUREMENT_HPP
