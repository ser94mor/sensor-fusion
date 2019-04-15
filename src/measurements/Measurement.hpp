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


#define MEASUREMENT_DEFINITION() \
  class Measurement : public ser94mor::sensor_fusion::Measurement<MeasurementVector, MeasurementCovarianceMatrix> \
  { \
  public: \
    Measurement(std::time_t timestamp, const MeasurementVector& measurement_vector, enum MeasurementModelKind mmk) \
    : ser94mor::sensor_fusion::Measurement<MeasurementVector, MeasurementCovarianceMatrix> \
        {timestamp, measurement_vector, mmk} { } \
  };


namespace ser94mor
{
  namespace sensor_fusion
  {

    template<class MeasurementVector, class MeasurementCovarianceMatrix>
    class Measurement
    {
    public:
      std::time_t t() const
      {
        return timestamp_;
      }

      const MeasurementVector& z() const
      {
        return measurement_vector_;
      }

      enum MeasurementModelKind MeasurementModelKind() const
      {
        return mmk_;
      }

      Measurement(std::time_t timestamp, const MeasurementVector& measurement_vector, enum MeasurementModelKind mmk)
      : timestamp_{timestamp}, measurement_vector_{measurement_vector}, mmk_{mmk}
      {

      }

    private:
      std::time_t timestamp_;
      MeasurementVector measurement_vector_;
      enum MeasurementModelKind mmk_;
    };

  }

}


#endif //SENSOR_FUSION_MEASUREMENT_HPP
