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

#ifndef SENSOR_FUSION_MEASUREMENTVECTORVIEW_HPP
#define SENSOR_FUSION_MEASUREMENTVECTORVIEW_HPP


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base class for so-called measurement vector views, that is, classes that provides a meaningful access to the
     * measurement vector dimensions.
     *
     * @tparam MeasurementVector a measurement vector class
     */
    template <class MeasurementVector>
    class MeasurementVectorView
    {
    public:
      /**
       * Constructor.
       * @param measurement_vector a measurement vector
       */
      explicit MeasurementVectorView(const MeasurementVector& measurement_vector)
      : measurement_vector_{measurement_vector}
      {

      }

      /**
       * @return X-axis coordinate
       */
      virtual double px() const = 0;

      /**
       * @return Y-axis coordinate
       */
      virtual double py() const = 0;

    protected:
      const MeasurementVector& measurement_vector_;
    };

  }
}


#endif //SENSOR_FUSION_MEASUREMENTVECTORVIEW_HPP
