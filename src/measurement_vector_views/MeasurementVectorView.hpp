/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
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


#include "definitions.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base class for read-only wrappers around MeasurementVector for some measurement model
     * that provides meaningful accessors to the MeasurementVector components.
     *
     * @tparam MeasurementVector_t a class of the measurement vector
     */
    template <class MeasurementVector_t>
    class ROMeasurementVectorView : public ROVectorView<MeasurementVector_t>
    {
    public:
      /**
       * @return X-axis coordinate
       */
      virtual double_t px() const = 0;

      /**
       * @return Y-axis coordinate
       */
      virtual double_t py() const = 0;

    protected:
      /**
       * Constructor.
       * @param mv a measurement vector
       */
      explicit ROMeasurementVectorView(const MeasurementVector_t& mv);

      /**
       * Destructor.
       */
      virtual ~ROMeasurementVectorView() = default;
    };

    /**
     * A base class for read-write wrappers around MeasurementVector for some measurement model
     * that provides meaningful accessors and setters to the MeasurementVector components.
     *
     * @tparam MeasurementVector_t a class of the measurement vector
     */
    template <class MeasurementVector_t>
    class RWMeasurementVectorView : public RWVectorView<MeasurementVector_t>
    {
    protected:
      /**
       * Constructor.
       * @param mv a measurement vector
       */
      explicit RWMeasurementVectorView(MeasurementVector_t& mv);

      /**
       * Destructor.
       */
      virtual ~RWMeasurementVectorView() = default;
    };



    ////////////////////
    // IMPLEMENTATION //
    ////////////////////

    template<class MeasurementVector_t>
    RWMeasurementVectorView<MeasurementVector_t>::RWMeasurementVectorView(MeasurementVector_t& mv) : RWVectorView<MeasurementVector_t>{mv}
    {

    }

    template<class MeasurementVector_t>
    ROMeasurementVectorView<MeasurementVector_t>::ROMeasurementVectorView(const MeasurementVector_t& mv) : ROVectorView<MeasurementVector_t>{mv}
    {

    }

  }
}


#endif //SENSOR_FUSION_MEASUREMENTVECTORVIEW_HPP
