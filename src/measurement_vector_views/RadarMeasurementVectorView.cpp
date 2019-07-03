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

#include "RadarMeasurementVectorView.hpp"

namespace ser94mor
{
  namespace sensor_fusion
  {


    RadarROMeasurementVectorView::RadarROMeasurementVectorView(const RadarMeasurementVector& mv)
        : ROMeasurementVectorView<RadarMeasurementVector>{mv}
    {

    }

    double_t RadarROMeasurementVectorView::px() const
    {
      return GetVector()(0) * std::cos(GetVector()(1));
    }

    double_t RadarROMeasurementVectorView::py() const
    {
      return GetVector()(0) * std::sin(GetVector()(1));
    }

    double_t RadarROMeasurementVectorView::range() const
    {
      return GetVector()(0);
    }

    double_t RadarROMeasurementVectorView::bearing() const
    {
      return GetVector()(1);
    }

    double_t RadarROMeasurementVectorView::range_rate() const
    {
      return GetVector()(2);
    }

    RadarRWMeasurementVectorView::RadarRWMeasurementVectorView(RadarMeasurementVector& mv)
        : RWMeasurementVectorView<RadarMeasurementVector>{mv}
    {

    }

    double_t& RadarRWMeasurementVectorView::range() const
    {
      return GetVector()(0);
    }

    double_t& RadarRWMeasurementVectorView::bearing() const
    {
      return GetVector()(1);
    }

    double_t& RadarRWMeasurementVectorView::range_rate() const
    {
      return GetVector()(2);
    }


  }
}
