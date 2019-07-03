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

#include "LidarMeasurementVectorView.hpp"

namespace ser94mor
{
  namespace sensor_fusion
  {


    LidarROMeasurementVectorView::LidarROMeasurementVectorView(const LidarMeasurementVector& mv)
    : ROMeasurementVectorView<LidarMeasurementVector>{mv}
    {

    }

    double_t LidarROMeasurementVectorView::px() const
    {
      return GetVector()(0);
    }


    double_t LidarROMeasurementVectorView::py() const
    {
      return GetVector()(1);
    }

    LidarRWMeasurementVectorView::LidarRWMeasurementVectorView(LidarMeasurementVector& mv)
        : RWMeasurementVectorView<LidarMeasurementVector>{mv}
    {

    }

    double_t& LidarRWMeasurementVectorView::px() const
    {
      return GetVector()(0);
    }

    double_t& LidarRWMeasurementVectorView::py() const
    {
      return GetVector()(1);
    }

  }
}
