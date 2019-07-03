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

#include "CVStateVectorView.hpp"

namespace ser94mor
{
  namespace sensor_fusion
  {


    CVRWStateVectorView::CVRWStateVectorView(CVStateVector& sv) : RWStateVectorView<CVStateVector>{sv}
    {

    }

    double_t& CVRWStateVectorView::px() const
    {
      return GetVector()(0);
    }

    double_t& CVRWStateVectorView::py() const
    {
      return GetVector()(1);
    }

    double_t& CVRWStateVectorView::vx() const
    {
      return GetVector()(2);
    }

    double_t& CVRWStateVectorView::vy() const
    {
      return GetVector()(3);
    }

    CVROStateVectorView::CVROStateVectorView(const CVStateVector& sv) : ROStateVectorView<CVStateVector>{sv}
    {

    }

    double_t CVROStateVectorView::px() const
    {
      return GetVector()(0);
    }

    double_t CVROStateVectorView::py() const
    {
      return GetVector()(1);
    }

    double_t CVROStateVectorView::vx() const
    {
      return GetVector()(2);
    }

    double_t CVROStateVectorView::vy() const
    {
      return GetVector()(3);
    }

    double_t CVROStateVectorView::v() const
    {
      return std::sqrt(vx()*vx() + vy()*vy());
    }

    double_t CVROStateVectorView::yaw() const
    {
      return std::acos(vx()/v());
    }

    double_t CVROStateVectorView::yaw_rate() const
    {
      return 0.0;
    }

    double_t CVROStateVectorView::range() const
    {
      const double_t rho{std::sqrt(px()*px() + py()*py())};
      return (rho < kEpsilon) ? kEpsilon : rho;
    }

    double_t CVROStateVectorView::bearing() const
    {
      return std::atan2(py(), px());
    }

    double_t CVROStateVectorView::range_rate() const
    {
      return (px()*vx() + py()*vy()) / range();
    }
  }
}

