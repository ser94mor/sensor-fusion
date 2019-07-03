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

#include "CTRVStateVectorView.hpp"

namespace ser94mor
{
  namespace sensor_fusion
  {


    CTRVROStateVectorView::CTRVROStateVectorView(const CTRVStateVector& sv)
        : ROStateVectorView<CTRVStateVector>{sv}
    {

    }


    double_t CTRVROStateVectorView::px() const
    {
      return GetVector()(0);
    }

    double_t CTRVROStateVectorView::py() const
    {
      return GetVector()(1);
    }

    double_t CTRVROStateVectorView::vx() const
    {
      return v() * std::cos(yaw());
    }

    double_t CTRVROStateVectorView::vy() const
    {
      return v() * std::sin(yaw());
    }

    double_t CTRVROStateVectorView::v() const
    {
      return GetVector()(2);
    }

    double_t CTRVROStateVectorView::yaw() const
    {
      return GetVector()(3);
    }

    double_t CTRVROStateVectorView::yaw_rate() const
    {
      return GetVector()(4);
    }

    double_t CTRVROStateVectorView::range() const
    {
      const double_t rho{std::sqrt(px()*px() + py()*py())};
      return (rho < kEpsilon) ? kEpsilon : rho;
    }

    double_t CTRVROStateVectorView::bearing() const
    {
      return std::atan2(py(), px());
    }

    double_t CTRVROStateVectorView::range_rate() const
    {
      return (px()*vx() + py()*vy()) / range();
    }

    CTRVROProcessNoiseVectorView::CTRVROProcessNoiseVectorView(const CTRVProcessNoiseVector& pnv)
        : ROProcessNoiseVectorView<CTRVProcessNoiseVector>{pnv}
    {

    }

    double_t CTRVROProcessNoiseVectorView::longitudinal_acceleration() const
    {
      return GetVector()(0);
    }

    double_t CTRVROProcessNoiseVectorView::yaw_acceleration() const
    {
      return GetVector()(1);
    }
  }
}
