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


#include "Utils.hpp"

#include <cmath>


namespace ser94mor
{
  namespace sensor_fusion
  {

    void Utils::NormalizeAngle(double_t* angle)
    {
      const auto times = std::round(std::fabs(*angle / (2. * M_PI)));  // for the case when angle is very very large

      if (*angle > M_PI)
      {
        *angle -= times * 2.0 * M_PI;
      }

      if (*angle < -M_PI)
      {
        *angle += times * 2.0 * M_PI;
      }
    }

  }
}
