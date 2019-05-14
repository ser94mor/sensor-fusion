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

#ifndef SENSOR_FUSION_UTILS_HPP
#define SENSOR_FUSION_UTILS_HPP


#include <cmath>


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * Class containing different helper methods.
     */
    class Utils
    {
    public:

      /**
       * Normalize angle to be between [-pi, pi].
       * @param angle the pointer to variable containing angle to be normalized.
       */
      static void NormalizeAngle(double_t* angle);

    };

  }
}


#endif //SENSOR_FUSION_UTILS_HPP
