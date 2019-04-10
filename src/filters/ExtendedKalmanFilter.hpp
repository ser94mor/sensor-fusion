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

#ifndef SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP
#define SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP


#include "KalmanFilter.hpp"


namespace ser94mor::sensor_fusion
{

  template <class Belief, class Measurement>
  class ExtendedKalmanFilter : public KalmanFilter<Belief, Measurement>
  {
  public:
    Belief Predict(const Belief& state, std::time_t dt) override
    {
      std::cout << "EXTENDED KALMAN FILTER" << std::endl;
      return KalmanFilter<Belief, Measurement>::Predict(state, dt);
    }

    Belief Update(const Belief& state, const Measurement& measurement) override
    {
      std::cout << "EXTENDED KALMAN FILTER" << std::endl;
      return state;
    }
  };

}


#endif //SENSOR_FUSION_EXTENDEDKALMANFILTER_HPP
