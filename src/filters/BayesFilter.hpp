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

#ifndef SENSOR_FUSION_BAYESFILTER_HPP
#define SENSOR_FUSION_BAYESFILTER_HPP


#include "SensorData.hpp"
#include "State.hpp"


class BayesFilter {

public:
  template <int dims> 
  void Predict(State<dims>& state)
  {

  }

  template <int state_dims, int sensor_data_dims>
  void Update(State<state_dims>& state, const SensorData<sensor_data_dims>& sensor_data)
  {
    
  }
  
protected:
  

};


#endif //SENSOR_FUSION_BAYESFILTER_HPP
