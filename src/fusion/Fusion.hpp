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

#ifndef SENSOR_FUSION_FUSION_HPP
#define SENSOR_FUSION_FUSION_HPP


#include <json.hpp>
#include <tuple>

namespace ser94mor::sensor_fusion
{

  template <class ProcessModel, class Filter, class... Sensor>
  class Fusion
  {
    using Belief = typename ProcessModel::Belief_type;

  public:
    /**
     * Constructor.
     */
    Fusion(std::initializer_list<const char*> json_configs);

    /**
    * Run the whole flow of the Kalman Filter from here.
    */
    [[noreturn]] void Start();

  private:

    // flag indicating whether the first measurement processed
    bool initialized_;

    // counter of processed measurements
    uint64_t meas_counter_;

    // timestamp of the previously processed measurement
    uint64_t prev_meas_timestamp_;


    Belief belief_;
    ProcessModel process_model_;
    Filter filter_;
    std::tuple<Sensor...> sensors_;

  };

  template<class ProcessModel, class Filter, class... Sensor>
  Fusion<ProcessModel, Filter, Sensor...>::Fusion(std::initializer_list<const char*> json_configs)
  {
    for (auto json_str : json_configs)
    {
      auto json = nlohmann::json::parse(json_str);

      
    }
  }

  template<class ProcessModel, class Filter, class... Sensor>
  void Fusion<ProcessModel, Filter, Sensor...>::Start()
  {

  }

}


#endif //SENSOR_FUSION_FUSION_HPP
