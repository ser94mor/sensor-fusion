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

//#ifndef SENSOR_FUSION_PROCESSMODEL_HPP
//#define SENSOR_FUSION_PROCESSMODEL_HPP
//
//
//#include "ProcessModel.hpp"
//
//#include <memory>
//
//namespace ser94mor::sensor_fusion
//{
//
//  class ProcessModelFactory
//  {
//  public:
//    static std::unique_ptr<ProcessModel> FromJson(const char* json_str);
//  };
//
//}
//
//
//
//
//template<typename SensorT>
//SensorT SensorFactory<SensorT>::FromJson(const char* json_str)
//{
//  Eigen::Matrix<double, SensorT::Dims(), SensorT::Dims()> measurement_covariance_matrix;
//
//  auto j = json::parse(json_str);
//
//  if (j["type"].get<std::string>() != SensorT::Type() or j["name"].get<std::string>() != SensorT::Name())
//  {
//    throw std::runtime_error("invalid json string");
//  }
//
//  auto mtx{j["measurement_covariance_matrix"].get<std::array<std::array<double, SensorT::Dims()>, SensorT::Dims()>>()};
//
//  for (int r = 0; r < mtx.size(); ++r)
//  {
//    for (int c = 0; c < mtx[0].size(); ++c)
//    {
//      measurement_covariance_matrix(r, c) = mtx[r][c];
//    }
//  }
//
//  return SensorT{measurement_covariance_matrix};
//}
//
//template<typename SensorT>
//SensorT SensorFactory<SensorT>::FromParams(const Eigen::Matrix<double, SensorT::Dims(), SensorT::Dims()>& mtx)
//{
//  return SensorT{mtx};
//}
//
//
//#endif //SENSOR_FUSION_PROCESSMODEL_HPP
