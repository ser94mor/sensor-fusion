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

#ifndef SENSOR_FUSION_DEFINITIONS_HPP
#define SENSOR_FUSION_DEFINITIONS_HPP


#include <tuple>
#include <Eigen/Dense>

namespace ser94mor
{
  namespace sensor_fusion
  {

    ////////////////////
    // process models //
    ////////////////////
    const char kProcessModelType[]{"PROCESS_MODEL"};
    const char kSensorType[]{"SENSOR"};
    const char kMeasurementModelType[]{"MEASUREMENT_MODEL"};
    using IndividualNoiseProcessesCovarianceMatrix = Eigen::Matrix<double, 2, 2>;


    /////////////
    // sensors //
    /////////////
    const int kMaxSensors = 2;


    //////////////
    // controls //
    //////////////
    const int kMaxControlDims = -1;
    using ControlVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxControlDims, 1>;
    using ControlMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxControlDims, kMaxControlDims>;


    ////////////////////
    // PROCESS MODELS //
    ////////////////////

    namespace CV
    {
      extern const char kProcessModelName[];
      const int kStateVectorDims{4};
      const int kControlVectorDims{4};
      const bool kIsLinear{true};
      using StateVector             = Eigen::Matrix<double, kStateVectorDims, 1>;
      using StateCovarianceMatrix   = Eigen::Matrix<double, kStateVectorDims, kStateVectorDims>;
      using StateTransitionMatrix   = Eigen::Matrix<double, kStateVectorDims, kStateVectorDims>;
      using ControlVector           = Eigen::Matrix<double, kControlVectorDims, 1>;
      using ControlTransitionMatrix = Eigen::Matrix<double, kStateVectorDims, kControlVectorDims>;
      using ProcessCovarianceMatrix = Eigen::Matrix<double, kStateVectorDims, kStateVectorDims>;
    }

    namespace CTRV
    {
      extern const char kProcessModelName[];
      const int kStateVectorDims{5};
      const int kControlVectorDims{5};
      const bool kIsLinear{false};
      using StateVector             = Eigen::Matrix<double, kStateVectorDims, 1>;
      using StateCovarianceMatrix   = Eigen::Matrix<double, kStateVectorDims, kStateVectorDims>;
      using StateTransitionMatrix   = Eigen::Matrix<double, kStateVectorDims, kStateVectorDims>;
      using ControlVector           = Eigen::Matrix<double, kControlVectorDims, 1>;
      using ControlTransitionMatrix = Eigen::Matrix<double, kStateVectorDims, kControlVectorDims>;
      using ProcessCovarianceMatrix = Eigen::Matrix<double, kStateVectorDims, kStateVectorDims>;
    }


    ////////////////////////
    // MEASUREMENT MODELS //
    ////////////////////////

#define MEASUREMENT_MODEL_DEFINITIONS(measurement_vector_dims, is_linear) \
  extern const char kSensorName[]; \
  const int kMeasurementVectorDims = 2; \
  const bool kIsLinear{is_linear}; \
  using MeasurementVector = Eigen::Matrix<double, kMeasurementVectorDims, 1>; \
  using MeasurementCovarianceMatrix = Eigen::Matrix<double, kMeasurementVectorDims, kMeasurementVectorDims>

    namespace Lidar
    {
      MEASUREMENT_MODEL_DEFINITIONS(2, true);
    }

    namespace Radar
    {
      MEASUREMENT_MODEL_DEFINITIONS(3, false);
    }

    enum class MeasurementModelKind
    {
      Lidar = 0,
      Radar = 1,
    };

    constexpr const char* MeasurementModelNameByKind(MeasurementModelKind mm)
    {
      switch (mm)
      {
        case MeasurementModelKind::Lidar:
          return "LIDAR";
        case MeasurementModelKind::Radar:
          return "RADAR";
        default:
          return "!!!___UNDEFINED_MEASUREMENT_MODEL__!!!";
      }
    }


    /////////////
    // HELPERS //
    /////////////

    namespace detail {
      template <class F, class Tuple, size_t... I>
      constexpr decltype(auto) apply_impl(F&& f, Tuple&& t, std::index_sequence<I...>)
      {
        return (f(std::get<I>(t)...));
      }
    }  // namespace detail

    template <class F, class Tuple>
    constexpr decltype(auto) apply(F&& f, Tuple&& t)
    {
      return detail::apply_impl(
          std::forward<F>(f), std::forward<Tuple>(t),
          std::make_index_sequence<std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
    }

  }
}


#endif //SENSOR_FUSION_DEFINITIONS_HPP
