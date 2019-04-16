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


#include <Eigen/Dense>


namespace ser94mor
{
  namespace sensor_fusion
  {

    enum class EntityType
    {
      ProcessModel = 0,
      MeasurementModel = 1,
      Sensor = 2,
    };

    constexpr const char* EntityNameByType(EntityType et)
    {
      switch (et)
      {
        case EntityType::ProcessModel:
          return "PROCESS_MODEL";
        case EntityType::MeasurementModel:
          return "MEASUREMENT_MODEL";
        case EntityType::Sensor:
          return "SENSOR";
        default:
          return "!!!___UNDEFINED_ENTITY_TYPE___!!!";
      }
    }


    ////////////////////
    // PROCESS MODELS //
    ////////////////////

    enum class ProcessModelKind
    {
      CV = 0,
      CTRV = 1,
    };

    constexpr const char* NameByKind(ProcessModelKind pmk)
    {
      switch (pmk)
      {
        case ProcessModelKind::CV:
          return "CV";
        case ProcessModelKind::CTRV:
          return "CTRV";
        default:
          return "!!!___UNDEFINED_PROCESS_MODEL_KIND___!!!";
      }
    }

    using IndividualNoiseProcessesCovarianceMatrix = Eigen::Matrix<double, 2, 2>;

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

    constexpr const char* NameByKind(MeasurementModelKind mm)
    {
      switch (mm)
      {
        case MeasurementModelKind::Lidar:
          return "LIDAR";
        case MeasurementModelKind::Radar:
          return "RADAR";
        default:
          return "!!!___UNDEFINED_MEASUREMENT_MODEL___!!!";
      }
    }


    /////////////
    // SENSORS //
    /////////////

    enum class SensorKind
    {
      Lidar = 0,
      Radar = 1,
    };

    constexpr const char* NameByKind(SensorKind sk)
    {
      switch (sk)
      {
        case SensorKind::Lidar:
          return "LIDAR";
        case SensorKind::Radar:
          return "RADAR";
        default:
          return "!!!___UNDEFINED_SENSOR___!!!";
      }
    }


    template <EntityType type, class Kind_type, Kind_type kind>
    class Entity
    {
    public:

      constexpr static EntityType Type()
      {
        return type;
      }

      constexpr static const char* TypeName()
      {
        return EntityNameByType(Type());
      }

      constexpr static Kind_type Kind()
      {
        return kind;
      }

      constexpr static const char* KindName()
      {
        return NameByKind(Kind());
      }
    };

    template <EntityType type, class Kind_type, Kind_type kind, bool is_linear>
    class ModelEntity : public Entity<type, Kind_type, kind>
    {
    public:
      constexpr static bool IsLinear()
      {
        return is_linear;
      }
    };


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
