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

#ifndef SENSOR_FUSION_DEFINITIONS_HEADER_HPP
#define SENSOR_FUSION_DEFINITIONS_HEADER_HPP

#include <Eigen/Dense>


namespace ser94mor
{
  namespace sensor_fusion
  {


    constexpr const double_t kEpsilon{1e-11};

    enum class EntityType
    {
      e_ProcessModel = 0,
      e_MeasurementModel = 1,
    };

    inline constexpr const char* EntityNameByType(EntityType et)
    {
      const char* ret{nullptr};
      switch (et)
      {
        case EntityType::e_ProcessModel:
        {
          ret = "PROCESS_MODEL";
          break;
        }
        case EntityType::e_MeasurementModel:
        {
          ret = "MEASUREMENT_MODEL";
          break;
        }
        default:
          throw std::logic_error("undefined entity type");
      }

      return ret;
    }


    ////////////////////
    // PROCESS MODELS //
    ////////////////////

    enum class PMKind
    {
      e_CV = 0,
      e_CTRV = 1,
    };

    inline constexpr const char* NameByKind(PMKind pmk)
    {
      const char* ret{nullptr};
      switch (pmk)
      {
        case PMKind::e_CV:
        {
          ret = "CV";
          break;
        }
        case PMKind::e_CTRV:
        {
          ret = "CTRV";
          break;
        }
        default:
          throw std::logic_error("undefined process model kind");
      }
      return ret;
    }

    //
    // CV process model definitions.
    //
    const size_t kCVStateVectorDims{4};
    const size_t kCVControlVectorDims{4};
    const size_t kCVProcessNoiseVectorDims{2};
    const bool kCVIsLinear{true};

    using CVStateVector                  = Eigen::Matrix<double_t, kCVStateVectorDims, 1>;
    using CVStateCovarianceMatrix        = Eigen::Matrix<double_t, kCVStateVectorDims, kCVStateVectorDims>;
    using CVStateTransitionMatrix        = Eigen::Matrix<double_t, kCVStateVectorDims, kCVStateVectorDims>;
    using CVControlVector                = Eigen::Matrix<double_t, kCVControlVectorDims, 1>;
    using CVControlTransitionMatrix      = Eigen::Matrix<double_t, kCVStateVectorDims, kCVControlVectorDims>;
    using CVProcessCovarianceMatrix      = Eigen::Matrix<double_t, kCVStateVectorDims, kCVStateVectorDims>;
    using CVProcessNoiseVector           = Eigen::Matrix<double_t, kCVProcessNoiseVectorDims, 1>;
    using CVProcessNoiseCovarianceMatrix = Eigen::Matrix<double_t, kCVProcessNoiseVectorDims, kCVProcessNoiseVectorDims>;



    //
    // CTRV process model definitions.
    //
    const size_t kCTRVStateVectorDims{5};
    const size_t kCTRVControlVectorDims{5};
    const size_t kCTRVProcessNoiseVectorDims{2};
    const bool kCTRVIsLinear{false};

    using CTRVStateVector                  = Eigen::Matrix<double_t, kCTRVStateVectorDims, 1>;
    using CTRVStateCovarianceMatrix        = Eigen::Matrix<double_t, kCTRVStateVectorDims, kCTRVStateVectorDims>;
    using CTRVStateTransitionMatrix        = Eigen::Matrix<double_t, kCTRVStateVectorDims, kCTRVStateVectorDims>;
    using CTRVControlVector                = Eigen::Matrix<double_t, kCTRVControlVectorDims, 1>;
    using CTRVControlTransitionMatrix      = Eigen::Matrix<double_t, kCTRVStateVectorDims, kCTRVControlVectorDims>;
    using CTRVProcessCovarianceMatrix      = Eigen::Matrix<double_t, kCTRVStateVectorDims, kCTRVStateVectorDims>;
    using CTRVProcessNoiseVector           = Eigen::Matrix<double_t, kCTRVProcessNoiseVectorDims, 1>;
    using CTRVProcessNoiseCovarianceMatrix =
        Eigen::Matrix<double_t, kCTRVProcessNoiseVectorDims, kCTRVProcessNoiseVectorDims>;


    ////////////////////////
    // MEASUREMENT MODELS //
    ////////////////////////

    //
    // Lidar measurement model definitions.
    //
    const size_t kLidarMeasurementVectorDims{2};
    const bool kLidarIsLinear{true};

    using LidarMeasurementVector = Eigen::Matrix<double_t, kLidarMeasurementVectorDims, 1>;
    using LidarMeasurementCovarianceMatrix =
        Eigen::Matrix<double_t, kLidarMeasurementVectorDims, kLidarMeasurementVectorDims>;


    //
    // Radar measurement model definitions.
    //
    const size_t kRadarMeasurementVectorDims{3};
    const bool kRadarIsLinear{false};

    using RadarMeasurementVector = Eigen::Matrix<double_t, kRadarMeasurementVectorDims, 1>;
    using RadarMeasurementCovarianceMatrix =
        Eigen::Matrix<double_t, kRadarMeasurementVectorDims, kRadarMeasurementVectorDims>;



    enum class MMKind
    {
      e_Lidar = 0,
      e_Radar = 1,
    };

    inline constexpr const char* NameByKind(MMKind mm)
    {
      const char* ret{nullptr};
      switch (mm)
      {
        case MMKind::e_Lidar:
        {
          ret = "LIDAR";
          break;
        }
        case MMKind::e_Radar:
        {
          ret = "RADAR";
          break;
        }
        default:
          throw std::logic_error("undefined measurement model kind");
      }

      return ret;
    }


    template <EntityType tp, class Kind_t, Kind_t kd>
    class Entity
    {
    public:

      constexpr static EntityType Type();

      constexpr static const char* TypeName();

      constexpr static Kind_t Kind();

      constexpr static const char* KindName();
    };

    template<EntityType tp, class Kind_t, Kind_t kd>
    constexpr EntityType Entity<tp, Kind_t, kd>::Type()
    {
      return tp;
    }

    template<EntityType tp, class Kind_t, Kind_t kd>
    constexpr const char* Entity<tp, Kind_t, kd>::TypeName()
    {
      return EntityNameByType(Type());
    }

    template<EntityType tp, class Kind_t, Kind_t kd>
    constexpr Kind_t Entity<tp, Kind_t, kd>::Kind()
    {
      return kd;
    }

    template<EntityType tp, class Kind_t, Kind_t kd>
    constexpr const char* Entity<tp, Kind_t, kd>::KindName()
    {
      return NameByKind(Kind());
    }

    template <EntityType tp, class Kind_t, Kind_t kd, bool is_linear>
    class ModelEntity : public Entity<tp, Kind_t, kd>
    {
    public:
      constexpr static bool IsLinear();
    };

    template<EntityType tp, class Kind_t, Kind_t kd, bool is_linear>
    constexpr bool ModelEntity<tp, Kind_t, kd, is_linear>::IsLinear()
    {
      return is_linear;
    }

    /**
     * A base class for read-write vector views.
     * Read-write vector views provide read-write meaningful access to vector dimensions.
     *
     * @tparam Vector_t a class of the vector
     */
    template<class Vector_t>
    class RWVectorView
    {
    protected:
      /**
       * Constructor.
       * @param v a vector
       */
      explicit RWVectorView(Vector_t& v) : vector_{v}
      {

      }

      /**
       * Destructor.
       */
      virtual ~RWVectorView() = default;

      /**
       * @return reference to a vector
       */
      Vector_t& GetVector() const
      {
        return vector_;
      }

      Vector_t& vector_;
    };

    /**
     * A base class for read-only vector views.
     * Read-only vector views provide read-only meaningful access to vector dimensions.
     *
     * @tparam Vector_t a class of the vector
     */
    template<class Vector_t>
    class ROVectorView
    {
    protected:
      /**
       * Constructor.
       * @param v a vector
       */
      explicit ROVectorView(const Vector_t& v) : vector_{v}
      {

      }

      /**
       * Destructor.
       */
      virtual ~ROVectorView() = default;

      /**
       * @return const reference to a vector
       */
      const Vector_t& GetVector() const
      {
        return vector_;
      }

    private:
      const Vector_t& vector_;
    };


    /////////////
    // HELPERS //
    /////////////

    namespace detail {
      template <class F, class Tuple, size_t... I>
      constexpr decltype(auto) apply_impl(const F&& f, Tuple&& t, std::index_sequence<I...>)
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


#endif //SENSOR_FUSION_DEFINITIONS_HEADER_HPP
