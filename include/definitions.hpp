/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
 *
 * Permission is hereby granted, free of charge, to any person 
 * obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
      ProcessModel = 0,
      MeasurementModel = 1,
      Sensor = 2,
    };

    inline constexpr const char* EntityNameByType(EntityType et)
    {
      const char* ret{nullptr};
      switch (et)
      {
        case EntityType::ProcessModel:
        {
          ret = "PROCESS_MODEL";
          break;
        }
        case EntityType::MeasurementModel:
        {
          ret = "MEASUREMENT_MODEL";
          break;
        }
        case EntityType::Sensor:
        {
          ret = "SENSOR";
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
      CV = 0,
      CTRV = 1,
    };

    inline constexpr const char* NameByKind(PMKind pmk)
    {
      const char* ret{nullptr};
      switch (pmk)
      {
        case PMKind::CV:
        {
          ret = "CV";
          break;
        }
        case PMKind::CTRV:
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



    namespace CTRV
    {
      const size_t kStateVectorDims{5};
      const size_t kControlVectorDims{5};
      const size_t kProcessNoiseVectorDims{2};
      const bool kIsLinear{false};

      using StateVector                  = Eigen::Matrix<double_t, kStateVectorDims, 1>;
      using StateCovarianceMatrix        = Eigen::Matrix<double_t, kStateVectorDims, kStateVectorDims>;
      using StateTransitionMatrix        = Eigen::Matrix<double_t, kStateVectorDims, kStateVectorDims>;
      using ControlVector                = Eigen::Matrix<double_t, kControlVectorDims, 1>;
      using ControlTransitionMatrix      = Eigen::Matrix<double_t, kStateVectorDims, kControlVectorDims>;
      using ProcessCovarianceMatrix      = Eigen::Matrix<double_t, kStateVectorDims, kStateVectorDims>;
      using ProcessNoiseVector           = Eigen::Matrix<double_t, kProcessNoiseVectorDims, 1>;
      using ProcessNoiseCovarianceMatrix = Eigen::Matrix<double_t, kProcessNoiseVectorDims, kProcessNoiseVectorDims>;
    }


    ////////////////////////
    // MEASUREMENT MODELS //
    ////////////////////////

    namespace Lidar
    {
      const size_t kMeasurementVectorDims{2};
      const bool kIsLinear{true};

      using MeasurementVector = Eigen::Matrix<double_t, kMeasurementVectorDims, 1>;
      using MeasurementCovarianceMatrix = Eigen::Matrix<double_t, kMeasurementVectorDims, kMeasurementVectorDims>;
    }

    namespace Radar
    {
      const size_t kMeasurementVectorDims{3};
      const bool kIsLinear{false};

      using MeasurementVector = Eigen::Matrix<double_t, kMeasurementVectorDims, 1>;
      using MeasurementCovarianceMatrix = Eigen::Matrix<double_t, kMeasurementVectorDims, kMeasurementVectorDims>;
    }

    enum class MMKind
    {
      Lidar = 0,
      Radar = 1,
    };

    inline constexpr const char* NameByKind(MMKind mm)
    {
      const char* ret{nullptr};
      switch (mm)
      {
        case MMKind::Lidar:
        {
          ret = "LIDAR";
          break;
        }
        case MMKind::Radar:
        {
          ret = "RADAR";
          break;
        }
        default:
          throw std::logic_error("undefined measurement model kind");
      }

      return ret;
    }


    /////////////
    // SENSORS //
    /////////////

    enum class SensorKind
    {
      Lidar = 0,
      Radar = 1,
    };

    inline constexpr const char* NameByKind(SensorKind sk)
    {
      const char* ret{nullptr};
      switch (sk)
      {
        case SensorKind::Lidar:
        {
          ret = "LIDAR";
          break;
        }
        case SensorKind::Radar:
        {
          ret = "RADAR";
          break;
        }
        default:
          throw std::logic_error("undefined sensor kind");
      }

      return ret;
    }


    template <EntityType type, class Kind_type, Kind_type kind>
    class Entity
    {
    public:

      constexpr static EntityType Type();

      constexpr static const char* TypeName();

      constexpr static Kind_type Kind();

      constexpr static const char* KindName();
    };

    template<EntityType type, class Kind_type, Kind_type kind>
    constexpr EntityType Entity<type, Kind_type, kind>::Type()
    {
      return type;
    }

    template<EntityType type, class Kind_type, Kind_type kind>
    constexpr const char* Entity<type, Kind_type, kind>::TypeName()
    {
      return EntityNameByType(Type());
    }

    template<EntityType type, class Kind_type, Kind_type kind>
    constexpr Kind_type Entity<type, Kind_type, kind>::Kind()
    {
      return kind;
    }

    template<EntityType type, class Kind_type, Kind_type kind>
    constexpr const char* Entity<type, Kind_type, kind>::KindName()
    {
      return NameByKind(Kind());
    }

    template <EntityType type, class Kind_type, Kind_type kind, bool is_linear>
    class ModelEntity : public Entity<type, Kind_type, kind>
    {
    public:
      constexpr static bool IsLinear();
    };

    template<EntityType type, class Kind_type, Kind_type kind, bool is_linear>
    constexpr bool ModelEntity<type, Kind_type, kind, is_linear>::IsLinear()
    {
      return is_linear;
    }

    /**
     * A base class for read-write vector views.
     * Read-write vector views provide read-write meaningful access to vector dimensions.
     *
     * @tparam Vector a class of the vector
     */
    template<class Vector>
    class RWVectorView
    {
    protected:
      /**
       * Constructor.
       * @param vector a vector
       */
      explicit RWVectorView(Vector& vector) : vector_{vector}
      {

      }

      /**
       * Destructor.
       */
      virtual ~RWVectorView() = default;

      /**
       * @return reference to a vector
       */
      Vector& GetVector() const
      {
        return vector_;
      }

    private:
      Vector& vector_;
    };

    /**
     * A base class for read-only vector views.
     * Read-only vector views provide read-only meaningful access to vector dimensions.
     *
     * @tparam Vector a class of the vector
     */
    template<class Vector>
    class ROVectorView
    {
    protected:
      /**
       * Constructor.
       * @param vector a vector
       */
      explicit ROVectorView(const Vector& vector) : vector_{vector}
      {

      }

      /**
       * Destructor.
       */
      virtual ~ROVectorView() = default;

      /**
       * @return const reference to a vector
       */
      const Vector& GetVector() const
      {
        return vector_;
      }

    private:
      const Vector& vector_;
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
