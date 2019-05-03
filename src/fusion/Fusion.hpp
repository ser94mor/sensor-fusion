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


#include "definitions.hpp"

#include <tuple>


namespace ser94mor
{
  namespace  sensor_fusion
  {

    /**
     * A top-level class which is responsible for sensor fusion operations. It is a template class. It accepts
     * as a template arguments a process model class and an arbitrary number of measurement models. The Fusion
     * class implements Predict-Update cycle for all variations of Kalman filters.
     *
     * @tparam Filter a template class for of concrete kind of a filter
     * @tparam ProcessModel a concrete process model
     * @tparam MeasurementModel concrete measurement models (arbitrary number)
     */
    template<template<class, class> class Filter, class ProcessModel,
        template<class> class... MeasurementModel>
    class Fusion
    {
      using Belief = typename ProcessModel::Belief_type;

    public:
      /**
       * Constructor.
       * @param process_noise_covariance_matrix an process noise covariance matrix
       * @param measurement_covariance_matrices
       */
      Fusion(typename ProcessModel::ProcessNoiseCovarianceMatrix_type process_noise_covariance_matrix,
             typename MeasurementModel<ProcessModel>::MeasurementCovarianceMatrix_type...
             measurement_covariance_matrices);

      template <class Measurement>
      Belief ProcessMeasurement(Measurement& measurement);

      void SetBelief(const Belief& belief);
      const Belief& GetBelief() const;

    private:
      template<class TupleOfMeasurementConvarianceMatrices, std::size_t... Is>
      void InitializeMeasurementCovarianceMatrices(
          const TupleOfMeasurementConvarianceMatrices& tup, std::index_sequence<Is...>);

      template<class Measurement_type, class MeasurementModel_type>
      auto ProcessMeasurement(Measurement_type& measurement, MeasurementModel_type& measurement_model)
      -> std::enable_if_t<Measurement_type::MeasurementModelKind() == MeasurementModel_type::Kind(), void>
      {
        using ControlVector = typename ProcessModel::ControlVector_type;

        if (processed_measurements_counter_ == 0)
        {
          belief_ = measurement_model.GetInitialBeliefBasedOn(measurement);
        }
        else
        {
          belief_ = Filter<ProcessModel, MeasurementModel_type>::
                      PredictUpdate(belief_, ControlVector::Zero(), measurement, process_model_, measurement_model);
        }

        ++processed_measurements_counter_;
      }

      template<class Measurement_type, class MeasurementModel_type>
      auto ProcessMeasurement(Measurement_type&, MeasurementModel_type&)
      -> std::enable_if_t<Measurement_type::MeasurementModelKind() != MeasurementModel_type::Kind(), void>
      {
        // Do nothing.
      }

      // counter of processed measurements
      uint64_t processed_measurements_counter_;

      Belief belief_;
      ProcessModel process_model_;
      std::tuple<MeasurementModel<ProcessModel>...> measurement_models_;
    };


    template<template<class, class> class Filter, class ProcessModel,
        template<class> class... MeasurementModel>
    Fusion<Filter, ProcessModel, MeasurementModel...>::
      Fusion(typename ProcessModel::ProcessNoiseCovarianceMatrix_type process_noise_covariance_matrix,
             typename MeasurementModel<ProcessModel>::MeasurementCovarianceMatrix_type...
             measurement_covariance_matrices) :
        processed_measurements_counter_{0},
        belief_{0, {}, {}},
        process_model_{},
        measurement_models_{}
    {

      process_model_.SetProcessNoiseCovarianceMatrix(process_noise_covariance_matrix);

      InitializeMeasurementCovarianceMatrices(
          std::forward_as_tuple(measurement_covariance_matrices...),
          std::index_sequence_for<MeasurementModel<ProcessModel>...>{});
    }

    template<template<class, class> class Filter, class ProcessModel,
        template<class> class... MeasurementModel>
    template<class Tuple_Of_MeasurementConvarianceMatrix, size_t... Is>
    void Fusion<Filter, ProcessModel, MeasurementModel...>::InitializeMeasurementCovarianceMatrices(
        const Tuple_Of_MeasurementConvarianceMatrix& tup, std::index_sequence<Is...>)
    {
      // For C++17:
      // ((std::get<Is>(measurement_model_sensor_map_).first.SetMeasurementCovarianceMatrix(std::get<Is>(tup))), ...);

      // For C++14:
      (void) std::initializer_list<int>
      {
        (std::get<Is>(measurement_models_).SetMeasurementCovarianceMatrix(std::get<Is>(tup)), void(), 0)...
      };
    }

    template<template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
    template<class Measurement>
    typename Fusion<Filter, ProcessModel, MeasurementModel...>::Belief
    Fusion<Filter, ProcessModel, MeasurementModel...>::ProcessMeasurement(Measurement& measurement)
    {
      ser94mor::sensor_fusion::apply(
        [this, &measurement](auto... measurement_model)
        {
          (void) std::initializer_list<int>{(this->ProcessMeasurement(measurement, measurement_model), void(), 0)...};
        },
        measurement_models_
      );

      return belief_;
    }

    template<template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
    void Fusion<Filter, ProcessModel, MeasurementModel...>::SetBelief(const Belief& belief)
    {
      belief_ = belief;
    }

    template<template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
    const typename Fusion<Filter, ProcessModel, MeasurementModel...>::Belief&
    Fusion<Filter, ProcessModel, MeasurementModel...>::GetBelief() const
    {
      return belief_;
    }

  }
}



#endif //SENSOR_FUSION_FUSION_HPP
