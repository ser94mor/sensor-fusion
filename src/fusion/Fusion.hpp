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
     * @tparam FilterTemplate_t a template class for a filter
     * @tparam ProcessModel_t a concrete process model
     * @tparam MeasurementModelTemplate_t a template class for (a) measurement model(s) (arbitrary number)
     */
    template<template<class, class> class FilterTemplate_t, class ProcessModel_t,
        template<class> class... MeasurementModelTemplate_t>
    class Fusion
    {
    private:
      using Belief_type = typename ProcessModel_t::Belief_type;
      using ProcessNoiseCovarianceMatrix_type = typename ProcessModel_t::ProcessNoiseCovarianceMatrix_type;

    public:
      /**
       * Constructor.
       * @param process_noise_covariance_matrix a process noise covariance matrix for the selected process model
       * @param measurement_covariance_matrices a measurement covariance matrices for the selected measurement models
       */
      explicit
      Fusion(const ProcessNoiseCovarianceMatrix_type& process_noise_covariance_matrix,
             const typename MeasurementModelTemplate_t<ProcessModel_t>::MeasurementCovarianceMatrix_type&...
               measurement_covariance_matrices)
      : processed_measurements_counter_{0}, belief_{0, {}, {}}, process_model_{}, measurement_models_{}
      {

        process_model_.SetProcessNoiseCovarianceMatrix(process_noise_covariance_matrix);

        InitializeMeasurementCovarianceMatrices(
            std::forward_as_tuple(measurement_covariance_matrices...),
            std::index_sequence_for<MeasurementModelTemplate_t<ProcessModel_t>...>{});
      }

      template <class Measurement_type>
      Belief_type ProcessMeasurement(const Measurement_type& measurement)
      {
        ser94mor::sensor_fusion::apply(
            [this, &measurement](const auto... measurement_model)
            {
              static_cast<void>(
                  std::initializer_list<int>{(this->ProcessMeasurement(measurement, measurement_model), void(), 0)...}
              );
            },
            measurement_models_
        );

        return belief_;
      }

      void SetBelief(const Belief_type& belief)
      {
        belief_ = belief;
      }

      const Belief_type& GetBelief() const
      {
        return belief_;
      }

    private:
      template<class TupleOfMeasurementConvarianceMatrices, std::size_t... Is>
      void InitializeMeasurementCovarianceMatrices(
          const TupleOfMeasurementConvarianceMatrices& tup, std::index_sequence<Is...>)
      {
        // For C++17:
        // ((std::get<Is>(measurement_model_sensor_map_).first.SetMeasurementCovarianceMatrix(std::get<Is>(tup))), ...);

        // For C++14:
        static_cast<void>(
            std::initializer_list<int>
                {
                    (std::get<Is>(measurement_models_).SetMeasurementCovarianceMatrix(std::get<Is>(tup)), void(), 0)...
                }
        );
      }

      template<class Measurement_type, class MeasurementModel_type>
      auto ProcessMeasurement(const Measurement_type& measurement, const MeasurementModel_type& measurement_model)
      -> std::enable_if_t<Measurement_type::MeasurementModelKind() == MeasurementModel_type::Kind(), void>
      {
        using ControlVector = typename ProcessModel_t::ControlVector_type;

        if (processed_measurements_counter_ == 0)
        {
          belief_ = measurement_model.GetInitialBeliefBasedOn(measurement);
        }
        else
        {
          belief_ = FilterTemplate_t<ProcessModel_t, MeasurementModel_type>::
                      PredictUpdate(belief_, ControlVector::Zero(), measurement, process_model_, measurement_model);
        }

        ++processed_measurements_counter_;
      }

      template<class Measurement_type, class MeasurementModel_type>
      auto ProcessMeasurement(const Measurement_type&, const MeasurementModel_type&)
      -> std::enable_if_t<Measurement_type::MeasurementModelKind() != MeasurementModel_type::Kind(), void>
      {
        // Do nothing.
      }

      // counter of processed measurements
      uint64_t processed_measurements_counter_;

      Belief_type belief_;
      ProcessModel_t process_model_;
      std::tuple<MeasurementModelTemplate_t<ProcessModel_t>...> measurement_models_;
    };

  }
}


#endif //SENSOR_FUSION_FUSION_HPP
