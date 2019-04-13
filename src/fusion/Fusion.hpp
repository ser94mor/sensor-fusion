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

#include <cstddef>
#include <iostream>
#include <tuple>
#include <utility>
#include <string_view>
#include <Eigen/Dense>


namespace ser94mor::sensor_fusion
{

  template <template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
  class Fusion
  {
    using Belief = typename ProcessModel::Belief_type;

  public:
    /**
     * Constructor.
     */
    Fusion(IndividualNoiseProcessesCovarianceMatrix individual_noise_processes_covariance_matrix,
           typename MeasurementModel<typename ProcessModel::StateVector_type>::MeasurementCovarianceMatrix_type...
             measurement_covariance_matrices);

    /**
    * Run the whole flow of the Kalman Filter from here.
    */
    void Start();

  private:

    template <class TupleOfMeasurementConvarianceMatrices, std::size_t... Is>
    void InitializeMeasurementCovarianceMatrices(
        const TupleOfMeasurementConvarianceMatrices& tup, std::index_sequence<Is...>);

    template <class MeasurementModel_Sensor_Pair>
    void ProcessMeasurement(const MeasurementModel_Sensor_Pair& mm_s_pair);

    // flag indicating whether the first measurement processed
    bool initialized_;

    // counter of processed measurements
    uint64_t processed_measurements_counter_;

    // timestamp of the previously processed measurement
    uint64_t previous_measurement_timestamp_;




    Belief belief_;
    ProcessModel process_model_;
    std::tuple<std::pair<MeasurementModel<typename ProcessModel::StateVector_type>,
                         typename MeasurementModel<typename ProcessModel::StateVector_type>::Sensor_type>...>
      measurement_model_sensor_map_;
  };


  template <template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
  void Fusion<Filter, ProcessModel, MeasurementModel...>::Start()
  {
    for (;;)
    {
      std::apply(
          [this](auto... mm_s_pair)
          {
            (this->ProcessMeasurement(mm_s_pair), ...);
          },
          measurement_model_sensor_map_);
      break;
    }
  }

  template <template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
  template <class MeasurementModel_Sensor_Pair>
  void Fusion<Filter, ProcessModel, MeasurementModel...>::
      ProcessMeasurement(const MeasurementModel_Sensor_Pair& mm_s_pair)
  {
    std::cout << mm_s_pair.first.Type() << ' ' << mm_s_pair.first.Name() << '\n'
              << mm_s_pair.second.Type() << ' ' << mm_s_pair.second.Name()<< ::std::endl;
    //if constexpr (std::string_view(.Name())
  }

  template <template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
  Fusion<Filter, ProcessModel, MeasurementModel...>::Fusion(
      IndividualNoiseProcessesCovarianceMatrix individual_noise_processes_covariance_matrix,
      typename MeasurementModel<typename ProcessModel::StateVector_type>::MeasurementCovarianceMatrix_type...
        measurement_covariance_matrices) :
    initialized_{false}, processed_measurements_counter_{0}, previous_measurement_timestamp_{0}
  {

    process_model_.SetIndividualNoiseProcessCovarianceMatrix(individual_noise_processes_covariance_matrix);

    InitializeMeasurementCovarianceMatrices(
        std::forward_as_tuple(measurement_covariance_matrices...),
        std::index_sequence_for<MeasurementModel<typename ProcessModel::StateVector_type>...>{});

  }

  template <template<class, class> class Filter, class ProcessModel, template<class> class... MeasurementModel>
  template<class Tuple_Of_MeasurementConvarianceMatrix, size_t... Is>
  void Fusion<Filter, ProcessModel, MeasurementModel...>::InitializeMeasurementCovarianceMatrices(
      const Tuple_Of_MeasurementConvarianceMatrix& tup, std::index_sequence<Is...>)
  {
    ((std::get<Is>(measurement_model_sensor_map_).first.SetMeasurementCovarianceMatrix(std::get<Is>(tup))), ...);
  }

}



#endif //SENSOR_FUSION_FUSION_HPP
