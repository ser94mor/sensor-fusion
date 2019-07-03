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

#ifndef SENSOR_FUSION_PROCESSMODEL_HPP
#define SENSOR_FUSION_PROCESSMODEL_HPP


#include "definitions.hpp"
#include "beliefs.hpp"

#include <ctime>


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base template class representing a process model.
     * It contains methods and functions common for all concrete process models.
     *
     * @tparam StateVector_t a state vector class
     * @tparam StateCovarianceMatrix_t a state covariance matrix class
     * @tparam StateVectorView a state vector view (accessor to the state vector dimensions)
     * @tparam ControlVector_t a control vector class
     * @tparam pmk a process model kind
     * @tparam is_linear a flag indicating whether the process model is linear or not
     */
    template<class StateVector_t, class StateCovarianceMatrix_t, class ControlVector_t,
             class ProcessNoiseCovarianceMatrix_t, class ROStateVectorView_t, class RWStateVectorView_t,
             PMKind pmk, bool is_linear>
    class ProcessModel : public ModelEntity<EntityType::e_ProcessModel, PMKind, pmk, is_linear>
    {
    public:
      /**
       * The typedefs below are needed in other places in the code. These typedefs, in fact, are attributes of the
       * process model.
       */
      using Belief_type = Belief<StateVector_t, StateCovarianceMatrix_t>;
      using StateVector_type = StateVector_t;
      using StateCovarianceMatrix_type = StateCovarianceMatrix_t;
      using ControlVector_type = ControlVector_t;
      using ProcessNoiseCovarianceMatrix_type = ProcessNoiseCovarianceMatrix_t;
      using RWStateVectorView_type = RWStateVectorView_t;
      using ROStateVectorView_type = ROStateVectorView_t;

      /**
       * @return a number of dimensions in the state vector
       */
      constexpr static size_t StateDims();

      /**
       * @return a number of dimensions in the control vector
       */
      constexpr static size_t ControlDims();

      /**
       * @return a number of dimensions in the process noise vector
       */
      constexpr static size_t ProcessNoiseDims();

      /**
       * Set a process noise covariance matrix. It is done explicitly by the user of the process model
       * due to the variadic templates used in this code. ProcessModel needs a default constructor.
       * @param mtx a process noise covariance matrix
       */
      void SetProcessNoiseCovarianceMatrix(const ProcessNoiseCovarianceMatrix_t& mtx);

      /**
       * Get a process noise covariance matrix.
       * @return a process noise covariance matirx.
       */
      const ProcessNoiseCovarianceMatrix_t& GetProcessNoiseCovarianceMatrix() const;

    private:
      ProcessNoiseCovarianceMatrix_t process_noise_covariance_matrix_;
    };



    ////////////////////
    // IMPLEMENTATION //
    ////////////////////

    template<class StateVector_t, class StateCovarianceMatrix_t, class ControlVector_t,
             class ProcessNoiseCovarianceMatrix_t, class ROStateVectorView_t,
             class RWStateVectorView_t, PMKind pmk, bool is_linear>
    constexpr size_t
    ProcessModel<StateVector_t, StateCovarianceMatrix_t, ControlVector_t, ProcessNoiseCovarianceMatrix_t,
                 ROStateVectorView_t, RWStateVectorView_t, pmk, is_linear>::StateDims()
    {
      return static_cast<size_t>(StateVector_t::SizeAtCompileTime);
    }

    template<class StateVector_t, class StateCovarianceMatrix_t, class ControlVector_t,
             class ProcessNoiseCovarianceMatrix_t, class ROStateVectorView_t,
             class RWStateVectorView_t, PMKind pmk, bool is_linear>
    constexpr size_t
    ProcessModel<StateVector_t, StateCovarianceMatrix_t, ControlVector_t, ProcessNoiseCovarianceMatrix_t,
                 ROStateVectorView_t, RWStateVectorView_t, pmk, is_linear>::ControlDims()
    {
      return static_cast<size_t>(ControlVector_t::SizeAtCompileTime);
    }

    template<class StateVector_t, class StateCovarianceMatrix_t, class ControlVector_t,
             class ProcessNoiseCovarianceMatrix_t, class ROStateVectorView_t,
             class RWStateVectorView_t, PMKind pmk, bool is_linear>
    constexpr size_t
    ProcessModel<StateVector_t, StateCovarianceMatrix_t, ControlVector_t, ProcessNoiseCovarianceMatrix_t,
                 ROStateVectorView_t, RWStateVectorView_t, pmk, is_linear>::ProcessNoiseDims()
    {
      return static_cast<size_t>(ProcessNoiseCovarianceMatrix_t::RowsAtCompileTime);
    }

    template<class StateVector_t, class StateCovarianceMatrix_t, class ControlVector_t,
             class ProcessNoiseCovarianceMatrix_t, class ROStateVectorView_t,
             class RWStateVectorView_t, PMKind pmk, bool is_linear>
    void
    ProcessModel<StateVector_t, StateCovarianceMatrix_t, ControlVector_t, ProcessNoiseCovarianceMatrix_t,
                 ROStateVectorView_t, RWStateVectorView_t, pmk, is_linear>::SetProcessNoiseCovarianceMatrix(
        const ProcessNoiseCovarianceMatrix_t& mtx)
    {
      process_noise_covariance_matrix_ = mtx;
    }

    template<class StateVector_t, class StateCovarianceMatrix_t, class ControlVector_t,
             class ProcessNoiseCovarianceMatrix_t, class ROStateVectorView_t,
             class RWStateVectorView_t, PMKind pmk, bool is_linear>
    const ProcessNoiseCovarianceMatrix_t&
    ProcessModel<StateVector_t, StateCovarianceMatrix_t, ControlVector_t, ProcessNoiseCovarianceMatrix_t,
                 ROStateVectorView_t, RWStateVectorView_t, pmk, is_linear>::GetProcessNoiseCovarianceMatrix() const
    {
      return process_noise_covariance_matrix_;
    }

  }
}


#endif //SENSOR_FUSION_PROCESSMODEL_HPP
