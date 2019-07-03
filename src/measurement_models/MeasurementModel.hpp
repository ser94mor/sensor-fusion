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

#ifndef SENSOR_FUSION_MEASUREMENTMODEL_HPP
#define SENSOR_FUSION_MEASUREMENTMODEL_HPP


#include "definitions.hpp"
#include "measurements.hpp"
#include "measurement_vector_views.hpp"
#include "state_vector_views.hpp"
#include "process_models.hpp"
#include "beliefs.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {

    /**
     * A base template class representing a measurement model. It contains methods and functions common for
     * all concrete measurement models. Notice that in addition to
     * {@tparam MeasurementVector_t} and {@tparam MeasurementCovarianceMatrix_t} template parameters it also accepts
     * {@tparam ProcessModel_t} parameter.
     *
     * @tparam MeasurementVector_t a class of the measurement vector
     * @tparam MeasurementCovarianceMatrix_t a class of the measurement covariance matrix
     * @tparam ROMeasurementVectorView_t a class of the measurement vector view
     *                               (accessor to the measurement vector dimensions)
     * @tparam ProcessModel_t a class of the process model
     * @tparam mmk a kind of a measurement model (from a corresponding enum class)
     * @tparam is_linear flag indicating whether this measurement model is linear or not
     */
    template<class MeasurementVector_t, class MeasurementCovarianceMatrix_t, class ROMeasurementVectorView_t,
             class ProcessModel_t, MMKind mmk, bool is_linear>
    class MeasurementModel : public ModelEntity<EntityType::e_MeasurementModel, MMKind, mmk, is_linear>
    {
    public:
      /**
       * The typedefs below are needed in other places in the code. These typedefs, in fact, are attributes of the
       * measurement model.
       */
      using Measurement_type = Measurement<MeasurementVector_t, MeasurementCovarianceMatrix_t, mmk>;
      using MeasurementVector_type = MeasurementVector_t;
      using ROMeasurementVectorView_type = ROMeasurementVectorView_t;
      using MeasurementCovarianceMatrix_type = MeasurementCovarianceMatrix_t;

      using RWStateVectorView_type = typename ProcessModel_t::RWStateVectorView_type;
      using StateVector_type = typename ProcessModel_t::StateVector_type;

      using Belief_type = typename ProcessModel_t::Belief_type;

      /**
       * @return a number of dimensions in measurement vector
       */
      constexpr static size_t MeasurementDims();

      /**
       * @return a number of state dimensions
       */
      constexpr static size_t StateDims();

      /**
       * The naming of measurement covariance matrix is taken from the
       * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
       * @return a measurement covariance matrix
       */
      const MeasurementCovarianceMatrix_t& Q() const;

      /**
       * Set measurement covariance matrix. It is done explicitly by the user of measurement model
       * due to the variadic templates used in this code. MeasurementModel needs a default constructor.
       * @param mtx a measurement covariance matrix
       */
      void SetMeasurementCovarianceMatrix(const MeasurementCovarianceMatrix_t& mtx);

      /**
       * During the sensor fusion process, we need to initialize our initial belief based on something. When we receive
       * a first measurement, it becomes the most precise notion of the object's state. So, it is reasonable to
       * form an initial belief based on the first measurement.
       *
       * TODO: measurement covariance matrix should have 1's in px, py dimensions and much higher values
       *       for the dimensions of the state vector about which we have no information from the measurement vector.
       *
       * @param meas the first received measurement
       * @return an initial belief
       */
      static auto GetInitialBeliefBasedOn(const Measurement_type& meas) -> Belief_type;

    private:
      MeasurementCovarianceMatrix_t measurement_covariance_matrix_;
    };



    ////////////////////
    // IMPLEMENTATION //
    ////////////////////

    template<class MeasurementVector_t, class MeasurementCovarianceMatrix_t, class ROMeasurementVectorView_t,
             class ProcessModel_t, MMKind mmk, bool is_linear>
    constexpr size_t
    MeasurementModel<MeasurementVector_t, MeasurementCovarianceMatrix_t, ROMeasurementVectorView_t,
                     ProcessModel_t, mmk, is_linear>::MeasurementDims()
    {
      return static_cast<size_t>(MeasurementVector_t::SizeAtCompileTime);
    }

    template<class MeasurementVector_t, class MeasurementCovarianceMatrix_t, class ROMeasurementVectorView_t,
             class ProcessModel_t, MMKind mmk, bool is_linear>
    constexpr size_t
    MeasurementModel<MeasurementVector_t, MeasurementCovarianceMatrix_t, ROMeasurementVectorView_t,
                     ProcessModel_t, mmk, is_linear>::StateDims()
    {
      return ProcessModel_t::StateDims();
    }

    template<class MeasurementVector_t, class MeasurementCovarianceMatrix_t, class ROMeasurementVectorView_t,
             class ProcessModel_t, MMKind mmk, bool is_linear>
    const MeasurementCovarianceMatrix_t&
    MeasurementModel<MeasurementVector_t, MeasurementCovarianceMatrix_t, ROMeasurementVectorView_t,
                     ProcessModel_t, mmk, is_linear>::Q() const
    {
      return measurement_covariance_matrix_;
    }

    template<class MeasurementVector_t, class MeasurementCovarianceMatrix_t, class ROMeasurementVectorView_t,
             class ProcessModel_t, MMKind mmk, bool is_linear>
    void
    MeasurementModel<MeasurementVector_t, MeasurementCovarianceMatrix_t, ROMeasurementVectorView_t,
                     ProcessModel_t, mmk, is_linear>::SetMeasurementCovarianceMatrix(
        const MeasurementCovarianceMatrix_t& mtx)
    {
      measurement_covariance_matrix_ = mtx;
    }

    template<class MeasurementVector_t, class MeasurementCovarianceMatrix_t, class ROMeasurementVectorView_t,
             class ProcessModel_t, MMKind mmk, bool is_linear>
    auto
    MeasurementModel<MeasurementVector_t, MeasurementCovarianceMatrix_t, ROMeasurementVectorView_t,
                     ProcessModel_t, mmk, is_linear>::GetInitialBeliefBasedOn(
        const MeasurementModel::Measurement_type& meas) -> Belief_type
    {
      StateVector_type sv{StateVector_type::Zero()};
      const RWStateVectorView_type svv{sv};
      const ROMeasurementVectorView_type mvv{meas.z()};

      svv.px() = mvv.px();
      svv.py() = mvv.py();

      const typename ProcessModel_t::StateCovarianceMatrix_type
          state_covariance_matrix{ProcessModel_t::StateCovarianceMatrix_type::Identity()};
      return Belief_type{meas.t(), sv, state_covariance_matrix};
    }

  }
}

#endif //SENSOR_FUSION_MEASUREMENTMODEL_HPP
