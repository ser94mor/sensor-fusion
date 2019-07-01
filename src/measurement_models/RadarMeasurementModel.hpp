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

#ifndef SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP
#define SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP


#include "definitions.hpp"
#include "utils.hpp"
#include "measurement_vector_views.hpp"
#include "MeasurementModel.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {
    template<class ProcessModel_t>
    using RadarMeasurementModelBase =
        MeasurementModel<RadarMeasurementVector, RadarMeasurementCovarianceMatrix, RadarROMeasurementVectorView,
        ProcessModel_t, MMKind::e_Radar, kRadarIsLinear>;

    /**
     * A concrete (Radar) measurement model. It is still a template because the dimensionality of
     * the measurement matrix depends on the process model kind, which we know only at compile time.
     * The naming of matrices and vectors are taken from the
     * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
     *
     * @tparam ProcessModel_t a process model class, which is needed to determine the number of state dimensions
     */
    template<class ProcessModel_t>
    class RadarMeasurementModel : public RadarMeasurementModelBase<ProcessModel_t>
    {
    public:
      using MeasurementMatrix_type =
          Eigen::Matrix<double_t, RadarMeasurementModel::MeasurementDims(), RadarMeasurementModel::StateDims()>;
      using StateVector_type = typename ProcessModel_t::StateVector_type;
      using ROStateVectorView_type = typename ProcessModel_t::ROStateVectorView_type;


      /**
       * Constructor.
       */
      RadarMeasurementModel() : RadarMeasurementModelBase<ProcessModel_t>{}
      {

      }

      /**
       * A measurement function. It transforms the a vector from the state space into a measurement space.
       * @param sv a state vector
       * @return a corresponding vector in measurement space
       */
      static RadarMeasurementVector h(const StateVector_type& sv)
      {
        const ROStateVectorView_type svv{sv};

        RadarMeasurementVector measurement_vector;
        measurement_vector << svv.range(), svv.bearing(), svv.range_rate();

        return measurement_vector;
      }

      /**
       * A Jacobian for non-linear case corresponding to the measurement matrix "C" for the linear case.
       * It depends on the process model kind.
       *
       * @param sv a state vector
       * @return a measurement matrix
       */
      static MeasurementMatrix_type H(const StateVector_type& sv)
      {
        const ROStateVectorView_type svv{sv};

        const auto rho{svv.range()};
        const auto rho_2{rho*rho};
        const auto rho_3{rho_2*rho};
        const auto tmp1{svv.px()/rho};
        const auto tmp2{svv.py()/rho};

        MeasurementMatrix_type measurement_matrix{MeasurementMatrix_type::Zero()};
        switch (ProcessModel_t::Kind())
        {
          case PMKind::e_CV:
          {
            const auto tmp3{svv.vx()*svv.py()-svv.vy()*svv.px()};

            measurement_matrix(0,0) = tmp1;
            measurement_matrix(0,1) = tmp2;

            measurement_matrix(1,0) = -svv.py() / rho_2;
            measurement_matrix(1,1) =  svv.px() / rho_2;

            measurement_matrix(2,0) =  svv.py() * tmp3 / rho_3;
            measurement_matrix(2,1) = -svv.px() * tmp3 / rho_3;
            measurement_matrix(2,2) = tmp1;
            measurement_matrix(2,3) = tmp2;

            break;
          }

          case PMKind::e_CTRV:
          {
            const auto sin1{std::sin(svv.yaw())};
            const auto cos1{std::cos(svv.yaw())};
            const auto tmp4{svv.v()/rho};
            const auto tmp5{svv.px()*cos1 + svv.py()*sin1};

            measurement_matrix(0,0) = tmp1;
            measurement_matrix(0,1) = tmp2;

            measurement_matrix(1,0) = -svv.py() / rho_2;
            measurement_matrix(1,1) =  svv.px() / rho_2;

            measurement_matrix(2,0) = tmp4 * (cos1 - svv.px() * tmp5 / rho_2);
            measurement_matrix(2,1) = tmp4 * (sin1 - svv.py() * tmp5 / rho_2);
            measurement_matrix(2,2) = tmp5 / rho;
            measurement_matrix(2,3) = tmp4 * (svv.py()*cos1 - svv.px()*sin1);

            break;
          }

          default:
          {
            throw std::logic_error("unknown process model");
          }
        }

        return measurement_matrix;
      }

      /**
       * Calculate the difference between two measurement vectors. In Radar case, there is a need to "normalize"
       * the bearing dimension because after the vector subtraction the bearing angle may fall out of the [-pi, pi]
       * interval.
       *
       * @param mv_1 the first measurement vector
       * @param mv_2 the second measurement vector
       * @return the difference between the two measurement vectors
       */
      static RadarMeasurementVector Diff(const RadarMeasurementVector& mv_1, const RadarMeasurementVector& mv_2)
      {
        RadarMeasurementVector diff{mv_1 - mv_2};
        const RadarRWMeasurementVectorView mvv{diff};
        Utils::NormalizeAngle(&mvv.bearing());

        return diff;
      }

    };

  }
}


#endif //SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP
