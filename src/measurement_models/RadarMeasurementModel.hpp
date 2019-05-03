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

#ifndef SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP
#define SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP


#include "definitions.hpp"
#include "utils.hpp"
#include "MeasurementModel.hpp"
#include "../measurement_vector_views/RadarMeasurementVectorView.hpp"


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace Radar
    {

      /**
       * A concrete (Radar) measurement model. It is still a template because the dimensionality of
       * the measurement matrix depends on the process model kind, which we know only at compile time.
       * The naming of matrices and vectors are taken from the
       * "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."
       *
       * @tparam ProcessModel a process model class, which is needed to determine the number of state dimensions
       */
      template<class ProcessModel>
      class MeasurementModel
      : public ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
                                                         ROMeasurementVectorView, ProcessModel,
                                                         MeasurementModelKind::Radar, kIsLinear>
      {
      public:
        using MeasurementMatrix_type =
          Eigen::Matrix<double_t, MeasurementModel::MeasurementDims(), MeasurementModel::StateDims()>;
        using StateVector_type = typename ProcessModel::StateVector_type;
        using ROStateVectorView_type = typename ProcessModel::ROStateVectorView_type;


        /**
         * Constructor.
         */
        MeasurementModel()
        : ser94mor::sensor_fusion::MeasurementModel<MeasurementVector, MeasurementCovarianceMatrix,
                                                    ROMeasurementVectorView, ProcessModel,
                                                    MeasurementModelKind::Radar, kIsLinear>{}
        {

        }

        /**
         * A measurement function. It transforms the a vector from the state space into a measurement space.
         * @param state_vector a state vector
         * @return a corresponding vector in measurement space
         */
        MeasurementVector h(const StateVector_type& state_vector) const
        {
          auto sv{state_vector};
          ROStateVectorView_type svv{sv};

          MeasurementVector measurement_vector;
          measurement_vector << svv.range(), svv.bearing(), svv.range_rate();

          return measurement_vector;
        }

        /**
         * A Jacobian for non-linear case corresponding to the measurement matrix "C" for the linear case.
         * It depends on the process model kind.
         * 
         * @param state_vector a state vector
         * @return a measurement matrix
         */
        MeasurementMatrix_type H(const StateVector_type& state_vector) const
        {
          auto sv{state_vector};
          ROStateVectorView_type svv{sv};

          auto rho{svv.range()};
          auto rho_2{rho*rho};
          auto rho_3{rho_2*rho};
          auto tmp1{svv.px()/rho};
          auto tmp2{svv.py()/rho};

          MeasurementMatrix_type measurement_matrix{MeasurementMatrix_type::Zero()};
          switch (ProcessModel::Kind())
          {
            case ProcessModelKind::CV:
            {
              auto tmp3{svv.vx()*svv.py()-svv.vy()*svv.px()};

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

            case ProcessModelKind::CTRV:
            {
              auto sin{std::sin(svv.yaw())};
              auto cos{std::cos(svv.yaw())};
              auto tmp4{svv.v()/rho};
              auto tmp5{svv.px()*cos + svv.py()*sin};

              measurement_matrix(0,0) = tmp1;
              measurement_matrix(0,1) = tmp2;

              measurement_matrix(1,0) = -svv.py() / rho_2;
              measurement_matrix(1,1) =  svv.px() / rho_2;

              measurement_matrix(2,0) = tmp4 * (cos - svv.px() * tmp5 / rho_2);
              measurement_matrix(2,1) = tmp4 * (sin - svv.py() * tmp5 / rho_2);
              measurement_matrix(2,2) = tmp5 / rho;
              measurement_matrix(2,3) = tmp4 * (svv.py()*cos - svv.px()*sin);

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
         * @param measurement_vector_1 the first measurement vector
         * @param measurement_vector_2 the second measurement vector
         * @return the difference between the two measurement vectors
         */
        MeasurementVector Diff(const MeasurementVector& measurement_vector_1,
                               const MeasurementVector& measurement_vector_2) const
        {
          MeasurementVector diff{measurement_vector_1 - measurement_vector_2};
          RWMeasurementVectorView mvv{diff};
          Utils::NormalizeAngle(&mvv.bearing());

          return diff;
        }

      };

    }
  }
}


#endif //SENSOR_FUSION_RADARMEASUREMENTMODEL_HPP
