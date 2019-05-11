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


#include "definitions.hpp"
#include "utils.hpp"
#include "CTRVProcessModel.hpp"

#include <cmath>
#include <Eigen/Dense>


namespace ser94mor
{
  namespace sensor_fusion
  {
    namespace CTRV
    {

      ProcessModel::ProcessModel() : state_transition_matrix_prototype_{StateTransitionMatrix::Identity()}
      {

      }

      StateVector ProcessModel::g(double_t dt, const ControlVector&, const StateVector& state_vector) const
      {
        StateVector sv_dst{state_vector};
        RWStateVectorView dst{sv_dst};
        ROStateVectorView src{state_vector};

        if (std::fabs(src.yaw_rate()) < kEpsilon)
        {
          dst.px()  += src.v() * std::cos(src.yaw())*dt;
          dst.py()  += src.v() * std::sin(src.yaw())*dt;
          dst.yaw() += src.yaw_rate()*dt;
        }
        else
        {
          auto v_yaw_rate{src.v()/src.yaw_rate()};
          dst.px()  += v_yaw_rate * (std::sin(src.yaw()+src.yaw_rate()*dt) - std::sin(src.yaw()));
          dst.py()  += v_yaw_rate * (-std::cos(src.yaw()+src.yaw_rate()*dt) + std::cos(src.yaw()));
          dst.yaw() += src.yaw_rate()*dt;
        }

        return sv_dst;
      }

      StateTransitionMatrix ProcessModel::G(double_t dt, const StateVector& state_vector) const
      {
        ROStateVectorView src{state_vector};
        StateTransitionMatrix stm{state_transition_matrix_prototype_};

        double_t sin1{std::sin(src.yaw())};
        double_t cos1{std::cos(src.yaw())};

        double_t v{src.v()};

        if (std::fabs(src.yaw_rate()) < kEpsilon)
        {
          stm(0,2) = dt * cos1;
          stm(0,3) = -dt * v * sin1;

          stm(1,2) = dt * sin1;
          stm(1,3) = dt * v * cos1;
        }
        else
        {
          double_t sin2{std::sin(dt * src.yaw_rate() + src.yaw())};
          double_t cos2{std::cos(dt * src.yaw_rate() + src.yaw())};

          double_t yaw_rate{src.yaw_rate()};
          double_t yaw_rate_2{yaw_rate * yaw_rate};

          stm(0, 2) = (-sin1 + sin2) / yaw_rate;
          stm(0, 3) = v * (-cos1 + cos2) / yaw_rate;
          stm(0, 4) = dt * v * cos2 / yaw_rate - v * (-sin1 + sin2) / yaw_rate_2;

          stm(1, 2) = (cos1 - cos2) / yaw_rate;
          stm(1, 3) = v * (-sin1 + sin2) / yaw_rate;
          stm(1, 4) = dt * v * sin2 / yaw_rate - v * (cos1 - cos2) / yaw_rate_2;

          stm(3, 4) = dt;
        }

        return stm;
      }

      ProcessCovarianceMatrix ProcessModel::R(double_t dt, const StateVector& state_vector) const
      {
        double_t dt_2{dt * dt};
        Eigen::Matrix<double_t, ProcessModel::StateDims(), 2> Gt;

        ROStateVectorView src{state_vector};

        Gt << 0.5 * dt_2 * std::cos(src.yaw()),        0.0,
              0.5 * dt_2 * std::sin(src.yaw()),        0.0,
                                            dt,        0.0,
                                           0.0, 0.5 * dt_2,
                                           0.0,         dt;
        return Gt * process_noise_covariance_matrix_ * Gt.transpose();
      }

      StateVector ProcessModel::g(double_t dt, const ControlVector& control_vector, const StateVector& state_vector,
                                  const ProcessNoiseVector& noise_vector) const
      {
        auto sv{g(dt, control_vector, state_vector)};
        RWStateVectorView svv{sv};
        ROProcessNoiseVectorView pnvv{noise_vector};

        auto dt_2{dt * dt};

        // add noise
        svv.px()       += 0.5 * dt_2 * std::cos(svv.yaw()) * pnvv.longitudinal_acceleration();
        svv.py()       += 0.5 * dt_2 * std::sin(svv.yaw()) * pnvv.longitudinal_acceleration();
        svv.v()        += dt * pnvv.longitudinal_acceleration();
        svv.yaw()      += 0.5 * dt_2 * pnvv.yaw_acceleration();
        svv.yaw_rate() += dt * pnvv.yaw_acceleration();

        return sv;
      }

    }
  }
}