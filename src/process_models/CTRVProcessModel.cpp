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

      ProcessModel::ProcessModel()
      {

      }

      StateVector ProcessModel::g(double dt, const ControlVector&, const StateVector& state_vector) const
      {
        StateVector sv_dst{state_vector};
        StateVectorView dst{sv_dst};
        ConstStateVectorView src{state_vector};

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

        Utils::NormalizeAngle(&dst.yaw());

        return sv_dst;
      }

      StateTransitionMatrix ProcessModel::G(double dt) const
      {
        return {};
      }

      ProcessCovarianceMatrix ProcessModel::R(double dt) const
      {
        Eigen::Matrix<double, ProcessModel::StateDims(), 2> Gt;
        double dt_2_2 = dt * dt / 2.;
        Gt << dt_2_2,    0.0,
            0.0, dt_2_2,
            dt,    0.0,
            0.0,     dt;
        return Gt * individual_noise_processes_covariance_matrix_ * Gt.transpose();
      }

    }
  }
}