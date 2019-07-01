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


#include "definitions.hpp"
#include "utils.hpp"
#include "CTRVProcessModel.hpp"

#include <cmath>
#include <Eigen/Dense>


namespace ser94mor
{
  namespace sensor_fusion
  {


    CTRVProcessModel::CTRVProcessModel() : CTRVProcessModelBase{},
      state_transition_matrix_prototype_{CTRVStateTransitionMatrix::Identity()}
    {

    }

    CTRVStateVector CTRVProcessModel::g(double_t dt, const CTRVControlVector&, const CTRVStateVector& sv)
    {
      CTRVStateVector sv_dst{sv};
      const CTRVRWStateVectorView dst{sv_dst};
      const CTRVROStateVectorView src{sv};

      if (std::fabs(src.yaw_rate()) < kEpsilon)
      {
        dst.px()  += src.v() * std::cos(src.yaw())*dt;
        dst.py()  += src.v() * std::sin(src.yaw())*dt;
        dst.yaw() += src.yaw_rate()*dt;
      }
      else
      {
        const auto v_yaw_rate{src.v()/src.yaw_rate()};
        dst.px()  += v_yaw_rate * (std::sin(src.yaw()+src.yaw_rate()*dt) - std::sin(src.yaw()));
        dst.py()  += v_yaw_rate * (-std::cos(src.yaw()+src.yaw_rate()*dt) + std::cos(src.yaw()));
        dst.yaw() += src.yaw_rate()*dt;
      }

      return sv_dst;
    }

    CTRVStateTransitionMatrix CTRVProcessModel::G(double_t dt, const CTRVStateVector& sv) const
    {
      const CTRVROStateVectorView src{sv};
      CTRVStateTransitionMatrix stm{state_transition_matrix_prototype_};

      const double_t v{src.v()};
      const double_t sin1{std::sin(src.yaw())};
      const double_t cos1{std::cos(src.yaw())};

      if (std::fabs(src.yaw_rate()) < kEpsilon)
      {
        stm(0,2) = dt * cos1;
        stm(0,3) = -dt * v * sin1;

        stm(1,2) = dt * sin1;
        stm(1,3) = dt * v * cos1;
      }
      else
      {
        const double_t sin2{std::sin(dt * src.yaw_rate() + src.yaw())};
        const double_t cos2{std::cos(dt * src.yaw_rate() + src.yaw())};

        const double_t yaw_rate{src.yaw_rate()};
        const double_t yaw_rate_2{yaw_rate * yaw_rate};

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

    CTRVProcessCovarianceMatrix CTRVProcessModel::R(double_t dt, const CTRVStateVector& sv) const
    {
      const double_t dt_2{dt * dt};
      Eigen::Matrix<double_t, CTRVProcessModel::StateDims(), 2> Gt;

      const CTRVROStateVectorView src{sv};

      Gt << 0.5 * dt_2 * std::cos(src.yaw()),        0.0,
          0.5 * dt_2 * std::sin(src.yaw()),        0.0,
          dt,        0.0,
          0.0, 0.5 * dt_2,
          0.0,         dt;
      return Gt * GetProcessNoiseCovarianceMatrix() * Gt.transpose();
    }

    CTRVStateVector CTRVProcessModel::g(double_t dt, const CTRVControlVector& cv, const CTRVStateVector& sv,
                                        const CTRVProcessNoiseVector& nv)
    {
      auto sv_copy{g(dt, cv, sv)};
      const CTRVRWStateVectorView svv{sv_copy};
      const CTRVROProcessNoiseVectorView pnvv{nv};

      const auto dt_2{dt * dt};

      // add noise
      svv.px()       += 0.5 * dt_2 * std::cos(svv.yaw()) * pnvv.longitudinal_acceleration();
      svv.py()       += 0.5 * dt_2 * std::sin(svv.yaw()) * pnvv.longitudinal_acceleration();
      svv.v()        += dt * pnvv.longitudinal_acceleration();
      svv.yaw()      += 0.5 * dt_2 * pnvv.yaw_acceleration();
      svv.yaw_rate() += dt * pnvv.yaw_acceleration();

      return sv_copy;
    }


  }
}