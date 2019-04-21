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

#include "utils.hpp"
#include "definitions.hpp"
#include "state_vector_views.hpp"

#include <iostream>
#include <catch.hpp>

using namespace ser94mor::sensor_fusion;


TEST_CASE("Utils::NormalizeAngle", "[utils]")
{
  CTRV::StateVector sv1;
  sv1 << 1., 2., 3., 98 * M_PI, M_PI_2;

  CTRV::StateVector sv2;
  sv2 << 1., 2., 3., -99 * M_PI - 0.001, M_PI_2;

  CTRV::RWStateVectorView svv1{sv1};

  CTRV::RWStateVectorView svv2{sv2};

  Utils::NormalizeAngle(&svv1.yaw());
  Utils::NormalizeAngle(&svv2.yaw());

  CTRV::StateVector sv_expected1;
  sv_expected1 << 1., 2., 3., 0., M_PI_2;

  CTRV::StateVector sv_expected2;
  sv_expected2 << 1., 2., 3., M_PI - 0.001, M_PI_2;

  REQUIRE(sv1.isApprox(sv_expected1));
  REQUIRE(sv2.isApprox(sv_expected2));
}
