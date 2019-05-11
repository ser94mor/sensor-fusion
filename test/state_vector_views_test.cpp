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


#include "state_vector_views.hpp"

#include <cmath>
#include <catch.hpp>


using namespace ser94mor::sensor_fusion;


/////////
// CV  //
/////////

TEST_CASE("CV::ROStateVectorView", "[state_vector_views]")
{
  CV::StateVector sv;
  sv << 1., 2., 3., 4.;
  CV::ROStateVectorView svv{sv};

  REQUIRE(Approx(1.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(3.) == svv.vx());
  REQUIRE(Approx(4.) == svv.vy());
  REQUIRE(Approx(5.) == svv.v());
  REQUIRE(Approx(std::asin(0.8)) == svv.yaw());
  REQUIRE(Approx(0.) == svv.yaw_rate());
  REQUIRE(Approx(std::sqrt(5.)) == svv.range());
  REQUIRE(Approx(std::atan2(2., 1.)) == svv.bearing());
  REQUIRE(Approx(11./std::sqrt(5.)) == svv.range_rate());
}


TEST_CASE("CV::RWStateVectorView", "[state_vector_views]")
{
  CV::StateVector sv;
  sv << 1., 2., 3., 4.;
  CV::RWStateVectorView svv{sv};

  //=====

  REQUIRE(Approx(1.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(3.) == svv.vx());
  REQUIRE(Approx(4.) == svv.vy());

  //=====

  svv.px() = 10.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(3.) == svv.vx());
  REQUIRE(Approx(4.) == svv.vy());

  //=====

  svv.py() = 11.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(3.) == svv.vx());
  REQUIRE(Approx(4.) == svv.vy());

  //=====

  svv.vx() = 12.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.vx());
  REQUIRE(Approx(4.) == svv.vy());

  //=====

  svv.vy() = 13.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.vx());
  REQUIRE(Approx(13.) == svv.vy());

  CV::StateVector sv_expected;
  sv_expected << 10., 11., 12., 13.;

  REQUIRE(sv.isApprox(sv_expected));
}


///////////
// CTRV  //
///////////

TEST_CASE("CTRV::ROStateVectorView", "[state_vector_views]")
{
  CTRV::StateVector sv;
  sv << 1., 2., 3., M_PI/6., M_PI/12.;
  CTRV::ROStateVectorView svv{sv};

  REQUIRE(Approx(1.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(std::sqrt(3.)*1.5) == svv.vx());
  REQUIRE(Approx(1.5) == svv.vy());
  REQUIRE(Approx(3.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());
  REQUIRE(Approx(std::sqrt(5.)) == svv.range());
  REQUIRE(Approx(std::atan2(2., 1.)) == svv.bearing());
  REQUIRE(Approx((1.5*std::sqrt(3.)+3.)/std::sqrt(5.)) == svv.range_rate());
}


TEST_CASE("CTRV::ROProcessNoiseVectorView", "[state_vector_views]")
{
  CTRV::ProcessNoiseVector pnv;
  pnv << 1., 2.;

  CTRV::ROProcessNoiseVectorView pnvv{pnv};

  REQUIRE(Approx(1.) == pnvv.longitudinal_acceleration());
  REQUIRE(Approx(2.) == pnvv.yaw_acceleration());
}


TEST_CASE("CTRV::RWStateVectorView", "[state_vector_views]")
{
  CTRV::StateVector sv;
  sv << 1., 2., 3., M_PI/6., M_PI/12.;
  CTRV::RWStateVectorView svv{sv};

  //=====

  REQUIRE(Approx(1.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(3.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.px() = 10.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(3.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.py() = 11.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(3.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.v() = 12.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.yaw() = M_PI/3.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.v());
  REQUIRE(Approx(M_PI/3.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.yaw_rate() = M_PI;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.v());
  REQUIRE(Approx(M_PI/3.) == svv.yaw());
  REQUIRE(Approx(M_PI) == svv.yaw_rate());

  CTRV::StateVector sv_expected;
  sv_expected << 10., 11., 12., M_PI/3., M_PI;

  REQUIRE(sv.isApprox(sv_expected));
}

TEST_CASE("CTRV::RWStateVectorViewBase", "[state_vector_views]")
{
  using StateVector_type = Eigen::Matrix<double_t, CTRV::kStateVectorDims + CTRV::kProcessNoiseVectorDims, 1>;
  StateVector_type sv;
  sv << 1., 2., 3., M_PI/6., M_PI/12., 11., 15.;
  CTRV::RWStateVectorViewBase<StateVector_type> svv{sv};

  //=====

  REQUIRE(Approx(1.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(3.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.px() = 10.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(2.) == svv.py());
  REQUIRE(Approx(3.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.py() = 11.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(3.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.v() = 12.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.v());
  REQUIRE(Approx(M_PI/6.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.yaw() = M_PI/3.;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.v());
  REQUIRE(Approx(M_PI/3.) == svv.yaw());
  REQUIRE(Approx(M_PI/12.) == svv.yaw_rate());

  //=====

  svv.yaw_rate() = M_PI;
  REQUIRE(Approx(10.) == svv.px());
  REQUIRE(Approx(11.) == svv.py());
  REQUIRE(Approx(12.) == svv.v());
  REQUIRE(Approx(M_PI/3.) == svv.yaw());
  REQUIRE(Approx(M_PI) == svv.yaw_rate());

  StateVector_type sv_expected;
  sv_expected << 10., 11., 12., M_PI/3., M_PI, 11., 15.;

  REQUIRE(sv.isApprox(sv_expected));
}
