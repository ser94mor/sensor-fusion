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
#include "process_models.hpp"

#include <catch.hpp>
#include <iostream>


using namespace ser94mor::sensor_fusion;
using namespace Eigen;


////////
// CV //
////////

TEST_CASE("CV::ProcessModel::A", "[process_models]")
{
  CV::ProcessModel cv_pm;
  CV::ProcessNoiseCovarianceMatrix mtx;
  mtx << 9.0, 0.0,
         0.0, 9.0;
  cv_pm.SetProcessNoiseCovarianceMatrix(mtx);

  Matrix4d A;
  A << 1, 0, 2, 0,
       0, 1, 0, 2,
       0, 0, 1, 0,
       0, 0, 0, 1;

  REQUIRE(cv_pm.A(2).isApprox(A));
}


TEST_CASE("CV::ProcessModel::B", "[process_models]")
{
  CV::ProcessModel cv_pm;
  CV::ProcessNoiseCovarianceMatrix mtx;
  mtx << 9.0, 0.0,
         0.0, 9.0;
  cv_pm.SetProcessNoiseCovarianceMatrix(mtx);

  Matrix4d B{Matrix4d::Zero()};

  REQUIRE(cv_pm.B().isApprox(B));
}


TEST_CASE("CV::ProcessModel::R", "[process_models]")
{
  CV::ProcessModel cv_pm1;
  CV::ProcessNoiseCovarianceMatrix mtx1;
  mtx1 << 9.0, 0.0,
          0.0, 9.0;
  cv_pm1.SetProcessNoiseCovarianceMatrix(mtx1);

  Matrix4d R1;
  R1 << 2.25,  0.0, 4.5, 0.0,
        0.0, 2.25, 0.0, 4.5,
        4.5,  0.0, 9.0, 0.0,
        0.0,  4.5, 0.0, 9.0;

  CV::ProcessModel cv_pm2;
  CV::ProcessNoiseCovarianceMatrix mtx2;
  mtx2 << 3.0, 7.0,
          7.0, 5.0;
  cv_pm2.SetProcessNoiseCovarianceMatrix(mtx2);

  Matrix4d R2;
  R2 << 12.0, 28.0, 12.0, 28.0,
        28.0, 20.0, 28.0, 20.0,
        12.0, 28.0, 12.0, 28.0,
        28.0, 20.0, 28.0, 20.0;

  REQUIRE(cv_pm1.R(1).isApprox(R1));
  REQUIRE(cv_pm2.R(2).isApprox(R2));
}


TEST_CASE("CV::ProcessModel::Type", "[process_models]")
{
  REQUIRE(CV::ProcessModel::Type() == EntityType::ProcessModel);
}


TEST_CASE("CV::ProcessModel::TypeName", "[process_models]")
{
  REQUIRE(std::string(CV::ProcessModel::TypeName()) == "PROCESS_MODEL");
}


TEST_CASE("CV::ProcessModel::Kind", "[process_models]")
{
  REQUIRE(CV::ProcessModel::Kind() == ProcessModelKind::CV);
}


TEST_CASE("CV::ProcessModel::KindName", "[process_models]")
{
  REQUIRE(std::string(CV::ProcessModel::KindName()) == "CV");
}


TEST_CASE("CV::ProcessModel::IsLinear", "[process_models]")
{
  REQUIRE(CV::ProcessModel::IsLinear() == true);
}


TEST_CASE("CV::ProcessModel::ControlDims", "[process_models]")
{
  REQUIRE(CV::ProcessModel::ControlDims() == 4);
}


TEST_CASE("CV::ProcessModel::StateDims", "[process_models]")
{
  REQUIRE(CV::ProcessModel::StateDims() == 4);
}


//////////
// CTRV //
//////////

TEST_CASE("CTRV::ProcessModel::g", "[process_models]")
{
  CTRV::ProcessModel ctrv_pm;

  CTRV::StateVector sv1;
  sv1 << 1., 2., 3., M_PI / 6., M_PI/12.;

  CTRV::StateVector sv2;
  sv2 << 1., 2., 3., M_PI / 6., -kEpsilon / 10.;

  CTRV::ControlVector cv{CTRV::ControlVector::Zero()};

  double_t dt{2.};

  CTRV::StateVector sv1_expected;
  sv1_expected << (1. + (18.*(std::sqrt(3.)-1.))/M_PI),
                  (2. + (18.*(std::sqrt(3.)-1.))/M_PI),
                  3.,
                  M_PI / 3.,
                  M_PI / 12.;

  CTRV::StateVector sv2_expected;
  sv2_expected << (1. + 3.*std::sqrt(3.)), 5., 3., M_PI / 6., 0.;


  REQUIRE(ctrv_pm.g(dt, cv, sv1).isApprox(sv1_expected));
  REQUIRE(ctrv_pm.g(dt, cv, sv2).isApprox(sv2_expected));
}


TEST_CASE("CTRV::ProcessModel::G", "[process_model]")
{
  // TODO: write this unit test
}


TEST_CASE("CTRV::ProcessModel::R", "[process_models]")
{
  CTRV::ProcessModel pm;
  CTRV::ProcessNoiseCovarianceMatrix mtx;
  mtx << 3.0, 7.0,
         7.0, 5.0;
  pm.SetProcessNoiseCovarianceMatrix(mtx);

  CTRV::StateCovarianceMatrix R_expected;
  R_expected <<         9.0, 5.1961524227066311, 10.392304845413264, 24.248711305964285,  24.248711305964285,
         5.1961524227066311,                3.0,                6.0,               14.0,                14.0,
         10.392304845413264,                6.0,               12.0,               28.0,                28.0,
         24.248711305964285,               14.0,               28.0,               20.0,                20.0,
         24.248711305964285,               14.0,               28.0,               20.0,                20.0;

  double_t dt{2.};
  CTRV::StateVector state_vector;
  state_vector << 1., 2., 3., M_PI/6., M_PI/12.;

  REQUIRE(pm.R(dt, state_vector).isApprox(R_expected));
}


TEST_CASE("CTRV::ProcessModel::Type", "[process_models]")
{
  REQUIRE(CTRV::ProcessModel::Type() == EntityType::ProcessModel);
}


TEST_CASE("CTRV::ProcessModel::TypeName", "[process_models]")
{
  REQUIRE(std::string(CTRV::ProcessModel::TypeName()) == "PROCESS_MODEL");
}


TEST_CASE("CTRV::ProcessModel::Kind", "[process_models]")
{
  REQUIRE(CTRV::ProcessModel::Kind() == ProcessModelKind::CTRV);
}


TEST_CASE("CTRV::ProcessModel::KindName", "[process_models]")
{
  REQUIRE(std::string(CTRV::ProcessModel::KindName()) == "CTRV");
}


TEST_CASE("CTRV::ProcessModel::IsLinear", "[process_models]")
{
  REQUIRE(CTRV::ProcessModel::IsLinear() == false);
}


TEST_CASE("CTRV::ProcessModel::ControlDims", "[process_models]")
{
  REQUIRE(CTRV::ProcessModel::ControlDims() == 5);
}


TEST_CASE("CTRV::ProcessModel::StateDims", "[process_models]")
{
  REQUIRE(CTRV::ProcessModel::StateDims() == 5);
}
