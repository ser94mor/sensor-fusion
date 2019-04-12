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


#include "process_models.hpp"

#include <catch.hpp>
#include <iostream>


using namespace ser94mor::sensor_fusion;
using namespace Eigen;


TEST_CASE("CV::ProcessModel::A", "[process_models]")
{
  CV::ProcessModel cv_pm;
  IndividualNoiseProcessesCovarianceMatrix mtx;
  mtx << 9.0, 0.0,
         0.0, 9.0;
  cv_pm.SetIndividualNoiseProcessCovarianceMatrix(mtx);

  Matrix4d A;
  A << 1, 0, 2, 0,
       0, 1, 0, 2,
       0, 0, 1, 0,
       0, 0, 0, 1;

  REQUIRE(cv_pm.A(2).isApprox(A));
}


TEST_CASE("CVProcessModel::B", "[process_models]")
{
  CV::ProcessModel cv_pm;
  IndividualNoiseProcessesCovarianceMatrix mtx;
  mtx << 9.0, 0.0,
         0.0, 9.0;
  cv_pm.SetIndividualNoiseProcessCovarianceMatrix(mtx);

  Matrix4d B{Matrix4d::Zero()};

  REQUIRE(cv_pm.B(2).isApprox(B));
}


TEST_CASE("CVProcessModel::R", "[process_models]")
{
  CV::ProcessModel cv_pm1;
  IndividualNoiseProcessesCovarianceMatrix mtx1;
  mtx1 << 9.0, 0.0,
          0.0, 9.0;
  cv_pm1.SetIndividualNoiseProcessCovarianceMatrix(mtx1);

  Matrix4d R1;
  R1 << 2.25,  0.0, 4.5, 0.0,
        0.0, 2.25, 0.0, 4.5,
        4.5,  0.0, 9.0, 0.0,
        0.0,  4.5, 0.0, 9.0;

  CV::ProcessModel cv_pm2;
  IndividualNoiseProcessesCovarianceMatrix mtx2;
  mtx2 << 3.0, 7.0,
          7.0, 5.0;
  cv_pm2.SetIndividualNoiseProcessCovarianceMatrix(mtx2);

  Matrix4d R2;
  R2 << 12.0, 28.0, 12.0, 28.0,
        28.0, 20.0, 28.0, 20.0,
        12.0, 28.0, 12.0, 28.0,
        28.0, 20.0, 28.0, 20.0;

  REQUIRE(cv_pm1.R(1).isApprox(R1));
  REQUIRE(cv_pm2.R(2).isApprox(R2));
}
