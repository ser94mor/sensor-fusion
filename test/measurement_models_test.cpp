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
#include "measurement_models.hpp"

#include <catch.hpp>
#include <iostream>
#include <cmath>


using namespace ser94mor::sensor_fusion;


///////////
// LIDAR //
///////////

TEST_CASE("Lidar::MeasurementModel<*>::C", "[measurement_models]")
{
  SECTION("CV::ProcessModel")
  {
    Lidar::MeasurementModel<CV::ProcessModel> lidar_mm_cv;
    Lidar::MeasurementModel<CV::ProcessModel>::MeasurementMatrix_type
        meas_mtx_cv{Lidar::MeasurementModel<CV::ProcessModel>::MeasurementMatrix_type::Identity()};

    REQUIRE(lidar_mm_cv.C().isApprox(meas_mtx_cv));
  }

  SECTION("CTRV::ProcessModel")
  {
    Lidar::MeasurementModel<CTRV::ProcessModel> lidar_mm_ctrv;
    Lidar::MeasurementModel<CTRV::ProcessModel>::MeasurementMatrix_type
        meas_mtx_ctrv{Lidar::MeasurementModel<CTRV::ProcessModel>::MeasurementMatrix_type::Identity()};

    REQUIRE(lidar_mm_ctrv.C().isApprox(meas_mtx_ctrv));
  }
}


TEST_CASE("Lidar::MeasurementModel<*>::Q", "[measurement_models]")
{
  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  SECTION("CV::ProcessModel")
  {
    Lidar::MeasurementModel<CV::ProcessModel> lidar_mm_cv;
    lidar_mm_cv.SetMeasurementCovarianceMatrix(lidar_mtx);

    REQUIRE(lidar_mm_cv.Q().isApprox(lidar_mtx));
  }

  SECTION("CTRV::ProcessModel")
  {
    Lidar::MeasurementModel<CTRV::ProcessModel> lidar_mm_ctrv;
    lidar_mm_ctrv.SetMeasurementCovarianceMatrix(lidar_mtx);

    REQUIRE(lidar_mm_ctrv.Q().isApprox(lidar_mtx));
  }
}


TEST_CASE("Lidar::MeasurementModel<*>::Type", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::Type() == EntityType::MeasurementModel);
  REQUIRE(Lidar::MeasurementModel<CTRV::ProcessModel>::Type() == EntityType::MeasurementModel);
}


TEST_CASE("Lidar::MeasurementModel<*>::TypeName", "[measurement_models]")
{
  REQUIRE(std::string(Lidar::MeasurementModel<CV::ProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
  REQUIRE(std::string(Lidar::MeasurementModel<CTRV::ProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
}


TEST_CASE("Lidar::MeasurementModel<*>::Kind", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::Kind() == MeasurementModelKind::Lidar);
  REQUIRE(Lidar::MeasurementModel<CTRV::ProcessModel>::Kind() == MeasurementModelKind::Lidar);
}


TEST_CASE("Lidar::MeasurementModel<*>::KindName", "[measurement_models]")
{
  REQUIRE(std::string(Lidar::MeasurementModel<CV::ProcessModel>::KindName()) == "LIDAR");
  REQUIRE(std::string(Lidar::MeasurementModel<CTRV::ProcessModel>::KindName()) == "LIDAR");
}


TEST_CASE("Lidar::MeasurementModel<*>::IsLinear", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::IsLinear() == true);
  REQUIRE(Lidar::MeasurementModel<CTRV::ProcessModel>::IsLinear() == true);
}


TEST_CASE("Lidar::MeasurementModel<*>::MeasurementDims", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::MeasurementDims() == 2);
  REQUIRE(Lidar::MeasurementModel<CTRV::ProcessModel>::MeasurementDims() == 2);
}


TEST_CASE("Lidar::MeasurementModel<*>::StateDims", "[measurement_models]")
{
  REQUIRE(Lidar::MeasurementModel<CV::ProcessModel>::StateDims() == 4);
  REQUIRE(Lidar::MeasurementModel<CTRV::ProcessModel>::StateDims() == 5);
}


TEST_CASE("Lidar::MeasurementModel<*>::GetInitialBeliefBasedOn", "[measurement_models]")
{
  Lidar::MeasurementVector mv;
  mv << 3., 4.;

  Lidar::Measurement measurement{2, mv};

  SECTION("CV::ProcessModel")
  {
    auto belief_cv{Lidar::MeasurementModel<CV::ProcessModel>::GetInitialBeliefBasedOn(measurement)};

    CV::StateVector sv_cv;
    sv_cv << 3., 4., 0., 0.;

    CV::StateCovarianceMatrix scm_cv{CV::StateCovarianceMatrix::Identity()};
    CV::ProcessModel::Belief_type belief_expected_cv{measurement.t(), sv_cv, scm_cv};

    REQUIRE(belief_cv == belief_expected_cv);
  }

  SECTION("CTRV::ProcessModel")
  {
    auto belief_ctrv{Lidar::MeasurementModel<CTRV::ProcessModel>::GetInitialBeliefBasedOn(measurement)};

    CTRV::StateVector sv_ctrv;
    sv_ctrv << 3., 4., 0., 0., 0.;

    CTRV::StateCovarianceMatrix scm_ctrv{CTRV::StateCovarianceMatrix::Identity()};
    CTRV::ProcessModel::Belief_type belief_expected_ctrv{measurement.t(), sv_ctrv, scm_ctrv};

    REQUIRE(belief_ctrv == belief_expected_ctrv);
  }
}


///////////
// RADAR //
///////////

TEST_CASE("Radar::MeasurementModel<*>::h", "[measurement_models]")
{
  SECTION("CV::ProcessModel")
  {
    Radar::MeasurementModel<CV::ProcessModel> radar_mm;

    CV::StateVector state_vector;
    state_vector << 3., 4., 2., 1.;

    Radar::MeasurementVector measurement_vector;
    measurement_vector << 5., std::atan2(4., 3.), 2.;

    REQUIRE(radar_mm.h(state_vector).isApprox(measurement_vector));
  }

  SECTION("CTRV::ProcessModel")
  {
    Radar::MeasurementModel<CTRV::ProcessModel> radar_mm;

    CTRV::StateVector state_vector;
    state_vector << 3., 4., 2., M_PI/6., M_PI/12.;

    Radar::MeasurementVector measurement_vector;
    measurement_vector << 5., std::atan2(4., 3.), (3.*std::sqrt(3.)+4.)/5.;

    REQUIRE(radar_mm.h(state_vector).isApprox(measurement_vector));
  }
}


TEST_CASE("Radar::MeasurementModel<*>::H", "[measurement_models]")
{
  SECTION("CV::ProcessModel")
  {
    Radar::MeasurementModel<CV::ProcessModel> radar_mm;

    CV::StateVector state_vector;
    state_vector << 3., 4., 2., 1.;

    Radar::MeasurementModel<CV::ProcessModel>::MeasurementMatrix_type measurement_matrix;
    measurement_matrix <<  3. / 5.,   4. / 5.,      0.,      0.,
                         -4. / 25.,  3. / 25.,      0.,      0.,
                          4. / 25., -3. / 25., 3. / 5., 4. / 5.;

    REQUIRE(radar_mm.H(state_vector).isApprox(measurement_matrix));
  }

  SECTION("CTRV::ProcessModel")
  {
    Radar::MeasurementModel<CTRV::ProcessModel> radar_mm;

    CTRV::StateVector state_vector;
    state_vector << 1., 2., 3., M_PI / 6., M_PI/12.;

    Radar::MeasurementModel<CTRV::ProcessModel>::MeasurementMatrix_type measurement_matrix;
    measurement_matrix <<     1./std::sqrt(5.), 2./std::sqrt(5.), 0., 0., 0.,
                                          -0.4,              0.2, 0., 0., 0.,
      (6.*std::sqrt(3.)-3.)/(5.*std::sqrt(5.)), ((3.-6*std::sqrt(3.))/(10.*std::sqrt((5.)))),
        (std::sqrt(3.)+2.)/(2.*std::sqrt(5.)), (6.*std::sqrt(3.)-3)/(2.*std::sqrt(5.)), 0.;

    REQUIRE(radar_mm.H(state_vector).isApprox(measurement_matrix));
  }
}


TEST_CASE("Radar::MeasurementModel<*>::Q", "[measurement_models]")
{
  Radar::MeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;

  SECTION("CV::ProcessModel")
  {
    Radar::MeasurementModel<CV::ProcessModel> radar_mm_cv;
    radar_mm_cv.SetMeasurementCovarianceMatrix(radar_mtx);

    REQUIRE(radar_mm_cv.Q().isApprox(radar_mtx));
  }

  SECTION("CTRV::ProcessModel")
  {
    Radar::MeasurementModel<CTRV::ProcessModel> radar_mm_ctrv;
    radar_mm_ctrv.SetMeasurementCovarianceMatrix(radar_mtx);

    REQUIRE(radar_mm_ctrv.Q().isApprox(radar_mtx));
  }
}


TEST_CASE("Radar::MeasurementModel<*>::Type", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::Type() == EntityType::MeasurementModel);
  REQUIRE(Radar::MeasurementModel<CTRV::ProcessModel>::Type() == EntityType::MeasurementModel);
}


TEST_CASE("Radar::MeasurementModel<*>::TypeName", "[measurement_models]")
{
  REQUIRE(std::string(Radar::MeasurementModel<CV::ProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
  REQUIRE(std::string(Radar::MeasurementModel<CTRV::ProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
}


TEST_CASE("Radar::MeasurementModel<*>::Kind", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::Kind() == MeasurementModelKind::Radar);
  REQUIRE(Radar::MeasurementModel<CTRV::ProcessModel>::Kind() == MeasurementModelKind::Radar);
}


TEST_CASE("Radar::MeasurementModel<*>::KindName", "[measurement_models]")
{
  REQUIRE(std::string(Radar::MeasurementModel<CV::ProcessModel>::KindName()) == "RADAR");
  REQUIRE(std::string(Radar::MeasurementModel<CTRV::ProcessModel>::KindName()) == "RADAR");
}


TEST_CASE("Radar::MeasurementModel<*>::IsLinear", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::IsLinear() == false);
  REQUIRE(Radar::MeasurementModel<CTRV::ProcessModel>::IsLinear() == false);
}


TEST_CASE("Radar::MeasurementModel<*>::MeasurementDims", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::MeasurementDims() == 3);
  REQUIRE(Radar::MeasurementModel<CTRV::ProcessModel>::MeasurementDims() == 3);
}


TEST_CASE("Radar::MeasurementModel<*>::StateDims", "[measurement_models]")
{
  REQUIRE(Radar::MeasurementModel<CV::ProcessModel>::StateDims() == 4);
  REQUIRE(Radar::MeasurementModel<CTRV::ProcessModel>::StateDims() == 5);
}


TEST_CASE("Radar::MeasurementModel<*>::GetInitialBeliefBasedOn", "[measurement_models]")
{
  Radar::MeasurementVector mv;
  mv << 3., M_PI/6., 1.;

  Radar::Measurement measurement{2, mv};

  SECTION("CV::ProcessModel")
  {
    CV::StateVector sv_cv;
    sv_cv << 3.*std::sqrt(3.)/2., 3./2., 0., 0.;

    CV::StateCovarianceMatrix scm_cv{CV::StateCovarianceMatrix::Identity()};

    auto belief_cv{Radar::MeasurementModel<CV::ProcessModel>::GetInitialBeliefBasedOn(measurement)};
    CV::ProcessModel::Belief_type belief_expected_cv{measurement.t(), sv_cv, scm_cv};

    REQUIRE(belief_cv == belief_expected_cv);
  }

  SECTION("CTRV::ProcessModel")
  {
    CTRV::StateVector sv_ctrv;
    sv_ctrv << 3. * std::sqrt(3.) / 2., 3. / 2., 0., 0., 0.;

    CTRV::StateCovarianceMatrix scm_ctrv{CTRV::StateCovarianceMatrix::Identity()};

    auto belief_ctrv{Radar::MeasurementModel<CTRV::ProcessModel>::GetInitialBeliefBasedOn(measurement)};
    CTRV::ProcessModel::Belief_type belief_expected_ctrv{measurement.t(), sv_ctrv, scm_ctrv};

    REQUIRE(belief_ctrv == belief_expected_ctrv);
  }
}
