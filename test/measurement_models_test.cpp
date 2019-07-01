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

TEST_CASE("LidarMeasurementModel<*>::C", "[measurement_models]")
{
  SECTION("CVProcessModel")
  {
    LidarMeasurementModel<CVProcessModel> lidar_mm_cv;
    LidarMeasurementModel<CVProcessModel>::MeasurementMatrix_type
        meas_mtx_cv{LidarMeasurementModel<CVProcessModel>::MeasurementMatrix_type::Identity()};

    REQUIRE(lidar_mm_cv.C().isApprox(meas_mtx_cv));
  }

  SECTION("CTRVProcessModel")
  {
    LidarMeasurementModel<CTRVProcessModel> lidar_mm_ctrv;
    LidarMeasurementModel<CTRVProcessModel>::MeasurementMatrix_type
        meas_mtx_ctrv{LidarMeasurementModel<CTRVProcessModel>::MeasurementMatrix_type::Identity()};

    REQUIRE(lidar_mm_ctrv.C().isApprox(meas_mtx_ctrv));
  }
}


TEST_CASE("LidarMeasurementModel<*>::Q", "[measurement_models]")
{
  LidarMeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
                  0.0, 0.0225;

  SECTION("CVProcessModel")
  {
    LidarMeasurementModel<CVProcessModel> lidar_mm_cv;
    lidar_mm_cv.SetMeasurementCovarianceMatrix(lidar_mtx);

    REQUIRE(lidar_mm_cv.Q().isApprox(lidar_mtx));
  }

  SECTION("CTRVProcessModel")
  {
    LidarMeasurementModel<CTRVProcessModel> lidar_mm_ctrv;
    lidar_mm_ctrv.SetMeasurementCovarianceMatrix(lidar_mtx);

    REQUIRE(lidar_mm_ctrv.Q().isApprox(lidar_mtx));
  }
}


TEST_CASE("LidarMeasurementModel<*>::Type", "[measurement_models]")
{
  REQUIRE(LidarMeasurementModel<CVProcessModel>::Type() == EntityType::e_MeasurementModel);
  REQUIRE(LidarMeasurementModel<CTRVProcessModel>::Type() == EntityType::e_MeasurementModel);
}


TEST_CASE("LidarMeasurementModel<*>::TypeName", "[measurement_models]")
{
  REQUIRE(std::string(LidarMeasurementModel<CVProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
  REQUIRE(std::string(LidarMeasurementModel<CTRVProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
}


TEST_CASE("LidarMeasurementModel<*>::Kind", "[measurement_models]")
{
  REQUIRE(LidarMeasurementModel<CVProcessModel>::Kind() == MMKind::e_Lidar);
  REQUIRE(LidarMeasurementModel<CTRVProcessModel>::Kind() == MMKind::e_Lidar);
}


TEST_CASE("LidarMeasurementModel<*>::KindName", "[measurement_models]")
{
  REQUIRE(std::string(LidarMeasurementModel<CVProcessModel>::KindName()) == "LIDAR");
  REQUIRE(std::string(LidarMeasurementModel<CTRVProcessModel>::KindName()) == "LIDAR");
}


TEST_CASE("LidarMeasurementModel<*>::IsLinear", "[measurement_models]")
{
  REQUIRE(LidarMeasurementModel<CVProcessModel>::IsLinear() == true);
  REQUIRE(LidarMeasurementModel<CTRVProcessModel>::IsLinear() == true);
}


TEST_CASE("LidarMeasurementModel<*>::MeasurementDims", "[measurement_models]")
{
  REQUIRE(LidarMeasurementModel<CVProcessModel>::MeasurementDims() == 2);
  REQUIRE(LidarMeasurementModel<CTRVProcessModel>::MeasurementDims() == 2);
}


TEST_CASE("LidarMeasurementModel<*>::StateDims", "[measurement_models]")
{
  REQUIRE(LidarMeasurementModel<CVProcessModel>::StateDims() == 4);
  REQUIRE(LidarMeasurementModel<CTRVProcessModel>::StateDims() == 5);
}


TEST_CASE("LidarMeasurementModel<*>::GetInitialBeliefBasedOn", "[measurement_models]")
{
  LidarMeasurementVector mv;
  mv << 3., 4.;

  LidarMeasurement measurement{2, mv};

  SECTION("CVProcessModel")
  {
    auto belief_cv{LidarMeasurementModel<CVProcessModel>::GetInitialBeliefBasedOn(measurement)};

    CVStateVector sv_cv;
    sv_cv << 3., 4., 0., 0.;

    CVStateCovarianceMatrix scm_cv{CVStateCovarianceMatrix::Identity()};
    CVProcessModel::Belief_type belief_expected_cv{measurement.t(), sv_cv, scm_cv};

    REQUIRE(belief_cv == belief_expected_cv);
  }

  SECTION("CTRVProcessModel")
  {
    auto belief_ctrv{LidarMeasurementModel<CTRVProcessModel>::GetInitialBeliefBasedOn(measurement)};

    CTRVStateVector sv_ctrv;
    sv_ctrv << 3., 4., 0., 0., 0.;

    CTRVStateCovarianceMatrix scm_ctrv{CTRVStateCovarianceMatrix::Identity()};
    CTRVProcessModel::Belief_type belief_expected_ctrv{measurement.t(), sv_ctrv, scm_ctrv};

    REQUIRE(belief_ctrv == belief_expected_ctrv);
  }
}


///////////
// RADAR //
///////////

TEST_CASE("RadarMeasurementModel<*>::h", "[measurement_models]")
{
  SECTION("CVProcessModel")
  {
    RadarMeasurementModel<CVProcessModel> radar_mm;

    CVStateVector state_vector;
    state_vector << 3., 4., 2., 1.;

    RadarMeasurementVector measurement_vector;
    measurement_vector << 5., std::atan2(4., 3.), 2.;

    REQUIRE(radar_mm.h(state_vector).isApprox(measurement_vector));
  }

  SECTION("CTRVProcessModel")
  {
    RadarMeasurementModel<CTRVProcessModel> radar_mm;

    CTRVStateVector state_vector;
    state_vector << 3., 4., 2., M_PI/6., M_PI/12.;

    RadarMeasurementVector measurement_vector;
    measurement_vector << 5., std::atan2(4., 3.), (3.*std::sqrt(3.)+4.)/5.;

    REQUIRE(radar_mm.h(state_vector).isApprox(measurement_vector));
  }
}


TEST_CASE("RadarMeasurementModel<*>::H", "[measurement_models]")
{
  SECTION("CVProcessModel")
  {
    RadarMeasurementModel<CVProcessModel> radar_mm;

    CVStateVector state_vector;
    state_vector << 3., 4., 2., 1.;

    RadarMeasurementModel<CVProcessModel>::MeasurementMatrix_type measurement_matrix;
    measurement_matrix <<  3. / 5.,   4. / 5.,      0.,      0.,
                         -4. / 25.,  3. / 25.,      0.,      0.,
                          4. / 25., -3. / 25., 3. / 5., 4. / 5.;

    REQUIRE(radar_mm.H(state_vector).isApprox(measurement_matrix));
  }

  SECTION("CTRVProcessModel")
  {
    RadarMeasurementModel<CTRVProcessModel> radar_mm;

    CTRVStateVector state_vector;
    state_vector << 1., 2., 3., M_PI / 6., M_PI/12.;

    RadarMeasurementModel<CTRVProcessModel>::MeasurementMatrix_type measurement_matrix;
    measurement_matrix <<     1./std::sqrt(5.), 2./std::sqrt(5.), 0., 0., 0.,
                                          -0.4,              0.2, 0., 0., 0.,
      (6.*std::sqrt(3.)-3.)/(5.*std::sqrt(5.)), ((3.-6*std::sqrt(3.))/(10.*std::sqrt((5.)))),
        (std::sqrt(3.)+2.)/(2.*std::sqrt(5.)), (6.*std::sqrt(3.)-3)/(2.*std::sqrt(5.)), 0.;

    REQUIRE(radar_mm.H(state_vector).isApprox(measurement_matrix));
  }
}


TEST_CASE("RadarMeasurementModel<*>::Q", "[measurement_models]")
{
  RadarMeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;

  SECTION("CVProcessModel")
  {
    RadarMeasurementModel<CVProcessModel> radar_mm_cv;
    radar_mm_cv.SetMeasurementCovarianceMatrix(radar_mtx);

    REQUIRE(radar_mm_cv.Q().isApprox(radar_mtx));
  }

  SECTION("CTRVProcessModel")
  {
    RadarMeasurementModel<CTRVProcessModel> radar_mm_ctrv;
    radar_mm_ctrv.SetMeasurementCovarianceMatrix(radar_mtx);

    REQUIRE(radar_mm_ctrv.Q().isApprox(radar_mtx));
  }
}


TEST_CASE("RadarMeasurementModel<*>::Type", "[measurement_models]")
{
  REQUIRE(RadarMeasurementModel<CVProcessModel>::Type() == EntityType::e_MeasurementModel);
  REQUIRE(RadarMeasurementModel<CTRVProcessModel>::Type() == EntityType::e_MeasurementModel);
}


TEST_CASE("RadarMeasurementModel<*>::TypeName", "[measurement_models]")
{
  REQUIRE(std::string(RadarMeasurementModel<CVProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
  REQUIRE(std::string(RadarMeasurementModel<CTRVProcessModel>::TypeName()) == "MEASUREMENT_MODEL");
}


TEST_CASE("RadarMeasurementModel<*>::Kind", "[measurement_models]")
{
  REQUIRE(RadarMeasurementModel<CVProcessModel>::Kind() == MMKind::e_Radar);
  REQUIRE(RadarMeasurementModel<CTRVProcessModel>::Kind() == MMKind::e_Radar);
}


TEST_CASE("RadarMeasurementModel<*>::KindName", "[measurement_models]")
{
  REQUIRE(std::string(RadarMeasurementModel<CVProcessModel>::KindName()) == "RADAR");
  REQUIRE(std::string(RadarMeasurementModel<CTRVProcessModel>::KindName()) == "RADAR");
}


TEST_CASE("RadarMeasurementModel<*>::IsLinear", "[measurement_models]")
{
  REQUIRE(RadarMeasurementModel<CVProcessModel>::IsLinear() == false);
  REQUIRE(RadarMeasurementModel<CTRVProcessModel>::IsLinear() == false);
}


TEST_CASE("RadarMeasurementModel<*>::MeasurementDims", "[measurement_models]")
{
  REQUIRE(RadarMeasurementModel<CVProcessModel>::MeasurementDims() == 3);
  REQUIRE(RadarMeasurementModel<CTRVProcessModel>::MeasurementDims() == 3);
}


TEST_CASE("RadarMeasurementModel<*>::StateDims", "[measurement_models]")
{
  REQUIRE(RadarMeasurementModel<CVProcessModel>::StateDims() == 4);
  REQUIRE(RadarMeasurementModel<CTRVProcessModel>::StateDims() == 5);
}


TEST_CASE("RadarMeasurementModel<*>::GetInitialBeliefBasedOn", "[measurement_models]")
{
  RadarMeasurementVector mv;
  mv << 3., M_PI/6., 1.;

  RadarMeasurement measurement{2, mv};

  SECTION("CVProcessModel")
  {
    CVStateVector sv_cv;
    sv_cv << 3.*std::sqrt(3.)/2., 3./2., 0., 0.;

    CVStateCovarianceMatrix scm_cv{CVStateCovarianceMatrix::Identity()};

    auto belief_cv{RadarMeasurementModel<CVProcessModel>::GetInitialBeliefBasedOn(measurement)};
    CVProcessModel::Belief_type belief_expected_cv{measurement.t(), sv_cv, scm_cv};

    REQUIRE(belief_cv == belief_expected_cv);
  }

  SECTION("CTRVProcessModel")
  {
    CTRVStateVector sv_ctrv;
    sv_ctrv << 3. * std::sqrt(3.) / 2., 3. / 2., 0., 0., 0.;

    CTRVStateCovarianceMatrix scm_ctrv{CTRVStateCovarianceMatrix::Identity()};

    auto belief_ctrv{RadarMeasurementModel<CTRVProcessModel>::GetInitialBeliefBasedOn(measurement)};
    CTRVProcessModel::Belief_type belief_expected_ctrv{measurement.t(), sv_ctrv, scm_ctrv};

    REQUIRE(belief_ctrv == belief_expected_ctrv);
  }
}
