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
#include "measurement_vector_views.hpp"

#include <catch.hpp>


using namespace ser94mor::sensor_fusion;

///////////
// LIDAR //
///////////

TEST_CASE("LidarROMeasurementVectorView", "[measurement_vector_views]")
{
  LidarMeasurementVector mv;
  mv << 2., 1.;

  LidarROMeasurementVectorView mvv{mv};

  REQUIRE(Approx(2.) == mvv.px());
  REQUIRE(Approx(1.) == mvv.py());
}

TEST_CASE("LidarRWMeasurementVectorView", "[measurement_vector_views]")
{
  LidarMeasurementVector mv;
  mv << 2., 1.;

  LidarRWMeasurementVectorView mvv{mv};

  //=====

  REQUIRE(mvv.px() == Approx(2.));
  REQUIRE(mvv.py() == Approx(1.));

  //=====

  mvv.px() = 3.;
  REQUIRE(mvv.px() == Approx(3.));
  REQUIRE(mvv.py() == Approx(1.));

  //=====

  mvv.py() = 4.;
  REQUIRE(mvv.px() == Approx(3.));
  REQUIRE(mvv.py() == Approx(4.));

  //=====

  LidarMeasurementVector mv_expected;
  mv_expected << 3., 4.;

  REQUIRE(mv.isApprox(mv_expected));
}


///////////
// RADAR //
///////////

TEST_CASE("RadarROMeasurementVectorView", "[measurement_vector_views]")
{
  RadarMeasurementVector mv;
  mv << 4., M_PI/6., 2.;

  RadarROMeasurementVectorView mvv{mv};

  REQUIRE(Approx(4.) == mvv.range());
  REQUIRE(Approx(M_PI/6.) == mvv.bearing());
  REQUIRE(Approx(2.) == mvv.range_rate());
  REQUIRE(Approx(2.*std::sqrt(3.)) == mvv.px());
  REQUIRE(Approx(2.) == mvv.py());
}

TEST_CASE("RadarRWMeasurementVectorView", "[measurement_vector_views]")
{
  RadarMeasurementVector mv;
  mv << 2., M_PI_4, 3.;

  RadarRWMeasurementVectorView mvv{mv};

  //=====

  REQUIRE(mvv.range() == Approx(2.));
  REQUIRE(mvv.bearing() == Approx(M_PI_4));
  REQUIRE(mvv.range_rate() == Approx(3.));

  //=====

  mvv.range() = 10.;
  REQUIRE(mvv.range() == Approx(10.));
  REQUIRE(mvv.bearing() == Approx(M_PI_4));
  REQUIRE(mvv.range_rate() == Approx(3.));

  //=====

  mvv.bearing() = M_PI_2;
  REQUIRE(mvv.range() == Approx(10.));
  REQUIRE(mvv.bearing() == Approx(M_PI_2));
  REQUIRE(mvv.range_rate() == Approx(3.));

  //=====

  mvv.range_rate() = 20.;
  REQUIRE(mvv.range() == Approx(10.));
  REQUIRE(mvv.bearing() == Approx(M_PI_2));
  REQUIRE(mvv.range_rate() == Approx(20.));

  //=====

  RadarMeasurementVector mv_expected;
  mv_expected << 10., M_PI_2, 20.;

  REQUIRE(mv.isApprox(mv_expected));
}
