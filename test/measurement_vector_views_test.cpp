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
#include "measurement_vector_views.hpp"

#include <catch.hpp>


using namespace ser94mor::sensor_fusion;


TEST_CASE("Lidar::ROMeasurementVectorView", "[measurement_vector_views]")
{
  Lidar::MeasurementVector mv;
  mv << 2., 1.;

  Lidar::ROMeasurementVectorView mvv{mv};

  REQUIRE(Approx(2.) == mvv.px());
  REQUIRE(Approx(1.) == mvv.py());
}


TEST_CASE("Radar::ROMeasurementVectorView", "[measurement_vector_views]")
{
  Radar::MeasurementVector mv;
  mv << 4., M_PI/6., 2.;

  Radar::ROMeasurementVectorView mvv{mv};

  REQUIRE(Approx(4.) == mvv.range());
  REQUIRE(Approx(M_PI/6.) == mvv.bearing());
  REQUIRE(Approx(2.) == mvv.range_rate());
  REQUIRE(Approx(2.*std::sqrt(3.)) == mvv.px());
  REQUIRE(Approx(2.) == mvv.py());
}
