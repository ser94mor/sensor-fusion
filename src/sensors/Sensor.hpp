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

#ifndef SENSOR_FUSION_SENSOR_HPP
#define SENSOR_FUSION_SENSOR_HPP


#include "definitions.hpp"

#include <optional>


#define SENSOR_DEFINITION() \
  class Sensor : public ser94mor::sensor_fusion::Sensor<Measurement, kSensorName> { };


namespace  ser94mor
{
  namespace sensor_fusion
  {

    template<class Measurement, const char* name>
    class Sensor
    {
    public:
      constexpr static const char* Type()
      {
        return kSensorType;
      }

      constexpr static const char* Name()
      {
        return name;
      }

      Measurement* GetMeasurementIfExists() const;

      void SetMeasurement(Measurement measurement);

    private:
      Measurement measurement_;
    };

    template<class Measurement, const char* name>
    Measurement* Sensor<Measurement, name>::GetMeasurementIfExists() const
    {
      return nullptr;
    }

    template<class Measurement, const char* name>
    void Sensor<Measurement, name>::SetMeasurement(const Measurement measurement)
    {
      this->measurement_ = measurement;
    }

  }
}


#endif //SENSOR_FUSION_SENSOR_HPP
