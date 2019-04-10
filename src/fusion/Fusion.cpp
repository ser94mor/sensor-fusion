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

#include "Fusion.hpp"
#include "../filters/ExtendedKalmanFilter.h"
#include "../filters/UnscentedKalmanFilter.h"
#include "models/ConstantTurnRateAndVelocityModel.h"
#include "models/ConstantVelocityModel.hpp"

SensorFusion::SensorFusion(filter_t filter, motion_model_t motion_model, int sensor_types) {
    switch (filter) {
        case EKF: {
            filter_ = ExtendedKalmanFilter();
            break;
        }
        case UKF: {
            filter_ = UnscentedKalmanFilter();
            break;
        }
        default: {
            throw std::invalid_argument("filter");
        }
    }

    switch (motion_model) {
        case CTRV: {
            motion_model_ = ConstantTurnRateAndVelocityModel();
            break;
        }
        case CV: {
            motion_model_ = CVProcessModel();
            break;
        }
        default: {
            throw std::invalid_argument("motion_model");
        }
    }

    if (not ((sensor_types & LIDAR) || (sensor_types & RADAR))) {
        throw std::invalid_argument("sensor_types");
    }
    sensor_types_ = sensor_types;

    initialized_ = false;

    prev_meas_timestamp_ = 0;

    meas_counter_ = 0;
}

void SensorFusion::ProcessMeasurement(const MeasurementPackage &meas_pack) {
    if (not initialized_) {


        initialized_ = true;
    }

    filter_.Predict(state)
}

SensorFusion::~SensorFusion() {}


