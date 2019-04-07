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

#ifndef SENSOR_FUSION_SENSORFUSION_HPP
#define SENSOR_FUSION_SENSORFUSION_HPP


#include "BayesFilter.hpp"
#include "State.hpp"

#include <eigen3/Eigen/Dense>

typedef enum {
    LIDAR = 0x01,
    RADAR = 0x02,
} sensor_t;

typedef enum {
    CTRV = 1,
    CV   = 2,
} motion_model_t;

typedef enum {
    EKF = 1,
    UKF = 2,
} filter_t;

struct MeasurementPackage {
    long long timestamp;

    sensor_t sensor_type;

    Eigen::VectorXd raw_measurements;
};

class SensorFusion {

public:
    /**
     * Constructor.
     */
    SensorFusion(filter_t filter, motion_model_t motion_model, int sensor_types);

    /**
    * Destructor.
    */
    virtual ~SensorFusion();

    /**
    * Run the whole flow of the Kalman Filter from here.
    */
    void ProcessMeasurement(const MeasurementPackage &meas_pack);

private:

    // concrete implementation of filter
    BayesFilter filter_;

    // mean and covariance matrix
    State state_;

    // flag indicating whether the first measurement processed
    bool initialized_;

    // counter of processed measurements
    uint64_t meas_counter_;

    // timestamp of the previously processed measurement
    uint64_t prev_meas_timestamp_;

    // data from which sensors to process
    int sensor_types_;



};


#endif //SENSOR_FUSION_SENSORFUSION_HPP
