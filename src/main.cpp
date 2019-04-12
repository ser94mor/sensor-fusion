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

#include "fusion/Fusion.hpp"

#include <syslog.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <boost/program_options.hpp>


using namespace boost::program_options;

const char *filter_to_str(filter_t filter) {
    switch (filter) {
        case EKF: return "EKF";
        case UKF: return "UKF";
        default: throw std::invalid_argument("filter");
    }
}

const char *motion_model_to_str(motion_model_t motion_model) {
    switch (motion_model) {
        case CTRV: return "CTRV";
        case CV: return "CV";
        default: throw std::invalid_argument("motion_model");
    }
}

std::string sensor_types_to_str(int sensor_types) {
    std::ostringstream oss;
    if (sensor_types | RADAR) {
        oss << " RADAR ";
    }

    if (sensor_types | LIDAR) {
        oss << " LIDAR ";
    }

    std::string str = oss.str();

    if (str.empty()) {
        throw std::invalid_argument("sensor_types");
    }

    return str;
}

void parse_args(int argc, char *argv[],
                motion_model_t *motion_model, filter_t *filter, int *sensor_types) {
    options_description desc("Options");
    desc.add_options()
            ("model,m",   value<std::string>()->required(), "motion model (either CTRV, or CV)")
            ("filter,f",  value<std::string>()->required(), "filter (either EKF, or UKF)")
            ("sensors,s", value< std::vector<std::string> >()->required()->multitoken(),
                          "sensor types to process (RADAR or LIDAR)")
            ("help,h", "print this message")
            ;
    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    // set filter type
    if (vm["filter"].as<std::string>() == "EKF") {
        *filter = EKF;
    } else if (vm["filter"].as<std::string>() == "UKF") {
        *filter = UKF;
    } else {
        throw validation_error(validation_error::invalid_option_value, "filter", "filter");
    }

    // set motion model type
    if (vm["model"].as<std::string>() == "CTRV") {
        *motion_model = CTRV;
    } else if (vm["model"].as<std::string>() == "CV") {
        *motion_model = CV;
    } else {
        throw validation_error(validation_error::invalid_option_value, "model", "model");
    }

    auto const &sts = vm["sensors"].as< std::vector<std::string> >();
    *sensor_types = 0;
    for (const std::string &sensor : sts) {
        if (sensor == "LIDAR") {
            *sensor_types |= LIDAR;
        } else if (sensor == "RADAR") {
            *sensor_types |= RADAR;
        } else {
            throw validation_error(validation_error::invalid_option_value, "sensors", "sensors");
        }
    }
}

int main(int argc, char *argv[]) {

    openlog(argv[0], LOG_PID, LOG_USER);

    motion_model_t motion_model;
    filter_t filter;
    int sensor_types;
    parse_args(argc, argv, &motion_model, &filter, &sensor_types);

    syslog(LOG_INFO,
           "Filter: %s; Motion Model: %s; Sensor types: %s",
           filter_to_str(filter),
           motion_model_to_str(motion_model),
           sensor_types_to_str(sensor_types).c_str());

    SensorFusion sensor_fusion(filter, motion_model, sensor_types);

    closelog();
    return 0;
}
