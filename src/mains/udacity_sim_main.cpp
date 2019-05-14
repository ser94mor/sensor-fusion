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
#include "fusion.hpp"
#include "filters.hpp"
#include "process_models.hpp"

#include <Eigen/Dense>
#include <json.hpp>
#include <uWS/uWS.h>
#include <syslog.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <boost/program_options.hpp>



using namespace ser94mor::sensor_fusion;


using namespace std;
using namespace Eigen;

// for convenience
using json = nlohmann::json;


class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.empty()){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of(']');
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

constexpr double_t us_to_s(std::time_t us)
{
  return us / 1000000.0;
}


int main(int, char* argv[])
{
  openlog(argv[0], LOG_PID, LOG_USER);

  CV::ProcessNoiseCovarianceMatrix cv_mtx;
  cv_mtx << 9.0, 0.0,
            0.0, 9.0;

  CTRV::ProcessNoiseCovarianceMatrix ctrv_mtx;
  ctrv_mtx << 0.126025,  0.0,
              0.0,      0.16;

  Lidar::MeasurementCovarianceMatrix lidar_mtx;
  lidar_mtx << 0.0225,    0.0,
               0.0, 0.0225;
  
  Radar::MeasurementCovarianceMatrix radar_mtx;
  radar_mtx << 0.09,    0.0,  0.0,
                0.0, 0.0009,  0.0,
                0.0,    0.0, 0.09;

  UKF_CTRV_LIDAR_RADAR_Fusion fusion{ctrv_mtx, lidar_mtx, radar_mtx};

  uWS::Hub h;

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;


  h.onMessage(
      [&fusion,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (!s.empty()) {

        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          string sensor_measurment = j[1]["sensor_measurement"];

          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type == "L") {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          } else if (sensor_type == "R") {

            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);

          VectorXd estimate(4);

          if (meas_package.sensor_type_ == MeasurementPackage::LASER)
          {
            Lidar::Measurement measurement{us_to_s(meas_package.timestamp_), meas_package.raw_measurements_};

            auto belief{fusion.ProcessMeasurement(measurement)};

            const auto& sv{belief.mu()};
            CTRV::ROStateVectorView state_vector_view{sv};
            estimate(0) = state_vector_view.px();
            estimate(1) = state_vector_view.py();
            estimate(2) = state_vector_view.vx();
            estimate(3) = state_vector_view.vy();

            estimations.push_back(estimate);
          } else {
            Radar::Measurement measurement{us_to_s(meas_package.timestamp_), meas_package.raw_measurements_};

            auto belief{fusion.ProcessMeasurement(measurement)};

            const auto& sv{belief.mu()};
            CTRV::ROStateVectorView state_vector_view{sv};
            estimate(0) = state_vector_view.px();
            estimate(1) = state_vector_view.py();
            estimate(2) = state_vector_view.vx();
            estimate(3) = state_vector_view.vy();

            estimations.push_back(estimate);
          }


          VectorXd RMSE = CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = estimate(0);
          msgJson["estimate_y"] = estimate(1);
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);


          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {

        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char*, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int, char*, size_t) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();




  closelog();
  return EXIT_SUCCESS;
}
