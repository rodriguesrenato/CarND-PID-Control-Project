#include <math.h>
#include <uWS/uWS.h>

#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "PID.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

enum Phase { LowSpeed, HighSpeed };

int main() {
  uWS::Hub h;

  PID pid;
  // Initialize the pid controller params
  std::map<Phase, vector<double>> pid_params;
  pid_params[Phase::LowSpeed] = {0.12, 0.0, 2.5};
  pid_params[Phase::HighSpeed] = {0.12, 0.006, 3.5};
  auto p = pid_params[Phase::LowSpeed];
  pid.Init(p[0], p[1], p[2]);
  Phase phase = Phase::LowSpeed;

  h.onMessage([&pid, &phase, &pid_params](uWS::WebSocket<uWS::SERVER> ws,
                                          char *data, size_t length,
                                          uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());

          double throttle = 0.30;
          if (speed > 20.0 && phase != Phase::HighSpeed) {
            auto p = pid_params[Phase::HighSpeed];
            pid.Init(p[0], p[1], p[2]);
            phase = Phase::HighSpeed;
          } else if (speed < 20.0 && phase != Phase::LowSpeed) {
            auto p = pid_params[Phase::LowSpeed];
            pid.Init(p[0], p[1], p[2]);
            phase = Phase::LowSpeed;
          }

          pid.UpdateError(cte);

          double steer_value = pid.TotalError();
          if (steer_value > 1) steer_value = 1;
          if (steer_value < -1) steer_value = -1;

          // DEBUG
          std::cout << std::fixed << std::setprecision(3) << std::setw(8)
                    << "CTE: " << cte << "\tangle/steer: " << std::setw(8)
                    << angle << "| " << std::setw(8) << steer_value;
          pid.Print();
          std::cout << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}