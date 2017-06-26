#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <stdlib.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[])
{
  std:: cout << argc << std::endl;
  std::cout << argv[1] << std::endl;
  std::cout << argv[2] << std::endl;
  std::cout << argv[3] << std::endl;

  // std::cout << atof(argv[0]) << endl;
  double Kp = atof(argv[1]);
  double Ki = atof(argv[2]);
  double Kd = atof(argv[3]);
  double throttle_value = atof(argv[4]);

  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(Kp, Ki, Kd, throttle_value);
  // 0.1, 0, 3 got around the track at 0.3 throttle
  // 0.1, 0, 20 made it around 0.5 throttle with a lot of (somewhat damped) oscillation
  // 0.15, 0, 20 did pretty goot at 0.4 throttle
  // 0.07, 0.001, 2 at 0.4 throtle was decent with a few excursions
  // 0.07, 0.001, 2 at 0.6 throttle was good with some oscillations, mostly stayed on track
//            double throttle = pid.throttle_value * (1.0-abs_cte*0.4);
//          if (throttle < 0.1) throttle = 0.1;

// 0.07, 0.001, 0.5 at 0.6 throttle was good with some oscillations, mostly stayed on track
//            double throttle = pid.throttle_value * (1.0-abs_cte*0.7);
//          if (throttle < 0.1) throttle = 0.1;

// 0.12, 0.001, 0.5 at 0.6 throttle was really good on turns but growing oscillation on straights and a bit slow in general
//            double throttle = pid.throttle_value * (1.0-abs_cte*1.0);
//          if (throttle < 0.1) throttle = 0.1;

// 0.1, 0.001, 0.8 at 0.7 best yet. All great. A little oscillation on straights and tiny overshoot on bigest right turn
//            double throttle = pid.throttle_value * (1.0-abs_cte*1.0);
//          if (throttle < 0.1) throttle = 0.1;

// 0.08, 0.001, 1.5 at 1.0 Oscillatory with and some exceedances on big turns, but not too bad for full throttle
//            double throttle = pid.throttle_value * (1.0-abs_cte*0.7);
//          if (throttle < 0.1) throttle = 0.1;

// 0.1, 0.001, 1.0 at 1.0 Pretty much stays in lines but way too oscillatory
//            double throttle = pid.throttle_value * (1.0-abs_cte*0.8);
//          if (throttle < 0.1) throttle = 0.1;

// 0.12, 0.001, 0.7 at 0.8 Pretty good, more oscillation than I would like. 
//            double throttle = pid.throttle_value * (1.0-abs_cte*0.8);
//          if (throttle < 0.1) throttle = 0.1;

// 0.08, 0.001, 0.5 at 0.8 less oscillation (still a decent amount) but not really making the tight corners
//            double throttle = pid.throttle_value * (1.0-abs_cte*0.8);
//          if (throttle < 0.1) throttle = 0.1;

// 0.08, 0.001, 0.5 at 0.8 much slowers and much less oscillation. Almost makes all corners, but not quite on big right
//            double throttle = pid.throttle_value * (1.0-abs_cte*1.2);
//          if (throttle < 0.1) throttle = 0.1;

// 0.1, 0.001, 0.8 at 1.0 not bad

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          double total_error = pid.TotalError();
          steer_value = total_error; 
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          double abs_cte = cte;
          if (abs_cte < 0) abs_cte *= -1;
          double throttle = pid.throttle_value * (1.0-abs_cte*1.2);
          if (throttle < 0.05) throttle = 0.05;
          if (speed < 25) throttle = 1.0;
          if (speed > 50) throttle = 0.0;
          msgJson["throttle"] = throttle;
          std::cout << cte << " " << throttle << std::endl;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
}
