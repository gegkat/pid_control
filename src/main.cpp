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

  double Kp, Ki, Kd;
  if (argc == 4) {
    Kp = atof(argv[1]);
    Ki = atof(argv[2]);
    Kd = atof(argv[3]);
  } else {
    Kp = 0.08;
    Ki = 0.001;
    Kd = 1.8;
  }

  uWS::Hub h;

  PID pid;
  // Initialize the pid variable.
  pid.Init(Kp, Ki, Kd);
  std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
  std::cout << "cte, steer, throttle, speed" << std::endl;
    
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    double THROTTLE_CTE_COEFFICIENT = 0.8; 
    double MIN_SPEED = 55;
    double MIN_THROTTLE = 0.05;
    double MAX_THROTTLE = 1.0;

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
          *  Calcuate steering value 
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          /*
          *  Calcuate throttle value
          */
          double abs_cte = cte;
          // Get absolute value of cross track error
          if (abs_cte < 0) abs_cte *= -1;

          // When abs_cte is 0, throttle = max_throttle, when cte is larger 
          // then reduce throttle
          double throttle = MAX_THROTTLE * (1.0-abs_cte*THROTTLE_CTE_COEFFICIENT);

          // Limit minimum throttle to 0.1
          if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;

          // If speed is below 25, full throttle, if above 50 no throttle
          if (speed < MIN_SPEED) throttle = MAX_THROTTLE;


          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // send data
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          std::cout << cte << ", " << steer_value << ", " << throttle << ", " << speed << std::endl;
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
