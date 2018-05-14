#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "Solution.h"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /************************************************************
           * Calculate Reference Points
           ***********************************************************/
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());

          for (unsigned int i = 1; i < ptsx.size(); i++)
          {
            // Reference trajectory.
            double n_x = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
            double n_y = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);
            next_x_vals.push_back(n_x);
            next_y_vals.push_back(n_y);
            xvals[i] = n_x;
            yvals[i] = n_y;
          }

          /************************************************************
          * Calculate Steering Angle & Throttle [-1, 1]
          ***********************************************************/
          // Fit a polynomial to the path.
          auto coeffs = polyfit(xvals, yvals, 3);
          double epsi = -atan(coeffs[1]);
          double cte = polyeval(coeffs, 0);

          /************************************************************
           * Adjust for Latency (Global Frame)
           ***********************************************************/
          const double latency_dt = 0.1;
          const double Lf = 2.67;

          psi = 0.0;
          px = v * cos(psi) * latency_dt;
          py = v * sin(psi) * latency_dt;
          psi = v * (-delta) / Lf * latency_dt;
          v += a * latency_dt;
          cte += v * sin(epsi) * latency_dt;
          epsi += v * (-delta) / Lf * latency_dt;

          // Add everything to the state
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          Solution solution = mpc.Solve(state, coeffs);

          // Update JSON message to simulator.
          json msgJson;
          msgJson["steering_angle"] = solution.steer/(deg2rad(25));
          msgJson["throttle"] = solution.throttle;
          msgJson["mpc_x"] = solution.xvals;
          msgJson["mpc_y"] = solution.yvals;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          /* Print Debug Message */
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
