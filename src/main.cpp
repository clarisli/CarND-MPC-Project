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

// plot
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
const int max_iters = 100;
const bool plot = FALSE;

// for convenience
using json = nlohmann::json;

const long latency = 100; // 100ms latency delay

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

  // plot
  int iters = 0;
  std::vector<double> x_vals = {};
  std::vector<double> y_vals = {};
  std::vector<double> psi_vals = {};
  std::vector<double> v_vals = {};
  std::vector<double> cte_vals = {};
  std::vector<double> epsi_vals = {};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};

  h.onMessage([&mpc, &iters, &x_vals, &y_vals, &psi_vals, &v_vals, &cte_vals, &epsi_vals, &delta_vals, &a_vals](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
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


          /* 1. Simulate latency
          *
          * In a real car, an actuation command won't execute instantly 
          * - there will be a delay as the command propagates through 
          * the system. A realistic delay might be on the order of 100 
          * milliseconds.
          *
          */
          std::vector<double> state0 = {px, py, psi, v};
          std::vector<double> actuations0 = {a, delta};
          std::vector<double> state1 = mpc.Simulate(state0, actuations0, latency/1000);


          /* 2. Transfrom the waypoints from map's coordinates to vehicle's coordinates
          *
          * 1. Shift ptsx and ptsy to car's origin.
          * 2. Rotate ptsx and ptsy from map's coordinates to car's coordinates, w.r.t psi
          *
          */
          // x, y, psi, v after latency 100ms
          double px1 = state1[0];
          double py1 = state1[1];
          double psi1 = state1[2];
          double v1 = state1[3];

          size_t n_waypoints = ptsx.size();
          auto ptsx_transformed = Eigen::VectorXd(n_waypoints);
          auto ptsy_transformed = Eigen::VectorXd(n_waypoints);
          for (unsigned int i = 0; i < n_waypoints; i++ ) {
            double dX = ptsx[i] - px1;
            double dY = ptsy[i] - py1;
            ptsx_transformed(i) = dX * cos(-psi1) - dY * sin(-psi1);
            ptsy_transformed(i) = dX * sin(-psi1) + dY * cos(-psi1);
          }

          // 3. Fit a polynomial to the waypoints
          auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);

          // 4. Calculate errors: cte and epsi
          double cte = polyeval(coeffs, 0) - 0;
          double epsi = 0 - atan(coeffs[1]);

          // plot
          if(plot){
            iters++;
            x_vals.push_back(px1);
            y_vals.push_back(py1);
            psi_vals.push_back(psi1);
            v_vals.push_back(v1);
            cte_vals.push_back(cte);
            epsi_vals.push_back(epsi);    
            delta_vals.push_back(delta);
            a_vals.push_back(a);
          }

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // State vector
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v1, cte, epsi;



          // Find MPC solution
          auto mpc_actuactions = mpc.Solve(state, coeffs); 
          double mpc_delta = mpc_actuactions[0];
          double mpc_a = mpc_actuactions[1];

          double steer_value = mpc_delta/deg2rad(25);
          double throttle_value = mpc_a;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (unsigned int i = 2; i < mpc_actuactions.size(); i++) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(mpc_actuactions[i]);
            } else {
              mpc_y_vals.push_back(mpc_actuactions[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < ptsx_transformed.size(); i++) {
            next_x_vals.push_back(ptsx_transformed[i]);
            next_y_vals.push_back(ptsy_transformed[i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latency));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if(iters > max_iters && plot){
            //Plot Values, first 100 iterations
            plt::subplot(3, 1, 1);
            plt::title("CTE");
            plt::plot(cte_vals);
            plt::subplot(3, 1, 2);
            plt::title("Delta (Radians)");
            plt::plot(delta_vals);
            plt::subplot(3, 1, 3);
            plt::title("Velocity");
            plt::plot(v_vals);      
            plt::show();
            iters = 0;
            exit(1);
          }
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
