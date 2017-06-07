#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.hpp"
#include "json.hpp"
#include "utils.hpp"

// for convenience
using json = nlohmann::json;

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
                    double v_mps = mph2mps(v);
                    double delta = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];
                    double a = throttle;


                    // These two vectors hold vehicle coordinates
                    vector<double> next_x(ptsx.size());
                    vector<double> next_y(ptsy.size());

                    State currentState = {px, py, psi, v_mps};
                    auto delayedState = globalKinematic(currentState, delta, a, 0.1, LF);

                    for (int i = 0; i < ptsx.size(); i++) {
                        double map_x = ptsx[i] - delayedState.x;
                        double map_y = ptsy[i] - delayedState.y;
                        double cos_psi = cos(delayedState.psi);
                        double sin_psi = sin(delayedState.psi);

                        next_x[i] = map_x * cos_psi + map_y * sin_psi;
                        next_y[i] = -map_x * sin_psi + map_y * cos_psi;
                    }

                    auto coeffs = polyfit(
                            Eigen::Map<Eigen::VectorXd>(&next_x[0], next_x.size()),
                            Eigen::Map<Eigen::VectorXd>(&next_y[0], next_y.size()),
                            2
                    );

                    double cte = polyeval(coeffs, 0);
                    double epsi = -atan(coeffs[1]);

                    Eigen::VectorXd state(6);
                    state << 0.0, 0.0, 0.0, delayedState.v, cte, epsi;

                    mpc.Solve(state, coeffs);


                    json msgJson;
                    msgJson["steering_angle"] = mpc.steer / deg2rad(25);
                    msgJson["throttle"] = mpc.throttle;
                    std::cout << "throttle: " << msgJson["throttle"] << std::endl;

                    // Projected trajectory
                    msgJson["mpc_x"] = mpc.x_vals;
                    msgJson["mpc_y"] = mpc.y_vals;

                    // Waypoints next X and Y coordinates
                    msgJson["next_x"] = next_x;
                    msgJson["next_y"] = next_y;


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
