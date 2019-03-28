#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "FSM.h"

using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    double ref_velocity = 0.0; // mph
    FSM fsm = FSM();
    const double DELTA_T = 0.02;

    h.onMessage([&ref_velocity, &fsm, &DELTA_T, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx,
                        &map_waypoints_dy]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */

                    int prev_size = previous_path_x.size();

                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    bool too_close = false;
                    double object_speed_m_s = 0;

                    for (auto &detected_object : sensor_fusion) {
                        float object_d = detected_object[6];
                        int egoLaneId = getLaneId(car_d);
                        const int objectLaneId = getLaneId(object_d);
                        if (objectLaneId == egoLaneId) {
                            float vx = detected_object[3];
                            float vy = detected_object[4];
                            object_speed_m_s = sqrt(vx * vx + vy * vy);
                            double object_s = detected_object[5];

                            object_s += prev_size * DELTA_T * object_speed_m_s;

                            too_close = object_s > car_s && object_s - car_s < 30;

                            if (too_close) {
                                if (egoLaneId != 0) {
                                    fsm.changeLaneLeft(egoLaneId);
                                } else {
                                    fsm.changeLaneRight(egoLaneId);
                                }
                            } else {
                                fsm.keepLane();
                            }
                        }
                    }

                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_yaw = deg2rad(car_yaw);

                    double ref_x;
                    double ref_y;
                    double ref_x_prev;
                    double ref_y_prev;
                    if (prev_size < 2) {
                        ref_x = car_x;
                        ref_y = car_y;
                        ref_x_prev = car_x - cos(car_yaw);
                        ref_y_prev = car_y - sin(car_yaw);
                    } else {
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];
                        ref_x_prev = previous_path_x[prev_size - 2];
                        ref_y_prev = previous_path_y[prev_size - 2];
                    }
                    ptsx.push_back(ref_x_prev);
                    ptsy.push_back(ref_y_prev);
                    ptsx.push_back(ref_x);
                    ptsy.push_back(ref_y);


                    double dIntended = dLaneCenter(fsm.getIntendedLaneId());
                    vector<double> next_wp0 = getXY(car_s + 30, dIntended, map_waypoints_s, map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 60, dIntended, map_waypoints_s, map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 90, dIntended, map_waypoints_s, map_waypoints_x,
                                                    map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    // transformation to car coordinate system
                    for (int i = 0; i < ptsx.size(); i++) {
                        vector<double> xy_vehicle = transformWorld2Vehicle(ptsx[i], ptsy[i], ref_yaw, ref_x, ref_y);
                        ptsx[i] = xy_vehicle[0];
                        ptsy[i] = xy_vehicle[1];
                    }

                    // start with points left from previous path
                    for (int i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // create spline
                    tk::spline spl;
                    spl.set_points(ptsx, ptsy);

                    double target_x = 30.0;
                    double target_y = spl(target_x);
                    double target_distance = sqrt(target_x * target_x + target_y * target_y);

                    double x_vehicle = 0;
                    const double MAX_SPEED = 49.5; // mph
                    const double DELTA_VELOCITY = 0.224; // corresponds to acceleration of around 5/m/s/s
                    for (int i = 0; i < 50 - previous_path_x.size(); i++) {
                        if (too_close && mph2mps(ref_velocity) > object_speed_m_s) {
                            ref_velocity -= DELTA_VELOCITY;
                        } else if (ref_velocity < MAX_SPEED) {
                            ref_velocity += DELTA_VELOCITY;
                        }
                        double x_increment = target_x * mph2mps(ref_velocity) * DELTA_T / target_distance;
                        x_vehicle += x_increment;
                        double y_vehicle = spl(x_vehicle);

                        vector<double> xy_world = transformVehicle2World(x_vehicle, y_vehicle, ref_yaw, ref_x, ref_y);

                        next_x_vals.push_back(xy_world[0]);
                        next_y_vals.push_back(xy_world[1]);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
