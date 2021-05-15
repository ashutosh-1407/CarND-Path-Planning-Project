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

// for convenience
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
  
  // defining lane and target speed
  
  int lane = 1;
  double target_speed = 0.0;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&target_speed]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // I have taken the reference from the Project Q&A video provided in the project module.
          
          int prev_path_size = previous_path_x.size();
          bool too_close = false;
          bool too_close_left = false;
          bool too_close_right = false;
          string behavior = "KL";
          
          // setting car_s to the previous path's end_s value if the previous path size is greater then 0
          if (prev_path_size > 0) {
            car_s = end_path_s;
          }
          
          // checking if there is a vehicle in front of us and is within range of 30m
          too_close = tooClose(lane, prev_path_size, car_s, sensor_fusion, behavior);
          
          if (too_close) {
            int intended_left_lane = lane - 1;
            int intended_right_lane = lane + 1;
            bool trying_lane_change = false;
            // checking if lane change left is safe
            if (intended_left_lane >= 0) {
              behavior = "LCL";
              too_close_left = tooClose(intended_left_lane, prev_path_size, car_s, sensor_fusion, behavior);
              if (!too_close_left) {
                lane = intended_left_lane;
                trying_lane_change = true;
              }
            }
            // checking if lane change right is safe
            if (intended_right_lane < 3 and !trying_lane_change) {
              behavior = "LCR";
              too_close_right = tooClose(intended_right_lane, prev_path_size, car_s, sensor_fusion, behavior);
              if (!too_close_right) {
                lane = intended_right_lane;
              }
            }
            // deacclerating the car
            target_speed-=.224;
          }
          else if (target_speed < 49.5){
            // acclerating the car
            target_speed+=.224;
          }
          
          // vectors to be used to define spline
          vector<double> pts_x;
          vector<double> pts_y;
          
          // car's reference points to be set either to car's x, y and yaw angle or from previous path's value
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if (prev_path_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);
            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
          }
          else {
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];
            double prev_ref_x = previous_path_x[prev_path_size - 2];
            double prev_ref_y = previous_path_y[prev_path_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
            pts_x.push_back(prev_ref_x);
            pts_x.push_back(ref_x);
            pts_y.push_back(prev_ref_y);
            pts_y.push_back(ref_y);
          }
          
          vector<double> next_pt1 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_pt2 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_pt3 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          pts_x.push_back(next_pt1[0]);
          pts_x.push_back(next_pt2[0]);
          pts_x.push_back(next_pt3[0]);
          pts_y.push_back(next_pt1[1]);
          pts_y.push_back(next_pt2[1]);
          pts_y.push_back(next_pt3[1]);
          
          // changing the reference frame to vehicle
          for(int i=0; i<pts_x.size(); i++) {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;
            pts_x[i] = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
            pts_y[i] = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);
          }
          
          // fitting spline to pts_x and pts_y
          tk::spline spl;
          spl.set_points(pts_x, pts_y);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // appending points to the main vectors next_x_vals and next_y_vals to be used by the controller
          for(int i=0; i<previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = spl(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double add_on = 0;
          
          for(int i=1; i<=50-previous_path_x.size(); i++) {
            double N = (target_dist/(0.02*target_speed/2.24));
            double next_x = add_on + (target_x/N);
            double next_y = spl(next_x);
            add_on = next_x;
            double x_ref =  next_x;
            double y_ref = next_y;
            // converting back to world frame
            next_x = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            next_y = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            next_x+=ref_x;
            next_y+=ref_y;
            
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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