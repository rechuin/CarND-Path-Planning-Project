

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"

// for convenience
using namespace std;
using nlohmann::json;
using std::string;
using std::vector;

double ref_vel = 0.0; // mph
double lane = 1; // I suggest lane should be int not double
const double MAX_VEL = 49.5;
const double MAX_ACC = .224;

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


  

  h.onMessage([&MAX_VEL, &MAX_ACC, &ref_vel, &lane,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          int prev_size = previous_path_x.size(); // previous path size


          //Define vector template parameters for calculating
          	vector<double> ptsx;
            vector<double> ptsy;
            vector<double> next_x_vals;
            vector<double> next_y_vals;
          
          // Prediction step. Analysing other cars positions and states.
            bool car_ahead = false;
            bool car_left = false;
            bool car_right = false;
            int car_lane = -1;
            
            //[Not important to initialize car_lane here. Only assign a value to car_lane after checking d value.Car_lane only take values 0, 1, 2]

          // Define car reference points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Prediction step. Analysing other cars positions and states.
          for ( int i = 0; i < sensor_fusion.size(); i++ )
          {
            float d = sensor_fusion[i][6];

            if (d>0 && d<4) // remote car is in lane 0
            {
              car_lane = 0;
            }

            if (d>4 && d<8) // remote car is in lane 1
            {
              car_lane = 1;
            }

            if (d>8 && d<12) // remote car is in lane 2
            
            //[for car to be in lane 2, d should be greater than 8 but less than 12]
            {
              car_lane = 2;
            }


            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double near_car_speed = sqrt(vx*vx + vy*vy);//magnitude calculation
            double near_car_s = sensor_fusion[i][5];
            
            // near_car_s needs to be updated e.g near_car_s += ((double)prev_size * .02 * near_car_speed);
            
            
            double roi_s = 28.0;
            double roi_offset = 15.0;

            // car go ahead
            if(car_lane == lane)
            {
              if (near_car_s > car_s && near_car_s-car_s < roi_s)
              
               
              //it should be "near_car_s-car_s < roi_s" and not "near_car_s-car_s > roi_s"
              // because if so then near car is pretty far away and we apparently don't consider that there's a car infront.
              // It's also important that we consider the speed(mph) of our near_car in this logic.Even if "near_car_s-car_s < roi_s" is true but "near_car_speed*2.24 > ref_vel",
              // we shouldn't consider there's a car ahead as it's moving faster than our ego car and w
              {
                car_ahead = true;
              }

              else { car_ahead = false; }
            }


            // car go right
            if (car_lane - lane > 1 )
            
            //[not good logic for me because this will be true for lane 1 and lane 2 but "if(car_lane == lane)" will be true for lane 1. So "if (car_lane - lane == 2 )" is preferable ]
            {
              if (car_s > near_car_s && abs(car_s - near_car_s) >= roi_offset)
              {
                car_right = true;
              }

              if (car_s < near_car_s && abs(car_s - near_car_s) >= roi_offset)
              {
                car_right = true;
              }
              else{ car_right = false;}            
            }


            // car go left
            else
            {
              if (car_s > near_car_s && abs(car_s - near_car_s) >= roi_offset)
              {
                car_left = true;
              }

              if (car_s < near_car_s && abs(car_s - near_car_s) >= roi_offset)
              {
                car_left = true;
              }

              else{ car_left = false; }
            }
            

          }

          // Behavior planner.
          int too_close = -1;   // I think too_close should be a flag( boolean) and int 
          
          
          
          double vel_diff = 0.0;  // this variable looks obsolete to me as I can't see it's use.
          //const double MAX_VEL = 49.5;
          //const double MAX_ACC = .224;

          if(car_ahead == true)
          {
            if (car_left == true && lane > 0) // change lane to left
            {
              lane = lane -1;
              too_close = 0;
            }

            if (car_right == true && lane < 2) // change lane to right
            {
              lane = lane + 1;
              too_close = 0;
            }

            else
            {
              lane = 1;
              too_close = 0;
            }
          }

          else
          {
            too_close = 1;
          }
          

          // motion planning
          if (too_close == 1)
          {
            ref_vel -= MAX_ACC;
          }
          if(ref_vel <= MAX_VEL)
          {
            ref_vel += MAX_ACC;
          }

          else if(ref_vel > MAX_VEL)
          {
            ref_vel = MAX_VEL - 0.1;
          }






          // check previous path size whether less than 4 elements
          // if so, need to creat some path tagent to the angle of the car
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y); // ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);  //ptsy.push_back(car_y);
          }

          else
          {
            // using the previous path's end point as starting reference
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];

            ref_yaw = atan2(prev_ref_y - ref_y, prev_ref_x - ref_x);

            ptsx.push_back(prev_ref_x);                    //   ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);              //  ptsx.push_back(ref_x);
            ptsy.push_back(prev_ref_y);                  //ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);             //ptsy.push_back(ref_y);
            
          }



          /**
            TODO: define a path made up of (x,y) points that the car will visit
            - Squentially every .02 seconds
            - 0.02 * 50MPH(or 22.352m/s)= 0.44704; neally 0.5
            - So in order to make 50 waypoints in the future, 
            - Vehicle position is update in every 0.5 distance interval*/

          /* In frenet add evenly 30 meters spaced points ahead of the staring reference
            The origin of the starting point is already calculated in above code. */
            vector<double> next_w0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_w1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_w2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_w0[0]);
            ptsy.push_back(next_w0[1]);

            ptsx.push_back(next_w1[0]);
            ptsy.push_back(next_w1[1]);

            ptsx.push_back(next_w2[0]);
            ptsy.push_back(next_w2[1]);

          // shift data -> shift the beginning points of path at the (0,0) and the car reference angle to 0 as well 
          // instead of map coodinates with angles 
          for (int i = 0; i < ptsx.size(); i ++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            // transfer the coordinate
            ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
            ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
            
            
            // ptsy should be ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw)); and not  ptsy[i] = (shift_x*sin(0 - ref_yaw) - shift_y*cos(0 - ref_yaw));

          }

          // creat a spline
          tk::spline s;

          // Set (x,y) points to the spline. ptsx and ptsy is a set of [x1,x2,x3...],[y1,y2,y3...]
          // here using 5 points to build a spline function
          s.set_points(ptsx, ptsy);// fitting the point into spline

          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_y*target_y + target_x*target_x);
          
          double x_add_on = 0;
          // fill up the rest of our path planner after filling it with previous points
          // here we will always output 50 points
          for (int i = 0; i <= 50 - previous_path_x.size(); i++)
          {


            double N = (target_dist/(0.02*ref_vel)/2.24); // dividing 2.24 because unity changing in miles/hour
            double x_point;
            double y_point;

            x_point = x_add_on + target_x / N;
            y_point = s(x_point);
            x_add_on = x_point;

            // rotate back to normal after rotating it earlier
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) - y_ref*cos(ref_yaw));

            x_point += ref_x;    // x_point += ref_x;
            y_point += ref_y;    //y_point += ref_y; 

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          /**
           * End
           */
          json msgJson;
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
