#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "PathPlanner.h"
#include "BehaviourPlanner.h"
#include "MovementPredictor.h"
#include "spline.h"


using namespace std;

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
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

int main() {
    uWS::Hub h;
    MovementPredictor predictor;
    PathPlanner pathPlanner;
    BehaviourPlanner behaviourPlanner;

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

    // TODO: Parameters!
    double lane = 1;
    double ref_speed = 0.0; //MPH

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
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

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane, &ref_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (!s.empty()) {
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

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;




                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    // AARON'S CODE START

                    int prev_size = previous_path_x.size();

                    //// SENSOR PART
                    bool too_close = false;

                    if(prev_size > 0) {
                        car_s = end_path_s;
                    }

                    for (auto &i : sensor_fusion) {
                        // Car in my lane
                        float d = i[6]; // other car's D value
                        if(d < (2+4*lane+2) && d > (2+4*lane-2)) {
                            double vx = i[3]; // other car's Vx value
                            double vy = i[4]; // other car's Vy value
                            double check_speed = std::sqrt(vx*vx + vy*vy);
                            double check_car_s = i[5]; // other car's S value

                            // if using previous points can project s value out
                            check_car_s += ((double)prev_size*0.02*check_speed); // predicted position 0,02s in the future

                            // check s values greater than mine and s gap
                            if((check_car_s > car_s) && ((check_car_s-car_s) < 30.0)) {
                                // Do some logic here, lower reference velocity so we don't crash into the car in front of us
                                // Could also flag to try to change lanes
                                ref_speed = 30.0; // MPH
                                too_close = true;

                                if(lane == 1) {
                                    lane = 0;
                                }
                            }
                        }
                    }

                    if(too_close) {
                        ref_speed -= 0.224; // MPH
                    } else if (ref_speed < 49.65) { // MPH
                        ref_speed += 0.224; // MPH
                    }

                    //*/

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    std::vector<double> ptsx;
                    std::vector<double> ptsy;

                    // if previous path is almost empty, use the car's reference point as a starting point
                    if(prev_size < 2) {
                        ptsx.push_back(car_x - std::cos(car_yaw)); // prev_car_x
                        ptsx.push_back(car_x);

                        ptsy.push_back(car_y - std::sin(car_yaw)); // prev_car_y
                        ptsy.push_back(car_y);

                    } else {
                        ref_x = previous_path_x[prev_size-1];
                        ref_y = previous_path_y[prev_size-1];

                        double ref_x_prev = previous_path_x[prev_size-2];
                        double ref_y_prev = previous_path_y[prev_size-2];

                        ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        // use two points that make the path tangent to the previous path 's end point
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    // Get points in Frenet coordinates, spaced evenly at 30m ahead of the starting reference
                    std::vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    std::vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    std::vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for(int i=0; i<ptsx.size(); ++i) {
                        // Shift car angle to 0 degrees
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * std::cos(0-ref_yaw) - shift_y * std::sin(0-ref_yaw)); // can also use eigen matrix to multiply this
                        ptsy[i] = (shift_x * std::sin(0-ref_yaw) + shift_y * std::cos(0-ref_yaw));
                    }

                    tk::spline spline;
                    // initialize the spline
                    spline.set_points(ptsx, ptsy);

                    // attach all previously unreached points
                    for(int i=0; i<previous_path_x.size(); ++i) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // interpolate spline points so that we travel at the reference velocity
                    // TIME INTERVAL BETWEEN POINTS: 0,2 seconds
                    double target_x = 30.0;
                    double target_y = spline(target_x);
                    double target_dist = std::sqrt(target_x*target_x + target_y*target_y);

                    double x_add_on = 0;

                    // Generate the rest of the points
                    for(int i=1; i<=50-previous_path_x.size(); ++i) {
                        double N = (target_dist/(0.02*ref_speed/2.24));
                        double x_pt = x_add_on + target_x/N;
                        double y_pt = spline(x_pt);

                        x_add_on = x_pt;
                        // transform car to world coordinates
                        double x_ref = x_pt;
                        double y_ref = y_pt;

                        x_pt = (x_ref*std::cos(ref_yaw) - y_ref*sin(ref_yaw) + ref_x);
                        y_pt = (x_ref*std::sin(ref_yaw) + y_ref*cos(ref_yaw) + ref_y);

                        next_x_vals.push_back(x_pt);
                        next_y_vals.push_back(y_pt);
                    }
                    // AARON'S CODE END


                    // Starter code
                    // START_STARTER_CODE
                    /*
                    double dist_inc = 0.45;
                    double next_s, next_d;
                    double next_x, next_y;
                    next_d = car_d;
                    for(int i=0; i<50; ++i) {
                        next_s = car_s + (i+1) * dist_inc;
                        std::vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        next_x_vals.push_back(xy[0]);
                        next_y_vals.push_back(xy[1]);
                        //next_x_vals.push_back(car_x + (dist_inc*i)*std::cos(deg2rad(car_yaw)));
                        //next_y_vals.push_back(car_y + (dist_inc*i)*std::sin(deg2rad(car_yaw)));
                    }
                    */ //END_STARTER_CODE

                    /*
                    // TODO: Predict the movement of each car currently in sensor fusion
                    std::vector<auto> movement(sizeof(sensor_fusion)); // TODO: find sensor fusion type!
                    for(auto car: sensor_fusion) {
                        std::vector<double> car_frenet(2);
                        car_frenet.push_back(car[0]);
                        car_frenet.push_back(car[1]);// TODO: Check
                        movement.push_back(predictor.predict(car_frenet));
                    }
                    // TODO: Generate trajectories for each car
                    for(auto car: movement) {
                        pathPlanner.generatePath(car);
                    }
                    // TODO: Plan car's behaviour and generate global waypoints
                    std::vector<double> behaviour = behaviourPlanner.plan(); // TODO: Add arguments! Typedef struct <T,Q> ?
                    // TODO: Interpolate a path using the global waypoints
                    double car_intention = behaviour[0]; // TODO: Replace with ENUM type
                    std::vector<double> global_waypoints(behaviour.size());
                    for(int i=1; i<behaviour.size(); ++i) {
                        global_waypoints.push_back(behaviour[i]);
                    }
                    std::vector<double> carPath = pathPlanner.generatePath(global_waypoints); // TODO: CHECK SIGNATURE!

                    for(int i=0; i<carPath.size(); i+=2) {
                        // TODO: Convert from frenet to XY if necessary here!
                        next_x_vals.push_back(carPath[i]);
                        next_y_vals.push_back(carPath[i+1]);
                    }
                    */

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
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
