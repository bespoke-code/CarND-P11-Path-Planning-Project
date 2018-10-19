//
// Created by andrej on 30.7.18.
//

#include "PathPlanner.h"

PathPlanner::PathPlanner() {

}

std::vector<std::vector<double>> PathPlanner::generatePath(const std::vector<double> &waypoints) {
    std::vector<double> ptsx;
    std::vector<double> ptsy;
    // TODO: Refactor code to plan path here (in the future)
    /*
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
    std::vector<double> next_wp0 = getXY(car_s+30, 2+4*newState.getLane(), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp1 = getXY(car_s+60, 2+4*newState.getLane(), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp2 = getXY(car_s+90, 2+4*newState.getLane(), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

    if(ref_speed > newState.getReferenceVelocity() && (ref_speed-0.224 > 0)) {
        ref_speed -= 0.224; // decelerate
    }
    if (ref_speed < newState.getReferenceVelocity() && (ref_speed+0.224 < 49.65)) {
        ref_speed += 0.224; // accelerate
    }

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
    //*/
}
