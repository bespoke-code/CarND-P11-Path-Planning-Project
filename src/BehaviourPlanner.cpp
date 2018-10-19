//
// Created by andrej on 5.8.18.
//

#include "BehaviourPlanner.h"
#include <cmath>
#include <iostream>

BehaviourPlanner::BehaviourPlanner() {
    // initialize with an empty lane set
    for(int lane=0; lane<3; ++lane) { // lane
        for (auto &segment : lane_occupancy) { // lane segment
            segment[lane] = 0; // number of cars in each segment
        }
    }
}

State BehaviourPlanner::plan(std::vector<std::vector<double>>& sensor_fusion, double currentLane, int prev_size, double car_s) {
    double lane = currentLane;     // the car's desired lane number
    bool changeLane = false; // false = keep lane
    bool matchSpeed = true; // true if speed is to be adjusted according to another car
    double refSpeed; // reference speed at which the car should drive

    // Reset the lane occupancy matrix
    for(int l=0; l<3; ++l) { // lane
        for (auto &seg : lane_occupancy) { // lane segment
            seg[l] = 0; // number of cars in each segment
        }
    }

    // Re-populate the lane occupancy matrix using the sensor fusion data
    // Find the closest car in front and in our lane, if there is one
    double samelaneCar_s = 0;
    double samelaneCar_v = -1;
    double min_dist = 0;
    std::cout << "Cars on road: " <<  sensor_fusion.size() << std::endl;
    for (auto &i : sensor_fusion) {
        double vx = i[3]; // other car's Vx value
        double vy = i[4]; // other car's Vy value
        double check_speed = std::sqrt(vx*vx + vy*vy);
        double check_car_s = i[5]; // other car's S value
        float check_car_d = i[6]; // other car's D value

        double dist_delta_s = car_s - check_car_s; // Negative values mean the other car is in the front
        dist_delta_s += prev_size*0.02*check_speed; // Predicted distance between cars, 0,02s in the future

        // Populate a lane segment depending on lane and distance from our car
        // left lane
        if ((check_car_d >= 0) && (check_car_d <= 4)) {
            occupyLane(dist_delta_s, LEFT);
        }
        // center lane
        if ((check_car_d > 4) && (check_car_d <= 8)) {
            occupyLane(dist_delta_s, CENTER);
        }
        // right lane
        if ((check_car_d > 8) && (check_car_d <= 12)) {
            occupyLane(dist_delta_s, RIGHT);
        }

        // if another car is in front and in the same lane, take note
        if((dist_delta_s < 0) && isSameLane(check_car_d, 4*lane+2)) {
            if(dist_delta_s < min_dist) {
                min_dist = dist_delta_s;
                // Save that car's velocity and distance from our vehicle
                std::cout << "Vehicle " << dist_delta_s << " meters in front!" << std::endl;
                samelaneCar_v = check_speed;
                samelaneCar_s = check_car_s;
            }
        }
    }

    for(int i=0; i<5; ++i) {
        for(int j=0; j<3; ++j) {
            std::cout << lane_occupancy[i][j] << " ";
        }
        std::cout << std::endl;
    }
    //if(samelaneCar_s == 0) std::cout << "Lane in front is free!" << std::endl;
    if(samelaneCar_v < 0) samelaneCar_v = 49.95; // Ensuring that if there's no car in front, the reference speed is ok

    // TODO: New planner. rework this.
    if(isMyLaneFree(lane)) {
        // if lane is free, keep the lane and cruise
        std::cout << "My lane is free. Cruising!" << std::endl;
        return {currentLane, false, false, 49.95};
    } else {
        if(haveCarInFront(lane)) { // try and overtake, if impossible - match speed
            std::cout << "Trying to overtake... ";
            return overtakeManeuver(lane, samelaneCar_v);
        }
        else // keep lane and cruise
        {
            std::cout << "No car in front! Cruising!" << std::endl;
            return {currentLane, false, false, 49.95};
        }
    }


    // TODO: Old planner. Modify.
    std::cout << "I'm in lane " << lane << " and car in front status is: " << haveCarInFront(lane) << std::endl;
    if(haveCarInFront(lane)) { // If there is a car inline with us and in our lane
        // Check if it is possible to overtake/change lanes
        if(lane == 1.0) { // If we are currently in the CENTER lane
            std::cout << "Minimal distance to car in front: " << min_dist << std::endl;
            if(min_dist < 0) { // Check the shortest distance to a car in front in the same lane, in meters
                // If there's a car in front indeed
                if(isLeftLaneFree()) { // Check overtake possibility, left
                    return {0.0, true, false, 49.65};
                } else {
                    if(isRightLaneFree()) { // Check overtake possibility, right
                        return {2.0, true, false, 49.65};
                    } else { // No overtake opportunity, keep lane and cruise
                        // Lower the reference velocity so we don't crash into the car in front of us
                        if(min_dist < 15.0) // if the car in front is less than 15m away
                            samelaneCar_v -= 10.0; // speed lowered additionally to create distance buffer // TODO: Tune
                        return {currentLane, false, true, samelaneCar_v};
                    }
                }
            } else { // If the min_dist is not lower than zero, then there's NO CAR in front of us in our lane
                // keep lane, accelerate
                return {currentLane, false, false, 49.65};
            }
        } else { // NOT in the CENTER lane
            if(isCenterLaneFree()) { // Check if a lane change to CENTER lane is possible
                // lane = 1.0;
                // changeLane = true;
                // matchSpeed = false;
                // refSpeed = 49.65;
                return {1.0, true, false, 49.65};
            } else { // Stay in lane and match speed
                // lane = currentLane;
                // changeLane = false;
                // matchSpeed = true;
                // refSpeed = samelaneCar_v;
                return {currentLane, false, true, samelaneCar_v};
            }
        }


    } else { // Lane in front is free of traffic
        if(currentLane != 1.0 && isCenterLaneFree()) {// if not in center lane, and the center is free
// Change lane to center lane for more future maneuverability
            // lane = 1.0;
            // changeLane = true;
            // matchSpeed = false;
            // refSpeed = 49.65;
            return {1.0, true, false, 49.65};
        } else { // we are in the center lane or the center lane has vehicles inside
            // lane = currentLane;
            // changeLane = false;
            // matchSpeed = false;
            // refSpeed = 49.65;
            return {currentLane, false, false, 49.65};
        }
    }

    return {lane, changeLane, matchSpeed, refSpeed};
}

bool BehaviourPlanner::isLeftLaneFree() {
    double sum = 0;
    for (auto &segment : lane_occupancy) {
        sum += segment[LEFT];
    }
    return sum > 0;
}

bool BehaviourPlanner::isRightLaneFree() {
    double sum = 0;
    for (auto &segment : lane_occupancy) {
        sum += segment[RIGHT];
    }
    return sum > 0;
}

bool BehaviourPlanner::isCenterLaneFree() {
    double sum = 0;
    for (auto &segment : lane_occupancy) {
        sum += segment[CENTER];
    }
    return sum > 0;
}

void BehaviourPlanner::occupyLane(double dist_delta_s, Lane lane) {
    if(dist_delta_s < -160.0) lane_occupancy[FAR_FRONT][lane] += 1; // more than 30m away in the front
    if(dist_delta_s >= -160.0 && dist_delta_s < -100.0) lane_occupancy[NEAR_FRONT][lane] += 1;
    if(dist_delta_s >= -100.0 && dist_delta_s <= 100.0) lane_occupancy[INLINE][lane] += 1; // +- 10m in front or back of the car
    if(dist_delta_s > 100.0 && dist_delta_s <= 160.0) lane_occupancy[NEAR_REAR][lane] += 1;
    if(dist_delta_s > 160.0) lane_occupancy[FAR_REAR][lane] += 1; // more than 30m away in the back
}

bool BehaviourPlanner::haveCarInFront(double lane) {
    Lane currentLane = CENTER;
    if(lane == 0.0)
        currentLane = LEFT;
    if(lane == 1.0)
        currentLane = CENTER;
    if(lane == 2.0)
        currentLane = RIGHT;
    std::cout << "Front car status in lane " << lane << "is " << lane_occupancy[INLINE][currentLane] \
                                                                + lane_occupancy[NEAR_FRONT][currentLane] \
                                                                + lane_occupancy[FAR_FRONT][currentLane] << std::endl;

    return (lane_occupancy[INLINE][currentLane]
            + lane_occupancy[NEAR_FRONT][currentLane]
            + lane_occupancy[FAR_FRONT][currentLane]) > 0;
}

bool BehaviourPlanner::isSameLane(double car1_d, double car2_d) {
    //std::cout << "Car 1D: " << car1_d << ", Car 2D: " << car2_d << std::endl;
    if ((car1_d >= 0) && (car1_d <= 4) &&
        (car2_d >= 0) && (car2_d <= 4)) return true; // left lane check
    if ((car1_d > 4) && (car1_d <= 8) &&
        (car2_d > 4) && (car2_d <= 8)) return true; // center lane check

    return (car1_d > 8) && (car1_d <= 12) &&
           (car2_d > 8) && (car2_d <= 12); // right lane check

}

bool BehaviourPlanner::isMyLaneFree(double lane) {
    if (lane == 0.0) {
        return isLeftLaneFree();
    } else {
        if (lane == 1.0)
            return isCenterLaneFree();
        else
            return isRightLaneFree();
    }
}

State BehaviourPlanner::overtakeManeuver(double currentLane, double car_in_front_speed) {
    std::cout << "TEST >>> Current lane " << currentLane << " (" << (int)currentLane << ")" << std::endl;
    switch ((int)currentLane) {
        case 0: // If the car is in the left lane
            if (isCenterLaneFree()) { // and the center lane is free
                std::cout << "Overtake left possible! Performing maneuver..." << std::endl;
                return {1.0, true, false, 49.95}; // overtake
            }
            else {
                std::cout << "Not possible! Matching speed..." << std::endl;
                return {currentLane, false, true, car_in_front_speed};  // else match speed
            }
            break;
        case 1:  // If the car is in the center lane
            if (isLeftLaneFree()) {  // and the left lane is free
                std::cout << "Overtake left possible! Performing maneuver..." << std::endl;
                return {0.0, true, false, 49.95};  // overtake left
            } else {
                if (isRightLaneFree()) {  // or if the right lane is free
                    std::cout << "Overtake right possible! Performing maneuver..." << std::endl;
                    return {2.0, true, false, 49.95};  // overtake right
                } else {
                    std::cout << "Not possible! Matching speed..." << std::endl;
                    return {currentLane, false, true, car_in_front_speed}; // else match speed
                }
            }
            break;
        case 2:  // If the car is in the right lane
            if (isCenterLaneFree()) {  // and the center lane is free
                std::cout << "Overtake left possible! Performing maneuver..." << std::endl;
                return {1.0, true, false, 49.95};  // overtake left
            } else {
                std::cout << "Not possible! Matching speed..." << std::endl;
                return {currentLane, false, true, car_in_front_speed}; // else match speed
            }
            break;
        default: // keep lane
            std::cout << "Not possible! Matching speed..." << std::endl;
            return {currentLane, false, true, car_in_front_speed};
            break;
    }
}
