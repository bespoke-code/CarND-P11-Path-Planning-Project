//
// Created by andrej on 5.8.18.
//

#ifndef PATH_PLANNING_BEHAVIOURPLANNER_H
#define PATH_PLANNING_BEHAVIOURPLANNER_H

#include <vector>
#include "State.h"

enum Distance { FAR_FRONT=0, NEAR_FRONT, INLINE, NEAR_REAR, FAR_REAR};
enum Lane { LEFT=0, CENTER, RIGHT };

class BehaviourPlanner {
private:
    int lane_occupancy[5][3];
    void occupyLane(double dist_delta_s, Lane lane);
    bool isSameLane(double car1_d, double car2_d);
    bool isMyLaneFree(double lane);
    State overtakeManeuver(double currentLane, double car_in_front_speed);
public:
    BehaviourPlanner();
    bool isLeftLaneFree();
    bool isRightLaneFree();
    bool isCenterLaneFree();
    bool haveCarInFront(double currentLane);
    State plan(std::vector<std::vector<double>>& sensor_fusion, double currentLane, int prev_size, double car_s);
};


#endif //PATH_PLANNING_BEHAVIOURPLANNER_H
