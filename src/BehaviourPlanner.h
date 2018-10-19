//
// Created by andrej on 5.8.18.
//

#ifndef PATH_PLANNING_BEHAVIOURPLANNER_H
#define PATH_PLANNING_BEHAVIOURPLANNER_H

#include <vector>
#include "State.h"
#include "json.hpp"

enum Distance { FAR_FRONT=0, NEAR_FRONT, INLINE, NEAR_REAR, FAR_REAR};
enum Lane { LEFT=0, CENTER, RIGHT };

class BehaviourPlanner {
private:
    int lane_occupancy[5][3];
    void occupyLane(double dist_delta_s, Lane lane);
    bool isSameLane(double car1_d, double car2_d);
    bool isMyLaneFree(double lane);
    bool isChangeLeftPossible(double lane, double speed);
    bool isChangeRightPossible(double lane, double speed);
    State overtakeManeuver(double currentLane, double car_in_front_speed, double curr_speed, bool too_close);
public:
    BehaviourPlanner();
    bool isLeftLaneFree();
    bool isRightLaneFree();
    bool isCenterLaneFree();
    bool haveCarInFront(double currentLane);
    State plan(nlohmann::basic_json<std::map, std::vector, std::string, bool, long, unsigned long, double, std::allocator, nlohmann::adl_serializer>& sensor_fusion, double currentLane, int prev_size, double car_s, double currSpeed);
};


#endif //PATH_PLANNING_BEHAVIOURPLANNER_H
