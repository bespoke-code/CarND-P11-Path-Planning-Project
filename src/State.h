//
// Created by andrej on 19.10.18.
//

#ifndef PATH_PLANNING_STATE_H
#define PATH_PLANNING_STATE_H


class State {
private:
    double lane;     // the car's desired lane number
    bool changeLane; // false = keep lane
    bool matchSpeed; // true if speed is to be adjusted according to another car
    double refSpeed; // reference speed at which the car should drive
public:
    State(double l, double refSpd);
    State(double l, bool laneChange, bool matchSpd, double refSpd);
    double getLane();
    double getReferenceVelocity();
    bool shouldMatchSpeed();
    bool shouldDoLaneChange();
};


#endif //PATH_PLANNING_STATE_H
