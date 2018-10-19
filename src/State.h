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
    bool tooClose;   // true if a car is too close (<=9m) in the front
public:
    State(double l, double refSpd);
    State(double l, bool laneChange, bool matchSpd, double refSpd, bool tooClose);
    double getLane();
    double getReferenceVelocity();
    bool shouldMatchSpeed();
    bool shouldDoLaneChange();
    void updateRefVelocity(double v);
    bool isCarTooClose();
};


#endif //PATH_PLANNING_STATE_H
