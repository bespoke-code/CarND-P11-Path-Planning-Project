//
// Created by andrej on 19.10.18.
//

#include "State.h"

State::State(double l, double refSpd) {
    this->lane = l;
    this->changeLane = false;
    this->matchSpeed = false;
    this->refSpeed = refSpd;
}

State::State(double l, bool laneChange, bool matchSpd, double refSpd) {
    this->lane = l;
    this->changeLane = laneChange;
    this->matchSpeed = matchSpd;
    this->refSpeed = refSpd;
}

double State::getLane() {
    return lane;
}

double State::getReferenceVelocity() {
    return refSpeed;
}

bool State::shouldMatchSpeed() {
    return matchSpeed;
}

bool State::shouldDoLaneChange() {
    return changeLane;
}
