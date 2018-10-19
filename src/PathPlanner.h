//
// Created by andrej on 30.7.18.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include <vector>

class PathPlanner {
private:

public:
    PathPlanner();
    std::vector<std::vector<double>> generatePath(const std::vector<double>& waypoints);


};


#endif //PATH_PLANNING_PATHPLANNER_H
