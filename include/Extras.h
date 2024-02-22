//
// Created by user on 2024-02-12.
//

#ifndef SINGLE_ROBOT_CONTROL_EXTRAS_H
#define SINGLE_ROBOT_CONTROL_EXTRAS_H

#include <vector>

// Robot related
struct Pose {
    double x;
    double y;
    double theta;
    double time;
};

struct Motion_Command{
    double delta_x;
    double delta_y;
};

// map related
struct Landmark{
    double x;
    double y;
    double radius;
};

struct Map{
    std::vector<Landmark> landmarks;
    double x_max, x_min, y_max, y_min;
};

// planning related
struct Linear_Path{
    Pose pose_0;
    double distance;
    double angle;
};



#endif //SINGLE_ROBOT_CONTROL_EXTRAS_H
