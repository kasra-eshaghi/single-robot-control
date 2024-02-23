/**
 * Classes and functions used to create problem instances
 */

#ifndef SINGLE_ROBOT_CONTROL_PROBLEM_INSTANCE_H
#define SINGLE_ROBOT_CONTROL_PROBLEM_INSTANCE_H

#include <random>

#include <matplot/matplot.h>

#include "Extras.h"

/**
 * \brief A problem is defined by the starting and final position of the robot, and the map of the operating environment
 */

// landmark specs
#define LANDMARK_WIDTH  2
#define LANDMARK_COLOR  "red"

// pose specs
#define POSE_MARKER_SIZE    10
#define STARTING_POSE_MARKER_COLOR   "blue"
#define FINAL_POSE_MARKER_COLOR   "green"

class Problem_Instance {
public:
    Problem_Instance(double map_width, int n_landmarks, double max_landmark_radius, std::default_random_engine& generator);
    void plot_problem_instance(matplot::figure_handle& fig);

    Pose pose_init, pose_final;
    Map map;
    bool feasible = true;

private:
    std::vector<std::pair<double, double>> get_circle_points(double center_x, double center_y, double radius);
    void find_quadrant_limits(double& x_min, double& x_max, double& y_min, double& y_max, double map_width, int quadrant);

};


#endif //SINGLE_ROBOT_CONTROL_PROBLEM_INSTANCE_H
