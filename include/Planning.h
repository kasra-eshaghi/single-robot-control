/**
 * \author Kasra Eshaghi
 * \brief Functions and classes for path planning purposes
 */

#ifndef SINGLE_ROBOT_CONTROL_PLANNING_H
#define SINGLE_ROBOT_CONTROL_PLANNING_H

#include <iostream>
#include <cmath>

#include "Extras.h"
#include "Problem_Instance.h"
#include <matplot/matplot.h>

// RRT node specs - FOR PLOTTING
#define RRT_NODE_MARKER_SIZE    10
#define RRT_NODE_MARKER_COLOR   "black"
#define RRT_PATH_COLOR          "black"
#define RRT_PATH_WIDTH          5

void plan_linear_path(Pose& pose_0, Pose& pose_f, Linear_Path& path);// plans linear path between endpoints
void get_desired_pose(Pose& pose_desired, Linear_Path& linear_path, double& s); // gets desired pose of robot at specific distance along path

struct Node {
    int number;
    Pose pose;
    int parent;
    std::vector<int> children;
};

class RRT {
public:
    RRT(Problem_Instance& problem_instance, std::default_random_engine& generator);
    bool plan_path_RRT(bool talk);

    std::vector<Node> planned_path;

private:

    // RRT planning functions
    Node sample_random_node();
    int find_nearest(Node& node);
    bool check_node_collision(Node& node);
    bool check_path_collision(Node& start_node, Node& end_node);

    // plotting functions
    void plot_RRT_nodes(matplot::figure_handle& fig);
    void plot_planned_path(matplot::figure_handle& fig);

    // RRT variables
    std::vector<Node> nodes;
    int highest_node_number;
    Problem_Instance* problem_instance;
    std::default_random_engine* generator;
};

#endif //SINGLE_ROBOT_CONTROL_PLANNING_H
