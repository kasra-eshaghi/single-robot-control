/**
 * \author Kasra Eshaghi
 * \brief Class for describing the robot
 */

#ifndef SINGLE_ROBOT_CONTROL_ROBOT_H
#define SINGLE_ROBOT_CONTROL_ROBOT_H

#include <vector>
#include <random>
#include <cmath>

#include "Extras.h"
#include "Planning.h"
#include "Controller.h"
#include "Localizer.h"
#include "Problem_Instance.h"
#include "Sensors.h"



class Robot {
public:
    Robot(Pose pose_init, RRT& rrt, Controller& controller, Localizer& localizer, Sensors& senors, double motion_noise);

    void run_control_architecture(Problem_Instance& problem, bool talk);

    void execute_motion_commands(); // function to execute the robot's motion commands with noise


    // robot poses
    Pose pose_true, pose_hat; // true and estimated pose
    std::vector<Pose> pose_true_history, pose_hat_history; // history of true and estimated poses
    Pose pose_desired; // next desired pose

    // robot paths
    Linear_Path linear_path;

    // robot motion commands
    Motion_Command motion_command;

private:
    // sensing and motion noise
    std::default_random_engine generator;
    std::normal_distribution<double> motion_noise_distribution;
    double motion_noise;

    // modules
    RRT* rrt;
    Controller* controller;
    Localizer* localizer;
    Sensors* sensors;

};


#endif //SINGLE_ROBOT_CONTROL_ROBOT_H
