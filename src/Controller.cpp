/**
 * \author Kasra Eshaghi
 * \brief Robot motion controller class
 */

#include "../include/Controller.h"

Controller::Controller(double K_P){
    std::cout << "Creating motion control module" << std::endl;

    this->K_P = K_P;
}


void Controller::calculate_motion_command(Pose& pose_estimated, Pose& pose_desired, Motion_Command& motion_command) {
    /**
     *  \brief Calculates the speed inputs to the robot based on desired input and control gains
     *  \param position_desired desired position of the robot
     */

    double distance_error = sqrt(pow(pose_desired.x - pose_estimated.x, 2) + pow(pose_desired.y - pose_estimated.y, 2));
    double angle = atan2(pose_desired.y - pose_estimated.y, pose_desired.x - pose_estimated.x);

    motion_command.delta_x = K_P * distance_error * cos(angle);
    motion_command.delta_y = K_P * distance_error * sin(angle);
}

