/**
 * \author Kasra Eshaghi
 * \brief Functions and classes for localization purposes
 */


#include "../include/Localizer.h"

Localizer::Localizer() {
    std::cout << "Creating localization module" << std::endl;
}

void Localizer::localize_robot(std::vector<Pose>& pose_estimated_history, Pose& pose_estimated, Motion_Command& motion_command, double& delta_t){

    pose_estimated.x += motion_command.delta_x;
    pose_estimated.y += motion_command.delta_y;

    pose_estimated_history.push_back(pose_estimated);

}