/**
 * \author Kasra Eshaghi
 * \brief Functions and classes for localization purposes
 */

#ifndef SINGLE_ROBOT_CONTROL_LOCALIZER_H
#define SINGLE_ROBOT_CONTROL_LOCALIZER_H

#include <iostream>
#include <vector>

#include "Extras.h"

class Localizer {
public:
    Localizer();

    void localize_robot(std::vector<Pose>& pose_estimated_history, Pose& pose_estimated, Motion_Command& motion_command, double& delta_t);


private:

};


#endif //SINGLE_ROBOT_CONTROL_LOCALIZER_H
