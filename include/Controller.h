/**
 * \author Kasra Eshaghi
 * \brief Robot motion controller class
 */

#ifndef SINGLE_ROBOT_CONTROL_CONTROLLER_H
#define SINGLE_ROBOT_CONTROL_CONTROLLER_H

#include <iostream>
#include <vector>
#include <cmath>

#include "Extras.h"


class Controller {
public:
    Controller(double K_P);

    void calculate_motion_command(Pose& pose_estimated, Pose& pose_desired, Motion_Command& motion_command); // calculates motion command

private:
    // controller gains
    double K_P;

};


#endif //SINGLE_ROBOT_CONTROL_CONTROLLER_H
