/**
 * \author Kasra Eshaghi
 * \brief Functions and classes for localization purposes
 */

#ifndef SINGLE_ROBOT_CONTROL_LOCALIZER_H
#define SINGLE_ROBOT_CONTROL_LOCALIZER_H

#include <iostream>
#include <vector>

#include <dlib/optimization.h>
#include <dlib/global_optimization.h>

#include "Extras.h"

typedef dlib::matrix<double,0,1> column_vector;


class Localizer {
public:
    Localizer();

    void localize_robot(std::vector<Pose>& pose_estimated_history, Pose& pose_estimated, Motion_Command& motion_command, std::vector<double>& delta_x_measurements, std::vector<double>& delta_y_measurements, std::vector<bool>& measurement_mask, Map& map);

    static double localization_objective_func(const column_vector& position, const Pose& pose_estimated_last, const Motion_Command& motion_command, const std::vector<double>& delta_x_measurements, const std::vector<double>& delta_y_measurements, const std::vector<bool>& measurement_mask, const Map& map);

private:


};


#endif //SINGLE_ROBOT_CONTROL_LOCALIZER_H
