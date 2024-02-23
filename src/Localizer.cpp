/**
 * \author Kasra Eshaghi
 * \brief Functions and classes for localization purposes
 */


#include "../include/Localizer.h"

Localizer::Localizer() {
    std::cout << "Creating localization module" << std::endl;
}

void Localizer::localize_robot(std::vector<Pose>& pose_estimated_history, Pose& pose_estimated, Motion_Command& motion_command, std::vector<double>& delta_x_measurements, std::vector<double>& delta_y_measurements, std::vector<bool>& measurement_mask, Map& map){


    column_vector solution = {pose_estimated.x + motion_command.delta_x, pose_estimated.y+motion_command.delta_y};
    auto objectiveWrapper = [=](const column_vector& solution) {
        return localization_objective_func(solution, pose_estimated, motion_command, delta_x_measurements, delta_y_measurements, measurement_mask, map);
    };

    dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(),dlib::objective_delta_stop_strategy(1e-7,  1000), objectiveWrapper, solution, -1);
    //std::cout << "Localization solution:\n" << solution << std::endl;

    pose_estimated.x = solution(0);
    pose_estimated.y = solution(1);

    pose_estimated_history.push_back(pose_estimated);

}

double Localizer::localization_objective_func(const column_vector& position, const Pose& pose_estimated_last, const Motion_Command& motion_command, const std::vector<double>& delta_x_measurements, const std::vector<double>& delta_y_measurements, const std::vector<bool>& measurement_mask, const Map& map) {
    /**
     * \brief Calculates the objective function value for a candidate robot position estimate
     */

    double cost = 0;

    // difference between candidate position and position estimated based on motion commands
    double x_position_motion_commands = pose_estimated_last.x + motion_command.delta_x;
    double y_position_motion_commands = pose_estimated_last.y + motion_command.delta_y;
    cost += sqrt(pow(position(0) - x_position_motion_commands, 2) + pow(position(1) - y_position_motion_commands, 2));


    // difference between true position of landmarks and their estimated positions based on sensor measurements
    for (auto i=0; i<measurement_mask.size(); i++){
        if (measurement_mask[i]){
            double x_position_landmark = position(0) + delta_x_measurements[i];
            double y_position_landmark = position(1) + delta_y_measurements[i];
            cost += sqrt(pow(map.landmarks[i].x - x_position_landmark, 2) + pow(map.landmarks[i].y - y_position_landmark, 2));
        }
    }
    return cost;
}