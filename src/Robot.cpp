/**
 * \author Kasra Eshaghi
 * \brief Class for describing the robot
 */

#include "../include/Robot.h"

Robot::Robot(Pose pose_init, Path_Planner& path_planner, Controller& controller, Localizer& localizer, double motion_noise){
    std::cout << "Initializing robot with its modules.." << std::endl;

    // robot's initial position
    pose_true = pose_init;
    pose_true_history.push_back(pose_init);
    pose_hat = pose_init;
    pose_hat_history.push_back(pose_init);

    // motion noise of the robot
    this->motion_noise = motion_noise;


    this->path_planner = &path_planner;
    this->controller = &controller;
    this->localizer = &localizer;
}
void Robot::run_control_architecture(Problem_Instance& problem, bool talk) {

    // plan motion from initial to desired destination
    if (talk){
        std::cout << "Planning path ..." << std::endl;
    }
    path_planner->plan_linear_path(problem.pose_init, problem.pose_final, linear_path);

    // motion control loop
    double n_iterations = 100;
    double delta_t = 1/n_iterations;
    for (auto i=0 ; i <n_iterations; i++){
        // determine next desired position
        double s = double(i) / n_iterations;
        path_planner->get_desired_pose(pose_desired, linear_path, s);

        if (talk){
            std::cout << "Next desired position: [" << pose_desired.x << ", " << pose_desired.y << "]" << std::endl;
        }

        // calculate motion commands through controller
        controller->calculate_motion_command(pose_hat, pose_desired, motion_command);

        // execute motion commands
        execute_motion_commands();

        // localize robot
        localizer->localize_robot(pose_hat_history, pose_hat, motion_command, delta_t);
    }

}

void Robot::execute_motion_commands() {
    /**
     * \brief executes the motion commands of the robot with noise and updates the true position of the robot
     */

    motion_noise_distribution.param(std::normal_distribution<double>::param_type(0, fabs(motion_command.delta_x)*motion_noise/3));
    pose_true.x += (motion_command.delta_x + motion_noise_distribution(generator));

    motion_noise_distribution.param(std::normal_distribution<double>::param_type(0, fabs(motion_command.delta_y)*motion_noise/3));
    pose_true.y += (motion_command.delta_y + motion_noise_distribution(generator));

    pose_true_history.push_back(pose_true);

}


