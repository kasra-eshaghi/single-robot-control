/**
 * \author Kasra Eshaghi
 * \brief Class for describing the robot
 */

#include "../include/Robot.h"

Robot::Robot(Pose pose_init, RRT& rrt, Controller& controller, Localizer& localizer, Sensors& sensors, double motion_noise){
    std::cout << "Initializing robot with its modules.." << std::endl;

    // robot's initial position
    pose_true = pose_init;
    pose_true_history.push_back(pose_init);
    pose_hat = pose_init;
    pose_hat_history.push_back(pose_init);

    // motion noise of the robot
    this->motion_noise = motion_noise;

    // robot's modules
    this->rrt = &rrt;
    this->controller = &controller;
    this->localizer = &localizer;
    this->sensors = &sensors;
}
void Robot::run_control_architecture(Problem_Instance& problem, bool talk) {

    // plan motion from initial to desired destination
    if (talk){
        std::cout << "Planning path ..." << std::endl;
    }
    rrt->plan_path_RRT(talk);

    for (auto j=0; j<(rrt->planned_path.size() - 1); j++){
        Pose starting_pose = rrt->planned_path[j].pose;
        Pose ending_pose = rrt->planned_path[j+1].pose;
        plan_linear_path(starting_pose, ending_pose, linear_path);

        // motion control loop
        double n_iterations = 5;
        double delta_t = 1/n_iterations;
        Pose pose_desired;
        for (auto i=1 ; i <=n_iterations; i++){
            // determine next desired position
            double s = double(i) / n_iterations;
            get_desired_pose(pose_desired, linear_path, s);

            // calculate motion commands through controller
            controller->calculate_motion_command(pose_hat, pose_desired, motion_command);

            // execute motion commands
            execute_motion_commands();

            // get sensor measurements
            sensors->get_measurements(pose_true);

            // localize robot
            localizer->localize_robot(pose_hat_history, pose_hat, motion_command, sensors->delta_x_measurements, sensors->delta_y_measurements, sensors->measurement_mask, problem.map);
        }
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


