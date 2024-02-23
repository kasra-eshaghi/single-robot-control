#include <iostream>
#include <cmath>

#include <fstream>
#include <vector>
#include <array>


#include <matplot/matplot.h>

#include "include/Robot.h"
#include "include/Planning.h"
#include "include/Controller.h"
#include "include/Localizer.h"
#include "include/Problem_Instance.h"


int main() {


    // generate problem instace
    std::default_random_engine generator;
    auto fig = matplot::figure();
    matplot::hold(matplot::on);
    // create problem instance
    Problem_Instance problem(0.5, 5, 0.1, generator);


    // generate robot and its modules
    RRT rrt(problem, generator);
    Controller controller(1.0);
    Localizer localizer;
    Sensors sensors(problem, 0.1, generator, 3);
    Robot robot(problem.pose_init, rrt, controller, localizer, sensors, 0.5);

    // run robot control architecture
    robot.run_control_architecture(problem, false);

    std::cout << "done" ;

    std::vector<double> pose_x_true, pose_y_true, pose_x_hat, pose_y_hat;
    for (auto i=0; i< robot.pose_hat_history.size(); i++){
        pose_x_true.push_back(robot.pose_true_history[i].x);
        pose_y_true.push_back(robot.pose_true_history[i].y);
        pose_x_hat.push_back(robot.pose_hat_history[i].x);
        pose_y_hat.push_back(robot.pose_hat_history[i].y);
    }

    problem.plot_problem_instance(fig);
    matplot::plot(pose_x_hat, pose_y_hat);
    matplot::plot(pose_x_true, pose_y_true);

    matplot::show();

    return 0;
}
