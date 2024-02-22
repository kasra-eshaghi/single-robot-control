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


using namespace matplot;

int main() {

    std::default_random_engine generator;
    auto fig = matplot::figure();
    matplot::hold(matplot::on);
    // create problem instance
    Problem_Instance problem(0.5, 5, 0.1, generator);

    problem.plot_problem_instance(fig);

    fig->show();
    matplot::cla(fig->current_axes());

    // create RRT instance
    RRT rrt(problem, generator);

    // plan using RRt
    rrt.plan_path_RRT(true);

//    bool success = rrt.sample_random_node();
//    if (success){
//        plot_RRT_nodes(problem, rrt.nodes, fig);
//    }

    //fig->show();

/*
    // create problem instance
    Pose pose_0{0, 0};
    Pose pose_f{10, 10};
    Pose pose_init{0, 0};
    Problem_Instance problem{pose_0, pose_f, pose_init};

    // create path planning module
    Path_Planner path_planner;

    // create motion controller module
    Controller controller(1.0);

    // create localization module
    Localizer localizer;

    // create robot with modules
    double motion_noise = 0.5;
    Robot robot(problem.pose_init, path_planner, controller, localizer, motion_noise);

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

    plot(pose_x_hat, pose_y_hat);
    hold(on);
    plot(pose_x_true, pose_y_true);

    show();

*/

    return 0;
}
