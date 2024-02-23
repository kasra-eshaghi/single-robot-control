//
// Created by user on 2024-02-16.
//

#include "../include/Problem_Instance.h"

Problem_Instance::Problem_Instance(double map_width, int n_landmarks, double max_landmark_radius, std::default_random_engine& generator) {
    /**
     * \brief This function generates a random problem instance
     */

    // randomly place landmarks in the environment
    std::uniform_real_distribution<double> landmark_radius_distribution(0.5*max_landmark_radius, max_landmark_radius);
    std::uniform_real_distribution<double> landmark_position_distribution(-map_width/2, map_width/2);
    for (auto i=0; i<n_landmarks; i++){
        double x = landmark_position_distribution(generator);
        double y = landmark_position_distribution(generator);
        double radius = landmark_radius_distribution(generator);
        map.landmarks.push_back(Landmark{x, y, radius});
    }

    map.x_min = -map_width/2;
    map.x_max = map_width/2;
    map.y_min = -map_width/2;
    map.y_max = map_width/2;

    // select initial robot position randomly
    std::uniform_int_distribution<int> quadrant_distribution(1, 4);
    std::uniform_real_distribution<double> robot_position_distribution;
    double x_init, y_init, x_final, y_final;
    bool collides;
    for (auto j=0; j<200; j++){
        // generate a random initial robot position
        int quadrant_start = quadrant_distribution(generator);
        double x_min, x_max, y_min, y_max;
        find_quadrant_limits(x_min, x_max, y_min, y_max, map_width, quadrant_start);

        robot_position_distribution.param(std::uniform_real_distribution<double>::param_type(x_min, x_max));
        x_init = robot_position_distribution(generator);
        robot_position_distribution.param(std::uniform_real_distribution<double>::param_type(y_min, y_max));
        y_init = robot_position_distribution(generator);

        // generate a random final robot position
        int quadrant_end;
        switch (quadrant_start){
            case 1:
                quadrant_end = 3;
            case 2:
                quadrant_end = 4;
            case 3:
                quadrant_end = 1;
            case 4:
                quadrant_end = 2;
        }
        find_quadrant_limits(x_min, x_max, y_min, y_max, map_width, quadrant_end);
        robot_position_distribution.param(std::uniform_real_distribution<double>::param_type(x_min, x_max));
        x_final = robot_position_distribution(generator);
        robot_position_distribution.param(std::uniform_real_distribution<double>::param_type(y_min, y_max));
        y_final = robot_position_distribution(generator);

        // make sure robot positions don't collide with a landmark
        collides = false;
        for (auto i=0; i<map.landmarks.size(); i++){
            double distance_initial = sqrt(pow(map.landmarks[i].x - x_init, 2) + pow(map.landmarks[i].y - y_init, 2));
            double distance_final = sqrt(pow(map.landmarks[i].x - x_final, 2) + pow(map.landmarks[i].y - y_final, 2));
            if ((distance_initial < (map.landmarks[i].radius + 0.1)) || (distance_final < (map.landmarks[i].radius + 0.1))){
                collides = true;
                break;
            }
        }
        if (!collides){
            break;
        }
    }

    if (collides){
        feasible = false;
    }

    // set positions
    pose_init.x = x_init;
    pose_init.y = y_init;
    pose_final.x = x_final;
    pose_final.y = y_final;

}

void Problem_Instance::find_quadrant_limits(double& x_min, double& x_max, double& y_min, double& y_max, double map_width, int quadrant) {
    /**
     * \brief sets the limits of the quadrant
     */

    switch (quadrant){
        case 1:
            x_min = -map_width/2;
            x_max = 0;
            y_min = -map_width/2;
            y_max = 0;
            break;
        case 2:
            x_min = -map_width/2;
            x_max = 0;
            y_min = 0;
            y_max = map_width/2;
            break;
        case 3:
            x_min = 0;
            x_max = map_width/2;
            y_min = 0;
            y_max = map_width/2;
            break;
        case 4:
            x_min = 0;
            x_max = map_width/2;
            y_min = -map_width/2;
            y_max = 0;
            break;
    }
}

void Problem_Instance::plot_problem_instance(matplot::figure_handle &fig) {
    /**
     * Plots the problem instance
     */

    auto ax = fig->current_axes();

    // plot landmark positions
    for (auto& landmark : map.landmarks){
        auto circle_points = get_circle_points(landmark.x, landmark.y, landmark.radius);

        std::vector<double> x(circle_points.size(), 0), y(circle_points.size(), 0);
        std::transform(begin(circle_points), end(circle_points), begin(x), [](auto pair){return pair.first;});
        std::transform(begin(circle_points), end(circle_points), begin(y), [](auto pair){return pair.second;});

        auto landmark_ax = ax->plot(x, y);
        landmark_ax->line_width(LANDMARK_WIDTH).color(LANDMARK_COLOR);
    }


    // plot initial and final robot positions
    std::vector<double> x{pose_init.x};
    std::vector<double> y{pose_init.y};
    auto starting_point_ax = ax->scatter(x, y);
    starting_point_ax->marker("o").marker_size(POSE_MARKER_SIZE).marker_color(STARTING_POSE_MARKER_COLOR).display_name("Start");

    x[0] = pose_final.x;
    y[0] = pose_final.y;
    auto ending_point_ax = ax->scatter(x, y);
    ending_point_ax->marker("o").marker_size(POSE_MARKER_SIZE).marker_color(FINAL_POSE_MARKER_COLOR).display_name("End");


}

std::vector<std::pair<double, double>> Problem_Instance::get_circle_points(double center_x, double center_y, double radius){
    /**
     * \brief Returns a vector of points representing a circle (for plotting purposes)
     */
    std::vector<std::pair<double, double>> circle_points;
    std::pair<double, double> point;

    for (auto i=0; i<=360; i=i+5){
        point.first = center_x + radius*cos(i*M_PI/180);
        point.second = center_y + radius*sin(i*M_PI/180);
        circle_points.push_back(point);
    }

    return circle_points;
}
