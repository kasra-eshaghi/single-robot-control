/**
 * \author Kasra Eshaghi
 * \brief Functions and classes for path planning purposes
 */

#include "../include/Planning.h"


void plan_linear_path(Pose& pose_0, Pose& pose_f, Linear_Path& path) {

    path.distance = sqrt(pow(pose_f.x - pose_0.x, 2) + pow(pose_f.y - pose_0.y, 2));
    path.angle = atan2(pose_f.y - pose_0.y, pose_f.x - pose_0.x);
    path.pose_0 = pose_0;
}

void get_desired_pose(Pose& pose_desired, Linear_Path& linear_path, double& s) {
    pose_desired.x = linear_path.pose_0.x + s * linear_path.distance * cos(linear_path.angle);
    pose_desired.y = linear_path.pose_0.y + s * linear_path.distance * sin(linear_path.angle);
}


RRT::RRT(Problem_Instance& problem_instance, std::default_random_engine& generator) {
    this->problem_instance = &problem_instance;
    this->generator = &generator;

    // add start and end locations as node 1 and node 2
    nodes.push_back(Node{0, problem_instance.pose_init});
    nodes.push_back(Node{1, problem_instance.pose_final});

    highest_node_number = 1;
}

bool RRT::plan_path_RRT(bool talk) {
    /**
     * \brief Plan the path for the problem instance at hand
     */

    bool path_found = false;

    double max_iterations = 50;
    double iteration = 0;
    while ((!path_found) && (iteration < max_iterations)){
        if (talk){
            std::cout << "Iteration number: " << iteration << std::endl;
        }

        iteration += 1;

        // ranomly sample a node
        Node random_node = sample_random_node();

        // check if node is feasible
        bool node_feasible = check_node_collision(random_node);

        if (node_feasible){
            // find closest node
            int closest_node = find_nearest(random_node);

            // check if path to node is collision free
            bool path_feasible = check_path_collision(random_node, nodes[closest_node]);

            if (path_feasible){
                // add random node to list and update its parent
                highest_node_number += 1;
                random_node.number = highest_node_number;
                random_node.parent = closest_node;
                nodes.push_back(random_node);

                // if random node is close enough to final node, try connecting to it.
                double max_distance_to_goal = 0.1;
                double distance_to_goal = sqrt(pow(random_node.pose.x - problem_instance->pose_final.x, 2) + pow(random_node.pose.y - problem_instance->pose_final.y, 2));

                if (distance_to_goal < max_distance_to_goal){
                    if (check_path_collision(random_node, nodes[1])){
                        nodes[1].parent = random_node.number;
                        path_found = true;
                    }
                }

                // plot progress
//                if (talk){
//                    auto fig = matplot::figure();
//                    matplot::hold(matplot::on);
//                    plot_RRT_nodes(fig);
//                }
            }
        }
    }

    // generate returned path as a vector of nodes starting from initial node and ending at final node
//    if (talk){
//        auto fig = matplot::figure();
//        matplot::hold(matplot::on);
//        plot_RRT_nodes(fig);
//        fig->show();
//    }

    if (path_found){
        std::vector<int> planned_path_node_ids{1};
        while (nodes[planned_path_node_ids.back()].parent != 0){
            planned_path_node_ids.push_back(nodes[planned_path_node_ids.back()].parent);
        }
        planned_path_node_ids.push_back(0);

        for (auto i=0; i<planned_path_node_ids.size(); i++){
            int id = planned_path_node_ids[planned_path_node_ids.size() - 1 - i];
            planned_path.push_back(nodes[id]);
        }

//        for (auto& id:planned_path_node_ids){
//            planned_path.push_back(nodes[id]);
//        }

        if (talk){
            auto fig = matplot::figure();
            matplot::hold(matplot::on);
            plot_planned_path(fig);
            fig->show();
        }
    }
    return path_found;

}

Node RRT::sample_random_node(){
    /**
     * \brief creates a random feasible node
     * \return random_node
     */

    std::uniform_real_distribution<double> node_x_position(problem_instance->map.x_min, problem_instance->map.x_max);
    std::uniform_real_distribution<double> node_y_position(problem_instance->map.y_min, problem_instance->map.y_max);

    // generate random node
    double x_random = node_x_position(*generator) ;
    double y_random = node_y_position(*generator);
    Pose random_pose{x_random, y_random};
    Node random_node;
    random_node.pose = random_pose;

    return random_node;

}

bool RRT::check_node_collision(Node& node) {
    /**
     * \brief checks if node is in a collision free configuration
     * \return true/false if node is collision free
     */
    bool collision_free = true;
    for (auto& landmark:problem_instance->map.landmarks){
        double distance = sqrt(pow(node.pose.x - landmark.x, 2) + pow(node.pose.y - landmark.y, 2));
        if (distance < landmark.radius){
            collision_free = false;
            break;
        }
    }

    return collision_free;
};

bool RRT::check_path_collision(Node& start_node, Node& end_node){
    /**
     * \brief Checks if path from start_node to end_node is collision free and within bounds of map
     */

    double distance = sqrt(pow(start_node.pose.x - end_node.pose.x, 2) + pow(start_node.pose.y - end_node.pose.y, 2));
    double angle = atan2(end_node.pose.y - start_node.pose.y, end_node.pose.x - start_node.pose.x);

    double increment_size = 0.01;
    int max_increment = int(distance / increment_size) + 1;
    increment_size = distance/double(max_increment);

    bool collision_free = true;
    for (auto i=1; i<max_increment; i++){
        double x_position = start_node.pose.x + i*increment_size*cos(angle);
        double y_position = start_node.pose.y + i*increment_size*sin(angle);
        Node node_temp;
        node_temp.pose = Pose{x_position, y_position};

        collision_free = check_node_collision(node_temp);
        if (!collision_free){
            return collision_free;
        }
    }

    return collision_free;
};


int RRT::find_nearest(Node& node){
    /**
     * \brief Finds nearest node to the node at hand
     * \param node - node to find nearest neighbor to
     */

    double min_distance = std::numeric_limits<double>::infinity();
    int closest_node;
    for (Node& node_to_check:nodes){
        // calcualte distance
        double distance = sqrt(pow(node_to_check.pose.x - node.pose.x, 2) + pow(node_to_check.pose.y - node.pose.y, 2));
        if ((distance < min_distance) && (node_to_check.number != 1)){ // ignore final configuration node
            min_distance = distance;
            closest_node = node_to_check.number;
        }
    }
    return closest_node;
};


void RRT::plot_RRT_nodes(matplot::figure_handle& fig) {
    /**
     * \brief This function plots the RRT instance
     */

    // plot environment
    problem_instance->plot_problem_instance(fig);

    // plot nodes
    for (Node& node:nodes){
        std::vector<double> x{node.pose.x};
        std::vector<double> y{node.pose.y};
        auto node_ax = fig->current_axes()->scatter(x, y);
        node_ax->marker("o").marker_size(RRT_NODE_MARKER_SIZE).marker_color(RRT_NODE_MARKER_COLOR);
    }

}

void RRT::plot_planned_path(matplot::figure_handle &fig) {

    plot_RRT_nodes(fig);

    for (auto i=0; i<(planned_path.size() - 1); i++){
        Node start_node = planned_path[i];
        Node end_node = planned_path[i+1];
        std::vector<double> x{start_node.pose.x, end_node.pose.x};
        std::vector<double> y{start_node.pose.y, end_node.pose.y};

        auto line_ax = fig->current_axes()->plot(x, y);
        line_ax->color(RRT_PATH_COLOR).line_width(RRT_PATH_WIDTH);
    }

}
