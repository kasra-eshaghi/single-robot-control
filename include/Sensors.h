//
// Created by user on 2024-02-22.
//

#ifndef SINGLE_ROBOT_CONTROL_SENSORS_H
#define SINGLE_ROBOT_CONTROL_SENSORS_H

#include "Extras.h"
#include "Problem_Instance.h"
class Sensors {
public:

    Sensors(Problem_Instance& problem_instance, double sensing_noise, std::default_random_engine& generator, int sensing_param);
    void get_measurements(Pose& pose_true);

    std::vector<double> distance_measurements;
    std::vector<double> bearing_measurements;
    std::vector<double> delta_x_measurements;
    std::vector<double> delta_y_measurements;
    std::vector<bool> measurement_mask;

private:
    double sensing_noise;
    int sensing_param; // number of closest neighbors to see

    std::default_random_engine* generator;
    Problem_Instance* problem_instance;
    std::normal_distribution<double> sensing_noise_distribution;

    void simulate_clean_measurements(Pose& pose_true);
    void add_noise();
    void convert_to_cartesian();
    void mask_measurements();
    std::vector<std::pair< double, int>> argsort(std::vector<double>& input);
    bool sort_by_first(std::pair<double,int> &a, std::pair<double,int> &b);

};


#endif //SINGLE_ROBOT_CONTROL_SENSORS_H
