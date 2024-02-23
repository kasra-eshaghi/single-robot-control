//
// Created by user on 2024-02-22.
//

#include "../include/Sensors.h"

Sensors::Sensors(Problem_Instance &problem_instance, double sensing_noise, std::default_random_engine &generator, int sensing_param) {
    this->problem_instance = &problem_instance;
    this->sensing_noise = sensing_noise;
    this->generator = &generator;
    this->sensing_param = sensing_param;

    sensing_noise_distribution.param(std::normal_distribution<double>::param_type(0, 0.1*sensing_noise/3));

    distance_measurements.resize(problem_instance.map.landmarks.size(), 0);
    bearing_measurements.resize(problem_instance.map.landmarks.size(), 0);
    delta_x_measurements.resize(problem_instance.map.landmarks.size(), 0);
    delta_y_measurements.resize(problem_instance.map.landmarks.size(), 0);
    measurement_mask.resize(problem_instance.map.landmarks.size(), 0);
}

void Sensors::get_measurements(Pose& pose_true) {
    /**
     * \brief Simulates sensor measurements
     */

    // simulate clean measurements, then add noise, and finally convert to cartesian coordinates
    simulate_clean_measurements(pose_true);
    add_noise();
    convert_to_cartesian();

    // mask based on sensing param
    mask_measurements();

}

void Sensors::simulate_clean_measurements(Pose& pose_true) {
    /**
     * \brief Simulates noise free sensor measurements
     */

    for (auto i=0; i<distance_measurements.size(); i++){
        distance_measurements[i] = sqrt(pow(problem_instance->map.landmarks[i].x - pose_true.x, 2) + pow(problem_instance->map.landmarks[i].y - pose_true.y, 2));
        bearing_measurements[i] = atan2(problem_instance->map.landmarks[i].y - pose_true.y, problem_instance->map.landmarks[i].x - pose_true.x);
    }
}

void Sensors::add_noise() {
    /**
     * \brief Adds noise to clean sensor measurements
     */

    for (auto i=0; i<distance_measurements.size(); i++){
        distance_measurements[i] += sensing_noise_distribution(*generator);
        bearing_measurements[i] += sensing_noise_distribution(*generator);

    }
}

void Sensors::convert_to_cartesian() {
    /**
     * \brief Converts sensor measurements to cartesian coordinates
     */

    for (auto i=0; i<distance_measurements.size(); i++){
        delta_x_measurements[i] = distance_measurements[i]*cos(bearing_measurements[i]);
        delta_y_measurements[i] = distance_measurements[i]*sin(bearing_measurements[i]);
    }
}

void Sensors::mask_measurements() {
    /**
     * \brief Masks the measurements based on the sensing param, so that the robot can only see a specific number of its closest landmarks
     */

    std::vector<std::pair<double, int>> distance_measurements_sorted = argsort(distance_measurements);

    for (auto i=0; i<distance_measurements.size(); i++){
        int idx = distance_measurements_sorted[i].second;
        if (i < sensing_param){
            measurement_mask[idx] = true;
        }
        else {
            measurement_mask[idx] = false;
            distance_measurements[idx] = -1;
            bearing_measurements[idx] = -1;
            delta_x_measurements[idx] = -1;
            delta_y_measurements[idx] = -1;
        }
    }

}

std::vector<std::pair< double, int>> Sensors::argsort(std::vector<double>& input) {



    std::vector<std::pair<double, int>> sorted_input;
    for (int i = 0; i < input.size(); i++) {
        sorted_input.push_back(std::make_pair(input[i], i));
    }

    // Sorting pair vector
    sort(sorted_input.begin(), sorted_input.end());

    return sorted_input;

}

bool Sensors::sort_by_first(std::pair<double,int> &a, std::pair<double,int> &b){
    return (a.first < b.first);
}