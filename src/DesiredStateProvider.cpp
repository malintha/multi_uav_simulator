//
// Created by malintha on 7/31/18.
//

#include "DesiredStateProvider.h"
#include <iostream>

DesiredStateProvider::DesiredStateProvider(int id) : id(id), counter(0) {
    this->isLastState = false;
    std::string location = "/home/malintha/geo_controller_ws/src/cf_simulator/trajectory_data/";
    std::stringstream ss, ss1, ss2;
    ss << "pos_"<<id-1<<".txt";
    std::string pos_fileName = location + ss.str();
    ss1 << id << "_vel.txt";
    std::string vel_fileName = location + ss1.str();
    ss2 << id << "_acc.txt";
    std::string acc_fileName = location + ss2.str();

    std::ifstream pos_file_stream(pos_fileName.data());
    // std::ifstream vel_file_stream(vel_fileName.data());
    // std::ifstream acc_file_stream(acc_fileName.data());

    std::string pos_line, vel_line, acc_line;
    double x_temp, y_temp, z_temp;
    while (pos_file_stream >> x_temp >> y_temp >> z_temp) {
        Eigen::Vector3d pos_v;
        pos_v << x_temp, y_temp, -z_temp;
        pos_vec.push_back(pos_v);
    }

    // while (vel_file_stream >> x_temp >> y_temp >> z_temp) {
    //     Eigen::Vector3d vel_v;
    //     vel_v << x_temp, y_temp, -z_temp;
    //     vel_vec.push_back(vel_v);
    // }

    // while (acc_file_stream >> x_temp >> y_temp >> z_temp) {
    //     Eigen::Vector3d acc_v;
    //     acc_v << x_temp, y_temp, -z_temp;
    //     acc_vec.push_back(acc_v);
    // }
    this->n_frames = pos_vec.size();
    b1d << 1, 0, 0;
    // std::cout << "size: " << id << " " << pos_vec.size() << std::endl;
}

bool DesiredStateProvider::lastState() {
    return isLastState;
}

std::vector<Vector3d> DesiredStateProvider::getPosVec() {
    return this->pos_vec;
}

std::vector<Vector3d> DesiredStateProvider::getNextState() {
    std::vector<Eigen::Vector3d> pose_vec;
    pose_vec.push_back(pos_vec[counter]);
    // pose_vec.push_back(vel_vec[counter]);
    // pose_vec.push_back(acc_vec[counter]);
    pose_vec.push_back(b1d);
    counter++;
    this->isLastState = counter==n_frames; 
    return pose_vec;
}
