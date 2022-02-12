//
// Created by malintha on 10/16/20.
//
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float32MultiArray.h"

using namespace std;
using namespace Eigen;

#ifndef MRF_DYNAMICS_TRAJECTORY_T_H
#define MRF_DYNAMICS_TRAJECTORY_T_H

struct Trajectory_t {
    vector<double> xc;
    vector<double> yc;
    vector<double> zc;
    int D;
    Trajectory_t(const std_msgs::Float32MultiArrayConstPtr& coeffs);
    Trajectory_t();
    Vector3d eval(double d);

};

Trajectory_t::Trajectory_t() = default;

Trajectory_t::Trajectory_t(const std_msgs::Float32MultiArrayConstPtr& coeffs) {
    this->D = coeffs->data.size()/3;
    for(int j=0;j<D;j++) {
        xc.push_back(coeffs->data[j]);
        yc.push_back(coeffs->data[j+ D]);
        zc.push_back(coeffs->data[j+ 2*D]);
    }
}

Vector3d Trajectory_t::eval(double t) {
    Vector3d x = {0, 0, 0};
    for(int i=0;i<D;i++) {
        x[0] += xc[i]*pow(t,i);
        x[1] += yc[i]*pow(t,i);
        x[2] += yc[i]*pow(t,i);
    }
    return x;
}

#endif //MRF_DYNAMICS_TRAJECTORY_T_H
