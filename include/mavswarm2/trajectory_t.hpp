/*

Copyright (c) 2024 Malintha Fernando (malintha@onmail.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;
using namespace Eigen;

#ifndef MRF_DYNAMICS_TRAJECTORY_T_H
#define MRF_DYNAMICS_TRAJECTORY_T_H

struct Trajectory_t {
    vector<double> xc;
    vector<double> yc;
    vector<double> zc;
    int D;
    Trajectory_t(const std_msgs::msg::Float32MultiArray& coeffs);
    Trajectory_t();
    Vector3d eval(double d);

};

Trajectory_t::Trajectory_t() = default;

Trajectory_t::Trajectory_t(const std_msgs::msg::Float32MultiArray& coeffs) {
    this->D = coeffs.data.size()/3;
    for(int j=0;j<D;j++) {
        xc.push_back(coeffs.data[j]);
        yc.push_back(coeffs.data[j+ D]);
        zc.push_back(coeffs.data[j+ 2*D]);
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
