//
// Created by malintha on 7/31/18.
//

#ifndef CF_SIMULATOR_DESIREDSTATEPROVIDER_H
#define CF_SIMULATOR_DESIREDSTATEPROVIDER_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>

using namespace Eigen;
class DesiredStateProvider {

public:
    DesiredStateProvider(int id);
    std::vector<Vector3d> getNextState();
    bool lastState();
    std::vector<Vector3d> getPosVec();
private:
    bool isLastState; 
    int id;
    int n_frames;
    int counter;
    std::vector<Eigen::Vector3d> pos_vec, vel_vec, acc_vec;
    Vector3d b1d;
};


#endif
