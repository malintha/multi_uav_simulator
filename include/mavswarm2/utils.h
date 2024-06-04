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

#ifndef PROJECT_CFSIMUTILS_H
#define PROJECT_CFSIMUTILS_H

#include <eigen3/Eigen/Dense>
#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include <iterator>
#include <fstream>
#include <limits.h>
#include <unistd.h>
#include "tf/tf.h"

using namespace Eigen;

namespace cfSimUtils {
    /**
     * @param gamma roll
     * @param beta pitch
     * @param alpha yaw
     */

    struct node {
        int id, cluster;
        float x, y, z;
        std::string type;
        node(int id, int cluster, float x, float y, float z, std::string type):id(id), cluster(cluster), x(x), y(y),
                                                                               z(z), type(type) {};
    };


    Matrix3d getR(double gamma, double beta, double alpha) {
        Matrix3d R;
        R <<
          cos(alpha) * cos(beta), cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma),
                cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
                sin(alpha) * cos(beta), sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma),
                sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma),
                -sin(beta), cos(beta) * sin(gamma), cos(beta) * cos(gamma);
        return R;
    }

    double getTargetPWM(double targetThrust) {
        double targetPWM;
        if (targetThrust > 0) {
            targetPWM = 1.732972 + 910.9813*targetThrust - 1863.517*pow(targetThrust,2);
        }
        return targetPWM;
    }

    Vector3d transformtoRPY(tf::Transform transform) {
        tfScalar roll, pitch, yaw;
        Vector3d rpy;
        tf::Matrix3x3(tf::Quaternion(
                transform.getRotation().x(),
                transform.getRotation().y(),
                transform.getRotation().z(),
                transform.getRotation().w()
        )).getRPY(roll, pitch, yaw);
        rpy << roll, pitch, yaw;
        return rpy;
    }

    std::vector<std::vector<int> > retrievePaths() {
        std::ifstream input_file("/home/malintha/geo_controller_ws/src/cf_simulator/trajectory_data/paths.txt");
        std::vector<std::vector<int> > paths_l;
        std::vector<int> path;
        assert(input_file.is_open());
        for(std::string line; getline(input_file, line);) {
            std::istringstream ss(line);
            std::string token;
            while(getline(ss, token, ' ')) {
                if(!token.empty())
                    path.push_back(std::atoi(token.data()));
            }
            paths_l.push_back(path);
        }
        return paths_l;
    }

    std::vector<node> retrieveTopologyConfig() {
        std::cout<<"here1"<<std::endl;
        std::vector<node> coords_list;
        std::ifstream input_file("/home/malintha/geo_controller_ws/src/cf_simulator/trajectory_data/topo.txt");
        assert(input_file.is_open());
        int id = 0;

        for(std::string line; getline(input_file, line);) {
            char str_char[line.size() + 1];
            strcpy(str_char, line.c_str());
            float x, y, z;
            uint cluster;
            char agent_type[4];
            int match_cnt = sscanf(str_char, "%s %f %f %f %u", agent_type, &x, &y, &z, &cluster);
            node n(id, cluster, x, y, z, std::string(agent_type));
            coords_list.push_back(n);
            id++;
        }
        return coords_list;
    }
}

#endif
