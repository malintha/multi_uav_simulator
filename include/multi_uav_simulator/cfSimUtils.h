//
// Created by malintha on 2/27/18.
//

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

    Matrix3d getR(double gamma, double beta, double alpha);
    double getTargetPWM(double targetThrust);
    Vector3d transformtoRPY(tf::Transform transform);
    std::vector<std::vector<int> > retrievePaths();
    std::vector<node> retrieveTopologyConfig();

}

#endif
