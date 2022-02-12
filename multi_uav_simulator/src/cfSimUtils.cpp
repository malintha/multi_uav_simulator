#include "cfSimUtils.h"

namespace cfSimUtils {

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