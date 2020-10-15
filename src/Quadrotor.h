#ifndef PROJECT_QUADROTOR_H
#define PROJECT_QUADROTOR_H

#include <geo_controller/controllerImpl.h>
#include <tf/transform_listener.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include "DynamicsProvider.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "simulator_utils/Waypoint.h"

#ifndef state
#define state
enum State {
    Idle,
    TakingOff,
    Landing,
    Hover,
    Autonomous
};
#endif

using namespace Eigen;
using namespace std;

class Quadrotor {
public:
    Quadrotor(int robot_id, double frequency, ros::NodeHandle &n);

    void move(const desired_state_t &d_state);

    void setState(State m_state);

    State getState();

    bool initialize(double dt);

    void desired_state_cb(const geometry_msgs::PointConstPtr &pt);

    void iteration(const ros::TimerEvent &e);

    void run();

private:

    State m_state;
    double sim_time;
    int robot_id;
    std::string worldframe;
    string localframe;
    gains_t gains;
    double frequency;
    double dt;

    Vector3d u;
    double tau;

    ControllerImpl *controller;
    state_space_t state_space;
    desired_state_t desired_state;
    ros::NodeHandle nh;
    string robot_link_name;
    DynamicsProvider *dynamics;
    params_t params;
    init_vals_t init_vals;

    ros::Publisher marker_pub;
    visualization_msgs::Marker m;
    ros::Subscriber desired_state_sub;
    ros::Publisher state_pub;

    void initPaths();
    void set_state_space();
    bool load_params();
    bool load_init_vals();
    void send_transform();
    void publish_path();
    void publish_state();

};

#endif
