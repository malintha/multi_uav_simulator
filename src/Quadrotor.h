#ifndef PROJECT_QUADROTOR_H
#define PROJECT_QUADROTOR_H

#include <geo_controller/controllerImpl.h>
#include <tf/transform_listener.h>
#include <string>
#include <visualization_msgs/Marker.h>

// #include "DesiredStateProvider.h"
#include "DynamicsProvider.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

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

    void move(double dt, const desired_state_t& d_state);

    void setState(State m_state);

    State getState();

    state_space_t get_state_space();

    bool initialize(double dt);

    void set_desired_state(const desired_state_t &desired_ss_);

    init_vals_t get_init_vals();

    DynamicsProvider *get_dynamics();

    void iteration(const ros::TimerEvent &e);

    void run();

private:

    State m_state;
    double sim_time;
    int robot_id;
    std::string worldframe;
    string prefix;
    gains_t gains;
    double frequency;

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

    void initPaths();

    void set_state_space();

    bool load_params();

    bool load_init_vals();

    void send_transform();

    void publish_path();
};

#endif
