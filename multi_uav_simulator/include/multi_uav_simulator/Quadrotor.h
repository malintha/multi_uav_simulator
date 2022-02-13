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
#include "Trajectory_t.h"

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

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
#include "std_msgs/Float32MultiArray.h"


class Quadrotor {
public:
    Quadrotor(int robot_id, double frequency, ros::NodeHandle &n);
    void move(const desired_state_t &d_state);
    void setState(State m_state);
    State getState();

    bool initialize(double dt);

    void desired_pos_cb(const geometry_msgs::Point &pt);

    void iteration(const ros::TimerEvent &e);
    void run();

private:

    State m_state;
    double sim_time, tau, dt, frequency;
    int robot_id;
    std::string worldframe, localframe, robot_link_name;
    gains_t gains;
    std::vector<Eigen::Vector3d> traj_piece;
    int xd_it;
    Vector3d u, target_pos;
    state_space_t x0, xd0;

    bool set_init_target;
    geometry_msgs::Point target_next;
    bool set_next_target = false;
    // Vector3d ;
    mav_trajectory_generation::Trajectory traj;
    // mav_trajectory_generation::Trajectory traj_temp;

    ControllerImpl *controller;
    state_space_t state_space;
    ros::NodeHandle nh;

    DynamicsProvider *dynamics;
    params_t params;
    init_vals_t init_vals;

    ros::Publisher marker_pub, goal_pub, state_pub;
    visualization_msgs::Marker m, g;
    std::vector<geometry_msgs::Point> points;
    ros::Subscriber desired_state_sub;
    vector<Vector3d> positions;

    bool quad_initialized = false;
    // Vector3d xd;

    void do_rhp();
    void initPaths();
    void set_state_space();
    bool load_params();
    bool load_init_vals();
    void send_transform();
    void publish_path();
    void publish_state();
    static mav_trajectory_generation::Trajectory get_opt_traj(const opt_t &ps, const Vector3d& pe);
};

#endif
