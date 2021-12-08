//
// Created by malintha on 2/14/18.
//
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <string>
#include "Quadrotor.h"
#include "cfSimUtils.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <ros/console.h>

using namespace Eigen;

const double FREQUENCY = 100;

class Simulator {
public:
    Simulator(const ros::NodeHandle &n) {
        ROS_DEBUG_STREAM("Starting simulator");
        ros::NodeHandle nh;
        dt = 1 / FREQUENCY;

        if (!loadConstants(n)) {
            ROS_ERROR_STREAM("Could not load the simulator parameters");
        }
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        for (int i = 0; i < n_drones; i++) {
            int robot_id = i + 1;
            Quadrotor *quad = new Quadrotor(robot_id, worldframe, prefix);
            if (!quad->initialize(dt, gains)) {
                ROS_ERROR_STREAM("Drone " << robot_id << " initializing failed.");
            }
            quad->setState(State::Autonomous);
            quadList.push_back(quad);
            frame = 0;
        }
        initPaths();
        ROS_DEBUG_STREAM("Simulator initialized");

        while (marker_pub.getNumSubscribers() < 1) {
            ROS_INFO("Waiting for subscriber");
            ros::Duration(1).sleep();
        }
    }

    void iteration(const ros::TimerEvent &e) {
        t += dt;
        frame += 1;
        for (int i = 0; i < n_drones; i++) {
            Quadrotor *quad = quadList[i];
            init_vals_t init_vals = quad->get_init_vals();
            Vector3d xd = simulator_utils::ned_nwu_rotation(init_vals.position);
            Vector3d b1d(1, 0, 0);
            
            Vector3d x1(0.1*t, 0.1*sin(M_PI*t), 0.1*cos(M_PI*t));
            xd = xd + x1;
            
            desired_state_t dss = {(xd), b1d};
            quad->move(dt, dss);

            // std::cout << t <<std::endl;

            Vector3d x = (quad->get_dynamics()->get_state().position);
            x[1] = -x[1];
            int s = quad->getState();
            if (s == State::Autonomous && frame % 10 == 0) {
                geometry_msgs::Point p;
                p.x = x[0];
                p.y = x[1];
                p.z = x[2];
                if (i == 1) {
                    m1.points.push_back(p);
                } else if (i == 2) {
                    m2.points.push_back(p);
                } else if (i == 3) {
                    m3.points.push_back(p);
                } else if (i == 4) {
                    m4.points.push_back(p);
                } else {
                    m5.points.push_back(p);
                }
                marker_pub.publish(m1);
                marker_pub.publish(m2);
                marker_pub.publish(m3);
                marker_pub.publish(m4);
                marker_pub.publish(m5);
            }
        }
    }

    void run(float frequency) {
        ros::Timer timer = node.createTimer(ros::Duration(1 / frequency), &Simulator::iteration, this);
        ros::spin();
    }

private:
    int frame;
    int n_drones;
    gains_t gains;
    std::string worldframe, prefix;
    std::vector<Quadrotor *> quadList;
    ros::NodeHandle node;
    double t;
    double dt;
    ros::Publisher marker_pub;
    visualization_msgs::Marker m1, m2, m3, m4, m5;

    bool loadConstants(const ros::NodeHandle &n) {
        vector<double> gains_;
        n.getParam("/controller/gains", gains_);
        n.getParam("/count", n_drones);
        n.getParam("/frame/fixed", worldframe);
        n.getParam("/frame/prefix", prefix);

        this->gains = {gains_[0], gains_[1], gains_[2], gains_[3]};
        ROS_DEBUG_STREAM("count: " << n_drones << " fixed_frame: " << worldframe << " prefix: " << prefix);

        return true;
    }

    void initPaths() {
        m1.header.stamp = m2.header.stamp = m3.header.stamp = m4.header.stamp = m5.header.stamp = ros::Time::now();
        m1.type = m1.type = m2.type = m3.type = m4.type = m5.type = visualization_msgs::Marker::SPHERE_LIST;
        m1.header.frame_id = m2.header.frame_id = m3.header.frame_id = m4.header.frame_id = m5.header.frame_id = worldframe;
        m1.action = m2.action = m3.action = m4.action = m5.action = visualization_msgs::Marker::ADD;
        m1.id = 1;
        m2.id = 2;
        m3.id = 3;
        m4.id = 4;
        m5.id = 5;

        m1.color.r = 0;
        m2.color.r = 1;
        m3.color.r = 1;
        m4.color.r = 0.5;
        m5.color.r = 0;
        m1.color.g = 0;
        m2.color.g = 0;
        m3.color.g = 0.5;
        m4.color.g = 0;
        m5.color.g = 1;
        m1.color.b = 1;
        m2.color.b = 0;
        m3.color.b = 0;
        m4.color.b = 0.5;
        m5.color.b = 0;

        m1.color.a = m2.color.a = m3.color.a = m4.color.a = m5.color.a = 1;
        m1.scale.x = m2.scale.x = m3.scale.x = m4.scale.x = m5.scale.x = 0.05;
        m1.scale.z = m2.scale.z = m3.scale.z = m4.scale.z = m5.scale.z = 0.05;
        m1.scale.y = m2.scale.y = m3.scale.y = m4.scale.y = m5.scale.y = 0.05;

        m1.pose.orientation.w = m2.pose.orientation.w = m3.pose.orientation.w = m4.pose.orientation.w = m5.pose.orientation.w = 1.0;
    }
};

int main(int argc, char **argv) {
    float frequency = FREQUENCY;
    ros::init(argc, argv, "drones_simulator");
    ros::NodeHandle n;
    Simulator simulator(n);
    simulator.run(frequency);
    return 0;
}