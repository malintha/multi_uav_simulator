//
// Created by malintha on 10/14/20.
//
#ifndef SIMULATOR_UTILS_DRONE_H
#define SIMULATOR_UTILS_DRONE_H

//#include <iostream>
#include "ros/ros.h"
#include "simulator_utils/Waypoint.h"
#include "ros/console.h"

//using namespace std;

class Drone {
    ros::Subscriber state_sub;
    ros::NodeHandle nh;
    void state_cb(const simulator_utils::WaypointConstPtr& wp);

public:
    int id;
    simulator_utils::Waypoint state;
    Drone(int id, const ros::NodeHandle &n);
    simulator_utils::Waypoint get_state() const;


};

#endif