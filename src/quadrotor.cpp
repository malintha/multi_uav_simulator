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

#include "quadrotor.hpp"
#include "rclcpp/rclcpp.hpp"

/*
ros2 run mavswarm2 mavswarm2 0 50 --ros-args --log-level DEBUG
ros2 topic pub --rate 1 /desired_state simulator_interfaces/msg/GeometricCtrl "{position:{x: 2.0, y: 2.4, z: -3.0},heading:{x: 1, y: 0, z: 0}}"

*/ 
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    int robot_id = std::atoi(argv[1]);
    double frequency = (double)std::atof(argv[2]);
    auto logger = rclcpp::get_logger("logger");


    rclcpp::spin(std::make_shared<Quadrotor>(robot_id, frequency));
    rclcpp::shutdown();

    return 0;
}
