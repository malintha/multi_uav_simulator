# Mavswarm2 (under development)

Mavswarm is a lightweight and fast Multi-Aerial Vehicle simulator built on Robot Operating System (ROS), entirely written in C++. Mavswarm2 is the ROS2-compatible version of Mavswarm. It supports simulating heterogenous quadrotor swarms of upto 10 robots on a single desktop with physics. 

It has 
- quadrotor control (ROS1/ROS2) 
- trajectory optimization and receding horizon planning (only working in ROS1. I welcome any PRs/Contributions). 

The internal controller uses the Lee's geometric tracking controller [3] and it is tuned for two different quadrotor models out of the box. 

**Compared to Mavswarm1, Mavswarm2 has a simpler installation, and the code is more self-contained. Everything is in the header files, so easier to manage and export as a library**.

Consider citing our work [1][2] if you find this code helpful for your publications. 

| ![Cover Image](https://github.com/malintha/multi_uav_simulator/blob/master/cover.gif?raw=true) |
|:--:| 
| *A hetergenous swarm of 5 quadrotors stabilizing from an upside-down initialization* |

## Installation

**Install the dependencies**

Install Eigen, Armadillo and GNU Science Library (GSL) before you continue.

**Clone the repositories**
    
    mkdir -p mavswarm2/src
    cd mavswarm2/src
    git clone -b ros2 git@github.com:malintha/simulator_utils.git
    git clone -b ros2 git@github.com:malintha/multi_uav_simulator.git

**Building the simulator**

    cd ../
    colcon build

**Running the simulator**

    source install/setup.bash
    ros2 launch mavswarm2 quadrotor.launch


**Publishing Goals**

- Under development, not working in right now

**Adding a new drone to the environment**

- under development, not working in right now



[1] Our work based on this controller:

    @inproceedings{fernando2019formation,
    title={Formation control and navigation of a quadrotor swarm},
    author={Fernando, Malintha and Liu, Lantao},
    booktitle={2019 International Conference on Unmanned Aircraft Systems (ICUAS)},
    pages={284--291},
    year={2019},
    organization={IEEE}
    }

[2] Mean-Field flocking control of UAVs (Find the complete code at: https://github.com/malintha/mean_field_flocking):

    @INPROCEEDINGS{9560899,
    author={Fernando, Malintha},
    booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)}, 
    title={Online Flocking Control of UAVs with Mean-Field Approximation}, 
    year={2021},
    volume={},
    number={},
    pages={8977-8983},
    doi={10.1109/ICRA48506.2021.9560899}
    }

[3] Geometric tracking controller:

    @inproceedings{lee2010geometric,
    title={Geometric tracking control of a quadrotor UAV on SE (3)},
    author={Lee, Taeyoung and Leok, Melvin and McClamroch, N Harris},
    booktitle={49th IEEE conference on decision and control (CDC)},
    pages={5420--5425},
    year={2010},
    organization={IEEE}
    }
