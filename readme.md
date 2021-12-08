# Multi-UAV-Simulator For Crazyflie 2.0 Nano-drone

An efficient and robust multi drone simulator based on ROS (Robot Operating System). The dynamics of the drones are modeled using a set of ODEs and solved via GSL. The trajectory tracking is performed using the geometric tracking controller proposed in [2], and a standalone implementation of the same can be found at https://github.com/malintha/geo_controller/. Consider citing our work [1] if you find this code helpful for your publications.

![Cover Image](https://raw.githubusercontent.com/Malintha/multi_uav_simulator/crazyflie/crazyflie_cover.png)

**Install dependencies**
Please install [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), [Armadillo](https://www.uio.no/studier/emner/matnat/fys/FYS4411/v13/guides/installing-armadillo/) and [GNU Science Library (GSL)](https://www.gnu.org/software/gsl/) before you continue.

**Building the simulator**

Clone the following packages to your ROS workspace. i.e: (~/catkin_ws/src/). 

    git clone  https://github.com/malintha/multi_uav_simulator
    git clone  https://github.com/malintha/geo_controller/
    git clone  https://github.com/malintha/simulator_utils
    
Use catkin build to build the packages as below.
    
    cd catkin_ws/ && source devel/setup.bash
    catkin build multi_uav_simulator

Run the simulator

    source devel/setup.bash
    roslaunch multi_uav_simulator simu.launch

[1] Our work based on this controller:

    @inproceedings{fernando2019formation,
    title={Formation control and navigation of a quadrotor swarm},
    author={Fernando, Malintha and Liu, Lantao},
    booktitle={2019 International Conference on Unmanned Aircraft Systems (ICUAS)},
    pages={284--291},
    year={2019},
    organization={IEEE}
    }

[2] Original paper on the geometric tracking controller:

    @inproceedings{lee2010geometric,
    title={Geometric tracking control of a quadrotor UAV on SE (3)},
    author={Lee, Taeyoung and Leok, Melvin and McClamroch, N Harris},
    booktitle={49th IEEE conference on decision and control (CDC)},
    pages={5420--5425},
    year={2010},
    organization={IEEE}
    }

