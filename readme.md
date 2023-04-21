# Mavswarm: A Lightweight Multi-Aerial Vehicle Simulator 

Mavswarm is a lightweight and fast Multi-Aerial Vehicle simulator based on ROS (Robot Operating System). It supports simulating heterogenous quadrotor swarms of upto 10 robots on a single desktop with physics. Mavswarm also supports quadrotor control, trajectory optimization and receding horizon planning (RHP). Currently, the internal controller uses the Lee's geometric tracking controller [3] and it is tuned for two different quadrotor models out of the box. 
In addition you can extend the swarm simulation with more quadrotor models by simply adding the new quadrotor model parameters through a yaml configuration file. 

Consider citing our work [1][2] if you find this code helpful for your publications. 

| ![Cover Image](https://github.com/malintha/multi_uav_simulator/blob/master/cover.gif?raw=true) |
|:--:| 
| *A hetergenous swarm of 5 quadrotors stabilizing from an upside-down initialization* |

## Mostly Automated Installation

A Makefile has been compiled to install ros, install dependancies and/or create and build a catkin workspace, and build the entire uav simulator package. 

Note: This package currently runs with ROS1 which is only compatible on older linux distros - for example, it is not supported on anything later than Ubuntu 20.04

**To get the makefile, run:**

```bash
wget https://gist.githubusercontent.com/matteovidali/caab443e66425e260b6a1c1bd842d28c/raw/188effd1f7e5c4a49cf636abd96bc325a9ac9bd9/Makefile
```

Then, if you wish to do a full desktop installation of ROS1, this can be done with:
```bash
make ros_desktop_install
```

To get and install the required dependancies, run:
```bash
make deps
```

To create and initialize a NEW catkin_ws with all required packages
First in the makefile, change the value of 'CATKIN_WS_NAME' from '../catkin_ws' to the path where your workspace should exist.
Then, run:
```bash
make wstool-new
```

If you have a PRE-EXISTING catkin workspace you would like to build this module into,
First change the 'CATKIN_WS_NAME' variable to the path of your catkin workspace, then run:

```bash
make wstool-exists
```

Finally, if you would like to run a catkin build, you may run:
```bash
make build
```
This will build the entire catkin space, so be sure to customize this if necessary

After building, you can launch the simulator with 
```bash
make launch
```

## Manual Installation
**Install dependencies**

Install [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page), [`Armadillo`](https://www.uio.no/studier/emner/matnat/fys/FYS4411/v13/guides/installing-armadillo/) and [`GNU Science Library (GSL)`](https://www.gnu.org/software/gsl/) before you continue.

Install eigen-coversions (make sure to change the ROS version name i.e.; melodic, kinetic).

    sudo apt install ros-noetic-eigen-conversions

**Building the simulator**

Create a new catkin workspace and use `wstool` to download the `multi_uav_simulator` package.

    mkdir catkin_ws
    wstool init src
    cd src/
    wstool set --git multi_uav_simulator git@github.com:malintha/multi_uav_simulator.git -y
    wstool update
    
Mavswarm uses `ethz-asl/mav_trajectory_generation` package to generate trajectories for the robots. Use the`rosinstall/dependencies.rosinstall` file to download the required dependencies.
    
    wstool merge multi_uav_simulator/rosinstall/dependencies.rosinstall
    wstool update
    
This will clone the following packages to the workspace automatically : `catkin_simple`,  `eigen_catkin`,  `eigen_checks`,  `glog_catkin`,  `mav_comm`,  `mav_trajectory_generation`,  `nlopt`.

Now build the simulator.

    cd ../
    catkin build multi_uav_simulator
 
 Run the simulator.

    source devel/setup.bash
    roslaunch multi_uav_simulator simu.launch


This will open an Rviz window and show 5 drones flip from an upside-down initialization.

**Publishing Goals**

Mavswarm supports Receding Horizon Planning (RHP) and trajectory optimization out of the box. You can publish a new desired goal position using rostopics for any drone to see it navigating. To do so, open a new command window and publish the following message. 

    rostopic pub /robot_1/desired_state geometry_msgs/Point '{x: 4, y: -1, z: 3}'
    
Each drone listnes to its `desired_state` topic which uses the ROS standard `geometry_msgs/Point` message type to recieve new goal locations. The command above simply adds the goal `{x: 4, y: -1, z: 3}` into the first drone's goal list. To See RHP in working, publish a different goal before the drone reaches the previous goal. This will recalculate a new trajectory to match the robot's current speed and acceleration to ensure smooth transitions between rapidly changing goal locations.

**Adding a new drone to the environment**

Mavswarm uses xacro to spawn new models into the simulation environment. Out of the box, it has 5 drones in the environment and two separate quadrotor models to choose from; namely, Bigquad (the quadrotor model from [2]) and Crazyflie. If you want to add a sixth drone to the environment, simply follow the steps below.

1) In the `launch/simu.launch` file, add a new line with the corresponding drone id.  Make sure to change `<param name>` and the robot_id accordingly.  For example,

        <param name="robot_6" command="$(find xacro)/xacro $(find multi_uav_simulator)/cf_description/crazyflie.urdf.xacro robot_id:=6" />

    This adds a new Crazyflie model to the environment. If you want a larger quadrotor model, you can use `bigquad_description/bigquad.urdf.xacro` instead. 

2) Now add the following block to bind a quadrotor instance to the URDF model. Here we pass the `robot_id` and the internal controller's frequency as arguments to the quadrotor instance. In addition, we use `rosparam` to specify the quadrotor's model parameters and the controller gains. The parameters for the Bigquad and the Crazyflie can be found in `config/bigquad_params.yaml` and `config/crazyflie_params.yaml` files respectively. 

        <group ns="robot_6">
            <node name="drone" pkg="multi_uav_simulator" type="multi_uav_simulator" output="screen" args="6 $(arg controller_frequency)">
              <rosparam file="$(find multi_uav_simulator)/config/crazyflie_params.yaml"/>
            </node>
          </group>

3) In order to specify the initial conditions for the new drone add a new block to the `config/initial_conditions.yaml` file as below.

        robot_6:
            position: [2, 4, 0]
            velocity: [0, 0, 0]
            rotation: [1, 0, 0, 0, -0.9995, -0.0314, 0, 0.0314, -0.9995]
            omega: [0, 0, 0]

    The rotation and omega stands for the initial rotation matrix (3x3) and the angular velocity (3x1) of the rigid body. Here, we initialize the drone up-side down. To make it right way up, change the rotation matrix to identity: `[1, 0, 0, 0, 1, 0, 0, 0, 1]`.


4) Finally, to visualize the newly added drone, add a robot_model visualization type to RViz. Change its description to corresponding param name of the newly added element in the launch file. i.e.: "`robot_6`".

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
