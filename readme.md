# Mavswarm: A Lightweight Multi-aerial vehicle Simulator 

Mavswarm is a lightweight and fast Multi-aerial vehicle simulator based on ROS (Robot Operating System). It can handle upto 10 quadrotors on a single desktop with real-time physics simulation. Mavswarm also supports quadrotor control, trajectory optimization and receding horizon planning (RHP). Currently, the internal controller uses the geometric tracking controller (Lee's controller) [2] tuned for two different quadrotor models (a large quadrotor and a Crazyflie).
Consider citing our work [1] if you find this code helpful for your publications. 

Checkout the ``crazyflie`` branch for the tuned controller for the Crazyflie nano-drone.

![Cover Image](https://raw.githubusercontent.com/Malintha/multi_uav_simulator/crazyflie/crazyflie_cover.png)

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

    catkin build multi_uav_simulator
 
 Run the simulator.

    source devel/setup.bash
    roslaunch multi_uav_simulator simu.launch


This will open an Rviz window and show 5 drones flip from an upside-down initialization.

**Publishing the Goal for a given drone**

Mavswarm supports Receding Horizon Planning (RHP) and trajectory optimization out of the box. You can publish a new desired goal position using rostopics for any drone to see it navigating. To do so, open a new command window and publish the following message. 

    rostopic pub /robot_1/desired_state geometry_msgs/Point '{x: 4, y: -1, z: 3}'
    
Each drone listnes to its `desired_state` topic which uses the ROS standard `geometry_msgs/Point` message type to recieve new goal locations. The command above simply adds the goal `{x: 4, y: -1, z: 3}` into the first drone's goal list. To See RHP in working, publish a different goal before the drone reaches the previous goal. This will recalculate a new trajectory to match the robot's current speed and acceleration to ensure smooth transitions between rapidly changing goal locations.

**Adding a new drone to the environment**

Mavswarm uses xacro to spawn new models into the simulation environment. Out of the box, it has 5 drones in the environment. If you need to add a sixth drone, simply follow the steps below.

1) In the `launch/simu.launch` file, add a new line with the corresponding drone id.  Make sure to change `<param name>` and the robot_id accordingly.  For example,

        <param  name="cf6"  command="$(find xacro)/xacro --inorder $(find multi_uav_simulator)/cf_description/crazyflie.urdf.xacro robot_id:=6" />

2) Now add the following block to bind a quadrotor instance to the drone model. 

        <group ns="robot_6">
            <node name="drone" pkg="multi_uav_simulator" type="multi_uav_simulator" output="screen">
              <param name="robot_id" value="1"/>
              <param name="frame/prefix" value="/base_link"/>
              <remap from="robot_6/drone" to="drone"/>
            </node>
          </group>

3) In order to specify the initial conditions for the new drone add a new block in the `config.yaml` file as below.

        robot_6:
            position: [2, 4, 0]
            velocity: [0, 0, 0]
            rotation: [1, 0, 0, 0, -0.9995, -0.0314, 0, 0.0314, -0.9995]
            omega: [0, 0, 0]

    The rotation and omega stands for the initial rotation matrix (3x3) and the angular velocity (3x1) of the rigid body. Here, we initialize the drone up-side down. To make it right way up, change the rotation matrix to identity: `[1, 0, 0, 0, 1, 0, 0, 0, 1]`.


4) Finally, to visualize the newly added drone, add a robot_model visualization type to RViz. Change its description to corresponding param name of the newly added element in the launch file. i.e.: "`cf6`".

        <param  name="cf6"  command="$(find xacro)/xacro --inorder $(find multi_uav_simulator)/cf_description/crazyflie.urdf.xacro robot_id:=6" />

**Mavswarm can support upto 8-10 drones on a single desktop computer with real-time physics simulations.**

[1] Our work based on this controller:

    @inproceedings{fernando2019formation,
    title={Formation control and navigation of a quadrotor swarm},
    author={Fernando, Malintha and Liu, Lantao},
    booktitle={2019 International Conference on Unmanned Aircraft Systems (ICUAS)},
    pages={284--291},
    year={2019},
    organization={IEEE}
    }

[2] Geometric tracking controller:

    @inproceedings{lee2010geometric,
    title={Geometric tracking control of a quadrotor UAV on SE (3)},
    author={Lee, Taeyoung and Leok, Melvin and McClamroch, N Harris},
    booktitle={49th IEEE conference on decision and control (CDC)},
    pages={5420--5425},
    year={2010},
    organization={IEEE}
    }