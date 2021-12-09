# SwarmSim2: A Lightweight Multi-UAV Simulator 

An efficient and robust multi drone simulator based on ROS (Robot Operating System). The dynamics of the drones are modeled using a set of ODEs and solved via GSL. The trajectory tracking is performed using the geometric tracking controller proposed in [2], and a standalone implementation of the same can be found at https://github.com/malintha/geo_controller/. Consider citing our work [1] if you find this code helpful for your publications.

Checkout the ``crazyflie`` branch for the tuned controller for the Crazyflie nano-drone.

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

**Adding a new drone to the environment**

SwarmSim2 uses xacro to spawn new models into the simulation environment. Out of the box, it has 5 drones in the environment. If you need to add a sixth drone, simply follow the steps below.

1) In the `launch/simu.launch` file, add a new line with the corresponding drone id.  Make sure to change `<param name>` and the robot_id accordingly.  For example,

		<param  name="cf6"  command="$(find xacro)/xacro --inorder $(find 		multi_uav_simulator)/cf_description/crazyflie.urdf.xacro robot_id:=6" />

   
2) In order to specify the initial conditions for the robot add a new block in the `config.yaml` file as below.

		robot_6:
			position: [2, 4, 0]
			velocity: [0, 0, 0]
			rotation: [1, 0, 0, 0, -0.9995, -0.0314, 0, 0.0314, -0.9995]
			omega: [0, 0, 0]

	The rotation and omega stands for the initial rotation matrix (3x3) and the angular velocity (3x1) of the rigid body. Here, we initialize the drone up-side down. To make it right way up, change the rotation matrix to `[1, 0, 0, 0, 1, 0, 0, 0, 1]`.


3) Finally, to visualize the newly added drone, add a robot_model visualization type to RViz. Change its description to corresponding param name of the newly added element in the launch file. i.e.: "`cf6`".

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
