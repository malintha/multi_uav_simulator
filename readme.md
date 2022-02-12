# Multi-UAV-Simulator

An efficient and robust multi drone simulator based on ROS (Robot Operating System). The dynamics of the drones are modeled using a set of ODEs and solved via GSL. The trajectory tracking is performed using the geometric tracking controller proposed in [2], and a standalone implementation of the same can be found at https://github.com/malintha/geo_controller/.

![Cover Image](https://raw.githubusercontent.com/Malintha/multi_uav_simulator/master/cover.gif)

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

Please consider citiing our work [1] if you find this code helpful for your publications.

[1] M. Fernando and L. Liu, "Formation Control and Navigation of a Quadrotor Swarm," 2019 International Conference on Unmanned Aircraft Systems (ICUAS), Atlanta, GA, USA, 2019, pp. 284-291, doi: 10.1109/ICUAS.2019.8798352.

[2] T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)," 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, 2010, pp. 5420-5425, doi: 10.1109/CDC.2010.5717652.
